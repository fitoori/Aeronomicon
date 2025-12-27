#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ONICS-T Remote Console (Flask + SSH over Tailscale)

Purpose
-------
A single-purpose, square-screen-friendly web UI that:
- Connects to WATNE over a Tailscale tailnet hostname.
- Runs ~/Aeronomicon/onics-t.py on WATNE when "ENGAGE" is pressed.
- Stops that process when "DISENGAGE" is pressed.
- Streams stdout/stderr live to the browser (Server-Sent Events).
- Shows extensive health/staleness/LOS indicators and falls back to an LOS screen
  on unceremonious termination.

Security Model
--------------
- The UI does not accept arbitrary commands. It only triggers a fixed remote command.
- SSH authentication is delegated to your existing ssh-agent and/or ~/.ssh keys.
- You should ensure the remote SSH account is appropriately constrained.

Operational Model
-----------------
- This app is designed to run on a separate device with a local browser.
- It assumes the device is joined to your tailnet and can resolve:
    watne.tail8d99b6.ts.net
"""

from __future__ import annotations

import dataclasses
import getpass
import json
import os
import queue
import re
import shutil
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional, Tuple

# ---- Dependency preflight (fail fast, with actionable errors) -----------------

_missing: List[str] = []

try:
    from flask import Flask, Response, jsonify, render_template
    from werkzeug.serving import WSGIRequestHandler, make_server
except Exception:  # pragma: no cover
    _missing.append("flask")

try:
    import paramiko
except Exception:  # pragma: no cover
    _missing.append("paramiko")

if _missing:  # pragma: no cover
    sys.stderr.write(
        "FATAL: Missing required Python dependencies: {deps}\n"
        "Install with:\n"
        "  python3 -m pip install --upgrade pip\n"
        "  python3 -m pip install -r requirements.txt\n".format(deps=", ".join(_missing))
    )
    sys.exit(2)

# ---- Configuration ------------------------------------------------------------

APP_TITLE = "ONICS-T"
TAILNET_HOSTNAME_DEFAULT = "watne.tail8d99b6.ts.net"

TAILNET_HOSTNAME_ENV = os.environ.get("WATNE_TAILNET_HOST")
TAILNET_HOSTNAME = (TAILNET_HOSTNAME_ENV or TAILNET_HOSTNAME_DEFAULT).strip()

SSH_PORT_ENV = os.environ.get("WATNE_SSH_PORT")
SSH_PORT = int(SSH_PORT_ENV or "22")
SSH_USER_ENV = os.environ.get("WATNE_SSH_USER")
SSH_USER = (SSH_USER_ENV or "pi").strip()
SSH_CONFIG_PATH = os.path.expanduser(os.environ.get("WATNE_SSH_CONFIG", "~/.ssh/config"))

# Remote script path (as provided)
REMOTE_ONICS_T_PATH = "~/Aeronomicon/onics-t.py"
REMOTE_PIDFILE = "/tmp/onics-t.pid"

# UI / streaming settings
MAX_LOG_LINES = int(os.environ.get("MAX_LOG_LINES", "600"))
HEALTH_TICK_HZ = float(os.environ.get("HEALTH_TICK_HZ", "1.0"))     # status push rate to UI
TAILSCALE_CHECK_S = float(os.environ.get("TAILSCALE_CHECK_S", "4.0"))  # local tailscale status check period
DNS_CHECK_S = float(os.environ.get("DNS_CHECK_S", "2.0"))
TCP_CHECK_S = float(os.environ.get("TCP_CHECK_S", "2.0"))
SERVICE_CHECK_S = float(os.environ.get("SERVICE_CHECK_S", "3.0"))
LTE_SIGNAL_CHECK_S = float(os.environ.get("LTE_SIGNAL_CHECK_S", "15.0"))
LTE_SIGNAL_CHECK_HIGH_LOAD_S = float(
    os.environ.get("LTE_SIGNAL_CHECK_HIGH_LOAD_S", "45.0")
)
LTE_SIGNAL_LOAD_THRESHOLD = float(os.environ.get("LTE_SIGNAL_LOAD_THRESHOLD", "4.0"))
SYSTEM_STATS_CHECK_S = float(os.environ.get("SYSTEM_STATS_CHECK_S", "5.0"))
REMOTE_LTE_SIGNAL_SCRIPT = os.environ.get(
    "WATNE_LTE_SIGNAL_SCRIPT",
    "$HOME/Aeronomicon/util/lte-signal-strength.sh",
).strip()
LTE_SIGNAL_TIMEOUT_S = float(os.environ.get("LTE_SIGNAL_TIMEOUT_S", "5.0"))

# "Staleness" thresholds (seconds)
REMOTE_HEALTH_STALE_S = float(os.environ.get("REMOTE_HEALTH_STALE_S", "6.0"))
REMOTE_HEALTH_LOS_S = float(os.environ.get("REMOTE_HEALTH_LOS_S", "15.0"))

# SSH timeouts
SSH_CONNECT_TIMEOUT_S = float(os.environ.get("SSH_CONNECT_TIMEOUT_S", "5.0"))
SSH_BANNER_TIMEOUT_S = float(os.environ.get("SSH_BANNER_TIMEOUT_S", "5.0"))
SSH_BANNER_TIMEOUT_RETRY_S = float(os.environ.get("SSH_BANNER_TIMEOUT_RETRY_S", "20.0"))
SSH_AUTH_TIMEOUT_S = float(os.environ.get("SSH_AUTH_TIMEOUT_S", "12.0"))

# Process stop behavior
STOP_GRACE_S = float(os.environ.get("STOP_GRACE_S", "3.0"))
STOP_POLL_S = float(os.environ.get("STOP_POLL_S", "0.25"))

# Auto-restart behavior (when ENGAGED)
AUTO_RESTART_DELAY_S = float(os.environ.get("AUTO_RESTART_DELAY_S", "3.0"))
AUTO_RESTART_WINDOW_S = float(os.environ.get("AUTO_RESTART_WINDOW_S", "120.0"))
AUTO_RESTART_MAX = int(os.environ.get("AUTO_RESTART_MAX", "3"))

# Flask server
FLASK_HOST = os.environ.get("FLASK_HOST", "0.0.0.0").strip()
FLASK_PORT = int(os.environ.get("FLASK_PORT", "8080"))

# ---- Helpers ----------------------------------------------------------------


def monotonic_s() -> float:
    return time.monotonic()


def wall_time_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime())


def safe_json_dumps(obj: Any) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False)


def which_or_none(exe: str) -> Optional[str]:
    p = shutil.which(exe)
    if p:
        return p
    for extra in ("/usr/bin", "/bin", "/usr/sbin", "/sbin", "/usr/local/sbin", "/usr/local/bin"):
        candidate = os.path.join(extra, exe)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def load_ssh_config() -> Optional[paramiko.SSHConfig]:
    if not SSH_CONFIG_PATH:
        return None
    if not os.path.exists(SSH_CONFIG_PATH):
        return None
    cfg = paramiko.SSHConfig()
    try:
        with open(SSH_CONFIG_PATH, "r", encoding="utf-8") as handle:
            cfg.parse(handle)
    except Exception:
        return None
    return cfg


def resolve_host(hostname: str) -> Tuple[bool, List[str], str]:
    try:
        infos = socket.getaddrinfo(hostname, None, proto=socket.IPPROTO_TCP)
        ips = sorted({i[4][0] for i in infos})
        return True, ips, ""
    except Exception as e:
        return False, [], f"{type(e).__name__}: {e}"


def tcp_probe(ip: str, port: int, timeout_s: float = 0.8) -> Tuple[bool, str]:
    s = socket.socket(socket.AF_INET6 if ":" in ip else socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout_s)
    try:
        s.connect((ip, port))
        return True, ""
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"
    finally:
        try:
            s.close()
        except Exception:
            pass


def now_ms() -> int:
    return int(time.time() * 1000)


# ---- SSE broker (fanout) -----------------------------------------------------


class SSEBroker:
    """A small fanout broker for Server-Sent Events subscribers."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subs: List[queue.Queue[str]] = []

    def subscribe(self) -> queue.Queue[str]:
        q: queue.Queue[str] = queue.Queue(maxsize=1000)
        with self._lock:
            self._subs.append(q)
        return q

    def unsubscribe(self, q: queue.Queue[str]) -> None:
        with self._lock:
            try:
                self._subs.remove(q)
            except ValueError:
                return

    def publish(self, event: str, data: Dict[str, Any]) -> None:
        payload = f"event: {event}\n" + f"data: {safe_json_dumps(data)}\n\n"
        with self._lock:
            subs = list(self._subs)
        # Best-effort fanout: drop if a client is too slow.
        for q in subs:
            try:
                q.put_nowait(payload)
            except queue.Full:
                # Unstable/slow client; drop newest update.
                pass


class QuietWSGIRequestHandler(WSGIRequestHandler):
    _SUPPRESSED_ERRORS = (
        "Bad request version",
        "Bad request syntax",
        "Bad HTTP/0.9 request type",
    )

    def log_error(self, format: str, *args: Any) -> None:
        message = format % args
        if any(token in message for token in self._SUPPRESSED_ERRORS):
            return
        super().log_error(format, *args)


# ---- State models ------------------------------------------------------------


@dataclasses.dataclass
class TailnetHealth:
    tailscale_present: bool = False
    tailscale_ok: bool = False
    tailscale_backend_state: str = "UNKNOWN"
    tailscale_error: str = ""
    magicdns_host: str = ""
    magicdns_ips: List[str] = dataclasses.field(default_factory=list)

    dns_ok: bool = False
    dns_ips: List[str] = dataclasses.field(default_factory=list)
    dns_error: str = ""

    tcp_ok: bool = False
    tcp_ip: str = ""
    tcp_error: str = ""

    last_ok_mono: float = 0.0
    last_check_mono: float = 0.0

    unstable_score_60s: int = 0
    stale: bool = False
    los: bool = False


@dataclasses.dataclass
class OnicsRuntime:
    state: str = "IDLE"  # IDLE|STARTING|RUNNING|STOPPING|ERROR|LOS
    running_since_mono: float = 0.0
    last_output_mono: float = 0.0
    last_ssh_ok_mono: float = 0.0
    last_state_change_mono: float = 0.0

    ssh_connected: bool = False
    ssh_connect_ms: int = 0
    remote_pid: Optional[int] = None
    remote_pidfile: str = REMOTE_PIDFILE
    last_error: str = ""
    login_required: bool = False
    login_message: str = ""
    engaged: bool = False
    restart_failures: int = 0


@dataclasses.dataclass
class AutopilotHealth:
    services: Dict[str, Dict[str, Any]] = dataclasses.field(default_factory=dict)
    system: Dict[str, Any] = dataclasses.field(default_factory=dict)


# ---- ONICS-T controller ------------------------------------------------------


class OnicsController:
    def __init__(self, broker: SSEBroker) -> None:
        self._broker = broker
        self._lock = threading.Lock()
        self._logs: Deque[str] = deque(maxlen=MAX_LOG_LINES)
        self._runtime = OnicsRuntime()
        self._health = TailnetHealth()
        self._autopilot = AutopilotHealth(
            services={
                "arducopter": {
                    "unit": "arducopter.service",
                    "status": "UNKNOWN",
                    "detail": "Awaiting status.",
                    "active": False,
                },
                "mavproxy": {
                    "unit": "mavproxy.service",
                    "status": "UNKNOWN",
                    "detail": "Awaiting status.",
                    "active": False,
                },
                "uplink": {
                    "unit": "uplink.service",
                    "status": "UNKNOWN",
                    "detail": "Awaiting status.",
                    "active": False,
                },
            },
            system={},
        )
        self._stop_requested = False
        self._ssh: Optional[paramiko.SSHClient] = None
        self._ssh_transport: Optional[paramiko.Transport] = None
        self._chan: Optional[paramiko.Channel] = None
        self._reader_thread: Optional[threading.Thread] = None

        self._fail_events_60s: Deque[float] = deque()
        self._restart_events: Deque[float] = deque()
        self._last_tailscale_check = 0.0
        self._last_dns_check = 0.0
        self._last_tcp_check = 0.0
        self._last_service_check = 0.0
        self._last_lte_check = 0.0
        self._last_system_check = 0.0
        self._lte_signal_cache: Dict[str, Any] = {}
        self._service_backoff_until = 0.0
        self._service_backoff_s = 0.0
        self._system_backoff_until = 0.0
        self._system_backoff_s = 0.0
        with self._lock:
            self._autopilot.system = self._blank_system_stats()

    # --- Logging & publishing

    def _append_log(self, line: str) -> None:
        line = line.rstrip("\n")
        if not line:
            return
        ts = wall_time_iso()
        full = f"[{ts}] {line}"
        with self._lock:
            self._logs.append(full)
            self._runtime.last_output_mono = monotonic_s()
        self._broker.publish("log", {"line": full, "ts": now_ms()})

    def clear_logs(self) -> None:
        with self._lock:
            self._logs.clear()
        self._broker.publish("log", {"line": "[CLEARED] Log buffer cleared", "ts": now_ms()})

    def _set_state(self, new_state: str, error: str = "") -> None:
        with self._lock:
            self._runtime.state = new_state
            self._runtime.last_state_change_mono = monotonic_s()
            if error:
                self._runtime.last_error = error
        self._broker.publish("state", self.snapshot())

    @staticmethod
    def _needs_login_prompt(err: str) -> bool:
        lowered = err.lower()
        return "no authentication methods available" in lowered or "authentication failed" in lowered

    @staticmethod
    def _needs_keygen_prompt(err: str) -> bool:
        lowered = err.lower()
        return "no identities found" in lowered

    def _set_login_prompt_state(self, required: bool, message: str = "") -> None:
        with self._lock:
            self._runtime.login_required = required
            self._runtime.login_message = message if required else ""

    def _set_login_prompt(self, required: bool, message: str = "") -> None:
        self._set_login_prompt_state(required, message)
        self._broker.publish("state", self.snapshot())

    def _resolve_ssh_settings(
        self,
    ) -> Tuple[str, str, int, List[str], Optional[str]]:
        cfg = load_ssh_config()
        cfg_entry: Dict[str, Any] = cfg.lookup(TAILNET_HOSTNAME) if cfg else {}

        hostname = cfg_entry.get("hostname", TAILNET_HOSTNAME)

        username = SSH_USER
        if SSH_USER_ENV is None and cfg_entry.get("user"):
            username = str(cfg_entry["user"])

        port = SSH_PORT
        if SSH_PORT_ENV is None and cfg_entry.get("port"):
            try:
                port = int(cfg_entry["port"])
            except (TypeError, ValueError):
                port = SSH_PORT

        identity_files = cfg_entry.get("identityfile", [])
        if isinstance(identity_files, str):
            identity_files = [identity_files]
        identity_files = [os.path.expanduser(path) for path in identity_files if path]

        proxycommand = cfg_entry.get("proxycommand")

        return hostname, username, port, identity_files, proxycommand

    def _handle_ssh_failure(self, err: str, *, publish_state: bool = True) -> None:
        if self._needs_login_prompt(err):
            hostname, username, port, _identity_files, _proxycommand = self._resolve_ssh_settings()
            msg = (
                "SSH authentication not found. Run "
                f"`ssh-copy-id -p {port} {username}@{hostname}` to set up your key, "
                f"then open `ssh -p {port} {username}@{hostname}` to continue."
            )
            if self._needs_keygen_prompt(err):
                msg = (
                    "No SSH identities found. Generate one with "
                    "`ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N \"\"`, "
                    f"then run `ssh-copy-id -p {port} {username}@{hostname}` and "
                    f"`ssh -p {port} {username}@{hostname}` to continue."
                )
            if publish_state:
                self._set_login_prompt(True, msg)
            else:
                self._set_login_prompt_state(True, msg)
            self._append_log("SSH AUTH REQUIRED: interactive login needed to continue.")
        else:
            if publish_state:
                self._set_login_prompt(False, "")
            else:
                self._set_login_prompt_state(False, "")

    def snapshot(self, *, include_logs: bool = True) -> Dict[str, Any]:
        hostname, username, port, _identity_files, _proxycommand = self._resolve_ssh_settings()
        with self._lock:
            rt = dataclasses.asdict(self._runtime)
            hl = dataclasses.asdict(self._health)
            ap = dataclasses.asdict(self._autopilot)
            logs = list(self._logs)[-MAX_LOG_LINES:] if include_logs else []
        ap["system"].update(self._lte_signal_stats())
        # Derived ages (computed server-side)
        now_m = monotonic_s()
        rt["state_age_s"] = round(max(0.0, now_m - rt.get("last_state_change_mono", 0.0)), 3)
        rt["last_output_age_s"] = (
            round(max(0.0, now_m - rt.get("last_output_mono", 0.0)), 3)
            if rt.get("last_output_mono", 0.0)
            else None
        )
        rt["last_ssh_ok_age_s"] = (
            round(max(0.0, now_m - rt.get("last_ssh_ok_mono", 0.0)), 3)
            if rt.get("last_ssh_ok_mono", 0.0)
            else None
        )
        hl["last_ok_age_s"] = (
            round(max(0.0, now_m - hl.get("last_ok_mono", 0.0)), 3)
            if hl.get("last_ok_mono", 0.0)
            else None
        )
        return {
            "meta": {
                "title": APP_TITLE,
                "hostname": hostname,
                "ssh_user": username,
                "ssh_port": port,
                "server_time_iso": wall_time_iso(),
            },
            "health": hl,
            "autopilot": ap,
            "onics": rt,
            "logs": logs,
        }

    def publish_status(self) -> None:
        self._broker.publish("status", self.snapshot(include_logs=False))

    # --- Health checks

    def _mark_failure(self) -> None:
        now_m = monotonic_s()
        self._fail_events_60s.append(now_m)
        while self._fail_events_60s and (now_m - self._fail_events_60s[0]) > 60.0:
            self._fail_events_60s.popleft()

    @staticmethod
    def _blank_system_stats() -> Dict[str, Any]:
        return {
            "load_1": None,
            "load_5": None,
            "load_15": None,
            "cpu_count": None,
            "mem_total_bytes": None,
            "mem_available_bytes": None,
            "mem_used_bytes": None,
            "disk_total_bytes": None,
            "disk_used_bytes": None,
            "disk_free_bytes": None,
        }

    @classmethod
    def _parse_remote_system_output(cls, output: str) -> Dict[str, Any]:
        stats = cls._blank_system_stats()
        parts = output.split("\n---\n")
        if len(parts) < 4:
            return stats

        load_line = parts[0].strip().splitlines()
        if load_line:
            load_parts = load_line[0].split()
            if len(load_parts) >= 3:
                try:
                    stats["load_1"] = float(load_parts[0])
                    stats["load_5"] = float(load_parts[1])
                    stats["load_15"] = float(load_parts[2])
                except ValueError:
                    pass

        mem_total = None
        mem_available = None
        meminfo_values: Dict[str, int] = {}
        for line in parts[1].splitlines():
            items = line.split()
            if len(items) < 2:
                continue
            key = items[0].rstrip(":")
            try:
                value = int(items[1])
            except ValueError:
                continue
            meminfo_values[key] = value
            if key == "MemTotal":
                mem_total = value * 1024
            elif key == "MemAvailable":
                mem_available = value * 1024

        if mem_available is None:
            mem_free = meminfo_values.get("MemFree")
            buffers = meminfo_values.get("Buffers")
            cached = meminfo_values.get("Cached")
            shmem = meminfo_values.get("Shmem", 0)
            reclaimable = meminfo_values.get("SReclaimable", 0)
            if mem_free is not None and buffers is not None and cached is not None:
                mem_available = max((mem_free + buffers + cached + reclaimable - shmem) * 1024, 0)

        if mem_total is not None:
            stats["mem_total_bytes"] = mem_total
        if mem_total is not None and mem_available is not None and mem_available > mem_total:
            mem_available = None
        if mem_available is not None:
            stats["mem_available_bytes"] = mem_available
        if mem_total is not None and mem_available is not None:
            stats["mem_used_bytes"] = max(mem_total - mem_available, 0)

        df_lines = [line for line in parts[2].splitlines() if line.strip()]
        if df_lines:
            columns = df_lines[-1].split()
            try:
                if len(columns) == 3:
                    stats["disk_total_bytes"] = int(columns[0])
                    stats["disk_used_bytes"] = int(columns[1])
                    stats["disk_free_bytes"] = int(columns[2])
                elif len(columns) >= 4:
                    stats["disk_total_bytes"] = int(columns[1])
                    stats["disk_used_bytes"] = int(columns[2])
                    stats["disk_free_bytes"] = int(columns[3])
            except ValueError:
                pass

        cpu_line = parts[3].strip().splitlines()
        if cpu_line:
            try:
                stats["cpu_count"] = int(cpu_line[0].strip())
            except ValueError:
                pass

        return stats

    def _fetch_remote_system_stats(self, client: paramiko.SSHClient) -> Dict[str, Any]:
        cmd = (
            "sh -c '"
            "cat /proc/loadavg 2>/dev/null; "
            "echo \"---\"; "
            "cat /proc/meminfo 2>/dev/null; "
            "echo \"---\"; "
            "df_line=$(df -B1 / 2>/dev/null | tail -n 1); "
            "if [ -n \"$df_line\" ]; then "
            "echo $df_line | awk \"{print \\$2, \\$3, \\$4}\"; "
            "else df -k / 2>/dev/null | tail -n 1 | awk \"{print \\$2 * 1024, \\$3 * 1024, \\$4 * 1024}\"; fi; "
            "echo \"---\"; "
            "getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || echo 0'"
        )
        _stdin, stdout, stderr = client.exec_command(cmd, get_pty=False)
        output = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        _ = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
        if err:
            output = output + ("\n" if output else "") + err
        return self._parse_remote_system_output(output)

    def _system_stats_check(self) -> None:
        now_m = monotonic_s()
        if now_m < self._system_backoff_until:
            return

        client: Optional[paramiko.SSHClient]
        created = False
        with self._lock:
            client = self._ssh if self._ssh_transport and self._ssh_transport.is_active() else None

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err)
                if self._system_backoff_s <= 0.0:
                    self._system_backoff_s = max(3.0, SYSTEM_STATS_CHECK_S)
                else:
                    self._system_backoff_s = min(self._system_backoff_s * 2.0, 60.0)
                self._system_backoff_until = monotonic_s() + self._system_backoff_s
                with self._lock:
                    self._autopilot.system = self._blank_system_stats()
                return
            created = True

        stats = self._blank_system_stats()
        try:
            stats = self._fetch_remote_system_stats(client)
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        with self._lock:
            self._autopilot.system = stats
        self._system_backoff_s = 0.0
        self._system_backoff_until = 0.0

    @staticmethod
    def _parse_lte_signal_output(output: str) -> Tuple[Optional[float], Optional[float]]:
        match = re.search(
            r"RSSI:\s*(?P<rssi>-?\d+(?:\.\d+)?)\s*dBm,\s*Signal Strength:\s*(?P<percent>\d+(?:\.\d+)?)%",
            output,
        )
        if not match:
            return None, None
        return float(match.group("rssi")), float(match.group("percent"))

    def _lte_signal_stats(self) -> Dict[str, Any]:
        now_m = monotonic_s()
        interval_s = LTE_SIGNAL_CHECK_S
        try:
            load_avg = os.getloadavg()[0]
        except OSError:
            load_avg = None
        if load_avg is not None and load_avg > LTE_SIGNAL_LOAD_THRESHOLD:
            interval_s = max(LTE_SIGNAL_CHECK_HIGH_LOAD_S, LTE_SIGNAL_CHECK_S)
        if (now_m - self._last_lte_check) < interval_s and self._lte_signal_cache:
            return dict(self._lte_signal_cache)
        self._last_lte_check = now_m

        stats: Dict[str, Any] = {
            "lte_signal_percent": None,
            "lte_rssi_dbm": None,
            "lte_signal_error": "LTE signal unavailable.",
        }

        client: Optional[paramiko.SSHClient]
        created = False
        with self._lock:
            client = self._ssh if self._ssh_transport and self._ssh_transport.is_active() else None

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err, publish_state=False)
                stats["lte_signal_error"] = f"SSH error: {err}"
                self._lte_signal_cache = stats
                return dict(stats)
            created = True

        remote_script = REMOTE_LTE_SIGNAL_SCRIPT.replace("'", "'\"'\"'")
        cmd = f"bash -lc 'exec \"{remote_script}\"'"
        try:
            _stdin, stdout, stderr = client.exec_command(
                cmd,
                get_pty=False,
                timeout=LTE_SIGNAL_TIMEOUT_S,
            )
            output = stdout.read().decode("utf-8", errors="replace")
            error_output = stderr.read().decode("utf-8", errors="replace").strip()
            status = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
        except Exception as exc:
            stats["lte_signal_error"] = f"Remote LTE signal error: {exc}"
            self._lte_signal_cache = stats
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass
            return dict(stats)
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        if status != 0:
            suffix = f" ({error_output})" if error_output else ""
            stats["lte_signal_error"] = (
                f"Remote LTE signal script failed with exit code {status}{suffix}."
            )
            self._lte_signal_cache = stats
            return dict(stats)

        rssi, percent = self._parse_lte_signal_output(output)
        if rssi is None or percent is None:
            stats["lte_signal_error"] = "Could not parse LTE signal output."
            self._lte_signal_cache = stats
            return dict(stats)

        stats["lte_signal_percent"] = round(percent, 2)
        stats["lte_rssi_dbm"] = round(rssi, 2)
        stats["lte_signal_error"] = ""
        self._lte_signal_cache = stats
        return dict(stats)

    def _tailscale_check(self) -> None:
        exe = which_or_none("tailscale")
        hl_update: Dict[str, Any]

        if not exe:
            hl_update = {
                "tailscale_present": False,
                "tailscale_ok": False,
                "tailscale_backend_state": "MISSING",
                "tailscale_error": "tailscale binary not found in PATH",
            }
        else:
            # Robust tailscale health probing:
            # 1) Prefer `tailscale status --json` when supported.
            # 2) Fall back to `tailscale ip -4` and/or plain `tailscale status`.
            backend = "UNKNOWN"
            ok = False
            errtxt = ""

            # Attempt JSON status.
            try:
                p = subprocess.run(
                    [exe, "status", "--json"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=1.2,
                    check=False,
                )
                if p.returncode == 0:
                    j = json.loads(p.stdout or "{}")
                    backend = str(j.get("BackendState", "UNKNOWN"))
                    ok = backend.lower() == "running"
                    errtxt = (p.stderr or "").strip()
                else:
                    # If --json is unsupported, fall back.
                    errtxt = (
                        p.stderr or p.stdout or f"tailscale status returned {p.returncode}"
                    ).strip()
                    backend = "UNKNOWN"
            except Exception as e:
                errtxt = f"{type(e).__name__}: {e}"
                backend = "UNKNOWN"

            if not ok:
                # Fall back: `tailscale ip -4` returns an address when connected.
                try:
                    p2 = subprocess.run(
                        [exe, "ip", "-4"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=1.0,
                        check=False,
                    )
                    ip4 = (p2.stdout or "").strip()
                    if p2.returncode == 0 and ip4:
                        ok = True
                        backend = backend if backend != "UNKNOWN" else "RUNNING?"
                        if not errtxt:
                            errtxt = (p2.stderr or "").strip()
                except Exception as e:
                    if not errtxt:
                        errtxt = f"{type(e).__name__}: {e}"

            if not ok:
                # Final fall back: plain `tailscale status` return code.
                try:
                    p3 = subprocess.run(
                        [exe, "status"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                        timeout=1.2,
                        check=False,
                    )
                    # Heuristic: non-empty output and returncode 0 generally means tailscale is up.
                    out3 = (p3.stdout or "").strip()
                    err3 = (p3.stderr or "").strip()
                    if p3.returncode == 0 and out3:
                        ok = True
                        backend = backend if backend != "UNKNOWN" else "RUNNING?"
                    if not errtxt:
                        errtxt = err3
                except Exception as e:
                    if not errtxt:
                        errtxt = f"{type(e).__name__}: {e}"

            hl_update = {
                "tailscale_present": True,
                "tailscale_ok": ok,
                "tailscale_backend_state": backend,
                "tailscale_error": errtxt,
            }

        with self._lock:
            for k, v in hl_update.items():
                setattr(self._health, k, v)

    def _dns_check(self) -> None:
        ok, ips, err = resolve_host(TAILNET_HOSTNAME)
        with self._lock:
            self._health.dns_ok = ok
            self._health.dns_ips = ips
            self._health.dns_error = err

    def _tcp_check(self) -> None:
        with self._lock:
            ips = list(self._health.dns_ips)
        if not ips:
            with self._lock:
                self._health.tcp_ok = False
                self._health.tcp_ip = ""
                self._health.tcp_error = "No resolved IPs"
            return

        # Probe first IP; prefer IPv4 for simplicity if present.
        ip = next((x for x in ips if ":" not in x), ips[0])
        ok, err = tcp_probe(ip, SSH_PORT, timeout_s=0.8)
        with self._lock:
            self._health.tcp_ok = ok
            self._health.tcp_ip = ip
            self._health.tcp_error = err

    def _service_status_remote(self, client: paramiko.SSHClient, unit: str) -> Dict[str, Any]:
        active_state = "UNKNOWN"
        sub_state = ""
        description = ""
        detail = ""

        try:
            cmd = (
                "systemctl show "
                f"{shlex.quote(unit)} "
                "--property=ActiveState,SubState,Description --no-pager"
            )
            _stdin, stdout, stderr = client.exec_command(cmd, get_pty=False)
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            exit_status = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
            if exit_status == 0:
                for line in (out or "").splitlines():
                    key, _, value = line.partition("=")
                    if key == "ActiveState":
                        active_state = value.strip() or active_state
                    elif key == "SubState":
                        sub_state = value.strip()
                    elif key == "Description":
                        description = value.strip()
            else:
                detail = (err or out or "").strip()
        except Exception as e:
            detail = f"{type(e).__name__}: {e}"

        if active_state == "UNKNOWN":
            try:
                cmd = f"systemctl is-active {shlex.quote(unit)}"
                _stdin, stdout, stderr = client.exec_command(cmd, get_pty=False)
                out = stdout.read().decode("utf-8", errors="replace")
                err = stderr.read().decode("utf-8", errors="replace")
                _ = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
                active_state = (out or err or "").strip() or active_state
                if not detail:
                    detail = f"systemctl is-active: {active_state}"
            except Exception as e:
                if not detail:
                    detail = f"{type(e).__name__}: {e}"

        detail_parts = [part for part in (description, sub_state) if part]
        detail_line = " · ".join(detail_parts) if detail_parts else detail

        return {
            "unit": unit,
            "status": active_state or "UNKNOWN",
            "detail": detail_line or detail,
            "active": active_state == "active",
        }

    def _services_check(self) -> None:
        now_m = monotonic_s()
        if now_m < self._service_backoff_until:
            return
        updates: Dict[str, Dict[str, Any]] = {}
        with self._lock:
            service_units = {
                key: entry.get("unit", "")
                for key, entry in self._autopilot.services.items()
                if entry.get("unit")
            }

        client: Optional[paramiko.SSHClient]
        created = False
        with self._lock:
            client = self._ssh if self._ssh_transport and self._ssh_transport.is_active() else None

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err)
                if self._service_backoff_s <= 0.0:
                    self._service_backoff_s = max(3.0, SERVICE_CHECK_S)
                else:
                    self._service_backoff_s = min(self._service_backoff_s * 2.0, 60.0)
                self._service_backoff_until = monotonic_s() + self._service_backoff_s
                for key, unit in service_units.items():
                    updates[key] = {
                        "unit": unit,
                        "status": "UNKNOWN",
                        "detail": f"SSH error: {err}",
                        "active": False,
                    }
                with self._lock:
                    for key, value in updates.items():
                        self._autopilot.services[key] = value
                return
            created = True

        try:
            for key, unit in service_units.items():
                updates[key] = self._service_status_remote(client, unit)
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        with self._lock:
            for key, value in updates.items():
                self._autopilot.services[key] = value
        self._service_backoff_s = 0.0
        self._service_backoff_until = 0.0

    def health_tick(self) -> None:
        now_m = monotonic_s()
        if (now_m - self._last_tailscale_check) >= TAILSCALE_CHECK_S:
            self._last_tailscale_check = now_m
            self._tailscale_check()

        if (now_m - self._last_dns_check) >= DNS_CHECK_S:
            self._last_dns_check = now_m
            self._dns_check()

        if (now_m - self._last_tcp_check) >= TCP_CHECK_S:
            self._last_tcp_check = now_m
            self._tcp_check()

        if (now_m - self._last_service_check) >= SERVICE_CHECK_S:
            self._last_service_check = now_m
            self._services_check()

        if (now_m - self._last_system_check) >= SYSTEM_STATS_CHECK_S:
            self._last_system_check = now_m
            self._system_stats_check()

        # Compute stale/LOS and stability score.
        with self._lock:
            hl = self._health
            rt = self._runtime

            # Consider "OK" when tailscale and tcp are OK.
            ok = bool(hl.tailscale_ok and hl.dns_ok and hl.tcp_ok)
            now_m2 = monotonic_s()
            hl.last_check_mono = now_m2
            if ok:
                hl.last_ok_mono = now_m2
            else:
                # Record path failures for instability scoring.
                # Rate-limit to ~1Hz to avoid unbounded growth.
                if not self._fail_events_60s or (now_m2 - self._fail_events_60s[-1]) >= 1.0:
                    self._mark_failure()

            # Staleness is driven by time since last OK check.
            if hl.last_ok_mono > 0.0:
                ok_age = now_m2 - hl.last_ok_mono
            else:
                ok_age = float("inf")

            hl.stale = ok_age > REMOTE_HEALTH_STALE_S
            hl.los = ok_age > REMOTE_HEALTH_LOS_S

            # Instability: count failures in last 60s.
            hl.unstable_score_60s = len(self._fail_events_60s)

            # If ONICS is running and SSH transport drops, force LOS.
            if rt.state in ("RUNNING", "STARTING", "STOPPING"):
                if self._ssh_transport is not None and (not self._ssh_transport.is_active()):
                    rt.state = "LOS"
                    rt.ssh_connected = False
                    rt.last_error = "SSH transport became inactive"
                    rt.last_state_change_mono = now_m2

        # Publish status at a fixed cadence.
        self.publish_status()

    # --- SSH operations

    def _ssh_connect(self) -> Tuple[Optional[paramiko.SSHClient], str, int]:
        hostname, username, port, identity_files, proxycommand = self._resolve_ssh_settings()
        start = time.time()

        def _connect(banner_timeout_s: float) -> Tuple[Optional[paramiko.SSHClient], str, Optional[Exception]]:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            sock = None
            if proxycommand:
                try:
                    sock = paramiko.ProxyCommand(proxycommand)
                except Exception:
                    sock = None

            try:
                client.connect(
                    hostname=hostname,
                    port=port,
                    username=username,
                    timeout=SSH_CONNECT_TIMEOUT_S,
                    banner_timeout=banner_timeout_s,
                    auth_timeout=SSH_AUTH_TIMEOUT_S,
                    allow_agent=True,
                    look_for_keys=True,
                    key_filename=identity_files or None,
                    sock=sock,
                )
                return client, "", None
            except Exception as e:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
                try:
                    client.close()
                except Exception:
                    pass
                return None, f"{type(e).__name__}: {e}", e

        def _is_banner_timeout(err: Exception) -> bool:
            msg = str(err).lower()
            return "error reading ssh protocol banner" in msg or "banner" in msg

        client, err, exc = _connect(SSH_BANNER_TIMEOUT_S)
        if client is None and exc is not None:
            retry_timeout = max(SSH_BANNER_TIMEOUT_RETRY_S, SSH_BANNER_TIMEOUT_S)
            if _is_banner_timeout(exc) and retry_timeout > SSH_BANNER_TIMEOUT_S:
                client, err, exc = _connect(retry_timeout)
                if client is None:
                    err = f"{err} (after banner-timeout retry)"

        ms = int((time.time() - start) * 1000)
        return client, err, ms

    def _ssh_close(self) -> None:
        try:
            if self._chan is not None:
                try:
                    self._chan.close()
                except Exception:
                    pass
        finally:
            self._chan = None

        try:
            if self._ssh is not None:
                try:
                    self._ssh.close()
                except Exception:
                    pass
        finally:
            self._ssh = None
            self._ssh_transport = None

        with self._lock:
            self._runtime.ssh_connected = False

    # --- Remote ONICS lifecycle

    def start_onics(self, *, auto_restart: bool = False) -> Tuple[bool, str]:
        with self._lock:
            if self._runtime.state in ("STARTING", "RUNNING", "STOPPING"):
                return False, f"ONICS-T is already {self._runtime.state}"
            # Gate on tailnet health
            if not (self._health.tailscale_ok and self._health.dns_ok and self._health.tcp_ok):
                return False, "Tailnet/SSH health is not OK; refusing to start"
            self._stop_requested = False
            self._runtime.engaged = True
            self._runtime.remote_pid = None
            self._runtime.last_error = ""
            self._runtime.ssh_connected = False
            self._runtime.ssh_connect_ms = 0
            self._runtime.running_since_mono = 0.0
            self._runtime.last_output_mono = 0.0
            self._runtime.last_ssh_ok_mono = 0.0
            self._runtime.login_required = False
            self._runtime.login_message = ""
            if not auto_restart:
                self._restart_events.clear()
                self._runtime.restart_failures = 0

        self._set_state("STARTING")
        t = threading.Thread(target=self._remote_run_thread, name="onics-ssh-reader", daemon=True)
        self._reader_thread = t
        t.start()
        return True, "STARTING"

    def stop_onics(self) -> Tuple[bool, str]:
        with self._lock:
            st = self._runtime.state
            pid = self._runtime.remote_pid
            self._stop_requested = True
            self._runtime.engaged = False
            if st not in ("STARTING", "RUNNING"):
                return False, f"ONICS-T is not running (state={st})"
            self._runtime.state = "STOPPING"
            self._runtime.last_state_change_mono = monotonic_s()
        self._broker.publish("state", self.snapshot())

        # Prefer killing by PID (recorded or via pidfile).
        ok, msg = self._remote_stop(pid)
        return ok, msg

    def restart_service(self, service_key: str) -> Tuple[bool, str]:
        with self._lock:
            service = self._autopilot.services.get(service_key)
            unit = service.get("unit") if service else ""

        if not unit:
            return False, f"Unknown service '{service_key}'"

        client: Optional[paramiko.SSHClient]
        created = False
        with self._lock:
            client = self._ssh if self._ssh_transport and self._ssh_transport.is_active() else None

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err)
                return False, f"SSH error: {err}"
            created = True

        ok = False
        detail = ""
        try:
            commands = [
                f"sudo -n systemctl restart {shlex.quote(unit)}",
                f"systemctl restart {shlex.quote(unit)}",
            ]
            for cmd in commands:
                _stdin, stdout, stderr = client.exec_command(cmd, get_pty=False)
                out = stdout.read().decode("utf-8", errors="replace")
                err = stderr.read().decode("utf-8", errors="replace")
                exit_status = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
                detail = (err or out or "").strip()
                if exit_status == 0:
                    ok = True
                    break
        except Exception as e:
            detail = f"{type(e).__name__}: {e}"
            ok = False
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        outcome = "OK" if ok else "FAILED"
        log_detail = f" ({detail})" if detail else ""
        self._append_log(f"SERVICE RESTART {outcome}: {unit}{log_detail}")
        self._services_check()
        return ok, detail or ("Restarted" if ok else "Restart failed")

    def reboot_vehicle(self) -> Tuple[bool, str]:
        with self._lock:
            if not (self._health.tailscale_ok and self._health.dns_ok and self._health.tcp_ok):
                return False, "Tailnet/SSH health is not OK; refusing to reboot"

        client: Optional[paramiko.SSHClient]
        created = False
        with self._lock:
            client = self._ssh if self._ssh_transport and self._ssh_transport.is_active() else None

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err)
                return False, f"SSH error: {err}"
            created = True

        ok = False
        detail = ""
        try:
            commands = [
                "sudo -n reboot now",
                "reboot now",
            ]
            for cmd in commands:
                _stdin, stdout, stderr = client.exec_command(cmd, get_pty=False)
                exit_status = stdout.channel.recv_exit_status()  # type: ignore[union-attr]
                out = stdout.read().decode("utf-8", errors="replace")
                err = stderr.read().decode("utf-8", errors="replace")
                detail = (err or out or "").strip()
                if exit_status == 0:
                    ok = True
                    break
        except Exception as e:
            detail = f"{type(e).__name__}: {e}"
            ok = False
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

        outcome = "OK" if ok else "FAILED"
        log_detail = f" ({detail})" if detail else ""
        self._append_log(f"VEHICLE RESTART {outcome}: reboot now{log_detail}")
        return ok, detail or ("Restarting" if ok else "Restart failed")

    def _remote_stop(self, pid: Optional[int]) -> Tuple[bool, str]:
        # Attempt to use existing SSH client if present; otherwise reconnect.
        client = None
        created = False
        with self._lock:
            client = self._ssh

        if client is None:
            client, err, _ms = self._ssh_connect()
            if client is None:
                self._mark_failure()
                self._handle_ssh_failure(err)
                self._append_log(f"STOP FAILED: SSH connect error: {err}")
                self._set_state("LOS", f"STOP SSH connect failed: {err}")
                return False, err
            created = True

        try:
            # Determine PID if missing.
            if pid is None:
                pid = self._remote_read_pid(client)
                if pid is not None:
                    with self._lock:
                        self._runtime.remote_pid = pid

            if pid is None:
                self._append_log("STOP FAILED: No PID available (pidfile missing/unreadable)")
                self._set_state("ERROR", "No PID available to stop ONICS-T")
                return False, "No PID available"

            expected_script = os.path.basename(REMOTE_ONICS_T_PATH)
            try:
                check_cmd = f"bash -lc 'ps -p {pid} -o args= 2>/dev/null || true'"
                _stdin, stdout, _stderr = client.exec_command(check_cmd, get_pty=False)
                args = stdout.read().decode("utf-8", errors="replace").strip()
            except Exception as e:
                args = ""
                self._append_log(f"STOP WARNING: could not verify PID {pid}: {e}")

            if not args or expected_script not in args:
                self._append_log(
                    f"STOP FAILED: PID {pid} does not look like ONICS-T "
                    f"(expected '{expected_script}', got '{args or 'no process'}')"
                )
                self._set_state("ERROR", "PID mismatch; refusing to stop")
                return False, "PID mismatch"

            # Execute an escalated stop sequence, bounded in time.
            stop_cmd = (
                "bash -lc 'set -euo pipefail; "
                f"PID={pid}; "
                "if kill -0 \"$PID\" 2>/dev/null; then kill -INT \"$PID\" 2>/dev/null || true; fi; "
                f"end=$((SECONDS+{int(max(1.0, STOP_GRACE_S))})); "
                "while [ $SECONDS -lt $end ]; do "
                "  if ! kill -0 \"$PID\" 2>/dev/null; then exit 0; fi; "
                f"  sleep {STOP_POLL_S}; "
                "done; "
                "kill -TERM \"$PID\" 2>/dev/null || true; "
                "sleep 0.5; "
                "kill -KILL \"$PID\" 2>/dev/null || true; "
                "exit 0'"
            )
            stdin, stdout, stderr = client.exec_command(stop_cmd, get_pty=False)
            _ = stdout.read().decode("utf-8", errors="replace") if hasattr(stdout, "read") else ""
            err = stderr.read().decode("utf-8", errors="replace") if hasattr(stderr, "read") else ""
            if err.strip():
                self._append_log(f"STOP NOTE: {err.strip()}")

            # Let reader thread observe process exit. Force-close channel if still open.
            self._append_log("STOP SIGNAL SENT: requested ONICS-T stop")
            return True, "STOPPING"
        except Exception as e:
            self._mark_failure()
            msg = f"{type(e).__name__}: {e}"
            self._append_log(f"STOP FAILED: {msg}")
            self._set_state("LOS", f"Stop failed: {msg}")
            return False, msg
        finally:
            if created and client is not None:
                try:
                    client.close()
                except Exception:
                    pass

    def _remote_read_pid(self, client: paramiko.SSHClient) -> Optional[int]:
        try:
            cmd = f"bash -lc 'cat {REMOTE_PIDFILE} 2>/dev/null || true'"
            _stdin, stdout, _stderr = client.exec_command(cmd, get_pty=False)
            s = stdout.read().decode("utf-8", errors="replace").strip()
            if not s:
                return None
            m = "".join(ch for ch in s if ch.isdigit())
            if not m:
                return None
            return int(m)
        except Exception:
            return None

    def _schedule_auto_restart(self, reason: str) -> None:
        with self._lock:
            if not self._runtime.engaged:
                return
            if self._stop_requested:
                return

            now_m = monotonic_s()
            while self._restart_events and (now_m - self._restart_events[0]) > AUTO_RESTART_WINDOW_S:
                self._restart_events.popleft()

            if len(self._restart_events) >= AUTO_RESTART_MAX:
                self._append_log(
                    f"AUTO-RESTART ABORTED: exceeded {AUTO_RESTART_MAX} attempts in "
                    f"{int(AUTO_RESTART_WINDOW_S)}s"
                )
                self._set_state("ERROR", "Auto-restart limit reached")
                return

            self._restart_events.append(now_m)
            self._runtime.restart_failures += 1

        self._append_log(
            f"AUTO-RESTART: scheduling restart in {AUTO_RESTART_DELAY_S:.1f}s ({reason})"
        )

        def _delayed_restart() -> None:
            time.sleep(max(0.0, AUTO_RESTART_DELAY_S))
            ok, msg = self.start_onics(auto_restart=True)
            if ok:
                self._append_log("AUTO-RESTART: starting ONICS-T")
            else:
                self._append_log(f"AUTO-RESTART FAILED: {msg}")

        threading.Thread(
            target=_delayed_restart,
            name="onics-auto-restart",
            daemon=True,
        ).start()

    def _remote_run_thread(self) -> None:
        # Connect SSH
        client, err, ms = self._ssh_connect()
        if client is None:
            self._mark_failure()
            self._handle_ssh_failure(err)
            self._append_log(f"SSH CONNECT FAILED ({ms} ms): {err}")
            self._set_state("LOS", f"SSH connect failed: {err}")
            return

        transport = client.get_transport()
        if transport is not None:
            # Keepalive helps detect link loss (best-effort).
            try:
                transport.set_keepalive(5)
            except Exception:
                pass

        with self._lock:
            self._ssh = client
            self._ssh_transport = transport
            self._runtime.ssh_connected = True
            self._runtime.ssh_connect_ms = ms
            self._runtime.last_ssh_ok_mono = monotonic_s()
            self._runtime.running_since_mono = monotonic_s()

        self._append_log(f"SSH CONNECTED: {SSH_USER}@{TAILNET_HOSTNAME}:{SSH_PORT} ({ms} ms)")
        self._set_login_prompt(False, "")

        # Remote preflight: ensure python3 and script exist.
        preflight_cmd = (
            "bash -lc 'set -euo pipefail; "
            "command -v python3 >/dev/null 2>&1; "
            f"test -f {REMOTE_ONICS_T_PATH}; "
            "echo OK'"
        )
        try:
            _stdin, stdout, stderr = client.exec_command(preflight_cmd, get_pty=False)
            out = stdout.read().decode("utf-8", errors="replace").strip()
            err2 = stderr.read().decode("utf-8", errors="replace").strip()
            if out.strip() != "OK":
                self._append_log(f"REMOTE PREFLIGHT FAILED: {out or err2 or 'unknown'}")
                self._set_state("ERROR", f"Remote preflight failed: {out or err2 or 'unknown'}")
                self._ssh_close()
                return
        except Exception as e:
            self._mark_failure()
            msg = f"{type(e).__name__}: {e}"
            self._append_log(f"REMOTE PREFLIGHT ERROR: {msg}")
            self._set_state("LOS", f"Remote preflight error: {msg}")
            self._ssh_close()
            return

        # Start ONICS-T in the foreground but capture PID early.
        # - PYTHONUNBUFFERED forces line-buffering.
        # - Writes PID into /tmp/onics-t.pid for deterministic stop.
        # - Uses 2>&1 to merge stderr into stdout for unified log stream.
        remote_cmd = (
            "bash -lc 'set -euo pipefail; "
            "export PYTHONUNBUFFERED=1; "
            f"python3 -u {REMOTE_ONICS_T_PATH} 2>&1 & "
            "PID=$!; "
            f"echo $PID > {REMOTE_PIDFILE}; "
            "echo __ONICS_PID__:${PID}; "
            "wait ${PID}'"
        )

        try:
            stdin, stdout, stderr = client.exec_command(remote_cmd, get_pty=False)
            # We only read stdout because stderr is merged (2>&1). Still drain stderr safely.
            self._chan = stdout.channel  # type: ignore[attr-defined]

            # Drain stderr in a background thread (defensive)
            def _drain_stderr() -> None:
                try:
                    _ = stderr.read()
                except Exception:
                    pass

            threading.Thread(target=_drain_stderr, name="onics-stderr-drain", daemon=True).start()

            self._set_state("RUNNING")

            for raw in iter(stdout.readline, ""):
                if raw == "":
                    break
                line = raw.rstrip("\n")

                # Parse PID marker
                if line.startswith("__ONICS_PID__:"):
                    try:
                        pid_str = line.split(":", 1)[1].strip()
                        pid = int(pid_str)
                        with self._lock:
                            self._runtime.remote_pid = pid
                            self._runtime.last_ssh_ok_mono = monotonic_s()
                        self._append_log(f"ONICS-T PID: {pid}")
                        continue
                    except Exception:
                        self._append_log(f"WARNING: could not parse PID line: {line}")
                        continue

                # Normal log line
                with self._lock:
                    self._runtime.last_ssh_ok_mono = monotonic_s()
                self._append_log(line)

                # Stop requested: keep reading until remote exits, but we can accelerate by closing channel.
                with self._lock:
                    if self._stop_requested:
                        # Do not close immediately; allow graceful stop logs to appear.
                        pass

            # Process ended
            exit_status = None
            try:
                if self._chan is not None:
                    exit_status = self._chan.recv_exit_status()
            except Exception:
                exit_status = None

            with self._lock:
                stop_req = self._stop_requested

            if stop_req:
                self._append_log(f"ONICS-T EXITED (requested stop). exit_status={exit_status}")
                self._set_state("IDLE")
            else:
                self._append_log(f"ONICS-T EXITED (unexpected). exit_status={exit_status}")
                self._set_state("LOS", "ONICS-T ended unexpectedly or SSH stream terminated")
                self._schedule_auto_restart("unexpected exit")

        except Exception as e:
            self._mark_failure()
            msg = f"{type(e).__name__}: {e}"
            tb = traceback.format_exc(limit=2)
            self._append_log(f"SSH STREAM ERROR: {msg}")
            self._append_log(tb)
            with self._lock:
                stop_req = self._stop_requested
            if stop_req:
                self._set_state("IDLE")
            else:
                self._set_state("LOS", msg)
                self._schedule_auto_restart("ssh stream error")
        finally:
            self._ssh_close()


# ---- Flask app ----------------------------------------------------------------


broker = SSEBroker()
controller = OnicsController(broker)

app = Flask(__name__)


@app.get("/")
def index() -> str:
    return render_template("index.html", title=APP_TITLE)


@app.get("/api/snapshot")
def api_snapshot() -> Response:
    return jsonify(controller.snapshot())


@app.get("/api/health")
def api_health() -> Response:
    return jsonify({"ok": True, "ts": time.time()})


@app.post("/api/engage")
def api_engage() -> Response:
    ok, msg = controller.start_onics()
    return jsonify({"ok": ok, "msg": msg, "snapshot": controller.snapshot()}), (200 if ok else 409)


@app.post("/api/disengage")
def api_disengage() -> Response:
    ok, msg = controller.stop_onics()
    return jsonify({"ok": ok, "msg": msg, "snapshot": controller.snapshot()}), (200 if ok else 409)


@app.post("/api/clear")
def api_clear() -> Response:
    controller.clear_logs()
    return jsonify({"ok": True, "snapshot": controller.snapshot()})


@app.post("/api/services/<service_key>/restart")
def api_service_restart(service_key: str) -> Response:
    ok, msg = controller.restart_service(service_key)
    return jsonify({"ok": ok, "msg": msg, "snapshot": controller.snapshot()}), (200 if ok else 409)


@app.post("/api/reboot")
def api_reboot() -> Response:
    ok, msg = controller.reboot_vehicle()
    return jsonify({"ok": ok, "msg": msg, "snapshot": controller.snapshot()}), (200 if ok else 409)


@app.get("/stream")
def stream() -> Response:
    """
    Server-Sent Events stream.

    Emits:
      event: status  -> periodic full snapshot
      event: state   -> state changes
      event: log     -> incremental log lines
    """
    q = broker.subscribe()

    # Initial snapshot and existing log buffer:
    snapshot = controller.snapshot()
    init_payloads: List[str] = []
    init_payloads.append(f"event: status\ndata: {safe_json_dumps(snapshot)}\n\n")
    for line in snapshot.get("logs", []):
        init_payloads.append(f"event: log\ndata: {safe_json_dumps({'line': line, 'ts': now_ms()})}\n\n")

    def gen() -> Iterable[str]:
        try:
            for p in init_payloads:
                yield p
            # Keepalive loop with bounded blocking.
            while True:
                try:
                    payload = q.get(timeout=10.0)
                    yield payload
                except queue.Empty:
                    # SSE keepalive comment prevents some intermediaries from closing the connection.
                    yield ": ping\n\n"
        finally:
            broker.unsubscribe(q)

    return Response(gen(), mimetype="text/event-stream")


def _health_loop(stop_evt: threading.Event) -> None:
    # Initial checks immediately
    controller.health_tick()
    period = max(0.2, 1.0 / max(0.2, HEALTH_TICK_HZ))
    while not stop_evt.is_set():
        t0 = monotonic_s()
        try:
            controller.health_tick()
        except Exception:
            # Health loop must never crash the process.
            pass
        dt = monotonic_s() - t0
        sleep_s = max(0.0, period - dt)
        stop_evt.wait(timeout=sleep_s)


def _install_signal_handlers(
    stop_evt: threading.Event, shutdown_server: Callable[[], None]
) -> None:
    shutdown_started = threading.Event()

    def _handler(signum: int, _frame: Any) -> None:
        stop_evt.set()
        if shutdown_started.is_set():
            return
        shutdown_started.set()
        threading.Thread(
            target=shutdown_server,
            name=f"shutdown-signal-{signum}",
            daemon=True,
        ).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _handler)
        except Exception:
            pass


def main() -> int:
    # Detect tailscale presence early; do not hard-exit, but make it obvious.
    if not which_or_none("tailscale"):
        sys.stderr.write(
            "WARNING: tailscale binary not found. Tailnet connectivity checks will fail.\n"
        )

    stop_evt = threading.Event()

    t = threading.Thread(target=_health_loop, args=(stop_evt,), name="health-loop", daemon=True)
    t.start()

    # Run Flask with a shutdown handle for clean SIGTERM/SIGINT exits.
    server = make_server(
        FLASK_HOST,
        FLASK_PORT,
        app,
        threaded=True,
        request_handler=QuietWSGIRequestHandler,
    )
    _install_signal_handlers(stop_evt, server.shutdown)
    try:
        server.serve_forever()
    finally:
        stop_evt.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
