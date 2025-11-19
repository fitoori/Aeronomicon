#!/usr/bin/env python3
##################################################################
##  ONICS – Optical Navigation and Interference Control System  ##
##                           Mk.II v0.5.3                       ##
##   env-tunable · auto-restart · T265 gating · WPNAV sync      ##
##################################################################
"""
Bridges ArduPilot ↔ sensors/cameras ↔ SiK radio, maintains an arming lock,
and keeps cruise speed in sync with an external air-density calculator.

Run `./ONICS.py --help` for a complete CLI/ENV overview.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import logging
import logging.handlers
import math
import multiprocessing as mp
import os
import queue
import random
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, List, Optional

from pymavlink import mavutil

# ───────────────────────── constants ─────────────────────────
SLEEP_FAST = 0.25            # seconds
SLEEP_SLOW = 5.0             # seconds
RESTART_BACKOFF_MAX = 60.0   # seconds
LOG_MAX_BYTES = 2 * 1024 * 1024
LOG_BACKUP_COUNT = 5

# ───────────────────────── helper functions ─────────────────────────
def env_bool(key: str, default: bool) -> bool:
    val = os.getenv(key)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")

def env_float(key: str, default: float) -> float:
    raw = os.getenv(key, None)
    try:
        return float(raw if raw is not None else default)
    except (TypeError, ValueError):
        logging.warning("Invalid float for %s; using default %.3f", key, default)
        return default

def env_path(key: str, default: str) -> str:
    return os.getenv(key, default)

def validate_baud(baud: int) -> int:
    valid = {57600, 115200, 230400, 460800, 921600}
    if baud not in valid:
        logging.warning("Unsupported baud %d; falling back to 921600", baud)
        return 921600
    return baud

def ensure_dir(path: Path) -> None:
    try:
        path.mkdir(parents=True, exist_ok=True)
    except PermissionError as exc:
        sys.exit(f"Cannot create directory {path}: {exc}")


def ensure_lock_dir() -> None:
    """Create and validate the lock directory."""

    if not LOCK_DIR.is_absolute():
        sys.exit(f"Lock directory must be absolute: {LOCK_DIR}")

    try:
        LOCK_DIR.mkdir(mode=0o755, parents=True, exist_ok=True)
    except OSError as exc:
        sys.exit(f"Cannot create lock directory {LOCK_DIR}: {exc}")

    if not LOCK_DIR.is_dir():
        sys.exit(f"Lock directory path is not a directory: {LOCK_DIR}")

    try:
        os.chmod(LOCK_DIR, 0o755)
    except OSError as exc:
        logging.warning("Unable to set permissions on %s: %s", LOCK_DIR, exc)


def _validate_lock_path(path: Path) -> None:
    if not path.is_absolute():
        raise ValueError(f"Lock path must be absolute: {path}")
    if path.parent != LOCK_DIR:
        raise ValueError(f"Lock path must reside within {LOCK_DIR}: {path}")
    if path.exists() and path.is_dir():
        raise ValueError(f"Lock path points to a directory: {path}")


def _open_lock_file(path: Path) -> int:
    _validate_lock_path(path)
    try:
        return os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    except OSError as exc:
        raise OSError(f"Unable to open lock file {path}: {exc}") from exc


def _acquire_lock(fd: int, path: Path) -> None:
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError(f"Lock already held: {path}") from exc
    except OSError as exc:
        raise RuntimeError(f"Unable to acquire lock {path}: {exc}") from exc


def _release_lock(fd: int, path: Path) -> None:
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    except OSError as exc:
        logging.error("Failed to release lock %s: %s", path, exc)


def _close_fd(fd: int, path: Path) -> None:
    try:
        os.close(fd)
    except OSError as exc:
        logging.error("Failed to close lock file %s: %s", path, exc)

def _validate_udp_hostport(hp: str) -> str:
    try:
        host, port_s = hp.rsplit(":", 1)
        port = int(port_s)
        if not host or not (0 < port < 65536):
            raise ValueError
        return f"{host}:{port}"
    except Exception:
        sys.exit(f"Invalid UDP host:port: {hp}")

def _is_serial_path(s: str) -> bool:
    return s.startswith("/dev/")

def _has_serial_baud_spec(s: str) -> bool:
    # mavproxy serial syntax usually "/dev/ttyUSB0,57600,8N1"
    return "," in s

def _finite(v: float) -> bool:
    return math.isfinite(v)

def _jittered(v: float) -> float:
    return max(0.5, min(RESTART_BACKOFF_MAX, v * random.uniform(0.8, 1.2)))

# ───────────────────────── configuration (env-tunable) ─────────────────────────
LOG_FILE_PATH = Path(env_path("ONICS_LOG", "/home/pi/onics.log"))
ensure_dir(LOG_FILE_PATH.parent)

LOCK_DIR = Path("/run/watne")
ONICS_LOCK_PATH = LOCK_DIR / "onics.lock"
ARMED_LOCK_PATH = LOCK_DIR / "armed.lock"

CONN_IN_PORT = _validate_udp_hostport(os.getenv("ONICS_MAV_PORT", "127.0.0.1:14550"))
CONN_IN_BAUD = validate_baud(int(os.getenv("ONICS_MAV_BAUD", "921600")))

CONN_OUT_P01 = _validate_udp_hostport(os.getenv("ONICS_T265_PORT", "127.0.0.1:14540"))
CONN_OUT_P02 = _validate_udp_hostport(os.getenv("ONICS_D4_PORT", "127.0.0.1:14560"))
CONN_OUT_P03 = os.getenv(
    "ONICS_SIK_DEV",
    "/dev/usb-FTDI_FT230X_Basic_UART_D30AAUZG-if00-port0",  # Holybro/FT230X default-like path
)

T265_GATE = env_bool("ONICS_T265_GATE", True)
T265_STOP_STREAM = env_bool("ONICS_T265_STOP_STREAM", True)
T265_GATE_HI = env_float("ONICS_T265_GATE_HI", 15.0)
T265_GATE_LO = env_float("ONICS_T265_GATE_LO", 10.0)
T265_GATE_ON = env_float("ONICS_T265_GATE_ON", 5.0)
T265_DEBUG = env_bool("ONICS_T265_DEBUG", False)

ENV_PATH = Path(env_path("ONICS_ENV_PATH", "/home/pi/.env.json"))
CRUISE_PARAM = os.getenv("ONICS_WPNAV_PARAM", "WPNAV_SPEED")
SYNC_PERIOD_S = int(env_float("ONICS_WPNAV_PERIOD", 10.0))
ENABLE_WPNAV = env_bool("ONICS_WPNAV_ENABLE", True)

REALSENSE_IDS = ("T265", "D4", "D435", "D455")

# ───────────────────────── logging (multiprocess-safe) ─────────────────────────
from logging.handlers import QueueHandler, QueueListener
_log_queue: "queue.Queue[logging.LogRecord]" = queue.Queue()
_log_listener: Optional[QueueListener] = None

def _utc_formatter() -> logging.Formatter:
    fmt = logging.Formatter("%(asctime)sZ - %(levelname)s - %(processName)s - %(message)s")
    fmt.converter = time.gmtime
    return fmt

def setup_master_logging() -> None:
    global _log_listener
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    rot = logging.handlers.RotatingFileHandler(
        LOG_FILE_PATH, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUP_COUNT
    )
    fmt = _utc_formatter()
    rot.setFormatter(fmt)
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(fmt)
    _log_listener = QueueListener(_log_queue, rot, stdout_handler, respect_handler_level=True)
    _log_listener.start()

def setup_worker_logging() -> None:
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    root.setLevel(logging.INFO)
    root.addHandler(QueueHandler(_log_queue))

# ───────────────────── path/helpers ─────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
D4_SCRIPT = SCRIPT_DIR / "d4xx_to_mavlink.py"
T265_SCRIPT = SCRIPT_DIR / "t265_precland_apriltags.py"

def require_cmd(cmd: str) -> None:
    if shutil.which(cmd) is None:
        sys.exit(f"Required command not found in PATH: {cmd}")

def require_file(p: Path) -> None:
    if not p.exists():
        sys.exit(f"Required file missing: {p}")
    if not os.access(p, os.R_OK):
        sys.exit(f"Required file not readable: {p}")

def preflight() -> None:
    ensure_dir(LOG_FILE_PATH.parent)
    require_cmd("python3")
    require_cmd("mavproxy.py")
    require_cmd("rs-enumerate-devices")
    require_file(T265_SCRIPT)
    require_file(D4_SCRIPT)
    # If SiK is a serial path, ensure it exists and has a baud spec in MAVProxy --out
    if _is_serial_path(CONN_OUT_P03):
        if not os.path.exists(CONN_OUT_P03.split(",", 1)[0]):
            sys.exit(f"SiK device not present: {CONN_OUT_P03}")
        if not _has_serial_baud_spec(CONN_OUT_P03):
            logging.warning("SiK --out missing baud spec (e.g. /dev/ttyUSB0,57600). Current: %s", CONN_OUT_P03)

# ───────────────────── rs-enumerate helpers (JSON) ─────────────────────
def run_rs_enum_json() -> Optional[dict]:
    try:
        out = subprocess.run(
            ["rs-enumerate-devices", "--json"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        return json.loads(out)
    except (subprocess.SubprocessError, FileNotFoundError, json.JSONDecodeError) as exc:
        logging.error("rs-enumerate-devices failed: %s", exc)
        return None

def is_device_connected(prefix: str) -> bool:
    data = run_rs_enum_json()
    if not data:
        return False
    devices = data.get("devices", [])
    for dev in devices:
        name = str(dev.get("name", "")).upper()
        pline = str(dev.get("product-line", "")).upper()
        if name.startswith(prefix.upper()) or pline.startswith(prefix.upper()):
            return True
    return False

def enumerate_devices() -> None:
    data = run_rs_enum_json()
    if not data:
        print("RealSense enumeration failed.")
        return
    devices = data.get("devices", [])
    hits = [
        f'{d.get("name","?")} | {d.get("serial_number","?")} | {d.get("product-line","?")}'
        for d in devices
        if any(tag in str(d.get("name","")) or tag in str(d.get("product-line","")) for tag in REALSENSE_IDS)
    ]
    if hits:
        print("Detected RealSense devices:")
        for h in hits:
            print(" ", h)
    else:
        print("No relevant RealSense devices found.")

# ───────────────────────── lock helpers ─────────────────────────
def acquire_onics_lock() -> int:
    ensure_lock_dir()
    try:
        fd = _open_lock_file(ONICS_LOCK_PATH)
        _acquire_lock(fd, ONICS_LOCK_PATH)
        return fd
    except Exception as exc:
        sys.exit(f"Unable to secure ONICS lock: {exc}")


def release_onics_lock(fd: Optional[int]) -> None:
    if fd is None:
        return
    _release_lock(fd, ONICS_LOCK_PATH)
    _close_fd(fd, ONICS_LOCK_PATH)

# ───────────────────── subprocess utilities ─────────────────────
def _popen(cmd: List[str]) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        start_new_session=True,          # new process group
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        close_fds=True,
    )

def _kill_proc(proc: subprocess.Popen, timeout: float = 2.5) -> None:
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=timeout)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass

# ───────────────────── MAVProxy bridge (self-restarting) ─────────────────────
def mavproxy_bridge() -> None:
    setup_worker_logging()
    cmd = [
        "mavproxy.py",
        f"--master={CONN_IN_PORT}",
        f"--baudrate={CONN_IN_BAUD}",
        "--out", f"udp:{CONN_OUT_P01}",
        "--out", f"udp:{CONN_OUT_P02}",
        "--out", CONN_OUT_P03,
    ]
    backoff = 1.0
    while True:
        try:
            proc = _popen(cmd)
            ret = proc.wait()
            logging.warning("MAVProxy exited (%s); restart in %.1fs", ret, backoff)
        except FileNotFoundError as exc:
            logging.error("MAVProxy not found: %s", exc)
            return
        except Exception as exc:
            logging.error("MAVProxy launcher error: %s", exc)
        time.sleep(backoff)
        backoff = _jittered(min(backoff * 2.0, RESTART_BACKOFF_MAX))

# ─────────────── generic RealSense-bridge launcher ───────────────
def device_loop(tag: str, script: Path, udp: str, extra_flags: Optional[List[str]] = None) -> None:
    setup_worker_logging()
    extra_flags = extra_flags or []
    proc: Optional[subprocess.Popen] = None
    backoff = 1.0
    while True:
        try:
            if is_device_connected(tag):
                if proc is None or proc.poll() is not None:
                    cmd = [sys.executable, str(script), "--connect", f"udp:{udp}", *extra_flags]
                    proc = _popen(cmd)
                    logging.info("%s bridge started (PID %d)", tag, proc.pid)
                    backoff = 1.0
                time.sleep(SLEEP_SLOW)
            else:
                if proc:
                    logging.info("%s disconnected – terminating bridge", tag)
                    _kill_proc(proc)
                    proc = None
                time.sleep(SLEEP_SLOW)
        except Exception as exc:
            logging.error("%s bridge loop error: %s", tag, exc)
            if proc:
                _kill_proc(proc)
                proc = None
            time.sleep(backoff)
            backoff = _jittered(min(backoff * 2.0, RESTART_BACKOFF_MAX))

def run_d4xx() -> None:
    device_loop("D4", D4_SCRIPT, CONN_OUT_P02)

def run_t265() -> None:
    flags: List[str] = []
    if T265_GATE:
        flags.append("--gate")
    if T265_STOP_STREAM:
        flags.append("--stop_stream")
    if T265_DEBUG:
        flags.append("--debug")
    flags += [
        "--gate_hi", str(T265_GATE_HI),
        "--gate_lo", str(T265_GATE_LO),
        "--gate_on", str(T265_GATE_ON),
    ]
    device_loop("T265", T265_SCRIPT, CONN_OUT_P01, flags)

# ───────────────────────── WPNAV synchroniser ─────────────────────────
def cruise_speed_sync() -> None:
    setup_worker_logging()
    if not ENABLE_WPNAV:
        logging.info("WPNAV sync disabled")
        return
    try:
        master = mavutil.mavlink_connection(f"udp:{CONN_IN_PORT}")
        master.wait_heartbeat(timeout=10)
    except Exception as exc:
        logging.error("MAVLink connect failed: %s", exc)
        return

    last_val: Optional[float] = None
    last_push_time = time.monotonic()
    while True:
        try:
            now = time.monotonic()
            if (now - last_push_time) >= SYNC_PERIOD_S:
                last_push_time = now
                try:
                    with open(ENV_PATH, "r", encoding="utf-8") as fp:
                        val = float(json.load(fp).get("cruise_speed_cm_s"))
                except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
                    logging.debug("ENV read error: %s", exc)
                    continue

                if not _finite(val) or val <= 0.0 or val > 300_000.0:
                    logging.warning("Ignoring invalid %s value: %s", CRUISE_PARAM, val)
                    continue

                # push at least every 5 min (300s)
                push_due = (last_val is None) or (val != last_val) or ((now % 300.0) < SYNC_PERIOD_S)
                if push_due:
                    try:
                        master.wait_heartbeat(timeout=3)
                    except Exception:
                        logging.warning("No heartbeat; skipping param push")
                        time.sleep(SLEEP_FAST)
                        continue

                    master.mav.param_set_send(
                        master.target_system,
                        master.target_component,
                        CRUISE_PARAM.encode("ascii", "ignore"),
                        float(val),
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                    )
                    logging.info("Set %s → %.1f cm/s", CRUISE_PARAM, val)
                    last_val = val
            time.sleep(SLEEP_FAST)
        except Exception as exc:
            logging.error("WPNAV sync error: %s", exc)
            time.sleep(SLEEP_SLOW)

# ───────────────────────── arming-status monitor ─────────────────────────
_last_arm_state: Optional[bool] = None


def _close_armed_fd(fd: Optional[int]) -> None:
    if fd is None:
        return
    _release_lock(fd, ARMED_LOCK_PATH)
    _close_fd(fd, ARMED_LOCK_PATH)


def monitor_arming() -> None:
    setup_worker_logging()
    ensure_lock_dir()
    global _last_arm_state
    armed_fd: Optional[int] = None

    def _shutdown(signo: int, _frm) -> None:
        logging.info("Arming monitor shutdown signal %d", signo)
        _close_armed_fd(armed_fd)
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGHUP, _shutdown)

    while True:
        try:
            mav = mavutil.mavlink_connection(f"udp:{CONN_IN_PORT}", input=False)
            mav.wait_heartbeat(timeout=10)
            while True:
                hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if hb is None:
                    break
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != _last_arm_state:
                    try:
                        if armed:
                            fd = _open_lock_file(ARMED_LOCK_PATH)
                            _acquire_lock(fd, ARMED_LOCK_PATH)
                            armed_fd = fd
                        else:
                            _close_armed_fd(armed_fd)
                            armed_fd = None
                        _last_arm_state = armed
                        logging.info("Armed state → %s", armed)
                    except Exception as exc:
                        logging.error("Arming lock update failed: %s", exc)
                        time.sleep(SLEEP_FAST)
        except Exception as exc:
            logging.error("Arming monitor error: %s", exc)
            time.sleep(SLEEP_SLOW)
        finally:
            if armed_fd is not None and _last_arm_state is False:
                _close_armed_fd(armed_fd)
                armed_fd = None

# ───────────────────────── process supervision ─────────────────────────
@dataclass
class ProcSpec:
    name: str
    target: Callable[[], None]
    process: Optional[mp.Process] = None
    backoff: float = 1.0
    last_restart: float = 0.0

def _proc_wrapper(fn: Callable[[], None]) -> Callable[[], None]:
    def wrapped() -> None:
        setup_worker_logging()
        fn()
    return wrapped

def spawn(spec: ProcSpec) -> ProcSpec:
    p = mp.Process(target=_proc_wrapper(spec.target), name=spec.name, daemon=True)
    p.start()
    logging.info("Launched %s (PID %d)", spec.name, p.pid)
    spec.process = p
    return spec

def supervise(specs: List[ProcSpec]) -> None:
    # simple status log on interval
    next_status = time.monotonic() + 30.0
    while True:
        now = time.monotonic()
        for spec in specs:
            p = spec.process
            if p is None or not p.is_alive():
                if p is not None:
                    logging.warning("%s died (exit %s); restarting after %.1fs",
                                    spec.name, p.exitcode, spec.backoff)
                time.sleep(spec.backoff)
                spec = spawn(spec)
                spec.last_restart = now
                spec.backoff = _jittered(min(spec.backoff * 2.0, RESTART_BACKOFF_MAX))
        if now >= next_status:
            alive = ", ".join(f"{s.name}:{'up' if s.process and s.process.is_alive() else 'down'}" for s in specs)
            logging.info("Supervisor status: %s", alive)
            next_status = now + 30.0
        time.sleep(1.0)

# ──────────────────────────── CLI / main ────────────────────────────
_HELP_EPILOG = f"""
ENVIRONMENT VARIABLES (defaults in brackets)
————————————————————————————————————————————————
  ONICS_LOG             Log file path [{str(LOG_FILE_PATH)}]
  ONICS_MAV_PORT        MAVLink input endpoint [{CONN_IN_PORT}]
  ONICS_MAV_BAUD        MAVLink baudrate [{CONN_IN_BAUD}]
  ONICS_T265_PORT       UDP out port for T265 [{CONN_OUT_P01}]
  ONICS_D4_PORT         UDP out port for D4XX [{CONN_OUT_P02}]
  ONICS_SIK_DEV         Serial device for SiK radio or MAVProxy endpoint [{CONN_OUT_P03}]

  — T265 altitude gate —
    ONICS_T265_GATE           Enable altitude gate [{T265_GATE}]
    ONICS_T265_STOP_STREAM    Stop stream when gated [{T265_STOP_STREAM}]
    ONICS_T265_GATE_HI/LO/ON  Altitude thresholds
    ONICS_T265_DEBUG          Extra verbose RealSense logs

  — WPNAV synchroniser —
    ONICS_ENV_PATH       JSON file produced by air-density calc [{str(ENV_PATH)}]
    ONICS_WPNAV_PARAM    FCU parameter to write [{CRUISE_PARAM}]
    ONICS_WPNAV_PERIOD   Seconds between syncs [{SYNC_PERIOD_S}]
    ONICS_WPNAV_ENABLE   Master on/off switch [{ENABLE_WPNAV}]

RUNTIME FLAGS
————————————————
  -e / --enumerate      List connected RealSense devices and exit
  -d / --disable-t265   Skip launching the T265 helper
  -t / --disable-d4xx   Skip launching the D4XX helper
  -s / --sik-only       Only run MAVProxy/SiK bridge (disables cameras)

EXAMPLES
————————————————
  $ ./ONICS.py --sik-only
  $ ONICS_WPNAV_PERIOD=15 ./ONICS.py
  $ ONICS_T265_PORT=127.0.0.1:14600 ONICS_D4_PORT=127.0.0.1:14610 ./ONICS.py
"""

def main() -> None:
    mp.set_start_method("fork", force=True)
    setup_master_logging()
    ensure_lock_dir()
    onics_lock_fd = acquire_onics_lock()
    preflight()

    ap = argparse.ArgumentParser(
        prog="ONICS",
        description=(
            "ONICS Mk II – Optical Navigation & Interference Control System\n"
            "Bridges ArduPilot ↔ sensors ↔ SiK radio, maintains an arming lock, "
            "and auto-tunes cruise speed."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP_EPILOG,
    )
    ap.add_argument("-e", "--enumerate", action="store_true")
    ap.add_argument("-d", "--disable-t265", action="store_true")
    ap.add_argument("-t", "--disable-d4xx", action="store_true")
    ap.add_argument("-s", "--sik-only", action="store_true")
    ap.add_argument("-V", "--version", action="version", version="ONICS Mk II v0.5.3")
    args = ap.parse_args()

    # Configuration dump (once)
    logging.info(
        "Config: in=%s baud=%d; out_t265=udp:%s out_d4=udp:%s out_sik=%s; env_path=%s; wpnav=%s/%ss; gate=%s hi=%.2f lo=%.2f on=%.2f",
        CONN_IN_PORT, CONN_IN_BAUD, CONN_OUT_P01, CONN_OUT_P02, CONN_OUT_P03, str(ENV_PATH),
        CRUISE_PARAM, SYNC_PERIOD_S, T265_GATE, T265_GATE_HI, T265_GATE_LO, T265_GATE_ON
    )

    if args.enumerate:
        enumerate_devices()
        if _log_listener:
            _log_listener.stop()
        return

    t265_ok = not (args.disable_t265 or args.sik_only)
    d4xx_ok = not (args.disable_d4xx or args.sik_only)

    specs: List[ProcSpec] = [
        ProcSpec("mavproxy", mavproxy_bridge),
        ProcSpec("arming", monitor_arming),
        ProcSpec("wpnav", cruise_speed_sync),
    ]
    if t265_ok:
        specs.append(ProcSpec("t265", run_t265))
    if d4xx_ok:
        specs.append(ProcSpec("d4xx", run_d4xx))

    # initial spawn
    specs = [spawn(s) for s in specs]

    def _shutdown(signo, _frm):
        logging.info("Shutdown signal %d", signo)
        for s in specs:
            p = s.process
            if p and p.is_alive():
                p.terminate()
                p.join(timeout=5)
        if _log_listener:
            _log_listener.stop()
        release_onics_lock(onics_lock_fd)
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGHUP, _shutdown)

    try:
        supervise(specs)
    finally:
        if _log_listener:
            _log_listener.stop()
        release_onics_lock(onics_lock_fd)

if __name__ == "__main__":
    main()
