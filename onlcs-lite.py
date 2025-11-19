#!/usr/bin/env python3
"""
ONICS‑lite – RealSense supervisor
---------------------------------
Based on ONICS2.py (Mk.II v0.5.3), this trimmed‑down version
runs only the RealSense bridges for the T‑265 and D4xx devices.
It watches for cameras connecting/disconnecting and restarts the
helper scripts when needed.  MAVProxy and cruise‑speed syncing
have been removed – run MAVProxy as a separate service.  The
T‑265 bridge uses the dedicated ONICS‑T wrapper (onics-t.py);
the D4xx bridge uses d4xx_to_mavlink.py.

Environment variables (defaults in brackets):
  ONICS_LOG             Log file path [/home/pi/onics.log]
  ONICS_MAV_PORT        MAVLink input for arming monitor [127.0.0.1:14550]
  ONICS_T265_PORT       UDP out port for T265 [127.0.0.1:14540]
  ONICS_D4_PORT         UDP out port for D4XX [127.0.0.1:14560]

T‑265 gate:
  ONICS_T265_GATE       Enable altitude gate [True]
  ONICS_T265_STOP_STREAM Stop stream when gated [True]
  ONICS_T265_GATE_HI/LO/ON  Altitude thresholds [15.0 / 10.0 / 5.0]
  ONICS_T265_DEBUG      Extra verbose RealSense logs [False]

Runtime flags:
  -e / --enumerate      List connected RealSense devices and exit
  -a / --disable-arming Skip managing the arming lock
  -d / --disable-t265   Skip launching the T265 helper
  -t / --disable-d4xx   Skip launching the D4XX helper
"""

from __future__ import annotations

import argparse
import fcntl
import json
import logging
import logging.handlers
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
from pathlib import Path
from typing import Callable, List, Optional

# ───────────────────────── cadence constants ─────────────────────────
SLEEP_FAST = 0.25
SLEEP_SLOW = 5.0

# ───────────────────────── helper functions ─────────────────────────
def env_bool(key: str, default: bool) -> bool:
    """Read a boolean environment variable."""
    val = os.getenv(key)
    if val is None:
        return default
    return val.strip().lower() in ("1", "true", "yes", "on")

def env_float(key: str, default: float) -> float:
    """Read a float environment variable, falling back to default on error."""
    raw = os.getenv(key, None)
    try:
        return float(raw if raw is not None else default)
    except (TypeError, ValueError):
        logging.warning("Invalid float for %s; using default %.3f", key, default)
        return default

def env_path(key: str, default: str) -> str:
    """Read a path from the environment."""
    return os.getenv(key, default)

def ensure_dir(path: Path) -> None:
    """Ensure that a directory exists; exit if not possible."""
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
    """Validate a UDP host:port string."""
    try:
        host, port_s = hp.rsplit(":", 1)
        port = int(port_s)
        if not host or not (0 < port < 65536):
            raise ValueError
        return f"{host}:{port}"
    except Exception:
        sys.exit(f"Invalid UDP host:port: {hp}")

def _finite(v: float) -> bool:
    """Check for finite floats."""
    return not (v is None or v != v or v in (float("inf"), float("-inf")))

def _jittered(v: float) -> float:
    """Return a jittered back‑off time (±20 %)."""
    RESTART_BACKOFF_MAX = 60.0
    return max(0.5, min(RESTART_BACKOFF_MAX, v * random.uniform(0.8, 1.2)))

# ───────────────────────── configuration ─────────────────────────
LOG_FILE_PATH = Path(env_path("ONICS_LOG", "/home/pi/onics.log"))
ensure_dir(LOG_FILE_PATH.parent)

LOCK_DIR = Path("/run/watne")
ONICS_LOCK_PATH = LOCK_DIR / "onics.lock"
ARMED_LOCK_PATH = LOCK_DIR / "armed.lock"

# MAVLink input used only for arming lock tracking
CONN_IN_PORT = _validate_udp_hostport(os.getenv("ONICS_MAV_PORT", "127.0.0.1:14550"))

# UDP endpoints for the RealSense bridges
CONN_OUT_P01 = os.getenv("ONICS_T265_PORT", "127.0.0.1:14540")
CONN_OUT_P02 = os.getenv("ONICS_D4_PORT", "127.0.0.1:14560")

# T‑265 gate & debug flags
T265_GATE = env_bool("ONICS_T265_GATE", True)
T265_STOP_STREAM = env_bool("ONICS_T265_STOP_STREAM", True)
T265_GATE_HI = env_float("ONICS_T265_GATE_HI", 15.0)
T265_GATE_LO = env_float("ONICS_T265_GATE_LO", 10.0)
T265_GATE_ON = env_float("ONICS_T265_GATE_ON", 5.0)
T265_DEBUG = env_bool("ONICS_T265_DEBUG", False)

# RealSense model prefixes to look for
REALSENSE_IDS = ("T265", "D4", "D435", "D455")

# ───────────────────── logging (multiprocess‑safe) ─────────────────────
from logging.handlers import QueueHandler, QueueListener
_log_queue: "queue.Queue[logging.LogRecord]" = queue.Queue()
_log_listener: Optional[QueueListener] = None

def _utc_formatter() -> logging.Formatter:
    fmt = logging.Formatter("%(asctime)sZ - %(levelname)s - %(processName)s - %(message)s")
    fmt.converter = time.gmtime
    return fmt

def setup_master_logging() -> None:
    """Configure root logging with a rotating file handler and stdout."""
    global _log_listener
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    rot = logging.handlers.RotatingFileHandler(
        LOG_FILE_PATH, maxBytes=2 * 1024 * 1024, backupCount=5
    )
    fmt = _utc_formatter()
    rot.setFormatter(fmt)
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(fmt)
    _log_listener = QueueListener(_log_queue, rot, stdout_handler, respect_handler_level=True)
    _log_listener.start()

def setup_worker_logging() -> None:
    """Set up logging for worker processes to use the queue handler."""
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    root.setLevel(logging.INFO)
    root.addHandler(QueueHandler(_log_queue))

# ───────────────────── path/helpers ─────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
D4_SCRIPT = SCRIPT_DIR / "d4xx_to_mavlink.py"
# Use the dedicated ONICS‑T wrapper for the T265 instead of the legacy bridge
T265_SCRIPT = SCRIPT_DIR / "onics-t.py"

def require_cmd(cmd: str) -> None:
    """Ensure a command exists in PATH."""
    if shutil.which(cmd) is None:
        sys.exit(f"Required command not found in PATH: {cmd}")

def require_file(p: Path) -> None:
    """Ensure a required file exists and is readable."""
    if not p.exists():
        sys.exit(f"Required file missing: {p}")
    if not os.access(p, os.R_OK):
        sys.exit(f"Required file not readable: {p}")

def preflight() -> None:
    """Perform pre‑flight checks: ensure dependencies and helper scripts exist."""
    ensure_dir(LOG_FILE_PATH.parent)
    require_cmd("python3")
    require_cmd("rs-enumerate-devices")
    require_file(T265_SCRIPT)
    require_file(D4_SCRIPT)

# ───────────────────── rs‑enumerate helpers (JSON) ─────────────────────
def run_rs_enum_json() -> Optional[dict]:
    """Run rs‑enumerate‑devices --json and parse the output."""
    try:
        out = subprocess.run(
            ["rs-enumerate-devices", "--json"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        return json.loads(out)
    except (subprocess.SubprocessError, FileNotFoundError, json.JSONDecodeError) as exc:
        logging.error("rs‑enumerate‑devices failed: %s", exc)
        return None

def is_device_connected(prefix: str) -> bool:
    """Return True if a RealSense device with the given prefix is connected."""
    data = run_rs_enum_json()
    if not data:
        return False
    for dev in data.get("devices", []):
        name = str(dev.get("name", "")).upper()
        pline = str(dev.get("product-line", "")).upper()
        if name.startswith(prefix.upper()) or pline.startswith(prefix.upper()):
            return True
    return False

def enumerate_devices() -> None:
    """Print a list of connected RealSense devices."""
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
    """Launch a subprocess detached from the parent session."""
    return subprocess.Popen(
        cmd,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        close_fds=True,
    )

def _kill_proc(proc: subprocess.Popen, timeout: float = 2.5) -> None:
    """Terminate a subprocess group gracefully, then force‑kill if needed."""
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=timeout)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass

# ─────────────── generic RealSense‑bridge launcher ───────────────
def device_loop(tag: str, script: Path, udp: str, extra_flags: Optional[List[str]] = None) -> None:
    """
    Start the RealSense helper script when the device is present.
    Restart if the script exits or the device is unplugged.
    """
    setup_worker_logging()
    extra_flags = extra_flags or []
    proc: Optional[subprocess.Popen] = None
    backoff = 1.0
    while True:
        try:
            if is_device_connected(tag):
                if proc is None or proc.poll() is not None:
                    # build the command: python helper --connect udp:<port> ...
                    cmd = [sys.executable, str(script), "--connect", f"udp:{udp}", *extra_flags]
                    proc = _popen(cmd)
                    logging.info("%s bridge started (PID %d)", tag, proc.pid)
                    backoff = 1.0
                time.sleep(5.0)
            else:
                # device disconnected: kill helper if running
                if proc:
                    logging.info("%s disconnected – terminating bridge", tag)
                    _kill_proc(proc)
                    proc = None
                time.sleep(5.0)
        except Exception as exc:
            logging.error("%s bridge loop error: %s", tag, exc)
            if proc:
                _kill_proc(proc)
                proc = None
            time.sleep(backoff)
            backoff = _jittered(min(backoff * 2.0, 60.0))

def run_d4xx() -> None:
    """Launch and supervise the D4xx helper."""
    device_loop("D4", D4_SCRIPT, CONN_OUT_P02)

def run_t265() -> None:
    """Launch and supervise the ONICS‑T helper with gating flags."""
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

# ───────────────────────── arming-status monitor ─────────────────────────
from pymavlink import mavutil

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

# ───────────────────── process supervision ─────────────────────
@dataclass
class ProcSpec:
    name: str
    target: Callable[[], None]
    process: Optional[mp.Process] = None
    backoff: float = 1.0
    last_restart: float = 0.0

def _proc_wrapper(fn: Callable[[], None]) -> Callable[[], None]:
    """Wrap a target function to configure logging in the subprocess."""
    def wrapped() -> None:
        setup_worker_logging()
        fn()
    return wrapped

def spawn(spec: ProcSpec) -> ProcSpec:
    """Spawn a worker process for the given spec."""
    p = mp.Process(target=_proc_wrapper(spec.target), name=spec.name, daemon=True)
    p.start()
    logging.info("Launched %s (PID %d)", spec.name, p.pid)
    spec.process = p
    return spec

def supervise(specs: List[ProcSpec]) -> None:
    """Monitor worker processes and restart them if they die."""
    next_status = time.monotonic() + 30.0
    while True:
        now = time.monotonic()
        for spec in specs:
            p = spec.process
            if p is None or not p.is_alive():
                if p is not None:
                    logging.warning(
                        "%s died (exit %s); restarting after %.1fs",
                        spec.name, p.exitcode, spec.backoff
                    )
                time.sleep(spec.backoff)
                spec = spawn(spec)
                spec.last_restart = now
                spec.backoff = _jittered(min(spec.backoff * 2.0, 60.0))
        if now >= next_status:
            alive = ", ".join(
                f"{s.name}:{'up' if s.process and s.process.is_alive() else 'down'}"
                for s in specs
            )
            logging.info("Supervisor status: %s", alive)
            next_status = now + 30.0
        time.sleep(1.0)

# ───────────────────── CLI / main ─────────────────────
def main() -> None:
    mp.set_start_method("fork", force=True)
    setup_master_logging()
    ensure_lock_dir()
    onics_lock_fd = acquire_onics_lock()
    preflight()

    ap = argparse.ArgumentParser(
        prog="ONICS‑lite",
        description=(
            "ONICS‑lite – supervise RealSense sensors only\n"
            "Launches helper bridges for T265 and D4xx cameras and restarts\n"
            "them if the device is unplugged or the script exits."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("-e", "--enumerate", action="store_true", help="List connected RealSense devices and exit")
    ap.add_argument("-a", "--disable-arming", action="store_true", help="Skip managing the arming lock")
    ap.add_argument("-d", "--disable-t265", action="store_true", help="Skip launching the T265 helper")
    ap.add_argument("-t", "--disable-d4xx", action="store_true", help="Skip launching the D4xx helper")
    ap.add_argument("-V", "--version", action="version", version="ONICS‑lite v0.1.0")
    args = ap.parse_args()

    # dump config once
    logging.info(
        "Config: in_mav=udp:%s locks=%s/%s out_t265=udp:%s out_d4=udp:%s; gate=%s hi=%.2f lo=%.2f on=%.2f",
        CONN_IN_PORT, ONICS_LOCK_PATH, ARMED_LOCK_PATH,
        CONN_OUT_P01, CONN_OUT_P02,
        T265_GATE, T265_GATE_HI, T265_GATE_LO, T265_GATE_ON
    )

    if args.enumerate:
        enumerate_devices()
        if _log_listener:
            _log_listener.stop()
        return

    # build process specs list
    specs: List[ProcSpec] = []
    if not args.disable_arming:
        specs.append(ProcSpec("arming", monitor_arming))
    if not args.disable_t265:
        specs.append(ProcSpec("onics-t", run_t265))
    if not args.disable_d4xx:
        specs.append(ProcSpec("d4xx", run_d4xx))

    # spawn workers
    specs = [spawn(s) for s in specs]

    # graceful shutdown handler
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
