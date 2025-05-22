#!/usr/bin/env python3
##################################################################
##  ONICS – Optical Navigation and Interference Control System  ##
##                           Mk.II v0.5.2                       ##
##   env-tunable · auto-restart · T265 gating · WPNAV sync      ##
##################################################################
"""
Bridges ArduPilot ↔ sensors ↔ SiK radio, publishes arming status,
and keeps cruise speed in sync with an external air-density calculator.

Run `./ONICS.py --help` for a complete CLI/ENV overview.
"""
from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import multiprocessing as mp
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from pymavlink import mavutil

# ───────────────────────── constants ─────────────────────────
SLEEP_FAST = 0.25          # seconds
SLEEP_SLOW = 5.0           # seconds
RESTART_BACKOFF_MAX = 60   # seconds
LOG_MAX_BYTES = 2 * 1024 * 1024
LOG_BACKUP_COUNT = 5

# ───────────────────────── helper functions ─────────────────────────
def env_bool(key: str, default: bool) -> bool:
    val = os.getenv(key)
    if val is None:
        return default
    return val.lower() in ("1", "true", "yes", "on")


def env_float(key: str, default: float) -> float:
    try:
        return float(os.getenv(key, str(default)))
    except ValueError:
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


# ───────────────────────── configuration (env-tunable) ─────────────────────────
LOG_FILE_PATH = Path(env_path("ONICS_LOG", "/home/pi/onics.log"))
ensure_dir(LOG_FILE_PATH.parent)

STATUS_FILE = Path(env_path("ONICS_ARM_JSON", "/home/pi/arming_status.json"))

CONN_IN_PORT = os.getenv("ONICS_MAV_PORT", "127.0.0.1:14550")
CONN_IN_BAUD = validate_baud(int(os.getenv("ONICS_MAV_BAUD", "921600")))

CONN_OUT_P01 = os.getenv("ONICS_T265_PORT", "127.0.0.1:14540")
CONN_OUT_P02 = os.getenv("ONICS_D4_PORT", "127.0.0.1:14560")
CONN_OUT_P03 = os.getenv(
    "ONICS_SIK_DEV",
    "/dev/usb-FTDI_FT230X_Basic_UART_D30AAUZG-if00-port0",
)

T265_GATE = env_bool("ONICS_T265_GATE", True)
T265_STOP_STREAM = env_bool("ONICS_T265_STOP_STREAM", True)
T265_GATE_HI = env_float("ONICS_T265_GATE_HI", 15.0)
T265_GATE_LO = env_float("ONICS_T265_GATE_LO", 10.0)
T265_GATE_ON = env_float("ONICS_T265_GATE_ON", 5.0)
T265_DEBUG = env_bool("ONICS_T265_DEBUG", False)

ENV_PATH = Path(env_path("ONICS_ENV_PATH", "/home/pi/.env.json"))
CRUISE_PARAM = os.getenv("ONICS_WPNAV_PARAM", "WPNAV_SPEED")
SYNC_PERIOD_S = int(env_float("ONICS_WPNAV_PERIOD", 10))
ENABLE_WPNAV = env_bool("ONICS_WPNAV_ENABLE", True)

REALSENSE_IDS = ("T265", "D4")

# ───────────────────────── logging ─────────────────────────
logger = logging.getLogger()
logger.setLevel(logging.INFO)

rot = logging.handlers.RotatingFileHandler(
    LOG_FILE_PATH, maxBytes=LOG_MAX_BYTES, backupCount=LOG_BACKUP_COUNT
)
rot.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
logger.addHandler(rot)

stdout_handler = logging.StreamHandler(sys.stdout)
stdout_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
logger.addHandler(stdout_handler)

# ───────────────────── path helpers ─────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
D4_SCRIPT = SCRIPT_DIR / "d4xx_to_mavlink.py"
T265_SCRIPT = SCRIPT_DIR / "t265_precland_apriltags.py"

# ───────────────────── device enumeration helpers ─────────────────────
def run_rs_enum() -> Optional[str]:
    try:
        out = subprocess.run(
            ["rs-enumerate-devices", "--json"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        return out
    except (subprocess.SubprocessError, FileNotFoundError) as exc:
        logging.error("rs-enumerate-devices failed: %s", exc)
        return None


def is_device_connected(prefix: str) -> bool:
    out = run_rs_enum()
    if not out:
        return False
    return prefix in out


def enumerate_devices() -> None:
    out = run_rs_enum()
    if not out:
        print("RealSense enumeration failed.")
        return
    lines = [ln.strip() for ln in out.splitlines() if any(tag in ln for tag in REALSENSE_IDS)]
    if lines:
        print("Detected RealSense devices:")
        for ln in lines:
            print(" ", ln)
    else:
        print("No relevant RealSense devices found.")

# ───────────────────── MAVProxy bridge (self-restarting) ─────────────────────
def mavproxy_bridge() -> None:
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
            proc = subprocess.Popen(cmd)
            ret = proc.wait()
            logging.warning("MAVProxy exited (%s); restart in %.1fs", ret, backoff)
        except FileNotFoundError as exc:
            logging.error("MAVProxy not found: %s", exc)
            return
        except Exception as exc:
            logging.error("MAVProxy launcher error: %s", exc)
        time.sleep(backoff)
        backoff = min(backoff * 2, RESTART_BACKOFF_MAX)

# ─────────────── generic RealSense-bridge launcher ───────────────
def device_loop(tag: str, script: Path, udp: str, extra_flags: List[str] | None = None) -> None:
    extra_flags = extra_flags or []
    proc: Optional[subprocess.Popen] = None
    backoff = 1.0
    while True:
        try:
            if is_device_connected(tag):
                if proc is None or proc.poll() is not None:
                    cmd = ["python3", str(script), "--connect", f"udp:{udp}", *extra_flags]
                    proc = subprocess.Popen(cmd)
                    logging.info("%s bridge started (PID %d)", tag, proc.pid)
                    backoff = 1.0
                time.sleep(SLEEP_SLOW)
            else:
                if proc:
                    logging.info("%s disconnected – terminating bridge", tag)
                    proc.terminate()
                    proc.wait(timeout=2)
                    proc = None
                time.sleep(SLEEP_SLOW)
        except Exception as exc:
            logging.error("%s bridge loop error: %s", tag, exc)
            if proc:
                proc.terminate()
                proc.wait(timeout=2)
                proc = None
            time.sleep(backoff)
            backoff = min(backoff * 2, RESTART_BACKOFF_MAX)

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
    last_push_time = time.time()
    while True:
        try:
            now = time.time()
            if now - last_push_time >= SYNC_PERIOD_S:
                last_push_time = now
                try:
                    with open(ENV_PATH, "r", encoding="utf-8") as fp:
                        val = float(json.load(fp).get("cruise_speed_cm_s"))
                except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
                    logging.debug("ENV read error: %s", exc)
                    continue

                if val != last_val or (now % 300 < SYNC_PERIOD_S):  # push at least every 5 min
                    master.mav.param_set_send(
                        master.target_system,
                        master.target_component,
                        CRUISE_PARAM.encode(),
                        val,
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

def _write_arm_json(armed: bool) -> None:
    tmp = STATUS_FILE.with_suffix(".tmp")
    payload: Dict[str, object] = {
        "armed": armed,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump(payload, fp)
        fp.flush()
        os.fsync(fp.fileno())
    tmp.replace(STATUS_FILE)

def monitor_arming() -> None:
    global _last_arm_state
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
                    _last_arm_state = armed
                    try:
                        _write_arm_json(armed)
                        logging.info("Armed state → %s", armed)
                    except OSError as exc:
                        logging.error("Arming JSON write failed: %s", exc)
        except Exception as exc:
            logging.error("Arming monitor error: %s", exc)
            time.sleep(SLEEP_SLOW)

# ───────────────────────── process supervision ─────────────────────────
def spawn(target, name: str) -> mp.Process:
    p = mp.Process(target=target, name=name, daemon=True)
    p.start()
    logging.info("Launched %s (PID %d)", name, p.pid)
    return p

def supervise(procs: List[mp.Process]) -> None:
    while True:
        for i, p in enumerate(procs):
            if not p.is_alive():
                logging.warning("%s died; restarting", p.name)
                procs[i] = spawn(p._target, p.name)  # type: ignore
        time.sleep(1)

# ──────────────────────────── CLI / main ────────────────────────────
_HELP_EPILOG = f"""
ENVIRONMENT VARIABLES (defaults in brackets)
————————————————————————————————————————————————
  ONICS_LOG             Log file path [{LOG_FILE_PATH}]
  ONICS_ARM_JSON        Arming-status JSON [{STATUS_FILE}]
  ONICS_MAV_PORT        MAVLink input endpoint [{CONN_IN_PORT}]
  ONICS_MAV_BAUD        MAVLink baudrate [{CONN_IN_BAUD}]
  ONICS_T265_PORT       UDP out port for T265 [{CONN_OUT_P01}]
  ONICS_D4_PORT         UDP out port for D4XX [{CONN_OUT_P02}]
  ONICS_SIK_DEV         Serial device for SiK radio [{CONN_OUT_P03}]

  — T265 altitude gate —
    ONICS_T265_GATE           Enable altitude gate [{T265_GATE}]
    ONICS_T265_STOP_STREAM    Stop stream when gated [{T265_STOP_STREAM}]
    ONICS_T265_GATE_HI/LO/ON  Altitude thresholds
    ONICS_T265_DEBUG          Extra verbose RealSense logs

  — WPNAV synchroniser —
    ONICS_ENV_PATH       JSON file produced by air-density calc [{ENV_PATH}]
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

    ap = argparse.ArgumentParser(
        prog="ONICS",
        description=(
            "ONICS Mk II – Optical Navigation & Interference Control System\n"
            "Bridges ArduPilot ↔ sensors ↔ SiK radio, publishes arming status, "
            "and auto-tunes cruise speed."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_HELP_EPILOG,
    )
    ap.add_argument("-e", "--enumerate", action="store_true")
    ap.add_argument("-d", "--disable-t265", action="store_true")
    ap.add_argument("-t", "--disable-d4xx", action="store_true")
    ap.add_argument("-s", "--sik-only", action="store_true")
    ap.add_argument("-V", "--version", action="version", version="ONICS Mk II v0.5.2")
    args = ap.parse_args()

    if args.enumerate:
        enumerate_devices()
        return

    t265_ok = not (args.disable_t265 or args.sik_only)
    d4xx_ok = not (args.disable_d4xx or args.sik_only)

    procs: List[mp.Process] = [
        spawn(mavproxy_bridge, "mavproxy"),
        spawn(monitor_arming, "arming"),
        spawn(cruise_speed_sync, "wpnav"),
    ]
    if t265_ok:
        procs.append(spawn(run_t265, "t265"))
    if d4xx_ok:
        procs.append(spawn(run_d4xx, "d4xx"))

    def _shutdown(signo, _frm):
        logging.info("Shutdown signal %d", signo)
        for p in procs:
            if p.is_alive():
                p.terminate()
                p.join(timeout=2)
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    supervise(procs)

if __name__ == "__main__":
    main()
