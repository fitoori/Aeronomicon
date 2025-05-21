#!/usr/bin/env python3
##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##                             Mk.II                            ##
##                  *now with multi-core support*               ##
##################################################################
# This script is under active development and may not be stable.
# If you experience connection issues, please consider temporarily
# reverting to ONICS 1.

import argparse
import json
import logging
import multiprocessing
import os
import signal
import subprocess
import time
from datetime import datetime, timezone
from typing import List

from pymavlink import mavutil

# ───────────────────────── Configuration ──────────────────────────
LOG_FILE_PATH = "/home/pi/onics.log"
STATUS_FILE   = "/home/pi/arming_status.json"

# MAVLink I/O
CONN_IN_PORT     = "127.0.0.1:14550"
CONN_IN_BAUD     = "921600"
CONN_OUT_P01     = "127.0.0.1:14540"   # T265
CONN_OUT_P02     = "127.0.0.1:14560"   # D435i
CONN_OUT_P03     = "/dev/usb-FTDI_FT230X_Basic_UART_D30AAUZG-if00-port0"  # SiK Radio

# RealSense device substrings we care about
REALSENSE_IDS = ("T265", "D4")

# Logging setup
logging.basicConfig(
    filename=LOG_FILE_PATH,
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

# ─────────────────────── Utility Functions ────────────────────────
def is_device_connected(device_prefix: str) -> bool:
    """Return True if any RealSense device line contains the given prefix."""
    try:
        result = subprocess.run(
            ["rs-enumerate-devices"],
            capture_output=True,
            text=True,
            check=True,
        )
        return any(device_prefix in line for line in result.stdout.splitlines())
    except subprocess.SubprocessError as exc:
        logging.error("RealSense device check failed: %s", exc)
        return False


def enumerate_devices() -> None:
    """Print all attached RealSense devices of interest."""
    try:
        result = subprocess.run(
            ["rs-enumerate-devices"],
            capture_output=True,
            text=True,
            check=True,
        )
        detected = [
            line.strip()
            for line in result.stdout.splitlines()
            if any(dev in line for dev in REALSENSE_IDS)
        ]
        if detected:
            print("Detected RealSense Devices:")
            for dev in detected:
                print(dev)
        else:
            print("No relevant RealSense devices detected.")
    except subprocess.SubprocessError as exc:
        logging.error("RealSense enumeration failed: %s", exc)


# ───────────────────────── MAVProxy Bridge ────────────────────────
def mavproxy_create_connection() -> None:
    """Launch MAVProxy with three output endpoints."""
    try:
        subprocess.run(
            [
                "mavproxy.py",
                f"--master={CONN_IN_PORT}",
                f"--baudrate={CONN_IN_BAUD}",
                "--out", f"udp:{CONN_OUT_P01}",
                "--out", f"udp:{CONN_OUT_P02}",
                "--out",  CONN_OUT_P03,
            ],
            check=True,
        )
    except subprocess.SubprocessError as exc:
        logging.error("MAVProxy exited with error: %s", exc)


# ─────────────── RealSense → MAVLink Helper Processes ─────────────
def run_d4xx() -> None:
    """Spawn D435i-to-MAVLink bridge when a D4-series camera is present."""
    proc = None
    while True:
        try:
            if is_device_connected("D4"):
                if proc is None or proc.poll() is not None:
                    logging.info("Starting D4XX bridge")
                    proc = subprocess.Popen(
                        ["python3", "d4xx_to_mavlink.py", f"--connect={CONN_OUT_P02}"]
                    )
                time.sleep(5)
            else:
                logging.warning("No D4XX device detected")
                if proc:
                    proc.terminate()
                    proc = None
                time.sleep(5)
        except Exception as exc:
            logging.error("D4XX bridge error: %s", exc)
            if proc:
                proc.terminate()
                proc = None
            time.sleep(5)


def run_t265() -> None:
    """Spawn T265-to-MAVLink bridge when the T265 is present."""
    proc = None
    while True:
        try:
            if is_device_connected("T265"):
                if proc is None or proc.poll() is not None:
                    logging.info("Starting T265 bridge")
                    proc = subprocess.Popen(
                        ["python3", "t265_precland_apriltags.py", f"--connect={CONN_OUT_P01}"]
                    )
                time.sleep(5)
            else:
                logging.warning("T265 device not detected")
                if proc:
                    proc.terminate()
                    proc = None
                time.sleep(5)
        except Exception as exc:
            logging.error("T265 bridge error: %s", exc)
            if proc:
                proc.terminate()
                proc = None
            time.sleep(5)


# ────────────────────────── Log Publisher ─────────────────────────
def publish_logs() -> None:
    """Tail the ONICS log file to stdout so the operator sees live events."""
    with open(LOG_FILE_PATH, "r", encoding="utf-8") as logfile:
        logfile.seek(0, os.SEEK_END)
        while True:
            line = logfile.readline()
            if line:
                print(line.rstrip())
            else:
                time.sleep(1)


# ─────────────── Arming-Status JSON Helper (NEW) ──────────────────
_last_armed_state = None  # module-level cache


def _write_arming_status(armed: bool) -> None:
    """Atomically write {'armed': bool, 'timestamp_utc': str} to STATUS_FILE."""
    tmp = f"{STATUS_FILE}.tmp"
    payload = {
        "armed": armed,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    with open(tmp, "w", encoding="utf-8") as fp:
        json.dump(payload, fp)
    os.replace(tmp, STATUS_FILE)  # atomic on POSIX


def _monitor_single_heartbeat(master: mavutil.mavfile) -> None:
    """Block up to 1 s for a heartbeat and write status on change."""
    global _last_armed_state
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if not msg:
        return  # timed out; keep looping
    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed != _last_armed_state:
        _last_armed_state = armed
        try:
            _write_arming_status(armed)
        except Exception as exc:  # never crash the monitor
            logging.error("Failed to write arming status: %s", exc)


def monitor_arming() -> None:
    """Background process: connects to MAVLink and forwards arming state."""
    while True:
        try:
            # input=False disables TX; we only listen
            master = mavutil.mavlink_connection(f"udp:{CONN_IN_PORT}", input=False)
            while True:
                _monitor_single_heartbeat(master)
        except Exception as exc:
            logging.error("Arming monitor failure: %s (retrying in 2 s)", exc)
            time.sleep(2)


# ───────────────────── Process Management Helpers ─────────────────
def terminate_processes(procs: List[multiprocessing.Process]) -> None:
    """Gracefully terminate and join each child process."""
    for p in procs:
        if p.is_alive():
            p.terminate()
            p.join(timeout=2)


# ──────────────────────────── Main CLI ────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="ONICS Mk II – Optical Navigation and Interference Control System",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-e", "--enumerate", action="store_true",
                        help="List connected RealSense devices and exit")
    parser.add_argument("-d", "--disable-t265", action="store_true",
                        help="Disable the T265 helper process")
    parser.add_argument("-t", "--disable-d4xx", action="store_true",
                        help="Disable the D4XX helper process")
    parser.add_argument("-s", "--sik-only",  action="store_true",
                        help="Only launch the SiK radio (disables T265 and D4XX)")
    args = parser.parse_args()

    if args.enumerate:
        enumerate_devices()
        return

    t265_enabled = not (args.disable_t265 or args.sik_only)
    d4xx_enabled = not (args.disable_d4xx or args.sik_only)

    print("ONICS – Optical Navigation and Interference Control System is initializing...")

    # Live log mirror
    log_proc = multiprocessing.Process(target=publish_logs, daemon=True)
    log_proc.start()

    # Core child processes
    processes: List[multiprocessing.Process] = []

    # MAVProxy bridge
    processes.append(multiprocessing.Process(target=mavproxy_create_connection, daemon=True))

    # Arming-status monitor (always running)
    processes.append(multiprocessing.Process(target=monitor_arming, daemon=True))

    # Optional RealSense helpers
    if t265_enabled:
        processes.append(multiprocessing.Process(target=run_t265, daemon=True))
    if d4xx_enabled:
        processes.append(multiprocessing.Process(target=run_d4xx, daemon=True))

    # Start everything
    for p in processes:
        p.start()

    # ───────────── Graceful shutdown on SIGINT/SIGTERM ─────────────
    def _shutdown_handler(signum, _frame):
        logging.info("Shutdown requested (signal %d)", signum)
        terminate_processes(processes)
        if log_proc.is_alive():
            log_proc.terminate()
            log_proc.join(timeout=2)
        print("All processes terminated.")
        raise SystemExit(0)

    signal.signal(signal.SIGINT,  _shutdown_handler)
    signal.signal(signal.SIGTERM, _shutdown_handler)

    # Keep parent alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _shutdown_handler(signal.SIGINT, None)


if __name__ == "__main__":
    main()
