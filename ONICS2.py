#!/usr/bin/env python3
##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##                             Mk.II                            ##
##                  *now with multi-core support*               ##
##################################################################
# This script is under development and may not be stable.
# If you experience connection issues, please consider temporarily reverting to ONICS 1.

import subprocess
import multiprocessing
import logging
import time
import os
import argparse
import signal
import json                                   
from datetime import datetime, timezone       
from pymavlink import mavutil                 

# Default log file path
log_file_path = "/home/pi/onics.log"

# ---------- Arming-status output ----------
STATUS_FILE = "/home/pi/arming_status.json"
_last_armed = None
def _write_status(armed: bool) -> None:
    """Atomically write arming state and timestamp to STATUS_FILE."""
    tmp = f"{STATUS_FILE}.tmp"
    payload = {
        "armed": armed,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec='seconds')
    }
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    os.replace(tmp, STATUS_FILE)

def _log_arming_status(master: mavutil.mavfile) -> None:
    """Monitor HEARTBEATs and write only on state change."""
    global _last_armed
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != _last_armed:
            _last_armed = armed
            try:
                _write_status(armed)
            except Exception as exc:          # never raise—just log
                logging.error("Arming-status write failure: %s", exc)

def monitor_arming():                        # ➋ NEW
    """Background process: connect to MAVLink stream and forward arming status."""
    while True:
        try:
            master = mavutil.mavlink_connection(f"udp:{connection_in_port}", input=False)
            while True:
                _log_arming_status(master)
        except Exception as exc:
            logging.error("Arming-monitor error: %s – retrying in 2 s", exc)
            time.sleep(2)

# ------------------------------------------

# Configure logging
logging.basicConfig(filename=log_file_path, level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Define connection parameters
connection_in_port = "127.0.0.1:14550"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14540"  # T265
connection_out_p02 = "127.0.0.1:14560"  # D435i
connection_out_p03 = "/dev/usb-FTDI_FT230X_Basic_UART_D30AAUZG-if00-port0"  # SiK Radio

# [ functions is_device_connected(), enumerate_devices(), mavproxy_create_connection(),
#   run_d4xx(), run_t265(), publish_logs(), terminate_processes() ... UNCHANGED ]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ONICS - Optical Navigation and Interference Control System")
    parser.add_argument("-e", "--enumerate", action="store_true", help="Enumerate detected RealSense devices")
    parser.add_argument("-d", "--disable-t265", action="store_true", help="Disable the T265 process")
    parser.add_argument("-t", "--disable-d4xx", action="store_true", help="Disable the D4XX process")
    parser.add_argument("-s", "--sik-only", action="store_true",
                        help="Initialize only the SiK radio (disables both T265 and D4XX)")
    args = parser.parse_args()

    if args.enumerate:
        enumerate_devices()
    else:
        t265_enabled = not (args.disable_t265 or args.sik_only)
        d4xx_enabled = not (args.disable_d4xx or args.sik_only)

        log_process = multiprocessing.Process(target=publish_logs)
        log_process.start()

        print("ONICS - Optical Navigation and Interference Control System is initializing...")

        processes = []

        # ── MAVProxy bridge
        mavproxy_process = multiprocessing.Process(target=mavproxy_create_connection)
        processes.append(mavproxy_process)

        # ── Arming-status monitor (always on)
        arming_process = multiprocessing.Process(target=monitor_arming)    # ➌ NEW
        processes.append(arming_process)

        # ── Optional RealSense helpers
        if t265_enabled:
            t265_process = multiprocessing.Process(target=run_t265)
            processes.append(t265_process)
        if d4xx_enabled:
            d4xx_process = multiprocessing.Process(target=run_d4xx)
            processes.append(d4xx_process)

        try:
            for process in processes:
                process.start()
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logging.info("Terminating processes...")
            terminate_processes(processes)
            log_process.terminate(); log_process.join()
            print("All processes terminated.")
