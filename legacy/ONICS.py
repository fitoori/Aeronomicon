#!/usr/bin/env python3

##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##################################################################

# This script uses single-core threading to manage connections. 
# ONICS has been rewritten with multiprocessing and released as ONICS 2.  
# If your device is lagging/freezing/hanging/crashing under load, please consider switching. 


import subprocess
import threading
import logging
import time
import os
import argparse
from pathlib import Path

LOG_ENV_VAR = "ONICS_LOG_PATH"
DEFAULT_LOG_FILE = Path("/home/pi/onics.log")


def _prepare_log_file():
    """Find a writable log file path and ensure it exists.

    We try, in order:
    1. A path specified via the ONICS_LOG_PATH environment variable.
    2. The historical default /home/pi/onics.log path.
    3. $HOME/onics.log
    4. ./onics.log (the current working directory)

    Returning None signals that no writable path could be found, in which
    case we will fall back to console logging only.
    """

    candidates = []
    env_path = os.environ.get(LOG_ENV_VAR)
    if env_path:
        candidates.append(Path(env_path))

    candidates.extend(
        [
            DEFAULT_LOG_FILE,
            Path.home() / "onics.log",
            Path.cwd() / "onics.log",
        ]
    )

    for candidate in candidates:
        try:
            candidate.parent.mkdir(parents=True, exist_ok=True)
            candidate.touch(exist_ok=True)
        except OSError:
            continue
        return candidate

    return None


_log_path = _prepare_log_file()
log_file_path = str(_log_path) if _log_path else None

if _log_path:
    logging.basicConfig(
        filename=log_file_path,
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )
else:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[logging.StreamHandler()],
    )
    logging.warning(
        "Unable to create a log file using %s; falling back to console logging.",
        LOG_ENV_VAR,
    )

# Define connection parameters
connection_in_port = "127.0.0.1:14550"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14540"  # T265
connection_out_p02 = "127.0.0.1:14560"  # D435i
connection_out_p03 = "/dev/usb-FTDI_FT230X_Basic_UART_D30AAUZG-if00-port0"     # SiK Radio

def is_device_connected(device_prefix):
    try:
        result = subprocess.run(["rs-enumerate-devices"], capture_output=True, text=True, check=True)
        return any(device_prefix in line for line in result.stdout.splitlines())
    except subprocess.SubprocessError as e:
        logging.error("Error checking for RealSense devices: %s", e)
        return False

def enumerate_devices():
    relevant_devices = ["T265", "D4"]
    try:
        result = subprocess.run(["rs-enumerate-devices"], capture_output=True, text=True, check=True)
        detected_devices = [line.strip() for line in result.stdout.splitlines() if any(dev in line for dev in relevant_devices)]
        if detected_devices:
            print("Detected RealSense Devices:")
            for device in detected_devices:
                print(device)
        else:
            print("No relevant RealSense devices detected.")
    except subprocess.SubprocessError as e:
        logging.error("Error enumerating RealSense devices: %s", e)

def mavproxy_create_connection():
    try:
        subprocess.run(["mavproxy.py",
                        f"--master={connection_in_port}",
                        f"--baudrate={connection_in_baud}",
                        "--out", f"udp:{connection_out_p01}",
                        "--out", f"udp:{connection_out_p02}",
                        "--out", connection_out_p03], check=True)
    except subprocess.SubprocessError as e:
        logging.error("mavproxy error: %s", e)

def run_d4xx():
    while True:
        if is_device_connected("D4"):
            try:
                subprocess.run(["python3", "d4xx_to_mavlink.py", f"--connect={connection_out_p02}"], check=True)
            except subprocess.SubprocessError as e:
                logging.error("D4XX script error: %s", e)
                break
        else:
            logging.warning("No D4XX device detected. Skipping D4XX script.")
            break
        time.sleep(1)

def run_t265():
    while True:
        if is_device_connected("T265"):
            try:
                subprocess.run(["python3", "t265_precland_apriltags.py", f"--connect={connection_out_p01}"], check=True)
            except subprocess.SubprocessError as e:
                logging.error("T265 script error: %s", e)
                break
        else:
            logging.warning("T265 device not detected. Skipping T265 script.")
            break
        time.sleep(1)

def ensure_log_file():
    """Ensure the log file exists so log tailing does not crash."""
    if not log_file_path:
        return False

    log_path = Path(log_file_path)
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.touch(exist_ok=True)
    except OSError as exc:
        logging.error("Failed to create log file '%s': %s", log_path, exc)
        return False

    return True


def publish_logs():
    if not log_file_path:
        logging.info("Skipping log publishing because no log file path is available.")
        return

    log_path = Path(log_file_path)

    while True:
        if not ensure_log_file():
            time.sleep(1)
            continue

        with open(log_path, "a+") as logfile:
            logfile.seek(0, os.SEEK_END)
            while True:
                line = logfile.readline()
                if line:
                    print(line.strip())
                else:
                    if not log_path.exists():
                        break
                    time.sleep(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ONICS - Optical Navigation and Interference Control System")
    parser.add_argument("-e", "--enumerate", action="store_true", help="Enumerate detected RealSense devices")
    parser.add_argument("-d", "--disable-t265", action="store_true", help="Disable the T265 thread")
    parser.add_argument("-t", "--disable-d4xx", action="store_true", help="Disable the D4XX thread")
    parser.add_argument("-s", "--sik-only", action="store_true", help="Initialize only the SiK radio (disables both T265 and D4XX)")

    args = parser.parse_args()

    if args.enumerate:
        enumerate_devices()
    else:
        t265_enabled = not (args.disable_t265 or args.sik_only)
        d4xx_enabled = not (args.disable_d4xx or args.sik_only)

        log_thread = threading.Thread(target=publish_logs, daemon=True)
        log_thread.start()

        print("ONICS - Optical Navigation and Interference Control System is initializing...")

        threads = [threading.Thread(target=mavproxy_create_connection)]
        if t265_enabled:
            threads.append(threading.Thread(target=run_t265))
        if d4xx_enabled:
            threads.append(threading.Thread(target=run_d4xx))

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()  # Wait for threads to complete or terminate
