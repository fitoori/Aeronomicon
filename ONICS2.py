#!/usr/bin/env python3

##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##################################################################

import subprocess
import multiprocessing
import logging
import time
import os
import argparse
import signal

# Default log file path
log_file_path = "/home/pi/onics.log"

# Configure logging
logging.basicConfig(filename=log_file_path, level=logging.INFO,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

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
        detected_devices = [line.strip() for line in result.stdout.splitlines()
                            if any(dev in line for dev in relevant_devices)]
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
    process = None
    while True:
        if is_device_connected("D4"):
            try:
                if process is None or process.poll() is not None:
                    logging.info("Starting D4XX script")
                    process = subprocess.Popen(["python3", "d4xx_to_mavlink.py", f"--connect={connection_out_p02}"])
                time.sleep(5)
            except Exception as e:
                logging.error("D4XX script error: %s", e)
                if process:
                    process.terminate()
                process = None
        else:
            logging.warning("No D4XX device detected. Waiting...")
            if process:
                process.terminate()
                process = None
            time.sleep(5)


def run_t265():
    process = None
    while True:
        if is_device_connected("T265"):
            try:
                if process is None or process.poll() is not None:
                    logging.info("Starting T265 script")
                    process = subprocess.Popen(["python3", "t265_precland_apriltags.py", f"--connect={connection_out_p01}"])
                time.sleep(5)
            except Exception as e:
                logging.error("T265 script error: %s", e)
                if process:
                    process.terminate()
                process = None
        else:
            logging.warning("T265 device not detected. Waiting...")
            if process:
                process.terminate()
                process = None
            time.sleep(5)


def publish_logs():
    with open(log_file_path, 'r') as logfile:
        logfile.seek(0, os.SEEK_END)
        while True:
            line = logfile.readline()
            if line:
                print(line.strip())
            else:
                time.sleep(1)


def terminate_processes(processes):
    for process in processes:
        if process.is_alive():
            process.terminate()
            process.join()


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

        mavproxy_process = multiprocessing.Process(target=mavproxy_create_connection)
        processes.append(mavproxy_process)

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
            log_process.terminate()
            log_process.join()
            print("All processes terminated.")
