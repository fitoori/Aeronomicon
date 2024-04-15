#!/usr/bin/env python3

##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##################################################################
import subprocess
import threading
import logging
import time
import os

# Define log file path (change this path as needed)
log_file_path = "/home/pi/onics.log"

# Configure logging
logging.basicConfig(filename=log_file_path, level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Define variables
connection_in_port = "127.0.0.1:14550"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14540"  # T265
connection_out_p02 = "127.0.0.1:14560"  # D435i
connection_out_p03 = "/dev/ttyUSB0"     # SiK Radio

t265_enabled = True
d4xx_enabled = True

# Function to check if a RealSense device is connected
def is_device_connected(device_prefix):
    try:
        result = subprocess.run(["rs-enumerate-devices"], capture_output=True, text=True)
        lines = result.stdout.splitlines()
        for line in lines:
            if line.startswith(device_prefix):
                return True
        return False
    except subprocess.CalledProcessError as e:
        logging.error("Error checking for RealSense devices: %s", e)
        return False

# Function to run mavproxy
def mavproxy_create_connection():
    try:
        subprocess.run(["mavproxy.py",
                        "--master=" + connection_in_port,
                        "--baudrate=" + connection_in_baud,
                        "--out", "udp:" + connection_out_p01,
                        "--out", "udp:" + connection_out_p02,
                        "--out", connection_out_p03], check=True, cwd=os.path.dirname(os.path.abspath(__file__)))
    except subprocess.CalledProcessError as e:
        logging.error("mavproxy error: %s", e)

# Function to run a specific script
def run_script(script_name, args):
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), script_name)
    if os.path.exists(script_path):
        try:
            subprocess.run(["python3", script_name] + args, check=True, cwd=os.path.dirname(os.path.abspath(__file__)))
        except subprocess.CalledProcessError as e:
            logging.error(f"{script_name} script error: {e}")
    else:
        logging.warning(f"{script_name} script not found.")

# Function to run RealSense D4XX scripts
def run_d4xx():
    global d4xx_enabled
    while d4xx_enabled:
        if is_device_connected("D4"):
            try:
                subprocess.run(["python3", "d4xx_to_mavlink.py", "--connect=" + connection_out_p02], check=True, cwd=os.path.dirname(os.path.abspath(__file__)))
            except subprocess.CalledProcessError as e:
                logging.error("D4XX script error: %s", e)
        else:
            logging.warning("No D4XX device detected. Skipping D4XX script.")
            d4xx_enabled = False
            break

# Function to run T265 script
def run_t265():
    global t265_enabled
    while t265_enabled:
        if is_device_connected("T265"):
            try:
                subprocess.run(["python3", "t265_precland_apriltags.py", "--connect=" + connection_out_p01], check=True, cwd=os.path.dirname(os.path.abspath(__file__)))
            except subprocess.CalledProcessError as e:
                logging.error("T265 script error: %s", e)
        else:
            logging.warning("T265 device not detected. Skipping T265 script.")
            t265_enabled = False
            break

# Function to continuously publish logs to shell
def publish_logs():
    with open(log_file_path, 'r') as logfile:
        logfile.seek(0, os.SEEK_END)  # Move file pointer to the end of the file
        while True:
            line = logfile.readline()
            if line:
                print(line.strip())  # Print the log line to the console
            else:
                time.sleep(1)  # Sleep briefly if no new log lines are available

# Start log publishing thread
log_thread = threading.Thread(target=publish_logs)
log_thread.start()

# Output initializing message to console
print("ONICS - Optical Navigation and Interference Control System is initializing...")

# Start threads for running scripts
thread1 = threading.Thread(target=mavproxy_create_connection)
thread1.start()

thread2 = threading.Thread(target=run_t265)
thread2.start()

thread3 = threading.Thread(target=run_d4xx)
thread3.start()

# Restart threads if they exit
while True:
    threads = [(thread1, mavproxy_create_connection), (thread2, run_t265), (thread3, run_d4xx)]
    for thread, func in threads:
        if not thread.is_alive():
            thread = threading.Thread(target=func)
            thread.start()
    time.sleep(1)
