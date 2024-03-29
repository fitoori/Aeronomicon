#!/usr/bin/env python3

##################################################################
##  ONICS - Optical Navigation and Interference Control System  ##
##################################################################

import subprocess
import threading
import logging
import time

# Configure logging
logging.basicConfig(filename='/home/pi/ONICS/onics.log', level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# Define variables
connection_in_port = "127.0.0.1:14550"
connection_in_baud = "921600"
connection_out_p01 = "127.0.0.1:14540"  # T265
connection_out_p02 = "127.0.0.1:14560"  # D435i
connection_out_p03 = "/dev/ttyUSB0"     # SiK Radio

t265_enabled = True
d435i_enabled = True

# Function to check if a RealSense device is connected
def is_device_connected(device_name):
    try:
        result = subprocess.run(["rs-enumerate-devices"], capture_output=True, text=True)
        return device_name in result.stdout
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
                        "--out", connection_out_p03], check=True, cwd="/home/pi/ONICS/")
    except subprocess.CalledProcessError as e:
        logging.error("mavproxy error: %s", e)

# Function to run T265 script
def run_t265():
    global t265_enabled
    while t265_enabled:
        if is_device_connected("T265"):
            try:
                subprocess.run(["python3", "t265_precland_apriltags.py", "--connect=" + connection_out_p01], check=True, cwd="/home/pi/ONICS/")
            except subprocess.CalledProcessError as e:
                logging.error("T265 script error: %s", e)
        else:
            logging.warning("T265 device not detected. Skipping T265 script.")
            t265_enabled = False
            break

# Function to run D435i script
def run_d435i():
    global d435i_enabled
    while d435i_enabled:
        if is_device_connected("D435i"):
            try:
                subprocess.run(["python3", "d4xx_to_mavlink.py", "--connect=" + connection_out_p02], check=True, cwd="/home/pi/ONICS/")
            except subprocess.CalledProcessError as e:
                logging.error("D435i script error: %s", e)
        else:
            logging.warning("D435i device not detected. Skipping D435i script.")
            d435i_enabled = False
            break

# Function to continuously publish logs to shell
def publish_logs():
    with open('/home/pi/ONICS/onics.log', 'r') as logfile:
        while True:
            line = logfile.readline()
            if line:
                print(line.strip())
            else:
                time.sleep(1)

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

thread3 = threading.Thread(target=run_d435i)
thread3.start()

# Restart threads if they exit
while True:
    if not thread1.is_alive():
        thread1 = threading.Thread(target=mavproxy_create_connection)
        thread1.start()
    if not thread2.is_alive() and t265_enabled:
        thread2 = threading.Thread(target=run_t265)
        thread2.start()
    if not thread3.is_alive() and d435i_enabled:
        thread3 = threading.Thread(target=run_d435i)
        thread3.start()
    time.sleep(1)
