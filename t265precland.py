#!/usr/bin/env python3

#####################################################
##   Precision Landing with T265 using AprilTags   ##
#####################################################
# Install required packages:
#   pip3 install transformations
#   pip3 install dronekit
#   pip3 install apscheduler

import sys
import os
import argparse
import threading
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import transformations as tf
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import apriltags3 

# Remove ROS path if present
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 

# Set MAVLink protocol to 2
os.environ["MAVLINK20"] = "1"

# Global variables
vehicle = None
pipe = None
data = None
current_time = 0
H_aeroRef_aeroBody = None
is_landing_tag_detected = False

# Function to connect to the vehicle
def vehicle_connect(connection_string, connection_baudrate):
    global vehicle
    while True:
        try:
            vehicle = connect(connection_string, wait_ready=True, baud=connection_baudrate, source_system=1)
            print("INFO: Vehicle connected.")
            return True
        except Exception as e:
            print(f"ERROR: Failed to connect to vehicle: {e}")
            time.sleep(1)

# Function to connect to the Realsense camera
def realsense_connect():
    global pipe
    try:
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        cfg.enable_stream(rs.stream.fisheye, 1)
        cfg.enable_stream(rs.stream.fisheye, 2)
        pipe.start(cfg)
        print("INFO: Realsense connected.")
    except Exception as e:
        print(f"ERROR: Failed to connect to Realsense: {e}")

# Function to send MAVLink messages in the background
def send_mavlink_messages():
    global current_time, H_aeroRef_aeroBody
    while True:
        if H_aeroRef_aeroBody is not None:
            # Send MAVLink messages here
            pass
        time.sleep(1)  # Adjust interval as needed

# Function to process Realsense frames
def process_realsense_frames():
    global data, current_time, H_aeroRef_aeroBody, is_landing_tag_detected
    while True:
        frames = pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
            current_time = int(time.time() * 1000000)
            data = pose.get_pose_data()
            # Process pose data and update H_aeroRef_aeroBody
            pass
        # Process image streams for AprilTag detection and other tasks
        pass

# Main function
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ArduPilot + Realsense T265 + AprilTags')
    parser.add_argument('--connect', help="Vehicle connection target string.", default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=float, help="Vehicle connection baudrate.", default=921600)
    args = parser.parse_args()

    # Connect to vehicle and Realsense camera in separate threads
    vehicle_thread = threading.Thread(target=vehicle_connect, args=(args.connect, args.baudrate), daemon=True)
    vehicle_thread.start()
    realsense_thread = threading.Thread(target=realsense_connect, daemon=True)
    realsense_thread.start()

    # Start threads for sending MAVLink messages and processing Realsense frames
    mavlink_thread = threading.Thread(target=send_mavlink_messages, daemon=True)
    mavlink_thread.start()
    realsense_process_thread = threading.Thread(target=process_realsense_frames, daemon=True)
    realsense_process_thread.start()

    # Wait for threads to finish (which they won't unless the program is terminated)
    vehicle_thread.join()
    realsense_thread.join()
    mavlink_thread.join()
    realsense_process_thread.join()

if __name__ == "__main__":
    main()
