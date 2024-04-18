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
import time
import argparse
import threading
import cv2
import numpy as np
import transformations as tf
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

try:
    import apriltags3
except ImportError:
    raise ImportError('Please install apriltags3: pip install apriltags3')

# Set MAVLink protocol to version 2
os.environ["MAVLINK20"] = "1"

# Default configurations
DEFAULT_CONNECTION_STRING = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 921600
DEFAULT_VISION_MSG_HZ = 20
DEFAULT_LANDING_TARGET_MSG_HZ = 20
DEFAULT_CONFIDENCE_MSG_HZ = 1
DEFAULT_CAMERA_ORIENTATION = 1

# Parse command-line arguments
parser = argparse.ArgumentParser(description='ArduPilot + Realsense T265 + AprilTags')
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--baudrate', type=int, help="Vehicle connection baudrate.")
parser.add_argument('--vision_msg_hz', type=float, help="Update frequency for VISION_POSITION_ESTIMATE message.")
parser.add_argument('--landing_target_msg_hz', type=float, help="Update frequency for LANDING_TARGET message.")
parser.add_argument('--confidence_msg_hz', type=float, help="Update frequency for confidence level.")
parser.add_argument('--camera_orientation', type=int, help="Camera orientation: 0 or 1.")
parser.add_argument('--visualization', action='store_true', help="Enable visualization.")

args = parser.parse_args()

# Use command-line arguments or defaults
connection_string = args.connect or DEFAULT_CONNECTION_STRING
baudrate = args.baudrate or DEFAULT_BAUDRATE
vision_msg_hz = args.vision_msg_hz or DEFAULT_VISION_MSG_HZ
landing_target_msg_hz = args.landing_target_msg_hz or DEFAULT_LANDING_TARGET_MSG_HZ
confidence_msg_hz = args.confidence_msg_hz or DEFAULT_CONFIDENCE_MSG_HZ
camera_orientation = args.camera_orientation or DEFAULT_CAMERA_ORIENTATION
visualization = args.visualization

# Connect to the vehicle
def connect_to_vehicle():
    try:
        return connect(connection_string, wait_ready=True, baud=baudrate, source_system=1)
    except Exception as e:
        print(f"ERROR: Failed to connect to vehicle: {e}")
        return None

vehicle = connect_to_vehicle()
if not vehicle:
    sys.exit(1)  # Exit if vehicle connection failed

# Realsense connection setup
def setup_realsense():
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    cfg.enable_stream(rs.stream.fisheye, 1)
    cfg.enable_stream(rs.stream.fisheye, 2)
    pipe.start(cfg)
    return pipe

# Main functionality
def main():
    pipe = setup_realsense()
    if not pipe:
        vehicle.close()
        sys.exit(1)  # Exit if Realsense setup failed

    try:
        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                process_pose_data(data)

            # Other processing and AprilTag detection here...

            if visualization:
                display_frames(frames)

            time.sleep(0.01)  # Adjust as needed for processing rate

    except KeyboardInterrupt:
        print("INFO: Keyboard Interrupt. Exiting...")

    finally:
        pipe.stop()
        vehicle.close()
        print("INFO: Realsense pipeline and vehicle connection closed.")

def process_pose_data(data):
    # Process pose data here
    pass

def display_frames(frames):
    # Display frames for visualization
    pass

if __name__ == "__main__":
    main()

