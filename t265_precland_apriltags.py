#!/usr/bin/env python3

#####################################################
##   Precision Landing with T265 using AprilTags   ##
#####################################################

# Set the path for pyrealsense2.[].so
import sys
sys.path.append("/usr/local/lib/")

# Remove the path to python2 version that is added by ROS, see here https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs
import cv2
import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading

from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil


try:
    import apriltags3 
except ImportError:
    raise ImportError('Please install the Python wrapper for apriltag3 (apriltags3.py).')

# Default configurations
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
vision_msg_hz_default = 20
landing_target_msg_hz_default = 20
confidence_msg_hz_default = 1
camera_orientation_default = 1

# Argument parser
parser = argparse.ArgumentParser(description='ArduPilot + Realsense T265 + AprilTags')
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--baudrate', type=float, help="Vehicle connection baudrate.")
parser.add_argument('--vision_msg_hz', type=float, help="Update frequency for VISION_POSITION_ESTIMATE message.")
parser.add_argument('--landing_target_msg_hz', type=float, help="Update frequency for LANDING_TARGET message.")
parser.add_argument('--confidence_msg_hz', type=float, help="Update frequency for confidence level.")
parser.add_argument('--scale_calib_enable', type=bool, help="Scale calibration. Only run while NOT in flight.")
parser.add_argument('--camera_orientation', type=int, help="Configuration for camera orientation.")
parser.add_argument('--visualization', type=int, help="Enable visualization.")
parser.add_argument('--debug_enable', type=int, help="Enable debug messages on terminal.")
args = parser.parse_args()

# Assigning arguments or defaults
connection_string = args.connect or connection_string_default
connection_baudrate = args.baudrate or connection_baudrate_default
vision_msg_hz = args.vision_msg_hz or vision_msg_hz_default
landing_target_msg_hz = args.landing_target_msg_hz or landing_target_msg_hz_default
confidence_msg_hz = args.confidence_msg_hz or confidence_msg_hz_default
scale_calib_enable = args.scale_calib_enable or False
camera_orientation = args.camera_orientation or camera_orientation_default
visualization = args.visualization or 0
debug_enable = args.debug_enable or 0

# MAVLink environment variable
os.environ["MAVLINK20"] = "1"

# Global variables
vehicle = None
pipe = None
current_time = 0

# Default global position
home_lat = 151269321  # Somewhere in Africa
home_lon = 16624301   # Somewhere in Africa
home_alt = 163000

# Tracking confidence levels
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

def connect_vehicle():
    global vehicle
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=connection_baudrate, source_system=1)
    except Exception as e:
        print(f'Error connecting to vehicle: {e}')
        sys.exit(1)

def realsense_connect():
    global pipe
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        cfg.enable_stream(rs.stream.fisheye, 1)
        cfg.enable_stream(rs.stream.fisheye, 2)
        pipe.start(cfg)
    except Exception as e:
        print(f'Error connecting to RealSense: {e}')
        sys.exit(1)

def setup_scheduler():
    global vehicle
    try:
        sched = BackgroundScheduler()
        sched.add_job(send_vision_position_message, 'interval', seconds=1/vision_msg_hz)
        sched.add_job(send_confidence_level_dummy_message, 'interval', seconds=1/confidence_msg_hz)
        sched.add_job(send_land_target_message, 'interval', seconds=1/landing_target_msg_hz_default)
        sched.start()
    except Exception as e:
        print(f'Scheduler setup failed: {e}')
        if vehicle:
            vehicle.close()
        sys.exit(1)

def send_vision_position_message():
    global vehicle, current_time
    try:
        if vehicle and current_time:
            # Send vision position message
            msg = vehicle.message_factory.vision_position_estimate_encode(current_time, 0, 0, 0, 0, 0, 0)
            vehicle.send_mavlink(msg)
            vehicle.flush()
    except Exception as e:
        print(f'Error sending vision position message: {e}')

def send_confidence_level_dummy_message():
    global vehicle, current_time
    try:
        if vehicle and current_time:
            # Send confidence level message
            msg = vehicle.message_factory.vision_position_delta_encode(0, 0, [0, 0, 0], [0, 0, 0], 0)
            vehicle.send_mavlink(msg)
            vehicle.flush()
    except Exception as e:
        print(f'Error sending confidence level message: {e}')

def send_land_target_message():
    global vehicle, current_time
    try:
        if vehicle and current_time:
            # Send landing target message
            msg = vehicle.message_factory.landing_target_encode(current_time, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0, 0, 0, 0, 0, 0, 0, (1, 0, 0, 0), 2, 1)
            vehicle.send_mavlink(msg)
            vehicle.flush()
    except Exception as e:
        print(f'Error sending landing target message: {e}')

def main():
    global current_time
    try:
        # Initialize connections
        connect_vehicle()
        realsense_connect()
        setup_scheduler()

        # Main loop
        while True:
            # Perform tasks and process data here
            current_time = int(round(time.time() * 1000000))
            time.sleep(1)  # Placeholder for actual processing
            
    except KeyboardInterrupt:
        print("INFO: KeyboardInterrupt caught. Cleaning up...")
    finally:
        if pipe:
            pipe.stop()
        if vehicle:
            vehicle.close()
        print("INFO: Cleanup complete.")

if __name__ == '__main__':
    main()


