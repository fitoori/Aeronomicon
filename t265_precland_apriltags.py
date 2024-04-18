import sys
import os
import argparse
import time
import cv2
import numpy as np
import transformations as tf
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from multiprocessing import Process, Pipe

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

def realsense_process(conn):
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        cfg.enable_stream(rs.stream.fisheye, 1)
        cfg.enable_stream(rs.stream.fisheye, 2)
        pipe.start(cfg)

        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                data = pose.get_pose_data()
                # Send pose data through pipe
                conn.send(data)
    except Exception as e:
        print(f'Error in RealSense process: {e}')
    finally:
        conn.close()

def send_vision_position_message(conn):
    global current_time
    try:
        while True:
            if current_time:
                # Receive data from RealSense process
                if conn.poll():
                    pose_data = conn.recv()
                    # Process pose data and send MAVLink message
                    # msg = vehicle.message_factory.vision_position_estimate_encode(...)
                    # vehicle.send_mavlink(msg)
                    # vehicle.flush()
                time.sleep(1 / vision_msg_hz)
    except Exception as e:
        print(f'Error sending vision position message: {e}')

def send_confidence_level_dummy_message():
    global current_time
    try:
        while True:
            if current_time:
                # Send dummy confidence level message
                # msg = vehicle.message_factory.vision_position_delta_encode(...)
                # vehicle.send_mavlink(msg)
                # vehicle.flush()
                time.sleep(1 / confidence_msg_hz)
    except Exception as e:
        print(f'Error sending confidence level message: {e}')

def send_land_target_message():
    global current_time
    try:
        while True:
            if current_time:
                # Send landing target message
                # msg = vehicle.message_factory.landing_target_encode(...)
                # vehicle.send_mavlink(msg)
                # vehicle.flush()
                time.sleep(1 / landing_target_msg_hz)
    except Exception as e:
        print(f'Error sending landing target message: {e}')

def main():
    global current_time
    try:
        # Initialize connections
        connect_vehicle()

        # Set up pipes for inter-process communication
        rs_conn, vision_conn = Pipe()

        # Start RealSense process
        rs_process = Process(target=realsense_process, args=(rs_conn,))
        rs_process.daemon = True
        rs_process.start()

        # Start vision message sender process
        vision_process = Process(target=send_vision_position_message, args=(vision_conn,))
        vision_process.daemon = True
        vision_process.start()

        # Start confidence message sender process
        confidence_process = Process(target=send_confidence_level_dummy_message)
        confidence_process.daemon = True
        confidence_process.start()

        # Start landing target message sender process
        landing_target_process = Process(target=send_land_target_message)
        landing_target_process.daemon = True
        landing_target_process.start()

        # Main loop
        while True:
            current_time = int(round(time.time() * 1000000))
            time.sleep(1)  # Placeholder for actual processing

    except KeyboardInterrupt:
        print("INFO: KeyboardInterrupt caught. Cleaning up...")
    finally:
        if vehicle:
            vehicle.close()
        print("INFO: Cleanup complete.")

if __name__ == '__main__':
    main()
