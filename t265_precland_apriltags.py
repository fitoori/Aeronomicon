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
import cv2
import numpy as np
import transformations as tf
import time
import argparse
import threading
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

try:
    import apriltags3
except ImportError:
    raise ImportError('Please download the Python wrapper for apriltag3 (apriltags3.py) and place it in the same directory as this script or add the directory path to the PYTHONPATH environment variable.')

# Default configurations
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
vision_msg_hz_default = 20
landing_target_msg_hz_default = 20
confidence_msg_hz_default = 1
camera_orientation_default = 1

# Global variables
vehicle = None
pipe = None
current_time = 0
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')
WINDOW_TITLE = 'Apriltag detection from T265 images'

def set_environment_paths():
    sys.path.append("/usr/local/lib/")
    if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

    os.environ["MAVLINK20"] = "1"

def parse_arguments():
    parser = argparse.ArgumentParser(description='ArduPilot + Realsense T265 + AprilTags')
    parser.add_argument('--connect', help="Vehicle connection target string.")
    parser.add_argument('--baudrate', type=int, help="Vehicle connection baudrate.")
    parser.add_argument('--vision_msg_hz', type=float, help="VISION_POSITION_ESTIMATE message update frequency.")
    parser.add_argument('--landing_target_msg_hz', type=float, help="LANDING_TARGET message update frequency.")
    parser.add_argument('--confidence_msg_hz', type=float, help="Confidence level update frequency.")
    parser.add_argument('--scale_calib_enable', action='store_true', help="Enable scale calibration.")
    parser.add_argument('--camera_orientation', type=int, help="Camera orientation configuration.")
    parser.add_argument('--visualization', action='store_true', help="Enable visualization.")
    parser.add_argument('--debug_enable', action='store_true', help="Enable debug messages.")
    return parser.parse_args()

def connect_vehicle(connection_string, baudrate):
    global vehicle
    try:
        vehicle = connect(connection_string, wait_ready=True, baud=baudrate, source_system=1)
        return True
    except Exception as e:
        print(f'ERROR: Failed to connect to vehicle: {e}')
        return False

def connect_realsense():
    global pipe
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    cfg.enable_stream(rs.stream.fisheye, 1)
    cfg.enable_stream(rs.stream.fisheye, 2)
    pipe.start(cfg)

def send_land_target_message():
    global current_time
    if is_landing_tag_detected:
        x, y, z = H_camera_tag[0][3], H_camera_tag[1][3], H_camera_tag[2][3]
        x_offset_rad = np.arctan(x / z)
        y_offset_rad = np.arctan(y / z)
        distance = np.sqrt(x * x + y * y + z * z)
        msg = vehicle.message_factory.landing_target_encode(
            current_time, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, x_offset_rad, y_offset_rad,
            distance, 0, 0, 0, 0, 0, (1,0,0,0), 2, 1)
        vehicle.send_mavlink(msg)
        vehicle.flush()

def send_vision_position_message():
    global current_time
    if H_aeroRef_aeroBody is not None:
        rpy_rad = np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))
        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time, H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3],
            rpy_rad[0], rpy_rad[1], rpy_rad[2])
        vehicle.send_mavlink(msg)
        vehicle.flush()

def send_confidence_level_dummy_message():
    global current_confidence
    if data is not None:
        confidence_percentage = data.tracker_confidence * 100 / 3
        print(f"INFO: Tracking confidence: {pose_data_confidence_level[data.tracker_confidence]}")
        msg = vehicle.message_factory.vision_position_delta_encode(
            0, 0, [0, 0, 0], [0, 0, 0], confidence_percentage)
        vehicle.send_mavlink(msg)
        vehicle.flush()
        if current_confidence is None or current_confidence != data.tracker_confidence:
            current_confidence = data.tracker_confidence
            status_msg = vehicle.message_factory.statustext_encode(3, f"Tracking confidence: {pose_data_confidence_level[data.tracker_confidence]}")
            vehicle.send_mavlink(status_msg)
            vehicle.flush()

def set_default_global_origin():
    msg = vehicle.message_factory.set_gps_global_origin_encode(int(vehicle._master.source_system), home_lat, home_lon, home_alt)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_default_home_position():
    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system), home_lat, home_lon, home_alt, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def update_timesync(ts=0, tc=0):
    ts = int(round(time.time() * 1000))
    msg = vehicle.message_factory.timesync_encode(tc, ts)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        print(f"INFO: Received first ATTITUDE message with heading yaw {heading_north_yaw * 180 / np.pi} degrees")
    else:
        heading_north_yaw = value.yaw
        print(f"INFO: Received ATTITUDE message with heading yaw {heading_north_yaw * 180 / np.pi} degrees")

def scale_update():
    global scale_factor
    while True:
        try:
            scale_factor = float(input("INFO: Type in the calibration square size (meters): "))
            print(f"INFO: Updated calibration scale factor to {scale_factor} m")
            break
        except ValueError:
            print("ERROR: Invalid input. Please enter a valid floating-point number.")

def main():
    set_environment_paths()
    args = parse_arguments()

    connection_string = args.connect if args.connect else connection_string_default
    baudrate = args.baudrate if args.baudrate else connection_baudrate_default
    vision_msg_hz = args.vision_msg_hz if args.vision_msg_hz else vision_msg_hz_default
    landing_target_msg_hz = args.landing_target_msg_hz if args.landing_target_msg_hz else landing_target_msg_hz_default
    confidence_msg_hz = args.confidence_msg_hz if args.confidence_msg_hz else confidence_msg_hz_default
    camera_orientation = args.camera_orientation if args.camera_orientation else camera_orientation_default

    if not connect_vehicle(connection_string, baudrate):
        return

    connect_realsense()

    # Create scheduler
    scheduler = BackgroundScheduler()
    scheduler.start()

    # Schedule message sending functions
    scheduler.add_job(send_vision_position_message, 'interval', seconds=1 / vision_msg_hz, id='vision_position_message')
    scheduler.add_job(send_land_target_message, 'interval', seconds=1 / landing_target_msg_hz, id='land_target_message')
    scheduler.add_job(send_confidence_level_dummy_message, 'interval', seconds=1 / confidence_msg_hz, id='confidence_message')

    # Handle user input for scale calibration
    if args.scale_calib_enable:
        scale_thread = threading.Thread(target=scale_update)
        scale_thread.start()

    try:
        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            if pose:
                pose_data = pose.get_pose_data()
                pose_transform = np.array(pose_data.rotation).reshape(3, 3)
                pose_translation = np.array(pose_data.translation)

                if camera_orientation == 1:
                    H_camera_aeroRef = tf.quaternion_matrix([pose_transform[1, 1], -pose_transform[0, 1], -pose_transform[2, 1], pose_transform[3, 1]])
                    H_aeroRef_camera = tf.inverse_matrix(H_camera_aeroRef)
                else:
                    H_camera_aeroRef = tf.quaternion_matrix([pose_transform[1, 1], -pose_transform[0, 1], -pose_transform[2, 1], pose_transform[3, 1]])
                    H_aeroRef_camera = H_camera_aeroRef

                H_aeroRef_aeroBody = tf.euler_matrix(0, 0, heading_north_yaw)
                H_camera_aeroBody = np.dot(H_aeroRef_aeroBody, H_aeroRef_camera)

            # AprilTag detection and landing logic
            if args.visualization:
                tags = detector.detect(fisheye_frame)
                if tags:
                    print(f'Detected tags: {tags}')
                    # Additional logic for handling detected tags

    except KeyboardInterrupt:
        print("INFO: Keyboard interrupt detected. Exiting...")

    finally:
        # Clean up
        if vehicle:
            vehicle.close()
        if pipe:
            pipe.stop()
        scheduler.shutdown()

if __name__ == '__main__':
    main()

