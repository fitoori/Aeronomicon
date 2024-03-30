#!/usr/bin/env python3

######################################################
##          librealsense D435i to MAVLink           ##
##             **now with IMU support**             ##  
######################################################

# WARNING: Enabling RTSP may cause unresponsiveness. 
# It is inadvisable to try and stream video from the device while it is acting as an obstacle detector.

import sys
import os
import signal
import time
import argparse
import threading
import socket
import numpy as np
import pyrealsense2 as rs
from pymavlink import mavutil
from apscheduler.schedulers.background import BackgroundScheduler
from gi.repository import Gst, GstRtspServer, GLib
import cv2

# Error-checking for system path
sys_path = "/usr/local/lib/"
if sys_path not in sys.path:
    sys.path.append(sys_path)

# Set MAVLink protocol to 2
os.environ["MAVLINK20"] = "1"

# Constants
STREAM_TYPE = [rs.stream.depth, rs.stream.color]
FORMAT = [rs.format.z16, rs.format.bgr8]
DEPTH_WIDTH, DEPTH_HEIGHT = 640, 480
COLOR_WIDTH, COLOR_HEIGHT = 640, 480
FPS = 30
DEPTH_RANGE_M = [0.1, 8.0]

# Other parameters
obstacle_line_height_ratio = 0.18
obstacle_line_thickness_pixel = 10
USE_PRESET_FILE = True
PRESET_FILE = "../cfg/d4xx-default.json"
RTSP_STREAMING_ENABLE = True
RTSP_PORT = "8554"
RTSP_MOUNT_POINT = "/d4xx"
IMU_STREAMING_ENABLE = True
filters = [
    [True,  "Decimation Filter",   rs.decimation_filter()],
    [True,  "Threshold Filter",    rs.threshold_filter()],
    [True,  "Depth to Disparity",  rs.disparity_transform(True)],
    [True,  "Spatial Filter",      rs.spatial_filter()],
    [True,  "Temporal Filter",     rs.temporal_filter()],
    [False, "Hole Filling Filter", rs.hole_filling_filter()],
    [True,  "Disparity to Depth",  rs.disparity_transform(False)]
]
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
camera_facing_angle_degree = 0
device_id = None
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0
lock = threading.Lock()
mavlink_thread_should_exit = False
exit_code = 1

# Global variables
pipe = None
depth_scale = 0
colorizer = rs.colorizer()
depth_hfov_deg = None
depth_vfov_deg = None
display_name = 'Input/output depth'
rtsp_streaming_img = None
vehicle_pitch_rad = None
data = None
current_time_us = 0
last_obstacle_distance_sent_ms = 0
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)
distances_array_length = 72
angle_offset = None
increment_f = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

# Parsing user inputs
parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float, help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--obstacle_distance_msg_hz', type=float, help="Update frequency for OBSTACLE_DISTANCE message. If not specified, a default value will be used.")
parser.add_argument('--imu_streaming_enable', type=bool, help="Enable IMU data streaming. Default is True.")
args = parser.parse_args()
connection_string = args.connect
connection_baudrate = args.baudrate
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz
imu_streaming_enable = args.imu_streaming_enable

# Signal handler functions
def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
signal.signal(signal.SIGINT, sigint_handler)

def sigterm_handler(sig, frame):
    global main_loop_should_quit, exit_code
    main_loop_should_quit = True
    exit_code = 0
signal.signal(signal.SIGTERM, sigterm_handler)

# MAVLink loop
def mavlink_loop(conn, callbacks):
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)

# Functions for MAVLink messages
def send_obstacle_distance_message():
    global current_time_us, distances, camera_facing_angle_degree, last_obstacle_distance_sent_ms
    if current_time_us == last_obstacle_distance_sent_ms:
        return
    last_obstacle_distance_sent_ms = current_time_us
    if angle_offset is None or increment_f is None:
        return
    conn.mav.obstacle_distance_send(current_time_us, 0, distances, 0, min_depth_cm, max_depth_cm, increment_f, angle_offset, 12)

def send_distance_sensor_message(distance_cm):
    global current_time_us, vehicle_pitch_rad
    if vehicle_pitch_rad is None:
        return
    pitch = int(vehicle_pitch_rad * 1000)
    conn.mav.distance_sensor_send(0, 0, 10, distance_cm, 0, 0, pitch, 0)

# Callback functions for MAVLink messages
def attitude_callback(m):
    global vehicle_pitch_rad
    vehicle_pitch_rad = m.pitch

def global_position_int_callback(m):
    pass

def heartbeat_callback(m):
    pass

# Set up MAVLink connection
if connection_string is None:
    connection_string = connection_string_default
if connection_baudrate is None:
    connection_baudrate = connection_baudrate_default
conn = mavutil.mavlink_connection(connection_string, baud=connection_baudrate)
callbacks = {
    'ATTITUDE': attitude_callback,
    'GLOBAL_POSITION_INT': global_position_int_callback,
    'HEARTBEAT': heartbeat_callback
}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, callbacks), daemon=True)
mavlink_thread.start()

# Set up RealSense pipeline
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
cfg.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
profile = pipe.start(cfg)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
pipeline_profile = pipe.get_active_profile()
depth_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

# Set up RealSense filters
for i in range(len(filters)):
    if filters[i][0]:
        pipe = pipe.apply_filter(filters[i][2])

# Create and start RTSP streaming server
if RTSP_STREAMING_ENABLE:
    Gst.init(None)
    rtsp_server = GstRtspServer.RTSPServer()
    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch('( v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96 )')
    factory.set_shared(True)
    rtsp_server.get_mount_points().add_factory(RTSP_MOUNT_POINT, factory)
    rtsp_server.attach(None)
    main_loop = GLib.MainLoop()
    threading.Thread(target=main_loop.run, daemon=True).start()

# IMU streaming thread
def imu_streaming_thread():
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 63)
    pipeline.start(config)
    try:
        while IMU_STREAMING_ENABLE:
            frames = pipeline.wait_for_frames()
            accel_frame = frames.get_accel_frame()
            gyro_frame = frames.get_gyro_frame()
            if accel_frame and gyro_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                msg = master.mav.message_factory.raw_imu_encode(
                    0,
                    accel_data.time_stamp,
                    accel_data.x,
                    accel_data.y,
                    accel_data.z,
                    gyro_data.x,
                    gyro_data.y,
                    gyro_data.z,
                    0, 0, 0
                )
                master.mav.send(msg)
    except Exception as e:
        print("IMU streaming thread exception:", str(e))
    finally:
        pipeline.stop()

if IMU_STREAMING_ENABLE:
    imu_thread = threading.Thread(target=imu_streaming_thread, daemon=True)
    imu_thread.start()

try:
    # Main loop
    while not main_loop_should_quit:
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        obstacle_line_height_pixel = int(DEPTH_HEIGHT * obstacle_line_height_ratio)
        cv2.line(depth_image, (0, obstacle_line_height_pixel), (DEPTH_WIDTH, obstacle_line_height_pixel), (0, 0, 255), obstacle_line_thickness_pixel)
        global distances, current_time_us
        distances = depth_image[obstacle_line_height_pixel]
        current_time_us = int(time.time() * 1e6)
        if RTSP_STREAMING_ENABLE:
            rtsp_streaming_img = cv2.resize(color_image, (640, 480))
        if enable_msg_obstacle_distance:
            send_obstacle_distance_message()
        if enable_msg_distance_sensor:
            send_distance_sensor_message(100)
except Exception as e:
    print("Exception:", str(e))
finally:
    if pipe:
        pipe.stop()
    if RTSP_STREAMING_ENABLE:
        rtsp_server.detach(None)
    os._exit(exit_code)
