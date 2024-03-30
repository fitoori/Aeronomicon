#!/usr/bin/env python3

######################################################
##          librealsense D4xx to MAVLink            ##
######################################################

# Set the path for pyrealsense2.[].so
# Otherwise, place the pyrealsense2.[].so file under the same directory as this script or modify PYTHONPATH


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
args = parser.parse_args()
connection_string = args.connect
connection_baudrate = args.baudrate
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz

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

try:
    # Main loop
    while not main_loop_should_quit:
        # RealSense processing
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Continue if any frame is missing
        if not depth_frame or not color_frame:
            continue

        # Convert RealSense frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Draw obstacle detection line
        obstacle_line_height_pixel = int(DEPTH_HEIGHT * obstacle_line_height_ratio)
        cv2.line(depth_image, (0, obstacle_line_height_pixel), (DEPTH_WIDTH, obstacle_line_height_pixel), (0, 0, 255), obstacle_line_thickness_pixel)

        # Update obstacle distance data
        global distances, current_time_us
        distances = depth_image[obstacle_line_height_pixel]
        current_time_us = int(time.time() * 1e6)

        # RTSP streaming
        if RTSP_STREAMING_ENABLE:
            rtsp_streaming_img = cv2.resize(color_image, (640, 480))

        # Send MAVLink messages
        if enable_msg_obstacle_distance:
            send_obstacle_distance_message()
        if enable_msg_distance_sensor:
            send_distance_sensor_message(100)  # Example distance in centimeters

except Exception as e:
    print("Exception:", str(e))
finally:
    # Clean up
    if pipe:
        pipe.stop()
    if RTSP_STREAMING_ENABLE:
        rtsp_server.detach(None)
    os._exit(exit_code)
