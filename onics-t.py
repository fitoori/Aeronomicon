#!/usr/bin/env python3
# -*- coding: utf-8 -*-
##################################################################
##  ONICS – Optical Navigation and Interference Control System  ##
##                            T265 Node                         ##
##                T265-to-EKF2 + AprilTag Landing               ##
##################################################################

import sys
import os
import signal
import time
import math as m
import threading
import argparse

import numpy as np
import cv2

import pyrealsense2.pyrealsense2 as rs
import transformations as tf

from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

try:
    import apriltags3
except ImportError:
    raise ImportError(
        "Please download the Python wrapper for apriltag3 (apriltags3.py) and "
        "put it in the same folder as this script or add the directory path to PYTHONPATH."
    )

# ------------------------------
# Defaults & configuration
# ------------------------------
connection_string_default = '127.0.0.1:14550'
connection_baudrate_default = 921600
vision_msg_hz_default = 20
landing_target_msg_hz_default = 20
confidence_msg_hz_default = 1
camera_orientation_default = 1  # 0 = forward/right-USB, 1 = down/right-USB

# In NED frame, offset from IMU/CoG to the camera origin (meters)
body_offset_enabled = 0
body_offset_x = 0.05
body_offset_y = 0.0
body_offset_z = 0.0

# Global position scaling (x,y,z)
scale_factor = 1.0

# Use compass heading to align yaw to north
compass_enabled = 0

# ------------------------------
# WATNE's Designated Emergency Landing Site (DELS)
# Field by Lawrence Park C.I.
# 43.72220° N, 79.41201° W; altitude ≈ 180 m AMSL
# MAVLink expects: lat/lon in degrees * 1e7 (int), alt in mm (int)
# ------------------------------
home_lat = 437222000     # 43.72220 * 1e7
home_lon = -794120100    # -79.41201 * 1e7
home_alt = 180000        # 180 m AMSL -> 180000 mm

# Timestamp (us, UNIX epoch)
current_time = 0

# Globals set at runtime
vehicle = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

# AprilTag detection configuration
# Target the specific tag: tag41_12_00113 => family Standard41h12, id=113
APRILTAG_FAMILY = 'tagStandard41h12'
tag_landing_id = 113
tag_landing_size = 0.144     # meters (edge length incl. border)
tag_image_source = "right"   # T265 supports "left" or "right"

# Shutdown event (set on SIGTERM/SIGINT)
shutdown_event = threading.Event()

# ------------------------------
# CLI args
# ------------------------------
parser = argparse.ArgumentParser(description='ArduPilot + RealSense T265 + AprilTags')
parser.add_argument('--connect',              help="Vehicle connection target string.")
parser.add_argument('--baudrate', type=float, help="Vehicle connection baudrate.")
parser.add_argument('--vision_msg_hz', type=float, help="VISION_POSITION_ESTIMATE frequency.")
parser.add_argument('--landing_target_msg_hz', type=float, help="LANDING_TARGET frequency.")
parser.add_argument('--confidence_msg_hz', type=float, help="Confidence report frequency.")
parser.add_argument('--scale_calib_enable', type=bool, help="Scale calibration (NOT in flight).")
parser.add_argument('--camera_orientation', type=int,
                    help="Camera orientation: 0=forward/right-USB, 1=down/right-USB")
parser.add_argument('--visualization', type=int, help="Enable visualization window (1 to enable).")
parser.add_argument('--debug_enable', type=int, help="Enable debug print (1 to enable).")
args = parser.parse_args()

# Helper: coalesce None to default but keep falsy numeric values like 0
def coalesce(val, default):
    return default if val is None else val

connection_string      = coalesce(args.connect, connection_string_default)
connection_baudrate    = coalesce(args.baudrate, connection_baudrate_default)
vision_msg_hz          = coalesce(args.vision_msg_hz, vision_msg_hz_default)
landing_target_msg_hz  = coalesce(args.landing_target_msg_hz, landing_target_msg_hz_default)
confidence_msg_hz      = coalesce(args.confidence_msg_hz, confidence_msg_hz_default)
scale_calib_enable     = bool(args.scale_calib_enable) if args.scale_calib_enable is not None else False
camera_orientation     = coalesce(args.camera_orientation, camera_orientation_default)
visualization          = 1 if (args.visualization and int(args.visualization) == 1) else 0
debug_enable           = 1 if (args.debug_enable and int(args.debug_enable) == 1) else 0

print(f"INFO: Using connection_string: {connection_string}")
print(f"INFO: Using connection_baudrate: {connection_baudrate}")
print(f"INFO: Using vision_msg_hz: {vision_msg_hz}")
print(f"INFO: Using landing_target_msg_hz: {landing_target_msg_hz}")
print(f"INFO: Using confidence_msg_hz: {confidence_msg_hz}")
print(f"INFO: Camera orientation: {camera_orientation} (0=forward/right-USB, 1=down/right-USB)")
print("INFO: Camera position offset:", "Enabled" if body_offset_enabled else "Disabled",
      f"({body_offset_x}, {body_offset_y}, {body_offset_z})")
print("INFO: Compass:", "Enabled (north-aligned yaw)" if compass_enabled else "Disabled")

if scale_calib_enable:
    print("\nINFO: SCALE CALIBRATION MODE — DO NOT RUN IN FLIGHT.\n"
          "INFO: Type a new scale (float) at any time.\n")
else:
    print(f"INFO: Scale factor: {scale_factor}")

if visualization == 1:
    WINDOW_TITLE = 'AprilTag detection from T265 images'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
    print("INFO: Visualization enabled (press 'q' to exit pop-up).")
else:
    print("INFO: Visualization disabled.")

if debug_enable == 1:
    np.set_printoptions(precision=4, suppress=True)
    print("INFO: Debug output enabled.")

# ------------------------------
# Coordinate transforms for camera orientation
# ------------------------------
if camera_orientation == 0:
    # Forward, USB to the right
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:
    # Down-facing, USB to the right
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
else:
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# ------------------------------
# AprilTag detector
# ------------------------------
at_detector = apriltags3.Detector(
    searchpath=['apriltags'],
    families=APRILTAG_FAMILY,       # <<--- Ensure family supports Standard41h12
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# ------------------------------
# MAVLink helpers
# ------------------------------
def send_land_target_message():
    global current_time, H_camera_tag, is_landing_tag_detected, vehicle
    if is_landing_tag_detected and H_camera_tag is not None and vehicle is not None:
        x = H_camera_tag[0][3]
        y = H_camera_tag[1][3]
        z = H_camera_tag[2][3]
        if z == 0:
            return
        x_offset_rad = m.atan(x / z)
        y_offset_rad = m.atan(y / z)
        distance = float(np.sqrt(x * x + y * y + z * z))

        msg = vehicle.message_factory.landing_target_encode(
            current_time,                       # time target processed (usec)
            0,                                  # target num
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            float(x_offset_rad),                # x angular offset (rad)
            float(y_offset_rad),                # y angular offset (rad)
            distance,                           # distance (m)
            0.0,                                # target x size (rad)
            0.0,                                # target y size (rad)
            0.0, 0.0, 0.0,                      # pos on frame (unused)
            (1,0,0,0),                          # orientation quaternion (w,x,y,z)
            2,                                  # type: 2 = Fiducial
            1                                   # position_valid
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

def send_vision_position_message():
    global current_time, H_aeroRef_aeroBody, vehicle
    if H_aeroRef_aeroBody is not None and vehicle is not None:
        rpy_rad = np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))
        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time,
            float(H_aeroRef_aeroBody[0][3]),
            float(H_aeroRef_aeroBody[1][3]),
            float(H_aeroRef_aeroBody[2][3]),
            float(rpy_rad[0]),
            float(rpy_rad[1]),
            float(rpy_rad[2])
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

def send_confidence_level_dummy_message():
    global data, current_confidence, vehicle
    if data is not None and vehicle is not None:
        print("INFO: Tracking confidence:", pose_data_confidence_level[data.tracker_confidence])
        msg = vehicle.message_factory.vision_position_delta_encode(
            0,                  # ts (unused)
            0,                  # time since last frame
            [0.0, 0.0, 0.0],    # angle_delta
            [0.0, 0.0, 0.0],    # position_delta
            float(data.tracker_confidence * 100 / 3)
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

        if (current_confidence is None) or (current_confidence != data.tracker_confidence):
            current_confidence = data.tracker_confidence
            confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
            status_msg = vehicle.message_factory.statustext_encode(
                3, confidence_status_string.encode()
            )
            vehicle.send_mavlink(status_msg)
            vehicle.flush()

def set_default_global_origin():
    if vehicle is None:
        return
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        int(vehicle._master.source_system),
        int(home_lat),
        int(home_lon),
        int(home_alt)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def set_default_home_position():
    if vehicle is None:
        return
    x = y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z
    approach_x, approach_y, approach_z = 0, 0, 1
    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system),
        int(home_lat), int(home_lon), int(home_alt),
        x, y, z,
        q,
        approach_x, approach_y, approach_z
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def update_timesync(ts=0, tc=0):
    if vehicle is None:
        return
    if ts == 0:
        ts = int(round(time.time() * 1000))
    msg = vehicle.message_factory.timesync_encode(tc, ts)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def statustext_callback(self, attr_name, value):
    if value.text in ("GPS Glitch", "GPS Glitch cleared", "EKF2 IMU1 ext nav yaw alignment complete"):
        time.sleep(0.1)
        print("INFO: Setting EKF home with default GPS (DELS)")
        set_default_global_origin()
        set_default_home_position()

def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    heading_north_yaw = value.yaw
    print("INFO: ATTITUDE yaw (deg):", heading_north_yaw * 180.0 / m.pi)

def scale_update():
    global scale_factor
    while not shutdown_event.is_set():
        try:
            new_scale = input("INFO: Type new scale (float):\n")
            if new_scale is None:
                continue
            scale_factor = float(new_scale)
            print("INFO: New scale is", scale_factor)
        except Exception as e:
            print("WARN: Invalid scale input:", e)

def vehicle_connect():
    global vehicle
    try:
        vehicle = connect(connection_string, wait_ready=True,
                          baud=connection_baudrate, source_system=1)
        return True
    except KeyboardInterrupt:
        print("INFO: KeyboardInterrupt during vehicle_connect")
        shutdown_event.set()
        return False
    except Exception as e:
        print(f"WARN: Vehicle connection error: {e}. Retrying...")
        return False

# ------------------------------
# RealSense helpers (resilient)
# ------------------------------
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx, 0, intrinsics.ppx],
                     [0, intrinsics.fy, intrinsics.ppy],
                     [0, 0, 1]])

def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

def realsense_connect():
    """Start RealSense pipeline with pose + fisheye streams."""
    global pipe
    p = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    cfg.enable_stream(rs.stream.fisheye, 1)
    cfg.enable_stream(rs.stream.fisheye, 2)
    p.start(cfg)
    pipe = p
    return p

def init_rectification_and_params(pipeline):
    """Initialize stereo rectification, projection, and AprilTag camera params."""
    profiles = pipeline.get_active_profile()
    streams = {
        "left":  profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
        "right": profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()
    }
    intrinsics = {
        "left":  streams["left"].get_intrinsics(),
        "right": streams["right"].get_intrinsics()
    }
    if debug_enable == 1:
        print("INFO: T265 Left  intrinsics:", intrinsics["left"])
        print("INFO: T265 Right intrinsics:", intrinsics["right"])

    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])

    (R, T) = get_extrinsics(streams["left"], streams["right"])
    # Stereo rectification parameters
    window_size = 5
    min_disp = 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp

    stereo_fov_rad = 90 * (m.pi/180)    # target FOV
    stereo_height_px = 300              # 300x300
    stereo_focal_px = stereo_height_px/2 / m.tan(stereo_fov_rad/2)

    R_left = np.eye(3)
    R_right = R

    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0, 0, 1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0] * stereo_focal_px

    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(
        K_left, D_left, R_left, P_left, stereo_size, m1type
    )
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(
        K_right, D_right, R_right, P_right, stereo_size, m1type
    )
    undistort_rectify = {"left": (lm1, lm2), "right": (rm1, rm2)}

    camera_params = [stereo_focal_px, stereo_focal_px, stereo_cx, stereo_cy]
    return undistort_rectify, camera_params

def restart_realsense_pipeline(backoff_s=2.0, max_backoff_s=30.0):
    """Restart RealSense pipeline with exponential backoff, unless shutting down."""
    global pipe
    delay = backoff_s
    while not shutdown_event.is_set():
        try:
            if pipe is not None:
                try:
                    pipe.stop()
                except Exception:
                    pass
            print("INFO: Restarting RealSense pipeline...")
            realsense_connect()
            undistort_rectify, camera_params = init_rectification_and_params(pipe)
            print("INFO: RealSense restarted.")
            return undistort_rectify, camera_params
        except Exception as e:
            print(f"WARN: RealSense restart failed: {e}. Retrying in {delay:.1f}s...")
            time.sleep(delay)
            delay = min(delay * 2.0, max_backoff_s)
    return None, None

# ------------------------------
# Signal handling (graceful stop)
# ------------------------------
def _handle_sigterm(signum, frame):
    print("INFO: SIGTERM received — shutting down gracefully.")
    shutdown_event.set()

def _handle_sigint(signum, frame):
    print("INFO: SIGINT received — shutting down gracefully.")
    shutdown_event.set()

signal.signal(signal.SIGTERM, _handle_sigterm)
signal.signal(signal.SIGINT, _handle_sigint)

# ------------------------------
# Start up: RealSense & Vehicle
# ------------------------------
print("INFO: Connecting to RealSense camera...")
try:
    realsense_connect()
    print("INFO: RealSense connected.")
except Exception as e:
    print(f"WARN: RealSense initial connect failed: {e}")
    undistort_rectify, camera_params = restart_realsense_pipeline()
else:
    undistort_rectify, camera_params = init_rectification_and_params(pipe)

print("INFO: Connecting to vehicle...")
while not shutdown_event.is_set() and not vehicle_connect():
    time.sleep(2.0)
if shutdown_event.is_set():
    print("INFO: Startup aborted due to shutdown request.")
    try:
        if pipe: pipe.stop()
    except Exception:
        pass
    sys.exit(0)
print("INFO: Vehicle connected.")

vehicle.add_message_listener('STATUSTEXT', statustext_callback)
if compass_enabled == 1:
    vehicle.add_message_listener('ATTITUDE', att_msg_callback)

data = None
current_confidence = None
H_aeroRef_aeroBody = None
H_camera_tag = None
is_landing_tag_detected = False
heading_north_yaw = None

# Background jobs
sched = BackgroundScheduler()
sched.add_job(send_vision_position_message, 'interval', seconds=max(1.0/vision_msg_hz, 0.001))
sched.add_job(send_confidence_level_dummy_message, 'interval', seconds=max(1.0/confidence_msg_hz, 0.001))
sched.add_job(send_land_target_message, 'interval', seconds=max(1.0/landing_target_msg_hz, 0.001))

if scale_calib_enable:
    scale_update_thread = threading.Thread(target=scale_update, daemon=True)
    scale_update_thread.start()

sched.start()

if compass_enabled == 1:
    time.sleep(1.0)

print("INFO: Starting main loop...")

# ------------------------------
# Main loop with resilient RealSense handling
# ------------------------------
try:
    while not shutdown_event.is_set():
        try:
            frames = pipe.wait_for_frames()
        except Exception as e:
            # Any failure fetching frames -> attempt restart unless shutting down
            print(f"WARN: RealSense frames error: {e}")
            if shutdown_event.is_set():
                break
            undistort_rectify, camera_params = restart_realsense_pipeline()
            if undistort_rectify is None:
                break
            continue

        # Pose stream
        pose = frames.get_pose_frame()
        if pose:
            current_time = int(round(time.time() * 1e6))
            data = pose.get_pose_data()

            H_T265Ref_T265body = tf.quaternion_matrix(
                [data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]
            )
            H_T265Ref_T265body[0][3] = data.translation.x * scale_factor
            H_T265Ref_T265body[1][3] = data.translation.y * scale_factor
            H_T265Ref_T265body[2][3] = data.translation.z * scale_factor

            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot(
                H_T265Ref_T265body.dot(H_T265body_aeroBody)
            )

            if body_offset_enabled == 1:
                H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                H_body_camera[0][3] = body_offset_x
                H_body_camera[1][3] = body_offset_y
                H_body_camera[2][3] = body_offset_z
                H_camera_body = np.linalg.inv(H_body_camera)
                H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

            if compass_enabled == 1 and heading_north_yaw is not None:
                H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot(
                    tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz')
                )

            if debug_enable == 1:
                try:
                    os.system('clear')
                except Exception:
                    pass
                print("DEBUG: Raw RPY[deg]: {}".format(
                    np.array(tf.euler_from_matrix(H_T265Ref_T265body, 'sxyz')) * 180 / m.pi))
                print("DEBUG: NED RPY[deg]: {}".format(
                    np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
                print("DEBUG: Raw pos xyz : {}".format(
                    np.array([data.translation.x, data.translation.y, data.translation.z])))
                print("DEBUG: NED pos xyz : {}".format(
                    np.array(tf.translation_from_matrix(H_aeroRef_aeroBody))))

        # Images (left/right fisheye)
        try:
            f1 = frames.get_fisheye_frame(1).as_video_frame()
            left_data = np.asanyarray(f1.get_data())
            f2 = frames.get_fisheye_frame(2).as_video_frame()
            right_data = np.asanyarray(f2.get_data())
        except Exception as e:
            print(f"WARN: RealSense image fetch error: {e}")
            if shutdown_event.is_set():
                break
            undistort_rectify, camera_params = restart_realsense_pipeline()
            if undistort_rectify is None:
                break
            continue

        frame_copy = {"left": left_data, "right": right_data}

        # Undistort + rectify
        try:
            center_undistorted = {
                "left": cv2.remap(frame_copy["left"],
                                  undistort_rectify["left"][0],
                                  undistort_rectify["left"][1],
                                  interpolation=cv2.INTER_LINEAR),
                "right": cv2.remap(frame_copy["right"],
                                   undistort_rectify["right"][0],
                                   undistort_rectify["right"][1],
                                   interpolation=cv2.INTER_LINEAR)
            }
        except Exception as e:
            print(f"WARN: Rectification error: {e}")
            if shutdown_event.is_set():
                break
            undistort_rectify, camera_params = restart_realsense_pipeline()
            if undistort_rectify is None:
                break
            continue

        # AprilTag detection (family tagStandard41h12), target id=113
        tags = []
        try:
            tags = at_detector.detect(center_undistorted[tag_image_source],
                                      True, camera_params, tag_landing_size)
        except Exception as e:
            print(f"WARN: AprilTag detect error: {e}")

        is_landing_tag_detected = False
        H_camera_tag = None

        if tags:
            for tag in tags:
                if int(tag.tag_id) == int(tag_landing_id):
                    is_landing_tag_detected = True
                    H_camera_tag = tf.euler_matrix(0, 0, 0, 'sxyz')
                    H_camera_tag[0][3] = float(tag.pose_t[0])
                    H_camera_tag[1][3] = float(tag.pose_t[1])
                    H_camera_tag[2][3] = float(tag.pose_t[2])
                    print(f"INFO: Detected landing tag {tag.tag_id} relative to camera at "
                          f"x:{H_camera_tag[0][3]:.3f}, y:{H_camera_tag[1][3]:.3f}, z:{H_camera_tag[2][3]:.3f}")
                    break  # We only care about the landing tag

        # Visualization
        if visualization == 1:
            try:
                tags_img = center_undistorted[tag_image_source].copy()
                for tag in (tags or []):
                    # draw bounding box
                    for idx in range(len(tag.corners)):
                        cv2.line(tags_img,
                                 tuple(tag.corners[idx-1, :].astype(int)),
                                 tuple(tag.corners[idx, :].astype(int)),
                                 thickness=2, color=(255, 0, 0))
                    text = str(tag.tag_id)
                    textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                    cx = int((tag.corners[0, 0] + tag.corners[2, 0] - textsize[0]) / 2)
                    cy = int((tag.corners[0, 1] + tag.corners[2, 1] + textsize[1]) / 2)
                    cv2.putText(tags_img, text, (cx, cy),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5,
                                thickness=2, color=(255, 0, 0))

                cv2.imshow(WINDOW_TITLE, tags_img)
                key = cv2.waitKey(1)
                if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                    print("INFO: Visualization window closed by user.")
                    shutdown_event.set()
                    break
            except Exception as e:
                print(f"WARN: Visualization error: {e}")

except Exception as e:
    print("ERROR:", e)
finally:
    try:
        sched.shutdown(wait=False)
    except Exception:
        pass
    try:
        if pipe is not None:
            pipe.stop()
    except Exception:
        pass
    try:
        if vehicle is not None:
            vehicle.close()
    except Exception:
        pass
    try:
        if visualization == 1:
            cv2.destroyAllWindows()
    except Exception:
        pass
    print("INFO: RealSense pipeline and vehicle closed. Exiting.")
    sys.exit(0)
