#!/usr/bin/env python3
# -*- coding: utf-8 -*-

##################################################################
##  ONICS – Optical Navigation and Interference Control System  ##
##                            T265 Node                         ##
##                T265-to-EKF2 + AprilTag Landing               ##
##################################################################

"""
ONICS‑T – T265-to-MAVLink bridge used by ONICS‑lite.

This wrapper is designed to be supervised by ONICS‑lite (onlcs-lite.py)
but remains runnable on its own for debugging or standalone use.
"""

import sys
import os
import signal
import time
import math as m
import threading
import argparse
import builtins
from datetime import datetime

# Ensure MAVLink 2.0 is enabled before importing pymavlink.
os.environ.setdefault("MAVLINK20", "1")

import numpy as np
import cv2

import pyrealsense2.pyrealsense2 as rs
import transformations as tf

from pymavlink import mavutil

# ---------- Timestamp all prints ----------
def _now_ts():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
_builtin_print = builtins.print
def print(*args, **kwargs):
    _builtin_print(f"[{_now_ts()}]", *args, **kwargs)
# -----------------------------------------

try:
    import apriltags3
except ImportError:
    raise ImportError(
        "Please download apriltags3.py and put it beside this script or add it to PYTHONPATH."
    )

try:
    import psutil  # optional, improves performance snapshots
    _HAVE_PSUTIL = True
except Exception:
    _HAVE_PSUTIL = False

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
mavlink_connection = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

# AprilTag detection configuration: tag41_12_00113
APRILTAG_FAMILY = 'tagStandard41h12'
tag_landing_id = 113
tag_landing_size = 0.144     # meters (edge length incl. border)
tag_image_source = "right"   # T265 supports "left" or "right"
legacy_tag_lost_us = 2_000_000
legacy_tag_max_alt = 10.0

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
parser.add_argument('--debug_enable', type=int, help="Enable debug print (1 to enable).")
parser.add_argument('--legacy', action='store_true',
                    help="Use legacy AprilTag handling from t265_precland_apriltags.py.")
args = parser.parse_args()

# Helper: coalesce None to default but keep falsy numeric values like 0
def coalesce(val, default):
    return default if val is None else val

def _validate_positive_rate(name: str, value: float, default: float) -> float:
    """Return a positive frequency value, falling back to default on invalid input."""
    try:
        coerced = float(coalesce(value, default))
    except (TypeError, ValueError):
        print(f"WARN: Invalid {name} {value!r}; using default {default} Hz instead.")
        return float(default)

    if not m.isfinite(coerced) or coerced <= 0:
        print(f"WARN: Invalid {name} {value!r}; using default {default} Hz instead.")
        return float(default)

    return coerced


connection_string      = coalesce(args.connect, connection_string_default)
connection_baudrate    = coalesce(args.baudrate, connection_baudrate_default)
vision_msg_hz          = _validate_positive_rate("vision_msg_hz", args.vision_msg_hz, vision_msg_hz_default)
landing_target_msg_hz  = _validate_positive_rate("landing_target_msg_hz", args.landing_target_msg_hz, landing_target_msg_hz_default)
confidence_msg_hz      = _validate_positive_rate("confidence_msg_hz", args.confidence_msg_hz, confidence_msg_hz_default)
scale_calib_enable     = bool(args.scale_calib_enable) if args.scale_calib_enable is not None else False
camera_orientation     = coalesce(args.camera_orientation, camera_orientation_default)
debug_enable           = 1 if (args.debug_enable and int(args.debug_enable) == 1) else 0
legacy_mode            = bool(args.legacy)

print(f"INFO: Using connection_string: {connection_string}")
print(f"INFO: Using connection_baudrate: {connection_baudrate}")
print(f"INFO: Using vision_msg_hz: {vision_msg_hz}")
print(f"INFO: Using landing_target_msg_hz: {landing_target_msg_hz}")
print(f"INFO: Using confidence_msg_hz: {confidence_msg_hz}")
print(f"INFO: T265 orientation: {camera_orientation} (0=forward/right-USB, 1=down/right-USB)")
print(f"INFO: AprilTag legacy mode: {'Enabled' if legacy_mode else 'Disabled'}")
print("INFO: Camera position offset:", "Enabled" if body_offset_enabled else "Disabled",
      f"({body_offset_x}, {body_offset_y}, {body_offset_z})")
print("INFO: Compass:", "Enabled (north-aligned yaw)" if compass_enabled else "Disabled")

if scale_calib_enable:
    print("\nINFO: SCALE CALIBRATION MODE — DO NOT RUN IN FLIGHT.\n"
          "INFO: Type a new scale (float) at any time.\n")
else:
    print(f"INFO: Scale factor: {scale_factor}")

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
def create_apriltag_detector():
    """Validate AprilTag prerequisites and return a detector or None."""
    if APRILTAG_FAMILY not in apriltags3.Detector._FAMILIES:
        print(
            f"WARN: Requested AprilTag family '{APRILTAG_FAMILY}' is unsupported; "
            f"supported families: {sorted(apriltags3.Detector._FAMILIES)}"
        )
        return None

    try:
        libc = apriltags3.Detector._load_library()
    except Exception as e:
        print(f"WARN: AprilTag shared library unavailable: {e}")
        return None

    create_fn, destroy_fn = apriltags3.Detector._FAMILIES[APRILTAG_FAMILY]
    required_symbols = {create_fn, destroy_fn, "apriltag_detector_create", "apriltag_detector_destroy"}
    missing_symbols = sorted(fn for fn in required_symbols if not hasattr(libc, fn))
    if missing_symbols:
        print(
            f"WARN: AprilTag library missing required symbol(s) {missing_symbols} for family "
            f"'{APRILTAG_FAMILY}'"
        )
        return None

    try:
        return apriltags3.Detector(
            families=APRILTAG_FAMILY,       # <<--- tagStandard41h12
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
    except Exception as e:
        print(f"WARN: Failed to initialize AprilTag detector: {e}")
        return None


at_detector = None
legacy_detector = None
if legacy_mode:
    legacy_detector = apriltags3.Detector(families=APRILTAG_FAMILY)
else:
    at_detector = create_apriltag_detector()


def _detect_apriltags(frames, camera_params, image_source):
    """Detect AprilTags with guarded source selection and error handling."""
    if at_detector is None:
        print("WARN: AprilTag detector unavailable; skipping detection.")
        return []

    if camera_params is None:
        print("WARN: Missing camera parameters; skipping AprilTag detection.")
        return []

    source = image_source if image_source in frames else None
    if source is None:
        for fallback in ("right", "left"):
            if fallback in frames:
                print(f"WARN: AprilTag source '{image_source}' unavailable; using '{fallback}' instead.")
                source = fallback
                break

    if source is None:
        print("WARN: No valid image source available for AprilTag detection.")
        return []

    frame = frames[source]
    if frame is None:
        print(f"WARN: AprilTag source '{source}' frame is None; skipping detection.")
        return []

    frame_u8 = np.ascontiguousarray(frame, dtype=np.uint8)

    try:
        detections = at_detector.detect(frame_u8, True, camera_params, tag_landing_size)
    except Exception as e:
        print(f"WARN: AprilTag detect error on {source} image: {e}")
        return []
    return detections

# ------------------------------
# MAVLink helpers
# ------------------------------
def att_msg_callback(value):
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
    global mavlink_connection
    try:
        mavlink_connection = mavutil.mavlink_connection(
            connection_string,
            baud=connection_baudrate,
            autoreconnect=True,
            source_system=255,
            source_component=0
        )
        return True
    except KeyboardInterrupt:
        print("INFO: KeyboardInterrupt during vehicle_connect")
        shutdown_event.set()
        return False
    except Exception as e:
        print(f"WARN: MAVLink connection error: {e}. Retrying...")
        return False

def mavlink_receive_loop():
    """Receive MAVLink messages only (no outbound MAVLink)."""
    global latest_altitude_m, heading_north_yaw
    while not shutdown_event.is_set():
        if mavlink_connection is None:
            time.sleep(0.1)
            continue
        try:
            msg = mavlink_connection.recv_match(blocking=True, timeout=1)
        except Exception as e:
            print(f"WARN: MAVLink receive error: {e}")
            time.sleep(0.5)
            continue

        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "GLOBAL_POSITION_INT":
            try:
                latest_altitude_m = float(msg.relative_alt) / 1000.0
            except Exception:
                continue
        elif msg_type == "VFR_HUD":
            try:
                latest_altitude_m = float(msg.alt)
            except Exception:
                continue
        elif compass_enabled == 1 and msg_type == "ATTITUDE":
            att_msg_callback(msg)

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
    raw_camera_params = [
        intrinsics["right"].fx,
        intrinsics["right"].fy,
        intrinsics["right"].ppx,
        intrinsics["right"].ppy,
    ]
    return undistort_rectify, camera_params, raw_camera_params

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
            undistort_rectify, camera_params, raw_camera_params = init_rectification_and_params(pipe)
            print("INFO: RealSense restarted.")
            return undistort_rectify, camera_params, raw_camera_params
        except Exception as e:
            print(f"WARN: RealSense restart failed: {e}. Retrying in {delay:.1f}s...")
            time.sleep(delay)
            delay = min(delay * 2.0, max_backoff_s)
    print("INFO: Skipping RealSense restart because shutdown was requested.")
    return None, None, None

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
print("INFO: Connecting to RealSense T265...")
try:
    realsense_connect()
    print("INFO: RealSense T265 connected.")
except Exception as e:
    print(f"WARN: RealSense initial connect failed: {e}")
    undistort_rectify, camera_params, raw_camera_params = restart_realsense_pipeline()
else:
    undistort_rectify, camera_params, raw_camera_params = init_rectification_and_params(pipe)

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
print("INFO: MAVLink connection ready (receive-only).")

mavlink_rx_thread = threading.Thread(target=mavlink_receive_loop, daemon=True)
mavlink_rx_thread.start()

data = None
current_confidence = None
H_aeroRef_aeroBody = None
H_camera_tag = None
is_landing_tag_detected = False
heading_north_yaw = None
landing_override = False
last_tag_us = 0
latest_altitude_m = 0.0

# ------------------------------
# Performance tracking & dynamic pose rate
# ------------------------------
perf_lock = threading.Lock()
loop_dt_ema = None  # seconds, exponential moving average
_ema_alpha = 0.1    # smoothing for loop time EMA

def _perf_update_loop_dt(dt):
    """Update EMA of main-loop iteration time."""
    global loop_dt_ema
    with perf_lock:
        if dt <= 0:
            return
        if loop_dt_ema is None:
            loop_dt_ema = dt
        else:
            loop_dt_ema = _ema_alpha * dt + (1 - _ema_alpha) * loop_dt_ema

def get_performance_snapshot():
    """Return a dict with current perf snapshot."""
    with perf_lock:
        dt = loop_dt_ema
    loop_fps = (1.0 / dt) if (dt and dt > 0) else None
    cpu = psutil.cpu_percent(interval=None) if _HAVE_PSUTIL else None
    return {"loop_dt_ema_s": dt, "loop_fps": loop_fps, "cpu_percent": cpu}

if scale_calib_enable:
    scale_update_thread = threading.Thread(target=scale_update, daemon=True)
    scale_update_thread.start()

print("INFO: Starting main loop...")

# ------------------------------
# Main loop with resilient RealSense handling + perf tracking
# ------------------------------
try:
    while not shutdown_event.is_set():
        iter_start = time.time()
        try:
            frames = pipe.wait_for_frames(timeout_ms=5000)
        except Exception as e:
            print(f"WARN: RealSense frames error: {e}")
            if shutdown_event.is_set():
                break
            undistort_rectify, camera_params, raw_camera_params = restart_realsense_pipeline()
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
            undistort_rectify, camera_params, raw_camera_params = restart_realsense_pipeline()
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
            undistort_rectify, camera_params, raw_camera_params = restart_realsense_pipeline()
            if undistort_rectify is None:
                break
            continue

        if legacy_mode:
            alt = latest_altitude_m

            now_us = int(round(time.time() * 1e6))
            if landing_override and now_us - last_tag_us > legacy_tag_lost_us:
                landing_override = False
                print("INFO: Tag lost – override cancelled")

            # ---- April-Tag detection up to 10 m
            H_camera_tag = None
            is_landing_tag_detected = False
            if alt <= legacy_tag_max_alt:
                img = frames.get_fisheye_frame(2)
                if img:
                    tags = legacy_detector.detect(np.asanyarray(img.get_data()),
                                                  True, [285, 285, 160, 160], tag_landing_size)
                    for tag in tags:
                        if tag.tag_id == tag_landing_id:
                            last_tag_us = now_us
                            if not landing_override:
                                print("INFO: Landing tag detected – override ON")
                            landing_override = True
                            mat = tf.euler_matrix(0, 0, 0, 'sxyz')
                            mat[0:3, 3] = tag.pose_t
                            H_camera_tag = mat
                            is_landing_tag_detected = True
                            break
        else:
            # AprilTag detection (legacy-style raw frame + native pose estimation)
            tags = _detect_apriltags(frame_copy, raw_camera_params, tag_image_source)

            is_landing_tag_detected = False
            H_camera_tag = None

            if tags:
                for tag in tags:
                    if int(tag.tag_id) == int(tag_landing_id):
                        pose_t = getattr(tag, "pose_t", None)
                        if pose_t is None:
                            print("WARN: Landing tag detected but pose translation is unavailable.")
                            break

                        pose_t_vec = np.asarray(pose_t, dtype=float).reshape(-1)
                        if pose_t_vec.size < 3 or not np.all(np.isfinite(pose_t_vec[:3])):
                            print(f"WARN: Landing tag pose translation invalid: {pose_t!r}")
                            break

                        is_landing_tag_detected = True
                        H_camera_tag = tf.euler_matrix(0, 0, 0, 'sxyz')
                        H_camera_tag[0][3] = float(pose_t_vec[0])
                        H_camera_tag[1][3] = float(pose_t_vec[1])
                        H_camera_tag[2][3] = float(pose_t_vec[2])
                        print(f"INFO: Detected landing tag {tag.tag_id} relative to T265 at "
                              f"x:{H_camera_tag[0][3]:.3f}, y:{H_camera_tag[1][3]:.3f}, z:{H_camera_tag[2][3]:.3f}")
                        break  # Only care about the landing tag

        # ---- Update performance metrics at end of loop ----
        iter_end = time.time()
        _perf_update_loop_dt(iter_end - iter_start)

except Exception as e:
    print("ERROR:", e)
finally:
    try:
        if pipe is not None:
            pipe.stop()
    except Exception:
        pass
    try:
        if mavlink_connection is not None:
            mavlink_connection.close()
    except Exception:
        pass
    print("INFO: RealSense pipeline and MAVLink connection closed. Exiting.")
    sys.exit(0)
