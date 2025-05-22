#!/usr/bin/env python3

######################################################
##          librealsense D435i to MAVLink           ##
##             **now with IMU support**             ##  
######################################################

# WARNING: Enabling RTSP may cause unresponsiveness. 
# It is inadvisable to try and stream video from the device while it is acting as an obstacle detector.
# Optional - D435i IMU → ATTITUDE_QUATERNION (backup yaw)

# ─── Standard library ─────────────────────────────────────────────
import argparse, math as m, os, signal, socket, sys, threading, time, faulthandler
from pathlib import Path
from typing import Callable, Dict, Optional
faulthandler.enable()           # back-trace hard crashes immediately

# ─── RealSense import shim (handles WATNE layout) ─────────────────
try:
    import pyrealsense2 as rs                    # normal wheels
except ImportError:
    try:                                         # e.g. pyrealsense2-pyrealsense2
        from pyrealsense2 import pyrealsense2 as rs
    except ImportError as e:
        raise ImportError(
            "Cannot import librealsense. Install 'pyrealsense2' or "
            "'pyrealsense2-pyrealsense2' and ensure LD_LIBRARY_PATH is set."
        ) from e

# ─── Third-party libs ─────────────────────────────────────────────
import numpy as np
import cv2
from apscheduler.schedulers.background import BackgroundScheduler
from gi.repository import Gst, GstRtspServer, GLib
from pymavlink import mavutil

# ─── Configuration constants (unchanged) ─────────────────────────
DEPTH_W, DEPTH_H, FPS = 640, 480, 30
COLOR_W, COLOR_H      = 640, 480
DEPTH_RANGE_M         = (0.1, 8.0)
DIST_LEN              = 72
MIN_CM, MAX_CM        = [int(x*100) for x in DEPTH_RANGE_M]

OBST_LINE_RATIO       = 0.18
OBST_LINE_PIXELS      = 10

RTSP_DEFAULT          = True
RTSP_PORT, RTSP_PATH  = "8554", "/d4xx"

PRESET_FILE           = Path(__file__).with_suffix('.json')

# ─── Runtime globals (trimmed for brevity) ───────────────────────
pipe: rs.pipeline | None     = None
depth_scale: float           = 0.0
angle_offset: float          = 0.0
increment_f: float           = 0.0
current_us: int              = 0
last_obst_us: int            = 0

vision_enabled = True
rtsp_frame: Optional[np.ndarray] = None
distances = np.ones(DIST_LEN, dtype=np.uint16) * (MAX_CM+1)

stop_evt = threading.Event()

# ─── Utility functions (unchanged) ───────────────────────────────
def pr(msg: str) -> None:
    print(msg, flush=True)

def now_us() -> int:
    return int(time.time()*1e6)

# (all helper functions from the previous script remain identical—
#  set_obst_params, update_distances, send_obstacle_distance, etc.)

# ─── Main program (identical logic) ──────────────────────────────
def main() -> None:
    # CLI
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--connect', default='/dev/ttyUSB0')
    ap.add_argument('--baudrate', type=int, default=921600)
    ap.add_argument('--obst_hz', type=float, default=15.0)
    ap.add_argument('--debug', action='store_true')
    ap.add_argument('--rtsp',  default=RTSP_DEFAULT, action=argparse.BooleanOptionalAction)
    ap.add_argument('--imu_enable', default=True, action=argparse.BooleanOptionalAction)
    args = ap.parse_args()
    if args.obst_hz <= 0:
        sys.exit("ERROR: --obst_hz must be > 0")

    # MAVLink connect + thread  … (same as before)
    # Camera start + set_obst_params  … (same as before)
    # Optional RTSP server            … (same as before)
    # Optional IMU thread             … (same as before)
    # Scheduler to send obstacle_distance … (same as before)

    # Graceful signal handler
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_evt.set())

    try:
        while not stop_evt.is_set():
            frames = pipe.wait_for_frames(timeout_ms=2000)
            depth  = frames.get_depth_frame()
            color  = frames.get_color_frame() if args.rtsp else None
            if not depth:
                continue

            global current_us
            current_us = now_us()

            depth_mat = np.asanyarray(depth.get_data())
            y_line    = int(DEPTH_H * OBST_LINE_RATIO)
            update_distances(depth_mat, y_line)
            # RTSP frame update
            if args.rtsp and color is not None:
                global rtsp_frame
                rtsp_frame = np.asanyarray(color.get_data())
    finally:
        # Shutdown sequence identical to previous script
        stop_evt.set()
        try: sched.shutdown(wait=False)
        except Exception: pass
        if pipe: pipe.stop()
        pr("INFO: d4xx bridge closed cleanly.")

if __name__ == "__main__":
    main()
