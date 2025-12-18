#!/usr/bin/env python3
"""
Precision-Landing & Vision-Nav with Intel® RealSense™ T265
──────────────────────────────────────────────────────────
Features
• April-Tag 0 auto-takes over for landing (override lasts 2 s after tag loss)
• Optional altitude hysteresis:
      USB stream off   above 15 m
      Vision muted     10 – 15 m
      Vision primed     5 – 10 m   (tag search ON)
      Vision active    ≤ 5 m
• Compatible with both `pyrealsense2` and `pyrealsense2.pyrealsense2`
• Clean shutdown on Ctrl-C / SIGTERM
• Minimal external deps: pyrealsense2, DroneKit-Python, pymavlink,
  apriltags3.py, OpenCV, apscheduler, PyGObject (only if you use visualization)

CLI (all optional)
  --connect PORT        serial device to FCU  (default /dev/ttyUSB0)
  --baudrate N          baud rate             (default 921600)
  --gate                enable altitude gating
  --stop_stream         (with --gate) power-saves USB above gate_hi
  --gate_hi 15          USB off threshold     (default 15 m)
  --gate_lo 10          vision mute threshold (default 10 m)
  --gate_on 5           vision resume thresh  (default  5 m)
  --debug               verbose logging

Example
  python3 t265_precland_apriltags.py --gate --stop_stream --debug
"""

# ─── stdlib
from __future__ import annotations
import argparse, math as m, os, signal, sys, threading, time, logging, faulthandler
from typing import Optional
faulthandler.enable()

# ─── add /usr/local/lib for custom builds
sys.path.append("/usr/local/lib/")

# ─── RealSense import shim (normal wheel *or* nested layout)
try:
    import pyrealsense2 as rs
except ImportError:
    from pyrealsense2 import pyrealsense2 as rs  # type: ignore

# ─── third-party
import numpy as np, cv2, transformations as tf, apriltags3
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil
from dronekit import connect

# ───────────── constants ─────────────
SERIAL_DEF     = "/dev/ttyUSB0"
BAUD_DEF       = 921600
TAG_ID         = 0
TAG_SIZE_M     = 0.144
TAG_SOURCE     = "right"
TAG_LOST_US    = 2_000_000

# default altitude gates (overridden by CLI)
GATE_HI_DEF = 15.0
GATE_LO_DEF = 10.0
GATE_ON_DEF = 5.0
TAG_MAX_ALT = 10.0

VISION_HZ  = 20.0
LAND_HZ    = 20.0
CONF_HZ    = 1.0

# EKF dummy home
HOME_LAT = HOME_LON = HOME_ALT = 0

# ───────────── runtime globals ─────────────
pipe: rs.pipeline | None = None
streaming = True
vision_enabled = True
landing_override = False
last_tag_us = 0
current_us = 0
H_aero_body: Optional[np.ndarray] = None
landing_H_cam_tag: Optional[np.ndarray] = None
tracker_conf = 0

stop_evt = threading.Event()

# ───────────── helper logging printer ─────────────
log = logging.getLogger("t265")

def now_us() -> int:
    return int(time.time() * 1e6)

# ───────────── MAVLink send helpers ─────────────
def send_vision(vehicle):
    if not vision_enabled or H_aero_body is None:
        return
    r,p,y = tf.euler_from_matrix(H_aero_body,'sxyz')
    x,y_,z = tf.translation_from_matrix(H_aero_body)
    vehicle.message_factory.vision_position_estimate_encode(
        current_us, x, y_, z, r, p, y
    ).send()

def send_landing(vehicle):
    if not vision_enabled or landing_H_cam_tag is None:
        return
    x,y,z = landing_H_cam_tag[0:3,3]
    vehicle.message_factory.landing_target_encode(
        current_us, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,
        m.atan2(x,z), m.atan2(y,z), float(m.sqrt(x*x+y*y+z*z)),
        0,0,0,0,0, (1,0,0,0), 2,1
    ).send()

def send_conf(vehicle):
    vehicle.message_factory.statustext_encode(
        3, f"Tracking {tracker_conf}".encode()).send()

# ───────────── graceful exit ─────────────
def safe_exit(code: int = 0):
    stop_evt.set()
    try: sched.shutdown(wait=False)
    except Exception: pass
    try: pipe.stop()               # type: ignore
    except Exception: pass
    try: vehicle.close()           # type: ignore
    except Exception: pass
    log.info("Bye.")
    sys.exit(code)

# ───────────── main ─────────────
def main():
    global pipe, streaming, vision_enabled, landing_override
    global last_tag_us, current_us, H_aero_body, landing_H_cam_tag, tracker_conf
    global vehicle, sched

    # CLI
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--connect',   default=SERIAL_DEF)
    ap.add_argument('--baudrate',  type=int, default=BAUD_DEF)
    ap.add_argument('--gate',      action='store_true')
    ap.add_argument('--stop_stream', action='store_true')
    ap.add_argument('--gate_hi',   type=float, default=GATE_HI_DEF)
    ap.add_argument('--gate_lo',   type=float, default=GATE_LO_DEF)
    ap.add_argument('--gate_on',   type=float, default=GATE_ON_DEF)
    ap.add_argument('--debug',     action='store_true')
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO,
                        format='%(asctime)s [%(levelname)s] %(message)s')

    # MAVLink
    log.info("Connecting MAVLink…")
    vehicle = connect(args.connect, baud=args.baudrate, wait_ready=True,
                      source_system=1)
    vehicle.message_factory.set_gps_global_origin_encode(
        vehicle._master.source_system, HOME_LAT, HOME_LON, HOME_ALT).send()

    # RealSense
    log.info("Starting RealSense T265…")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.pose)
    cfg.enable_stream(rs.stream.fisheye,1)
    cfg.enable_stream(rs.stream.fisheye,2)
    pipe.start(cfg)

    # Orientation (down-facing, USB-right)
    H_aero_T265 = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_cam_body  = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])

    detector = apriltags3.Detector(families='tag36h11')

    # Schedulers
    sched = BackgroundScheduler()
    sched.add_job(lambda: send_vision(vehicle), 'interval', seconds=1/VISION_HZ)
    sched.add_job(lambda: send_landing(vehicle),'interval', seconds=1/LAND_HZ)
    sched.add_job(lambda: send_conf(vehicle),  'interval', seconds=1/CONF_HZ)
    sched.start()

    # Signals
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, lambda *_: safe_exit(0))

    log.info("Main loop…")
    try:
        while not stop_evt.is_set():
            alt = vehicle.location.global_relative_frame.alt or 0.0
            now = now_us()

            # ---- landing-override timeout
            if landing_override and now - last_tag_us > TAG_LOST_US:
                landing_override = False
                log.info("Tag lost – override cancelled")

            # ---- altitude gate
            if args.gate:
                # USB power save
                if args.stop_stream and not landing_override:
                    if streaming and alt > args.gate_hi:
                        pipe.stop(); streaming=False
                        log.info("T265 stream OFF")
                    elif not streaming and alt <= args.gate_hi:
                        pipe.start(cfg); streaming=True
                        log.info("T265 stream ON")
                # vision mute
                if landing_override:
                    vision_enabled = True
                elif alt <= args.gate_on:
                    vision_enabled = True
                elif alt >= args.gate_lo:
                    vision_enabled = False
            else:
                vision_enabled = True

            if not streaming:
                time.sleep(0.05); continue

            frames = pipe.wait_for_frames(timeout_ms=1000)
            pose   = frames.get_pose_frame()
            if pose:
                current_us = now
                pdata  = pose.get_pose_data()
                tracker_conf = pdata.tracker_confidence
                H_T265Ref_T265body = tf.quaternion_matrix(
                    [pdata.rotation.w, pdata.rotation.x, pdata.rotation.y, pdata.rotation.z])
                H_T265Ref_T265body[0:3,3] = [pdata.translation.x,
                                              pdata.translation.y,
                                              pdata.translation.z]
                H_aero_body = H_aero_T265 @ H_T265Ref_T265body @ H_cam_body

            # ---- April-Tag detection up to 10 m
            landing_H_cam_tag = None
            if alt <= TAG_MAX_ALT and streaming:
                img = frames.get_fisheye_frame(2)
                if img:
                    tags = detector.detect(np.asanyarray(img.get_data()),
                                           True, [285,285,160,160], TAG_SIZE_M)
                    for t in tags:
                        if t.tag_id == TAG_ID:
                            last_tag_us = now
                            if not landing_override:
                                log.info("Landing tag detected – override ON")
                            landing_override = True
                            mat = tf.euler_matrix(0,0,0,'sxyz')
                            mat[0:3,3] = t.pose_t
                            landing_H_cam_tag = mat
                            break

    except KeyboardInterrupt:
        pass
    finally:
        safe_exit(0)

if __name__ == "__main__":
    main()
