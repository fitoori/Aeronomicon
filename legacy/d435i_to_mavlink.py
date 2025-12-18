#!/usr/bin/env python3

######################################################
##          librealsense D435i to MAVLink           ##
##             **now with IMU support**             ##  
######################################################

# WARNING: Enabling RTSP may cause unresponsiveness. 
# It is inadvisable to try and stream video from the device while it is acting as an obstacle detector.
# Optional - D435i IMU → ATTITUDE_QUATERNION (backup yaw)

# ─── Standard library ──────────────────────────────────────────────
from __future__ import annotations
import argparse, math as m, os, signal, socket, sys, threading, time, faulthandler
from pathlib import Path
from typing import Callable, Dict, Optional
faulthandler.enable()                        # show tracebacks on segfault

# ─── RealSense import with compatibility shim (handles WATNE) ─────
try:
    import pyrealsense2 as rs                # normal wheel layout
except ImportError:
    try:
        from pyrealsense2 import pyrealsense2 as rs  # pyrealsense2-pyrealsense2 layout
    except ImportError as e:
        raise ImportError(
            "Cannot import Intel RealSense SDK.\n"
            "Install either 'pyrealsense2' or 'pyrealsense2-pyrealsense2' "
            "and ensure LD_LIBRARY_PATH is set."
        ) from e

# ─── Third-party packages ─────────────────────────────────────────
import numpy as np
import cv2
from apscheduler.schedulers.background import BackgroundScheduler
from gi.repository import Gst, GstRtspServer, GLib
from pymavlink import mavutil

# ─── Configuration constants ──────────────────────────────────────
DEPTH_W, DEPTH_H, FPS = 640, 480, 30
COLOR_W, COLOR_H      = 640, 480
DEPTH_RANGE_M         = (0.1, 8.0)                     # sensor working range
DIST_LEN              = 72                             # rays in OBSTACLE_DISTANCE
MIN_CM, MAX_CM        = [int(x * 100) for x in DEPTH_RANGE_M]

OBST_LINE_RATIO       = 0.18    # depth scan height (0-top)
OBST_LINE_PIXELS      = 10      # thickness of scan band

DEFAULT_CONN          = "/dev/ttyUSB0"
DEFAULT_BAUD          = 921600
DEFAULT_OBST_HZ       = 15.0

RTSP_DEFAULT          = True
RTSP_PORT, RTSP_PATH  = "8554", "/d4xx"

PRESET_FILE           = Path(__file__).with_suffix(".json")  # same dir as script

# ─── Globals set at runtime ───────────────────────────────────────
pipe:           Optional[rs.pipeline] = None
depth_scale:    float   = 0.0
angle_offset:   float   = 0.0
increment_f:    float   = 0.0
current_us:     int     = 0
last_obst_us:   int     = 0

rtsp_frame:     Optional[np.ndarray] = None
distances = np.ones(DIST_LEN, dtype=np.uint16) * (MAX_CM + 1)

# Thread / process control
stop_evt = threading.Event()

# ─── Utility print ------------------------------------------------
def pr(msg: str) -> None:
    print(msg, flush=True)

def now_us() -> int:
    return int(time.time() * 1e6)

# ────────────────────────── MAVLink helpers ──────────────────────
def mavlink_loop(conn: mavutil.mavfile, callbacks: Dict[str, Callable]) -> None:
    """Background RX loop (heartbeats + callbacks)."""
    types = list(callbacks)
    while not stop_evt.is_set():
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
        msg = conn.recv_match(type=types, timeout=1, blocking=True)
        if msg and msg.get_type() in callbacks:
            callbacks[msg.get_type()](msg)

def imu_scale(acc, gyro):
    """Convert to milli-G and milli-rad/s (RAW_IMU units)."""
    return (int(acc.x * 1000 / 9.80665),
            int(acc.y * 1000 / 9.80665),
            int(acc.z * 1000 / 9.80665),
            int(gyro.x * 1000),
            int(gyro.y * 1000),
            int(gyro.z * 1000))

# ───────────────────── Camera geometry helpers ───────────────────
def cam_fov(profile):
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().intrinsics
    hfov = m.degrees(2 * m.atan(DEPTH_W / (2 * intr.fx)))
    vfov = m.degrees(2 * m.atan(DEPTH_H / (2 * intr.fy)))
    return hfov, vfov

def set_obst_params(profile):
    global angle_offset, increment_f
    hfov, _ = cam_fov(profile)
    angle_offset = -(hfov / 2)
    increment_f  = hfov / DIST_LEN
    pr(f"INFO: HFOV {hfov:.2f}°, increment {increment_f:.2f}°")

def update_distances(depth_mat: np.ndarray, line_y: int) -> None:
    """Fill global 'distances' from depth image row band."""
    step = depth_mat.shape[1] / DIST_LEN
    upper = min(line_y + OBST_LINE_PIXELS // 2, depth_mat.shape[0] - 1)
    lower = max(line_y - OBST_LINE_PIXELS // 2, 0)
    slice_rows = depth_mat[int(lower):int(upper)]
    for i in range(DIST_LEN):
        d_m = np.min(slice_rows[:, int(i * step)]) * depth_scale
        distances[i] = MAX_CM + 1
        if DEPTH_RANGE_M[0] < d_m < DEPTH_RANGE_M[1]:
            distances[i] = int(d_m * 100)

def send_obstacle_distance(conn: mavutil.mavfile) -> None:
    global last_obst_us
    if current_us == last_obst_us:
        return
    last_obst_us = current_us
    conn.mav.obstacle_distance_send(
        current_us, 0, distances,
        0, MIN_CM, MAX_CM, increment_f, angle_offset, 12)

# ───────────────────── RTSP helper classes ───────────────────────
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.frames = 0
        self.dur = Gst.SECOND // FPS
        self.launch_string = (
            f'appsrc name=src is-live=true block=true format=GST_FORMAT_TIME '
            f'caps=video/x-raw,format=BGR,width={COLOR_W},height={COLOR_H},framerate={FPS}/1 '
            '! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency '
            '! rtph264pay config-interval=1 name=pay0 pt=96'
        )

    def on_need(self, src, _):
        if rtsp_frame is None:
            return
        data = rtsp_frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.duration = self.dur
        ts = self.frames * self.dur
        buf.pts = buf.dts = buf.offset = ts
        self.frames += 1
        src.emit('push-buffer', buf)

    def do_create_element(self, _):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, media):
        self.frames = 0
        media.get_element().get_child_by_name('src').connect('need-data', self.on_need)

def start_rtsp():
    Gst.init(None)
    srv = GstRtspServer.RTSPServer()
    factory = SensorFactory(); factory.set_shared(True)
    srv.get_mount_points().add_factory(RTSP_PATH, factory)
    srv.attach(None)
    ip = socket.gethostbyname(socket.gethostname())
    pr(f"INFO: RTSP stream at rtsp://{ip}:{RTSP_PORT}{RTSP_PATH}")
    threading.Thread(target=GLib.MainLoop().run, daemon=True).start()

# ───────────────────── IMU thread (optional) ─────────────────────
def imu_thread(conn: mavutil.mavfile):
    imu_pipe = rs.pipeline()
    imu_cfg  = rs.config()
    imu_cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)
    imu_cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 63)
    imu_pipe.start(imu_cfg)
    try:
        while not stop_evt.is_set():
            frames = imu_pipe.wait_for_frames(timeout_ms=1000)
            accel  = frames.first_or_default(rs.stream.accel)
            gyro   = frames.first_or_default(rs.stream.gyro)
            if accel and gyro:
                ax, ay, az, gx, gy, gz = imu_scale(
                    accel.as_motion_frame().get_motion_data(),
                    gyro .as_motion_frame().get_motion_data())
                conn.mav.raw_imu_send(now_us(), ax, ay, az, gx, gy, gz, 0, 0, 0)
    finally:
        imu_pipe.stop()

# ──────────────────── main() ─────────────────────────────────────
def main() -> None:
    # CLI
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument('--connect',  default=DEFAULT_CONN)
    ap.add_argument('--baudrate', type=int, default=DEFAULT_BAUD)
    ap.add_argument('--obst_hz',  type=float, default=DEFAULT_OBST_HZ)
    ap.add_argument('--debug',    action='store_true')
    ap.add_argument('--rtsp',     default=RTSP_DEFAULT, action=argparse.BooleanOptionalAction)
    ap.add_argument('--imu_enable', default=True, action=argparse.BooleanOptionalAction)
    args = ap.parse_args()
    if args.obst_hz <= 0:
        sys.exit("ERROR: --obst_hz must be >0")

    # MAVLink connection
    pr("INFO: Connecting MAVLink…")
    conn = mavutil.mavlink_connection(args.connect, baud=args.baudrate,
                                      autoreconnect=True,
                                      source_system=1, source_component=93,
                                      force_connected=True)
    threading.Thread(target=mavlink_loop, args=(conn, {}), daemon=True).start()

    # RealSense pipeline
    pr("INFO: Starting RealSense D4xx…")
    global pipe, depth_scale
    pipe = rs.pipeline()
    rscfg = rs.config()
    rscfg.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, FPS)
    if args.rtsp:
        rscfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, FPS)
    profile = pipe.start(rscfg)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    set_obst_params(profile)
    pr(f"INFO: Depth scale {depth_scale}")

    # Optional RTSP
    if args.rtsp:
        start_rtsp()

    # Optional IMU
    if args.imu_enable:
        threading.Thread(target=imu_thread, args=(conn,), daemon=True).start()

    # Scheduler for obstacle_distance
    sched = BackgroundScheduler()
    sched.add_job(lambda: send_obstacle_distance(conn), 'interval',
                  seconds=1/args.obst_hz)
    sched.start()

    # Graceful shutdown signals
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_evt.set())

    last_fps = time.time()
    try:
        while not stop_evt.is_set():
            frames = pipe.wait_for_frames(timeout_ms=2000)
            depth  = frames.get_depth_frame()
            if not depth:
                continue

            global current_us
            current_us = now_us()

            depth_mat = np.asanyarray(depth.get_data())
            y_line    = int(DEPTH_H * OBST_LINE_RATIO)
            update_distances(depth_mat, y_line)

            if args.rtsp:
                color = frames.get_color_frame()
                if color:
                    global rtsp_frame
                    rtsp_frame = np.asanyarray(color.get_data())

            if args.debug:
                fps = 1/(time.time() - last_fps); last_fps = time.time()
                pr(f"DEBUG: {fps:.1f} fps | mid {distances[DIST_LEN//2]} cm")
    finally:
        stop_evt.set()
        sched.shutdown(wait=False)
        try: pipe.stop()
        except Exception: pass
        pr("INFO: d4xx bridge closed.")

if __name__ == "__main__":
    main()
