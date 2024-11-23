#!/usr/bin/env python3

######################################################
##  librealsense D4xx to MAVLink                    ##
######################################################

# Requirements:
#   x86 based Companion Computer (for compatibility with Intel),
#   Ubuntu 18.04 or newer,
#   Python3
# Install required packages:
#   pip3 install pyrealsense2 numpy pyserial transformations pymavlink apscheduler opencv-python
#   sudo apt -y install python3-gst-1.0 gir1.2-gst-rtsp-server-1.0 gstreamer1.0-plugins-base \
#                       gstreamer1.0-plugins-ugly libx264-dev

# Import standard libraries
import sys
import os
import signal
import time
import argparse
import threading
import math as m
import socket
import logging

# Import third-party libraries
try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: pyrealsense2 module not found. Please install it using 'pip3 install pyrealsense2'")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("Error: numpy module not found. Please install it using 'pip3 install numpy'")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("Error: opencv-python module not found. Please install it using 'pip3 install opencv-python'")
    sys.exit(1)

try:
    from apscheduler.schedulers.background import BackgroundScheduler
except ImportError:
    print("Error: apscheduler module not found. Please install it using 'pip3 install apscheduler'")
    sys.exit(1)

try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink module not found. Please install it using 'pip3 install pymavlink'")
    sys.exit(1)

# For RTSP streaming
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstRtspServer', '1.0')
    from gi.repository import Gst, GstRtspServer, GLib
except ImportError:
    print("Error: GStreamer modules not found. Please install the required GStreamer packages.")
    sys.exit(1)

# Optionally import numba if installed
try:
    from numba import njit
    numba_available = True
except ImportError:
    numba_available = False

# Set MAVLink protocol to 2
os.environ["MAVLINK20"] = "1"

# Remove ROS Kinetic's Python2 path from sys.path if present
ros_python2_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_python2_path in sys.path:
    sys.path.remove(ros_python2_path)

# Optionally add Anaconda's site-packages to sys.path
anaconda_site_packages = os.path.expanduser('~/anaconda3/lib/python3.7/site-packages')
if os.path.exists(anaconda_site_packages):
    sys.path.append(anaconda_site_packages)

######################################################
##  Depth parameters - reconfigurable               ##
######################################################

# Sensor-specific parameter, for D435: https://www.intelrealsense.com/depth-camera-d435/
STREAM_TYPE  = [rs.stream.depth, rs.stream.color]
FORMAT       = [rs.format.z16, rs.format.bgr8]
DEPTH_WIDTH  = 640
DEPTH_HEIGHT = 480
COLOR_WIDTH  = 640
COLOR_HEIGHT = 480
FPS          = 30
DEPTH_RANGE_M = [0.1, 8.0]  # Replace with your sensor's specifics, in meters

obstacle_line_height_ratio = 0.18  # [0-1]: 0-Top, 1-Bottom
obstacle_line_thickness_pixel = 10  # [1-DEPTH_HEIGHT]

USE_PRESET_FILE = True
PRESET_FILE  = "../cfg/d4xx-default.json"  # Path to the preset file

RTSP_STREAMING_ENABLE = True
RTSP_PORT = "8554"
RTSP_MOUNT_POINT = "/d4xx"

# List of filters to be applied
filters = [
    {'enabled': True,  'name': "Decimation Filter",   'filter': rs.decimation_filter()},
    {'enabled': True,  'name': "Threshold Filter",    'filter': rs.threshold_filter()},
    {'enabled': True,  'name': "Depth to Disparity",  'filter': rs.disparity_transform(True)},
    {'enabled': True,  'name': "Spatial Filter",      'filter': rs.spatial_filter()},
    {'enabled': True,  'name': "Temporal Filter",     'filter': rs.temporal_filter()},
    {'enabled': False, 'name': "Hole Filling Filter", 'filter': rs.hole_filling_filter()},
    {'enabled': True,  'name': "Disparity to Depth",  'filter': rs.disparity_transform(False)},
]

# Set threshold filter options
threshold_min_m = DEPTH_RANGE_M[0]
threshold_max_m = DEPTH_RANGE_M[1]
for f in filters:
    if f['enabled'] and f['name'] == "Threshold Filter":
        f['filter'].set_option(rs.option.min_distance, threshold_min_m)
        f['filter'].set_option(rs.option.max_distance, threshold_max_m)

######################################################
##  ArduPilot-related parameters - reconfigurable   ##
######################################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600

# Use this to rotate all processed data
camera_facing_angle_degree = 0

# Enable/disable each message/function individually
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz_default = 15.0

# Lock for thread synchronization
lock = threading.Lock()

mavlink_thread_should_exit = False

######################################################
##  Global variables                                ##
######################################################

# Camera-related variables
pipe = None
depth_scale = 0
colorizer = rs.colorizer()
depth_hfov_deg = None
depth_vfov_deg = None

# The name of the display window
display_name  = 'Input/output depth'
rtsp_streaming_img = None

# Data variables
vehicle_pitch_rad = None
data = None
current_time_us = 0
last_obstacle_distance_sent_us = 0  # value of current_time_us when obstacle_distance last sent

# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
min_depth_cm = int(DEPTH_RANGE_M[0] * 100)  # In cm
max_depth_cm = int(DEPTH_RANGE_M[1] * 100)  # In cm
distances_array_length = 72
angle_offset = None
increment_f  = None
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_depth_cm + 1)

######################################################
##  Parsing user inputs                             ##
######################################################

parser = argparse.ArgumentParser(description='ArduPilot + RealSense D4xx Integration')
parser.add_argument('--connect',
                    help="Vehicle connection target string",
                    default=connection_string_default)
parser.add_argument('--baudrate', type=int,
                    help="Vehicle connection baudrate",
                    default=connection_baudrate_default)
parser.add_argument('--obstacle_distance_msg_hz', type=float,
                    help="Update frequency for OBSTACLE_DISTANCE message",
                    default=obstacle_distance_msg_hz_default)
parser.add_argument('--debug_enable', action='store_true',
                    help="Enable debugging information")
parser.add_argument('--camera_name', type=str,
                    help="Camera name to be connected to. If not specified, any valid camera will be connected to randomly. For example: 'D435I'")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
obstacle_distance_msg_hz = args.obstacle_distance_msg_hz
debug_enable = args.debug_enable
camera_name = args.camera_name

# Configure logging
if debug_enable:
    logging_level = logging.DEBUG
else:
    logging_level = logging.INFO

logging.basicConfig(level=logging_level, format='%(asctime)s [%(levelname)s] %(message)s')

logging.info(f"Using connection_string: {connection_string}")
logging.info(f"Using connection_baudrate: {connection_baudrate}")
logging.info(f"Using obstacle_distance_msg_hz: {obstacle_distance_msg_hz}")

# Print which filters are being applied
for f in filters:
    if f['enabled']:
        logging.info(f"Applying filter: {f['name']}")
    else:
        logging.info(f"Not applying filter: {f['name']}")

if debug_enable:
    logging.info("Debugging option enabled")
    cv2.namedWindow(display_name, cv2.WINDOW_AUTOSIZE)
else:
    logging.info("Debugging option disabled")

######################################################
##  Functions - MAVLink                             ##
######################################################

def mavlink_loop(conn, callbacks):
    '''Main routine for MAVLink communication thread.'''
    while not mavlink_thread_should_exit:
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0,
                0,
                0)
            m = conn.recv_match(type=list(callbacks.keys()), timeout=1, blocking=True)
            if m is None:
                continue
            callbacks[m.get_type()](m)
        except Exception as e:
            logging.error(f"MAVLink error: {e}")
            time.sleep(1)

def send_obstacle_distance_message():
    global current_time_us, distances, camera_facing_angle_degree, last_obstacle_distance_sent_us
    if current_time_us == last_obstacle_distance_sent_us:
        return
    last_obstacle_distance_sent_us = current_time_us
    if angle_offset is None or increment_f is None:
        logging.error("Obstacle distance parameters not set. Call set_obstacle_distance_params() before continuing.")
        return
    try:
        conn.mav.obstacle_distance_send(
            current_time_us,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_UNKNOWN,
            distances,
            increment_f,
            min_depth_cm,
            max_depth_cm,
            increment_f,
            angle_offset,
            mavutil.mavlink.MAV_FRAME_BODY_FRD)
    except Exception as e:
        logging.error(f"Error sending obstacle_distance message: {e}")

def send_distance_sensor_message():
    global distances
    curr_dist = int(np.mean(distances[33:38]))
    try:
        conn.mav.distance_sensor_send(
            0,
            min_depth_cm,
            max_depth_cm,
            curr_dist,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND,
            0,
            int(camera_facing_angle_degree / 45),
            0)
    except Exception as e:
        logging.error(f"Error sending distance_sensor message: {e}")

def send_msg_to_gcs(text_to_be_sent):
    text_msg = 'D4xx: ' + text_to_be_sent
    try:
        conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
        logging.info(text_to_be_sent)
    except Exception as e:
        logging.error(f"Error sending message to GCS: {e}")

def att_msg_callback(msg):
    global vehicle_pitch_rad
    vehicle_pitch_rad = msg.pitch
    if debug_enable:
        logging.debug(f"Received ATTITUDE message, pitch: {m.degrees(vehicle_pitch_rad):.2f} degrees")

######################################################
##  Functions - D4xx cameras                        ##
######################################################

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C"]

def find_device_that_supports_advanced_mode():
    global device_id
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            name = dev.get_info(rs.camera_info.name)
            if not camera_name or (camera_name.lower() in name.lower()):
                logging.info(f"Found device supporting advanced mode: {name}")
                device_id = dev.get_info(rs.camera_info.serial_number)
                return dev
    raise Exception("No device that supports advanced mode was found")

def realsense_enable_advanced_mode(advnc_mode):
    if not advnc_mode.is_enabled():
        logging.info("Enabling advanced mode...")
        advnc_mode.toggle_advanced_mode(True)
        time.sleep(5)
        dev = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        if advnc_mode.is_enabled():
            logging.info("Advanced mode enabled.")
        else:
            raise Exception("Failed to enable advanced mode.")

def realsense_load_settings_file(advnc_mode, setting_file):
    if not os.path.isfile(setting_file):
        logging.warning(f"Settings file '{setting_file}' not found. Proceeding with default settings.")
        return
    with open(setting_file, 'r') as file:
        json_text = file.read().strip()
    advnc_mode.load_json(json_text)
    logging.info(f"Loaded settings from {setting_file}")

def realsense_connect():
    global pipe, depth_scale
    try:
        pipe = rs.pipeline()
        config = rs.config()
        if device_id:
            config.enable_device(device_id)
        config.enable_stream(STREAM_TYPE[0], DEPTH_WIDTH, DEPTH_HEIGHT, FORMAT[0], FPS)
        if RTSP_STREAMING_ENABLE:
            config.enable_stream(STREAM_TYPE[1], COLOR_WIDTH, COLOR_HEIGHT, FORMAT[1], FPS)
        profile = pipe.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        logging.info(f"Depth scale is: {depth_scale}")
    except Exception as e:
        logging.error(f"Error connecting to RealSense camera: {e}")
        sys.exit(1)

def realsense_configure_setting(setting_file):
    try:
        device = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(device)
        realsense_enable_advanced_mode(advnc_mode)
        realsense_load_settings_file(advnc_mode, setting_file)
    except Exception as e:
        logging.error(f"Error configuring RealSense settings: {e}")
        sys.exit(1)

def set_obstacle_distance_params():
    global angle_offset, camera_facing_angle_degree, increment_f, depth_scale, depth_hfov_deg, depth_vfov_deg, obstacle_line_height_ratio, obstacle_line_thickness_pixel

    try:
        profiles = pipe.get_active_profile()
        depth_intrinsics = profiles.get_stream(STREAM_TYPE[0]).as_video_stream_profile().intrinsics
        logging.info(f"Depth camera intrinsics: {depth_intrinsics}")

        depth_hfov_deg = m.degrees(2 * m.atan(DEPTH_WIDTH / (2 * depth_intrinsics.fx)))
        depth_vfov_deg = m.degrees(2 * m.atan(DEPTH_HEIGHT / (2 * depth_intrinsics.fy)))
        logging.info(f"Depth camera HFOV: {depth_hfov_deg:.2f} degrees")
        logging.info(f"Depth camera VFOV: {depth_vfov_deg:.2f} degrees")

        angle_offset = camera_facing_angle_degree - (depth_hfov_deg / 2)
        increment_f = depth_hfov_deg / distances_array_length
        logging.info(f"OBSTACLE_DISTANCE angle_offset: {angle_offset:.2f}")
        logging.info(f"OBSTACLE_DISTANCE increment_f: {increment_f:.2f}")
        logging.info(f"OBSTACLE_DISTANCE coverage: from {angle_offset:.2f} to {angle_offset + increment_f * distances_array_length:.2f} degrees")

        if not 0 <= obstacle_line_height_ratio <= 1:
            raise ValueError("obstacle_line_height_ratio must be between 0 and 1.")
        if not 1 <= obstacle_line_thickness_pixel <= DEPTH_HEIGHT:
            raise ValueError(f"obstacle_line_thickness_pixel must be between 1 and {DEPTH_HEIGHT}.")
    except Exception as e:
        logging.error(f"Error setting obstacle distance parameters: {e}")
        sys.exit(1)

def find_obstacle_line_height():
    global vehicle_pitch_rad, depth_vfov_deg, DEPTH_HEIGHT

    obstacle_line_height = DEPTH_HEIGHT * obstacle_line_height_ratio

    if vehicle_pitch_rad is not None and depth_vfov_deg is not None:
        delta_height = m.sin(vehicle_pitch_rad / 2) / m.sin(m.radians(depth_vfov_deg) / 2) * DEPTH_HEIGHT
        obstacle_line_height += delta_height

    obstacle_line_height = max(0, min(obstacle_line_height, DEPTH_HEIGHT))

    return obstacle_line_height

if numba_available:
    @njit
    def distances_from_depth_image(obstacle_line_height, depth_mat, distances, min_depth_m, max_depth_m, obstacle_line_thickness_pixel):
        depth_img_width = depth_mat.shape[1]
        depth_img_height = depth_mat.shape[0]
        step = depth_img_width / distances_array_length

        for i in range(distances_array_length):
            center_pixel = obstacle_line_height
            upper_pixel = center_pixel + obstacle_line_thickness_pixel / 2
            lower_pixel = center_pixel - obstacle_line_thickness_pixel / 2

            upper_pixel = min(depth_img_height, max(1, upper_pixel))
            lower_pixel = min(depth_img_height - 1, max(0, lower_pixel))

            min_point_in_scan = np.min(depth_mat[int(lower_pixel):int(upper_pixel), int(i * step)])
            dist_m = min_point_in_scan * depth_scale

            distances[i] = 65535
            if min_depth_m < dist_m < max_depth_m:
                distances[i] = dist_m * 100
else:
    def distances_from_depth_image(obstacle_line_height, depth_mat, distances, min_depth_m, max_depth_m, obstacle_line_thickness_pixel):
        depth_img_width = depth_mat.shape[1]
        depth_img_height = depth_mat.shape[0]
        step = depth_img_width / distances_array_length

        for i in range(distances_array_length):
            center_pixel = obstacle_line_height
            upper_pixel = center_pixel + obstacle_line_thickness_pixel / 2
            lower_pixel = center_pixel - obstacle_line_thickness_pixel / 2

            upper_pixel = min(depth_img_height, max(1, upper_pixel))
            lower_pixel = min(depth_img_height - 1, max(0, lower_pixel))

            min_point_in_scan = np.min(depth_mat[int(lower_pixel):int(upper_pixel), int(i * step)])
            dist_m = min_point_in_scan * depth_scale

            distances[i] = 65535
            if min_depth_m < dist_m < max_depth_m:
                distances[i] = dist_m * 100

######################################################
##  Functions - RTSP Streaming                      ##
######################################################

class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.number_frames = 0
        self.fps = FPS
        self.duration = 1 / self.fps * Gst.SECOND
        self.launch_string = (
            f'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME '
            f'caps=video/x-raw,format=BGR,width={COLOR_WIDTH},height={COLOR_HEIGHT},framerate={self.fps}/1 '
            '! videoconvert ! video/x-raw,format=I420 '
            '! x264enc speed-preset=ultrafast tune=zerolatency '
            '! rtph264pay config-interval=1 name=pay0 pt=96'
        )

    def on_need_data(self, src, length):
        global rtsp_streaming_img
        frame = rtsp_streaming_img
        if frame is not None:
            data = frame.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = self.duration
            timestamp = self.number_frames * self.duration
            buf.pts = buf.dts = int(timestamp)
            buf.offset = timestamp
            self.number_frames += 1
            retval = src.emit('push-buffer', buf)
            if retval != Gst.FlowReturn.OK:
                logging.error(f"Error pushing buffer: {retval}")

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        factory = SensorFactory()
        factory.set_shared(True)
        self.get_mount_points().add_factory(RTSP_MOUNT_POINT, factory)
        self.attach(None)

def get_local_ip():
    local_ip_address = "127.0.0.1"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 1))
        local_ip_address = s.getsockname()[0]
    except Exception as e:
        logging.error(f"Error getting local IP: {e}")
        local_ip_address = socket.gethostbyname(socket.gethostname())
    return local_ip_address

######################################################
##  Main code starts here                           ##
######################################################

if __name__ == "__main__":
    try:
        logging.info(f"pyrealsense2 version: {rs.__version__}")
    except Exception:
        pass

    logging.info("Starting vehicle communications...")
    try:
        conn = mavutil.mavlink_connection(
            connection_string,
            autoreconnect=True,
            source_system=1,
            source_component=93,
            baud=connection_baudrate,
            force_connected=True,
        )
    except Exception as e:
        logging.error(f"Error connecting to vehicle: {e}")
        sys.exit(1)

    mavlink_callbacks = {
        'ATTITUDE': att_msg_callback,
    }
    mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
    mavlink_thread.daemon = True
    mavlink_thread.start()

    signal.signal(signal.SIGINT, lambda sig, frame: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda sig, frame: sys.exit(0))

    logging.info("Connecting to RealSense camera...")
    try:
        if USE_PRESET_FILE:
            realsense_configure_setting(PRESET_FILE)
        else:
            # Proceed with default settings
            device = find_device_that_supports_advanced_mode()
            advnc_mode = rs.rs400_advanced_mode(device)
            realsense_enable_advanced_mode(advnc_mode)
            logging.info("Using default RealSense settings.")
        realsense_connect()
    except Exception as e:
        logging.error(f"Error initializing RealSense camera: {e}")
        sys.exit(1)

    set_obstacle_distance_params()

    sched = BackgroundScheduler()
    if enable_msg_obstacle_distance:
        sched.add_job(send_obstacle_distance_message, 'interval', seconds=1/obstacle_distance_msg_hz)
        send_msg_to_gcs('Sending obstacle distance messages to FCU')
    elif enable_msg_distance_sensor:
        sched.add_job(send_distance_sensor_message, 'interval', seconds=1/obstacle_distance_msg_hz)
        send_msg_to_gcs('Sending distance sensor messages to FCU')
    else:
        send_msg_to_gcs('No messages to send. Check parameters to enable functionality.')
        pipe.stop()
        conn.close()
        sys.exit(1)

    glib_loop = None
    if RTSP_STREAMING_ENABLE:
        send_msg_to_gcs(f'RTSP streaming at rtsp://{get_local_ip()}:{RTSP_PORT}{RTSP_MOUNT_POINT}')
        Gst.init(None)
        server = GstServer()
        glib_loop = GLib.MainLoop()
        glib_thread = threading.Thread(target=glib_loop.run)
        glib_thread.daemon = True
        glib_thread.start()
    else:
        send_msg_to_gcs('RTSP streaming not enabled')

    sched.start()

    last_time = time.time()
    try:
        while True:
            frames = pipe.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame() if RTSP_STREAMING_ENABLE else None

            if not depth_frame:
                continue

            current_time_us = int(round(time.time() * 1e6))

            filtered_frame = depth_frame
            for f in filters:
                if f['enabled']:
                    filtered_frame = f['filter'].process(filtered_frame)

            depth_data = filtered_frame.as_frame().get_data()
            depth_mat = np.asanyarray(depth_data)

            obstacle_line_height = find_obstacle_line_height()
            distances_from_depth_image(obstacle_line_height, depth_mat, distances, DEPTH_RANGE_M[0], DEPTH_RANGE_M[1], obstacle_line_thickness_pixel)

            if RTSP_STREAMING_ENABLE and color_frame:
                rtsp_streaming_img = np.asanyarray(color_frame.get_data())

            if debug_enable:
                input_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                output_image = np.asanyarray(colorizer.colorize(filtered_frame).get_data())

                x1, y1 = 0, int(obstacle_line_height)
                x2, y2 = DEPTH_WIDTH, int(obstacle_line_height)
                line_thickness = int(obstacle_line_thickness_pixel)
                cv2.line(output_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=line_thickness)
                display_image = np.hstack((input_image, cv2.resize(output_image, (DEPTH_WIDTH, DEPTH_HEIGHT))))

                processing_speed = 1 / (time.time() - last_time)
                cv2.putText(display_image, f"{processing_speed:.2f} fps", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                cv2.imshow(display_name, display_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                logging.debug(f"Distances: {distances}")

                last_time = time.time()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Exiting...")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        if pipe:
            pipe.stop()
        if glib_loop:
            glib_loop.quit()
        if conn:
            conn.close()
        logging.info("Resources cleaned up. Exiting.")
        sys.exit(0)
