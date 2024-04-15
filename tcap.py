#############################################################
##     Image Capture Utility for Intel RealSense T265      ##
#############################################################

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# Specify the directory to save the images
save_directory = "/home/pi/Pictures/"

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.fisheye, 1)  # Left camera
config.enable_stream(rs.stream.fisheye, 2)  # Right camera

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()

        # Get the left and right camera frames
        left_frame = frames.get_fisheye_frame(1)
        right_frame = frames.get_fisheye_frame(2)

        if not left_frame or not right_frame:
            continue

        # Convert the frames to numpy arrays
        left_image = np.asanyarray(left_frame.get_data())
        right_image = np.asanyarray(right_frame.get_data())

        # Stitch the images horizontally
        stitched_image = np.hstack((left_image, right_image))

        # Get the current date and time
        current_datetime = datetime.now()
        timestamp = current_datetime.strftime("%d-%m-%Y_%H:%M:%S")

        # Construct the file name
        file_name = f"T265_{timestamp}.jpg"

        # Save the stitched image to the specified directory
        save_path = os.path.join(save_directory, file_name)
        cv2.imwrite(save_path, stitched_image)

        print(f"Image saved: {save_path}")

        # Break the loop after capturing one image
        break

finally:
    # Stop the pipeline
    pipeline.stop()
