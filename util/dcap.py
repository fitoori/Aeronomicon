#############################################################
##     Image Capture Utility for Intel RealSense D4XX      ##
#############################################################

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# Set the default directory for saving images
save_dir = '/home/pi/Pictures/'

def capture_image():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color)

    # Start streaming
    pipeline.start(config)
    
    try:
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            print("No color frame captured.")
            return
        
        # Convert the color image to a numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Generate filename based on current date and time
        now = datetime.now()
        timestamp = now.strftime("%d-%m-%Y_%H-%M-%S")
        filename = f"D4XX_{timestamp}.jpg"
        filepath = os.path.join(save_dir, filename)
        
        # Save the image
        cv2.imwrite(filepath, cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
        
        print(f"Image saved: {filepath}")
    
    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    capture_image()
