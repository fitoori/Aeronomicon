#!/bin/bash

# Update package lists
sudo apt-get update

# Install required dependencies
sudo apt-get install -y aptitude
sudo aptitude install -y build-essential cmake git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev python3-dev python3-numpy python3-pip


# Install OpenCV
echo "Installing OpenCV..."
sudo aptitude install -y libopencv-dev python3-opencv

# Clone the RealSense GitHub repository
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Build and install the RealSense SDK with pyrealsense2
mkdir build && cd build
#cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.7m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.7
cmake -D CMAKE_BUILD_TYPE="Release"\
-D FORCE_LIBUVC=ON \
-D BUILD_PYTHON_BINDINGS=ON \
-D BUILD_EXAMPLES=ON \
-D PYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.7m.so \
-D PYTHON_INCLUDE_DIR=/usr/include/python3.7 ..

make && sudo make install &
sudo ldconfig

# Verify installation
echo "RealSense installation completed, cloning realsense_to_mavros repository..."

# Clone the vision_to_mavros repository
cd ~/
git clone https://github.com/thien94/vision_to_mavros.git

# Change directory to the cloned repository
cd vision_to_mavros

# Create and activate a Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies using pip3
pip3 install -r requirements.txt

# Deactivate the virtual environment
deactivate

cd ~/

pip3 install mavproxy
