#!/bin/bash

set -e

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root"
  exit 1
fi

# Prompt for installation options
echo "ROS 2 Installation Options:"
echo "1. Build from source"
echo "2. Install pre-built binaries"
read -p "Enter your choice (1 or 2): " choice

# Add the ROS 2 apt repository
sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu bullseye main" > /etc/apt/sources.list.d/ros2.list'

# Set up the ROS 2 keys
apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update the package lists
apt update

# Install ROS 2 packages using aptitude
apt-get install -y aptitude

# Option 1: Build ROS 2 from source
if [ "$choice" -eq 1 ]; then
  # Install build tools
  aptitude install -y build-essential cmake git

  # Install additional dependencies for building from source
  aptitude install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool

  # Create the workspace
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws

  # Download the ROS 2 source code
  wget https://github.com/ros2/ros2/archive/foxy.zip
  unzip foxy.zip
  rm foxy.zip
  mv ros2-foxy/* src/

  # Resolve dependencies
  aptitude install -y python3-rosdep
  rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro foxy -y

  # Build ROS 2
  colcon build --symlink-install

  # Source the ROS 2 workspace
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc

# Option 2: Install pre-built binaries using aptitude
elif [ "$choice" -eq 2 ]; then
  aptitude install -y ros-foxy-desktop

  # Set up the ROS 2 environment
  source /opt/ros/foxy/setup.bash
fi

echo "ROS 2 Humble installation complete."

