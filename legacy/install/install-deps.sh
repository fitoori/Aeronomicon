#!/usr/bin/env bash
set -euo pipefail

PIP_FLAGS=()
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  PIP_FLAGS+=(--break-system-packages)
fi

sudo apt update -y
sudo apt-get full-upgrade -y
sudo apt install -y \
  cmake \
  python3 \
  python3-pip \
  python3-gst-1.0 \
  gir1.2-gst-rtsp-server-1.0 \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-ugly \
  libx264-dev \
  python3-opencv

pip3 install "${PIP_FLAGS[@]}" transformations pymavlink apscheduler pyserial numba opencv-python

cd ~
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake .
sudo make install
