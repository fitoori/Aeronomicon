#!/bin/bash
set -euo pipefail

# This script installs Intel RealSense SDK with Python bindings following the
# upstream procedure. It is idempotent and can repair broken installations by
# rebuilding and reinstalling librealsense when verification checks fail.

ROOT_DIR="${HOME}"
LIBREALSENSE_DIR="${ROOT_DIR}/librealsense"

log() {
  echo "[$(date -Iseconds)] $*"
}

ensure_dependencies() {
  sudo apt-get update
  sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-dev \
    python3-numpy \
    python3-pip

  sudo apt-get install -y libopencv-dev python3-opencv
}

ensure_repo() {
  if [ -d "${LIBREALSENSE_DIR}/.git" ]; then
    log "Updating existing librealsense repository"
    git -C "${LIBREALSENSE_DIR}" fetch --all --tags
    git -C "${LIBREALSENSE_DIR}" checkout master
    git -C "${LIBREALSENSE_DIR}" pull --ff-only
  else
    log "Cloning librealsense repository"
    git clone https://github.com/IntelRealSense/librealsense.git "${LIBREALSENSE_DIR}"
  fi
}

apply_udev_rules() {
  local rules_path="/etc/udev/rules.d/99-realsense-libusb.rules"

  if [ ! -f "${rules_path}" ]; then
    log "Applying udev rules for RealSense devices"
    sudo "${LIBREALSENSE_DIR}/scripts/setup_udev_rules.sh"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
  else
    log "Udev rules already present; skipping installation"
  fi
}

verify_realsense_install() {
  local python_ok=0
  local pkgconfig_ok=0

  if python3 - <<'PY' >/dev/null 2>&1
import pyrealsense2
PY
  then
    python_ok=1
  fi

  if pkg-config --exists realsense2; then
    pkgconfig_ok=1
  fi

  if [ "${python_ok}" -eq 1 ] && [ "${pkgconfig_ok}" -eq 1 ]; then
    return 0
  fi

  return 1
}

build_and_install() {
  log "Configuring librealsense build"
  rm -rf "${LIBREALSENSE_DIR}/build"
  cmake -S "${LIBREALSENSE_DIR}" -B "${LIBREALSENSE_DIR}/build" \
    -D CMAKE_BUILD_TYPE=Release \
    -D FORCE_LIBUVC=ON \
    -D BUILD_PYTHON_BINDINGS=ON \
    -D BUILD_EXAMPLES=ON \
    -D PYTHON_EXECUTABLE="$(command -v python3)"

  log "Building librealsense"
  cmake --build "${LIBREALSENSE_DIR}/build"

  log "Installing librealsense"
  sudo cmake --install "${LIBREALSENSE_DIR}/build"
  sudo ldconfig
}

main() {
  ensure_dependencies
  ensure_repo
  apply_udev_rules

  if verify_realsense_install; then
    log "Existing RealSense installation detected; no rebuild required"
  else
    log "RealSense installation missing or broken; rebuilding"
    build_and_install

    if verify_realsense_install; then
      log "RealSense installation verified"
    else
      log "Verification failed after rebuild" >&2
      exit 1
    fi
  fi
}

main "$@"
