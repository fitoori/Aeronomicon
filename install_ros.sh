#!/usr/bin/env bash

set -euo pipefail

log() {
  echo "[install_ros] $*"
}

require_root() {
  if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    log "Please run this script as root."
    exit 1
  fi
}

load_os_release() {
  if [[ ! -r /etc/os-release ]]; then
    log "Unable to detect operating system (missing /etc/os-release)."
    exit 1
  fi
  # shellcheck disable=SC1091
  source /etc/os-release
}

select_ros_distro() {
  case "${ID:-}:${ID_LIKE:-}" in
    ubuntu:*)
      PLATFORM="ubuntu"
      case "${UBUNTU_CODENAME:-}" in
        noble)
          DEFAULT_ROS_DISTRO="jazzy"
          ;;
        jammy)
          DEFAULT_ROS_DISTRO="humble"
          ;;
        *)
          log "Unsupported Ubuntu release: ${UBUNTU_CODENAME:-unknown}. Supported: Ubuntu 22.04 (jammy) or 24.04 (noble)."
          exit 1
          ;;
      esac
      ;;
    raspbian:*|raspios:*|*:raspbian*|*:raspios*)
      PLATFORM="raspbian"
      case "${VERSION_CODENAME:-}" in
        bookworm)
          DEFAULT_ROS_DISTRO="jazzy"
          ;;
        bullseye)
          DEFAULT_ROS_DISTRO="humble"
          ;;
        *)
          log "Unsupported Raspberry Pi OS release: ${VERSION_CODENAME:-unknown}. Supported: bullseye or bookworm."
          exit 1
          ;;
      esac
      ;;
    *)
      log "Unsupported distribution ID: ${ID:-unknown}. Only Ubuntu or Raspberry Pi OS are supported."
      exit 1
      ;;
  esac

  ROS_DISTRO=${ROS_DISTRO:-${DEFAULT_ROS_DISTRO}}
}

detect_headless() {
  HEADLESS_INSTALL=0
  if [[ "${HEADLESS:-}" =~ ^(1|true|yes)$ ]]; then
    HEADLESS_INSTALL=1
    log "Headless installation requested via HEADLESS environment variable."
    return
  fi

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    HEADLESS_INSTALL=1
    log "Headless environment detected (no display). Installing CLI-friendly ROS variant."
  fi
}

ensure_base_dependencies() {
  log "Ensuring required repositories and tools are present."
  apt-get update
  apt-get install -y curl gnupg lsb-release ca-certificates software-properties-common
  if [[ "${PLATFORM}" == "ubuntu" ]]; then
    add-apt-repository -y universe
  fi
}

configure_ros_repository() {
  if [[ "${PLATFORM}" != "ubuntu" ]]; then
    return
  fi

  local keyring_dir=/etc/apt/keyrings
  local keyring_file=${keyring_dir}/ros-archive-keyring.gpg
  local repo_file=/etc/apt/sources.list.d/ros2.list
  local arch
  arch=$(dpkg --print-architecture)

  install -d -m 0755 "${keyring_dir}"

  log "Adding ROS 2 GPG key."
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o "${keyring_file}" >/dev/null
  chmod a+r "${keyring_file}"

  log "Configuring ROS 2 apt repository."
  local repo_line="deb [arch=${arch} signed-by=${keyring_file}] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main"
  if [[ ! -f "${repo_file}" ]] || ! grep -Fxq "${repo_line}" "${repo_file}"; then
    echo "${repo_line}" > "${repo_file}"
  fi
}

refresh_package_index() {
  log "Updating apt package index."
  apt-get update
}

install_ros_packages() {
  if [[ "${PLATFORM}" != "ubuntu" ]]; then
    return
  fi

  local meta_package="ros-${ROS_DISTRO}-desktop"
  if [[ ${HEADLESS_INSTALL} -eq 1 ]]; then
    meta_package="ros-${ROS_DISTRO}-ros-base"
  fi

  log "Installing ROS 2 (${meta_package}) and build tools (idempotent upgrade)."
  apt-get install -y "${meta_package}" python3-rosdep python3-colcon-common-extensions build-essential git python3-vcstool
}

initialize_rosdep() {
  log "Initializing rosdep."
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
  fi
  rosdep update
}

configure_environment() {
  local install_prefix
  if [[ "${PLATFORM}" == "ubuntu" ]]; then
    local workspace_prefix="${ROS_WS}/install"
    local distro_prefix="/opt/ros/${ROS_DISTRO}"
    if [[ -f "${workspace_prefix}/setup.bash" ]]; then
      install_prefix="${workspace_prefix}"
    else
      install_prefix="${distro_prefix}"
    fi
  else
    install_prefix="${ROS_WS}/install"
  fi

  local ros_profile="/etc/profile.d/ros-${ROS_DISTRO}.sh"
  local source_line="source ${install_prefix}/setup.bash"

  log "Ensuring shell automatically sources ROS environment from ${install_prefix}."
  if [[ ! -f "${ros_profile}" ]]; then
    echo "${source_line}" > "${ros_profile}"
  fi
  if ! grep -Fxq "${source_line}" /root/.bashrc; then
    echo "${source_line}" >> /root/.bashrc
  fi
  if [[ -f "${install_prefix}/setup.bash" ]]; then
    # Source immediately for subsequent commands in this script.
    # shellcheck disable=SC1090
    source "${install_prefix}/setup.bash"
  fi
}

prepare_workspace() {
  if [[ "${PLATFORM}" == "ubuntu" ]]; then
    ROS_WS="/root/ros2_ws"
    log "Creating ROS 2 workspace at ${ROS_WS}."
    mkdir -p "${ROS_WS}/src"

    log "Installing Intel RealSense ROS packages."
    if [[ ! -d "${ROS_WS}/src/realsense-ros" ]]; then
      git clone https://github.com/IntelRealSense/realsense-ros.git "${ROS_WS}/src/realsense-ros"
    else
      git -C "${ROS_WS}/src/realsense-ros" fetch --all --prune
      git -C "${ROS_WS}/src/realsense-ros" checkout --quiet main || true
      git -C "${ROS_WS}/src/realsense-ros" pull --ff-only || true
    fi
  else
    ROS_WS="/root/ros2_${ROS_DISTRO}_src"
    log "Creating ROS 2 source workspace at ${ROS_WS}."
    mkdir -p "${ROS_WS}/src"
    local repos_file="/tmp/ros2_${ROS_DISTRO}.repos"
    local repo_url="https://raw.githubusercontent.com/ros2/ros2/${ROS_DISTRO}/ros2.repos"
    log "Fetching ROS 2 repos file from ${repo_url}."
    curl -sSL "${repo_url}" -o "${repos_file}"
    log "Importing ROS 2 sources (idempotent)."
    vcs import --input "${repos_file}" --force "${ROS_WS}/src"
    log "Updating existing ROS 2 sources."
    vcs pull "${ROS_WS}/src" || true
  fi
}

install_workspace_dependencies() {
  log "Installing workspace dependencies with rosdep."
  if [[ "${PLATFORM}" == "ubuntu" ]]; then
    rosdep install --from-paths "${ROS_WS}/src" --ignore-src --rosdistro "${ROS_DISTRO}" -y
  else
    rosdep install --from-paths "${ROS_WS}/src" --ignore-src --rosdistro "${ROS_DISTRO}" -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
  fi
}

build_workspace() {
  cd "${ROS_WS}"
  if [[ "${PLATFORM}" == "ubuntu" ]]; then
    log "Building workspace packages (realsense-ros)."
    colcon build --symlink-install --packages-select realsense_ros
  else
    if [[ ${HEADLESS_INSTALL} -eq 1 ]]; then
      log "Building ROS 2 from source (ros_base target)."
      colcon build --merge-install --symlink-install --packages-up-to ros_base
    else
      log "Building ROS 2 from source (desktop)."
      colcon build --merge-install --symlink-install
    fi
  fi
}

raspbian_build_dependencies() {
  log "Installing Raspberry Pi OS build dependencies."
  apt-get install -y build-essential cmake git python3-rosdep python3-colcon-common-extensions python3-vcstool python3-pip \
    python3-flake8 python3-pytest-cov python3-numpy python3-empy python3-setuptools python3-venv libbullet-dev libasio-dev \
    libtinyxml2-dev libcunit1-dev wget
}

main() {
  require_root
  load_os_release
  select_ros_distro
  detect_headless
  log "Preparing to install ROS 2 ${ROS_DISTRO} on ${PRETTY_NAME:-${ID}}."
  ensure_base_dependencies
  configure_ros_repository
  refresh_package_index

  if [[ "${PLATFORM}" == "raspbian" ]]; then
    raspbian_build_dependencies
  fi

  install_ros_packages
  initialize_rosdep
  prepare_workspace
  install_workspace_dependencies
  build_workspace
  configure_environment
  log "ROS 2 ${ROS_DISTRO} installation and workspace build complete."
}

main "$@"
