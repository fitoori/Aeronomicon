#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: install-services.sh [--install|--update]

Installs or updates Aeronomicon systemd services.

  --install  Install service unit files and enable them (default if no flag).
  --update   Reinstall service unit files and restart enabled services.
  -h, --help Show this help text.
USAGE
}

mode="install"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install)
      mode="install"
      shift
      ;;
    --update)
      mode="update"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
service_dir="${script_dir}"
uplink_available=false
uplink_was_active=""
uplink_was_enabled=""
wwan_present=false
wwan_was_up=false

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

detect_uplink_service() {
  if systemctl status uplink.service >/dev/null 2>&1; then
    uplink_available=true
  fi
}

record_uplink_state() {
  if [[ "${uplink_available}" != true ]]; then
    return
  fi
  uplink_was_active="$(systemctl is-active uplink.service || true)"
  uplink_was_enabled="$(systemctl is-enabled uplink.service || true)"
}

restore_uplink_state() {
  if [[ "${uplink_available}" != true ]]; then
    return
  fi
  case "${uplink_was_enabled}" in
    enabled)
      systemctl enable uplink.service
      ;;
    disabled)
      systemctl disable uplink.service >/dev/null 2>&1 || true
      ;;
    *)
      ;;
  esac

  if [[ "${uplink_was_active}" == "active" ]]; then
    systemctl start uplink.service
  else
    systemctl stop uplink.service >/dev/null 2>&1 || true
  fi
}

record_wwan_state() {
  if ip link show wwan0 >/dev/null 2>&1; then
    wwan_present=true
    if ip -o link show wwan0 | awk '{for (i=1;i<=NF;i++) if ($i=="state"){print $(i+1); exit}}' | grep -q "UP"; then
      wwan_was_up=true
    fi
  fi
}

restore_wwan_state() {
  if [[ "${wwan_present}" != true ]]; then
    return
  fi
  if [[ "${wwan_was_up}" == true ]]; then
    ip link set wwan0 up
  else
    ip link set wwan0 down >/dev/null 2>&1 || true
  fi
}

cleanup() {
  restore_wwan_state
  restore_uplink_state
}

check_non_wwan_internet() {
  local route_dev
  route_dev="$(ip -o route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev"){print $(i+1); exit}}')"
  if [[ -z "${route_dev}" ]]; then
    echo "Unable to determine a route to the internet." >&2
    exit 1
  fi
  if [[ "${route_dev}" == "wwan0" ]]; then
    echo "Default route uses wwan0. A non-wwan0 connection is required before installing services." >&2
    exit 1
  fi
  if ! ping -I "${route_dev}" -c 1 -W 3 1.1.1.1 >/dev/null 2>&1; then
    echo "No internet connectivity detected on ${route_dev}." >&2
    exit 1
  fi
}

prepare_network_for_install() {
  if [[ "${mode}" == "update" ]]; then
    return
  fi
  check_non_wwan_internet
  detect_uplink_service
  record_uplink_state
  record_wwan_state
  trap '' HUP
  trap cleanup EXIT
  if [[ "${uplink_available}" == true ]]; then
    systemctl stop uplink.service
    if [[ "${uplink_was_enabled}" == "enabled" ]]; then
      systemctl disable uplink.service
    fi
  fi
  if [[ "${wwan_present}" == true ]]; then
    ip link set wwan0 down
  fi
}

arming_lock_scripts=(
  "${repo_root}/uplink.sh"
  "${repo_root}/legacy/ONICS2.py"
  "${repo_root}/legacy/onlcs-lite.py"
)

ensure_executable_scripts() {
  for script in "${arming_lock_scripts[@]}"; do
    if [[ ! -f "${script}" ]]; then
      echo "Expected script not found: ${script}" >&2
      exit 1
    fi
    chmod +x "${script}"
  done
}

services=(
  arducopter.service
  mavproxy.service
  uplink.service
)

prepare_network_for_install
ensure_executable_scripts

for service in "${services[@]}"; do
  service_path="${service_dir}/${service}"
  if [[ ! -f "${service_path}" ]]; then
    echo "Service file ${service} not found at ${service_path}." >&2
    exit 1
  fi
  install -m 644 "${service_path}" "/etc/systemd/system/${service}"
done

systemctl daemon-reload

if [[ "${mode}" == "install" ]]; then
  for service in "${services[@]}"; do
    systemctl enable "${service}"
  done
  echo "Services installed and enabled: ${services[*]}"
else
  for service in "${services[@]}"; do
    if systemctl is-enabled "${service}" >/dev/null 2>&1; then
      systemctl try-restart "${service}" || true
    fi
  done
  echo "Services updated (daemon reloaded, enabled services restarted): ${services[*]}"
fi
