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

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

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
  mavproxy.service
  uplink.service
)

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
