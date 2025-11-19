#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: install-services.sh [--update]

Installs or updates Aeronomicon systemd services.

  --update   Reinstall service unit files and restart enabled services.
  -h, --help Show this help text.
USAGE
}

mode="install"
while [[ $# -gt 0 ]]; do
  case "$1" in
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

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

services=(
  mavproxy.service
  uplink.service
)

for service in "${services[@]}"; do
  service_path="${script_dir}/${service}"
  if [[ ! -f "${service_path}" ]]; then
    echo "Service file ${service} not found in ${script_dir}." >&2
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
