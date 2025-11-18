#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

services=(
  mavproxy.service
  uplink.service
)

for service in "${services[@]}"; do
  if [[ ! -f "${service}" ]]; then
    echo "Service file ${service} not found in $(pwd)." >&2
    exit 1
  fi
  install -m 644 "${service}" "/etc/systemd/system/${service}"
done

systemctl daemon-reload
for service in "${services[@]}"; do
  systemctl enable "${service}"
done

echo "Services installed and enabled: ${services[*]}"
