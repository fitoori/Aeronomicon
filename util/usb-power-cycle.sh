#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: usb-power-cycle.sh [--delay SECONDS] [--hub usbX ...]

Power-cycles USB root hubs by toggling /sys/bus/usb/devices/usbX/authorized.
Requires root (or write access to sysfs).

Options:
  --delay SECONDS  Seconds to wait between power-off and power-on (default: 2)
  --hub usbX       One or more root hubs to cycle (e.g. usb1 usb2). If omitted,
                   all detected root hubs are cycled.
  -h, --help       Show this help text.
USAGE
}

require_root() {
  if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    echo "ERROR: must be run as root to write to /sys/bus/usb/devices." >&2
    exit 1
  fi
}

main() {
  local delay=2
  local -a hubs=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --delay)
        delay="$2"
        shift 2
        ;;
      --hub)
        hubs+=("$2")
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "ERROR: unknown argument: $1" >&2
        usage
        exit 2
        ;;
    esac
  done

  require_root

  if [[ ${#hubs[@]} -eq 0 ]]; then
    mapfile -t hubs < <(ls -1 /sys/bus/usb/devices/usb* 2>/dev/null | xargs -n1 basename)
  fi

  if [[ ${#hubs[@]} -eq 0 ]]; then
    echo "ERROR: no USB root hubs found in /sys/bus/usb/devices." >&2
    exit 1
  fi

  for hub in "${hubs[@]}"; do
    local auth="/sys/bus/usb/devices/${hub}/authorized"
    if [[ ! -w "$auth" ]]; then
      echo "WARN: skipping ${hub}; no writable ${auth}" >&2
      continue
    fi

    echo "INFO: power off ${hub}"
    echo 0 >"$auth"
  done

  sleep "$delay"

  for hub in "${hubs[@]}"; do
    local auth="/sys/bus/usb/devices/${hub}/authorized"
    if [[ ! -w "$auth" ]]; then
      continue
    fi

    echo "INFO: power on ${hub}"
    echo 1 >"$auth"
  done
}

main "$@"
