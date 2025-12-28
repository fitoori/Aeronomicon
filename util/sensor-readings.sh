#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SENSOR_SCRIPT="${SCRIPT_DIR}/navio2-sensors.py"

if [[ ! -f "$SENSOR_SCRIPT" ]]; then
    echo "ERROR: Sensor script not found at ${SENSOR_SCRIPT}." >&2
    exit 1
fi

python3 "$SENSOR_SCRIPT" --once
