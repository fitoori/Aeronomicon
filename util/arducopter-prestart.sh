#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SIGNAL_SCRIPT="${SCRIPT_DIR}/lte-signal-strength.sh"
SENSOR_SCRIPT="${SCRIPT_DIR}/sensor-readings.sh"
DISCORD_SCRIPT="/home/pi/Aeronomicon/util/discord.py"
DISCORD_WEBHOOK="https://discord.com/api/webhooks/1454669304749621282/8LQlnXje9YyXtImmy7eDvARFAg7tYKqf16vuLiJPkQHW4OFXzJLAPSje7vVAi43jHqop"

log() {
    printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$*"
}

signal_percent="NaN"
if [[ -x "$SIGNAL_SCRIPT" ]]; then
    if signal_output=$("$SIGNAL_SCRIPT" 2>/dev/null); then
        parsed_percent=$(echo "$signal_output" | awk -F'Signal Strength: ' '{print $2}' | sed -n 's/%.*//p')
        if [[ -n "$parsed_percent" ]]; then
            signal_percent="$parsed_percent"
        fi
    else
        log "LTE signal script failed; reporting NaN%."
    fi
else
    log "LTE signal script not executable at $SIGNAL_SCRIPT; reporting NaN%."
fi

sensor_data="sensor data unavailable"
if [[ -x "$SENSOR_SCRIPT" ]]; then
    if sensor_output=$("$SENSOR_SCRIPT" 2>/dev/null); then
        sensor_output=$(echo "$sensor_output" | sed '/^\s*$/d')
        if [[ -n "$sensor_output" ]]; then
            sensor_data=$(echo "$sensor_output" | tr '\n' '; ' | sed 's/[; ]*$//')
        fi
    else
        log "Sensor script failed; reporting unavailable sensor data."
    fi
else
    log "Sensor script not executable at $SENSOR_SCRIPT; reporting unavailable sensor data."
fi

message="insert data here"

if [[ ! -x "$DISCORD_SCRIPT" ]]; then
    log "Discord script not executable at $DISCORD_SCRIPT; unable to send message."
    exit 1
fi

/usr/bin/python3 "$DISCORD_SCRIPT" --webhook "$DISCORD_WEBHOOK" "$message"
log "Discord notification sent."
