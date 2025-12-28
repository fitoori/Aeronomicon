#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SIGNAL_SCRIPT="${SCRIPT_DIR}/lte-signal-strength.sh"
SENSOR_SCRIPT="${SCRIPT_DIR}/sensor-readings.sh"
DISCORD_SCRIPT="${SCRIPT_DIR}/discord/discord.py"
DISCORD_WEBHOOK="${DISCORD_WEBHOOK:-}"

log() {
    printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$*"
}

if [[ -z "$DISCORD_WEBHOOK" ]]; then
    log "DISCORD_WEBHOOK is not set; skipping Discord notification."
    exit 0
fi

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

message="I'm alive! Signal Strength is ${signal_percent}% and my sensors report ${sensor_data}"

if [[ ! -x "$DISCORD_SCRIPT" ]]; then
    log "Discord script not executable at $DISCORD_SCRIPT; unable to send message."
    exit 1
fi

python3 "$DISCORD_SCRIPT" --webhook "$DISCORD_WEBHOOK" "$message"
log "Discord notification sent."
