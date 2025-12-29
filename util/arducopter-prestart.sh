#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SIGNAL_SCRIPT="${SCRIPT_DIR}/lte-signal-strength.sh"
SENSOR_SCRIPT="${SCRIPT_DIR}/sensor-readings.sh"
DISCORD_SCRIPT="/home/pi/Aeronomicon/util/discord.py"
DISCORD_WEBHOOK="https://discord.com/api/webhooks/1454669304749621282/8LQlnXje9YyXtImmy7eDvARFAg7tYKqf16vuLiJPkQHW4OFXzJLAPSje7vVAi43jHqop"
DRY_RUN=false

log() {
    printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$*"
}

is_ardupilot_running() {
    if pgrep -f "/usr/bin/arducopter" >/dev/null 2>&1; then
        return 0
    fi
    if pgrep -x "arducopter" >/dev/null 2>&1; then
        return 0
    fi
    if command -v systemctl >/dev/null 2>&1; then
        if systemctl is-active --quiet arducopter 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

signal_percent="NaN"
if [[ -x "$SIGNAL_SCRIPT" ]]; then
    if signal_output=$("$SIGNAL_SCRIPT" 2>/dev/null); then
        parsed_percent=$(echo "$signal_output" | awk -F'Signal Strength: ' '{print $2}' | sed -n 's/%.*//p')
        if [[ -n "$parsed_percent" ]]; then
            if [[ "$parsed_percent" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                signal_percent=$(awk -v val="$parsed_percent" 'BEGIN {printf "%.2f", val}')
            else
                signal_percent="$parsed_percent"
            fi
        fi
    else
        log "LTE signal script failed; reporting NaN%."
    fi
else
    log "LTE signal script not executable at $SIGNAL_SCRIPT; reporting NaN%."
fi

sensor_status="sensor readings were unavailable."
if [[ -x "$SENSOR_SCRIPT" ]]; then
    if sensor_output=$("$SENSOR_SCRIPT" 2>/dev/null); then
        sensor_output=$(echo "$sensor_output" | sed '/^\s*$/d')
        if [[ -n "$sensor_output" ]]; then
            if echo "$sensor_output" | grep -q "Accelerometer:"; then
                accelerometer_line=$(echo "$sensor_output" | awk '/^Accelerometer:/ {print}')
                gyroscope_line=$(echo "$sensor_output" | awk '/^Gyroscope:/ {print}')
                magnetometer_line=$(echo "$sensor_output" | awk '/^Magnetometer:/ {print}')
                pressure_line=$(echo "$sensor_output" | awk '/^Pressure:/ {print}')
                temperature_line=$(echo "$sensor_output" | awk '/^Temperature:/ {print}')

                average_vector() {
                    local line="$1"
                    if [[ -z "$line" ]]; then
                        return 1
                    fi
                    local numbers
                    numbers=$(echo "$line" | sed 's/^[^[]*\[//; s/\].*$//; s/,/ /g')
                    if [[ -z "$numbers" ]]; then
                        return 1
                    fi
                    awk '
                        {
                            sum=0
                            count=0
                            for (i=1; i<=NF; i++) {
                                sum += $i
                                count++
                            }
                            if (count > 0) {
                                printf "%.2f", sum / count
                            }
                        }' <<<"$numbers"
                }

                average_accel=$(average_vector "$accelerometer_line" || true)
                average_gyro=$(average_vector "$gyroscope_line" || true)
                average_mag=$(average_vector "$magnetometer_line" || true)
                average_accel=${average_accel:-unknown}
                average_gyro=${average_gyro:-unknown}
                average_mag=${average_mag:-unknown}

                pressure_value=$(echo "$pressure_line" | awk '{print $2}')
                temperature_value=$(echo "$temperature_line" | awk '{print $2}')

                if [[ -n "$pressure_value" ]] && [[ "$pressure_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                    pressure_value=$(awk -v val="$pressure_value" 'BEGIN {printf "%.2f", val}')
                fi
                if [[ -n "$temperature_value" ]] && [[ "$temperature_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                    temperature_value=$(awk -v val="$temperature_value" 'BEGIN {printf "%.2f", val}')
                fi
                pressure_value=${pressure_value:-unknown}
                temperature_value=${temperature_value:-unknown}

                sensor_status="sensor readings: average accelerometer ${average_accel}, average gyroscope ${average_gyro}, average magnetometer ${average_mag}, pressure ${pressure_value} Pa, temperature ${temperature_value} C."
            else
                log "Sensor output missing expected fields; reporting unavailable sensor data."
            fi
        fi
    else
        log "Sensor script failed; reporting unavailable sensor data."
    fi
else
    log "Sensor script not executable at $SENSOR_SCRIPT; reporting unavailable sensor data."
fi

if [[ "$sensor_status" == "sensor readings were unavailable." ]]; then
    if is_ardupilot_running; then
        sensor_status="I wasn't able to get sensor readings because ArduPilot is running."
    fi
fi

message="I'm alive! LTE Signal Strength is ${signal_percent}% and ${sensor_status}"

if [[ "$DRY_RUN" == "true" ]]; then
    log "Dry run enabled; Discord message would be:"
    printf '%s\n' "$message"
    exit 0
fi

if [[ ! -x "$DISCORD_SCRIPT" ]]; then
    log "Discord script not executable at $DISCORD_SCRIPT; unable to send message."
    exit 1
fi

/usr/bin/python3 "$DISCORD_SCRIPT" --webhook "$DISCORD_WEBHOOK" "$message"
log "Discord notification sent."
