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
                temperature_display="$temperature_value"
                if [[ "$temperature_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                    temperature_display="${temperature_value}℃"
                fi

                cpu_temp_value=""
                if [[ -r /sys/class/thermal/thermal_zone0/temp ]]; then
                    cpu_temp_raw=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || true)
                    if [[ "$cpu_temp_raw" =~ ^[0-9]+$ ]]; then
                        cpu_temp_value=$(awk -v val="$cpu_temp_raw" 'BEGIN {printf "%.2f", val / 1000}')
                    fi
                fi
                if [[ -z "$cpu_temp_value" ]] && command -v vcgencmd >/dev/null 2>&1; then
                    cpu_temp_value=$(vcgencmd measure_temp 2>/dev/null | sed 's/temp=//; s/[^0-9.]*//g')
                fi
                if [[ -z "$cpu_temp_value" ]] && command -v sensors >/dev/null 2>&1; then
                    cpu_temp_value=$(
                        sensors 2>/dev/null | awk '
                            BEGIN {IGNORECASE=1}
                            /°C/ && /(Package id 0|Tctl|Tdie|CPU|Core 0|temp1)/ {
                                for (i = 1; i <= NF; i++) {
                                    if ($i ~ /[0-9]+\.[0-9]+°C/) {
                                        gsub(/[^0-9.]/, "", $i)
                                        print $i
                                        exit
                                    }
                                }
                            }
                        '
                    )
                    if [[ -z "$cpu_temp_value" ]]; then
                        cpu_temp_value=$(
                            sensors 2>/dev/null | awk '
                                /°C/ {
                                    for (i = 1; i <= NF; i++) {
                                        if ($i ~ /[0-9]+\.[0-9]+°C/) {
                                            gsub(/[^0-9.]/, "", $i)
                                            print $i
                                            exit
                                        }
                                    }
                                }
                            '
                        )
                    fi
                fi
                cpu_temp_value=${cpu_temp_value:-unknown}
                cpu_temp_display="$cpu_temp_value"
                if [[ "$cpu_temp_value" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
                    cpu_temp_display="${cpu_temp_value}℃"
                fi

                printf -v temperature_line '%-12s %s (CPU %s )' "Temperature:" "$temperature_display" "$cpu_temp_display"
                printf -v pressure_line '%-9s %s Pa' "Barometer:" "$pressure_value"
                printf -v accel_line '%-13s %s' "Accelerometer:" "$average_accel"
                printf -v gyro_line '%-7s %s' "Gyroscope:" "$average_gyro"
                printf -v mag_line '%-10s %s' "Magnetometer:" "$average_mag"

                sensor_status=$(cat <<EOF
sensor readings are available.

$temperature_line
$pressure_line

$accel_line
$gyro_line
$mag_line
EOF
)
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

message="I'm alive! LTE Signal Strength is **${signal_percent}%** and ${sensor_status}"

if [[ "$DRY_RUN" == "true" ]]; then
    log "Dry run enabled; Discord message would be:"
    printf '%s\n' "$message"
    exit 0
fi

if [[ ! -x "$DISCORD_SCRIPT" ]]; then
    log "Discord script not executable at $DISCORD_SCRIPT; unable to send message."
    exit 0
fi

if /usr/bin/python3 "$DISCORD_SCRIPT" --webhook "$DISCORD_WEBHOOK" "$message"; then
    log "Discord notification sent."
else
    log "Discord notification failed; continuing startup."
fi
