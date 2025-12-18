#!/usr/bin/env bash

set -uo pipefail

ARDUPILOT_SERVICE="${ARDUPILOT_SERVICE:-ardupilot}"
LTE_SIGNAL_SCRIPT="${LTE_SIGNAL_SCRIPT:-/home/pi/lte-signal-strength.sh}"
ONICS2_PATH="${ONICS2_PATH:-./ONICS2.py}"

ARDUPILOT_AVAILABLE=0
ARDUPILOT_WAS_ACTIVE=0

log() { printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$*"; }
section() { printf '\n=== %s ===\n' "$*"; }

format_duration() {
    local seconds=$1
    local days=$((seconds / 86400))
    local hours=$(( (seconds % 86400) / 3600 ))
    local minutes=$(( (seconds % 3600) / 60 ))
    local secs=$(( seconds % 60 ))
    if (( days > 0 )); then
        printf '%dd %02dh %02dm %02ds' "$days" "$hours" "$minutes" "$secs"
    else
        printf '%02dh %02dm %02ds' "$hours" "$minutes" "$secs"
    fi
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        log "Missing required command: $1"
        return 1
    fi
}

detect_ardupilot_unit() {
    if systemctl list-unit-files | grep -q "^${ARDUPILOT_SERVICE}\.service"; then
        ARDUPILOT_AVAILABLE=1
    fi
}

stop_ardupilot() {
    detect_ardupilot_unit
    if (( ! ARDUPILOT_AVAILABLE )); then
        log "ArduPilot service ${ARDUPILOT_SERVICE}.service not installed; skipping stop."
        return 0
    fi

    local status
    status=$(systemctl is-active "$ARDUPILOT_SERVICE" 2>/dev/null || true)
    if [[ $status == "active" ]]; then
        log "Stopping ${ARDUPILOT_SERVICE}.service before running sensor diagnostics…"
        if systemctl stop "$ARDUPILOT_SERVICE"; then
            ARDUPILOT_WAS_ACTIVE=1
            log "${ARDUPILOT_SERVICE}.service stopped."
        else
            log "Unable to stop ${ARDUPILOT_SERVICE}.service; continuing without sensor check."
            return 1
        fi
    else
        log "${ARDUPILOT_SERVICE}.service is not active (state: $status)."
    fi
}

restart_ardupilot() {
    if (( ! ARDUPILOT_AVAILABLE )); then
        log "Skipping ArduPilot restart – service not installed."
        return 0
    fi

    log "Requesting restart of ${ARDUPILOT_SERVICE}.service to confirm it resumes after diagnostics…"
    if systemctl restart "$ARDUPILOT_SERVICE"; then
        sleep 2
        local state
        state=$(systemctl is-active "$ARDUPILOT_SERVICE" 2>/dev/null || true)
        if [[ $state == "active" ]]; then
            log "${ARDUPILOT_SERVICE}.service restarted successfully."
        else
            log "${ARDUPILOT_SERVICE}.service restart attempted but state is '$state'."
        fi
    else
        log "Failed to restart ${ARDUPILOT_SERVICE}.service."
        return 1
    fi
}

run_sensors() {
    section "Sensor check"
    if (( EUID != 0 )); then
        log "Warning: sensors.py expects root privileges; current uid=$EUID."
    fi

    if [[ -f sensors.py ]]; then
        log "Running sensors.py (Navio2 IMU/barometer quick check)…"
        if python3 sensors.py; then
            log "Sensor check completed."
        else
            log "sensors.py reported an error (see above)."
        fi
    else
        log "sensors.py not found in $(pwd); skipping sensor check."
    fi
}

lte_signal_strength() {
    section "LTE signal strength"
    local candidate
    if command -v lte-signal-strength.sh >/dev/null 2>&1; then
        candidate=$(command -v lte-signal-strength.sh)
    elif [[ -x $LTE_SIGNAL_SCRIPT ]]; then
        candidate=$LTE_SIGNAL_SCRIPT
    else
        log "LTE signal script not found (looked for lte-signal-strength.sh and $LTE_SIGNAL_SCRIPT)."
        return
    fi

    log "Running $candidate for LTE signal report…"
    if "$candidate"; then
        log "LTE signal strength captured."
    else
        log "lte-signal-strength script failed (see output above)."
    fi
}

cpu_health() {
    section "CPU health"
    if [[ -f /sys/class/thermal/thermal_zone0/temp ]]; then
        local raw
        raw=$(< /sys/class/thermal/thermal_zone0/temp)
        local celsius
        celsius=$(awk -v t="$raw" 'BEGIN { printf "%.1f", t/1000 }')
        log "CPU temperature: ${celsius}°C"
    else
        log "CPU temperature sensor not available."
    fi

    if require_command uptime; then
        log "System uptime/load: $(uptime)"
    fi
}

realsense_devices() {
    section "Intel RealSense devices"
    if [[ -x $ONICS2_PATH ]]; then
        log "Enumerating RealSense devices via $ONICS2_PATH -e…"
        if "$ONICS2_PATH" -e; then
            log "RealSense enumeration complete."
        else
            log "RealSense enumeration failed (see output)."
        fi
    else
        log "ONICS2.py not executable or missing at $ONICS2_PATH."
    fi
}

lte_modem_status() {
    local interfaces
    interfaces=$(ip -brief link 2>/dev/null | awk '$1 ~ /^wwan/ {print $1 ":" $2}')
    if [[ -n $interfaces ]]; then
        log "LTE modem: connected (${interfaces//$'\n'/, })"
    else
        log "LTE modem: disconnected (no wwan interfaces detected)."
    fi
}

sik_radio_status() {
    local sik_by_id
    sik_by_id=$(ls /dev/serial/by-id/*SiK* 2>/dev/null || true)
    if [[ -n $sik_by_id ]]; then
        log "SiK radio: connected (${sik_by_id//$'\n'/, })"
        return
    fi

    if compgen -G "/dev/ttyUSB*" >/dev/null; then
        local usb_list
        usb_list=$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')
        log "SiK radio: detected USB serial device(s) (${usb_list}); unable to confirm SiK signature."
    else
        log "SiK radio: disconnected (no USB serial adapters detected)."
    fi
}

service_report() {
    local unit=$1 label=$2
    section "$label service status"

    if ! systemctl list-unit-files | grep -q "^${unit}\.service"; then
        log "${unit}.service is not installed."
        return
    fi

    local active sub start_mono uptime_seconds="N/A"
    active=$(systemctl is-active "$unit" 2>/dev/null || true)
    sub=$(systemctl show -p SubState --value "$unit" 2>/dev/null || echo "unknown")
    start_mono=$(systemctl show -p ActiveEnterTimestampMonotonic --value "$unit" 2>/dev/null || echo 0)

    if [[ $active == "active" && $start_mono =~ ^[0-9]+$ ]]; then
        local now
        now=$(cut -d' ' -f1 /proc/uptime)
        uptime_seconds=$(awk -v now="$now" -v start="$start_mono" 'BEGIN { printf "%.0f", now - start/1000000 }')
        log "$label: active ($sub) – uptime $(format_duration "$uptime_seconds")"
    else
        log "$label: $active ($sub)"
    fi
}

connected_devices() {
    section "Connected devices"
    realsense_devices
    lte_modem_status
    sik_radio_status
}

main() {
    section "ArduPilot pause"
    stop_ardupilot

    run_sensors

    section "ArduPilot resume"
    restart_ardupilot

    lte_signal_strength
    cpu_health
    connected_devices

    service_report uplink "uplink"
    service_report mavproxy "MAVProxy"
}

main "$@"
