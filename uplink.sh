
#!/usr/bin/env bash
###############################################################################
#  LTE Uplink for WATNE – self-healing, arming-aware connection manager       #
###############################################################################

set -euo pipefail
IFS=$'\n\t'

########################### configurable parameters ###########################
LOG_FILE="/home/pi/uplink.log"
PING_TARGET="dataplicity.com"
WWAN_INTERFACE="wwan0"
PING_INTERVAL=300           # seconds between health checks
RETRY_INTERVAL=60           # seconds between reconnection attempts
MAX_RETRIES=3               # after this many failures → safe_reboot
APN="hologram"

DAILY_REBOOT_TIME="01:00"   # empty to disable daily reboot

# Arming-status JSON written by ONICS
ARMING_STATUS_FILE="/home/pi/arming_status.json"
STALE_THRESHOLD=3600        # seconds; older == “stale”
STALE_AS_ARMED=true         # true => stale = armed, false => stale = disarmed
# WATNE's ArduPilot runs directly off the Linux stack and a reboot would cause 
# the intricate flying machine to fall to the ground with all the elegance of 
# a very expensive toaster oven. Therefore set the above value to false at
# your own risk. 
###############################################################################

log() { printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$1" >>"$LOG_FILE"; }

# ─────────── arming status helper ────────────────────────────────────────────
# Returns 0 if DISARMED (incl. stale treated as DISARMED), 1 otherwise
is_vehicle_disarmed() {
    if [[ ! -r $ARMING_STATUS_FILE ]]; then
        log "Arming-status file missing → assuming ARMED"
        return 1
    fi

    local armed_raw ts_raw
    armed_raw=$(grep -o '"armed":[[:space:]]*\(true\|false\)' "$ARMING_STATUS_FILE" \
                | head -n1 | awk -F: '{gsub(/[[:space:]]*/, "", $2);print $2}') || true
    ts_raw=$(grep -o '"timestamp_utc":[[:space:]]*"[^"]\+"' "$ARMING_STATUS_FILE" \
             | head -n1 | cut -d'"' -f4) || true

    if [[ -z $armed_raw || -z $ts_raw ]]; then
        log "Arming-status JSON incomplete → assuming ARMED"
        return 1
    fi

    local ts_epoch now_epoch age
    ts_epoch=$(date -d "$ts_raw" '+%s' 2>/dev/null) || {
        log "Timestamp parse failure → assuming ARMED"
        return 1
    }
    now_epoch=$(date -u '+%s')
    age=$(( now_epoch - ts_epoch ))

    if (( age > STALE_THRESHOLD )); then
        if $STALE_AS_ARMED; then
            log "Arming-status STALE (${age}s) – treated as ARMED"
            return 1
        else
            log "Arming-status STALE (${age}s) – treated as DISARMED"
            return 0
        fi
    fi

    log "Arming-status fresh (${age}s) – armed=$armed_raw"
    [[ $armed_raw == "false" ]] && return 0 || return 1
}

safe_reboot() {
    if is_vehicle_disarmed; then
        log "Conditions met → rebooting now"
        sudo reboot
    else
        log "Reboot aborted: vehicle armed or status unknown"
    fi
}

# ───────── connection management  ───────────────────────────────────────────
check_connection() {
    local loss
    loss=$(ping -I "$WWAN_INTERFACE" -c 3 -W 3 "$PING_TARGET" \
            | grep -oP '\d+(?=% packet loss)' || echo "Error")
    if [[ $loss == "Error" || $loss -ge 100 ]]; then
        log "Connection check FAILED – packet loss ${loss:-Unknown}%"
        return 1
    fi
    log "Ping OK – packet loss $loss%"
    return 0
}

reconnect_wwan() {
    log "Re-initialising $WWAN_INTERFACE …"
    sudo ip link set "$WWAN_INTERFACE" down
    echo 'Y' | sudo tee "/sys/class/net/$WWAN_INTERFACE/qmi/raw_ip" >/dev/null
    sudo ip link set "$WWAN_INTERFACE" up
    sudo qmicli -p -d /dev/cdc-wdm0 \
         --device-open-net='net-raw-ip|net-no-qos-header' \
         --wds-start-network="apn='$APN',ip-type=4" --client-no-release-cid
    if sudo udhcpc -q -f -i "$WWAN_INTERFACE" &>/dev/null; then
        log "DHCP lease obtained."
    else
        log "DHCP lease FAILED."
    fi
}

handle_retries() {
    local retries=$1
    if (( retries >= MAX_RETRIES )); then
        log "Max retries reached ($MAX_RETRIES) – initiating safe reboot"
        safe_reboot
        return 0
    fi
    log "Retry #$(( retries + 1 )) of $MAX_RETRIES …"
    reconnect_wwan
    sleep "$RETRY_INTERVAL"
    return 1
}

# ───────── daily reboot gate ────────────────────────────────────────────────
last_reboot_day=""
maybe_daily_reboot() {
    [[ -z $DAILY_REBOOT_TIME ]] && return
    local now day
    now=$(date '+%H:%M'); day=$(date '+%F')
    if [[ $now == "$DAILY_REBOOT_TIME" && $day != "$last_reboot_day" ]]; then
        log "Daily reboot window reached ($DAILY_REBOOT_TIME)."
        if is_vehicle_disarmed; then
            last_reboot_day=$day
            log "Daily reboot authorised."
            sudo reboot
        else
            log "Daily reboot postponed: vehicle armed/unknown."
        fi
    fi
}

# ───────── main loop ────────────────────────────────────────────────────────
log "===== LTE-uplink script started ====="
while true; do
    if check_connection; then
        maybe_daily_reboot
        sleep "$PING_INTERVAL"
        continue
    fi

    log "Connection lost – attempting recovery"
    sudo pkill -f "qmicli|udhcpc" || true

    retry_counter=0
    until check_connection; do
        handle_retries "$retry_counter" || break
        ((retry_counter++))
    done
done
