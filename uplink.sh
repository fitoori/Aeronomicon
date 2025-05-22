#!/usr/bin/env bash

#################################
##    LTE Uplink for WATNE     ##
##         Version 5.0         ##
#################################

# Self-healing LTE connection for when you need it most.
# Messed up so bad you can't connect to your drone anymore?
# If it has enough power to last until reboot time you get another shot!
# Now integrates with ONICS to parse arming status before reboot.

# Requires: bash ≥ 4, sudo, iproute2, qmicli, udhcpc; jq optional 

# 300s  = 5mins
# 600s  = 10mins
# 1200s = 20mins
# 3600s = 1hr
# You figure out the rest.

set -euo pipefail
shopt -s lastpipe
IFS=$'\n\t'

############################### user parameters ###############################

## log-related parameters
readonly LOG_FILE="/home/pi/uplink.log"                # logfile location
readonly LOG_MAX_BYTES=$((1 * 1024 * 1024))            # rotate at 1 MiB

## LTE-related parameters
readonly PING_TARGET="dataplicity.com"                 # website to ping when testing internet connectivity
readonly WWAN_INTERFACE="wwan0"                        # LTE interface as parsed by ifconfig
readonly APN="hologram"                                # APN to be passed to qmicli

## persistency-related parameters
readonly PING_INTERVAL=300          # time between health checks (s)
readonly RETRY_INTERVAL=60          # base back-off
readonly MAX_RETRIES=3              # amount of failures to tolerate before rebooting
readonly MAX_BACKOFF=900            # cap back-off (s)

readonly DAILY_REBOOT_TIME="01:00"  # scheduled time to run reboot (local HH:MM; 24hr; empty ⇒ no reboot)
readonly PRE_REBOOT_CHECK_SEC=45    # run arming check this many seconds prior to scheduled reboot

## flight-related parameters
readonly ARMING_STATUS_FILE="/home/pi/arming_status.json"
readonly FRESH_THRESHOLD=600        # this value should be the maximum theoretical flight time of the vehicle (s)

#─ STALE_AS_ARMED ─ what it does and why it matters ─

#+----------+---------------------------+-------------------------+---------------------------+
#| Setting  | When status file is       | Benefit                 | Risk / Cost               |
#|          | missing or >10 min old    |                         |                           |
#+==========+===========================+=========================+===========================+
#| false    | Assume DISARMED → reboot  | • Box always reboots,   | • 1-in-a-million chance of|
#|(default) | proceeds                  |   restoring control     |  rebooting mid-flight if  |
#|          |                           | • Best for long ground  |  status feed dies during  |
#|          |                           |  tests & unattended use |  a short flight           |
#+----------+---------------------------+-------------------------+---------------------------+
#| true     | Assume ARMED → cancel     | • Absolute guarantee    | • If status pipeline dies |
#|          | reboot                    |   against airborne      |  vehicle may stay offline |
#|          |                           |   reboot                |  forever                  |
#+----------+---------------------------+-------------------------+---------------------------+

# Choose **false** for availability (ground testing, weeks of uptime).  
# Flip to **true** only when an in-air reboot is intolerable *and* you trust the status file to stay healthy.

readonly STALE_AS_ARMED=false       # true ⇒ indicates whether the script should constitute a stale value as ARMED or DISARMED
###############################################################################


############################ logging & housekeeping ###########################
rotate_log() {
    [[ -f $LOG_FILE && $(stat -c%s "$LOG_FILE") -gt $LOG_MAX_BYTES ]] || return
    mv "$LOG_FILE" "${LOG_FILE}.$(date -u +%Y%m%d%H%M%S)"
    : >"$LOG_FILE"
}

log() { rotate_log; printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$1" >>"$LOG_FILE"; }

trap 'log "Termination caught – exiting."; exit 0' TERM INT
trap 'log "Script aborted by set -e."; exit 1' ERR

############################# single-instance lock ############################
exec 200>/run/uplink.lock
flock -n 200 || { echo "uplink.sh already running – exit." >&2; exit 0; }

############################### arming helpers ################################
# 0 ⇒ proceed with reboot; 1 ⇒ cancel reboot
pre_reboot_safety_check() {
    log "Pre-reboot check…"

    # read status -------------------------------------------------------------
    if [[ -r $ARMING_STATUS_FILE ]]; then
        if command -v jq >/dev/null 2>&1; then
            armed=$(jq -r '.armed // empty' "$ARMING_STATUS_FILE" 2>/dev/null || true)
            ts=$(jq  -r '.timestamp_utc // empty' "$ARMING_STATUS_FILE" 2>/dev/null || true)
        else
            armed=$(awk '/"armed":/{gsub(/[^tf]/,"");print;exit}' "$ARMING_STATUS_FILE")
            ts=$(awk -F'"' '/"timestamp_utc":/{print $4;exit}' "$ARMING_STATUS_FILE")
        fi
    fi

    # missing or corrupt file -------------------------------------------------
    if [[ -z ${armed:-} || -z ${ts:-} ]]; then
        $STALE_AS_ARMED && { log "Status missing/corrupt – cancelling reboot."; sudo shutdown -c; return 1; }
        log "Status missing/corrupt – proceeding with reboot."
        return 0
    fi

    # freshness check ---------------------------------------------------------
    ts_epoch=$(date -d "$ts" +%s 2>/dev/null) || {
        $STALE_AS_ARMED && { log "Timestamp parse fail – cancelling reboot."; sudo shutdown -c; return 1; }
        log "Timestamp parse fail – proceeding with reboot."
        return 0
    }

    age=$(( $(date -u +%s) - ts_epoch ))

    if (( age > FRESH_THRESHOLD )); then
        $STALE_AS_ARMED && { log "Status stale (${age}s) – cancelling reboot."; sudo shutdown -c; return 1; }
        log "Status stale (${age}s) – proceeding with reboot."
        return 0
    fi

    # fresh record ------------------------------------------------------------
    if [[ $armed == "true" ]]; then
        log "Vehicle ARMED (fresh) – cancelling reboot."
        sudo shutdown -c
        return 1
    fi

    log "Vehicle DISARMED (fresh) – proceeding with reboot."
    return 0
}

############################## reboot scheduler ###############################
next_reboot_job() {
    local target today now seconds_until
    today=$(date '+%F')
    target="${today} ${DAILY_REBOOT_TIME}"
    seconds_until=$(( $(date -d "$target" +%s) - $(date +%s) ))
    (( seconds_until <= PRE_REBOOT_CHECK_SEC )) && seconds_until=$(( seconds_until + 86400 ))
    printf '%s' "$seconds_until"
}

schedule_daily_reboot() {
    [[ -z $DAILY_REBOOT_TIME ]] && return
    sudo shutdown -c || true
    sudo shutdown -r "$DAILY_REBOOT_TIME"

    sec_until=$(next_reboot_job)
    log "Scheduled reboot at ${DAILY_REBOOT_TIME} (in ${sec_until}s)."

    (   sleep $(( sec_until - PRE_REBOOT_CHECK_SEC ))
        pre_reboot_safety_check
        schedule_daily_reboot
    ) &
}

################ connection / recovery primitives (unchanged) #################
discover_wdm() {
    readlink -f "/sys/class/net/$WWAN_INTERFACE/device" 2>/dev/null \
        | grep -o 'cdc-wdm[0-9]*' || echo "cdc-wdm0"
}
toggle_raw_ip() { f="/sys/class/net/$WWAN_INTERFACE/qmi/raw_ip"; [[ -w $f ]] && echo Y | sudo tee "$f" >/dev/null || log "raw_ip toggle unavailable"; }
qmi_start() { sudo qmicli -p -d "/dev/$(discover_wdm)" --device-open-net='net-raw-ip|net-no-qos-header' --wds-start-network="apn='$APN',ip-type=4" --client-no-release-cid; }
dhcp_lease() { timeout 20s sudo udhcpc -q -i "$WWAN_INTERFACE" &>/dev/null; }
check_connection() { loss=$(ping -I "$WWAN_INTERFACE" -c 3 -W 3 "$PING_TARGET" | awk -F', ' '/packet loss/{print $(NF-2)+0}' || echo 100); (( loss >= 100 )) && { log "Ping FAIL – ${loss}%."; return 1; }; log "Ping OK – ${loss}%."; }
reconnect_wwan() { log "Re-initialising $WWAN_INTERFACE…"; sudo ip link set "$WWAN_INTERFACE" down; toggle_raw_ip; sudo ip link set "$WWAN_INTERFACE" up; qmi_start && log "QMI up" || log "QMI fail"; dhcp_lease && log "DHCP ok" || log "DHCP fail"; }
safe_kill_tools() { sudo pkill -x -f '^qmicli .*(--wds-start-network|--wda-.*-data-.*)$' || true; sudo pkill -x udhcpc || true; sudo pkill -o -x ping -f " -I $WWAN_INTERFACE " || true; }
handle_retries() { retries=$1; (( retries >= MAX_RETRIES )) && { log "Max retries – forcing reboot."; sudo reboot; }; backoff=$(( RETRY_INTERVAL << retries )); (( backoff > MAX_BACKOFF )) && backoff=$MAX_BACKOFF; log "Retry $(( retries + 1 ))/$MAX_RETRIES – sleep ${backoff}s."; reconnect_wwan; sleep "$backoff"; }

################################## startup ###################################
log "========== LTE-uplink script started =========="
schedule_daily_reboot

################################### main loop #################################
while true; do
    if check_connection; then
        sleep "$PING_INTERVAL"
        continue
    fi
    log "Connection lost – recovery."
    safe_kill_tools
    retry_counter=0
    until check_connection; do
        handle_retries "$retry_counter"
        (( retry_counter++ ))
    done
done
