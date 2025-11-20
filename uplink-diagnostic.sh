#!/usr/bin/env bash

# LTE uplink diagnostic utility
# Reports available LTE devices, prompts for APN, and attempts to establish
# connectivity using the same primitives as uplink.sh. Provides clear reasons
# for failures or confirmation of success.

set -uo pipefail

PING_TARGET="dataplicity.com"
DEFAULT_APN="${APN:-hologram}"
WWAN_PREFIX="wwan"
QMI_TIMEOUT=30
DHCP_TIMEOUT=20

log() {
    printf '%s %s\n' "$(date -u '+%Y-%m-%d %H:%M:%S')" "$*"
}

require_cmd() {
    local missing=()
    for cmd in "$@"; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            missing+=("$cmd")
        fi
    done

    if (( ${#missing[@]} > 0 )); then
        log "Missing required commands: ${missing[*]}"
        log "Install the missing dependencies and re-run the diagnostic."
        return 1
    fi
}

prompt_apn() {
    local input
    read -r -p "Enter APN [${DEFAULT_APN}]: " input
    if [[ -z $input ]]; then
        printf '%s' "$DEFAULT_APN"
    else
        printf '%s' "$input"
    fi
}

find_wwan_interfaces() {
    ip -brief link 2>/dev/null | awk -v prefix="$WWAN_PREFIX" '$1 ~ "^" prefix {print $1":"$2}'
}

discover_wdm() {
    local iface=$1
    readlink -f "/sys/class/net/$iface/device" 2>/dev/null | grep -o 'cdc-wdm[0-9]*' || true
}

toggle_raw_ip() {
    local iface=$1
    local path="/sys/class/net/$iface/qmi/raw_ip"
    [[ -w $path ]] && echo Y | sudo tee "$path" >/dev/null || log "raw_ip toggle unavailable for $iface"
}

start_qmi() {
    local iface=$1 apn=$2
    local wdm
    wdm=$(discover_wdm "$iface")

    if [[ -z $wdm ]]; then
        log "No cdc-wdm device found for $iface; cannot start QMI session."
        return 1
    fi

    log "Starting QMI session on $iface using /dev/$wdm (APN: $apn)…"
    if timeout "$QMI_TIMEOUT" sudo qmicli -p -d "/dev/$wdm" \
        --device-open-net='net-raw-ip|net-no-qos-header' \
        -wds-start-network="apn='$apn',ip-type=4" --client-no-release-cid; then
        log "QMI session established."
        return 0
    fi

    log "QMI start failed (exit $?)."
    return 1
}

acquire_dhcp() {
    local iface=$1
    log "Requesting DHCP lease on $iface…"
    if timeout ${DHCP_TIMEOUT}s sudo udhcpc -q -i "$iface" &>/dev/null; then
        log "DHCP lease acquired."
        return 0
    fi
    log "DHCP failed (exit $?)."
    return 1
}

check_internet() {
    local iface=$1
    log "Pinging $PING_TARGET over $iface…"
    local loss
    loss=$(ping -I "$iface" -c 3 -W 3 "$PING_TARGET" | \
        awk -F', ' '/packet loss/{
            for (i = 1; i <= NF; i++) {
                if ($i ~ /packet loss/) {
                    sub(/[^0-9]*([0-9]+).*/, "\\1", $i)
                    print $i + 0
                    exit
                }
            }
        }' || echo 100)
    if (( loss >= 100 )); then
        log "Ping failed with ${loss}% packet loss."
        return 1
    fi
    log "Ping succeeded with ${loss}% packet loss."
    return 0
}

bring_up_interface() {
    local iface=$1 apn=$2
    log "Configuring $iface for raw IP…"
    sudo ip link set "$iface" down 2>/dev/null || true
    toggle_raw_ip "$iface"
    sudo ip link set "$iface" up 2>/dev/null || true

    start_qmi "$iface" "$apn" || return 1
    acquire_dhcp "$iface" || return 1
    check_internet "$iface" || return 1
    return 0
}

print_interface_report() {
    local iface=$1
    local wdm driver state
    wdm=$(discover_wdm "$iface")
    driver=$(readlink -f "/sys/class/net/$iface/device/driver" 2>/dev/null | xargs basename 2>/dev/null)
    state=$(cat "/sys/class/net/$iface/operstate" 2>/dev/null)

    log "Interface: $iface"
    log "  Driver    : ${driver:-unknown}"
    log "  Operstate : ${state:-unknown}"
    log "  cdc-wdm   : ${wdm:-not found}"
}

main() {
    if ! require_cmd ip sudo qmicli udhcpc ping; then
        exit 1
    fi

    log "Scanning for LTE interfaces…"
    mapfile -t interfaces < <(find_wwan_interfaces)

    if (( ${#interfaces[@]} == 0 )); then
        log "No interfaces matching prefix '${WWAN_PREFIX}' found."
        exit 1
    fi

    log "Found ${#interfaces[@]} candidate interface(s)."
    for entry in "${interfaces[@]}"; do
        IFS=":" read -r iface status <<<"$entry"
        print_interface_report "$iface"
    done

    local selected
    if (( ${#interfaces[@]} > 1 )); then
        log "Multiple LTE interfaces detected."
        printf 'Select interface to test:\n'
        select opt in "${interfaces[@]}"; do
            if [[ -n $opt ]]; then
                selected=${opt%%:*}
                break
            fi
        done
    else
        selected=${interfaces[0]%%:*}
    fi

    local apn
    apn=$(prompt_apn)
    log "Using APN: $apn"

    if bring_up_interface "$selected" "$apn"; then
        log "Diagnostic succeeded – internet reachable via $selected."
        exit 0
    fi

    log "Diagnostic failed. Review the messages above for the failure stage."
    exit 1
}

main "$@"
