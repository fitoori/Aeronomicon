#!/bin/bash

DEFAULT_LOG_FILE="/home/pi/uplink.log"
DEFAULT_PING_TARGET="sixfab.com"
WWAN_INTERFACE="wwan0"
PING_INTERVAL=300  # 5 minutes
RETRY_INTERVAL=60
RETRY_ON_FAILURE=true
MAX_RETRIES=3
APN="hologram"

# Set default values
LOG_FILE="$DEFAULT_LOG_FILE"
PING_TARGET="$DEFAULT_PING_TARGET"

# Schedule daily reboot at 01:00
sudo shutdown -r 01:00

function log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1" >> "$LOG_FILE"
}

function check_connection() {
    local packet_loss=$(ping -I "$WWAN_INTERFACE" -c 3 -W 3 "$PING_TARGET" | grep -oP '\d+(?=% packet loss)' || echo "Error")

    if [[ "$packet_loss" == "Error" ]]; then
        log "Failed to parse packet loss. Connection check failed."
        return 1
    elif [[ "$packet_loss" -lt 100 ]]; then
        log "Received response from ping. Packet loss: $packet_loss%"
        return 0  # Connection is successful
    else
        log "No response from ping. Packet loss: $packet_loss%"
        return 1  # Connection failed
    fi
}

function reconnect_wwan() {
    log "Attempting to reconnect $WWAN_INTERFACE..."
    
    # Step 1: Disable WWAN interface
    sudo ip link set "$WWAN_INTERFACE" down
    
    # Step 2: Set raw_ip mode
    echo 'Y' | sudo tee "/sys/class/net/$WWAN_INTERFACE/qmi/raw_ip"
    
    # Step 3: Enable WWAN interface
    sudo ip link set "$WWAN_INTERFACE" up
    
    # Step 4: Start network with qmicli
    sudo qmicli -p -d /dev/cdc-wdm0 --device-open-net='net-raw-ip|net-no-qos-header' --wds-start-network="apn='$APN',ip-type=4" --client-no-release-cid
    
    # Step 5: Run udhcpc and log relevant messages
    udhcpc_output=$(sudo udhcpc -q -f -i "$WWAN_INTERFACE" 2>&1)
    if [[ $? -eq 0 ]]; then
        log "DHCP lease obtained successfully."
    else
        log "DHCP configuration failure: $udhcpc_output"
    fi
}

function handle_retries() {
    local retries=$1

    if [[ "$retries" -ge "$MAX_RETRIES" ]]; then
        log "Exceeded maximum retries. Rebooting device..."
        sudo reboot
    else
        log "Retrying connection. Attempt $((retries + 1)) of $MAX_RETRIES..."
        reconnect_wwan
        sleep "$RETRY_INTERVAL"
    fi
}

# Log first attempt since boot
log "First attempt since boot."

# Main loop
while true; do
    if check_connection; then
        sleep "$PING_INTERVAL"
    else
        log "Connection lost. Reconnecting..."
        
        # Kill qmicli and udhcpc processes
        sudo pkill -f "qmicli|udhcpc"
        
        # Retry connection steps every RETRY_INTERVAL seconds
        if "$RETRY_ON_FAILURE"; then
            retry_counter=0
            while ! check_connection; do
                handle_retries "$retry_counter"
                ((retry_counter++))
            done
        else
            reconnect_wwan
            sleep "$RETRY_INTERVAL"
        fi
    fi
done
