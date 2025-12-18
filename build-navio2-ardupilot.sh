#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

LOG="/var/log/ardupilot_navio2_build.log"
LOG_DIR="$(dirname "$LOG")"

mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG") 2>&1

say() { printf '\n%s\n' "$*"; }
err() { printf 'ERROR: %s\n' "$*" >&2; }

cleanup() {
    status=$?
    if [[ $status -ne 0 ]]; then
        err "Build failed with exit code $status. Check $LOG for details."
    fi
}
trap cleanup EXIT

say "=== ArduPilot Navio2 Build Script ==="
say "Log: $LOG"

#############################################
# 1. Root Check
#############################################
if [[ "$(id -u)" -ne 0 ]]; then
    err "Script must be run as root."
    exit 1
fi

#############################################
# 2. Required Commands Check (pre-install)
#############################################
REQUIRED_CMDS_PRE=(apt-get git tee)
for cmd in "${REQUIRED_CMDS_PRE[@]}"; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        err "Required command missing: $cmd"
        exit 1
    fi
done

#############################################
# 3. Install Dependencies
#############################################
say "Installing build dependencies..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -y
apt-get install -y \
    python3 python3-pip python3-dev python3-setuptools python3-wheel \
    python3-future python3-numpy python3-opencv python3-lxml \
    git build-essential g++ gawk flex bison libffi-dev libssl-dev \
    libxml2-dev libxslt-dev pkg-config screen bc

#############################################
# 4. Required Commands Check (post-install)
#############################################
REQUIRED_CMDS_POST=(gcc g++ make python3 pip3 wget bc)
for cmd in "${REQUIRED_CMDS_POST[@]}"; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        err "Required command missing after install: $cmd"
        exit 1
    fi
done

#############################################
# 5. Verify Navio2 Kernel Overlay
#############################################
BOOTCFG="/boot/config.txt"

ensure_config_line() {
    local line="$1"
    if ! grep -Fxq "$line" "$BOOTCFG"; then
        echo "$line" >> "$BOOTCFG"
        return 0
    fi
    return 1
}

need_reboot=false
if ensure_config_line "dtoverlay=navio2"; then
    say "Added navio2 overlay to $BOOTCFG."
    need_reboot=true
fi
if ensure_config_line "dtparam=i2c_arm=on"; then
    say "Enabled I2C in $BOOTCFG."
    need_reboot=true
fi

if [[ "$need_reboot" == true ]]; then
    say "Reboot required. Run script again after reboot to complete build."
    exit 0
fi

say "Navio2 overlay detected."

#############################################
# 6. Verify LED sysfs Availability (Optional)
#############################################
LED_PATH="/sys/class/leds"
if [[ ! -d "$LED_PATH/rgbled_r" ]] || [[ ! -d "$LED_PATH/rgbled_g" ]] || [[ ! -d "$LED_PATH/rgbled_b" ]]; then
    say "WARNING: Navio2 RGB LED sysfs not detected. Overlays may not have loaded correctly."
else
    say "Navio2 RGB LED sysfs detected."
fi

#############################################
# 7. Clone or Update ArduPilot
#############################################
ARDUPILOT_DIR="/root/ardupilot"

if [[ -d "$ARDUPILOT_DIR/.git" ]]; then
    say "Updating existing ArduPilot checkout at $ARDUPILOT_DIR ..."
    cd "$ARDUPILOT_DIR"
    git fetch --all
    git reset --hard origin/master
else
    say "Cloning ArduPilot to $ARDUPILOT_DIR ..."
    git clone https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
    cd "$ARDUPILOT_DIR"
fi

say "Initializing submodules..."
git submodule update --init --recursive

#############################################
# 8. Configure Waf for Navio2
#############################################
say "Configuring Waf for Navio2..."
./waf distclean
./waf configure --board=navio2

#############################################
# 9. Build ArduCopter (default)
#############################################
say "Building ArduCopter..."
./waf copter

BIN_PATH="$ARDUPILOT_DIR/build/navio2/bin/arducopter"
if [[ ! -f "$BIN_PATH" ]]; then
    err "Build failed. Binary not found at $BIN_PATH"
    exit 1
fi

#############################################
# 10. Install Binary
#############################################
TARGET_BIN="/usr/bin/arducopter"
say "Installing ArduCopter to $TARGET_BIN ..."
install -m 755 "$BIN_PATH" "$TARGET_BIN"

#############################################
# 11. Final Hardware Sanity Check
#############################################
say "Checking Navio2 kernel modules..."
STATUS_FILE="/sys/kernel/rcio/rcio/status"

if [[ -f "$STATUS_FILE" ]]; then
    STATUS_CONTENT=$(cat "$STATUS_FILE")
    say "RCIO Status: $STATUS_CONTENT"
else
    say "WARNING: RCIO status not available. Hardware may not be initialized yet."
fi

#############################################
# 12. Summary
#############################################
say "=== COMPLETED SUCCESSFULLY ==="
say "ArduPilot (Navio2) binary installed at: $TARGET_BIN"
say "Run with:"
say "  sudo $TARGET_BIN -A udp:0.0.0.0:14550"
say ""
say "If LED status doesn't work, set this parameter in ArduPilot:"
say "  NTF_LED_TYPES = 127"
say ""
say "Full build log saved to: $LOG"
say "============================================="
