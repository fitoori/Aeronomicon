#!/usr/bin/env bash
set -euo pipefail

# LTE signal strength thresholds
RSSI_MIN=-113
RSSI_MAX=-51

# LTE signal variables
OUTPUT=""
RSSI_DBM=""
NUMERATOR=0
DENOMINATOR=0
PERCENT=""

# System info variables
HOSTNAME=""
CPU_LINE=""
CPU_USAGE=""
MEM_LINE=""
DISK_LINE=""
UPTIME_LINE=""

# CPU temperature variable
CPU_TEMP="[UNKNOWN]"

OUTPUT=$(sudo qmicli --device=/dev/cdc-wdm0 --nas-get-signal-strength 2>&1) || {
  echo "ERROR: qmicli command failed."
  exit 1
}

# Extract the RSSI (dBm)
RSSI_DBM=$(echo "$OUTPUT" | sed -n "/RSSI:/{n;s/.*'\([0-9.-]*\) dBm'.*/\1/p}")
if [ -z "$RSSI_DBM" ]; then
  echo "ERROR: Could not parse RSSI (dBm) from qmicli output."
  echo "$OUTPUT"
  exit 1
fi

# Ensure RSSI_DBM is an integer
if ! [[ "$RSSI_DBM" =~ ^-?[0-9]+$ ]]; then
  echo "ERROR: RSSI value is not a valid integer."
  exit 1
fi

# Clamp RSSI to expected range
if [ "$RSSI_DBM" -lt "$RSSI_MIN" ]; then
  RSSI_DBM=$RSSI_MIN
elif [ "$RSSI_DBM" -gt "$RSSI_MAX" ]; then
  RSSI_DBM=$RSSI_MAX
fi

# Compute approximate signal percentage
NUMERATOR=$((RSSI_DBM - RSSI_MIN))
DENOMINATOR=$((RSSI_MAX - RSSI_MIN))
PERCENT=$(
  awk -v n="$NUMERATOR" -v d="$DENOMINATOR" '
    BEGIN {
      if (d == 0) {
        print 0;
      } else {
        printf "%.2f", (n / d) * 100;
      }
    }
  '
)

# HOSTNAME
HOSTNAME=$(hostname 2>/dev/null || true)
[ -z "$HOSTNAME" ] && HOSTNAME="[UNKNOWN]"

# CPU usage
CPU_LINE=$(grep 'cpu ' /proc/stat 2>/dev/null || true)
if [ -z "$CPU_LINE" ]; then
  echo "ERROR: Could not read CPU info from /proc/stat."
  exit 1
fi
CPU_USAGE=$(echo "$CPU_LINE" | awk '{usage=($2+$4)*100/($2+$4+$5); printf "%.2f", usage}')

# CPU temperature (check multiple sources)
if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
  CPU_TEMP_RAW=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo "")
  if [[ "$CPU_TEMP_RAW" =~ ^[0-9]+$ ]]; then
    CPU_TEMP=$(awk "BEGIN { printf \"%.1f°C\", $CPU_TEMP_RAW / 1000 }")
  fi
elif command -v vcgencmd >/dev/null 2>&1; then
  CPU_TEMP=$(vcgencmd measure_temp 2>/dev/null | sed "s/temp=//")
elif command -v sensors >/dev/null 2>&1; then
  CPU_TEMP=$(sensors 2>/dev/null | grep -m1 -Eo '\+[0-9]+\.[0-9]+°C' | sed 's/^+//') || CPU_TEMP="[UNKNOWN]"
fi

# Memory usage
MEM_LINE=$(free -h 2>/dev/null | awk '/^Mem:/ {print $3 "/" $2}')
if [ -z "$MEM_LINE" ]; then
  echo "ERROR: Could not parse memory usage."
  exit 1
fi

# Disk usage
DISK_LINE=$(df -h / 2>/dev/null | awk 'NR==2 {print $5}')
if [ -z "$DISK_LINE" ]; then
  echo "ERROR: Could not parse disk usage."
  exit 1
fi

# Uptime
UPTIME_LINE=$(uptime -p 2>/dev/null || true)
[ -z "$UPTIME_LINE" ] && UPTIME_LINE="[UNKNOWN]"
echo
echo "Welcome to $HOSTNAME!"
echo "System Status:"
echo "  - LTE: ${PERCENT}% (RSSI: ${RSSI_DBM} dBm)"
echo "  - CPU: ${CPU_USAGE}% (${CPU_TEMP})"
echo "  - Memory Usage: $MEM_LINE"
echo "  - Disk Usage: $DISK_LINE"
echo "  - Uptime: $UPTIME_LINE"
echo

exit 0
