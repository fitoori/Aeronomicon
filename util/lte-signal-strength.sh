#!/usr/bin/env bash
#
# Query LTE signal strength from /dev/cdc-wdm0 using qmicli,
# parse RSSI from the known multi-line output format, and print
# a one-line summary with RSSI (dBm) and an approximate percentage.

set -euo pipefail

# Define thresholds for RSSI (dBm) to percentage conversion
RSSI_MIN=-113
RSSI_MAX=-51

# Run qmicli command with error checking. Prefer direct invocation and
# fall back to non-interactive sudo if available.
OUTPUT=""
EXIT_CODE=0

if OUTPUT=$(qmicli --device=/dev/cdc-wdm0 --nas-get-signal-strength 2>&1); then
  EXIT_CODE=0
else
  EXIT_CODE=$?
  if command -v sudo >/dev/null 2>&1; then
    if OUTPUT=$(sudo -n qmicli --device=/dev/cdc-wdm0 --nas-get-signal-strength 2>&1); then
      EXIT_CODE=0
    else
      EXIT_CODE=$?
    fi
  fi
fi

if [ $EXIT_CODE -ne 0 ]; then
  echo "ERROR: qmicli command failed with exit code $EXIT_CODE."
  echo "$OUTPUT"
  exit 1
fi

# Extract the RSSI (dBm) from the lines after "RSSI:".
# Example lines:
#   RSSI:
#         Network 'lte': '-59 dBm'
RSSI_DBM=$(echo "$OUTPUT" \
  | sed -n "/RSSI:/{n;s/.*'\([0-9.-]*\) dBm'.*/\1/p}")

# Validate that we found an RSSI value
if [ -z "$RSSI_DBM" ]; then
  echo "ERROR: Could not parse RSSI (dBm) from qmicli output."
  echo "$OUTPUT"
  exit 1
fi
# Clamp the RSSI to the known range
if [ "$RSSI_DBM" -lt "$RSSI_MIN" ] 2>/dev/null; then
  RSSI_DBM=$RSSI_MIN
elif [ "$RSSI_DBM" -gt "$RSSI_MAX" ] 2>/dev/null; then
  RSSI_DBM=$RSSI_MAX
fi

# Calculate approximate signal strength percentage
# Formula: ((RSSI - RSSI_MIN) / (RSSI_MAX - RSSI_MIN)) * 100
NUMERATOR=$((RSSI_DBM - RSSI_MIN))
DENOMINATOR=$((RSSI_MAX - RSSI_MIN))

PERCENT=$(
  awk -v num="$NUMERATOR" -v den="$DENOMINATOR" '
    BEGIN {
      if (den == 0) {
        print 0;
      } else {
        printf "%.2f", (num / den) * 100;
      }
    }
  '
)

# Print a single-line summary
echo "RSSI: ${RSSI_DBM} dBm, Signal Strength: ${PERCENT}%"

exit 0
