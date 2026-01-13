#!/usr/bin/env bash
set -euo pipefail

log()  { echo "[update] $*"; }
warn() { echo "[update][WARN] $*" >&2; }
die()  { echo "[update][ERROR] $*" >&2; exit 1; }

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    die "Required command not found: $1"
  fi
}

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUDO=()
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  SUDO=(sudo)
fi

FULL_UPDATE=false

UPLINK_AVAILABLE=false
UPLINK_WAS_ACTIVE=""
UPLINK_WAS_ENABLED=""

WWAN_PRESENT=false
WWAN_WAS_UP=false

# PiSugar runtime state
PISUGAR_SOFTWARE_AVAILABLE=false   # pisugar-server service present
PISUGAR_VALIDATED=false           # end-to-end validated via pisugar-server
PISUGAR_MODEL=""
PISUGAR_I2C_BUS=""
PISUGAR_BATTERY_PCT=""

check_hostname() {
  local current_hostname=""
  if command -v hostname >/dev/null 2>&1; then
    current_hostname="$(hostname 2>/dev/null || true)"
  fi
  case "${current_hostname}" in
    WATNE|watne)
      FULL_UPDATE=true
      log "hostname verified - proceeding with full update procedure and verification. Please ensure the vehicle is grounded."
      ;;
    *)
      FULL_UPDATE=false
      log "Hostname mismatch - treating device as Base Station / Auxiliary Device."
      ;;
  esac
}

detect_uplink_service() {
  local load_state
  load_state="$(systemctl show -p LoadState --value uplink.service 2>/dev/null || true)"
  if [[ -n "${load_state}" && "${load_state}" != "not-found" ]]; then
    UPLINK_AVAILABLE=true
  fi
}

record_uplink_state() {
  if [[ "${UPLINK_AVAILABLE}" != true ]]; then
    return
  fi
  UPLINK_WAS_ACTIVE="$(systemctl is-active uplink.service || true)"
  UPLINK_WAS_ENABLED="$(systemctl is-enabled uplink.service || true)"
}

restore_uplink_state() {
  if [[ "${UPLINK_AVAILABLE}" != true ]]; then
    return
  fi

  case "${UPLINK_WAS_ENABLED}" in
    enabled)
      log "Re-enabling uplink.service."
      "${SUDO[@]}" systemctl enable uplink.service
      ;;
    disabled)
      log "Keeping uplink.service disabled."
      "${SUDO[@]}" systemctl disable uplink.service >/dev/null 2>&1 || true
      ;;
    *)
      log "Leaving uplink.service enablement state unchanged (${UPLINK_WAS_ENABLED})."
      ;;
  esac

  if [[ "${UPLINK_WAS_ACTIVE}" == "active" ]]; then
    log "Starting uplink.service."
    "${SUDO[@]}" systemctl start uplink.service
  else
    log "Keeping uplink.service stopped."
    "${SUDO[@]}" systemctl stop uplink.service >/dev/null 2>&1 || true
  fi
}

record_wwan_state() {
  if ip link show wwan0 >/dev/null 2>&1; then
    WWAN_PRESENT=true
    if ip -o link show wwan0 | awk '{for (i=1;i<=NF;i++) if ($i=="state"){print $(i+1); exit}}' | grep -q "UP"; then
      WWAN_WAS_UP=true
    fi
  fi
}

restore_wwan_state() {
  if [[ "${WWAN_PRESENT}" != true ]]; then
    return
  fi
  if [[ "${WWAN_WAS_UP}" == true ]]; then
    log "Bringing wwan0 back up."
    "${SUDO[@]}" ip link set wwan0 up
  else
    log "Keeping wwan0 down."
    "${SUDO[@]}" ip link set wwan0 down >/dev/null 2>&1 || true
  fi
}

cleanup() {
  restore_wwan_state
  restore_uplink_state
}

check_non_wwan_internet() {
  local route_dev
  route_dev="$(ip -o route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev"){print $(i+1); exit}}')"
  if [[ -z "${route_dev}" ]]; then
    die "Unable to determine a route to the internet."
  fi
  if [[ "${route_dev}" == "wwan0" ]]; then
    die "Default route uses wwan0. A non-wwan0 connection is required before updating."
  fi
  if ! curl --interface "${route_dev}" --connect-timeout 5 --max-time 10 -fsS https://1.1.1.1/cdn-cgi/trace >/dev/null 2>&1; then
    die "No internet connectivity detected on ${route_dev}."
  fi
}

update_repo() {
  log "Updating repository in ${REPO_DIR}."
  git -C "${REPO_DIR}" fetch --prune
  git -C "${REPO_DIR}" pull --ff-only
}

ensure_scripts_executable() {
  log "Ensuring scripts are executable."
  local scripts=(
    "${REPO_DIR}/util/arducopter-prestart.sh"
    "${REPO_DIR}/util/lte-signal-strength.sh"
    "${REPO_DIR}/util/sensor-readings.sh"
    "${REPO_DIR}/util/discord.py"
    "${REPO_DIR}/uplink.sh"
    "${REPO_DIR}/legacy/ONICS2.py"
    "${REPO_DIR}/legacy/onlcs-lite.py"
  )
  for script in "${scripts[@]}"; do
    [[ -f "${script}" ]] || die "Expected script not found: ${script}"
    "${SUDO[@]}" chmod +x "${script}"
  done
  "${SUDO[@]}" find "${REPO_DIR}" -type f -name "*.sh" -exec chmod +x {} +
}

prompt_service_replacement() {
  local response="n"
  if [[ -r /dev/tty ]]; then
    read -r -p "Replace systemd service files with newer versions? [y/N] " response </dev/tty || response="n"
  elif [[ -t 0 ]]; then
    read -r -p "Replace systemd service files with newer versions? [y/N] " response || response="n"
  fi
  case "${response}" in
    [Yy]|[Yy][Ee][Ss])
      run_installer_update
      ;;
    *)
      log "Skipping service file replacement."
      ;;
  esac
}

run_installer_update() {
  local installer="${REPO_DIR}/util/services/install-services.sh"
  [[ -x "${installer}" ]] || die "Installer not found or not executable: ${installer}"
  log "Running installer in update mode."
  "${SUDO[@]}" bash -c "cd '${REPO_DIR}' && exec '${installer}' --update"
}

update_system_packages() {
  log "Updating apt package lists."
  "${SUDO[@]}" apt-get update
  log "Upgrading installed packages."
  "${SUDO[@]}" apt-get upgrade -y
  log "Autoremoving unused packages."
  "${SUDO[@]}" apt-get autoremove -y
}

# --- PiSugar support ---------------------------------------------------------

detect_pisugar_server() {
  local load_state
  load_state="$(systemctl show -p LoadState --value pisugar-server.service 2>/dev/null || true)"
  if [[ -n "${load_state}" && "${load_state}" != "not-found" ]]; then
    PISUGAR_SOFTWARE_AVAILABLE=true
    return
  fi

  # Some installs may register without ".service" in user docs; systemd resolves it,
  # but we keep the explicit check above.
  PISUGAR_SOFTWARE_AVAILABLE=false
}

ensure_i2c_tools() {
  # Only needed for the explicit I2C validation scan.
  if command -v i2cdetect >/dev/null 2>&1; then
    return
  fi
  log "Installing i2c-tools (required for PiSugar I2C validation)."
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install -y i2c-tools
}

enable_soft_i2c_bus_if_vehicle() {
  # Your original script tried to force-enable a software I2C bus.
  # We keep that capability, but only on WATNE, and we DO NOT reference undefined vars.
  #
  # Note: If you have NOT physically wired PiSugar to these pins, this bus will not
  # show PiSugar addresses. The validation phase below will report what it sees.

  [[ "${FULL_UPDATE}" == true ]] || return

  local config_file="/boot/config.txt"
  local overlay_line="dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=4,i2c_gpio_scl=5"

  if [[ ! -f "${config_file}" ]]; then
    warn "${config_file} not found; skipping soft I2C overlay."
    return
  fi
  if ! command -v raspi-config >/dev/null 2>&1; then
    warn "raspi-config not found; skipping I2C enablement steps."
    return
  fi

  if ! grep -qxF "${overlay_line}" "${config_file}" 2>/dev/null; then
    local backup_path="${config_file}.bak.$(date +%Y%m%d%H%M%S)"
    log "Backing up ${config_file} to ${backup_path}."
    "${SUDO[@]}" cp "${config_file}" "${backup_path}"
    log "Adding soft I2C overlay (bus 3) to ${config_file}."
    printf '%s\n' "${overlay_line}" | "${SUDO[@]}" tee -a "${config_file}" >/dev/null
    warn "Soft I2C overlay added. A reboot may be required before /dev/i2c-3 is available."
  else
    log "Soft I2C overlay already present."
  fi

  log "Ensuring Raspberry Pi I2C interface is enabled via raspi-config."
  "${SUDO[@]}" raspi-config nonint do_i2c 0 || warn "raspi-config I2C enablement step returned non-zero."
}

detect_pisugar_i2c_bus_and_model() {
  # Uses PiSugar official expected addresses:
  # - PiSugar2: 0x75 & 0x32
  # - PiSugar3: 0x57 & 0x68
  #
  # We scan all /dev/i2c-* busses that exist.
  ensure_i2c_tools

  local dev bus scan has32 has75 has57 has68
  PISUGAR_I2C_BUS=""
  PISUGAR_MODEL=""

  shopt -s nullglob
  for dev in /dev/i2c-*; do
    [[ -e "${dev}" ]] || continue
    bus="${dev##*/i2c-}"
    scan="$("${SUDO[@]}" i2cdetect -y "${bus}" 2>/dev/null || true)"

    has32=false; has75=false; has57=false; has68=false
    if echo "${scan}" | grep -qw "32"; then has32=true; fi
    if echo "${scan}" | grep -qw "75"; then has75=true; fi
    if echo "${scan}" | grep -qw "57"; then has57=true; fi
    if echo "${scan}" | grep -qw "68"; then has68=true; fi

    if [[ "${has32}" == true && "${has75}" == true ]]; then
      PISUGAR_I2C_BUS="${bus}"
      PISUGAR_MODEL="PiSugar2-family (0x32/0x75)"
      return
    fi
    if [[ "${has57}" == true && "${has68}" == true ]]; then
      PISUGAR_I2C_BUS="${bus}"
      PISUGAR_MODEL="PiSugar3-family (0x57/0x68)"
      return
    fi
  done
  shopt -u nullglob
}

pisugar_cmd() {
  # Prefer UDS (/tmp/pisugar-server.sock). Fallback to TCP 127.0.0.1:8423.
  # Per PiSugar docs, both are supported and expose commands like:
  #   get battery
  #   get rtc_time
  #   rtc_rtc2pi / rtc_pi2rtc / rtc_web
  # 
  local cmd="$1"
  local out=""

  if [[ -S /tmp/pisugar-server.sock ]]; then
    if command -v python3 >/dev/null 2>&1; then
      out="$("${SUDO[@]}" python3 - "${cmd}" <<'PY'
import socket, sys
cmd = (sys.argv[1] + "\n").encode("utf-8", errors="ignore")
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(1.5)
s.connect("/tmp/pisugar-server.sock")
s.sendall(cmd)
chunks = []
while True:
  try:
    data = s.recv(4096)
    if not data:
      break
    chunks.append(data)
  except socket.timeout:
    break
s.close()
sys.stdout.write(b"".join(chunks).decode("utf-8", errors="ignore"))
PY
)"
    elif command -v nc >/dev/null 2>&1; then
      # Use a hard timeout to avoid hanging if server keeps connection open.
      out="$(printf '%s\n' "${cmd}" | "${SUDO[@]}" timeout 2s nc -U /tmp/pisugar-server.sock 2>/dev/null || true)"
    else
      die "PiSugar UDS present but neither python3 nor nc available to query it."
    fi
  else
    if command -v python3 >/dev/null 2>&1; then
      out="$("${SUDO[@]}" python3 - "${cmd}" <<'PY'
import socket, sys
cmd = (sys.argv[1] + "\n").encode("utf-8", errors="ignore")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(1.5)
s.connect(("127.0.0.1", 8423))
s.sendall(cmd)
chunks = []
while True:
  try:
    data = s.recv(4096)
    if not data:
      break
    chunks.append(data)
  except socket.timeout:
    break
s.close()
sys.stdout.write(b"".join(chunks).decode("utf-8", errors="ignore"))
PY
)"
    elif command -v nc >/dev/null 2>&1; then
      out="$(printf '%s\n' "${cmd}" | "${SUDO[@]}" timeout 2s nc -q 0 127.0.0.1 8423 2>/dev/null || true)"
    else
      die "Neither python3 nor nc available to query PiSugar TCP API on 127.0.0.1:8423."
    fi
  fi

  # Normalize whitespace a bit
  echo "${out}" | sed -e 's/\r$//' | sed -e '/^[[:space:]]*$/d'
}

ensure_pisugar_validated() {
  # On WATNE we treat PiSugar verification as REQUIRED.
  [[ "${FULL_UPDATE}" == true ]] || return 0

  detect_pisugar_server
  if [[ "${PISUGAR_SOFTWARE_AVAILABLE}" != true ]]; then
    die "pisugar-server.service not found, but WATNE expects PiSugar telemetry/RTC support."
  fi

  log "Ensuring pisugar-server is active."
  "${SUDO[@]}" systemctl start pisugar-server.service || true

  # Verify the command interface returns sane responses
  local model_out battery_out rtc_out
  model_out="$(pisugar_cmd "get model" || true)"
  battery_out="$(pisugar_cmd "get battery" || true)"
  rtc_out="$(pisugar_cmd "get rtc_time" || true)"

  if ! echo "${battery_out}" | grep -q "^battery:"; then
    die "PiSugar validation failed: 'get battery' did not return expected output. Output was: ${battery_out}"
  fi
  if ! echo "${rtc_out}" | grep -q "^rtc_time:"; then
    die "PiSugar validation failed: 'get rtc_time' did not return expected output. Output was: ${rtc_out}"
  fi

  PISUGAR_MODEL="$(echo "${model_out}" | sed -n 's/^model:[[:space:]]*//p' | head -n1 || true)"
  if [[ -z "${PISUGAR_MODEL}" ]]; then
    PISUGAR_MODEL="(unknown model string)"
  fi

  PISUGAR_BATTERY_PCT="$(echo "${battery_out}" | awk -F': ' '/^battery:/{print $2}' | head -n1 || true)"
  PISUGAR_VALIDATED=true

  log "PiSugar server validated. Model: ${PISUGAR_MODEL}. Battery: ${PISUGAR_BATTERY_PCT}%."
}

pisugar_rtc_sync_sequence() {
  # Deterministic sync:
  #  1) rtc_rtc2pi (RTC -> Pi) so the Pi time is immediately plausible
  #  2) after internet is confirmed, rtc_web (Web -> RTC & Pi) to guarantee both match
  #
  # Commands are defined in PiSugar docs. 
  [[ "${PISUGAR_VALIDATED}" == true ]] || return 0

  log "PiSugar RTC pre-sync: applying rtc_rtc2pi (RTC => Pi)."
  pisugar_cmd "rtc_rtc2pi" >/dev/null || warn "rtc_rtc2pi returned non-zero (continuing)."

  log "System time after rtc_rtc2pi: $(date --iso-8601=seconds 2>/dev/null || date)"
  log "PiSugar RTC after rtc_rtc2pi: $(pisugar_cmd "get rtc_time" | tr '\n' ' ' || true)"
}

pisugar_rtc_web_sync_and_verify() {
  [[ "${PISUGAR_VALIDATED}" == true ]] || return 0

  log "PiSugar RTC authoritative sync: applying rtc_web (Web => RTC & Pi)."
  pisugar_cmd "rtc_web" >/dev/null || die "rtc_web failed; cannot guarantee RTC+Pi are synchronized."

  local rtc_line rtc_iso sys_epoch rtc_epoch delta
  rtc_line="$(pisugar_cmd "get rtc_time" | awk -F': ' '/^rtc_time:/{print $2; exit}' || true)"
  [[ -n "${rtc_line}" ]] || die "Unable to read rtc_time after rtc_web."

  # Compare system time vs RTC time (allow small drift)
  sys_epoch="$(date +%s)"
  rtc_epoch="$(date -d "${rtc_line}" +%s 2>/dev/null || true)"
  if [[ -z "${rtc_epoch}" ]]; then
    warn "Could not parse rtc_time ('${rtc_line}') via date -d; skipping drift check."
    return 0
  fi

  if (( sys_epoch > rtc_epoch )); then delta=$((sys_epoch - rtc_epoch)); else delta=$((rtc_epoch - sys_epoch)); fi
  if (( delta > 120 )); then
    die "RTC sync validation failed: system vs PiSugar RTC differ by ${delta}s after rtc_web."
  fi

  log "RTC sync validation passed (system vs PiSugar RTC delta ${delta}s)."
}

pisugar_report_battery() {
  [[ "${PISUGAR_VALIDATED}" == true ]] || return 0
  local battery_out pct
  battery_out="$(pisugar_cmd "get battery" || true)"
  pct="$(echo "${battery_out}" | awk -F': ' '/^battery:/{print $2; exit}' || true)"
  [[ -n "${pct}" ]] || die "Unable to parse PiSugar battery percentage from: ${battery_out}"
  log "PiSugar battery percentage (end of test): ${pct}%"
}

# --- main --------------------------------------------------------------------

main() {
  require_command git
  require_command apt-get
  require_command ip
  require_command curl
  require_command systemctl
  require_command hostname

  check_hostname
  detect_uplink_service
  record_uplink_state
  record_wwan_state

  trap '' HUP
  trap cleanup EXIT

  # If WATNE: validate PiSugar early and do RTC->Pi sync before any TLS network ops.
  if [[ "${FULL_UPDATE}" == true ]]; then
    # Try to ensure soft I2C is configured (vehicle-only); does not hard-fail if not applicable.
    enable_soft_i2c_bus_if_vehicle

    # Validate PiSugar server path (end-to-end I2C read via server).
    ensure_pisugar_validated

    # Optional explicit I2C scan validation (also vehicle-only).
    detect_pisugar_i2c_bus_and_model
    if [[ -n "${PISUGAR_I2C_BUS}" ]]; then
      log "PiSugar I2C devices detected on /dev/i2c-${PISUGAR_I2C_BUS} (${PISUGAR_MODEL})."
    else
      warn "PiSugar I2C addresses not detected via i2cdetect. If using pogo pins, check contact/solder mask per PiSugar FAQ."
    fi

    # Use RTC immediately for plausible system time.
    pisugar_rtc_sync_sequence
  else
    log "Skipping PiSugar validation steps on non-vehicle host."
  fi

  # Now confirm we have non-wwan internet for update.
  check_non_wwan_internet

  # Disable uplink + wwan during update (as your original intent)
  if [[ "${UPLINK_AVAILABLE}" == true ]]; then
    log "Stopping and disabling uplink.service for the update."
    "${SUDO[@]}" systemctl stop uplink.service
    if [[ "${UPLINK_WAS_ENABLED}" == "enabled" ]]; then
      "${SUDO[@]}" systemctl disable uplink.service
    else
      log "uplink.service already disabled or not enable-able (${UPLINK_WAS_ENABLED})."
    fi
  else
    log "uplink.service not found; skipping."
  fi

  if [[ "${WWAN_PRESENT}" == true ]]; then
    log "Bringing down wwan0 for the update."
    "${SUDO[@]}" ip link set wwan0 down
  else
    log "wwan0 not present; skipping."
  fi

  update_repo
  ensure_scripts_executable
  update_system_packages

  if [[ "${FULL_UPDATE}" == true ]]; then
    prompt_service_replacement
  else
    log "Skipping service installation on non-vehicle host."
  fi

  # Final authoritative RTC sync + verification + battery report (vehicle only)
  if [[ "${FULL_UPDATE}" == true ]]; then
    pisugar_rtc_web_sync_and_verify
    pisugar_report_battery
  fi

  log "Update routine completed."
}

main "$@"

