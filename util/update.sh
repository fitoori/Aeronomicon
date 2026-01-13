#!/usr/bin/env bash
set -euo pipefail

DIAG_FILE=""

init_diag_log() {
  local dir="/var/log/aeronomicon"
  "${SUDO[@]}" mkdir -p "${dir}" || true
  DIAG_FILE="${dir}/update-pisugar-$(date +%Y%m%d%H%M%S).log"
  "${SUDO[@]}" touch "${DIAG_FILE}" || true
  "${SUDO[@]}" chmod 0644 "${DIAG_FILE}" || true
  log "PiSugar diagnostic log: ${DIAG_FILE}"
}

log_to_diag() {
  # Best-effort append to diag file; never fail update because logging failed.
  local line="$1"
  if [[ -n "${DIAG_FILE}" ]]; then
    printf '%s\n' "${line}" | "${SUDO[@]}" tee -a "${DIAG_FILE}" >/dev/null 2>&1 || true
  fi
}

log()  { local m="[update] $*"; echo "${m}"; log_to_diag "${m}"; }
warn() { local m="[update][WARN] $*"; echo "${m}" >&2; log_to_diag "${m}"; }
die()  { local m="[update][ERROR] $*"; echo "${m}" >&2; log_to_diag "${m}"; exit 1; }

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

PLYMOUTH_AVAILABLE=false

# PiSugar runtime state
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

detect_plymouth() {
  if command -v plymouthd >/dev/null 2>&1; then
    PLYMOUTH_AVAILABLE=true
    return
  fi
  if command -v plymouth-set-default-theme >/dev/null 2>&1; then
    PLYMOUTH_AVAILABLE=true
    return
  fi
  if command -v dpkg >/dev/null 2>&1; then
    if dpkg -s plymouth >/dev/null 2>&1; then
      PLYMOUTH_AVAILABLE=true
    fi
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

run_plymouth_installer() {
  local installer="${REPO_DIR}/util/plymouth/install.sh"
  [[ -x "${installer}" ]] || die "Plymouth installer not found or not executable: ${installer}"
  log "Installing Plymouth boot theme."
  "${SUDO[@]}" bash -c "cd '${REPO_DIR}' && exec '${installer}'"
}

prompt_plymouth_install() {
  if [[ "${PLYMOUTH_AVAILABLE}" != true ]]; then
    log "Plymouth not detected; skipping boot theme install offer."
    return
  fi

  local response="n"
  if [[ -r /dev/tty ]]; then
    read -r -p "Install Aeronomicon Plymouth boot theme? [y/N] " response </dev/tty || response="n"
  elif [[ -t 0 ]]; then
    read -r -p "Install Aeronomicon Plymouth boot theme? [y/N] " response || response="n"
  fi

  case "${response}" in
    [Yy]|[Yy][Ee][Ss])
      run_plymouth_installer
      ;;
    *)
      log "Skipping Plymouth boot theme installation."
      ;;
  esac
}

update_system_packages() {
  log "Updating apt package lists."
  "${SUDO[@]}" apt-get update
  log "Upgrading installed packages."
  "${SUDO[@]}" apt-get upgrade -y
  log "Autoremoving unused packages."
  "${SUDO[@]}" apt-get autoremove -y
}

ensure_apt_pkg() {
  local pkg="$1"
  if command -v dpkg >/dev/null 2>&1; then
    if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
      log "Installing required package: ${pkg}"
      "${SUDO[@]}" apt-get update
      "${SUDO[@]}" apt-get install -y "${pkg}"
    fi
  else
    warn "dpkg not available; cannot ensure package ${pkg}"
  fi
}

dump_cmd() {
  # Usage: dump_cmd "title" cmd args...
  local title="$1"; shift
  log "---- ${title} ----"
  log_to_diag "---- ${title} ----"
  # shellcheck disable=SC2068
  ( "$@" 2>&1 || true ) | while IFS= read -r line; do
    log_to_diag "${line}"
  done
  # Also print short to console if useful:
  "$@" 2>/dev/null || true
}

# --- PiSugar support ---------------------------------------------------------

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

ensure_kernel_i2c_modules() {
  "${SUDO[@]}" modprobe i2c-dev >/dev/null 2>&1 || true
  # Different kernels name these differently; harmless if missing.
  "${SUDO[@]}" modprobe i2c-bcm2835 >/dev/null 2>&1 || true
  "${SUDO[@]}" modprobe i2c-bcm2708 >/dev/null 2>&1 || true
}

boot_config_enable_i2c_arm() {
  local cfg="/boot/config.txt"
  [[ -f "${cfg}" ]] || { warn "${cfg} not found; cannot persistently enable I2C"; return 1; }

  local changed=false
  if grep -Eq '^\s*dtparam=i2c_arm=on\b' "${cfg}"; then
    :
  else
    # Flip off->on if present, otherwise append.
    if grep -Eq '^\s*dtparam=i2c_arm=off\b' "${cfg}"; then
      log "Setting dtparam=i2c_arm=on in ${cfg}"
      "${SUDO[@]}" cp "${cfg}" "${cfg}.bak.$(date +%Y%m%d%H%M%S)"
      "${SUDO[@]}" sed -i 's/^\s*dtparam=i2c_arm=off\b/dtparam=i2c_arm=on/' "${cfg}"
      changed=true
    else
      log "Appending dtparam=i2c_arm=on to ${cfg}"
      "${SUDO[@]}" cp "${cfg}" "${cfg}.bak.$(date +%Y%m%d%H%M%S)"
      printf '%s\n' "dtparam=i2c_arm=on" | "${SUDO[@]}" tee -a "${cfg}" >/dev/null
      changed=true
    fi
  fi

  if [[ "${changed}" == true ]]; then
    echo "BOOTCFG_CHANGED"
  fi
}

ensure_i2c_runtime_ready_or_reboot() {
  # Returns:
  #   0: runtime I2C is ready (at least /dev/i2c-1 exists)
  #   2: changes were made that require reboot (I2C not active yet)
  #
  ensure_apt_pkg i2c-tools
  ensure_kernel_i2c_modules

  if [[ -e /dev/i2c-1 ]]; then
    return 0
  fi

  # Try enabling via raspi-config (persistent) + config.txt fix.
  if command -v raspi-config >/dev/null 2>&1; then
    log "Enabling I2C via raspi-config (persistent)."
    "${SUDO[@]}" raspi-config nonint do_i2c 0 || warn "raspi-config do_i2c failed (continuing)."
  else
    warn "raspi-config not found; will only edit /boot/config.txt."
  fi

  boot_config_enable_i2c_arm || true
  ensure_kernel_i2c_modules

  # If still not present, reboot is required for DT changes to take effect.
  if [[ ! -e /dev/i2c-1 ]]; then
    log "I2C device node /dev/i2c-1 not present after enablement. Reboot required."
    return 2
  fi

  # If boot config changed but device node exists now, it might still be prudent to reboot,
  # but we do not force it.
  return 0
}

i2c_scan_all_busses() {
  ensure_apt_pkg i2c-tools
  ensure_kernel_i2c_modules

  local dev bus
  shopt -s nullglob
  for dev in /dev/i2c-*; do
    bus="${dev##*/i2c-}"
    log "I2C scan: bus ${bus} (${dev})"
    log_to_diag "i2cdetect -y ${bus}"
    "${SUDO[@]}" i2cdetect -y "${bus}" 2>&1 | tee -a /dev/null | while IFS= read -r line; do
      log_to_diag "${line}"
    done
  done
  shopt -u nullglob
}

i2c_has_addr_in_table() {
  # Usage: i2c_has_addr_in_table "<table>" "57"
  local table="$1"
  local addr="$2"
  echo "${table}" | grep -qw "${addr}"
}

i2c_read_byte_reg() {
  # Usage: i2c_read_byte_reg <bus> <addr_hex_no0x> <reg_hex_no0x>
  # Returns: hex byte like "0x1a" on stdout, empty on failure.
  local bus="$1" addr="$2" reg="$3"
  "${SUDO[@]}" i2cget -y "${bus}" "0x${addr}" "0x${reg}" 2>/dev/null || true
}

hex_to_dec() {
  # "0x1a" -> 26 ; "1a" -> 26
  local h="${1#0x}"
  printf '%d' "$((16#${h}))"
}

count_ones_7bit() {
  local x="$1" c=0
  # x should be 0..127
  while (( x )); do
    ((c += x & 1))
    ((x >>= 1))
  done
  echo "${c}"
}

pisugar3_signature_score() {
  # Read-only signature check against PiSugar3 register map.
  # Returns 0 if strong match, 1 otherwise.
  # Uses regs documented in PiSugar3 I2C datasheet. 
  local bus="$1" addr_hex="$2"
  local addr_dec ones parity expected50 reg50 reg2a reg22 reg23
  local bat_dec vh vl mv

  addr_dec="$(hex_to_dec "${addr_hex}")"
  reg50="$(i2c_read_byte_reg "${bus}" "${addr_hex}" "50")"
  reg2a="$(i2c_read_byte_reg "${bus}" "${addr_hex}" "2a")"
  reg22="$(i2c_read_byte_reg "${bus}" "${addr_hex}" "22")"
  reg23="$(i2c_read_byte_reg "${bus}" "${addr_hex}" "23")"

  [[ -n "${reg50}" && -n "${reg2a}" && -n "${reg22}" && -n "${reg23}" ]] || return 1

  # Validate 0x50 parity-encoded address (datasheet example: 0x57 -> 0xD7). 
  ones="$(count_ones_7bit "${addr_dec}")"
  parity=$(( ones % 2 ))                 # odd -> 1, even -> 0
  expected50=$(( addr_dec | (parity << 7) ))
  if [[ "$(hex_to_dec "${reg50}")" -ne "${expected50}" ]]; then
    return 1
  fi

  # Battery percent 0x2A should be 0-100. 
  bat_dec="$(hex_to_dec "${reg2a}")"
  if (( bat_dec < 0 || bat_dec > 100 )); then
    return 1
  fi

  # Voltage bytes 0x22/0x23 should combine into plausible mV. 
  vh="$(hex_to_dec "${reg22}")"
  vl="$(hex_to_dec "${reg23}")"
  mv=$(( (vh << 8) | vl ))
  if (( mv < 2500 || mv > 5000 )); then
    return 1
  fi

  return 0
}

set_boot_i2c1_baudrate() {
  # Ensures dtparam=i2c1_baudrate=<target> in /boot/config.txt.
  local target="$1"
  local cfg="/boot/config.txt"
  [[ -f "${cfg}" ]] || die "/boot/config.txt not found"

  local backup="${cfg}.bak.$(date +%Y%m%d%H%M%S)"
  "${SUDO[@]}" cp "${cfg}" "${backup}"

  # Replace any of the common aliases, else append.
  if grep -Eq '^\s*dtparam=(i2c1_baudrate|i2c_arm_baudrate|i2c_baudrate)=' "${cfg}"; then
    "${SUDO[@]}" sed -i \
      -E "s/^\s*dtparam=(i2c1_baudrate|i2c_arm_baudrate|i2c_baudrate)=.*/dtparam=i2c1_baudrate=${target}/" \
      "${cfg}"
  else
    printf '%s\n' "dtparam=i2c1_baudrate=${target}" | "${SUDO[@]}" tee -a "${cfg}" >/dev/null
  fi

  log "Updated I2C1 baudrate in ${cfg} (backup: ${backup}) to ${target}."
}

get_boot_i2c1_baudrate() {
  local cfg="/boot/config.txt"
  [[ -f "${cfg}" ]] || return 0
  # return last seen value if multiple
  grep -E '^\s*dtparam=(i2c1_baudrate|i2c_arm_baudrate|i2c_baudrate)=' "${cfg}" \
    | tail -n1 \
    | awk -F'=' '{print $NF}' \
    | tr -dc '0-9' \
    || true
}

set_pisugar_server_i2c_addr() {
  # Sets/creates i2c_addr in /etc/pisugar-server/config.json (decimal).
  # PiSugar docs: if i2c_addr isn't present, add it. 
  local addr_dec="$1"
  local cfg="/etc/pisugar-server/config.json"
  [[ -f "${cfg}" ]] || die "${cfg} not found (pisugar-server installed but config missing?)"

  require_command python3

  local backup="${cfg}.bak.$(date +%Y%m%d%H%M%S)"
  "${SUDO[@]}" cp "${cfg}" "${backup}"

  "${SUDO[@]}" python3 - "${cfg}" "${addr_dec}" <<'PY'
import json, sys
path=sys.argv[1]
addr=int(sys.argv[2])
with open(path,"r",encoding="utf-8") as f:
    data=json.load(f)
data["i2c_addr"]=addr
with open(path,"w",encoding="utf-8") as f:
    json.dump(data,f,indent=2)
    f.write("\n")
PY

  log "Set pisugar-server i2c_addr=${addr_dec} (backup: ${backup})."
}

find_pisugar3_bus() {
  # PiSugar 3 Power Manager occupies 0x57 & 0x68.
  # We treat presence of 0x57 as the primary indicator.
  local dev bus scan
  shopt -s nullglob
  for dev in /dev/i2c-*; do
    bus="${dev##*/i2c-}"
    scan="$("${SUDO[@]}" i2cdetect -y "${bus}" 2>/dev/null || true)"
    if echo "${scan}" | grep -qw "57"; then
      echo "${bus}"
      shopt -u nullglob
      return 0
    fi
  done
  shopt -u nullglob
  return 1
}

pisugar_server_user() {
  local u
  u="$(systemctl show -p User --value pisugar-server.service 2>/dev/null || true)"
  # Empty means root in many units; normalize.
  if [[ -z "${u}" || "${u}" == "root" || "${u}" == "0" ]]; then
    echo "root"
  else
    echo "${u}"
  fi
}

ensure_pisugar_service_i2c_permissions() {
  local u
  u="$(pisugar_server_user)"
  log "pisugar-server.service User=${u}"

  if [[ "${u}" == "root" ]]; then
    return 0
  fi

  if ! getent group i2c >/dev/null 2>&1; then
    warn "Group 'i2c' not found on this system; cannot grant ${u} I2C access via groups."
    return 0
  fi

  if id -nG "${u}" 2>/dev/null | tr ' ' '\n' | grep -qx "i2c"; then
    return 0
  fi

  log "Adding ${u} to group i2c (required to access /dev/i2c-*)."
  "${SUDO[@]}" usermod -a -G i2c "${u}" || warn "usermod failed; service may still lack I2C permissions."
}

update_pisugar_config_json_bus() {
  local bus="$1"
  local cfg="/etc/pisugar-server/config.json"

  if [[ ! -f "${cfg}" ]]; then
    warn "${cfg} not found; skipping i2c_bus fix."
    return 0
  fi

  ensure_apt_pkg python3

  log "Setting i2c_bus=${bus} in ${cfg}"
  "${SUDO[@]}" cp "${cfg}" "${cfg}.bak.$(date +%Y%m%d%H%M%S)"
  "${SUDO[@]}" python3 - "${cfg}" "${bus}" <<'PY'
import json, sys
path=sys.argv[1]
bus=int(sys.argv[2])
with open(path,"r",encoding="utf-8") as f:
    data=json.load(f)
data["i2c_bus"]=bus
with open(path,"w",encoding="utf-8") as f:
    json.dump(data,f,indent=2)
    f.write("\n")
PY
}

restart_pisugar_server() {
  log "Restarting pisugar-server.service"
  "${SUDO[@]}" systemctl restart pisugar-server.service || true
  "${SUDO[@]}" systemctl --no-pager --full status pisugar-server.service >/dev/null 2>&1 || true
}

detect_pisugar_i2c_bus_and_model() {
  # Uses PiSugar official expected addresses:
  # - PiSugar2: 0x75 & 0x32
  # - PiSugar3: 0x57 & 0x68
  #
  # We scan all /dev/i2c-* busses that exist.
  ensure_apt_pkg i2c-tools
  ensure_kernel_i2c_modules

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
  # UDS preferred, TCP fallback. Commands are documented (get battery/get model).
  local cmd="$1"
  local out=""

  if [[ -S /tmp/pisugar-server.sock ]]; then
    if command -v python3 >/dev/null 2>&1; then
      out="$("${SUDO[@]}" python3 - "${cmd}" <<'PY'
import socket, sys
cmd=(sys.argv[1]+"\n").encode()
s=socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(1.5)
s.connect("/tmp/pisugar-server.sock")
s.sendall(cmd)
chunks=[]
while True:
  try:
    d=s.recv(4096)
    if not d: break
    chunks.append(d)
  except socket.timeout:
    break
s.close()
print(b"".join(chunks).decode(errors="ignore"), end="")
PY
)"
    else
      out="$(printf '%s\n' "${cmd}" | "${SUDO[@]}" timeout 2s nc -U /tmp/pisugar-server.sock 2>/dev/null || true)"
    fi
  else
    if command -v python3 >/dev/null 2>&1; then
      out="$("${SUDO[@]}" python3 - "${cmd}" <<'PY'
import socket, sys
cmd=(sys.argv[1]+"\n").encode()
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(1.5)
s.connect(("127.0.0.1", 8423))
s.sendall(cmd)
chunks=[]
while True:
  try:
    d=s.recv(4096)
    if not d: break
    chunks.append(d)
  except socket.timeout:
    break
s.close()
print(b"".join(chunks).decode(errors="ignore"), end="")
PY
)"
    else
      out="$(printf '%s\n' "${cmd}" | "${SUDO[@]}" timeout 2s nc -q 0 127.0.0.1 8423 2>/dev/null || true)"
    fi
  fi

  echo "${out}" | sed -e 's/\r$//' | sed -e '/^[[:space:]]*$/d'
}

pisugar_health_check() {
  # Returns 0 only if battery is numeric and rtc_time is parseable.
  local battery_out pct rtc_out rtc_str

  battery_out="$(pisugar_cmd "get battery" || true)"
  pct="$(echo "${battery_out}" | awk -F': ' '/^battery:/{print $2; exit}' || true)"

  # reject "I2C not connected"
  if [[ -z "${pct}" || ! "${pct}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
    return 1
  fi

  rtc_out="$(pisugar_cmd "get rtc_time" || true)"
  rtc_str="$(echo "${rtc_out}" | awk -F': ' '/^rtc_time:/{print $2; exit}' || true)"
  if [[ -z "${rtc_str}" ]]; then
    return 1
  fi
  if ! date -d "${rtc_str}" >/dev/null 2>&1; then
    return 1
  fi

  PISUGAR_BATTERY_PCT="${pct}"
  return 0
}

pisugar_collect_failure_bundle() {
  log "Collecting PiSugar failure diagnostics (see ${DIAG_FILE})."
  dump_cmd "OS release" bash -c 'cat /etc/os-release'
  dump_cmd "Kernel" uname -a
  dump_cmd "I2C dev nodes" bash -c 'ls -l /dev/i2c-* 2>/dev/null || true'
  dump_cmd "Loaded I2C modules" bash -c 'lsmod | grep -i i2c || true'
  dump_cmd "boot config i2c lines" bash -c "grep -nE '^(dtparam=i2c|dtoverlay=.*i2c)' /boot/config.txt 2>/dev/null || true"
  dump_cmd "pisugar-server unit" systemctl cat pisugar-server.service
  dump_cmd "pisugar-server recent logs" bash -c 'journalctl -u pisugar-server.service -n 120 --no-pager || true'
  i2c_scan_all_busses
  dump_cmd "pisugar get model" bash -c 'echo "get model" | nc -U -q 0 /tmp/pisugar-server.sock 2>/dev/null || true'
  dump_cmd "pisugar get battery" bash -c 'echo "get battery" | nc -U -q 0 /tmp/pisugar-server.sock 2>/dev/null || true'
  dump_cmd "pisugar get rtc_time" bash -c 'echo "get rtc_time" | nc -U -q 0 /tmp/pisugar-server.sock 2>/dev/null || true'
}

pisugar_repair_and_validate() {
  [[ "${FULL_UPDATE}" == true ]] || return 0

  init_diag_log

  # Ensure i2c-tools exists for i2cdetect/i2cdump per PiSugar docs. 
  ensure_apt_pkg i2c-tools

  log "PiSugar remediation: probing I2C bus 1 (expected PiSugar3 addrs: 0x57 & 0x68)."
  local t_quick t_read
  t_quick="$("${SUDO[@]}" i2cdetect -y 1 2>/dev/null || true)"
  t_read="$("${SUDO[@]}" i2cdetect -y -r 1 2>/dev/null || true)"
  log_to_diag "i2cdetect -y 1\n${t_quick}"
  log_to_diag "i2cdetect -y -r 1\n${t_read}"

  local seen57=false seen68=false
  if i2c_has_addr_in_table "${t_quick}" "57" || i2c_has_addr_in_table "${t_read}" "57"; then seen57=true; fi
  if i2c_has_addr_in_table "${t_quick}" "68" || i2c_has_addr_in_table "${t_read}" "68"; then seen68=true; fi

  if [[ "${seen57}" == true || "${seen68}" == true ]]; then
    log "PiSugar addresses appear present on I2C bus 1 (57=${seen57}, 68=${seen68}). Proceeding to server validation."
    restart_pisugar_server
    if pisugar_health_check; then
      log "PiSugar OK. Battery=${PISUGAR_BATTERY_PCT}%"
      return 0
    fi
  fi

  # Vendor recommended deeper probe: i2cdump expected addrs. 
  log "PiSugar not visible in i2cdetect; attempting vendor i2cdump probes (read-only)."
  "${SUDO[@]}" i2cdump -y 1 0x57 >/dev/null 2>&1 || true
  "${SUDO[@]}" i2cdump -y 1 0x68 >/dev/null 2>&1 || true

  # If server now works, stop here.
  "${SUDO[@]}" systemctl start pisugar-server.service >/dev/null 2>&1 || true
  if pisugar_health_check; then
    log "PiSugar restored after deeper probe. Battery=${PISUGAR_BATTERY_PCT}%"
    return 0
  fi

  # Phase B: bus clock remediation (your config shows 1MHz). This is the highest-probability software fix.
  local br
  br="$(get_boot_i2c1_baudrate || true)"
  if [[ -n "${br}" && "${br}" -gt 400000 ]]; then
    warn "I2C1 baudrate is ${br} (>400000). PiSugar3 may not ACK reliably. Downgrading to 400000 and requiring reboot."
    set_boot_i2c1_baudrate 400000

    local response="n"
    if [[ -r /dev/tty ]]; then
      read -r -p "Reboot now to apply I2C baudrate change and re-test PiSugar? [y/N] " response </dev/tty || response="n"
    fi
    if [[ "${response}" =~ ^[Yy]([Ee][Ss])?$ ]]; then
      log "Rebooting now. Re-run update after boot; PiSugar probe will retry automatically."
      "${SUDO[@]}" shutdown -r now "Apply I2C baudrate change for PiSugar"
      exit 0
    fi
    die "Reboot required to apply I2C baudrate change; cannot proceed on WATNE without PiSugar telemetry."
  fi

  # Phase C: mutable address detection (read-only identification via PiSugar3 register map). 
  log "Attempting PiSugar3 mutable-address detection on I2C bus 1 (read-only)."
  local addrs addr
  addrs="$( (echo "${t_read}"; echo "${t_quick}") \
    | awk 'NR>1 {for(i=2;i<=NF;i++) if($i!="--" && $i!="UU") print $i}' \
    | sort -u )"

  while IFS= read -r addr; do
    [[ -n "${addr}" ]] || continue
    if pisugar3_signature_score 1 "${addr}"; then
      local addr_dec
      addr_dec="$(hex_to_dec "${addr}")"
      warn "Found PiSugar3-like signature at I2C addr 0x${addr}. Updating pisugar-server i2c_addr=${addr_dec} and restarting."
      set_pisugar_server_i2c_addr "${addr_dec}"
      restart_pisugar_server
      if pisugar_health_check; then
        log "PiSugar restored via mutable-address fix. Battery=${PISUGAR_BATTERY_PCT}%"
        return 0
      fi
    fi
  done <<< "${addrs}"

  pisugar_collect_failure_bundle
  die "PiSugar still not reachable on I2C. Expected 0x57/0x68 per PiSugar docs. See ${DIAG_FILE}."
}

ensure_pisugar_validated() {
  # On WATNE we treat PiSugar verification as REQUIRED.
  [[ "${FULL_UPDATE}" == true ]] || return 0

  pisugar_repair_and_validate

  local model_out
  model_out="$(pisugar_cmd "get model" || true)"
  PISUGAR_MODEL="$(echo "${model_out}" | sed -n 's/^model:[[:space:]]*//p' | head -n1 || true)"
  if [[ -z "${PISUGAR_MODEL}" ]]; then
    PISUGAR_MODEL="(unknown model string)"
  fi

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
  detect_plymouth
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
  prompt_plymouth_install

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
