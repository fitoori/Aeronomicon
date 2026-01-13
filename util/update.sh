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

  # Ensure service exists
  local load_state
  load_state="$(systemctl show -p LoadState --value pisugar-server.service 2>/dev/null || true)"
  if [[ -z "${load_state}" || "${load_state}" == "not-found" ]]; then
    warn "pisugar-server.service not installed on WATNE; skipping PiSugar validation."
    return 1
  fi

  # Start service (if stopped)
  "${SUDO[@]}" systemctl start pisugar-server.service || true

  # If already healthy, done.
  if pisugar_health_check; then
    log "PiSugar OK. Battery=${PISUGAR_BATTERY_PCT}%"
    return 0
  fi

  log "PiSugar reports I2C failure (e.g., 'I2C not connected'). Starting software remediation."

  # 1) Ensure runtime I2C exists; if not, enable and require reboot.
  local i2c_state
  if ! i2c_state="$(ensure_i2c_runtime_ready_or_reboot; echo $?)"; then
    :
  fi
  if [[ "${i2c_state}" == "2" ]]; then
    pisugar_collect_failure_bundle
    warn "I2C was enabled in boot config but is not active yet. Reboot is required to bring up /dev/i2c-1."

    local response="n"
    if [[ -r /dev/tty ]]; then
      read -r -p "Reboot now to activate I2C and retry PiSugar validation? [y/N] " response </dev/tty || response="n"
    fi
    if [[ "${response}" =~ ^[Yy]([Ee][Ss])?$ ]]; then
      log "Rebooting now (required for I2C activation). Re-run update after boot."
      "${SUDO[@]}" shutdown -r now "Reboot required to activate I2C for PiSugar"
      exit 0
    fi
    warn "Cannot proceed on WATNE until I2C is active and PiSugar is readable."
    return 1
  fi

  # 2) Ensure pisugar-server has I2C permissions.
  ensure_pisugar_service_i2c_permissions

  # 3) Find which I2C bus has PiSugar (0x57 expected).
  local bus
  bus="$(find_pisugar3_bus || true)"
  if [[ -z "${bus}" ]]; then
    pisugar_collect_failure_bundle
    warn "PiSugar3 I2C address 0x57 not detected on any /dev/i2c-* bus. Software remediation exhausted."
    return 1
  fi
  log "Detected PiSugar3 on I2C bus ${bus} (0x57 present)."

  # 4) Force pisugar-server config to use that bus (i2c_bus is supported in config.json).
  update_pisugar_config_json_bus "${bus}"

  # 5) Restart service and re-check health.
  restart_pisugar_server
  if pisugar_health_check; then
    log "PiSugar restored. Battery=${PISUGAR_BATTERY_PCT}%"
    return 0
  fi

  # Final: diagnostics + fail hard.
  pisugar_collect_failure_bundle
  warn "PiSugar still reports I2C failure after remediation. See ${DIAG_FILE} for full diagnostics."
  return 1
}

ensure_pisugar_validated() {
  # On WATNE we treat PiSugar verification as REQUIRED.
  [[ "${FULL_UPDATE}" == true ]] || return 0

  if ! pisugar_repair_and_validate; then
    warn "PiSugar validation failed; continuing update without PiSugar integration."
    return 1
  fi

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
  if ! pisugar_cmd "rtc_web" >/dev/null; then
    warn "rtc_web failed; cannot guarantee RTC+Pi are synchronized."
    return 1
  fi

  local rtc_line rtc_iso sys_epoch rtc_epoch delta
  rtc_line="$(pisugar_cmd "get rtc_time" | awk -F': ' '/^rtc_time:/{print $2; exit}' || true)"
  if [[ -z "${rtc_line}" ]]; then
    warn "Unable to read rtc_time after rtc_web."
    return 1
  fi

  # Compare system time vs RTC time (allow small drift)
  sys_epoch="$(date +%s)"
  rtc_epoch="$(date -d "${rtc_line}" +%s 2>/dev/null || true)"
  if [[ -z "${rtc_epoch}" ]]; then
    warn "Could not parse rtc_time ('${rtc_line}') via date -d; skipping drift check."
    return 0
  fi

  if (( sys_epoch > rtc_epoch )); then delta=$((sys_epoch - rtc_epoch)); else delta=$((rtc_epoch - sys_epoch)); fi
  if (( delta > 120 )); then
    warn "RTC sync validation failed: system vs PiSugar RTC differ by ${delta}s after rtc_web."
    return 1
  fi

  log "RTC sync validation passed (system vs PiSugar RTC delta ${delta}s)."
}

pisugar_report_battery() {
  [[ "${PISUGAR_VALIDATED}" == true ]] || return 0
  local battery_out pct
  battery_out="$(pisugar_cmd "get battery" || true)"
  pct="$(echo "${battery_out}" | awk -F': ' '/^battery:/{print $2; exit}' || true)"
  if [[ -z "${pct}" ]]; then
    warn "Unable to parse PiSugar battery percentage from: ${battery_out}"
    return 1
  fi
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

  # Run PiSugar checks last to avoid interfering with the update sequence.
  if [[ "${FULL_UPDATE}" == true ]]; then
    # Try to ensure soft I2C is configured (vehicle-only); does not hard-fail if not applicable.
    enable_soft_i2c_bus_if_vehicle || true

    # Validate PiSugar server path (end-to-end I2C read via server).
    ensure_pisugar_validated || true

    # Optional explicit I2C scan validation (also vehicle-only).
    detect_pisugar_i2c_bus_and_model || true
    if [[ -n "${PISUGAR_I2C_BUS}" ]]; then
      log "PiSugar I2C devices detected on /dev/i2c-${PISUGAR_I2C_BUS} (${PISUGAR_MODEL})."
    else
      warn "PiSugar I2C addresses not detected via i2cdetect. If using pogo pins, check contact/solder mask per PiSugar FAQ."
    fi

    # Use RTC immediately for plausible system time.
    pisugar_rtc_sync_sequence || true

    # Final authoritative RTC sync + verification + battery report (vehicle only)
    pisugar_rtc_web_sync_and_verify || true
    pisugar_report_battery || true
  else
    log "Skipping PiSugar validation steps on non-vehicle host."
  fi

  log "Update routine completed."
}

main "$@"
