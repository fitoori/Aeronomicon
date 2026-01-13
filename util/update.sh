#!/usr/bin/env bash
set -euo pipefail

log() {
  echo "[update] $*"
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Required command not found: $1" >&2
    exit 1
  fi
}

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUDO=()
if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  SUDO=(sudo)
fi
UPLINK_AVAILABLE=false
UPLINK_WAS_ACTIVE=""
UPLINK_WAS_ENABLED=""
WWAN_PRESENT=false
WWAN_WAS_UP=false
PISUGAR_PRESENT=false
PISUGAR_PACKAGES=()

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

detect_pisugar_packages() {
  local filter_cmd=()
  if command -v rg >/dev/null 2>&1; then
    filter_cmd=(rg -i '^pisugar')
  else
    filter_cmd=(grep -i '^pisugar')
  fi

  if command -v dpkg-query >/dev/null 2>&1; then
    mapfile -t PISUGAR_PACKAGES < <(dpkg-query -W -f='${Package}\n' | "${filter_cmd[@]}" || true)
  elif command -v rpm >/dev/null 2>&1; then
    mapfile -t PISUGAR_PACKAGES < <(rpm -qa | "${filter_cmd[@]}" || true)
  else
    log "Package manager not detected; skipping PiSugar package detection."
    return
  fi

  if ((${#PISUGAR_PACKAGES[@]} > 0)); then
    PISUGAR_PRESENT=true
    log "Detected PiSugar packages: ${PISUGAR_PACKAGES[*]}"
  else
    log "No PiSugar packages detected."
  fi
}

configure_pisugar_for_navio2() {
  if [[ "${PISUGAR_PRESENT}" != true ]]; then
    log "PiSugar configuration not needed; skipping."
    return
  fi

  local config_files=(
    "/etc/pisugar-server/config.json"
    "/etc/pisugar-server.json"
    "/etc/pisugar.conf"
    "/etc/pisugar/pisugar.conf"
  )
  local config
  local found_configs=()

  for config in "${config_files[@]}"; do
    if [[ -f "${config}" ]]; then
      found_configs+=("${config}")
    fi
  done

  if ((${#found_configs[@]} == 0)); then
  local updated=false
  local config
  local has_config=false

  for config in "${config_files[@]}"; do
    if [[ ! -f "${config}" ]]; then
      continue
    fi
    has_config=true
    if [[ "${config}" == *.json ]]; then
      if ! command -v python3 >/dev/null 2>&1; then
        log "python3 not available; skipping PiSugar JSON config update (${config})."
        continue
      fi
      log "Updating PiSugar JSON config to avoid Navio2 I2C/pin conflicts: ${config}"
      "${SUDO[@]}" python3 - "${config}" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as handle:
    data = json.load(handle)

original = json.dumps(data, sort_keys=True)

def update_node(node):
    if isinstance(node, dict):
        for key, value in list(node.items()):
            key_lower = key.lower()
            if isinstance(value, (dict, list)):
                update_node(value)
                continue
            if isinstance(value, str) and key_lower in {"connection", "conntype", "conn", "interface"}:
                if value.lower() == "i2c":
                    node[key] = "uart"
            if isinstance(value, bool) and ("i2c" in key_lower or "gpio" in key_lower or "pin" in key_lower or "button" in key_lower):
                node[key] = False
    elif isinstance(node, list):
        for entry in node:
            update_node(entry)

update_node(data)

updated = json.dumps(data, sort_keys=True) != original
if updated:
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")
PY
      updated=true
    else
      log "Updating PiSugar config to avoid Navio2 I2C/pin conflicts: ${config}"
      "${SUDO[@]}" sed -i.bak \
        -e 's/^I2C_ENABLED=.*/I2C_ENABLED=0/' \
        -e 's/^USE_I2C=.*/USE_I2C=0/' \
        -e 's/^GPIO_ENABLED=.*/GPIO_ENABLED=0/' \
        -e 's/^PIN_ENABLED=.*/PIN_ENABLED=0/' \
        "${config}" || true
      updated=true
    fi
  done

  if [[ "${has_config}" != true ]]; then
    log "PiSugar packages detected but no known configuration files were found."
    return
  fi

  log "PiSugar configs detected: ${found_configs[*]}"
  log "Skipping automatic PiSugar config changes; only apply adjustments documented by PiSugar."
  log "To avoid Navio2 I2C/pin conflicts while keeping battery/RTC telemetry, follow the official PiSugar docs for supported configuration options."
}

enable_soft_i2c_bus() {
  local config_file="/boot/config.txt"
  local overlay_line="dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=4,i2c_gpio_scl=5"
  local backup_path
  local match_cmd=()
  local trap_set=false

  trap 'log "Soft I2C enablement failed."; exit 1' ERR
  trap_set=true

  if [[ ! -f "${config_file}" ]]; then
    log "ERROR: ${config_file} not found."
    exit 1
  fi

  if ! command -v raspi-config >/dev/null 2>&1; then
    log "ERROR: raspi-config not found; cannot enable I2C."
    exit 1
  fi

  if command -v dpkg >/dev/null 2>&1; then
    if ! dpkg -s i2c-tools >/dev/null 2>&1; then
      log "Installing i2c-tools."
      "${SUDO[@]}" apt-get install -y i2c-tools
    fi

    if ! dpkg -s raspberrypi-kernel-headers >/dev/null 2>&1; then
      log "Installing raspberrypi-kernel-headers."
      "${SUDO[@]}" apt-get install -y raspberrypi-kernel-headers
    fi
  else
    log "dpkg not available; skipping package checks for i2c-tools and kernel headers."
  fi

  if command -v rg >/dev/null 2>&1; then
    match_cmd=(rg -q "^${overlay_line}$")
  else
    match_cmd=(grep -q "^${overlay_line}$")
  fi

  local overlay_added=false
  if "${match_cmd[@]}" "${config_file}"; then
    log "Soft I2C overlay already present."
  else
    backup_path="${config_file}.bak.$(date +%Y%m%d%H%M%S)"
    log "Backing up ${config_file} to ${backup_path}."
    "${SUDO[@]}" cp "${config_file}" "${backup_path}"
    log "Adding soft I2C overlay to ${config_file}."
    printf '%s\n' "${overlay_line}" | "${SUDO[@]}" tee -a "${config_file}" >/dev/null
    overlay_added=true
  fi

  log "Enabling I2C via raspi-config."
  "${SUDO[@]}" raspi-config nonint do_i2c 0

  if [[ "${overlay_added}" == true ]]; then
    log "Soft I2C overlay added; reboot required before validation."
  else
    log "Validating I2C bus 3."
    "${SUDO[@]}" i2cdetect -y 3 >/dev/null
  fi

  if [[ "${trap_set}" == true ]]; then
    trap - ERR
  fi
  log "SOFT_I2C_ENABLED"
  if [[ "${updated}" == true ]]; then
    local load_state
    load_state="$(systemctl show -p LoadState --value pisugar-server.service 2>/dev/null || true)"
    if [[ -n "${load_state}" && "${load_state}" != "not-found" ]]; then
      if "${SUDO[@]}" systemctl is-active --quiet pisugar-server.service; then
        log "Restarting pisugar-server.service to apply configuration changes."
        "${SUDO[@]}" systemctl restart pisugar-server.service
      else
        log "pisugar-server.service is not active; skipping restart."
      fi
    fi
  fi
}

check_non_wwan_internet() {
  local route_dev
  route_dev="$(ip -o route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev"){print $(i+1); exit}}')"
  if [[ -z "${route_dev}" ]]; then
    echo "Unable to determine a route to the internet." >&2
    exit 1
  fi
  if [[ "${route_dev}" == "wwan0" ]]; then
    echo "Default route uses wwan0. A non-wwan0 connection is required before updating." >&2
    exit 1
  fi
  if ! curl --interface "${route_dev}" --connect-timeout 5 --max-time 10 -fsS https://1.1.1.1/cdn-cgi/trace >/dev/null 2>&1; then
    echo "No internet connectivity detected on ${route_dev}." >&2
    exit 1
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
    if [[ ! -f "${script}" ]]; then
      echo "Expected script not found: ${script}" >&2
      exit 1
    fi
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
  if [[ ! -x "${installer}" ]]; then
    echo "Installer not found or not executable: ${installer}" >&2
    exit 1
  fi
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

main() {
  require_command git
  require_command apt-get
  require_command ip
  require_command curl
  require_command systemctl
  check_non_wwan_internet
  detect_uplink_service
  detect_pisugar_packages
  record_uplink_state
  record_wwan_state
  trap '' HUP
  trap cleanup EXIT
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
  enable_soft_i2c_bus
  configure_pisugar_for_navio2
  prompt_service_replacement
  log "Update routine completed."
}

main "$@"
