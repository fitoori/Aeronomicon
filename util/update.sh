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

update_repo() {
  log "Updating repository in ${REPO_DIR}."
  git -C "${REPO_DIR}" fetch --prune
  git -C "${REPO_DIR}" pull --ff-only
}

prompt_service_replacement() {
  local response
  if [[ -t 0 ]]; then
    read -r -p "Replace systemd service files with newer versions? [y/N] " response
  else
    response="n"
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
  update_repo
  update_system_packages
  prompt_service_replacement
  log "Update routine completed."
}

main "$@"
