#!/usr/bin/env bash
set -euo pipefail

THEME_NAME="wireless-autonomous-terrestrial-navigation"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
THEME_SRC="${SCRIPT_DIR}/${THEME_NAME}"
THEME_DST="/usr/share/plymouth/themes/${THEME_NAME}"

if [[ ! -d "${THEME_SRC}" ]]; then
  echo "Theme source not found at ${THEME_SRC}" >&2
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

install -d "${THEME_DST}"

if command -v rsync >/dev/null 2>&1; then
  rsync -a --delete "${THEME_SRC}/" "${THEME_DST}/"
else
  rm -rf "${THEME_DST}"
  cp -a "${THEME_SRC}" "${THEME_DST}"
fi

if command -v plymouth-set-default-theme >/dev/null 2>&1; then
  plymouth-set-default-theme -R "${THEME_NAME}"
else
  echo "plymouth-set-default-theme not found; theme copied but not activated." >&2
fi
