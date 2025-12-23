#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${WEBAPP_SERVICE:-webapp.service}"
HOST="${WEBAPP_HOST:-127.0.0.1}"
PORT="${WEBAPP_PORT:-8080}"
URL="http://${HOST}:${PORT}/api/health"

if ! curl --fail --silent --max-time 2 "$URL" >/dev/null; then
  systemctl restart "${SERVICE_NAME}"
fi
