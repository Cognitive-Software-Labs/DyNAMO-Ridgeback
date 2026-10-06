#!/usr/bin/env bash
set -euo pipefail

DISPLAY_ID=1
GEOMETRY=1280x720
DEPTH=24
WEB_ROOT=/home/robot/ridgeback-remote-screen
VNC_HOST=127.0.0.1
VNC_PORT=5901
WEB_PORT=9100

if ! vncserver -list | awk '{print $1}' | grep -qx "${DISPLAY_ID}"; then
  vncserver ":${DISPLAY_ID}" -geometry "${GEOMETRY}" -depth "${DEPTH}" -localhost yes >/tmp/ridgeback-remote-screen-vnc.log 2>&1
fi

exec websockify --web="${WEB_ROOT}" 0.0.0.0:${WEB_PORT} ${VNC_HOST}:${VNC_PORT}
