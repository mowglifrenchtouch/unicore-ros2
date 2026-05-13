#!/bin/bash
# =============================================================================
# Unicore GNSS driver startup + NTRIP client
#
# Launches:
#   1. unicore_node       — Unicore GNSS driver and RTCM injector on /dev/gps
#   2. ntrip_client_node  — NTRIP caster client publishing /ntrip_client/rtcm
# Config read from /ws/install/share/unicore_gnss/config/unicore.yaml
# Serial device default: /dev/gps (configurable via params)
# =============================================================================
set -euo pipefail

CONFIG="${CONFIG:-/config/robot.yaml}"
GPS_PID=""
NTRIP_PID=""

if [ ! -f "$CONFIG" ]; then
  if [ "$CONFIG" = "/config/robot.yaml" ] && [ -f /config/mowgli_robot.yaml ]; then
    CONFIG="/config/mowgli_robot.yaml"
  fi
fi

if [ ! -f "$CONFIG" ]; then
  echo "[start_gps.sh] ERROR: $CONFIG not found. Bind-mount your robot config to /config."
  exit 1
fi

parse_yaml() {
  awk -F: -v key="$1" '
    $1 ~ "^[[:space:]]*" key "[[:space:]]*$" {
      value = substr($0, index($0, ":") + 1)
      sub(/^[[:space:]]+/, "", value)
      sub(/[[:space:]]+$/, "", value)
      gsub(/^["'"'"']|["'"'"']$/, "", value)
      print value
      exit
    }
  ' "$CONFIG"
}

is_truthy() {
  case "${1,,}" in
    true|1|yes|y|on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

cleanup() {
  [ -n "$NTRIP_PID" ] && kill "$NTRIP_PID" 2>/dev/null || true
  [ -n "$GPS_PID" ] && kill "$GPS_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

NTRIP_ENABLED=$(parse_yaml ntrip_enabled)
NTRIP_HOST=$(parse_yaml ntrip_host)
NTRIP_PORT=$(parse_yaml ntrip_port)
NTRIP_USER=$(parse_yaml ntrip_user)
NTRIP_PASSWORD=$(parse_yaml ntrip_password)
NTRIP_MOUNTPOINT=$(parse_yaml ntrip_mountpoint)
NTRIP_ENABLED="${NTRIP_ENABLED:-false}"

echo "[start_gps.sh] Launching Unicore GNSS driver..."
ros2 launch unicore_gnss unicore_launch.py &
GPS_PID=$!

if is_truthy "$NTRIP_ENABLED"; then
  echo "[start_gps.sh] NTRIP enabled: ${NTRIP_HOST}:${NTRIP_PORT}/${NTRIP_MOUNTPOINT}"
  sleep 3
  ros2 run ntrip_client_node ntrip_client_node --ros-args \
    -p "host:=${NTRIP_HOST}" \
    -p "port:=${NTRIP_PORT}" \
    -p "mountpoint:=${NTRIP_MOUNTPOINT}" \
    -p "username:=${NTRIP_USER}" \
    -p "password:=${NTRIP_PASSWORD}" &
  NTRIP_PID=$!
fi

wait -n || true
wait
