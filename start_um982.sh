#!/bin/bash
# =============================================================================
# UM982 GNSS driver startup
#
# Launches: um982_node
# Config read from /opt/unicore_gnss/share/unicore_gnss/config/um982.yaml
# Serial device default: /dev/ttyUSB0 (configurable via params)
# =============================================================================
set -euo pipefail

echo "[start_um982.sh] Launching Unicore UM982 GNSS driver..."

# Use the launch file to start the driver
ros2 launch unicore_gnss um982_launch.py
