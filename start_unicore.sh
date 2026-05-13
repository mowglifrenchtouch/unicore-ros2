#!/bin/bash
# =============================================================================
# Unicore GNSS driver startup
#
# Launches: unicore_node
# Config read from /opt/unicore_gnss/share/unicore_gnss/config/unicore.yaml
# Serial device default: /dev/ttyUSB0 (configurable via params)
# =============================================================================
set -euo pipefail

echo "[start_unicore.sh] Launching Unicore GNSS driver..."

ros2 launch unicore_gnss unicore_launch.py
