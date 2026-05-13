#!/bin/bash
# Compatibility wrapper for older integrations.
# Prefer ./start_unicore.sh for new deployments.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "$SCRIPT_DIR/start_unicore.sh" "$@"
