#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"
exec ./test_unicore_live_validate.sh "$@"
