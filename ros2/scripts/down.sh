#!/usr/bin/env bash
# ==============================================================================
# down.sh - Stop only the ROS 2 container
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

echo "=== Stopping ROS 2 Container ==="
sudo -n docker compose down

echo "=== Container Stopped ==="
