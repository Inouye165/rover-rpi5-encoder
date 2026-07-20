#!/usr/bin/env bash
# ==============================================================================
# build.sh - Build the Docker image and compile the workspace with resource limits
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

echo "=== 1. Building Docker Image ==="
sudo -n docker compose build

echo "=== 2. Cleaning previous build artifacts ==="
# Remove only build, install, and log directories
rm -rf ros2_ws/build ros2_ws/install ros2_ws/log

echo "=== 3. Cleaning old builder container ==="
sudo -n docker rm -f rover-ros2-builder 2>/dev/null || true

echo "=== 4. Building ROS 2 Workspace in resource-limited builder ==="
# Execute the build inside a temporary container with strict resource constraints
sudo -n docker run --rm \
    --name rover-ros2-builder \
    --cpus=2 \
    --memory=3g \
    --memory-swap=3g \
    --pids-limit=256 \
    -v "$(pwd)/ros2_ws:/ros2_ws" \
    -v "$(pwd)/compose.yaml:/ros2_ws/compose.yaml:ro" \
    -v "$(pwd)/Dockerfile:/ros2_ws/Dockerfile:ro" \
    -w /ros2_ws \
    rover-ros2:jazzy \
    /bin/bash -c "
      set -euo pipefail
      colcon build --symlink-install --executor sequential
      source install/setup.bash
      colcon test --executor sequential
      colcon test-result --verbose
    "

echo "=== Build & Verification Complete ==="
