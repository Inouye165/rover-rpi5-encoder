#!/usr/bin/env bash
# ==============================================================================
# shell.sh - Open an interactive shell in the ROS 2 container
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

container_id="$(sudo -n docker ps -q -f name=rover-ros2 -f status=running)"
if [ -n "$container_id" ]; then
    echo "=== Entering Running Container ==="
    sudo -n docker compose exec -it rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec /bin/bash"
else
    echo "=== Container not running. Starting temporary interactive container ==="
    sudo -n docker compose run --rm -it rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec /bin/bash"
fi
