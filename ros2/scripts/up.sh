#!/usr/bin/env bash
# ==============================================================================
# up.sh - Start the ROS 2 container after running safety checks
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

# 1. Verify workspace has been built
if [ ! -f ros2_ws/install/setup.bash ]; then
    echo "ERROR: ros2_ws/install/setup.bash not found! Please run ./scripts/build.sh first." >&2
    exit 1
fi

# 2. Run pre-startup host doctor
echo "=== Running Host Doctor (Pre-Startup) ==="
/home/ron/yahboom-encoder/rpi5/rover-doctor.sh

# 3. Create volume directories normally
mkdir -p volumes/maps volumes/logs volumes/rosbag

echo "=== Starting ROS 2 Container ==="
sudo -n docker compose up -d --force-recreate

# 4. Wait five seconds
echo "Waiting 5 seconds for initialization..."
sleep 5

# 5. Show container status and logs
echo "=== Container Status ==="
sudo -n docker compose ps -a

echo "=== Container Logs (Last 100 Lines) ==="
sudo -n docker compose logs --tail=100

# 6. Fail if container is not running
if [ -z "$(sudo -n docker ps -q -f name=rover-ros2 -f status=running)" ]; then
    echo "ERROR: Container rover-ros2 is NOT running!" >&2
    exit 1
fi

# 7. Run post-startup host doctor
echo "=== Running Host Doctor (Post-Startup) ==="
/home/ron/yahboom-encoder/rpi5/rover-doctor.sh

echo "=== Startup Complete ==="
