#!/usr/bin/env bash
# run_gold_standard_home_test.sh - Canonical Entrypoint for Gold Standard HOME Acceptance Test
# Strictly adheres to docs/EXACT_MOTION_CONTRACT.md and rover-exact-motion skill.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source .env if present
if [ -f "${REPO_DIR}/.env" ]; then
    set -a
    source "${REPO_DIR}/.env"
    set +a
elif [ -f "/home/ron/yahboom-encoder/.env" ]; then
    set -a
    source "/home/ron/yahboom-encoder/.env"
    set +a
fi

# Automatic ROS bag recording for /scan, /tf, /tf_static
RUN_TS="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="bag_${RUN_TS}"
echo "[LAUNCHER] Starting background ROS 2 bag recording: ${BAG_NAME} (/scan, /tf, /tf_static)..."
sudo docker exec -d rover-ros2 bash -c "export ROS_DOMAIN_ID=42 && source /opt/ros/jazzy/setup.bash && exec ros2 bag record -o /ros2_ws/rosbag/${BAG_NAME} /scan /tf /tf_static /odom /amcl_pose /particle_cloud /plan /local_plan /cmd_vel_smoothed"

cleanup() {
    echo "[LAUNCHER] Stopping ROS 2 bag recording..."
    sudo docker exec rover-ros2 pkill -INT -f "ros2 bag record" 2>/dev/null || true
    echo "[LAUNCHER] Rosbag preserved at /home/ron/yahboom-encoder/ros2/volumes/rosbag/${BAG_NAME}"
}
trap cleanup EXIT

echo "[LAUNCHER] Executing Gold Standard Acceptance Test on host Python..."
export PYTHONPATH="${REPO_DIR}:${PYTHONPATH:-}"
python3 -m tools.gold_standard_home_test.cli "$@"

