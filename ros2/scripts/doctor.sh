#!/usr/bin/env bash
# ==============================================================================
# doctor.sh - ROS 2 Foundation Doctor for Yahboom Rover
# ==============================================================================
set -euo pipefail

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

print_status() {
    local status="$1"
    local message="$2"
    if [ "$status" = "PASS" ]; then
        echo -e "[ ${GREEN}PASS${NC} ] $message"
    elif [ "$status" = "WARN" ]; then
        echo -e "[ ${YELLOW}WARN${NC} ] $message"
    else
        echo -e "[ ${RED}FAIL${NC} ] $message"
    fi
}

echo "============================================="
echo "        ROS 2 FOUNDATION DOCTOR              "
echo "============================================="

# 1. Host architecture
arch=$(uname -m)
if [ "$arch" = "aarch64" ] || [ "$arch" = "arm64" ]; then
    print_status "PASS" "Host architecture is ARM64 ($arch)."
else
    print_status "FAIL" "Host architecture is NOT ARM64 ($arch)!"
fi

# 2. Docker and Docker Compose on Host
if command -v docker &>/dev/null; then
    print_status "PASS" "Docker is available: $(docker --version)"
else
    print_status "FAIL" "Docker is NOT available!"
fi

if sudo -n docker compose version &>/dev/null; then
    print_status "PASS" "Docker Compose is available: $(sudo -n docker compose version)"
else
    print_status "FAIL" "Docker Compose is NOT available!"
fi

# 3. Host services status
echo "--- Host Services ---"
for svc in "rover-server.service" "rover-lidar.service"; do
    state=$(systemctl is-active "$svc" 2>/dev/null || echo "inactive")
    if [ "$state" = "active" ]; then
        print_status "PASS" "Host service $svc is active (running)."
    else
        print_status "FAIL" "Host service $svc is INACTIVE/FAILED!"
    fi
done

# 4. Host APIs availability
echo "--- Host APIs ---"
if curl -s -m 1 http://127.0.0.1:3000/api/status &>/dev/null; then
    print_status "PASS" "Host Cockpit API is reachable at http://127.0.0.1:3000/api/status"
else
    print_status "FAIL" "Host Cockpit API is UNREACHABLE!"
fi

if curl -s -m 1 http://127.0.0.1:3002/status &>/dev/null; then
    print_status "PASS" "Host LiDAR API is reachable at http://127.0.0.1:3002/status"
else
    print_status "FAIL" "Host LiDAR API is UNREACHABLE!"
fi

# 5. Container status
echo "--- Container Status ---"
container_id="$(sudo -n docker ps -q -f name=rover-ros2 -f status=running)"
if [ -n "$container_id" ]; then
    print_status "PASS" "Container 'rover-ros2' is running (ID: $container_id)."
    
    # 6. Container ROS 2 setup and CLI
    echo "--- ROS 2 inside Container ---"
    distro=$(sudo -n docker exec rover-ros2 /bin/bash -c 'echo $ROS_DISTRO' 2>/dev/null | tr -d '\r' || echo "unknown")
    if [ "$distro" = "jazzy" ]; then
        print_status "PASS" "ROS_DISTRO is $distro."
    else
        print_status "FAIL" "ROS_DISTRO is NOT jazzy ($distro)!"
    fi

    if sudo -n docker exec rover-ros2 /bin/bash -c 'source /opt/ros/jazzy/setup.bash && command -v ros2' &>/dev/null; then
        print_status "PASS" "ros2 CLI works inside the container."
    else
        print_status "FAIL" "ros2 CLI does NOT work inside the container!"
    fi

    # 7. Package installation checks
    for pkg in "nav2_bringup" "slam_toolbox" "nav2_msgs"; do
        if sudo -n docker exec rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 pkg prefix $pkg" &>/dev/null; then
            print_status "PASS" "ROS 2 package '$pkg' is installed."
        else
            print_status "FAIL" "ROS 2 package '$pkg' is NOT installed!"
        fi
    done

    # 8. Node and Topic checks
    if sudo -n docker exec rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list" 2>/dev/null | grep -q "rover_system_health"; then
        print_status "PASS" "rover_system_health node is running."
    else
        print_status "FAIL" "rover_system_health node is NOT running!"
    fi

    if sudo -n docker exec rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list" 2>/dev/null | grep -q "/diagnostics"; then
        print_status "PASS" "diagnostic topic (/diagnostics) exists."
    else
        print_status "FAIL" "diagnostic topic does NOT exist!"
    fi

    # 9. Phase 1 Forbidden Topics check
    echo "--- Published Topics Verification ---"
    active_topics=$(sudo -n docker exec rover-ros2 /bin/bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list" 2>/dev/null || echo "")
    for forbidden in "/scan" "/odom" "/cmd_vel"; do
        if echo "$active_topics" | grep -q "^${forbidden}$"; then
            print_status "FAIL" "Forbidden topic '$forbidden' is published!"
        else
            print_status "PASS" "Forbidden topic '$forbidden' is NOT published."
        fi
    done

    # 10. USB / Hardware Isolation checks
    echo "--- Hardware Isolation & Mount Checks ---"
    
    # Check docker inspect for /dev mounts
    dev_mounts=$(sudo -n docker inspect -f '{{range .Mounts}}{{.Destination}}{{"\n"}}{{end}}' rover-ros2 2>/dev/null | grep -E '^/dev' || true)
    if [ -n "$dev_mounts" ]; then
        print_status "FAIL" "Container mounts /dev or a subdirectory: $dev_mounts"
    else
        print_status "PASS" "Container metadata confirms no /dev mounts are configured."
    fi

    # Check file existence inside container
    for dev in "/dev/rover-esp32" "/dev/rover-lidar" "/dev/ttyUSB0" "/dev/ttyUSB1"; do
        if sudo -n docker exec rover-ros2 ls "$dev" &>/dev/null; then
            print_status "FAIL" "Device file '$dev' is visible inside the container!"
        else
            print_status "PASS" "Device file '$dev' is isolated (not visible inside the container)."
        fi
    done

else
    print_status "FAIL" "Container 'rover-ros2' is NOT running! Skipping container internal checks."
fi

echo "============================================="
