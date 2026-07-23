#!/usr/bin/env bash
# ==============================================================================
# doctor.sh - ROS 2 Doctor for Yahboom Rover (Phase 3: Encoder Odometry & TF)
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
echo "        ROS 2 PHASE 3 DOCTOR                 "
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

if curl -s -m 1 http://127.0.0.1:3000/api/encoders &>/dev/null; then
    print_status "PASS" "Host Encoder Telemetry API is reachable at http://127.0.0.1:3000/api/encoders"
else
    print_status "FAIL" "Host Encoder Telemetry API is UNREACHABLE!"
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

    # 7. Node checks
    echo "--- Node Checks ---"
    node_list="$(sudo -n docker exec rover-ros2 /bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list' 2>/dev/null | tr -d '\r' || true)"

    for req_node in "/rover_system_health" "/rover_lidar_bridge" "/rover_encoder_odometry"; do
        if echo "$node_list" | grep -q "$req_node"; then
            print_status "PASS" "Node $req_node is running."
        else
            print_status "FAIL" "Node $req_node is NOT running!"
        fi
    done

    # 8. Topic checks
    echo "--- Topic Checks ---"
    topic_list="$(sudo -n docker exec rover-ros2 /bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list' 2>/dev/null | tr -d '\r' || true)"

    for req_topic in "/diagnostics" "/scan" "/odom" "/tf" "/tf_static"; do
        if grep -qx "$req_topic" <<< "$topic_list"; then
            print_status "PASS" "Topic $req_topic exists."
        else
            print_status "FAIL" "Topic $req_topic does NOT exist!"
        fi
    done

    # 9. /odom type and validation
    echo "--- /odom Type and Content Checks ---"
    odom_type="$(sudo -n docker exec rover-ros2 /bin/bash -c \
        "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic type /odom" \
        2>/dev/null | tr -d '\r' || echo "")"
    if [ "$odom_type" = "nav_msgs/msg/Odometry" ]; then
        print_status "PASS" "/odom type is nav_msgs/msg/Odometry."
    else
        print_status "FAIL" "/odom type is NOT nav_msgs/msg/Odometry (got: '$odom_type')!"
    fi

    # Echo one /odom message
    odom_msg="$(sudo -n docker exec rover-ros2 /bin/bash -c \
        "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && \
         timeout 5 ros2 topic echo /odom --once 2>/dev/null" \
        || true)"

    if [ -n "$odom_msg" ]; then
        print_status "PASS" "Received at least one /odom message within 5 seconds."

        if echo "$odom_msg" | grep -q "frame_id: '*odom'*"; then
            print_status "PASS" "/odom header.frame_id is 'odom'."
        else
            print_status "FAIL" "/odom header.frame_id is NOT 'odom'!"
        fi

        if echo "$odom_msg" | grep -q "child_frame_id: '*base_link'*"; then
            print_status "PASS" "/odom child_frame_id is 'base_link'."
        else
            print_status "FAIL" "/odom child_frame_id is NOT 'base_link'!"
        fi
    else
        print_status "FAIL" "No /odom message received within 5 seconds!"
    fi

    # 10. TF Tree Validation (odom -> base_link and base_link -> laser_frame)
    echo "--- TF Tree Verification ---"
    tf_odom="$(sudo -n docker exec rover-ros2 /bin/bash -c \
        "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && \
         timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null" \
        || true)"
    if echo "$tf_odom" | grep -q "Translation:"; then
        print_status "PASS" "Dynamic transform odom -> base_link is active."
    else
        print_status "FAIL" "Dynamic transform odom -> base_link is NOT active!"
    fi

    tf_laser="$(sudo -n docker exec rover-ros2 /bin/bash -c \
        "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && \
         timeout 5 ros2 run tf2_ros tf2_echo base_link laser_frame 2>/dev/null" \
        || true)"
    if echo "$tf_laser" | grep -q "Translation:"; then
        print_status "PASS" "Static transform base_link -> laser_frame is active."
    else
        print_status "FAIL" "Static transform base_link -> laser_frame is NOT active!"
    fi

    # 11. Forbidden Topics & Nodes Verification (/cmd_vel must NOT be published)
    echo "--- Forbidden Topics & Nodes Verification ---"
    if grep -qx "/cmd_vel" <<< "$topic_list"; then
        print_status "FAIL" "Forbidden topic '/cmd_vel' IS published!"
    else
        print_status "PASS" "Forbidden topic '/cmd_vel' is NOT published."
    fi

    for forbidden_node in "/slam_toolbox" "/nav2_container" "/bt_navigator"; do
        if grep -qx "$forbidden_node" <<< "$node_list"; then
            print_status "FAIL" "Forbidden node '$forbidden_node' IS running!"
        else
            print_status "PASS" "Forbidden node '$forbidden_node' is NOT running."
        fi
    done

    # 12. USB / Hardware Isolation checks
    echo "--- Hardware Isolation & Mount Checks ---"
    dev_mounts=$(sudo -n docker inspect -f '{{range .Mounts}}{{.Destination}}{{"\n"}}{{end}}' rover-ros2 2>/dev/null | grep -E '^/dev' || true)
    if [ -n "$dev_mounts" ]; then
        print_status "FAIL" "Container mounts /dev or a subdirectory: $dev_mounts"
    else
        print_status "PASS" "Container metadata confirms no /dev mounts are configured."
    fi

    for dev in "/dev/rover-esp32" "/dev/rover-lidar" "/dev/ttyUSB0" "/dev/ttyUSB1"; do
        if sudo -n docker exec rover-ros2 ls "$dev" &>/dev/null; then
            print_status "FAIL" "Device file '$dev' is visible inside the container!"
        else
            print_status "PASS" "Device file '$dev' is isolated (not visible inside container)."
        fi
    done

else
    print_status "FAIL" "Container 'rover-ros2' is NOT running!"
fi

echo "============================================="
