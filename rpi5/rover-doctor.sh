#!/usr/bin/env bash
# ==============================================================================
# rover-doctor.sh - Diagnostic Utility for Yahboom Rover system
# Run this on the Raspberry Pi 5 to check system, service, and hardware health.
# ==============================================================================

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Helper: print status tag
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
echo "        ROVER DIAGNOSTIC SYSTEM (DOCTOR)     "
echo "============================================="

# 1. Hostname & IP
host_name=$(hostname)
ip_addr=$(ip route get 1 2>/dev/null | awk '{print $7}' || hostname -I | awk '{print $1}')
echo "Hostname: $host_name"
echo "IP Address: $ip_addr"
echo "---------------------------------------------"

# 2. Dialout membership
if id -nG "$USER" | grep -q "dialout"; then
    print_status "PASS" "User '$USER' is a member of the 'dialout' group."
else
    print_status "FAIL" "User '$USER' is NOT in the 'dialout' group! Serial connections will fail."
fi

# 3. USB Devices check (lsusb overview)
echo "---------------------------------------------"
echo "USB Controller Devices connected:"
lsusb | grep -E "1a86:7523|10c4:ea60" || echo "  No matching USB serial hardware controllers found."
echo "---------------------------------------------"

# 4. /dev/rover-esp32 Verification
if [ -e "/dev/rover-esp32" ]; then
    esp_target=$(readlink -f /dev/rover-esp32)
    esp_info=$(udevadm info -q property -n "$esp_target" 2>/dev/null || true)
    esp_vid=$(echo "$esp_info" | grep -E '^ID_VENDOR_ID=' | cut -d= -f2 || true)
    esp_pid=$(echo "$esp_info" | grep -E '^ID_MODEL_ID=' | cut -d= -f2 || true)
    
    if [ "$esp_vid" = "1a86" ] && [ "$esp_pid" = "7523" ]; then
        print_status "PASS" "/dev/rover-esp32 symlink maps to $esp_target ($esp_vid:$esp_pid CH340)."
    else
        print_status "FAIL" "/dev/rover-esp32 maps to $esp_target with invalid hardware ID ($esp_vid:$esp_pid). Expected 1a86:7523."
    fi
else
    print_status "FAIL" "/dev/rover-esp32 symlink does not exist!"
fi

# 5. /dev/rover-lidar Verification
if [ -e "/dev/rover-lidar" ]; then
    lidar_target=$(readlink -f /dev/rover-lidar)
    lidar_info=$(udevadm info -q property -n "$lidar_target" 2>/dev/null || true)
    lidar_vid=$(echo "$lidar_info" | grep -E '^ID_VENDOR_ID=' | cut -d= -f2 || true)
    lidar_pid=$(echo "$lidar_info" | grep -E '^ID_MODEL_ID=' | cut -d= -f2 || true)
    lidar_serial=$(echo "$lidar_info" | grep -E '^ID_SERIAL_SHORT=' | cut -d= -f2 || true)
    
    if [ "$lidar_vid" = "10c4" ] && [ "$lidar_pid" = "ea60" ]; then
        print_status "PASS" "/dev/rover-lidar symlink maps to $lidar_target ($lidar_vid:$lidar_pid CP2102N, serial: $lidar_serial)."
    else
        print_status "FAIL" "/dev/rover-lidar maps to $lidar_target with invalid hardware ID ($lidar_vid:$lidar_pid). Expected 10c4:ea60."
    fi
else
    print_status "FAIL" "/dev/rover-lidar symlink does not exist!"
fi

# 6. Process ownership of serial devices
for dev in "/dev/rover-esp32" "/dev/rover-lidar"; do
    if [ -e "$dev" ]; then
        target_path=$(readlink -f "$dev")
        pids=$(fuser "$target_path" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            processes=""
            for pid in $pids; do
                pname=$(ps -p "$pid" -o comm= 2>/dev/null || true)
                processes="$processes$pname(PID:$pid) "
            done
            print_status "PASS" "Device $dev ($target_path) is currently in use by: $processes"
        else
            print_status "WARN" "Device $dev ($target_path) is free (no active processes using it)."
        fi
    fi
done

# 7. Systemd Services check
echo "---------------------------------------------"
for svc in "rover-server.service" "rover-lidar.service"; do
    state=$(systemctl is-active "$svc" 2>/dev/null || echo "inactive")
    restarts=$(systemctl show -p NRestarts "$svc" 2>/dev/null | cut -d= -f2 || echo "0")
    
    if [ "$state" = "active" ]; then
        print_status "PASS" "Service $svc is active (running). Restarts count: $restarts"
    else
        print_status "FAIL" "Service $svc is INACTIVE/FAILED! Restarts count: $restarts"
    fi
done

# 8. Port connections & bindings
echo "---------------------------------------------"
(echo > /dev/tcp/127.0.0.1/3000) &>/dev/null && port_3000="open" || port_3000="closed"
(echo > /dev/tcp/127.0.0.1/3002) &>/dev/null && port_3002="open" || port_3002="closed"

if [ "$port_3000" = "open" ]; then
    print_status "PASS" "Cockpit port 3000 is open/listening on localhost."
else
    print_status "FAIL" "Cockpit port 3000 is CLOSED! Cockpit server is not reachable."
fi

if [ "$port_3002" = "open" ]; then
    print_status "PASS" "LiDAR sidecar port 3002 is open/listening on localhost."
else
    print_status "WARN" "LiDAR sidecar port 3002 is CLOSED! LiDAR service is down or not listening."
fi

# 9. Cockpit & ESP32 connection via HTTP API
echo "---------------------------------------------"
if [ "$port_3000" = "open" ]; then
    status_response=$(curl -s http://localhost:3000/api/status || true)
    if [ -n "$status_response" ]; then
        serial_connected=$(echo "$status_response" | grep -o '"serialConnected":[^,]*' | cut -d: -f2 | tr -d ' ' || echo "false")
        last_age=$(echo "$status_response" | grep -o '"lastPacketAgeMs":[^,]*' | cut -d: -f2 | tr -d ' ' | tr -d '}' || echo "null")
        is_armed=$(echo "$status_response" | grep -o '"armed":[^,]*' | cut -d: -f2 | tr -d ' ' | tr -d '}' || echo "false")
        
        if [ "$serial_connected" = "true" ]; then
            print_status "PASS" "Cockpit API reports ESP32 is connected on port $(echo "$status_response" | grep -o '"port":"[^"]*' | cut -d'"' -f4)."
            if [ "$last_age" != "null" ]; then
                if [ "$last_age" -lt 2000 ]; then
                    print_status "PASS" "ESP32 binary telemetry is flowing (packet age: ${last_age}ms, Armed: $is_armed)."
                else
                    print_status "FAIL" "ESP32 telemetry is STALE! Packet age: ${last_age}ms. Expected < 2000ms."
                fi
            else
                print_status "WARN" "No telemetry packets received by Cockpit yet."
            fi
        else
            print_status "FAIL" "Cockpit API reports ESP32 is DISCONNECTED!"
        fi
    else
        print_status "FAIL" "Failed to parse Cockpit status API response."
    fi
else
    print_status "FAIL" "Cannot query Cockpit API (port 3000 is closed)."
fi

# 10. LiDAR Sidecar Health API
if [ "$port_3002" = "open" ]; then
    lidar_response=$(curl -s http://localhost:3002/status || true)
    if [ -n "$lidar_response" ]; then
        lidar_connected=$(echo "$lidar_response" | grep -o '"connected":[^,]*' | cut -d: -f2 | tr -d ' ' || echo "false")
        lidar_health=$(echo "$lidar_response" | grep -o '"health":"[^"]*' | cut -d'"' -f4 || echo "unknown")
        lidar_hz=$(echo "$lidar_response" | grep -o '"scanHz":[^,]*' | cut -d: -f2 | tr -d ' ' || echo "0.0")
        
        if [ "$lidar_connected" = "true" ] && [ "$lidar_health" = "Good" -o "$lidar_health" = "OK" ]; then
            print_status "PASS" "LiDAR sidecar reports connected=true, health=$lidar_health, scanHz=${lidar_hz}Hz."
        else
            print_status "FAIL" "LiDAR sidecar reports health issue! connected=$lidar_connected, health=$lidar_health."
        fi
    else
        print_status "FAIL" "Failed to parse LiDAR status API response."
    fi
fi

# 11. Boot issues & logs verification
echo "---------------------------------------------"
recent_logs=$(journalctl -u rover-server.service -b 0 -n 100 --no-pager 2>/dev/null || true)
if echo "$recent_logs" | grep -qi "invalid header"; then
    print_status "FAIL" "Detected ESP32 boot issue: 'invalid header' matches found in current boot logs!"
    echo "$recent_logs" | grep -i "invalid header" | tail -n 3
else
    print_status "PASS" "No 'invalid header' boot errors found in the current boot logs."
fi

# Check for reset loop (restarts in quick succession)
boot_count=$(echo "$recent_logs" | grep -o "Firmware Name: Maker-ESP32-Unified-Rover" | wc -l)
if [ "$boot_count" -gt 3 ]; then
    print_status "FAIL" "Possible reset loop detected! ESP32 booted $boot_count times during current service runtime."
else
    print_status "PASS" "ESP32 startup sequence count: $boot_count (no reset loop detected)."
fi

echo "============================================="
