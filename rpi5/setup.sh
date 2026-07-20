#!/usr/bin/env bash
# ==============================================================================
# RPi5 Setup Script for Yahboom Encoder Rover Server
# Run this script on the RPi5 to install dependencies and systemd services.
# ==============================================================================

set -e

# Ensure script is run with sudo for system commands, but we need the original user for service config
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root or using sudo: sudo ./setup.sh"
  exit 1
fi

# Detect original user if run under sudo
REAL_USER=${SUDO_USER:-$USER}
REAL_HOME=$(eval echo ~$REAL_USER)
WORKING_DIR=$(pwd)

echo "=== RPi5 Rover Server Setup ==="
echo "User: $REAL_USER"
echo "Working directory: $WORKING_DIR"

# Helper: Extract VID and PID of a device path using udevadm
get_vid_pid() {
  local dev="$1"
  if [ -e "$dev" ]; then
    local info
    info=$(udevadm info -q property -n "$dev" 2>/dev/null || true)
    local vid
    vid=$(echo "$info" | grep -E '^ID_VENDOR_ID=' | cut -d= -f2 || true)
    local pid
    pid=$(echo "$info" | grep -E '^ID_MODEL_ID=' | cut -d= -f2 || true)
    echo "${vid}:${pid}"
  fi
}

# Helper: Check for multiple CH340 devices to detect ambiguity
check_ch340_ambiguity() {
  local ch340_count=0
  local dev
  for dev in /sys/class/tty/ttyUSB* /sys/class/tty/ttyACM*; do
    [ ! -e "$dev" ] && continue
    local devname="/dev/$(basename "$dev")"
    local vid_pid
    vid_pid=$(get_vid_pid "$devname")
    if [ "$vid_pid" = "1a86:7523" ]; then
      ch340_count=$((ch340_count + 1))
    fi
  done
  if [ "$ch340_count" -gt 1 ]; then
    echo "ERROR: Multiple CH340 devices (1a86:7523) found ($ch340_count). Selection is ambiguous!" >&2
    return 1
  fi
  return 0
}

# 1. Enable I2C interface on Raspberry Pi if not already enabled (kept for general hardware support)
echo "Enabling I2C interface..."
if command -v raspi-config &> /dev/null; then
  raspi-config nonint do_i2c 0
else
  CONFIG_TXT="/boot/firmware/config.txt"
  [ ! -f "$CONFIG_TXT" ] && CONFIG_TXT="/boot/config.txt"
  if [ -f "$CONFIG_TXT" ]; then
    if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_TXT"; then
      echo "dtparam=i2c_arm=on" >> "$CONFIG_TXT"
      echo "I2C enabled in $CONFIG_TXT. A reboot might be required."
    fi
  fi
fi

# 2. Install System Dependencies
echo "Updating apt package list..."
apt-get update -y

echo "Installing required system packages..."
apt-get install -y python3-pip python3-venv i2c-tools curl build-essential

# Install Node.js if not present
if ! command -v node &> /dev/null; then
  echo "Node.js not found. Installing Node.js LTS (v20)..."
  curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
  apt-get install -y nodejs
fi

NODE_PATH=$(which node)
echo "Node.js path: $NODE_PATH ($(node -v))"

# 3. Setup Env File
if [ ! -f .env ]; then
  echo "No .env found. Copying from .env.example..."
  cp .env.example .env
  chown $REAL_USER:$REAL_USER .env
  echo "Created .env file. Please ensure it contains correct values."
fi

# Load variables from .env
sed -i 's/\r$//' .env
# Idempotently update .env file on the Pi with new variables before sourcing
if ! grep -q '^ROVER_ESP32_DEVICE=' .env; then
  echo "Appending ROVER_ESP32_DEVICE=/dev/rover-esp32 to .env"
  echo "ROVER_ESP32_DEVICE=/dev/rover-esp32" >> .env
fi
if ! grep -q '^ROVER_LIDAR_DEVICE=' .env; then
  echo "Appending ROVER_LIDAR_DEVICE=/dev/rover-lidar to .env"
  echo "ROVER_LIDAR_DEVICE=/dev/rover-lidar" >> .env
fi
if ! grep -q '^ROVER_PI_APP_DIR=' .env; then
  echo "Appending ROVER_PI_APP_DIR=$WORKING_DIR to .env"
  echo "ROVER_PI_APP_DIR=$WORKING_DIR" >> .env
fi
source .env

# 4. Configure WiFi using NetworkManager if SSID and Password are provided
if [ -n "$WIFI_SSID" ] && [ "$WIFI_SSID" != "YourNetworkName" ]; then
  echo "Configuring WiFi connection to SSID: $WIFI_SSID..."
  if command -v nmcli &> /dev/null; then
    if nmcli dev wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD"; then
      echo "WiFi connection successful!"
    else
      echo "Warning: WiFi connection failed, but proceeding."
    fi
  else
    echo "Warning: nmcli tool not found. Skipping auto-WiFi config."
  fi
fi

# 5. Node package installation
echo "Installing Node dependencies..."
sudo -u $REAL_USER npm install --prefix "$WORKING_DIR"

# 6. Python Virtual Environment Setup & Validation
echo "Creating/updating Python virtual environment..."
if [ ! -d "$WORKING_DIR/.venv" ]; then
  sudo -u $REAL_USER python3 -m venv "$WORKING_DIR/.venv"
fi

echo "Upgrading pip and installing requirements in .venv..."
sudo -u $REAL_USER "$WORKING_DIR/.venv/bin/pip" install --upgrade pip
sudo -u $REAL_USER "$WORKING_DIR/.venv/bin/pip" install -r "$WORKING_DIR/rpi5/requirements-lidar.txt"

echo "Validating Python imports in virtual environment..."
if ! sudo -u $REAL_USER "$WORKING_DIR/.venv/bin/python" -c "import rplidarc1, serial, numpy, scipy; print('Imports OK')" 2>/dev/null; then
  echo "ERROR: Python imports validation failed! Required libraries (rplidarc1, pyserial, numpy, scipy) are missing or broken."
  exit 1
fi

echo "Checking rplidar_sidecar.py syntax..."
if ! sudo -u $REAL_USER "$WORKING_DIR/.venv/bin/python" -m py_compile "$WORKING_DIR/rplidar_sidecar.py"; then
  echo "ERROR: Syntax error in rplidar_sidecar.py"
  exit 1
fi

echo "Marking diagnostic scripts executable..."
chmod +x "$WORKING_DIR/rpi5/rover-doctor.sh"

# 7. Install Udev Rules & Verify Symlinks
echo "Installing udev rules..."
cp rpi5/99-rover-lidar.rules /etc/udev/rules.d/99-rover-lidar.rules
udevadm control --reload-rules
udevadm trigger

echo "Waiting for udev rules to settle..."
sleep 2

# Check for CH340 ambiguity
if ! check_ch340_ambiguity; then
  exit 1
fi

# Device symlink verification
ESP32_DEV=${ROVER_ESP32_DEVICE:-/dev/rover-esp32}
LIDAR_DEV=${ROVER_LIDAR_DEVICE:-/dev/rover-lidar}

echo "Verifying device path endpoints: ESP32=$ESP32_DEV, LiDAR=$LIDAR_DEV"

if [ ! -e "$ESP32_DEV" ]; then
  echo "ERROR: ESP32 device path '$ESP32_DEV' does not exist."
  exit 1
fi
esp_id=$(get_vid_pid "$ESP32_DEV")
if [ "$esp_id" != "1a86:7523" ]; then
  echo "ERROR: ESP32 device '$ESP32_DEV' has invalid hardware ID '$esp_id'. Expected 1a86:7523."
  exit 1
fi

if [ ! -e "$LIDAR_DEV" ]; then
  echo "ERROR: LiDAR device path '$LIDAR_DEV' does not exist."
  exit 1
fi
lidar_id=$(get_vid_pid "$LIDAR_DEV")
if [ "$lidar_id" != "10c4:ea60" ]; then
  echo "ERROR: LiDAR device '$LIDAR_DEV' has invalid hardware ID '$lidar_id'. Expected 10c4:ea60."
  exit 1
fi

echo "Udev rules configured and verified successfully."

# Stop and Disable legacy I2C Sidecar if active
echo "Cleaning up any legacy rover-i2c service..."
if systemctl is-active --quiet rover-i2c.service 2>/dev/null; then
  systemctl stop rover-i2c.service || true
fi
if systemctl is-enabled --quiet rover-i2c.service 2>/dev/null; then
  systemctl disable rover-i2c.service || true
fi
if [ -f /etc/systemd/system/rover-i2c.service ]; then
  rm -f /etc/systemd/system/rover-i2c.service
fi

# 8. Install Systemd Services
echo "Installing systemd services..."
# Stop both services to release all serial ports before installing and verifying
echo "Stopping running services to release serial ports..."
systemctl stop rover-server.service || true
systemctl stop rover-lidar.service || true

VENV_PYTHON_PATH="$WORKING_DIR/.venv/bin/python3"
SED_EXPR="s|{{USER}}|$REAL_USER|g; s|{{WORKING_DIR}}|$WORKING_DIR|g; s|{{NODE_PATH}}|$NODE_PATH|g; s|{{PYTHON_PATH}}|$VENV_PYTHON_PATH|g; s|{{ROVER_LIDAR_DEVICE}}|$LIDAR_DEV|g; s|{{ROVER_ESP32_DEVICE}}|$ESP32_DEV|g"

sed "$SED_EXPR" rpi5/rover-server.service.template > /etc/systemd/system/rover-server.service
sed "$SED_EXPR" rpi5/rover-lidar.service.template > /etc/systemd/system/rover-lidar.service

# Validate that the generated ExecStart does not contain empty device argument (using POSIX [[:space:]] and grep -E)
if grep -E -q -i 'execstart=.*--dev[[:space:]]*""' /etc/systemd/system/rover-lidar.service; then
  echo "ERROR: Validation failed! Generated rover-lidar.service contains --dev \"\" in ExecStart." >&2
  exit 1
fi
if grep -E -q -i 'execstart=.*--dev[[:space:]]*$' /etc/systemd/system/rover-lidar.service; then
  echo "ERROR: Validation failed! Generated rover-lidar.service has trailing/missing --dev argument in ExecStart." >&2
  exit 1
fi

# Fail if any unresolved {{...}} placeholder remains in a generated service
if grep -E -q '\{\{[a-zA-Z0-9_]+\}\}' /etc/systemd/system/rover-server.service || grep -E -q '\{\{[a-zA-Z0-9_]+\}\}' /etc/systemd/system/rover-lidar.service; then
  echo "ERROR: Validation failed! Generated service files contain unresolved templates." >&2
  exit 1
fi

# Run systemd-analyze verify before daemon-reload
echo "Running systemd-analyze verify on service files..."
if ! systemd-analyze verify /etc/systemd/system/rover-server.service; then
  echo "ERROR: systemd-analyze verify failed for rover-server.service" >&2
  exit 1
fi
if ! systemd-analyze verify /etc/systemd/system/rover-lidar.service; then
  echo "ERROR: systemd-analyze verify failed for rover-lidar.service" >&2
  exit 1
fi

chmod 644 /etc/systemd/system/rover-server.service
chmod 644 /etc/systemd/system/rover-lidar.service

systemctl daemon-reload

echo "Enabling and starting rover-lidar service..."
systemctl enable rover-lidar.service
systemctl restart rover-lidar.service

# Poll status API of lidar service and fail setup unless connected=true, device=rover-lidar, health=Good, scans begin
echo "Verifying and validating rover-lidar.service status endpoint..."
lidar_ok=0
for i in {1..15}; do
  echo "Polling localhost:3002/status (attempt $i/15)..."
  response=$(curl -s http://127.0.0.1:3002/status || true)
  if [ -n "$response" ]; then
    connected=$(echo "$response" | grep -o '"connected":[^,]*' | cut -d: -f2 | tr -d ' ' || echo "false")
    device=$(echo "$response" | grep -o '"device":"[^"]*' | cut -d'"' -f4 || echo "")
    health=$(echo "$response" | grep -o '"health":"[^"]*' | cut -d'"' -f4 || echo "")
    hz=$(echo "$response" | grep -o '"scanHz":[^,]*' | cut -d: -f2 | tr -d ' ' || echo "0.0")
    
    if [ "$connected" = "true" ] && [ "$device" = "$LIDAR_DEV" ] && [ "$health" = "Good" ] && [ "$hz" != "0.0" ] && [ "$hz" != "0" ]; then
      echo "LiDAR verification success: connected=true, device=$device, health=$health, scanHz=$hz."
      lidar_ok=1
      break
    fi
  fi
  sleep 1
done

if [ "$lidar_ok" -ne 1 ]; then
  echo "ERROR: LiDAR validation failed after 15 seconds! Output: $response" >&2
  exit 1
fi

echo "Enabling and starting rover-server service..."
systemctl enable rover-server.service
fuser -k 3000/tcp || true
systemctl restart rover-server.service

echo "=== Setup Completed Successfully ==="
echo "Status of rover-server:"
systemctl status rover-server.service --no-pager || true
echo "Status of rover-lidar:"
systemctl status rover-lidar.service --no-pager || true
