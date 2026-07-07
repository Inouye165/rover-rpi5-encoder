#!/usr/bin/env bash
# ==============================================================================
# RPi5 Setup Script for Encoder Rover Server
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

# 1. Enable I2C interface on Raspberry Pi if not already enabled
echo "Enabling I2C interface..."
if command -v raspi-config &> /dev/null; then
  raspi-config nonint do_i2c 0
else
  # Fallback: add dtparam to config.txt if raspi-config isn't present
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
apt-get install -y python3-smbus python3-pip i2c-tools curl build-essential

# Install Node.js if not present
if ! command -v node &> /dev/null; then
  echo "Node.js not found. Installing Node.js LTS (v20)..."
  curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
  apt-get install -y nodejs
fi

NODE_PATH=$(which node)
PYTHON_PATH=$(which python3)

echo "Node.js path: $NODE_PATH ($(node -v))"
echo "Python path: $PYTHON_PATH ($(python3 --version))"

# 3. Setup Env File
if [ ! -f .env ]; then
  echo "No .env found. Copying from .env.example..."
  cp .env.example .env
  # Set permissions so the real user owns it
  chown $REAL_USER:$REAL_USER .env
  echo "Created .env file. Please ensure it contains correct values."
fi

# Load variables from .env
# Remove windows carriage returns if any
sed -i 's/\r$//' .env
source .env

# 4. Configure WiFi using NetworkManager if SSID and Password are provided
if [ -n "$WIFI_SSID" ] && [ "$WIFI_SSID" != "YourNetworkName" ]; then
  echo "Configuring WiFi connection to SSID: $WIFI_SSID..."
  if command -v nmcli &> /dev/null; then
    # Try to connect to WiFi using nmcli (standard on RPi OS Bookworm)
    if nmcli dev wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD"; then
      echo "WiFi connection successful!"
    else
      echo "Warning: WiFi connection failed, but proceeding."
    fi
  else
    echo "Warning: nmcli tool not found. Skipping auto-WiFi config."
    echo "Ensure your RPi5 is connected to the '$WIFI_SSID' network manually."
  fi
fi

# 5. Node package installation
echo "Installing Node dependencies..."
# Run npm install as the original user to ensure file ownership is correct
sudo -u $REAL_USER npm install --prefix "$WORKING_DIR"

# 6. Stop and Disable legacy I2C Sidecar if active
echo "Cleaning up any legacy rover-i2c service..."
if systemctl is-active --quiet rover-i2c.service 2>/dev/null; then
  echo "Stopping active rover-i2c service..."
  systemctl stop rover-i2c.service || true
fi
if systemctl is-enabled --quiet rover-i2c.service 2>/dev/null; then
  echo "Disabling rover-i2c service..."
  systemctl disable rover-i2c.service || true
fi
if [ -f /etc/systemd/system/rover-i2c.service ]; then
  rm -f /etc/systemd/system/rover-i2c.service
fi

# 7. Install Systemd Services
echo "Installing systemd services..."

# Replace templates with actual paths
SED_EXPR="s|{{USER}}|$REAL_USER|g; s|{{WORKING_DIR}}|$WORKING_DIR|g; s|{{NODE_PATH}}|$NODE_PATH|g; s|{{PYTHON_PATH}}|$PYTHON_PATH|g"

sed "$SED_EXPR" rpi5/rover-server.service.template > /etc/systemd/system/rover-server.service

# Set correct permissions
chmod 644 /etc/systemd/system/rover-server.service

# Reload systemd and enable/start services
echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling and starting rover-server service..."
systemctl enable rover-server.service
systemctl restart rover-server.service

echo "=== Setup Completed Successfully ==="
echo "Status of rover-server:"
systemctl status rover-server.service --no-pager || true

echo "You can view logs using:"
echo "  journalctl -u rover-server -f"
