# Yahboom Rover RPi5 Encoder Dashboard

A high-performance web interface, control cockpit, and real-time telemetry dashboard for the Yahboom 4WD / Mecanum Rover tank, running on a Raspberry Pi 5.

This project enables manual control of the rover's wheels and monitors real-time feedback (ticks, speed, battery, and 9-axis IMU orientation) over both serial and I2C protocols.

---

## Architecture

The project consists of two key components:

1. **Python I2C Sidecar (`yahboom_i2c.py`)**: 
   A lightweight background service that polls the low-level STM32 expansion board at I2C address `0x34` (Bus 1) to fetch high-frequency telemetry (9-axis IMU orientation, accelerometer, gyro, magnetometer, and battery voltage) and serves it over a local JSON API.
   
2. **Node.js Web & Serial Server (`server.js`)**:
   An Express + WebSocket server that manages the serial interface (`/dev/ttyAMA0` or `COM18`) for motor controls, polls the Python I2C sidecar, and streams telemetry directly to WebSocket clients on the web dashboard.

---

## Getting Started

### 1. Prerequisites
Ensure your Raspberry Pi has I2C enabled:
```bash
sudo raspi-config nonint do_i2c 0
```

Install System dependencies:
```bash
sudo apt-get update
sudo apt-get install -y python3-smbus i2c-tools
```

### 2. Environment Configuration
Copy the configuration template and create your local environment variables:
```bash
cp .env.example .env
```
Edit `.env` to configure your serial port, I2C bus, and target address values.

### 3. Installation
Install the Node.js package dependencies:
```bash
npm install
```

### 4. Running the Dashboard
Start the Python I2C sidecar:
```bash
python3 yahboom_i2c.py
```

In a separate terminal, start the main Node.js dashboard server:
```bash
npm start
```
Open your browser and navigate to `http://<your-rpi-ip>:3000` to access the Cockpit Dashboard!

---

## File Structure

* `server.js` - Main Express + WebSocket server
* `yahboom_i2c.py` - Python SMBus I2C telemetry sidecar
* `public/` - Dashboard client-side assets (HTML/JS/CSS)
* `.env.example` - Template config file
* `.gitignore` - Standard git exclusions (ignores `.env`, `node_modules`, log files, etc.)
