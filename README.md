# Yahboom Rover Cockpit & Controller

A high-performance control cockpit and telemetry dashboard for a custom 4WD / Mecanum Rover tank. The system is designed to run on a **Raspberry Pi 5** on board the rover, which communicates with a **Maker ESP32 Pro Board** (the low-level motor/encoder controller) and drives **4 wheel encoder motors**.

---

## 🏗️ Architecture

```
                                +-----------------------------+
                                |      Web Client Browser     |
                                |     (Control Cockpit UI)    |
                                +--------------+--------------+
                                               | (WebSockets / HTTP)
                                               v
+----------------------------------------------+----------------------------------------------+
| Raspberry Pi 5 On-Board Computer                                                            |
|                                                                                             |
|   +---------------------------------------+       +---------------------------------------+ |
|   |         Node.js Cockpit Server        |       |          Python I2C Sidecar           | |
|   |             (server.js)               |       |           (yahboom_i2c.py)            | |
|   +-------------------+-------------------+       +-------------------+-------------------+ |
|                       |                                               |                     |
|                       | (USB Serial /dev/ttyUSB0)                     | (I2C Bus 1)         |
|                       v                                               v                     |
+-----------------------+-----------------------------------------------+---------------------+
                        |                                               |
                        v                                               v
        +---------------+---------------+               +---------------+---------------+
        |     Maker ESP32 Pro Board     |               |    Yahboom STM32 Subsystem    |
        |    (Low-level Motor Controller) |               |     (Battery/IMU Backup)      |
        +---------------+---------------+               +-------------------------------+
                        |
                        +---> 4x Encoder Motors
```

The system comprises three core parts:
1. **Low-level Brains (Maker ESP32 Pro Board)**: Directly interfaces with the 4 encoder motors (M1..M4), processing encoder ticks and driving motor power. It communicates with the host server over a fast binary protocol via USB serial.
2. **Host Server (Node.js)**: Runs on the RPi5. Serves the web interface, manages the serial connection to the ESP32 (`/dev/ttyUSB0`), processes joystick speed commands, and handles motor closed-loop position control.
3. **I2C Sidecar (Python)**: Reads secondary sensor data (attitude/battery telemetry) from the Yahboom STM32 controller board at I2C address `0x34` (Bus 1) and exposes it locally for the Node.js server.

---

## 🛠️ Deploying to the Raspberry Pi 5

We have automated the deployment pipeline using RPi5-specific systemd service configurations and automated scripts.

### 1. Configure Local Environment
Create or edit your local `.env` file (which is git-ignored) in the root of the project to define the target network credentials and serial port:
```env
# WiFi Credentials (used by setup scripts)
WIFI_SSID=Dobby
WIFI_PASSWORD=sanmina-1

# Server Settings
PORT=3000
SERIAL_PORT=/dev/ttyUSB0
BAUD_RATE=115200

# I2C Sidecar Settings
I2C_SIDECAR_URL=http://127.0.0.1:3001/data
I2C_PORT=3001
I2C_BUS=1
I2C_ADDRESS=0x34
I2C_INTERVAL=0.1
```

### 2. Deploy from Windows to RPi5
Open a PowerShell terminal in the project directory on your Windows dev machine and execute:
```powershell
.\rpi5\deploy.ps1
```
*This script will pack the project files (excluding dependencies/cache), transfer them to the Pi at `10.0.0.247`, and execute the remote installation script using the password `Sanmina-1`.*

### 3. Remote Setup Details
The script automatically executes `rpi5/setup.sh` on the RPi5, which:
* Installs system prerequisites (`nodejs`, `python3-smbus`, `i2c-tools`, `build-essential`).
* Ensures I2C hardware is enabled.
* Configures local NetworkManager to connect to the configured WiFi (e.g. `Dobby`).
* Installs all Node dependency modules.
* Installs and launches two persistent systemd background services:
  * **`rover-server.service`**: Automatically starts the Node server on boot.
  * **`rover-i2c.service`**: Automatically starts the Python I2C telemetry poller on boot.

---

## 🩺 Monitoring and Maintenance

Once deployed, the Cockpit webpage is served at:
**`http://10.0.0.247:3000`**

To monitor the background services on the Pi, SSH into `10.0.0.247` and run:

```bash
# Check service status
systemctl status rover-server.service
systemctl status rover-i2c.service

# View real-time output logs
journalctl -u rover-server.service -f
journalctl -u rover-i2c.service -f
```

---

## 📂 Project Structure

* `server.js` - Primary Express & WebSocket telemetry + control server.
* `yahboom_i2c.py` - Python background poller for the STM32 board.
* `public/` - Dashboard web frontend (HTML/CSS/JS).
* `maker_esp32_pro/` - Firmware project directory for the ESP32 microcontroller.
* `rpi5/` - Deployment artifacts:
  * `deploy.ps1` - Windows packaging & SSH/SFTP deployment automation.
  * `setup.sh` - System configuration, dependencies, and service installation.
  * `rover-server.service.template` / `rover-i2c.service.template` - Service unit configuration templates.
