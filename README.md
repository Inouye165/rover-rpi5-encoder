# Rover Cockpit & Telemetry Dashboard

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
|   +---------------------------------------------------------------------------------------+ |
|   |                                 Node.js Cockpit Server                                | |
|   |                                       (server.js)                                     | |
|   +-------------------------------------------+-------------------------------------------+ |
|                                               |                                             |
|                                               | (USB Serial /dev/ttyUSB0)                   |
|                                               v                                             |
+-----------------------------------------------+---------------------------------------------+
                                                |
                                                v
                                +---------------+---------------+
                                |     Maker ESP32 Pro Board     |
                                |  (Low-level Motor Controller) |
                                +---------------+---------------+
                                                |
                                                +---> 4x Encoder Motors
```

The system comprises two core parts:
1. **Low-level Brains (Maker ESP32 Pro Board)**: Directly interfaces with the 4 encoder motors (M1..M4), processing encoder ticks and driving motor power. It communicates with the host server over a fast binary protocol via USB serial. It also mocks gyro/accelerometer data based on motor control and sends simulated battery level.
2. **Host Server (Node.js)**: Runs on the RPi5. Serves the web interface, manages the serial connection to the ESP32 (`/dev/ttyUSB0`), processes joystick speed commands, and handles motor closed-loop position control.

---

## 🔩 Hardware References

Use this section to store authoritative hardware links (board repositories, schematics, pinout docs, and vendor resources).

* **Maker ESP32 Pro (physical driver/control board) repository:** https://github.com/nulllaborg/maker-esp32-pro.git
* **Raspberry Pi 5 GPIO Pinout:**
  ![Raspberry Pi 5 GPIO Pinout](pictures/rpi5_gpio.jpg)

---

## 🛠️ Deploying to the Raspberry Pi 5

We have automated the deployment pipeline using RPi5-specific systemd service configurations and automated scripts.

### 1. Configure Local Environment
Create or edit your local `.env` file (which is git-ignored) in the root of the project to define the target network credentials and serial port:
```env
# WiFi Credentials (used by setup scripts)
WIFI_SSID=<your_wifi_ssid>
WIFI_PASSWORD=<your_wifi_password>

# RPi5 Deployment & Target Configurations
RPI_IP=<your_rpi5_ip_address>
RPI_USER=ron
RPI_PASSWORD=<your_rpi5_password>

# Server Settings
PORT=3000
SERIAL_PORT=/dev/ttyUSB0
BAUD_RATE=115200
```

### 2. Deploy from Windows to RPi5
Open a PowerShell terminal in the project directory on your Windows dev machine and execute:
```powershell
.\rpi5\deploy.ps1
```
*This script will pack the project files (excluding dependencies/cache), transfer them to the Pi (using details specified in your `.env` file), and execute the remote installation script.*

### 3. Remote Setup Details
The script automatically executes `rpi5/setup.sh` on the RPi5, which:
* Installs system prerequisites (`nodejs`, `build-essential`).
* Configures local NetworkManager to connect to the configured WiFi (defined in `.env` as `WIFI_SSID`).
* Installs all Node dependency modules.
* Installs and launches the persistent systemd background service:
  * **`rover-server.service`**: Automatically starts the Node server on boot.

---

## 🩺 Monitoring and Maintenance

Once deployed, the Cockpit webpage is served at:
**`http://<your_rpi5_ip_address>:3000`**

To monitor the background services on the Pi, SSH into `<your_rpi5_ip_address>` and run:

```bash
# Check service status
systemctl status rover-server.service

# View real-time output logs
journalctl -u rover-server.service -f
```

---

## 📂 Project Structure

* `server.js` - Primary Express & WebSocket telemetry + control server.
* `public/` - Dashboard web frontend (HTML/CSS/JS).
* `maker_esp32_pro/` - Firmware project directory for the ESP32 microcontroller.
* `rpi5/` - Deployment artifacts:
  * `deploy.ps1` - Windows packaging & SSH/SFTP deployment automation.
  * `setup.sh` - System configuration, dependencies, and service installation.
  * `rover-server.service.template` - Service unit configuration template.
