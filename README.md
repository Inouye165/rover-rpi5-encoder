# Rover 1 — Host Cockpit, Sensor Bridges & ROS 2 Jazzy / Nav2 Stack (`yahboom-encoder`)

Primary Raspberry Pi 5 host repository (`/home/ron/yahboom-encoder` on `ron@10.0.0.246`) for **Rover 1** (4-motor differential-drive wheel rover).

> [!IMPORTANT]
> **Legacy Repository Naming Clarification**:
> `yahboom-encoder` is a legacy repository and folder name. Rover 1 **does not use a Yahboom motor-driver board**. The active motor controller is the **NULLLAB Maker-ESP32-Pro** (`1.3.4-remap-r03`, `/dev/rover-esp32`) and the auxiliary ToF/Status LED controller is the **Lonely Binary ESP32-S3 N16R8** (`1.2.4-tof-mux`, `/dev/ttyAMA0`).

For authoritative operating rules, hardware pinouts, current session handoff, and open tasks, see:
- [`AGENTS.md`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/AGENTS.md) — Hardware reference, safety boundaries, and verification commands.
- [`HANDOFF.md`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/HANDOFF.md) & [`TODO.md`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/TODO.md) — Current session state and prioritized tasks.
- [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) *(in `esp-maker-usba-4motor`)* — Complete roadmap, test catalog, chronological test ledger, and daily engineering log.

---

## 1. Two-Repository Architecture

| Repository | Local Path | Deployment Target | Role & Ownership |
| :--- | :--- | :--- | :--- |
| **`yahboom-encoder`** *(This Repo)* | `c:\Users\Ron\electronic_projects\yahboom-encoder` | **Raspberry Pi 5** (`ron@10.0.0.246:/home/ron/yahboom-encoder`) | Host software: Node.js Cockpit Server (`server.js` on `:3000`), Rover Ground Control HUD (`public/hud.html`), Mobile Touch HUD (`public/hud-mobile.html`), 8×8 ToF Sensor Bench (`public/sensors.html`), RPLiDAR C1 sidecar (`:3002`), Geekworm X1206 UPS monitor, and ROS 2 Jazzy / Nav2 Docker stack (`ros2/`). |
| **`esp-maker-usba-4motor`** *(Firmware Repo)* | `c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` | **Maker-ESP32-Pro** (`/dev/rover-esp32`) & **ESP32-S3 N16R8** (`/dev/ttyAMA0` / OTA) | Embedded C++ PlatformIO firmware for the 4-motor controller (`src/`, `1.3.4-remap-r03`), ESP32-S3 ToF/LED controller (`esp32s3_status_led/`, `1.2.4-tof-mux`), and reusable exact-motion test suite (`tools/rover_tests/`). |

---

## 2. Hardware Ownership & System Services on Raspberry Pi 5 (`10.0.0.246`)

| Service / Container | Owned Device / Port | Role |
| :--- | :--- | :--- |
| **`rover-server.service`** (`server.js`) | `/dev/rover-esp32` → `/dev/ttyUSB0` (`1a86:7523`, `115200` baud), HTTP/WS `:3000` | Exclusively owns the Maker-ESP32-Pro serial link (`v1.1` binary protocol), enforces operator token auth, 500 ms autonomy watchdog, 600 ms manual drive keepalive watchdog (`manual_drive_watchdog.js`), and serves the HUD Cockpit & REST APIs. |
| **`esp32s3-status-led.service`** | `/dev/ttyAMA0` (`115200` baud GPIO UART0) | Streams RPi5 status heartbeats (`G`/`B`/`R`) to the Lonely Binary ESP32-S3 and writes live 8-channel `TCA9548A` (`0x70`) ToF telemetry to `/tmp/esp32s3_tof_state.json` (served at `GET /api/tof-sensors`). |
| **`rover-lidar.service`** (`rplidar_sidecar.py`) | `/dev/rover-lidar` (`10c4:ea60`, `460800` baud), HTTP `:3002` | Exclusively owns the RPLiDAR C1 serial device and serves 360° scan frames (`GET /scan`, `GET /status`), proxied by `server.js` at `/api/lidar/*`. |
| **`hailo-powersave.service`** | PCIe `0001:04:00.0` (`/dev/hailo0`) | Places the Hailo-8 M.2 AI module into `D3hot` runtime PM with upstream ASM1182e bridge `ASPM L1` when idle. |
| **Docker `rover-ros2`** (`ros2/compose.yaml`) | Host network (`:3010` Nav Bridge, `:8765` Foxglove Bridge), **no `/dev` mounts** | Runs ROS 2 Jazzy (`bringup.launch.py` = `foundation.launch.py` + `navigation.launch.py`): `/odom`, `/imu/data`, `/scan`, `/tof/front_left/*`, `/tof/front_right/*`, `/tf`, AMCL, Smac2D Planner, DWB Controller, Collision Monitor, and `rover_cmd_vel_bridge`. |

---

## 3. Cockpit Web Interfaces (`http://10.0.0.246:3000`)

- **Desktop Ground Control HUD (`/` or `/hud.html`)**:
  - High-contrast tactical HUD with live Geekworm X1206 UPS State-of-Charge (`MAX17043` on RPi5 I2C `0x36`) & charging trend (`battery_trend.js`), BNO08x non-magnetic IMU attitude (`0x4B`), 4-wheel PID/PWM/encoder telemetry, Nav2 route preview & dispatch, and a **combined 360° LiDAR + Front 8×8 ToF Obstacle Radar** with close-in zoom steps (**`0.25 m (10 in)`**, **`0.5 m (20 in)`**, **`1.0 m (3.3 ft)`**, `2 m`, `3 m`, `5 m`, `8 m`).
- **Mobile Touch HUD (`/hud-mobile.html`)**:
  - Landscape 3-column non-overlapping touch interface with dual virtual joysticks, E-STOP, ARM toggle, and close-in obstacle radar.
- **8-Channel ToF Sensor Bench (`/sensors.html`)**:
  - Live 8×8 zone heatmaps and distance matrices for `CH 0` (`front_right_l7cx` `VL53L7CX`, $-8^\circ$ yaw), `CH 1` (`front_left_l7cx` `VL53L7CX`, $+8^\circ$ yaw), and `CH 2` (`VL53L4CD` single-zone).
- **Legacy Engineering Cockpit (`/index.html`)**:
  - Detailed diagnostic tabs and single-motor raised-wheel maintenance test runner.

---

## 4. Sensor & Robot TF Tree (`foundation.launch.py`)

With `base_link` at the physical center of the rover (`+X` forward, `+Y` left, `+Z` up):

| Parent Frame | Child Frame | Translation `(x, y, z)` (m) | Rotation `(roll, pitch, yaw)` | Sensor / Notes |
| :--- | :--- | :--- | :--- | :--- |
| `odom` | `base_link` | Dynamic (`rover_encoder_odometry`) | Dynamic | Calibrated effective wheel diameter `0.06695 m` (`1974.1667` ticks/rev). |
| `base_link` | `laser_frame` | `(+0.03175, 0.0, +0.17145)` | `(0, 0, 0)` | RPLiDAR C1 optical center (`17.15 cm` above `base_link`). |
| `base_link` | `imu_link` | `(-0.0254, -0.0254, +0.14605)` | `(0, 0, 0)` | Adafruit BNO08x IMU (`SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`). |
| `base_link` | `tof_front_left_link` | `(+0.14, +0.0625, +0.08)` *(x, z unverified)* | `(0, 0, +0.139626 rad [+8°])` | Front-Left `VL53L7CX` 8×8 (`CH 1`), 125 mm separation, angled $+8^\circ$ outward. |
| `base_link` | `tof_front_right_link` | `(+0.14, -0.0625, +0.08)` *(x, z unverified)* | `(0, 0, -0.139626 rad [-8°])` | Front-Right `VL53L7CX` 8×8 (`CH 0`), 125 mm separation, angled $-8^\circ$ outward. |

---

## 5. Verification & Deployment Commands

### A. Run Automated Unit Tests Locally
```powershell
# 1. HUD Cockpit, Battery & Watchdog Node.js Tests
node test/test_hud_cockpit.js
node test/test_geekworm_battery.js
node test/test_manual_drive_watchdog.js

# 2. ROS 2 Bridge & Navigation Policy Python Tests
pytest ros2/ros2_ws/src/rover_bringup/test/test_tof_bridge.py ros2/ros2_ws/src/rover_bringup/test/test_lidar_bridge.py ros2/ros2_ws/src/rover_bringup/test/test_nav2_config.py ros2/ros2_ws/src/rover_bringup/test/test_config.py
pytest test/test_production_map_assets.py test/test_cold_start_localization_policy.py test/test_dispatch_localization_refresh.py
```

### B. Deploy Host & ROS 2 Stack to Raspberry Pi 5 (`ron@10.0.0.246`)
```powershell
cd c:\Users\Ron\electronic_projects\yahboom-encoder
$env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
```
*(Uploads updated host & ROS 2 files over SSH key auth, rebuilds `rover_bringup` in `rover-ros2` when needed, restarts services, and verifies all expected ROS 2 nodes and disarmed safety state.)*
