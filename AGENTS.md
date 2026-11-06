# Rover 1 Agent Instructions (`AGENTS.md`)

Authoritative operating instructions, hardware reference, workflow rules, and safety boundaries for AI coding agents working in the **Rover 1** host/cockpit/ROS 2 repository (`yahboom-encoder`) and companion firmware repository (`esp-maker-usba-4motor`).

---

## 1. Project Identity & Rover 1 Build

- **Rover 1 Only**: This repository (`yahboom-encoder`) and `esp-maker-usba-4motor` are exclusively for **Rover 1** (4-motor differential-drive rover). Keep **Rover 2** hardware, IP addresses, and configurations strictly separate.
- **Authoritative References** (consult and maintain rather than duplicating tables):
  - [`PROJECT_STATUS.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/PROJECT_STATUS.md) — Active firmware build, roadmap, reusable test catalog, chronological test ledger, and **Section 6 Daily Engineering & Activity Log**.
  - [`HANDOFF.md`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/HANDOFF.md) & [`TODO.md`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/TODO.md) — Current session handoff state, ToF/ROS 2/Radar architecture, and prioritized task list.
  - [`docs/HANDOFF_TOF_SENSOR_BENCH_AND_RADAR_20261105.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/HANDOFF_TOF_SENSOR_BENCH_AND_RADAR_20261105.md) — 2026-11-05 ESP32-S3 3-sensor ToF multiplexer bench, ROS 2 `rover_tof_bridge`, HUD radar micro-zoom, and code audit fixes.
  - [`docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/HANDOFF_REBUILT_CHASSIS_CALIBRATION_20261018.md) — Rebuilt chassis port remap, UPS power isolation validation, and exact-motion calibration baselines.
  - [`docs/EXACT_MOTION_CONTRACT.md`](file:///c:/Users/Ron/electronic_projects/esp/esp-maker-usba-4motor/docs/EXACT_MOTION_CONTRACT.md) — Mandatory controllers and motion profiles for physical movement.

### Two-Repository Architecture & Host Connections
| Component | Path / Target | Role & Ownership |
| :--- | :--- | :--- |
| **Firmware Repo** | `c:\Users\Ron\electronic_projects\esp\esp-maker-usba-4motor` | PlatformIO C++ firmware for NULLLAB Maker-ESP32-Pro (`env:esp32dev`), ESP32-S3 ToF/LED (`esp32s3_status_led`), + `tools/rover_tests` exact-motion test suite. |
| **Companion Host Repo (This Repo)** | `c:\Users\Ron\electronic_projects\yahboom-encoder` → `/home/ron/yahboom-encoder` on RPi5 | Legacy repo name (Yahboom driver board is **not** used). Hosts Cockpit API (`:3000`), Nav Bridge (`:3010`), LiDAR sidecar, and ROS 2 Jazzy Docker stack. |
| **Raspberry Pi 5 Host** | `ron@10.0.0.246` (SSH key auth) | Primary onboard compute. `rover-server.service` exclusively owns `/dev/rover-esp32`; `rover-lidar.service` exclusively owns `/dev/rover-lidar`; `esp32s3-status-led.service` owns `/dev/ttyAMA0`. |

### Installed Hardware (Verified)
- **Power Architecture**:
  - **Compute & Logic Rail**: **Geekworm X1206 UPS** (1S4P 21700 lithium cells mounted beneath RPi5; MAX17043 fuel gauge on RPi5 I2C `0x36`) powers the Raspberry Pi 5 and USB-connected ESP32 boards, isolating logic from motor load sag.
  - **Motor Rail**: Dedicated **3S LiPo** pack supplies motor power (`VMOT`) on the Maker-ESP32-Pro (controlled by the board's physical power switch).
- **Primary Motor Controller**: **NULLLAB Maker-ESP32-Pro** (`ESP32-WROOM-32E`), connected to RPi5 via USB CH340 (`1a86:7523`, `/dev/rover-esp32` → `/dev/ttyUSB0`, `115200` baud).
  - **Active Verified Firmware**: **`1.3.4-remap-r03`** (`source: rev42-remap-c`, commit `a2e3d43`).
  - **Calibrated Kinematics**: Effective wheel diameter `0.06695 m` (radius `0.033475 m`), `1974.1667` ticks/rev, effective differential track width `0.3408575433 m`.
- **Auxiliary Sensor/LED Controller**: **Lonely Binary ESP32-S3 N16R8** on RPi5 GPIO UART0 (`/dev/ttyAMA0`, `115200` baud; USB `303a:1001` MAC `58:e6:c5:71:6a:98` unplugged during normal operation; OTA hostname `rover1-esp32s3`).
  - **I2C Multiplexer (`TCA9548A` / `PCA9548A`, addr `0x70`)**: `SDA = GPIO 8`, `SCL = GPIO 9` at `200 kHz` (`150 ms` timeout).
  - **`CH 0` (`front_right_l7cx`)**: Front-Right **`VL53L7CX`** 8×8 (`0x29`, `MODEL=0xF002`, angled $-8^\circ$ outward, `rot="90_ccw"`).
  - **`CH 1` (`front_left_l7cx`)**: Front-Left **`VL53L7CX`** 8×8 (`0x29`, `MODEL=0xF002`, angled $+8^\circ$ outward, `rot="90_ccw"`).
  - **`CH 2` (`unassigned` bench/cliff)**: Single-zone **`VL53L4CD`** (`0x29`, `MODEL=0xEBAA`, monitored in `/api/tof-sensors` and `/sensors.html`, excluded from ROS 2 and radar until mounted/mapped).
  - **ROS 2 & Radar Integration**: [`rover_tof_bridge.py`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_tof_bridge.py) publishes `/tof/front_left/{points,range}` (`tof_front_left_link`) and `/tof/front_right/{points,range}` (`tof_front_right_link`), and [`public/hud-core.js`](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/hud-core.js) overlays 8×8 front-corner detections as magenta squares on the HUD radar with close-in zoom steps (`0.25 m` / `10 in`, `0.50 m` / `20 in`, `1.0 m` / `3.3 ft`).

---

## 2. Verification & Deployment Commands

- **Run HUD & Node.js Unit Tests**:
  ```powershell
  node test/test_hud_cockpit.js
  node test/test_geekworm_battery.js
  node test/test_manual_drive_watchdog.js
  ```
- **Run Python & ROS 2 Bridge Unit Tests**:
  ```powershell
  pytest ros2/ros2_ws/src/rover_bringup/test/test_tof_bridge.py ros2/ros2_ws/src/rover_bringup/test/test_lidar_bridge.py ros2/ros2_ws/src/rover_bringup/test/test_nav2_config.py ros2/ros2_ws/src/rover_bringup/test/test_config.py
  pytest test/test_production_map_assets.py test/test_cold_start_localization_policy.py test/test_dispatch_localization_refresh.py
  ```
- **Deploy Host & ROS 2 Changes to RPi5 (`ron@10.0.0.246`)**:
  ```powershell
  $env:ROVER_PI_HOST="10.0.0.246"; $env:ROVER_PI_USER="ron"; python deploy_yahboom.py
  ```
