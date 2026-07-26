# Cockpit Target Information Architecture & Canonical Data Model

**Repository:** `C:\Users\Ron\electronic_projects\yahboom-encoder`  
**Branch:** `feature/ros2-encoder-odom-tf`  
**Date:** July 26, 2026  
**Stage:** Stage 1 — Inventory & Analysis  

---

## 1. Feature Disposition & Classification

Every existing feature in the cockpit is assigned exactly one proposed disposition:

| Feature / Panel | Existing Location | Proposed Disposition | Proposed Destination | Backend Code Status | Test Impact | Motor / Safety Risk |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Header Status Bar (WS, Serial, Battery, Time)** | Global Header | **KEEP** | Global Header | Retain backend serial & battery broadcast | None | None |
| **Driving Control Room (Arm/Disarm, Gamepad, Drive Telemetry)** | `tab-dashboard` | **KEEP** | **Tab 1: Drive** | Retain `/api/drive/*` endpoints | None | High (Manual drive) |
| **RPi Camera Feed** | `tab-dashboard` | **KEEP** | **Tab 1: Drive** | Retain `/api/camera/*` endpoints | None | None |
| **Global Emergency Stop (E-Stop)** | Header / Dashboard | **KEEP** | Global Header & **Tab 1: Drive** | Retain `/api/stop` & `FUNC_EMERGENCY_STOP` | None | High (Safety stop) |
| **Path Backtracking Controls** | `tab-dashboard` & `tab-motion-cal` | **CONSOLIDATE** | **Tab 1: Drive** | Retain `/api/path/*` endpoints | Remove duplicate assertions from `tab-motion-cal` | High (Autonomous return) |
| **ROS 2 Stack Health & Service Badges** | `tab-ros2` | **MOVE** | **Tab 2: Autonomy** | Retain ROS 2 system health bridge | Update selector tests for `tab-autonomy` | None |
| **TF & Costmap Diagnostics** | `tab-ros2` / New | **MOVE** | **Tab 2: Autonomy** | Add backend ROS 2 TF listener metrics | Add TF health test | None |
| **Nav2 Lifecycle & Goal Progress** | New | **KEEP** | **Tab 2: Autonomy** | Add Nav2 action server status listener | Add Nav2 goal test | High (Nav2 movement) |
| **Foxglove Access Banner** | `tab-ros2` | **MOVE** | **Tab 2: Autonomy** | Retain static link (Port 8765) | Update tab test | None |
| **LiDAR 360° Polar Display & Health** | `tab-lidar` | **MOVE** | **Tab 3: Sensors** | Retain `/api/lidar/*` & `rplidar_sidecar.py` | Update tab selector tests | None |
| **3D Rover Attitude & IMU Sensor Readings** | `tab-imu` | **MOVE** | **Tab 3: Sensors** | Retain IMU WebSocket stream (`TYPE_IMU`) | Update tab selector tests | None |
| **Wheel Odometry & 2D Position Canvas** | `tab-imu` | **MOVE** | **Tab 3: Sensors** | Retain `/api/encoders` & `/odom` topic | Update tab selector tests | None |
| **Wheel Verification & Pulse Tests** | `tab-encoder` | **MOVE** | **Tab 4: Calibration** | Retain `set_pwm` WebSocket handlers | Update tab selector tests | High (Individual motor spin) |
| **Distance Calibration (2-Meter)** | `tab-motion-cal` | **MOVE** | **Tab 4: Calibration** | Retain `calibration_db.json` handlers | Update tab selector tests | High (2m drive test) |
| **Rotation Calibration (360°)** | `tab-motion-cal` | **MOVE** | **Tab 4: Calibration** | Retain `calibration_db.json` handlers | Update tab selector tests | High (360° spin test) |
| **Automatic 1 m / 90° Closed-Loop Tests** | `tab-calibrate` | **CONSOLIDATE** | **Tab 4: Calibration** | Retain `/api/calibration/auto/*` endpoints | Retain `test_auto_calibration.py` | High (Autonomous floor movement) |
| **Phase 5 Repeatability Statistics & History** | `tab-motion-cal` | **MOVE** | **Tab 4: Calibration** | Retain `/api/calibration/repeatability/*` | Retain `test_repeatability_sim.js` | High (5-trial series) |
| **Phase 3 Raised-Wheel Maintenance Panel** | `tab-calibrate` | **ADVANCED** | **Tab 4: Calibration (Advanced Section)** | Retain `/api/maintenance/*` & C++ firmware | Retain `test_maintenance_safety.py` | Moderate (Raised wheel 2s test) |
| **LiDAR Straight-Line Correction & Wizard** | `tab-encoder` | **ADVANCED** | **Tab 4: Calibration (Advanced Section)** | Retain `/api/lidar/test/*` endpoints | Update DOM contract test | Moderate (Applies steering effort) |
| **Service Status & ROS Topic Rates** | `tab-ros2` | **MOVE** | **Tab 5: Diagnostics** | Retain ROS 2 diagnostics bridge | Update selector tests | None |
| **Firmware & Serial Protocol Info** | Footer / `tab-calibrate` | **MOVE** | **Tab 5: Diagnostics** | Retain `/api/firmware` endpoint | Update selector tests | None |
| **Raw Terminal & Serial Command Input** | Global Footer | **MOVE** | **Tab 5: Diagnostics** | Retain WebSocket `raw_command` handler | Update selector tests | High (Raw serial packet write) |
| **Calibration Database Raw Inspector** | New | **MOVE** | **Tab 5: Diagnostics** | Retain `calibration_db.json` endpoints | None | None |
| **Breakaway Calibration Simulation** | `tab-calibrate` | **ARCHIVE** | **Archive Code Base** | Deprecate `/api/calibration/simulate/*` | Update test mocks | None (Simulation disabled) |
| **Yahboom Legacy Flash Config Form** | `tab-dashboard` | **ARCHIVE** | **Archive Code Base** | Retain warning handler in `server.js` | None | None |
| **Duplicate Backtracking Panel** | `tab-motion-cal` | **REMOVE-LATER** | N/A | None (Remove frontend markup duplicate) | Remove HTML duplicate assertions | None |
| **Phase-Number UI Labels** | Across tabs | **REMOVE-LATER** | N/A | None (Text label cleanup) | Update string match assertions | None |

---

## 2. Proposed Top-Level Navigation & Tab Structure

The redesigned cockpit top-level navigation consists of exactly 5 clean, functional tabs:

```
[🤖 MAKER ESP32 PRO COCKPIT] | [🔋 12.4V] | [🚨 E-STOP] | [COM18 Connected]
└─ Nav Tabs: [1. 🎮 Drive] [2. 🤖 Autonomy] [3. 📡 Sensors] [4. 🛡️ Calibration] [5. 💻 Diagnostics]
```

### Tab 1: 🎮 Drive
- **Primary User Goal:** Reliable manual rover teleoperation, live camera monitoring, drive arming, deadman state inspection, and path backtracking home.
- **Information Shown by Default:**
  - Connection Summary (WS, Serial port, battery voltage)
  - Gamepad Live Controller Inputs HUD
  - Drive State (Armed / Disarmed, Deadman switch state)
  - E-Stop State & Global Emergency Stop Button
  - Current Linear / Angular Velocity Commands (Requested vs Limited)
  - Skid-Steer Wheel Odometry HUD
  - RPi Camera Feed
  - Path Recording & Return Home Backtrack Control
  - Direction D-Pad & Speed Sliders
- **Information Collapsed by Default:**
  - Advanced Gamepad Axis Deadband Tuning
- **Controls That Can Move Rover:**
  - Gamepad Joysticks (when Deadman held and Armed)
  - Direction D-Pad (▲, ◀, ▶, ▼, ↺, ↻)
  - Speed Sliders (Synchronized & Individual M1..M4)
  - "Backtrack Home" Button
- **Controls Requiring Confirmation:**
  - "Arm Normal Drive" Button
  - "Backtrack Home" Button
- **Emergency Actions:**
  - Global "🚨 EMERGENCY STOP ALL MOTORS" Button (Always visible)
  - Gamepad Buttons A / B (Hardware E-Stop trigger)
- **Empty / Loading / Error States:**
  - Camera Feed Offline placeholder graphic with "Start Feed" button
  - Gamepad Disconnected banner when controller is unplugged
  - Serial Disconnected warning badge when COM port is offline

---

### Tab 2: 🤖 Autonomy
- **Primary User Goal:** Monitor ROS 2 navigation stack, SLAM mapping, localization state, Nav2 lifecycle, goal progress, and access Foxglove web visualizer.
- **Information Shown by Default:**
  - ROS 2 Stack Health Status Cards (`rover_bringup`, `rover_encoder_odometry`, `rover_lidar_bridge`)
  - Localization State (Pose X, Y, Yaw, covariance confidence)
  - Map & SLAM State (Active map name, grid resolution, update rate)
  - Nav2 Lifecycle State (Unconfigured, Inactive, Active, Finalized)
  - Current Navigation Goal & Distance-to-Goal Progress Bar
  - Costmap Health & Obstacle Warning Banner
  - Foxglove Web Visualizer Link Card (Port 8765)
- **Information Collapsed by Default:**
  - Detailed TF Tree Transform Latencies (`odom` -> `base_link`, `base_link` -> `laser`)
  - Nav2 Action Server Goal History
- **Controls That Can Move Rover:**
  - Nav2 Goal Dispatcher (when autonomous mode is active)
- **Controls Requiring Confirmation:**
  - "Set Nav Goal" Action
- **Emergency Actions:**
  - "🛑 Cancel Navigation Goal" Button
  - Global E-Stop (Fixed in header bar)
- **Empty / Loading / Error States:**
  - "ROS 2 Navigation Stack Inactive" banner when Nav2 nodes are not publishing
  - "No Active Navigation Goal" state when rover is stationary

---

### Tab 3: 📡 Sensors
- **Primary User Goal:** Detailed inspection and monitoring of all onboard sensor payloads (LiDAR, IMU, Wheel Encoders, future Ultrasonics/Cameras).
- **Information Shown by Default:**
  - RPLIDAR C1 360° Polar Scan Canvas & Connection Status
  - 3D Rover Attitude Visualizer (Roll, Pitch, Yaw)
  - 3-Axis Accelerometer & 3-Axis Gyroscope Readouts
  - Wheel Odometry Trajectory Canvas & Speed Readouts
  - Per-Sensor Health Status Cards (LiDAR Scan Hz, IMU Update Hz, Encoder Packet Rate)
- **Information Collapsed by Default:**
  - Raw LiDAR Sample Point Table (Angle, Distance mm, Quality)
  - Raw IMU Register Calibration Offsets
- **Controls That Can Move Rover:** None (Read-only sensor monitoring tab)
- **Controls Requiring Confirmation:** None
- **Emergency Actions:** Global E-Stop (Fixed in header bar)
- **Empty / Loading / Error States:**
  - LiDAR Stale Data Overlay on polar canvas when scan age exceeds 1.0s
  - "IMU Disconnected" alert card if IMU telemetry stops

---

### Tab 4: 🛡️ Calibration
- **Primary User Goal:** Commissioning and calibrating physical rover kinematics (wheel diameter, track width, balance trims, closed-loop auto-calibration, and Phase 5 repeatability verification).
- **Information Shown by Default:**
  - Wheel Verification & Direction Pulse Tests (M1..M4 FWD/REV)
  - Distance Calibration Card (2-Meter test & actual measurement inputs)
  - Rotation Calibration Card (360° CW/CCW test & actual measurement inputs)
  - Automatic Closed-Loop Tests (1m Forward, 90° Left, 90° Right)
  - Live Closed-Loop Test Progress HUD & Completion Result Card
  - Repeatability Counters & Pass Rate Statistics Cards
  - Calibrated Motion Command Test Panel
- **Information Collapsed by Default:**
  - Advanced Maintenance Section (Phase 3 Raised-Wheel Individual Motor Tests)
  - Advanced Commissioning Section (LiDAR Straight-Line Correction & Orientation Wizard)
  - Repeatability Test Logs History Table & Export Buttons (JSON/CSV)
- **Controls That Can Move Rover:**
  - Wheel Test Pulse Buttons (M1..M4 FWD/REV)
  - "Run 2-Meter Distance Test" Button
  - "Run 360° Rotation Test" Button
  - "Auto 1 m Forward", "Auto 90° Left", "Auto 90° Right" Buttons
  - "Run 5-Trial Repeatability Series" Button
  - Individual Wheel Maintenance Test Buttons (in Advanced section)
- **Controls Requiring Confirmation:**
  - Auto Calibration Modal Confirmation ("Confirm Physical Movement")
  - "Save/Apply Trims to ESP32 NVS" Button
  - "Clear Calibration Database" Action
- **Emergency Actions:**
  - "🛑 Abort Auto Test" Button
  - "🛑 STOP ALL MAINTENANCE" Button
  - Global E-Stop (Fixed in header bar)
- **Empty / Loading / Error States:**
  - "No Calibration Trial Logged" empty state in trial tables
  - "Encoder Telemetry Stale" error banner blocking test start

---

### Tab 5: 💻 Diagnostics
- **Primary User Goal:** Deep developer troubleshooting, service monitoring, serial packet inspection, firmware debugging, raw command terminal, and calibration database inspection.
- **Information Shown by Default:**
  - System Service Status Overview (Node.js Server, Python Sidecars, ROS 2 Nodes)
  - Firmware & Binary Protocol Identity (Firmware version, build timestamp, protocol version)
  - Serial Connection Details (Baud rate 115200, COM port, TX/RX packet counters)
  - ROS 2 Topic Rate Monitor (`/diagnostics`, `/scan`, `/odom`, `/tf`)
  - Control Loop Timing & Latency Statistics
  - Live Driver Logs Console & Command Terminal
- **Information Collapsed by Default:**
  - Raw Serial Hex Packet Stream Log
  - Active `calibration_db.json` Raw File Inspector
  - Archived Commissioning Logs & Hardware PDF Documentation Links
- **Controls That Can Move Rover:**
  - Raw Hex Serial Command Prompt (if user sends motor packets directly)
- **Controls Requiring Confirmation:**
  - "Clear Driver Logs" Action
  - "Reset Timing Statistics" Action
- **Emergency Actions:** Global E-Stop (Fixed in header bar)
- **Empty / Loading / Error States:**
  - "Serial Port Offline" warning card
  - "No Driver Log Output" empty console state

---

## 3. Canonical Sources of Truth

To eliminate data ambiguity, every repeated telemetry or configuration parameter is assigned exactly one canonical backend source of truth:

| Data Parameter | Canonical Backend Source of Truth | Secondary / Legacy Sources to Deprecate |
| :--- | :--- | :--- |
| **Armed State** | `server.js` `normalDriveState.armed` & ESP32 `TYPE_NORMAL_DRIVE_STATUS` telemetry | Local frontend UI variables |
| **Motor Command Ownership** | `server.js` `sendMotorSpeeds()` wrapper & Firmware E-stop lock | Direct uncoordinated WebSocket calls |
| **Wheel Odometry (X, Y, Yaw)** | ROS 2 `rover_encoder_odometry` node (`/odom` topic) | Frontend dead-reckoning canvas integration |
| **IMU Attitude (Roll, Pitch, Yaw)** | ESP32 `TYPE_ATTITUDE` / `TYPE_IMU` binary telemetry packet | Frontend local integration |
| **LiDAR Scan & Health** | `rplidar_sidecar.py` process & `/scan` topic | Direct serial port polling |
| **ROS 2 Stack Health** | ROS 2 `/diagnostics` topic bridge on RPi5 | Static HTML badges |
| **Kinematic Constants** | `calibration_db.json` (`wheelDiameter`, `effectiveTrackWidth`) | Static HTML text descriptions & `config-form` inputs |
| **Balance Trims (FWD / REV)** | `calibration_db.json` (`fwdTrim`, `revTrim`) & ESP32 NVS | Frontend trim input defaults |
| **Repeatability History & Stats** | `calibration_db.json` (`testLogs` array) & `/api/calibration/repeatability/history` | Local browser localStorage |
| **Firmware Identity** | ESP32 `TYPE_FIRMWARE_INFO` binary packet response | Hardcoded HTML string labels |
| **Serial Port State** | `server.js` `serialPort.isOpen` & `COM_PORT` variable | Frontend `serial-status` badge text |

---

## 4. Backend vs. Frontend Calculation Ownership

1. **Safety State Ownership (100% Backend):**
   - The backend `server.js` and ESP32 firmware maintain strict ownership of E-Stop lock, Deadman timeout, raised-wheel maintenance mode, and drive arming state. The frontend must NEVER calculate or assume safety readiness independently.

2. **Calibration Results & Statistics Ownership (100% Backend):**
   - Proposed wheel diameters, proposed track widths, mean errors, standard deviations, pass rates, and recommendation statuses MUST be computed by `server.js` (or python scripts) and written to `calibration_db.json`. The frontend strictly renders the computed JSON responses.

3. **Motor-Command Concurrency Ownership (100% Backend):**
   - The backend enforces single-owner motor control (e.g., rejecting an auto-calibration start if maintenance test is active, or rejecting joystick drive during an active 3ft auto-test).

4. **Identified State Derivation Divergences to Fix:**
   - **Odometry Yaw:** Currently derived via wheel integration in `app.js` canvas, via IMU in `app.js` 3D model, and via ICP in LiDAR sidecar. Fixed: `/odom` topic output is the sole canonical source for vehicle pose.
   - **Kinematic Parameters:** Currently declared as `0.065m / 0.382m` in static HTML text, `11 phase lines / 30 ratio` in Yahboom form, and dynamic float values in `calibration_db.json`. Fixed: `calibration_db.json` serves as sole dynamic truth.
