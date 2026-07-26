# Cockpit Redundancy & Legacy Feature Matrix

**Repository:** `C:\Users\Ron\electronic_projects\yahboom-encoder`  
**Branch:** `feature/ros2-encoder-odom-tf`  
**Date:** July 26, 2026  
**Stage:** Stage 1 — Inventory & Analysis  

---

## 1. Multi-Location Redundancy Matrix

The table below catalogs every piece of telemetry, state, control, or diagnostic data that currently appears in multiple locations across the cockpit tabs and panels:

| Telemetry / Feature Category | Location 1 (Primary / Current) | Location 2 (Secondary / Duplicate) | Location 3 (Tertiary / Legacy) | Summary of Duplication / Divergence |
| :--- | :--- | :--- | :--- | :--- |
| **Encoder Counts** | `tab-dashboard` (Wheel Telemetry Cards: `telemetry-total-m1`..`m4`) | `tab-dashboard` (Skid-Steer HUD: `odom-enc-m1`..`m4`) | `tab-encoder` (`test-ticks-m1`..`m4`) & `tab-calibrate` (`enc-start-m1`..`m4`) | Appears in 4 separate panels. `tab-dashboard` shows total boot ticks and real-time RPM, `tab-encoder` shows test ticks, `tab-calibrate` shows per-test start/end snapshots. |
| **Wheel Odometry (X, Y)** | `tab-dashboard` (Skid-Steer Odometry HUD: `odom-x-real`, `odom-y-real`) | `tab-imu` (Rover Position Tracker: `odom-x`, `odom-y`) | `tab-calibrate` (Auto Calib HUD: `auto-calib-val-dist`) | Formatted differently (`odom-x-real` in meters, `odom-x` in meters with canvas plot, `auto-calib-val-dist` as displacement). |
| **Heading Yaw** | `tab-dashboard` (Skid-Steer Odometry HUD: `odom-yaw-real`) | `tab-imu` (3D Rover Attitude: `imu-yaw`) | `tab-encoder` (LiDAR Stat: `stat-lidar-yaw`) & `tab-calibrate` (`auto-calib-val-yaw`) | IMU yaw comes from IMU fusion (`TYPE_ATTITUDE`), Odometry yaw comes from wheel differential integration, LiDAR yaw comes from ICP scan matching. |
| **Distance Calibration** | `tab-motion-cal` (Distance Calibration Card: 2-Meter test) | `tab-ros2` (Procedure Box 1: 1-Meter test instructions) | `tab-calibrate` (Closed-Loop Auto 1m test) | 2-meter physical trial calculation on `tab-motion-cal` vs 1-meter automated closed-loop test on `tab-calibrate` vs manual text procedure on `tab-ros2`. |
| **Turn / Rotation Calibration** | `tab-motion-cal` (Rotation Calibration Card: 360° test) | `tab-ros2` (Procedure Box 2: 90° turn test instructions) | `tab-calibrate` (Closed-Loop Auto 90° Turn Left/Right) | 360° manual spin test on `tab-motion-cal` vs 90° automated closed-loop turn test on `tab-calibrate` vs text instructions on `tab-ros2`. |
| **Effective Track Width** | `tab-motion-cal` (`cal-rot-current-track`) | `tab-ros2` (Active Kinematic Params: `track_width_m`) | `calibration_db.json` (`effectiveTrackWidth`) | Displayed as `0.382 m` on `tab-ros2` and dynamically read from `calibration_db.json` on `tab-motion-cal`. |
| **Wheel Diameter** | `tab-motion-cal` (`cal-dist-current-diameter`) | `tab-ros2` (Active Kinematic Params: `wheel_diameter_m`) | `tab-dashboard` (`config-form` input) & `calibration_db.json` | Displayed as `0.065 m` on `tab-ros2`, input field on `tab-dashboard`, and active value on `tab-motion-cal`. |
| **Ticks Per Revolution** | `tab-ros2` (Active Kinematic Params: `ticks_per_revolution`) | `tab-dashboard` (`config-form` encoder phase lines & gear ratio) | ROS 2 bringup node param (`1894.0`) | `tab-ros2` displays static `1894.0 (Calibrated)`, while `tab-dashboard` form has inputs for phase lines (11) and reduction ratio (30). |
| **LiDAR Health & Plot** | `tab-lidar` (360° Polar Display & System Status) | `tab-encoder` (LiDAR Straight-Line Correction Panel) | RPi5 `rplidar_sidecar.py` & `/scan` topic | Full 360° polar visualization on `tab-lidar` vs specialized 3ft path comparison plot on `tab-encoder`. |
| **ROS Topic Health** | `tab-ros2` (ROS 2 Topics & Rate Monitor) | `tab-dashboard` (Telemetry indicators) | `/diagnostics` topic stream | `tab-ros2` shows explicit Hz rates (`/scan` 6.5Hz, `/odom` 20Hz, `/tf` 20Hz). |
| **TF & Service Status** | `tab-ros2` (Service Badges & TF list) | `tab-autonomy` (Proposed) | Node logs | Currently lives only in `tab-ros2`. |
| **Firmware Identity** | Main Header (`ui-firmware-version`, `ui-firmware-build`) | `tab-calibrate` (`pi-protocol-display`) | Terminal Footer & `/api/firmware` | Shown in footer text, main header, and simulation threshold header. |
| **Serial Port Connection** | Main Header (`serial-status`, `com-port-input`, `btn-change-port`) | Terminal Footer (`[System]` log messages) | Server process stdout | COM port selector lives in header; status logged in footer. |
| **Motor Drive State** | `tab-dashboard` (Driving Control Room: `tele-drive-state`) | `tab-calibrate` (Maintenance HUD: `maint-test-armed`) | `tab-encoder` (`normal-drive-badge`) | Armed/Disarmed state shown in 3 separate panels. |
| **Arm / Disarm State** | `tab-dashboard` (`btn-arm-drive`, `btn-disarm-drive`) | Gamepad controller (Start / Select buttons) | Endpoints `/api/drive/arm`, `/api/drive/disarm` | Triggerable via UI buttons or gamepad buttons. |
| **Safety / Fault State** | Main Header & `tab-dashboard` (`btn-estop`) | `tab-calibrate` (`btn-stop-all-maint`, E-stop gates) | Firmware fault telemetry (`TYPE_FAULT_REPORT`) | E-stop buttons exist on Dashboard, Encoder, Maintenance, and Auto-Calib panels. |
| **Calibration History** | `tab-motion-cal` (Phase 5 Repeatability Table) | `calibration_db.json` (`testLogs` array) | `/home/ron/.../odom_calibration.json` | Repeatability test logs stored in `calibration_db.json` and rendered in `tab-motion-cal`. |
| **Repeatability Statistics** | `tab-motion-cal` (Phase 5 Verification Cards) | Backend `/api/calibration/repeatability/history` | `autotest_report.md` | Calculated on backend and rendered on `tab-motion-cal`. |
| **Raw Terminal Logs** | Global Footer (`footer.logs-panel`) | Browser Console (`console.log`) | `server.js` stdout | All WebSocket packets and raw serial inputs/outputs logged to footer. |
| **Gamepad Inputs** | `tab-dashboard` (Gamepad Live Controller Inputs HUD) | Header (`gamepad-status`) | Gamepad API event loop | Live axes and buttons rendered in `tab-dashboard`. |
| **Battery Voltage** | Main Header (`battery-container`, `battery-value`) | `tab-dashboard` (`TYPE_BATTERY` telemetry) | ESP32 ADC reading | Voltage meter permanently visible in header bar. |
| **Path Backtracking** | `tab-dashboard` (Path Breadcrumbs & Return Home Panel) | `tab-motion-cal` (Encoder Odometry & Path Backtracking Panel) | Endpoints `/api/path/record/*`, `/api/path/backtrack/*` | EXACT DUPLICATE: Complete backtracking button group appears on both `tab-dashboard` and `tab-motion-cal`. |

---

## 2. Legacy and Obsolete Feature Analysis

The following features represent legacy revisions, simulated tools, obsolete Yahboom protocol elements, or duplicated controls that must be evaluated for disposition:

### Feature 1: Breakaway Calibration (Simulation Only)
- **Location:** `tab-calibrate` (Top Panel: Breakaway Calibration & Safety Control)
- **Description:** Legacy Phase 2 simulated calibration state machine. Displays simulated PWM ramp-up, fake encoder speed delta, and fake threshold table (`pi-val-m1-fwd`..`m4-rev`).
- **Nature:** Backend-supported simulation (`/api/calibration/simulate/start`).
- **Motor Risk:** None (Physical motor output is physically disabled in simulation code).
- **Assessment:** Obsolete. Superseded by Phase 3 Maintenance Testing and Phase 5 Closed-Loop Auto Calibration.

### Feature 2: Old Readiness Checklists & Gates
- **Location:** `tab-calibrate` ("Pre-Calibration Safety Checklist" & "Real Calibration Readiness Gate")
- **Description:** Checkboxes (`pi-safety-chk-*` and `gate-chk-*`) that unlock simulation buttons.
- **Nature:** Frontend-only state gates.
- **Motor Risk:** None.
- **Assessment:** Cluttered and unnecessary. Modern arming safety is governed by the explicit Gamepad Deadman switch and global E-Stop.

### Feature 3: Duplicated Path Backtracking Panel
- **Location:** `tab-motion-cal` (Bottom Panel: Encoder Odometry & Path Backtracking Control)
- **Description:** Identical copy of `btn-record-start`, `btn-record-stop`, `btn-record-clear`, `btn-start-backtrack`, `btn-abort-backtrack` button group present on `tab-dashboard`.
- **Nature:** Frontend duplicate calling backend endpoints (`/api/path/record/*`, `/api/path/backtrack/*`).
- **Motor Risk:** Low (commands backtrack motion).
- **Assessment:** Pure redundancy. Should exist only in the primary **Drive** tab.

### Feature 4: Board Configurations & Tuning Form (Legacy Yahboom ASCII Form)
- **Location:** `tab-dashboard` (Right Column: Board Configurations & Tuning)
- **Description:** Form with inputs for Motor Type, Deadband Pulse, Encoder Phase Lines, Reduction Ratio, Wheel Diameter, and PID coefficients.
- **Nature:** Frontend form sending unsupported WebSocket messages (`config_*`, `read_flash`, `flash_reset`).
- **Motor Risk:** None (Firmware ignores these messages with a warning response).
- **Assessment:** Obsolete legacy code. Maker ESP32 Pro binary protocol v3.0 manages kinematic parameters in `calibration_db.json` and firmware NVS.

### Feature 5: Experimental LiDAR Straight-Line Correction & Orientation Wizard
- **Location:** `tab-encoder` (Bottom Panel: LiDAR Straight-Line Correction & Calibration)
- **Description:** Complex experimental panel containing an ICP straight-line correction mode, coordinate orientation wizard, track interference monitor, 3-tier speed canvas, and proposed trim calculator.
- **Nature:** Fully supported backend sidecar (`rplidar_sidecar.py`) and WebSocket handlers (`start_lidar_test`, `stop_lidar_test`).
- **Motor Risk:** Moderate (Can apply steering corrections while driving).
- **Assessment:** Advanced commissioning tool. Should be relocated to **Calibration -> Advanced Maintenance** or **Diagnostics**.

### Feature 6: Manual 2-Meter & 360° Calibration Cards
- **Location:** `tab-motion-cal` (Distance & Rotation Calibration Cards)
- **Description:** Manual input forms for Trial 1 & Trial 2 physical measurements to compute wheel diameter and track width.
- **Nature:** Fully integrated with `calibration_db.json` via WebSockets (`log_test_run`, `save_proposed_config`, `apply_calibration`).
- **Motor Risk:** Low (Runs 2m drive or 360° spin).
- **Assessment:** Core commissioning tool. Relocate to unified **Calibration** tab.

### Feature 7: "Phase-Number" Labels
- **Location:** Across all tabs (`Phase 3 Maintenance`, `Phase 4 Platform Drive`, `Phase 5 Repeatability`).
- **Description:** Descriptive text headings referencing internal project phase numbers.
- **Nature:** Frontend label text.
- **Motor Risk:** None.
- **Assessment:** User interface clutter. Replace with functional tab and section titles (Drive, Calibration, Diagnostics).

---

## 3. Legacy Candidate Disposition & Safety Matrix

| Feature / Candidate | Frontend / Backend Status | Used by Automated Tests? | Motor / Safety Risk | Proposed Disposition | Justification |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Breakaway Simulation** | Backend-supported (`/api/calibration/simulate/start`) | Checked in `test_dom_contract.py` | None (Motors disabled) | **ARCHIVE** | Superseded by real closed-loop calibration. Preserve code for reference, remove from main UI. |
| **Simulated Readiness Gates** | Frontend-only (`app.js`) | Checked in `test_dom_contract.py` | None | **REMOVE-LATER** | Unnecessary manual checkboxes; replace with streamlined safety banner. |
| **Duplicate Backtracking Panel** | Frontend duplicate | Checked in `test_dom_contract.py` | Low (Commands backtrack) | **CONSOLIDATE** | Remove duplicate panel from `tab-motion-cal`; retain single canonical instance in **Drive**. |
| **Yahboom Config Form** | WebSocket messaging (Unsupported) | Checked in `test_dom_contract.py` | None (Ignored by firmware) | **ARCHIVE** | Board configuration is handled via ROS parameters and `calibration_db.json`. |
| **Phase 3 Raised-Wheel Maintenance** | Backend API (`/api/maintenance/*`) & Firmware C++ | Enforced in `test_maintenance_safety.py` | Low (Single motor, 2s auto-stop) | **ADVANCED** | Move behind collapsed "Advanced Maintenance" section under **Calibration**. |
| **LiDAR Straight-Line Correction** | Backend sidecar (`rplidar_sidecar.py`) | Checked in `test_dom_contract.py` | Moderate (Applies steering effort) | **ADVANCED** | Relocate to **Calibration -> Advanced Commissioning**. |
| **Phase 5 Repeatability Verification** | Backend API (`/api/calibration/repeatability/*`) | Verified in `test_repeatability_sim.js` | Moderate (Runs 5-trial series) | **MOVE** | Core commissioning tool; relocate to primary **Calibration** tab. |
| **Closed-Loop Auto Calibration (1m / 90°)** | Backend API (`/api/calibration/auto/*`) | Verified in `test_auto_calibration.py` & `test_button_binding.py` | High (Autonomous movement) | **MOVE** | Core calibration tool; relocate to primary **Calibration** tab with confirmation modal intact. |
| **Raw Terminal & Serial Prompt** | WebSocket (`raw_command`) | Checked in `test_dom_contract.py` | High (Raw serial packet write) | **MOVE** | Developer tool; relocate to **Diagnostics** tab. |
| **Phase-Number Labels** | Frontend text labels | Checked in HTML contract tests | None | **REMOVE-LATER** | Clean up UI wording to remove legacy phase numbers. |
