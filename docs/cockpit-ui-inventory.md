# Rover Cockpit UI/UX Inventory

**Repository:** `C:\Users\Ron\electronic_projects\yahboom-encoder`  
**Branch:** `feature/ros2-encoder-odom-tf`  
**Date:** July 26, 2026  
**Stage:** Stage 1 — Inventory & Analysis  

---

## 1. Top-Level Tab Navigation Overview

The legacy cockpit interface contains 7 top-level tabs:

| Tab ID | Tab Button Label | Primary Function |
| :--- | :--- | :--- |
| `tab-dashboard` | 🎮 Cockpit Dashboard | Driving control room, camera feed, wheel telemetry cards, board tuning form |
| `tab-imu` | 🧭 IMU & Position Tracking | 3D attitude visualizer, raw IMU readings, 2D odometer canvas plot |
| `tab-encoder` | 🔍 Encoder & Rotate Testing | Individual motor pulse tests, revolution stops, straight drive lock & LiDAR correction |
| `tab-ros2` | 🤖 ROS 2 & Odometry Testing | ROS 2 Jazzy stack health, topic rate monitors, active kinematic params, manual procedures |
| `tab-calibrate` | 🛡️ Calibration & Safety | Breakaway simulation, raised-wheel maintenance tests, closed-loop auto calib tests |
| `tab-motion-cal` | 🎛️ Motion Cal & Backtrack | 2m distance calib, 360° rotation calib, Phase 5 repeatability, backtrack control |
| `tab-lidar` | 📡 LiDAR Monitor | RPLIDAR C1 connection status, 360° polar canvas display, live sample table |

*Note: In addition to top-level tabs, there is a global header (`header.main-header`), a global footer (`footer.logs-panel`), and two global overlay modals (`#autotest-modal`, `#modal-auto-calib-confirm`).*

---

## 2. Complete Panel-by-Panel Inventory

### Global Elements (Header, Footer, and Modals)

#### Panel G-1: Main Header & Connection Status
- **Visible Title:** MAKER ESP32 PRO COCKPIT (Main Header)
- **HTML Container ID:** `header.main-header`
- **Parent Container:** `div.container` (Global Header)
- **Purpose:** System clock, WebSocket status, Serial COM port selector, Gamepad status, Battery voltage meter.
- **Buttons & Controls:**
  - `time-status` (Time readout)
  - `ws-status` (WebSocket connection indicator)
  - `serial-status` (Serial port status indicator)
  - `gamepad-status` (Gamepad status indicator)
  - `com-port-input` (Text input for serial COM port, default `COM18`)
  - `btn-change-port` (Button to switch/connect serial port)
  - `battery-container`, `battery-fill`, `battery-value` (Battery voltage visual bar and text)
- **Frontend Handlers:** `btnChangePort.onclick` -> `changePort()`
- **Backend Endpoints Used:** WebSocket message `change_port`
- **WebSocket Messages Consumed:** `status`, `battery`, `cockpit_info`
- **Data Sources:** ESP32 binary telemetry (`TYPE_BATTERY`), Node.js serial port manager
- **Can Command Motors:** No
- **Can Arm/Disarm:** No
- **Can Change Calibration:** No
- **Read-Only:** No (allows changing COM port)
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** WebSockets, `server.js` serial port instance

#### Panel G-2: Live Driver Logs & Command Terminal (Global Footer)
- **Visible Title:** 💻 Live Driver Logs & Command Terminal
- **HTML Container ID:** `footer.logs-panel`
- **Parent Container:** `div.container` (Global Footer)
- **Purpose:** Displays live driver logs, raw serial packet traffic, and provides a raw hex command prompt.
- **Buttons & Controls:**
  - `btn-clear-logs` (Button to clear console text)
  - `terminal-console` (Terminal scroll div)
  - `terminal-command-input` (Text input for raw serial hex string)
  - `btn-send-raw-command` (Button to send raw hex string)
  - Version readouts: `ui-cockpit-deployed`, `ui-firmware-version`, `ui-firmware-build`
- **Frontend Handlers:** `btnClearLogs.onclick`, `btnSendRawCommand.onclick`, `terminalCommandInput.onkeydown`
- **Backend Endpoints Used:** WebSocket message `raw_command`
- **WebSocket Messages Consumed:** `raw_serial_in`, `raw_serial_out`, `raw_serial_out_err`, `message`
- **Data Sources:** Node.js serial packet logger, firmware identification telemetry (`TYPE_FIRMWARE_INFO`)
- **Can Command Motors:** YES (via raw hex serial packets)
- **Can Arm/Disarm:** YES (via raw hex serial packets)
- **Can Change Calibration:** YES (via raw hex serial packets)
- **Read-Only:** No
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** WebSocket connection

#### Panel G-3: Auto Test Track Visualizer Modal
- **Visible Title:** Auto Test Live Tracker
- **HTML Container ID:** `autotest-modal`
- **Parent Container:** `body` (Fixed overlay modal)
- **Purpose:** Displays cyberpunk animated visualizer of 3ft straight-drive auto test, top-down rover sprite, live progress metrics, and stage results table.
- **Buttons & Controls:**
  - `autotest-modal-step` (Step/Cycle badge)
  - `autotest-rover-sprite` (Rover top-down graphic)
  - Readouts: `autotest-metrics-dist`, `autotest-metrics-mismatch`, `autotest-metrics-drift`, `autotest-metrics-speeds`
  - `autotest-stage-results-list` (Results list container)
  - `autotest-status-text` (Status text)
  - Buttons: `btn-autotest-modal-copy`, `btn-autotest-modal-abort`, `btn-autotest-modal-close`
- **Frontend Handlers:** `updateAutoTestModal()`, modal button listeners
- **Backend Endpoints Used:** `/api/autotest/start`, `/api/autotest/abort`
- **WebSocket Messages Consumed:** `autotest_status`
- **Data Sources:** `server.js` auto-test runner
- **Can Command Motors:** YES (via start/abort test endpoints)
- **Can Arm/Disarm:** No
- **Can Change Calibration:** No
- **Read-Only:** No
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** `autotest_status` WebSocket events

#### Panel G-4: Auto Calibration Confirmation Modal
- **Visible Title:** ⚠️ Confirm Auto Calibration
- **HTML Container ID:** `modal-auto-calib-confirm`
- **Parent Container:** `body` (Fixed overlay modal)
- **Purpose:** Prompts safety confirmation before launching physical closed-loop auto-calibration tests.
- **Buttons & Controls:**
  - `confirmAndStartAutoCalib()` button ("Yes, Start Test")
  - `closeAutoCalibModal()` button ("Cancel")
- **Frontend Handlers:** `confirmAndStartAutoCalib()`, `closeAutoCalibModal()`
- **Backend Endpoints Used:** `/api/calibration/auto/start`
- **WebSocket Messages Consumed:** None
- **Data Sources:** User explicit click
- **Can Command Motors:** YES (initiates physical motion test)
- **Can Arm/Disarm:** No
- **Can Change Calibration:** No
- **Read-Only:** No
- **Tests Covering It:** `test_button_binding.py`, `test_dom_contract.py`
- **Dependencies:** `pendingAutoCalibTest` variable in `app.js`

---

### Tab 1: `tab-dashboard` (🎮 Cockpit Dashboard)

#### Panel 1-1: Driving Control Room
- **Visible Title:** 🕹️ Driving Control Room
- **HTML Container ID:** `section.panel.controls-panel.glass`
- **Parent Container:** `div#tab-dashboard > main.dashboard-grid`
- **Purpose:** Manual driving interface with joystick mapping cheat-sheet, live gamepad state HUD, arm/disarm buttons, normal drive telemetry HUD, wheel odometry HUD, direction d-pad, speed sliders, and emergency stop.
- **Buttons & Controls:**
  - `normal-drive-badge` (Disarmed/Armed state indicator)
  - `limits-floor-testing` (Checkbox: max 0.17 m/s floor speed limit)
  - Gamepad HUD: `gp-live-arm`, `gp-live-deadman`, `gp-live-linear`, `gp-live-angular`, `gp-live-stop`, `gp-live-estop`, `gp-live-buttons`
  - Arming buttons: `btn-arm-drive`, `btn-disarm-drive`
  - Drive Telemetry HUD: `tele-drive-state`, `tele-drive-mode`, `tele-drive-phys-lock`, `tele-drive-source`, `tele-drive-age`, `tele-drive-req-lin`, `tele-drive-req-ang`, `tele-drive-lim-lin`, `tele-drive-lim-ang`
  - Skid-Steer Odometry HUD: `odom-x-real`, `odom-y-real`, `odom-yaw-real`, `odom-left-dist`, `odom-right-dist`, `odom-v-real`, `odom-w-real`, `odom-enc-m1`..`m4`
  - Path Backtracking controls: `btn-record-start`, `btn-record-stop`, `btn-record-clear`, `btn-start-backtrack`, `btn-abort-backtrack`, `path-recording-lbl`, `path-breadcrumbs-lbl`, `backtrack-state-lbl`, `backtrack-progress-bar`
  - Direction D-Pad: `ctrl-forward`, `ctrl-left`, `ctrl-stop-center`, `ctrl-right`, `ctrl-reverse`, `ctrl-spin-left`, `ctrl-spin-right`
  - Sliders: `sync-speed-slider`, `sync-speed-readout`, speed presets (`data-val="-500"`, `0`, `500`, `1000`), individual sliders `speed-m1`..`m4`, readouts `readout-m1`..`m4`
  - E-Stop & Diagnostic: `btn-estop`, `btn-motor-proof`, `encoder-activity`
- **Frontend Handlers:** `armNormalDrive()`, `disarmNormalDrive()`, `driveRover()`, `triggerEstop()`, slider listeners, backtrack button listeners
- **Backend Endpoints Used:** `/api/drive/arm`, `/api/drive/disarm`, `/api/drive/limits`, `/api/stop`, `/api/path/record/*`, `/api/path/backtrack/*`
- **WebSocket Messages Consumed:** `normal_drive_status`, `odom`, `battery`, `backtrack_status`, `path_status`
- **Data Sources:** ESP32 normal drive telemetry, dead-reckoning odometry node
- **Can Command Motors:** YES
- **Can Arm/Disarm:** YES
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_maintenance_safety.py`, `test_dom_contract.py`
- **Dependencies:** Gamepad API, serial connection, normal drive state machine in firmware

#### Panel 1-2: RPi Camera Feed
- **Visible Title:** 📷 RPi Camera Feed
- **HTML Container ID:** `section.panel.camera-panel.glass`
- **Parent Container:** `div#tab-dashboard > main.dashboard-grid`
- **Purpose:** Displays live video stream from Raspberry Pi camera sidecar.
- **Buttons & Controls:**
  - `camera-status-badge`, `camera-status-dot`, `camera-status-text`
  - `camera-viewport`, `camera-stream` (`<img>`), `camera-placeholder`
  - `btn-toggle-camera` ("Start Feed" / "Stop Feed")
  - `btn-fullscreen-camera` ("Fullscreen")
- **Frontend Handlers:** `btnToggleCamera.onclick` -> `toggleCameraFeed()`, `btnFullscreenCamera.onclick`
- **Backend Endpoints Used:** `/api/camera`, `/api/camera/status`
- **WebSocket Messages Consumed:** `camera_status`
- **Data Sources:** HTTP MJPEG stream on Raspberry Pi
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** RPi camera HTTP endpoint

#### Panel 1-3: Real-time Wheel Telemetry Grid
- **Visible Title:** 📊 Real-time Wheel Telemetry
- **HTML Container ID:** `div.panel.telemetry-panel.glass`
- **Parent Container:** `div#tab-dashboard > main.dashboard-grid > section.telemetry-section`
- **Purpose:** Cards for Front Left (M1), Front Right (M3), Rear Left (M2), Rear Right (M4) displaying speed (MPH), RPM, total ticks, and rotating wheel graphics.
- **Buttons & Controls:**
  - Stream toggles: `stream-total`, `stream-realtime`, `stream-speed`
  - Wheel cards: `card-m1`, `card-m3`, `card-m2`, `card-m4`
  - Readouts: `telemetry-speed-m1`..`m4`, `telemetry-real-m1`..`m4`, `telemetry-total-m1`..`m4`
- **Frontend Handlers:** Telemetry stream checkbox listeners, WebSocket message handler
- **Backend Endpoints Used:** None (WebSocket stream)
- **WebSocket Messages Consumed:** `encoder_total`, `encoder_realtime`, `motor_speeds`
- **Data Sources:** ESP32 encoder telemetry packets (`TYPE_ENCODER`, `TYPE_BATTERY`)
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** ESP32 encoder telemetry

#### Panel 1-4: Board Configurations & Tuning (Legacy Yahboom Form)
- **Visible Title:** ⚙️ Board Configurations & Tuning
- **HTML Container ID:** `div.panel.config-panel.glass`
- **Parent Container:** `div#tab-dashboard > main.dashboard-grid > section.telemetry-section`
- **Purpose:** Form intended for tuning Yahboom expansion board flash parameters (Motor Type, Deadband, Phase Lines, Reduction Ratio, Wheel Diameter, PID).
- **Buttons & Controls:**
  - `config-form`
  - Inputs: `motor-type`, `deadband`, `phase-lines`, `reduction-ratio`, `wheel-diameter`, `pid-p`, `pid-i`, `pid-d`
  - Buttons: `btn-read-flash`, `btn-reset-flash`, Submit button ("Apply Settings to Board")
- **Frontend Handlers:** `configForm.onsubmit` -> `sendUploadConfig()`, `btnReadFlash.onclick`, `btnResetFlash.onclick`
- **Backend Endpoints Used:** WebSocket messages `config_*`, `read_flash`, `flash_reset`
- **WebSocket Messages Consumed:** `message` (returns warning: "Command is not supported on ROS Expansion Board V3.0 binary protocol")
- **Data Sources:** Static form inputs
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO (Commands ignored by firmware)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Legacy serial protocol (OBSOLETE on Maker ESP32 Pro binary protocol)

---

### Tab 2: `tab-imu` (🧭 IMU & Position Tracking)

#### Panel 2-1: 3D Rover Attitude Visualizer
- **Visible Title:** 🧭 3D Rover Attitude
- **HTML Container ID:** `section.panel.orientation-panel.glass`
- **Parent Container:** `div#tab-imu > main.imu-grid`
- **Purpose:** 3D CSS model representing rover chassis orientation in real-time.
- **Buttons & Controls:**
  - `btn-reset-imu` ("Reset Orientation")
  - `rover-3d-model` (3D CSS box with 6 faces and 4 wheel models)
  - Readouts: `imu-roll`, `imu-pitch`, `imu-yaw`
- **Frontend Handlers:** `btnResetIMU.onclick` -> `resetIMUOrientation()`, WebSocket handler
- **Backend Endpoints Used:** None (WebSocket stream)
- **WebSocket Messages Consumed:** `attitude`, `imu`
- **Data Sources:** ESP32 IMU telemetry packet (`TYPE_ATTITUDE`)
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES (except local zeroing of visual offset)
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** ESP32 IMU sensor

#### Panel 2-2: IMU Sensor Readings
- **Visible Title:** 📈 IMU Sensor Readings
- **HTML Container ID:** `section.panel.sensors-panel.glass`
- **Parent Container:** `div#tab-imu > main.imu-grid`
- **Purpose:** Readouts for 3-axis Accelerometer (X, Y, Z in g) and 3-axis Gyroscope (X, Y, Z in °/s).
- **Buttons & Controls:**
  - Readouts: `imu-ax`, `imu-ay`, `imu-az`, `imu-gx`, `imu-gy`, `imu-gz`
- **Frontend Handlers:** WebSocket IMU message handler
- **Backend Endpoints Used:** None
- **WebSocket Messages Consumed:** `imu`
- **Data Sources:** ESP32 IMU telemetry packet (`TYPE_IMU`)
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** ESP32 IMU sensor

#### Panel 2-3: Rover Position Tracker
- **Visible Title:** 🗺️ Rover Position Tracker
- **HTML Container ID:** `section.panel.map-panel.glass`
- **Parent Container:** `div#tab-imu > main.imu-grid`
- **Purpose:** 2D Canvas plotting rover position history (X, Y) and speed.
- **Buttons & Controls:**
  - `btn-reset-odometry` ("Reset Odometer")
  - Readouts: `odom-x`, `odom-y`, `odom-speed`
  - `path-canvas` (<canvas> element)
- **Frontend Handlers:** `btnResetOdometry.onclick` -> `resetOdometry()`, `renderCanvasPath()`
- **Backend Endpoints Used:** `/api/encoders`
- **WebSocket Messages Consumed:** `odom`
- **Data Sources:** Dead-reckoning odometry calculations
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES (except canvas trajectory clear)
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Wheel encoder odometry stream

---

### Tab 3: `tab-encoder` (🔍 Encoder & Rotate Testing)

#### Panel 3-1: Encoder Diagnostics & Calibration
- **Visible Title:** 🔍 Encoder Diagnostics & Calibration
- **HTML Container ID:** `section.panel.encoder-panel.glass`
- **Parent Container:** `div#tab-encoder > main.encoder-grid`
- **Purpose:** Individual manual motor pulse buttons (FWD/REV) for M1..M4 to verify encoder direction.
- **Buttons & Controls:**
  - `btn-reset-encoders-ui` ("Zero Counts")
  - Encoder cards for LF (M1), RF (M2), LR (M3), RR (M4)
  - Readouts: `test-ticks-m1`..`m4`, `test-pwm-m1`..`m4`, `test-rpm-m1`..`m4`
  - Controls: Buttons `.btn-test.btn-fwd`, `.btn-test.btn-rev` with `data-motor="1"`..`"4"`
- **Frontend Handlers:** `mousedown`/`touchstart` -> `startMotor()`, `mouseup`/`mouseleave`/`touchend` -> `stopMotor()`, `btnResetEncodersUI.onclick`
- **Backend Endpoints Used:** WebSocket `set_pwm` / `set_speed`
- **WebSocket Messages Consumed:** `encoder_total`, `encoder_realtime`
- **Data Sources:** ESP32 encoder telemetry
- **Can Command Motors:** YES (Direct motor pulse)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Serial connection to ESP32

#### Panel 3-2: Rotation Control & Stops
- **Visible Title:** 🔄 Rotation Control & Stops
- **HTML Container ID:** `section.panel.rotate-panel.glass`
- **Parent Container:** `div#tab-encoder > main.encoder-grid`
- **Purpose:** Commands fixed wheel revolutions for individual wheels or all wheels simultaneously.
- **Buttons & Controls:**
  - `test-num-turns` (Number input for target turns, default `1.0`)
  - Wheel buttons: `btn-turn` with `data-wheel="m1"`..`"m4"`
  - `btn-turn-all` ("Rotate All Wheels")
  - `btn-estop-rotate` ("⚠️ ESTOP POSITION MODE")
- **Frontend Handlers:** `btnTurn.onclick` -> `triggerTurnWheel()`, `btnTurnAll.onclick` -> `triggerTurnAll()`, `btnEstopRotate.onclick` -> `triggerEstop()`
- **Backend Endpoints Used:** `/api/turn`, `/api/stop`
- **WebSocket Messages Consumed:** `status`
- **Data Sources:** Server position control loop
- **Can Command Motors:** YES
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Server-side turn controller

#### Panel 3-3: Straight Drive Rotation Monitor & Steering Lock
- **Visible Title:** ⚖️ Straight Drive Rotation Monitor & Steering Lock
- **HTML Container ID:** `section.panel.straight-drive-panel.glass`
- **Parent Container:** `div#tab-encoder > main.encoder-grid`
- **Purpose:** Steering lock toggle, manual trim adjustment inputs, rotational symmetry balance bar, and 3ft auto test launcher.
- **Buttons & Controls:**
  - `straight-lock-badge` (Steering Lock status badge)
  - `btn-reset-straight-test` ("Zero Test Encoders")
  - `btn-copy-test-data` ("Copy Last Test Data")
  - `btn-auto-test` ("Auto Test (3ft)")
  - `straight-drive-lock-toggle` (Switch checkbox)
  - Trim inputs: `label-active-trims`, `input-left-trim`, `input-right-trim`, `btn-save-trims`
  - Balance indicator: `straight-symmetry-status`, `straight-balance-bar`, `straight-balance-cursor`
  - Readouts: `straight-avg-left`, `straight-avg-right`, `straight-mismatch-delta`
- **Frontend Handlers:** `straightToggle.onchange` -> `toggleStraightLock()`, `btnSaveTrims.onclick` -> `saveTrims()`, `btnAutoTest.onclick` -> `runAutoTestTrack()`
- **Backend Endpoints Used:** `/api/autotest/start`, `/api/autotest/abort`
- **WebSocket Messages Consumed:** `autotest_status`, `rover_trims_sync`
- **Data Sources:** Server auto-test manager, active trim database
- **Can Command Motors:** YES (via Auto Test 3ft)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** YES (Saves left/right motor trims)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** `autotest_status` WebSocket broadcasts, active motor trims in `calibration_db.json`

#### Panel 3-4: LiDAR Straight-Line Correction & Calibration
- **Visible Title:** 📡 LiDAR Straight-Line Correction & Calibration
- **HTML Container ID:** `section.panel.glass` (nested in `tab-encoder`)
- **Parent Container:** `div#tab-encoder > main.encoder-grid`
- **Purpose:** Experimental LiDAR ICP straight-line correction, orientation wizard, track interference monitor, 3-tier speed path canvas plot, and proposed motor trim manager.
- **Buttons & Controls:**
  - Status badge: `lidar-test-state-badge`
  - Gates: `chk-rigid-mount`, `chk-level-mount`
  - Orientation wizard: `orientation-verified-badge`, `btn-start-wizard`, `wizard-step-text`, `wizard-live-range`, `btn-wizard-yes`, `btn-wizard-cancel`
  - Track interference monitor: `interference-warning-badge`, `monitored-track-width`, `box-interfere-front/left/right`, `val-interfere-front/left/right`
  - Radio modes: `lidar-test-mode` (`observe`, `correct`, `learn`)
  - Action buttons: `btn-start-lidar-test`, `btn-stop-lidar-test`, low-end calibration button
  - Live stats: `stat-lidar-x`, `stat-lidar-y`, `stat-lidar-yaw`, `stat-lidar-conf`
  - Path plot: `chk-toggle-slow`, `chk-toggle-med`, `chk-toggle-fast`, `lidar-path-canvas`
  - Proposed trims: `pass-count-label`, `active-fwd-trims`, `proposed-fwd-trims`, `active-rev-trims`, `proposed-rev-trims`, `btn-apply-proposed`, `btn-rollback-proposed`, `btn-reset-trims`
  - Motor power output HUD: `power-active-tier`, `lbl-power-left/right`, `bar-power-left/right`, `lbl-control-effort`, `bar-control-effort`
- **Frontend Handlers:** `btnStartLidarTest.onclick`, `btnStopLidarTest.onclick`, `btnApplyProposed.onclick`, `btnRollbackProposed.onclick`, `btnResetTrims.onclick`
- **Backend Endpoints Used:** `/api/lidar/test/start`, `/api/lidar/test/stop`
- **WebSocket Messages Consumed:** `lidar_test_status`, `lidar_test_telemetry`, `lidar_test_results`
- **Data Sources:** Node.js LiDAR ICP bridge on RPi5
- **Can Command Motors:** YES (Applies active steering corrections)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** YES (Saves proposed FWD/REV trims to NVS)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** RPLIDAR C1 sidecar python process

---

### Tab 4: `tab-ros2` (🤖 ROS 2 & Odometry Testing)

#### Panel 4-1: ROS 2 Service Health & Odometry Validation
- **Visible Title:** 🤖 ROS 2 Service Health & Odometry Validation
- **HTML Container ID:** `div.tab-content#tab-ros2 > main > section.panel.glass`
- **Parent Container:** `div#tab-ros2 > main`
- **Purpose:** Read-only system dashboard displaying ROS 2 Jazzy stack health, topic rate monitors (`/diagnostics`, `/scan`, `/odom`, `/tf`), active kinematic parameters (`wheel_diameter_m`, `ticks_per_revolution`, `track_width_m`), and physical calibration procedures.
- **Buttons & Controls:** Read-only badges and code blocks (No interactive controls)
- **Frontend Handlers:** None (Static markup / WebSocket status readouts)
- **Backend Endpoints Used:** None
- **WebSocket Messages Consumed:** `ros2_health`, `status`
- **Data Sources:** ROS 2 Jazzy node diagnostics on RPi5
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_maintenance_safety.py` (`test_read_only_ros2_status_tab`), `test_dom_contract.py`
- **Dependencies:** ROS 2 bringup nodes on Raspberry Pi

---

### Tab 5: `tab-calibrate` (🛡️ Calibration & Safety)

#### Panel 5-1: Breakaway Calibration & Safety Control (Legacy Simulation)
- **Visible Title:** 🛡️ Breakaway Calibration & Safety Control
- **HTML Container ID:** `section.panel.glass` (Top section of `tab-calibrate`)
- **Parent Container:** `div#tab-calibrate > main.dashboard-grid`
- **Purpose:** Legacy simulated calibration interface with pre-calibration checklist, simulation start/cancel buttons, readiness gate checklist, progress HUD, and simulated threshold table.
- **Buttons & Controls:**
  - Badges: `pi-lock-badge`, `pi-mode-badge`
  - Checklists: `pi-safety-chk-elevated/clear/estop/auto`, `gate-chk-motor-dir/enc-dir/maint-stop/estop/deadman`
  - Action buttons: `btn-start-calibration`, `btn-cancel-calibration`, `btn-start-real-calibration`
  - Progress HUD: `pi-cal-panel`, `pi-session-display`, `pi-cal-state`, `pi-cal-active-motor-lbl`, `pi-cal-direction-lbl`, `pi-cal-pwm-lbl`, `pi-cal-delta-lbl`, `pi-cal-movement-lbl`, `pi-cal-pwm-val`, `pi-cal-progress`
  - Simulated thresholds table: `pi-val-m1-fwd`..`m4-rev`, `pi-protocol-display`
- **Frontend Handlers:** `btnStartCalibration.onclick` -> `triggerCalibrateStart()`, `btnCancelCalibration.onclick` -> `triggerCalibrateCancel()`, `btnStartRealCalibration.onclick` -> `triggerRealCalibrateStart()`
- **Backend Endpoints Used:** `/api/calibration/simulate/start`, `/api/calibration/real/start`, `/api/calibration/abort`, `/api/calibration/status`, `/api/calibration/verify_ready`
- **WebSocket Messages Consumed:** `calibration_status`
- **Data Sources:** Server calibration state machine simulation
- **Can Command Motors:** NO (Simulation explicitly disables motor output)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO (Simulated thresholds are not written to NVS)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Legacy Phase 2 state machine in `server.js`

#### Panel 5-2: Phase 3 — Individual-Wheel Maintenance Testing
- **Visible Title:** 🛠️ Phase 3 — Individual-Wheel Maintenance Testing
- **HTML Container ID:** `section.panel.glass` (Middle section of `tab-calibrate`)
- **Parent Container:** `div#tab-calibrate > main.dashboard-grid`
- **Purpose:** Raised-wheel safety verification and individual motor testing (2.0s auto-stop), isolation check, and 4-wheel encoder snapshot table.
- **Buttons & Controls:**
  - `maint-status-badge`, `maint-safety-chk`
  - Motor test buttons: `btn-m1-fwd`..`btn-m4-rev` (8 buttons for M1..M4 FWD/REV)
  - `btn-stop-all-maint` ("🛑 STOP ALL")
  - Result card: `maint-test-result-card`, `btn-clear-test-result`, `maint-test-error-banner`, `maint-test-error-msg`, `maint-test-motor`, `maint-test-label`, `maint-test-cmd`, `maint-test-time`, `maint-test-autostop`, `maint-test-armed`, `maint-test-delta-enc`, `maint-test-steady`, `maint-test-isolation`
  - Snapshot table: `enc-start-m1`..`m4`, `enc-end-m1`..`m4`, `enc-delta-m1`..`m4`, `enc-iso-m1`..`m4`
- **Frontend Handlers:** `btnM1Fwd.onclick` -> `runSingleMotorTest()`, `btnStopAllMaint.onclick` -> `stopAllMaintenance()`, `btnClearTestResult.onclick` -> `clearTestResult()`
- **Backend Endpoints Used:** `/api/maintenance/enter`, `/api/maintenance/set_output`, `/api/maintenance/exit`, `/api/maintenance/run_test`, `/api/maintenance/status`
- **WebSocket Messages Consumed:** `maintenance_status`, `maintenance_test_complete`
- **Data Sources:** Server maintenance manager and ESP32 binary protocol
- **Can Command Motors:** YES (Requires raised-wheel safety acknowledgment)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_maintenance_safety.py` (Full file), `test_dom_contract.py`
- **Dependencies:** ESP32 `MaintenanceManager` firmware class

#### Panel 5-3: Experimental Closed-Loop Calibration Tests
- **Visible Title:** 🎯 Experimental Closed-Loop Calibration Tests
- **HTML Container ID:** `section.panel.glass` (Bottom section of `tab-calibrate`)
- **Parent Container:** `div#tab-calibrate > main.dashboard-grid`
- **Purpose:** Triggers closed-loop automatic floor tests (1m forward, 90° left turn, 90° right turn) with safety HUD and result card.
- **Buttons & Controls:**
  - Warnings: `auto-calib-warning-box`, `auto-calib-active-banner`
  - Test buttons: `btn-auto-fwd-1m`, `btn-auto-turn-left`, `btn-auto-turn-right`, `btn-auto-abort`
  - HUD: `auto-calib-hud`, `auto-calib-hud-phase`, `auto-calib-val-test`, `auto-calib-val-elapsed`, `auto-calib-val-dist`, `auto-calib-val-yaw`, `auto-calib-val-ages`, `auto-calib-val-cmd`, `auto-calib-val-reason`
  - Result card: `auto-calib-result-card`, `auto-calib-result-badge`, `res-test-type`, `res-start-pose`, `res-ending-pose`, `res-measured-dist`, `res-measured-yaw`, `res-target-errors`, `res-elapsed-time`, `res-stop-reason`, `btn-clear-calib-result`
- **Frontend Handlers:** `btnAutoFwd1m.onclick` -> `promptAutoCalib()`, `btnAutoAbort.onclick` -> `abortAutoCalibrationTest()`
- **Backend Endpoints Used:** `/api/calibration/auto/start`, `/api/calibration/auto/abort`, `/api/calibration/auto/clear_result`, `/api/calibration/auto/status`
- **WebSocket Messages Consumed:** `auto_calib_status`
- **Data Sources:** Closed-loop controller in `server.js`
- **Can Command Motors:** YES (Autonomous closed-loop movement)
- **Can Arm/Disarm:** YES (Auto-disarms upon completion/abort)
- **Can Change Calibration:** NO (Generates test metrics for calibration calculation)
- **Read-Only:** NO
- **Tests Covering It:** `test_auto_calibration.py`, `test_button_binding.py`, `test_dom_contract.py`
- **Dependencies:** ROS odometry stream, ESP32 binary motor control

---

### Tab 6: `tab-motion-cal` (🎛️ Motion Cal & Backtrack)

#### Panel 6-1: Distance Calibration Card
- **Visible Title:** 📏 Distance Calibration (2-Meter)
- **HTML Container ID:** `section.panel.glass` (Top-left section of `tab-motion-cal`)
- **Parent Container:** `div#tab-motion-cal > main.dashboard-grid`
- **Purpose:** Interactive 2-meter drive test and 2-trial actual physical measurement inputs to calculate average proposed wheel diameter.
- **Buttons & Controls:**
  - Status: `cal-dist-status-badge`, `cal-dist-current-diameter`
  - Button: `btn-cal-run-distance` ("🚗 Run 2-Meter Distance Test")
  - Inputs: `cal-dist-trial1`, `cal-dist-trial2`
  - Readouts: `cal-dist-prop1`, `cal-dist-prop2`, `cal-dist-avg`, `cal-dist-diff`, `cal-dist-warning`
  - Action buttons: `btn-cal-dist-apply`, `btn-cal-dist-reject`, `btn-cal-dist-restore`
- **Frontend Handlers:** `btnCalRunDistance.onclick` -> `startDistanceTest()`, `btnCalDistApply.onclick` -> `applyWheelCalibration()`, `btnCalDistReject.onclick` -> `clearDistanceTrials()`, `btnCalDistRestore.onclick` -> `restorePreviousCalibration()`
- **Backend Endpoints Used:** WebSocket `log_test_run`, `save_proposed_config`, `apply_calibration`, `restore_previous`
- **WebSocket Messages Consumed:** `calibration_db`
- **Data Sources:** `calibration_db.json`
- **Can Command Motors:** YES (Runs 2m drive test)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** YES (Updates `wheelDiameter` in `calibration_db.json`)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** `calibration_db.json`

#### Panel 6-2: Rotation Calibration Card
- **Visible Title:** 🔄 Rotation Calibration (360°)
- **HTML Container ID:** `section.panel.glass` (Top-right section of `tab-motion-cal`)
- **Parent Container:** `div#tab-motion-cal > main.dashboard-grid`
- **Purpose:** Interactive 360° CW/CCW rotation tests and trial measurement inputs to calibrate effective track width.
- **Buttons & Controls:**
  - Status: `cal-rot-status-badge`, `cal-rot-current-track`
  - Buttons: `btn-cal-run-rot-cw`, `btn-cal-run-rot-ccw`
  - Inputs: `cal-rot-trial-cw`, `cal-rot-trial-ccw`
  - Readouts: `cal-rot-prop-cw`, `cal-rot-prop-ccw`, `cal-rot-avg`, `cal-rot-diff`, `cal-rot-warning`
  - Action buttons: `btn-cal-rot-apply`, `btn-cal-rot-reject`, `btn-cal-rot-restore`
- **Frontend Handlers:** `btnCalRunRotCw.onclick`, `btnCalRunRotCcw.onclick`, `btnCalRotApply.onclick` -> `applyTrackCalibration()`, `btnCalRotReject.onclick` -> `clearRotationTrials()`, `btnCalRotRestore.onclick` -> `restorePreviousCalibration()`
- **Backend Endpoints Used:** WebSocket `log_test_run`, `save_proposed_config`, `apply_calibration`, `restore_previous`
- **WebSocket Messages Consumed:** `calibration_db`
- **Data Sources:** `calibration_db.json`
- **Can Command Motors:** YES (Runs 360° rotation)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** YES (Updates `effectiveTrackWidth` in `calibration_db.json`)
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** `calibration_db.json`

#### Panel 6-3: Phase 5 — Software Repeatability Verification
- **Visible Title:** 📊 Phase 5 — Software Repeatability Verification
- **HTML Container ID:** `section.panel.glass` (Middle section of `tab-motion-cal`)
- **Parent Container:** `div#tab-motion-cal > main.dashboard-grid`
- **Purpose:** Manages 5-trial repeatability testing (1m forward & 90° turn) to calculate pass rate %, mean error, standard deviation, recommendation status, and export history (JSON/CSV).
- **Buttons & Controls:**
  - Status: `repeatability-mode-badge`
  - Checkboxes: `chk-repeat-ground-cleared`, `chk-repeat-estop-ready`
  - Buttons: `btn-repeat-run-1m`, `btn-repeat-run-90deg`, `btn-repeat-clear-history`, `btn-repeat-export-json`, `btn-repeat-export-csv`
  - Readouts: `repeat-pass-count`, `repeat-pass-rate`, `repeat-mean-dist`, `repeat-std-dist`, `repeat-mean-yaw`, `repeat-std-yaw`, `repeat-recom-status`
  - History Table: `repeat-table-body`
- **Frontend Handlers:** `btnRepeatRun1m.onclick` -> `runRepeatabilitySeries()`, `btnRepeatClearHistory.onclick` -> `clearRepeatabilityHistory()`, `btnRepeatExportJson.onclick` -> `exportRepeatabilityJSON()`, `btnRepeatExportCsv.onclick` -> `exportRepeatabilityCSV()`
- **Backend Endpoints Used:** `/api/calibration/repeatability/history`, `/api/calibration/repeatability/clear`, `/api/calibration/repeatability/export/json`, `/api/calibration/repeatability/export/csv`
- **WebSocket Messages Consumed:** `calibration_db`
- **Data Sources:** `calibration_db.json` `testLogs` array
- **Can Command Motors:** YES (Runs repeated physical test series)
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** YES (Generates recommendation status)
- **Read-Only:** NO
- **Tests Covering It:** `test_repeatability_sim.js`, `test_dom_contract.py`
- **Dependencies:** `calibration_db.json`

#### Panel 6-4: Calibrated Motion Command Testing
- **Visible Title:** ⚡ Calibrated Motion Command Testing
- **HTML Container ID:** `section.panel.glass` (Lower section of `tab-motion-cal`)
- **Parent Container:** `div#tab-motion-cal > main.dashboard-grid`
- **Purpose:** Executes precise calibrated linear displacement (meters) or angular rotation (degrees) commands.
- **Buttons & Controls:**
  - Linear inputs: `cal-motion-distance`, `cal-motion-speed`
  - Linear buttons: `btn-run-cal-forward`, `btn-run-cal-reverse`
  - Angular inputs: `cal-motion-angle`, `cal-motion-rot-speed`
  - Angular buttons: `btn-run-cal-ccw`, `btn-run-cal-cw`
- **Frontend Handlers:** `btnRunCalForward.onclick` -> `runCalibratedLinear()`, `btnRunCalCcw.onclick` -> `runCalibratedAngular()`
- **Backend Endpoints Used:** WebSocket `test_drive`
- **WebSocket Messages Consumed:** `status`
- **Data Sources:** Active calibrated parameters in `calibration_db.json`
- **Can Command Motors:** YES
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Active calibration constants

#### Panel 6-5: Encoder Odometry & Path Backtracking Control (Secondary Panel)
- **Visible Title:** 📂 Encoder Odometry & Path Backtracking Control
- **HTML Container ID:** `section.panel.glass` (Bottom section of `tab-motion-cal`)
- **Parent Container:** `div#tab-motion-cal > main.dashboard-grid`
- **Purpose:** Duplicated control panel for path recording and dead-reckoning return home.
- **Buttons & Controls:**
  - Duplicate buttons: `btn-record-start-2`, `btn-record-stop-2`, `btn-record-clear-2`, `btn-start-backtrack-2`, `btn-abort-backtrack-2`
  - Status labels: `path-recording-lbl-2`, `path-breadcrumbs-lbl-2`, `backtrack-state-lbl-2`, `backtrack-progress-bar-2`
- **Frontend Handlers:** Same backtrack function handlers (`startPathRecording()`, `startBacktrack()`, etc.)
- **Backend Endpoints Used:** `/api/path/record/*`, `/api/path/backtrack/*`
- **WebSocket Messages Consumed:** `path_status`, `backtrack_status`
- **Data Sources:** Dead-reckoning path recorder
- **Can Command Motors:** YES
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** NO
- **Tests Covering It:** `test_dom_contract.py`
- **Dependencies:** Dead-reckoning encoder odometry

---

### Tab 7: `tab-lidar` (📡 LiDAR Monitor)

#### Panel 7-1: LiDAR System Status
- **Visible Title:** 📡 LiDAR System Status
- **HTML Container ID:** `section.panel.glass` (Left panel of `tab-lidar`)
- **Parent Container:** `div#tab-lidar > main.dashboard-grid`
- **Purpose:** Status panel for RPLIDAR C1 (connection, device link, model, health, firmware, hardware, scan rate Hz, pts/s, points/rev, uptime, reconnects, last error).
- **Buttons & Controls:**
  - Readouts: `lidar-val-state`, `lidar-val-device`, `lidar-val-model`, `lidar-val-health`, `lidar-val-firmware`, `lidar-val-hardware`, `lidar-val-scanHz`, `lidar-val-pps`, `lidar-val-pointCount`, `lidar-val-uptime`, `lidar-val-reconnects`
  - Error Card: `lidar-error-card`, `lidar-val-error`
- **Frontend Handlers:** WebSocket LiDAR scan message handler
- **Backend Endpoints Used:** `/api/lidar/status`
- **WebSocket Messages Consumed:** `lidar_scan`
- **Data Sources:** `rplidar_sidecar.py` Python process on RPi5
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_lidar.js`, `test_dom_contract.py`
- **Dependencies:** `rplidar_sidecar.py`

#### Panel 7-2: 360° Polar Scan Display
- **Visible Title:** 🌐 360° Polar Scan Display
- **HTML Container ID:** `section.panel.glass` (Center/Right panel of `tab-lidar`)
- **Parent Container:** `div#tab-lidar > main.dashboard-grid`
- **Purpose:** Interactive HTML5 canvas rendering 360° polar scan points with range filter dropdown and stale data overlay.
- **Buttons & Controls:**
  - `lidar-range-select` (Dropdown: 1m, 3m, 6m, 12m)
  - `lidar-polar-canvas` (<canvas> element)
  - `lidar-stale-overlay` (Warning overlay)
- **Frontend Handlers:** `lidarRangeSelect.onchange`, `renderPolarScan()`
- **Backend Endpoints Used:** `/api/lidar/scan`
- **WebSocket Messages Consumed:** `lidar_scan`
- **Data Sources:** `rplidar_sidecar.py`
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_lidar.js`, `test_dom_contract.py`
- **Dependencies:** `rplidar_sidecar.py`

#### Panel 7-3: Live Scan Samples
- **Visible Title:** 📋 Live Scan Samples
- **HTML Container ID:** `section.panel.glass` (Bottom panel of `tab-lidar`)
- **Parent Container:** `div#tab-lidar > main.dashboard-grid`
- **Purpose:** Scrollable table displaying live sample points (Sample #, Angle °, Distance mm, Distance ft/in, Quality).
- **Buttons & Controls:** `lidar-sample-table-body`
- **Frontend Handlers:** Table DOM update loop in `app.js`
- **Backend Endpoints Used:** None
- **WebSocket Messages Consumed:** `lidar_scan`
- **Data Sources:** `rplidar_sidecar.py`
- **Can Command Motors:** NO
- **Can Arm/Disarm:** NO
- **Can Change Calibration:** NO
- **Read-Only:** YES
- **Tests Covering It:** `test_lidar.js`, `test_dom_contract.py`
- **Dependencies:** `rplidar_sidecar.py`

---

## 3. Developer, Maintenance, Firmware, and Raw-Log Elements

The following developer-focused and maintenance elements exist across the cockpit:

1. **Raw Serial Command Terminal:**
   - Container: `footer.logs-panel`
   - Inputs: `terminal-command-input`, `btn-send-raw-command`
   - Purpose: Direct injection of raw hex binary packets to serial port (`/dev/rover-esp32` or `COM18`).

2. **Phase 3 Individual-Wheel Maintenance Controls:**
   - Container: `div#tab-calibrate > main > section (Middle)`
   - Controls: `maint-safety-chk`, `btn-m1-fwd`..`btn-m4-rev` (8 buttons), `btn-stop-all-maint`
   - Purpose: Invokes firmware `MaintenanceManager` to pulse individual motors for 2.0s with raised-wheel requirement.

3. **Board Configurations & Tuning (Legacy Yahboom Flash Form):**
   - Container: `div#tab-dashboard > main > section > div.panel.config-panel.glass`
   - Inputs: `motor-type`, `deadband`, `phase-lines`, `reduction-ratio`, `wheel-diameter`, PID fields
   - Purpose: Obsolete form targeting old Yahboom ASCII protocol.

4. **Motor Power Proof Button:**
   - Container: `div#tab-dashboard > main > section.panel.controls-panel > div.diag-row`
   - Controls: `btn-motor-proof`
   - Purpose: Triggers server-side motor power proof script.

5. **Repeatability Export Tools:**
   - Container: `div#tab-motion-cal > main > section (Middle)`
   - Controls: `btn-repeat-export-json`, `btn-repeat-export-csv`, `btn-repeat-clear-history`
   - Purpose: Direct raw database manipulation and export of `calibration_db.json`.

---
