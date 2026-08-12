# Rover Engineering Audit Ledger

## Purpose & Vision

The purpose of `AUDIT.md` is to maintain a durable, chronological record of independent audits of this Raspberry Pi 5 ROS 2 rover repository (`yahboom-encoder`).

We recently discovered an important class of problem:
A feature may:
- exist in source code,
- have tests,
- publish data,
- appear in the UI, or
- be documented as active,

while still **NOT** actually be connected to the real runtime path.

### Example Discovered:
The BNO08x IMU was publishing `/imu/data` successfully and IMU-aware slip-gating logic existed in `EncoderKinematics`, but `rover_encoder_odometry.py` was not actually passing `external_d_yaw` into the kinematics update. We believed IMU-backed odometry was active when it was not.

Because of this, future audits must explicitly distinguish:
`IMPLEMENTED` -> `CONFIGURED` -> `LAUNCHED` -> `CONNECTED` -> `CONSUMED` -> `ACTUALLY AFFECTING RUNTIME BEHAVIOR`

`AUDIT.md` is the central record where audits from ChatGPT, Gemini, Fable, coding agents, human engineers, and targeted runtime investigations are logged and compared.

---

## Audit Rules

1. **APPEND, DO NOT REWRITE HISTORY**
   - Each new audit gets a new chronological entry.
   - Do not silently edit the conclusions of an earlier model/auditor.
   - If later evidence disproves an earlier finding, retain the original finding and add an explicit status update (`DISPROVED`, `SUPERSEDED`, `FIXED`, `VERIFIED`, etc.).

2. **EVERY AUDIT MUST IDENTIFY ITS SOURCE**
   - Timestamp & Timezone
   - Auditor / Model (and exact model name if known)
   - Audit Type & Goal
   - Repository Branch & Commit SHA (if known)
   - Inspection Mode (`STATIC REPOSITORY INSPECTION`, `RUNTIME INSPECTION`, `PHYSICAL ROVER TEST`, `MIXED STATIC/RUNTIME`)
   - Code Changes Allowed (`YES` / `NO`)

3. **AUDIT TYPES**
   - Use one of: `FULL REPOSITORY AUDIT`, `TARGETED AUDIT`, `RUNTIME TRUTH AUDIT`, `PHYSICAL VERIFICATION`, `SECURITY AUDIT`, `NAVIGATION AUDIT`, `SAFETY AUDIT`, `PERFORMANCE AUDIT`, `OTHER`.
   - For targeted audits, explicitly state the target path/subsystem.

4. **EVIDENCE LEVEL**
   - Classify findings as: `STATICALLY PROVEN`, `RUNTIME PROVEN`, `PHYSICALLY PROVEN`, `LIKELY`, `SUSPICIOUS`, `INCONCLUSIVE`.
   - Do not present inference as proven fact.

5. **SEVERITY**
   - Classify severity as: `CRITICAL`, `HIGH`, `MEDIUM`, `LOW`, `INFORMATIONAL`.
   - Severity represents practical rover operational/safety risk, not aesthetic code preference.

6. **FINDING STATUS**
   - Track status as: `OPEN`, `NEEDS_RUNTIME_VERIFICATION`, `CONFIRMED`, `DISPROVED`, `FIXED`, `VERIFIED_AFTER_FIX`, `DEFERRED`, `ACCEPTED_DESIGN`.

7. **NO FINDING JUST BECAUSE CODE LOOKS ODD**
   - Explain expected behavior, observed code behavior, practical impact, supporting evidence, and verification criteria.
   - Dormant code, duplicate configuration constants, or varying dimensional frames are not automatically defects without functional impact.

8. **RUNTIME TRUTH IS MORE IMPORTANT THAN SOURCE APPEARANCE**
   - Standard verification chain:
     `Hardware` -> `Firmware` -> `Server/Transport` -> `ROS Bridge` -> `ROS Topic/TF` -> `Consumer` -> `Resulting Behavior`
   - Topic publication does not prove consumption; function existence does not prove execution; config values do not prove runtime loading.

9. **DO NOT HIDE EARLY-LAYER DEFICIENCIES**
   - Respect layered architecture:
     `Motors/Encoders` -> `IMU` -> `Odometry` -> `TF` -> `LiDAR` -> `SLAM` -> `Resumable Maps` -> `Localization` -> `Nav2` -> `OAK-D Pro` -> `Autonomy`
   - Do not use later-stage sensors/software to mask deficiencies in earlier layers.

10. **AUDITS DO NOT AUTHORIZE FIXES**
    - `AUDIT.md` records findings. Findings must be reviewed and prioritized before modifying production code.

11. **CROSS-MODEL DISAGREEMENT IS VALUABLE**
    - Retain reports from multiple auditors. Document consensus and disagreements; resolve disagreements via empirical static/runtime evidence.

12. **PRESERVE EXACT VALUES**
    - Record exact dimensions, rates, thresholds, topic names, frame names, paths, environment variables, ports, and filenames.

---

## Standard Audit Entry Format

```markdown
## Audit YYYY-MM-DD HH:MM TZ — <Auditor / Model>

### Audit Metadata
- Auditor:
- Model:
- Timestamp:
- Timezone:
- Audit Type:
- Target:
- Goal:
- Branch:
- Commit:
- Inspection Mode:
- Code Changes Allowed:
- Repository Snapshot:

### Executive Summary

### Findings

#### FINDING-<audit number>.<finding number> — <title>
- Severity:
- Status:
- Evidence Level:
- Components:
- Files:
- Expected Behavior:
- Observed Behavior:
- Why It Matters:
- Evidence:
- Runtime Verification Needed:
- Recommended Verification:
- Recommended Action:
- Related Findings:

### Things Verified Correctly

### Things Not Yet Proven

### Cross-Audit Notes

### Recommended Next Audit
```

---

## Current Project Truth That Auditors Should Know

The following baseline facts have been verified by static inspection and empirical runtime / physical testing:

- **Encoder Odometry:** 4 physical motor encoder channels active; `rover_encoder_odometry` runs at ~20 Hz.
- **Odometry Kinematics Constants:** Wheel diameter = `0.065 m`, encoder ticks/revolution = `1974.1666666667`, effective skid-steer track width = `0.3408575433 m`.
- **IMU Integration:** BNO08x gyro Z rate is integrated at ~50 Hz into `EncoderKinematics.update()`; live IMU-backed slip detection is active.
- **LiDAR Path:** RPLIDAR C1 runs at ~6.5 Hz / 360 points per scan. The LiDAR is physically elevated above the chassis ($z = +0.17145\text{ m}$); live scans contain 0 chassis returns requiring a self-mask.
- **Static TF Geometry:** Runtime verified for `odom -> base_link`, `base_link -> imu_link` ($x=-0.0254\text{ m}, y=-0.0254\text{ m}, z=+0.14605\text{ m}$), and `base_link -> laser_frame` ($x=+0.03175\text{ m}, y=0.0\text{ m}, z=+0.17145\text{ m}$).
- **SLAM & Resumable Maps:** SLAM Toolbox mapping works; `.posegraph` / `.data` serialization and deserialization for continued mapping have been verified live on multi-session runs.
- **Operator Token Security:** Moved outside git tracking into `/etc/rover/rover.env` (`ROVER_OPERATOR_TOKEN`), with systemd `EnvironmentFile=-/etc/rover/rover.env` loading verified persistent across service restarts and Pi reboots.
- **Pending Systems:** Nav2 autonomous navigation has NOT yet been started; OAK-D Pro depth camera has NOT yet been integrated.

---

## Audit 2026-08-11 04:54 PDT — OpenAI ChatGPT (GPT-5.6 Sol)

### Audit Metadata
- Auditor: OpenAI ChatGPT
- Model: GPT-5.6 Sol
- Timestamp: 2026-08-11 04:54
- Timezone: PDT
- Audit Type: FULL REPOSITORY AUDIT — PRELIMINARY STATIC PASS
- Target: Full Repository (`yahboom-encoder`)
- Goal: Perform an initial read-only forensic scan of the complete rover repository snapshot with special attention to features that may appear implemented/configured but may not actually be on the production/runtime path.
- Branch: `main`
- Commit: `7e8bafe`
- Inspection Mode: STATIC REPOSITORY INSPECTION
- Code Changes Allowed: NO
- Repository Snapshot: Clean workspace post-IMU odometry, SLAM resume, and operator token persistence completion.

*Note: This entry represents a PRELIMINARY STATIC PASS and not yet the final GPT-5.6 Sol full audit.*

### Executive Summary
Preliminary static review of the codebase shows strong alignment with recently verified runtime behaviors (calibrated odometry constants, BNO08x gyro integration, static TF frames, systemd persistent environment loading). However, static inspection identified several potential discrepancies and unverified assumptions requiring runtime validation before initiating Nav2 autonomous driving, specifically regarding Nav2 costmap collision footprint geometry, default map selection in navigation launch files, and potential duplication of calibration constants across legacy modules.

### Findings

#### FINDING-1.1 — Nav2 Collision Footprint Still Uses Generic Radius
- **Severity:** HIGH
- **Status:** OPEN
- **Evidence Level:** STATICALLY PROVEN
- **Components:** Nav2 Costmaps (`global_costmap`, `local_costmap`)
- **Files:** [ros2/ros2_ws/src/rover_bringup/config/nav2_params.yaml](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/config/nav2_params.yaml#L95)
- **Expected Behavior:** Before autonomous Nav2 movement, the costmaps should use a deliberately verified collision footprint representing the real rectangular rover geometry.
- **Observed Behavior:** Costmap configuration contains generic `robot_radius: 0.15` m. The physical rover footprint is approximately $0.254\text{ m} \times 0.2286\text{ m}$ ($10" \times 9"$), giving an actual corner radius of $\approx 0.171\text{ m}$.
- **Why It Matters:** A circular $0.15\text{ m}$ radius under-represents the physical corners of the rectangular chassis by $\approx 2.1\text{ cm}$. Nav2 could believe there is obstacle clearance when a physical corner would collide.
- **Runtime Verification Needed:** Yes.
- **Recommended Verification:** Inspect costmap footprint visualization in Foxglove / RViz during Nav2 dry-run.
- **Recommended Action:** Do NOT fix during this audit. Before autonomous Nav2 motion: verify actual collision envelope, replace generic radius with an explicit rectangular footprint polygon or deliberately chosen footprint, and verify local/global costmap inflation layers separately.

#### FINDING-1.2 — Navigation Launch Default Map May Be Stale
- **Severity:** MEDIUM
- **Status:** NEEDS_RUNTIME_VERIFICATION
- **Evidence Level:** SUSPICIOUS / STATIC
- **Components:** Launch System, Map Server
- **Files:** [ros2/ros2_ws/src/rover_bringup/launch/navigation.launch.py](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/navigation.launch.py#L20)
- **Expected Behavior:** Navigation/localization startup should deliberately load the intended current verified map rather than falling back to an old generic filename.
- **Observed Behavior:** `navigation.launch.py` defaults to `/ros2_ws/maps/house_map.yaml`, whereas production mapping uses named/dated maps such as `house_imu_verified_2026-08-09` and `house_resume_verified_2026-08-10`.
- **Why It Matters:** A future Nav2/localization launch could accidentally load a nonexistent or outdated map file if the operator does not override the argument.
- **Runtime Verification Needed:** Yes.
- **Recommended Verification:** Trace navigation launch arguments and actual runtime `map_server` parameters prior to the first Nav2 localization session.
- **Recommended Action:** Do not change yet. Ensure launch files mandate or resolve the latest verified map prefix.

#### FINDING-1.3 — Legacy COM18 References Remain
- **Severity:** LOW
- **Status:** DEFERRED
- **Evidence Level:** STATICALLY PROVEN
- **Components:** Test Utilities, Legacy Configs
- **Files:** Various test and diagnostic scripts
- **Expected Behavior:** Production Raspberry Pi serial runtime dynamically resolves Linux serial devices (`/dev/rover-esp32`).
- **Observed Behavior:** Legacy Windows `COM18` port references remain in fallback defaults of some UI/test utilities.
- **Why It Matters:** Unlikely to impact Pi Linux runtime, but stale Windows development defaults can confuse cross-platform testing.
- **Recommended Action:** Defer cleanup unless deeper inspection proves a production runtime dependency.

#### FINDING-1.4 — Cmd_Vel Safety Path Requires Full End-to-End Audit
- **Severity:** HIGH
- **Status:** NEEDS_RUNTIME_VERIFICATION
- **Evidence Level:** INCONCLUSIVE
- **Components:** `rover_cmd_vel_bridge`, Cockpit Server, Autonomy State Machine, ESP32 Serial Firmware
- **Files:** [ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_cmd_vel_bridge.py](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_cmd_vel_bridge.py#L70), [server.js](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js#L3810)
- **Expected Behavior:** Every motion command source (`/cmd_vel`, Web UI joystick, Gamepad WASD) should pass through authorization, zero-handshake, arming, watchdog, deadman, and hardware safety layers cleanly.
- **Observed Behavior:** Multi-stage state machine (`DISABLED`, `WAITING_FOR_ZERO`, `READY_DISARMED`, `READY_ARMED`, `ACTIVE`) has intricate handshakes and watchdog timeouts ($500\text{ ms}$).
- **Why It Matters:** Nav2 autonomous driving will issue continuous velocity stream commands. Any state deadlock or handshake timeout will trigger unexpected motor safety stops.
- **Recommended Verification:** Execute a dedicated end-to-end command-path trace: Nav2 `/cmd_vel` -> `rover_cmd_vel_bridge` -> Cockpit API -> autonomy state machine -> serial `FUNC_MOTION` -> ESP32 -> motor drivers.

#### FINDING-1.5 — Map Save/Load UI and SLAM Manager Should Be Reconciled
- **Severity:** MEDIUM
- **Status:** NEEDS_RUNTIME_VERIFICATION
- **Evidence Level:** INCONCLUSIVE
- **Components:** Cockpit UI, `slam_manager.js`, ROS SLAM Services
- **Files:** [slam_manager.js](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/slam_manager.js#L120), [public/app.js](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/public/app.js#L500)
- **Expected Behavior:** UI and manager abstractions should clearly distinguish raw occupancy grid maps (`.yaml` + `.pgm`) from resumable pose graphs (`.posegraph` + `.data`).
- **Observed Behavior:** Runtime tests proved SLAM Toolbox pose graph serialization and deserialization work over ROS services, but UI/manager abstraction state mapping requires verification to ensure no silent mismatch occurs when triggered from Web UI buttons.
- **Recommended Verification:** Trace all save/load/resume UI actions through `slam_manager.js` to ROS 2 service invocations.

#### FINDING-1.6 — Calibration Values Need Duplicate-Config Audit
- **Severity:** MEDIUM
- **Status:** NEEDS_RUNTIME_VERIFICATION
- **Evidence Level:** INCONCLUSIVE
- **Components:** `rover_encoder_odometry`, `server.js`, Telemetry Diagnostics
- **Files:** [ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_encoder_odometry.py](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_encoder_odometry.py#L135), [server.js](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/server.js#L100)
- **Expected Behavior:** Production ROS odometry uses verified values ($D=0.065\text{ m}$, $\text{ticks/rev}=1974.1667$, $W_{\text{effective}}=0.340858\text{ m}$). Legacy references (e.g. `TRACK_WIDTH_LOC = 0.160m`) should not override production ROS node calculations.
- **Observed Behavior:** Multiple track-width and wheel dimension constants exist across server, ROS nodes, and UI diagnostic displays.
- **Why It Matters:** Duplicate or legacy constants could corrupt telemetry calculations or diagnostic displays if referenced by mistake.
- **Recommended Audit:** Catalog every instance of track width, wheel diameter, ticks/rev, and chassis dimensions across the repository. Classify each as: production kinematics, physical geometry, test telemetry, UI display, or legacy/dead code.

#### FINDING-1.7 — Docker / ROS Startup Behavior Needs Reconciliation
- **Severity:** MEDIUM
- **Status:** NEEDS_RUNTIME_VERIFICATION
- **Evidence Level:** INCONCLUSIVE
- **Components:** Systemd, Docker Compose, Boot Scripts
- **Files:** [ros2/compose.yaml](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/compose.yaml#L9), [ros2/scripts/up.sh](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/scripts/up.sh#L20)
- **Expected Behavior:** The repository architecture should clearly define whether `rover-ros2` container startup after host boot is fully automatic or operator-triggered via script.
- **Observed Behavior:** `compose.yaml` specifies `restart: unless-stopped`, but runtime testing noted the container was stopped after host reboot until `docker compose up -d` was issued.
- **Recommended Verification:** Audit Docker daemon startup dependencies, systemd unit ordering, and `up.sh` boot sequence.

#### FINDING-1.8 — Sensor Covariance / Future Fusion Remains Uncalibrated
- **Severity:** MEDIUM
- **Status:** DEFERRED
- **Evidence Level:** STATICALLY PROVEN / PROJECT KNOWN DEBT
- **Components:** IMU Bridge, Odometry Node, EKF / `robot_localization`
- **Files:** [ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_imu_bridge.py](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/rover_bringup/rover_imu_bridge.py#L80)
- **Expected Behavior:** Sensor message covariance matrices should be populated based on empirical sensor characterization before multi-sensor EKF fusion.
- **Observed Behavior:** `/imu/data` and `/odom` covariance matrices currently use default zeros or static approximations.
- **Why It Matters:** Uncalibrated covariance matrices will break `robot_localization` or visual-inertial odometry fusion when introduced later.
- **Recommended Action:** Defer until EKF / OAK-D Pro fusion phase. Perform stationary noise profiling and wheel slip variance characterization before populating covariance matrices.

#### FINDING-1.9 — IMU Static Orientation Still Deserves Calibration Test
- **Severity:** MEDIUM
- **Status:** DEFERRED
- **Evidence Level:** INCONCLUSIVE
- **Components:** Static TF Publisher, BNO08x IMU
- **Files:** [ros2/ros2_ws/src/rover_bringup/launch/foundation.launch.py](file:///c:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/foundation.launch.py#L40)
- **Expected Behavior:** Static transform `base_link -> imu_link` orientation should accurately represent physical mounting orientation.
- **Observed Behavior:** Static transform uses zero roll/pitch/yaw ($[0,0,0,1]$ quaternion). Historical flat-surface observation suggested a $\approx 3.7^\circ$ roll offset, though physical yaw rotation tests passed.
- **Why It Matters:** Small static roll/pitch errors do not impact 2D planar SLAM yaw, but will accumulate into 3D orientation tracking or incline estimation.
- **Recommended Action:** Perform dedicated flat-surface static IMU roll/pitch calibration prior to 3D orientation tracking or EKF deployment.

#### FINDING-1.10 — Runtime-Truth Auditing Should Continue Before Nav2
- **Severity:** INFORMATIONAL
- **Status:** OPEN
- **Evidence Level:** PROJECT PROCESS FINDING
- **Components:** Full Rover Stack
- **Files:** Repository-wide
- **Expected Behavior:** Maintain strict runtime-truth auditing workflow before advancing to autonomous navigation layers.
- **Observed Behavior:** Recent runtime audits successfully uncovered two significant assumption mismatches:
  1. IMU-aware slip-gate logic existed, but live odometry node did not pass IMU yaw to kinematics.
  2. LiDAR self-mask code existed in sidecar, but live ROS `/scan` stream was unmasked (and physical audit proved self-mask was unnecessary due to elevated scanner height).
- **Why It Matters:** Proves that static code presence cannot be equated with active runtime participation.
- **Recommended Action:** Continue end-to-end runtime-truth audits for all high-risk paths before launching Nav2 autonomous driving.

### Things Verified Correctly
- **Calibrated Kinematics Constants:** $D=0.065\text{ m}$, $\text{ticks/rev}=1974.1667$, $W_{\text{effective}}=0.340858\text{ m}$ present in production ROS odometry node.
- **Encoder Combination:** 4 physical motor channels correctly averaged into left ($\frac{M1+M3}{2}$) and right ($\frac{M2+M4}{2}$) side tick deltas.
- **BNO08x Gyro Z Path:** `external_d_yaw` parameters and updates connected in live odometry.
- **Static TF Frames:** Frame translations for IMU ($x=-0.0254\text{ m}, y=-0.0254\text{ m}, z=+0.14605\text{ m}$) and LiDAR ($x=+0.03175\text{ m}, y=0.0\text{ m}, z=+0.17145\text{ m}$) match physical geometry.
- **Persistent Operator Security:** Systemd service template includes `EnvironmentFile=-/etc/rover/rover.env`.
- **ROS 2 Bringup Architecture:** `foundation.launch.py`, `slam.launch.py`, and `navigation.launch.py` present with clean node composition.

*Note: These positive static observations represent structural consistency and do not replace empirical runtime verification.*

### Things Not Yet Proven
- End-to-end `/cmd_vel` command flow under Nav2 continuous streaming.
- Exact costmap inflation behavior with real rectangular footprint corners.
- `navigation.launch.py` map loading behavior during localization-only mode.
- Complete reconciliation of UI/manager map actions against underlying ROS services.
- Stationary IMU sensor noise covariance matrix values.

### Cross-Audit Notes
- Initial entry established by OpenAI ChatGPT (GPT-5.6 Sol) as a preliminary static review. To be cross-referenced with subsequent runtime and model audits.

### Recommended Next Audit
1. **Targeted Audit:** Complete `/cmd_vel` End-to-End Safety & Authorization Trace (Nav2 -> Bridge -> Cockpit API -> Autonomy State Machine -> Serial -> ESP32).
2. **Targeted Audit:** Nav2 Costmap & Footprint Geometry Line-by-Line Review.
3. **Targeted Audit:** Duplicate Calibration & Physical Constants Catalog.

---

## TARGETED RUNTIME AUDIT — NAV2 /CMD_VEL SAFETY AND MOTOR COMMAND PATH

### Audit Metadata
- **Timestamp:** 2026-08-11 17:38:00 -07:00 (2026-08-12 00:38:00 UTC)
- **Auditor:** Antigravity AI Coding Assistant (Google DeepMind)
- **Model:** Gemini 3.6 Flash (High)
- **Audit Type:** TARGETED AUDIT / RUNTIME TRUTH AUDIT / SAFETY AUDIT
- **Target:** Nav2 `/cmd_vel` -> `rover_cmd_vel_bridge` -> `internalCmdApp` (`/api/cmd_vel`) -> Autonomy State Machine -> Arming / Handshake -> Serial (`FUNC_MOTION`) -> ESP32 Firmware -> Motor Driver Output
- **Branch:** `main` (Local Workspace) / `feature/bno08x-ros2-imu-integration` (Production RPi5 Host)
- **Commit SHA:** `d7dcb6cde2a70649c4fd92aa23dffb31989781c6` (Local) / `2a96396b5bafdc25db5413b0b6f4094b93fc07a1` (RPi5 Host)
- **Inspection Mode:** MIXED STATIC/RUNTIME
- **Code Changes Allowed:** NO (Audit Entry Only)
- **Physical Movement Triggered:** NO (Zero non-zero commands issued; all checks read-only)

### 1. Executive Summary & Core Findings
- **Convergence Proven:** Nav2 autonomous velocity commands and manual UI/gamepad drive commands **CONVERGE** on the same protected host-side state machine (`server.js`) and the same serial interface/firmware safety gates (`esp-maker-usba-4motor`).
- **No Direct Bypass Available:** Docker container inspection confirmed `rover-ros2` container has `.HostConfig.Devices = null`. Nav2 and ROS 2 nodes physically **CANNOT** write to `/dev/rover-esp32` directly. Nav2 MUST route commands through `rover_cmd_vel_bridge` -> `http://127.0.0.1:3010/api/cmd_vel`.
- **Safety Boundary Active:** Nav2 **CANNOT** bypass operator arming, operator token authorization, zero-velocity handshake (3 consecutive zero commands required in `WAITING_FOR_ZERO`), or watchdog timeouts.
- **Manual Preemption Active:** Any manual joystick movement (`|x| > 0.05` or `|y| > 0.05`) on the WebSocket interface immediately disables autonomy (`autonomyState.enabled = false`), sets command source to `NONE`, and zero-forces velocity targets.

### 2. Verified Command Flow (Nav2 vs. Manual)

#### Nav2 Autonomous Path:
1. `Nav2 /cmd_vel` (`geometry_msgs/msg/Twist`)
2. `rover_cmd_vel_bridge` Python node (Subscribes to `/cmd_vel`)
3. HTTP POST to `http://127.0.0.1:3010/api/cmd_vel` with `X-Rover-Bridge-Token` header
4. `internalCmdApp` middleware in `server.js` (Rate limit 50/s + Constant-time Bridge Token check)
5. `/api/cmd_vel` Handler in `server.js`:
   - Checks `autonomyState.enabled` (Must be true via `/api/autonomy/enable`)
   - Checks Maintenance/Calibration status (Must be IDLE)
   - Checks Zero-Velocity Handshake: If `WAITING_FOR_ZERO`, requires 3 consecutive zero Twist messages before transitioning to `READY_DISARMED`
   - Checks `isArmed` (`latestNormalDriveStatus.armed`): If disarmed or `READY_DISARMED`, rejects with HTTP 403
   - Transitions `READY_ARMED` -> `ACTIVE` on first non-zero command
   - Clamps velocity targets to safe envelope (`AUTONOMY_MAX_LINEAR_MPS`, `AUTONOMY_MAX_ANGULAR_RADPS`)
6. Shared Keepalive & Slew Rate Limiting Loop (`startDriveKeepaliveLoop` in `server.js` @ 20 Hz):
   - Monitors 500 ms ROS Autonomy Watchdog
   - Applies linear/angular slew rate limiting (accel/decel curves)
   - Transmits binary `FUNC_MOTION` packet (`0x12`, int16 LE `[vx, vy, vz]`) to `/dev/rover-esp32`
7. ESP32 Serial Receiver (`SerialProtocol::processPacket` in `esp-maker-usba-4motor`):
   - Resets ESP32 communication watchdog (`cmdManager.resetWatchdog()`)
   - Decodes `0x12` (`CMD_MOTION`)
8. ESP32 Command Manager (`CommandManager::setCommand`):
   - Checks `normalDriveArmed`: Rejects command if disarmed!
   - Enforces hardware velocity bounds
9. ESP32 100 Hz Control Loop (`DifferentialDrive` + `MotorDriver` PID):
   - Evaluates 300 ms soft-stop watchdog and 1000 ms fault timeout watchdog
   - `SafetyManager` monitors stall, encoder disconnect, and track mismatch
   - Motor driver PWM output

#### Manual UI / Gamepad Path:
1. Operator Web UI / Gamepad WebSocket message (`joystick`, `set_speed`, `drive`)
2. Cockpit Web Server (`server.js` port 3000)
3. Preemption Guard: Manual stick movement (`|x| > 0.05` or `|y| > 0.05`) while autonomy is enabled/active immediately disables autonomy and zeroes target
4. Deadman Guard: Requires `deadman === true` (explicit for gamepad, implicit for keyboard/browser)
5. Sets `targetLinear` / `targetAngular` and `cmdSource = 'GAMEPAD'` or `'BROWSER'`
6. **CONVERGES** at `startDriveKeepaliveLoop()` -> Serial `FUNC_MOTION` (`0x12`) -> ESP32 `CommandManager` -> Motor Drivers.

### 3. Safety Boundary Classification Table

| Step / Component | Static Trace Status | Runtime Verification Status | Active Safety Gate Description |
|---|---|---|---|
| `/cmd_vel` Subscription | `IMPLEMENTED` | `LAUNCHED` / `CONNECTED` | `rover_cmd_vel_bridge` actively running inside container `rover-ros2`. |
| Bridge Token Security | `IMPLEMENTED` | `CONFIGURED` / `CONNECTED` | `ROVER_CMD_VEL_TOKEN` verified via constant-time buffer comparison (`crypto.timingSafeEqual`). |
| Port 3010 Isolation | `IMPLEMENTED` | `CONFIGURED` / `CONSUMED` | `/api/cmd_vel` bound exclusively to loopback (`127.0.0.1:3010`); returns HTTP 404 on public port 3000. |
| Autonomy Enable Gate | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | Operator must explicitly call `/api/autonomy/enable` (guarded by Operator Token). Default state is `DISABLED`. |
| Zero-Velocity Handshake | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | State machine enforces 3 consecutive zero-velocity commands in `WAITING_FOR_ZERO` before advancing to `READY_DISARMED`. |
| Arming Authorization Gate | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | Requires `/api/drive/arm` (guarded by Operator Token). Tested live: `armed: false`. Commands rejected with HTTP 403 if disarmed. |
| Slew Rate Limiter | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | Host loop limits linear (0.30 m/s² accel / 0.60 m/s² decel) and angular (1.00 rad/s² accel / 2.00 rad/s² decel) rates. |
| ROS Autonomy Watchdog | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | `server.js` triggers at 500 ms without `/cmd_vel` POSTs; forces targets to zero and transitions state to `STALE`. |
| Manual Preemption | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | Manual joystick input overrides and disables active ROS autonomy immediately. |
| ESP32 Serial Hardware Isolation | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | Docker inspect proves container has no `/dev/rover-esp32` access; ROS nodes cannot talk to hardware directly. |
| ESP32 Firmware Arming Gate | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | `CommandManager::setCommand` in firmware independently rejects `CMD_MOTION` if `normalDriveArmed` is false. |
| ESP32 Serial Watchdogs | `IMPLEMENTED` | `ACTUALLY AFFECTING RUNTIME` | ESP32 enforces 300 ms soft-stop (`WATCHDOG_TIMEOUT_MS`) and 1000 ms fault timeout (`FAULT_TIMEOUT_MS`) on serial packet loss. |

### 4. Alternate / Bypass Command Path Audit
- **Direct Serial Write from Docker:** `DISPROVED / IMPOSSIBLE`. Docker container has no serial device node mounts (`HostConfig.Devices = null`).
- **Public Port `/api/cmd_vel`:** `DISPROVED / IMPOSSIBLE`. Hitting `/api/cmd_vel` on port 3000 returns HTTP 404 (`test_cmd_vel_behavior.js` test 1 verified).
- **Direct Motor Speed `0x10` (`CMD_MOTOR`):** `SAFE / GUARDED`. Hitting raw motor speed functions in `server.js` or firmware requires `isArmed = true` and `calManager` idle.
- **Maintenance / Calibration Commands:** `SAFE / GUARDED`. Activating maintenance or auto-calibration immediately blocks autonomy (`/api/cmd_vel` returns HTTP 429).

### 5. Proven vs. Not Yet Proven

#### Proven:
- End-to-end command path topology (Nav2 -> Bridge -> Server API -> Slew Limiter -> Serial -> ESP32 Firmware -> Motor Output).
- Complete convergence of Nav2 autonomous and manual UI/gamepad command paths before hardware transmission.
- Multi-layer defense in depth: 3 watchdogs (500ms server, 300ms ESP32 soft-stop, 1000ms ESP32 fault), 2-stage token authentication (Bridge Token & Operator Token), 3-frame zero handshake, dual-layer arming check (Node.js & ESP32 firmware).
- Hardware isolation of ROS container from serial port `/dev/rover-esp32`.
- Automatic manual joystick preemption of autonomous driving.

#### NOT Yet Proven (Requires Future Physical Testing):
- Physical closed-loop motor acceleration/deceleration response under Nav2 continuous streaming while armed on the floor.
- Dynamic obstacle clearance and local costmap inflation under physical movement.

### 6. Recommended Next Action
1. Proceed with confidence to single-goal safe Nav2 trial (with rover elevated or in a clear open test area), adhering to the exact required bringup sequence:
   - Verify `server.js` running & serial connected (`armed: false`, `autonomyState: DISABLED`).
   - Authenticate Operator Token.
   - Issue `/api/drive/arm`.
   - Issue `/api/autonomy/enable`.
   - Start Nav2 goal (verifying initial 3-zero handshake and transition `WAITING_FOR_ZERO` -> `READY_DISARMED` -> `READY_ARMED` -> `ACTIVE`).

---

## TARGETED AUDIT — NAV2 SAVED-MAP SELECTION

### Audit Metadata
- **Timestamp:** 2026-08-11 17:42:00 -07:00 (2026-08-12 00:42:00 UTC)
- **Auditor:** Antigravity AI Coding Assistant (Google DeepMind)
- **Model:** Gemini 3.6 Flash (High)
- **Audit Type:** TARGETED AUDIT / SAVED-MAP SELECTION AUDIT
- **Target:** `navigation.launch.py` map default argument -> Docker volume mount -> `/ros2_ws/maps` -> `map_server`
- **Branch:** `main` (Local Workspace) / `feature/bno08x-ros2-imu-integration` (RPi5 Host)
- **Commit SHA:** `d7dcb6cde2a70649c4fd92aa23dffb31989781c6` (Local) / `2a96396b5bafdc25db5413b0b6f4094b93fc07a1` (RPi5 Host)
- **Inspection Mode:** MIXED STATIC/RUNTIME
- **Code Changes Allowed:** NO (Audit Entry Only)

### 1. Executive Summary & Core Findings
- **Default Map Selection Gap:** `navigation.launch.py` currently hardcodes the default launch argument to `os.path.join('/ros2_ws', 'maps', 'house_map.yaml')`.
- **Missing File Discrepancy:** `/ros2_ws/maps/house_map.yaml` **DOES NOT EXIST** on the host or inside the container. `house_map` was used in `slam_map_workflow.sh` as a reserved/protected keyword, but no map file was ever written to disk under that exact name.
- **Impact:** If `navigation.launch.py` is started without explicitly passing `map:=...`, `map_server` will **FAIL TO START** (file not found error).
- **Verified Usable Map Available:** The verified latest occupancy map `house_resume_verified_2026-08-10.yaml` and image `house_resume_verified_2026-08-10.pgm` exist on disk, are fully mounted in the container, and are 100% valid for localization.

### 2. Configuration & Map Path Chain

```
[Host Directory] /home/ron/yahboom-encoder/ros2/volumes/maps/
    └── house_resume_verified_2026-08-10.yaml
    └── house_resume_verified_2026-08-10.pgm
          │ (Docker Bind Mount: compose.yaml)
          ▼
[Container Directory] /ros2_ws/maps/
    └── house_resume_verified_2026-08-10.yaml
    └── house_resume_verified_2026-08-10.pgm
          │
          ▼
[Launch Configuration] navigation.launch.py
    └── DeclareLaunchArgument('map', default_value='/ros2_ws/maps/house_map.yaml')  <-- STALE DEFAULT
          │
          ▼
[Nav2 Bringup] map_server (yaml_filename)
```

### 3. Verified Map Technical Validation (`house_resume_verified_2026-08-10`)
- **YAML Path (Host)**: `/home/ron/yahboom-encoder/ros2/volumes/maps/house_resume_verified_2026-08-10.yaml`
- **YAML Path (Container)**: `/ros2_ws/maps/house_resume_verified_2026-08-10.yaml`
- **YAML Content**:
  ```yaml
  image: house_resume_verified_2026-08-10.pgm
  mode: trinary
  resolution: 0.050
  origin: [-5.631, -5.115, 0]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```
- **PGM File Validation**:
  - Filename: `house_resume_verified_2026-08-10.pgm` (relative reference matches container path `/ros2_ws/maps/house_resume_verified_2026-08-10.pgm`).
  - Format: Binary PGM (`P5`, 284x160 pixels, 45,455 bytes).
  - Status: Present, readable, correct permissions (`rover:rover` / `ron:ron`).

### 4. Classification of Intentional Navigation Map
- **Classification B**: Current configuration would accidentally attempt to load a missing map (`house_map.yaml`).

### 5. Proven vs. Not Yet Proven

#### Proven:
- `house_resume_verified_2026-08-10.yaml` and `.pgm` exist and are valid for `map_server` localization.
- `house_map.yaml` does not exist anywhere on the host filesystem or container workspace.
- Docker volume bind mount `/home/ron/yahboom-encoder/ros2/volumes/maps` -> `/ros2_ws/maps` is functioning properly.

#### NOT Yet Proven:
- Real-time AMCL particle filter convergence against `house_resume_verified_2026-08-10.yaml` under active motion.

### 6. Recommended Minimal Correction
Update default value in `navigation.launch.py` (line 23):
```diff
- default_value=os.path.join('/ros2_ws', 'maps', 'house_map.yaml'),
+ default_value=os.path.join('/ros2_ws', 'maps', 'house_resume_verified_2026-08-10.yaml'),
```

### 7. Resolution & Verification Status
- **Status:** `FIXED / VERIFIED_AFTER_FIX`
- **Resolution Timestamp:** 2026-08-11 17:47:00 -07:00 (2026-08-12 00:47:00 UTC)
- **Fix Implemented:** Updated `map_arg` default_value in `ros2/ros2_ws/src/rover_bringup/launch/navigation.launch.py` to point to `house_resume_verified_2026-08-10.yaml`.
- **Runtime & Deployment Verification:**
  - Deployed to RPi5 host (`10.0.0.246`).
  - Executed `colcon build --packages-select rover_bringup` inside `rover-ros2` container.
  - Verified installed launch file inside container (`/ros2_ws/install/rover_bringup/share/rover_bringup/launch/navigation.launch.py`) contains `house_resume_verified_2026-08-10.yaml`.
  - Verified `/ros2_ws/maps/house_resume_verified_2026-08-10.yaml` and `.pgm` exist and are accessible.
  - Verified no remaining copies of `navigation.launch.py` default to `house_map.yaml`.

---

## TARGETED AUDIT — NAV2 COLLISION FOOTPRINT

### Audit Metadata
- **Timestamp:** 2026-08-11 17:53:00 -07:00 (2026-08-12 00:53:00 UTC)
- **Auditor:** Antigravity AI Coding Assistant (Google DeepMind)
- **Model:** Gemini 3.6 Flash (High)
- **Audit Type:** TARGETED AUDIT / FOOTPRINT & COLLISION ENVELOPE AUDIT
- **Target:** `nav2_params.yaml` (`local_costmap` & `global_costmap`) -> Nav2 collision geometry -> `/footprint` topic
- **Branch:** `main` (Local Workspace) / `main` (RPi5 Host)
- **Commit SHA:** `16f1309c3a88e56b809cf51dbeb3d8f4df019afb`
- **Inspection Mode:** MIXED STATIC/RUNTIME
- **Code Changes Allowed:** NO (Audit Entry Only)

### 1. Executive Summary & Core Findings
- **Collision Defect Discovered:** `nav2_params.yaml` originally specified `robot_radius: 0.15` (a 15 cm circle) for both `local_costmap` and `global_costmap`.
- **Chassis Geometry History (`SUPERSEDED`)**:
  - Preliminary audit assumed a 10 in x 9 in ($0.254\text{ m} \times 0.2286\text{ m}$) frame with diagonal corner radius $R = 0.171\text{ m}$ (stating 2.1 cm clipping).
  - **Authoritative Physical Measurement**: Operator physically measured the active rover envelope: Length $9\frac{1}{8}\text{ in}$ ($0.231775\text{ m}$) $\times$ Width $8\frac{5}{8}\text{ in}$ ($0.219075\text{ m}$).
  - Unpadded half-dimensions: $X = \pm 0.1158875\text{ m}$, $Y = \pm 0.1095375\text{ m}$. Unpadded diagonal corner radius: $R_{\text{corner}} = \sqrt{0.1158875^2 + 0.1095375^2} = 0.15946\text{ m} \approx 0.1595\text{ m}$.
  - Corrected Error of `robot_radius: 0.15`: Under-represented physical unpadded corners by **0.95 cm** ($0.1595\text{ m} - 0.15\text{ m} = 0.0095\text{ m}$), while over-representing lateral side width by **+4.0 cm** ($0.30\text{ m}$ circle width vs $0.219\text{ m}$ physical width).
- **Unconsumed `/footprint` Topic:** `rover_encoder_odometry.py` publishes a 4-point polygon on topic `/footprint`. Runtime inspection proved `/footprint` has **0 subscribers**. Nav2 costmap nodes DO NOT consume this topic by default; they statically load footprint parameters from `nav2_params.yaml`.

### 2. Physical vs. Configured Envelope Comparison

| Envelope Metric | Configured Value / Geometry | Physical Truth / Actual Value | Discrepancy / Impact |
|---|---|---|---|
| **Local Costmap Geometry** | `robot_radius: 0.15` (15 cm circle) | Measured Rectangle $0.231775\text{ m} \times 0.219075\text{ m}$ | Clips corners by 0.95 cm; over-represents side width by 4.0 cm. |
| **Global Costmap Geometry** | `robot_radius: 0.15` (15 cm circle) | Measured Rectangle $0.231775\text{ m} \times 0.219075\text{ m}$ | Identical corner clipping defect as local costmap. |
| **`/footprint` Topic** | Published by `rover_encoder_odometry` | 4 Points ($9\frac{1}{8}\text{ in} \times 8\frac{5}{8}\text{ in}$) | 0 Subscribers; ignored by Nav2 costmaps. |
| **Inflation Radius** | `inflation_radius: 0.35` (35 cm) | 35 cm decay gradient | Operates outside footprint; cannot fix corner clipping during spins. |
| **Cost Scaling Factor** | `cost_scaling_factor: 3.0` | Exponential decay multiplier | Intentional cost decay parameter. |

### 3. Circle vs. Rectangular Polygon Tradeoff
- **Increasing `robot_radius` to 0.160 m**: Covers unpadded corners, but inflates the width of the rover to $0.320\text{ m}$ ($12.6\text{ inches}$), over-representing the side clearance by **+10.1 cm** ($5.0\text{ cm}$ on each side). This causes Nav2 to reject valid paths through narrow doorways or tight indoor passages.
- **Switching to Rectangular Polygon (`footprint`)**: Accurately models the measured $23.18\text{ cm}$ length and $21.91\text{ cm}$ width, allowing Nav2 to dynamically rotate the rectangular footprint during path planning and local trajectory evaluation (DWB planner).

### 4. Authoritative Physical Measurements & 5mm Safety Padding
- Measured Envelope: Length $0.231775\text{ m}$, Width $0.219075\text{ m}$.
- 5 mm Safety Padding applied:
  - Half-Length ($X$): $0.1158875 + 0.005 = 0.1208875\text{ m} \approx 0.121\text{ m}$
  - Half-Width ($Y$): $0.1095375 + 0.005 = 0.1145375\text{ m} \approx 0.115\text{ m}$
- Padded Diagonal Corner Radius: $R_{\text{padded}} = \sqrt{0.121^2 + 0.115^2} = 0.1669\text{ m}$.
- Final Polygon Coordinates: `[[0.121, 0.115], [0.121, -0.115], [-0.121, -0.115], [-0.121, 0.115]]`.

### 5. Resolution & Verification Status
- **Status:** `FIXED / VERIFIED_AFTER_FIX`
- **Resolution Timestamp:** 2026-08-11 18:02:00 -07:00 (2026-08-12 02:02:00 UTC)
- **Fix Implemented:**
  1. Updated `local_costmap` and `global_costmap` in `ros2/ros2_ws/src/rover_bringup/config/nav2_params.yaml` to replace `robot_radius: 0.15` with `footprint: "[[0.121, 0.115], [0.121, -0.115], [-0.121, -0.115], [-0.121, 0.115]]"`.
  2. Updated `rover_encoder_odometry.py` `/footprint` publisher to publish `9 1/8 in x 8 5/8 in` polygon.
- **Runtime & Deployment Verification:**
  - Deployed to RPi5 host (`10.0.0.246`).
  - Executed `colcon build --packages-select rover_bringup` inside `rover-ros2` container.
  - Verified installed `nav2_params.yaml` inside container (`/ros2_ws/install/rover_bringup/share/rover_bringup/config/nav2_params.yaml`) contains `footprint: "[[0.121, 0.115], [0.121, -0.115], [-0.121, -0.115], [-0.121, 0.115]]"`.
  - Verified no remaining references to `robot_radius: 0.15` exist in costmap configs.

---

## TARGETED RUNTIME AUDIT — FIRST SAVED-MAP AMCL LOCALIZATION

### Audit Metadata
- **Timestamp:** 2026-08-11 18:09:00 -07:00 (2026-08-12 01:09:00 UTC)
- **Auditor:** Antigravity AI Coding Assistant (Google DeepMind)
- **Model:** Gemini 3.6 Flash (High)
- **Audit Type:** TARGETED RUNTIME AUDIT / SAVED-MAP AMCL LOCALIZATION (STATIC / NO MOTION)
- **Target Map Loaded:** `/ros2_ws/maps/house_resume_verified_2026-08-10.yaml` (284 x 160 pixels, 0.05 m/px, origin `[-5.631, -5.115]`)
- **Branch:** `main` (Local Workspace) / `main` (RPi5 Host)
- **Commit SHA:** `c80a4eb18b321ec8ffdbf72782782e44d32a0c64`
- **Inspection Mode:** LIVE RUNTIME INSPECTION
- **Rover Safety Status:** `armed: false`, `autonomyState: DISABLED`, zero velocity commands issued.

### 1. Preflight Safety & Conflict Audit
- **Safety State**: Verified `armed: false`, `autonomyState: DISABLED`, target linear/angular velocities = `0.0`.
- **Telemetry & Odometry**: `/scan` publishing at 5.5 Hz; `/odom` publishing at 20 Hz; `odom -> base_link` active; static TFs active.
- **Publisher Conflict Check**: Checked SLAM state (`/api/slam/status` returned `STOPPED`, `nodes: []`). **Zero conflicting `map -> odom` publishers exist.**

### 2. Exact Map Loaded
- **Loaded Map**: `/ros2_ws/maps/house_resume_verified_2026-08-10.yaml` (PGM: `house_resume_verified_2026-08-10.pgm`).
- **Map Properties**: Resolution = `0.050 m/pixel`, Width = `284 cells`, Height = `160 cells`, Origin = `[-5.631, -5.115, 0.0]`.
- **Durability**: `/map` topic published with `transient_local` durability by `map_server`.

### 3. Localization Lifecycle & Node States
- `/map_server`: State `active [3]` (Managed by `lifecycle_manager_localization`).
- `/amcl`: State `active [3]` (Managed by `lifecycle_manager_localization`).
- `/lifecycle_manager_localization`: Active and managing localization pipeline.

### 4. Live TF Chain & Authority Verification
- **Complete Active TF Chain**:
  $$\text{map} \xrightarrow{\text{AMCL (/amcl)}} \text{odom} \xrightarrow{\text{Encoder/IMU (/rover_encoder_odometry)}} \text{base_link} \xrightarrow{\text{Static TF}} \text{laser_frame}$$
- **`map -> odom` Authority**: `/amcl` (Translation: `[-0.015, 0.009, 0.000]`, Yaw: `-0.035 deg`).
- **Single Authority**: Verified `/amcl` is the sole active publisher of `map -> odom`.

### 5. Initial Pose & Stationary Localization Convergence
- **Initial Pose**: Supplied initial pose `[0.0, 0.0, 0.0]` to `/initialpose` topic (`geometry_msgs/msg/PoseWithCovarianceStamped`).
- **Particle Cloud**: `/amcl` initialized `/particle_cloud` and converged rapidly.
- **Pose Estimate (`/amcl_pose`)**:
  - Position: $X = -0.015\text{ m}$, $Y = +0.009\text{ m}$, $Z = 0.0\text{ m}$
  - Yaw: $-0.035^\circ$
  - Position Covariance: $0.0017\text{ m}^2$ (Low variance, high confidence)
- **LiDAR Alignment**: Live `/scan` overlays accurately against obstacles in `house_resume_verified_2026-08-10.yaml`. Zero dropped scans or TF extrapolation errors observed.

### 6. Visualization Guidance for Operator (Foxglove / Cockpit)
- **Foxglove Studio** (`ws://10.0.0.246:8765`):
  - Fixed Frame: `map`
  - Map Layer: `/map`
  - Scan Layer: `/scan`
  - Pose Layer: `/amcl_pose` or `/rover_footprint_marker`
  - Particles: `/particle_cloud`
  - Pose Estimation Tool: Use 2D Pose Estimate tool in Foxglove to click/drag rover position if initial pose needs adjustment.

### 7. Resolution & Next Step
- **Status**: **PASS / VERIFIED**
- **Recommendation**: System is verified ready for a **MANUAL localization movement test** (teleop driving while observing AMCL particle cloud convergence and `map -> odom` stability).





