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
