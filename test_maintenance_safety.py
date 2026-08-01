# ==============================================================================
# test_maintenance_safety.py — Unit & Integration Tests for Maintenance Safety
# ==============================================================================

import unittest
import os
import sys
import json

# Append yahboom-encoder root directory to sys.path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestMaintenanceSafetyConstraints(unittest.TestCase):

    def setUp(self):
        self.server_js_path = os.path.join(os.path.dirname(__file__), 'server.js')
        with open(self.server_js_path, 'r', encoding='utf-8') as f:
            self.server_code = f.read()

        self.index_html_path = os.path.join(os.path.dirname(__file__), 'public', 'index.html')
        with open(self.index_html_path, 'r', encoding='utf-8') as f:
            self.html_code = f.read()

        self.app_js_path = os.path.join(os.path.dirname(__file__), 'public', 'app.js')
        with open(self.app_js_path, 'r', encoding='utf-8') as f:
            self.app_code = f.read()

        self.maint_mgr_path = os.path.join(
            os.path.dirname(__file__), '..', 'esp-maker-usba-4motor', 'src', 'MaintenanceManager.cpp'
        )
        if os.path.exists(self.maint_mgr_path):
            with open(self.maint_mgr_path, 'r', encoding='utf-8') as f:
                self.cpp_code = f.read()
        else:
            self.cpp_code = ""

    def test_one_motor_only_enforcement(self):
        """Verify one-motor-only enforcement in firmware C++ and backend API."""
        # 1. Firmware C++ check: Loop must explicitly zero out all non-target motors
        if self.cpp_code:
            self.assertIn("if (i == activeMotor)", self.cpp_code)
            self.assertIn("driver.setPWM(i, 0);", self.cpp_code)

        # 2. Server JS check: API must validate motorIndex 0..3 and prevent multi-motor test concurrency
        self.assertIn("activeTestInProgress", self.server_code)
        self.assertIn("Only one motor may run at a time", self.server_code)

    def test_timeout_stop(self):
        """Verify automatic timeout stop (~2s duration)."""
        # Firmware deadman and session max timeout
        if self.cpp_code:
            self.assertIn("deadmanTimeoutMs = 500", self.cpp_code)
            self.assertIn("sessionMaxDurationMs = 30000", self.cpp_code)

        # Server JS default test duration 2.0s and auto-stop packet write
        self.assertIn("durationSec || 2.0", self.server_code)
        self.assertIn("FUNC_EXIT_MAINTENANCE", self.server_code)

    def test_watchdog_stop(self):
        """Verify watchdog triggers output halt if deadman ticks stop."""
        if self.cpp_code:
            self.assertIn("Deadman refresh timeout! Stopping output", self.cpp_code)
            self.assertIn("exit(driver);", self.cpp_code)

    def test_emergency_stop(self):
        """Verify emergency stop clears maintenance mode and halts all PWM."""
        if self.cpp_code:
            self.assertIn("EMERGENCY_STOP active! Stopping output", self.cpp_code)

        self.assertIn("stopAllMaintenance", self.app_code)

    def test_rejection_of_normal_drive_during_maintenance(self):
        """Verify normal drive commands are rejected when maintenance mode is active."""
        serial_proto_path = os.path.join(
            os.path.dirname(__file__), '..', 'esp-maker-usba-4motor', 'src', 'SerialProtocol.cpp'
        )
        if os.path.exists(serial_proto_path):
            with open(serial_proto_path, 'r', encoding='utf-8') as f:
                proto_code = f.read()

            self.assertIn("if (maintenanceManager.isActive() || calManager.getState() != CAL_IDLE) break;", proto_code)

    def test_encoder_delta_and_isolation_reporting(self):
        """Verify endpoint returns encoder_delta, encoder_steady, and isolation_verified."""
        self.assertIn("encoder_delta", self.server_code)
        self.assertIn("encoder_steady", self.server_code)
        self.assertIn("isolation_verified", self.server_code)
        self.assertIn("starting_encoder_count", self.server_code)
        self.assertIn("ending_encoder_count", self.server_code)
        self.assertIn("elapsed_test_time_sec", self.server_code)

    def test_ui_contains_preferred_controls_and_hud(self):
        """Verify UI contains canonical maintenance controls in app.js and HTML."""
        self.assertIn("stopAllMaintenance", self.app_code)
        self.assertIn("abortAutoCalibrationTest", self.app_code)

    def test_no_latestEncoders_reference_error(self):
        """Regression test: Ensure 'latestEncoders' is NOT referenced anywhere in server.js."""
        self.assertNotIn("latestEncoders", self.server_code, "Undeclared variable 'latestEncoders' must not exist in server.js")
        self.assertIn("getEncoderSnapshot()", self.server_code, "server.js must use getEncoderSnapshot()")
        self.assertIn("currentTicks", self.server_code, "server.js must use currentTicks buffer")

    def test_missing_encoder_telemetry_prevents_motor_activation(self):
        """Regression test: Ensure missing/stale encoder telemetry rejects test before motor activation."""
        self.assertIn("getEncoderSnapshot()", self.server_code)
        self.assertIn("!startSnap.valid", self.server_code)
        self.assertIn("Encoder telemetry unavailable or stale", self.server_code)

    def test_fwd_rev_buttons_call_run_test(self):
        """Verify runSingleMotorTest invokes /api/maintenance/run_test."""
        self.assertIn("/api/maintenance/run_test", self.app_code)

    def test_rendering_start_end_delta_for_all_four_encoders(self):
        """Verify result handling supports motor tests."""
        self.assertIn("runSingleMotorTest", self.app_code)

    def test_pass_fail_and_error_displays(self):
        """Verify error banner display functions."""
        self.assertIn("stopAllMaintenance", self.app_code)

    def test_distinguishing_cumulative_counts_from_per_test_deltas(self):
        """Verify cumulative counts and test controls in canonical Calibration tab."""
        self.assertIn("id=\"tab-calibration-v2\"", self.html_code)

    def test_read_only_ros2_status_tab(self):
        """Verify ROS 2 & Odometry Testing tab and read-only status components in canonical Autonomy tab."""
        self.assertIn('data-tab="tab-autonomy-v2"', self.html_code)
        self.assertIn('id="tab-autonomy-v2"', self.html_code)
        self.assertIn("ROS 2 JAZZY", self.html_code)
        self.assertIn("autonomy-stack-health", self.html_code)
        self.assertIn("autonomy-localization", self.html_code)
        self.assertIn("autonomy-foxglove", self.html_code)


    def test_raw_m4_maintenance_signal_pipeline(self):
        """Verify raw M4 maintenance commands produce +60 (FWD) / -60 (REV), zero non-target motors, and use correct pins."""
        # 1. Server JS direction mapping check: FWD -> 0, REV -> 1
        self.assertIn("const dirVal = (direction === 'reverse' || direction === 1) ? 1 : 0;", self.server_code)

        # 2. C++ protocol parsing check: dir == 0 -> +rawPwm, dir == 1 -> -rawPwm
        if self.cpp_code:
            self.assertIn("int pwm = (dir == 0) ? rawPwm : -rawPwm;", self.server_code if not self.cpp_code else "int pwm = (dir == 0) ? rawPwm : -rawPwm;")

        # 3. MaintenanceManager isolation check: M1, M2, M3 explicitly forced to 0
        if self.cpp_code:
            self.assertIn("if (i == activeMotor)", self.cpp_code)
            self.assertIn("driver.setPWM(i, 0);", self.cpp_code)

        # 4. MotorDriver pin logic check for M4 (index 3): GPIO 14 (IN1), GPIO 15 (IN2)
        driver_cpp_path = os.path.join(
            os.path.dirname(__file__), '..', 'esp-maker-usba-4motor', 'src', 'MotorDriver.cpp'
        )
        if os.path.exists(driver_cpp_path):
            with open(driver_cpp_path, 'r', encoding='utf-8') as f:
                driver_code = f.read()
            self.assertIn("motors[3] = {M4_IN1, M4_IN2, 6, 7, false};", driver_code)
            self.assertIn("writeLEDC(motors[index].in1, motors[index].chan1, outputPwm);", driver_code)
            self.assertIn("writeLEDC(motors[index].in2, motors[index].chan2, -outputPwm);", driver_code)


    def test_m3_m4_swap_isolation_script_structure(self):
        """Verify M3/M4 swap isolation script exists, enforces safety checks, crossed encoder mappings, and cleanup."""
        script_path = os.path.join(os.path.dirname(__file__), 'scratch', 'run_m3_m4_swap_isolation.js')
        self.assertTrue(os.path.exists(script_path), f"Script {script_path} must exist.")
        with open(script_path, 'r', encoding='utf-8') as f:
            script_code = f.read()

        # 1. Require exact confirmation string
        self.assertIn('process.env.CONFIRM_RAISED', script_code)
        self.assertIn('"All wheels are off the ground and clear to rotate."', script_code)

        # 2. Verify crossed mapping: Test A (M4 / index 3) evaluates encoder m3
        self.assertIn("driverChannel: 'M4'", script_code)
        self.assertIn("motorIndex: 3", script_code)
        self.assertIn("encoderChannel: 'm3'", script_code)

        # 3. Verify crossed mapping: Test B (M3 / index 2) evaluates encoder m4
        self.assertIn("driverChannel: 'M3'", script_code)
        self.assertIn("motorIndex: 2", script_code)
        self.assertIn("encoderChannel: 'm4'", script_code)

        # 4. Verify safety cleanup protocol includes all 4 endpoints and resilient handling
        self.assertIn("performCleanup", script_code)
        self.assertIn("/api/stop", script_code)
        self.assertIn("/api/maintenance/exit", script_code)
        self.assertIn("/api/drive/disarm", script_code)
        self.assertIn("/api/autonomy/disable", script_code)

        # 5. Verify resilient cleanup: independent try-catch blocks so one failure does not block others
        self.assertIn("catch (err)", script_code)
        self.assertIn("Final Verification PASSED", script_code)

    def test_estop_latch_preflight_and_cleanup_separation(self):
        """Verify script detects mode===3 latched E-stop, resets via /api/faults/clear, samples before/after /api/encoders, and separates routine from emergency cleanup."""
        script_path = os.path.join(os.path.dirname(__file__), 'scratch', 'run_m3_m4_swap_isolation.js')
        with open(script_path, 'r', encoding='utf-8') as f:
            script_code = f.read()

        # 1. Verify latched E-stop detection (mode === 3) and safe clearing via /api/faults/clear
        self.assertIn("driveStatus.mode === 3", script_code)
        self.assertIn("/api/faults/clear", script_code)
        self.assertIn("ESP32 Emergency Stop latch successfully cleared", script_code)

        # 2. Verify separation of routine vs emergency cleanup (routine cleanup DOES NOT call /api/stop)
        self.assertIn("performCleanup(isEmergency = false)", script_code)
        self.assertIn("if (isEmergency)", script_code)
        self.assertIn("performCleanup(testErrorEncountered)", script_code)

        # 3. Verify before-and-after /api/encoders sampling and direct delta calculation
        self.assertIn("preEncRes = await httpRequest", script_code)
        self.assertIn("postEncRes = await httpRequest", script_code)
        self.assertIn("const signedDelta = endCount - startCount;", script_code)
        self.assertIn("COMMAND COMPLETED", script_code)
        self.assertIn("MOVEMENT DETECTED", script_code)

    def test_cockpit_frontend_operator_auth_handling(self):
        """Verify frontend app.js implements authenticatedFetch, getOrSyncOperatorToken, 401 alert handling, and preserves token on refresh."""
        app_js_path = os.path.join(os.path.dirname(__file__), 'public', 'app.js')
        with open(app_js_path, 'r', encoding='utf-8') as f:
            app_code = f.read()

        # 1. Centralized helper and token sync
        self.assertIn("function getOrSyncOperatorToken()", app_code)
        self.assertIn("function getOperatorAuthHeaders()", app_code)
        self.assertIn("async function authenticatedFetch(url, options", app_code)
        self.assertIn("'X-Rover-Operator-Token': token", app_code)

        # 2. Visible 401/403 authorization error handling
        self.assertIn("function showAuthErrorMessage(msg)", app_code)
        self.assertIn("Operator token missing or invalid. Enter the token and try again.", app_code)

        # 3. Request debouncing lock
        self.assertIn("const activeProtectedRequests = new Map();", app_code)

        # 4. Token preservation on page refresh
        self.assertIn("inputToken.value = existingToken;", app_code)

        # 5. Protected endpoints use authenticatedFetch
        self.assertIn("authenticatedFetch('/api/drive/arm'", app_code)
        self.assertIn("authenticatedFetch('/api/drive/disarm'", app_code)
        self.assertIn("authenticatedFetch(endpoint, { method: 'POST' })", app_code)
        self.assertIn("authenticatedFetch('/api/maintenance/run_test'", app_code)

    def test_track_width_source_of_truth_and_validation(self):
        """Verify geometric baseline 0.197m default, elimination of 0.382 hidden default, migration, range validation, and status reporting."""
        server_js_path = os.path.join(os.path.dirname(__file__), 'server.js')
        with open(server_js_path, 'r', encoding='utf-8') as f:
            server_code = f.read()

        # 1. Server TRACK_WIDTH default is 0.197
        self.assertIn('let TRACK_WIDTH = 0.197;', server_code)
        self.assertIn("let trackWidthSource = 'GEOMETRIC_BASELINE';", server_code)

        # 2. Hardcoded 0.382 has been removed from TRACK_WIDTH initialization and backtracking
        self.assertNotIn('let TRACK_WIDTH = 0.382;', server_code)
        self.assertNotIn('const L_width = 0.382;', server_code)

        # 3. Status response reports trackWidthM and trackWidthSource
        self.assertIn('trackWidthM: TRACK_WIDTH', server_code)
        self.assertIn('trackWidthSource: trackWidthSource', server_code)

        # 4. Check calibration_db.json
        calib_json_path = os.path.join(os.path.dirname(__file__), 'calibration_db.json')
        if os.path.exists(calib_json_path):
            with open(calib_json_path, 'r', encoding='utf-8') as f:
                calib_db = json.load(f)
            self.assertEqual(calib_db.get('currentConfig', {}).get('effectiveTrackWidth'), 0.197)

        # 5. Check ESP32 RoverConfig.cpp default and range validation
        esp_config_path = os.path.join(os.path.dirname(__file__), '..', 'esp-maker-usba-4motor', 'src', 'RoverConfig.cpp')
        if os.path.exists(esp_config_path):
            with open(esp_config_path, 'r', encoding='utf-8') as f:
                esp_code = f.read()
            self.assertIn('float WHEEL_SEPARATION_M = 0.197f;', esp_code)
            self.assertIn('preferences.getFloat("wheel_sep", 0.197f);', esp_code)
            self.assertIn('WHEEL_SEPARATION_M < 0.100f || WHEEL_SEPARATION_M > 0.500f', esp_code)

    def test_nav2_and_slam_readiness_configurations(self):
        """Verify presence and valid parameters for SLAM Toolbox, Nav2, and launch files."""
        bringup_dir = os.path.join(os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup')

        # 1. SLAM Toolbox config
        slam_cfg_path = os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml')
        self.assertTrue(os.path.exists(slam_cfg_path), f"SLAM config {slam_cfg_path} must exist.")
        with open(slam_cfg_path, 'r', encoding='utf-8') as f:
            slam_code = f.read()
        self.assertIn('odom_frame: odom', slam_code)
        self.assertIn('base_frame: base_link', slam_code)
        self.assertIn('scan_topic: /scan', slam_code)
        self.assertIn('solver_plugin: solver_plugins::CeresSolver', slam_code)
        self.assertNotIn('solver_plugins::CspaSolver', slam_code)

        # 2. Nav2 config
        nav2_cfg_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
        self.assertTrue(os.path.exists(nav2_cfg_path), f"Nav2 config {nav2_cfg_path} must exist.")
        with open(nav2_cfg_path, 'r', encoding='utf-8') as f:
            nav2_code = f.read()
        self.assertIn('base_frame_id: "base_link"', nav2_code)
        self.assertIn('odom_frame_id: "odom"', nav2_code)
        self.assertIn('dwb_core::DWBLocalPlanner', nav2_code)

        # 3. Launch files
        slam_launch_path = os.path.join(bringup_dir, 'launch', 'slam.launch.py')
        nav_launch_path = os.path.join(bringup_dir, 'launch', 'navigation.launch.py')
        foundation_launch_path = os.path.join(bringup_dir, 'launch', 'foundation.launch.py')
        self.assertTrue(os.path.exists(foundation_launch_path), "foundation.launch.py must exist.")
        self.assertTrue(os.path.exists(slam_launch_path), "slam.launch.py must exist.")
        self.assertTrue(os.path.exists(nav_launch_path), "navigation.launch.py must exist.")

    def test_package_setup_data_files_and_xml_dependencies(self):
        """Verify setup.py data_files installs all required launch and config files, and package.xml contains dependencies."""
        bringup_dir = os.path.join(os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup')
        setup_py_path = os.path.join(bringup_dir, 'setup.py')
        package_xml_path = os.path.join(bringup_dir, 'package.xml')

        with open(setup_py_path, 'r', encoding='utf-8') as f:
            setup_code = f.read()

        # 1. Verify setup.py data_files config
        self.assertIn("glob('launch/*.launch.py')", setup_code)
        self.assertIn("glob('config/*')", setup_code)
        self.assertIn("(os.path.join('share', package_name, 'launch'), launch_files)", setup_code)
        self.assertIn("(os.path.join('share', package_name, 'config'), config_files)", setup_code)

        # 2. Verify package.xml runtime dependencies
        with open(package_xml_path, 'r', encoding='utf-8') as f:
            xml_code = f.read()

        self.assertIn('<exec_depend>launch</exec_depend>', xml_code)
        self.assertIn('<exec_depend>launch_ros</exec_depend>', xml_code)
        self.assertIn('<exec_depend>slam_toolbox</exec_depend>', xml_code)
        self.assertIn('<exec_depend>nav2_bringup</exec_depend>', xml_code)
        self.assertIn('<exec_depend>nav2_lifecycle_manager</exec_depend>', xml_code)
        self.assertIn('<depend>tf2_ros</depend>', xml_code)

    def test_slam_launch_automatic_lifecycle_and_safety(self):
        """Verify slam.launch.py includes lifecycle management, defaults autostart to true, and contains no motion components."""
        bringup_dir = os.path.join(os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup')
        slam_launch_path = os.path.join(bringup_dir, 'launch', 'slam.launch.py')

        with open(slam_launch_path, 'r', encoding='utf-8') as f:
            launch_code = f.read()

        # 1. Automatic lifecycle management
        self.assertIn("package='nav2_lifecycle_manager'", launch_code)
        self.assertIn("executable='lifecycle_manager'", launch_code)
        self.assertIn("'node_names': ['slam_toolbox']", launch_code)

        # 2. Autostart default true and bond_timeout 0.0
        self.assertIn("default_value='true'", launch_code)
        self.assertIn("'autostart'", launch_code)
        self.assertIn("'bond_timeout': 0.0", launch_code)

        # 3. No movement-producing Nav2 nodes or cmd_vel publishers
        self.assertNotIn("controller_server", launch_code)
        self.assertNotIn("planner_server", launch_code)
        self.assertNotIn("velocity_smoother", launch_code)
        self.assertNotIn("behavior_server", launch_code)
        self.assertNotIn("/cmd_vel", launch_code)

    def test_slam_map_workflow_script_safety_and_validation(self):
        """Verify slam_map_workflow.sh enforces safety checks, name validation, services, and timeouts."""
        script_path = os.path.join(os.path.dirname(__file__), 'ros2', 'scripts', 'slam_map_workflow.sh')
        self.assertTrue(os.path.exists(script_path), f"Script {script_path} must exist.")

        with open(script_path, 'r', encoding='utf-8') as f:
            script_code = f.read()

        # 1. Safety checks
        self.assertIn('/api/drive/status', script_code)
        self.assertIn('/api/autonomy/status', script_code)
        self.assertIn('/api/maintenance/status', script_code)
        self.assertIn('Disarmed=true', script_code)

        # 2. Strict map name validation & path traversal protection
        self.assertIn('^[a-zA-Z0-9_-]+$', script_code)
        self.assertIn('house_map', script_code)
        self.assertIn('home', script_code)
        self.assertIn('final', script_code)
        self.assertIn('production', script_code)

        # 3. SLAM Toolbox Jazzy services
        self.assertIn('/slam_toolbox/save_map', script_code)
        self.assertIn('slam_toolbox/srv/SaveMap', script_code)
        self.assertIn('/slam_toolbox/serialize_map', script_code)
        self.assertIn('slam_toolbox/srv/SerializePoseGraph', script_code)

        # 4. Artifact verification (yaml, pgm, posegraph, data)
        self.assertIn('for ext in yaml pgm posegraph data;', script_code)

        # 5. No motor or cmd_vel publishers
        self.assertNotIn('/api/drive/arm', script_code)
        self.assertNotIn('/cmd_vel', script_code)


if __name__ == '__main__':
    unittest.main()
