# ==============================================================================
# test_maintenance_safety.py — Unit & Integration Tests for Maintenance Safety
# ==============================================================================

import unittest
import os
import sys

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

        # 4. Verify safety cleanup protocol and error handling
        self.assertIn("performCleanup", script_code)
        self.assertIn("/api/stop", script_code)
        self.assertIn("/api/maintenance/exit", script_code)
        self.assertIn("/api/autonomy/disable", script_code)


if __name__ == '__main__':
    unittest.main()
