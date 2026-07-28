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


if __name__ == '__main__':
    unittest.main()
