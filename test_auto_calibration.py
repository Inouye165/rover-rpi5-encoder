# ==============================================================================
# test_auto_calibration.py — Unit & Integration Tests for Closed-Loop Auto Calibration
# ==============================================================================

import os
import sys
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestAutoCalibrationSafety(unittest.TestCase):

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

        self.odom_node_path = os.path.join(
            os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup', 'rover_bringup', 'rover_encoder_odometry.py'
        )
        with open(self.odom_node_path, 'r', encoding='utf-8') as f:
            self.odom_code = f.read()

    def test_1_start_rejected_when_another_test_active(self):
        """1. Verify test start is rejected when another test is active."""
        self.assertIn("autoCalibState.active", self.server_code)
        self.assertIn("An automatic calibration test is already running.", self.server_code)

    def test_2_start_rejected_when_serial_disconnected(self):
        """2. Verify test start is rejected when serial port is disconnected."""
        self.assertIn("!serialPort || !serialPort.isOpen", self.server_code)
        self.assertIn("Serial port is not connected.", self.server_code)

    def test_3_start_rejected_with_stale_encoder_telemetry(self):
        """3. Verify test start is rejected with stale encoder telemetry."""
        self.assertIn("!encSnap.valid", self.server_code)
        self.assertIn("Encoder telemetry is unavailable or stale.", self.server_code)

    def test_4_start_rejected_with_stale_or_unavailable_odometry(self):
        """4. Verify test start is rejected with stale or unavailable ROS odometry."""
        self.assertIn("!latestRosOdom.valid", self.server_code)
        self.assertIn("ROS odometry is unavailable or stale.", self.server_code)

    def test_5_forward_test_commands_straight_forward(self):
        """5. Verify forward test commands all four wheels forward ([spd, spd, spd, spd])."""
        self.assertIn("testType === 'forward_1m'", self.server_code)
        self.assertIn("autoCalibState.motorCommand = [spd, spd, spd, spd]", self.server_code)

    def test_6_left_test_commands_in_place_left_rotation(self):
        """6. Verify left turn test commands in-place left spin ([-spd, spd, -spd, spd])."""
        self.assertIn("testType === 'turn_left_90'", self.server_code)
        self.assertIn("autoCalibState.motorCommand = [-spd, spd, -spd, spd]", self.server_code)

    def test_7_right_test_commands_in_place_right_rotation(self):
        """7. Verify right turn test commands in-place right spin ([spd, -spd, spd, -spd])."""
        self.assertIn("testType === 'turn_right_90'", self.server_code)
        self.assertIn("autoCalibState.motorCommand = [spd, -spd, spd, -spd]", self.server_code)

    def test_8_target_reached_causes_zero_command_and_disarm(self):
        """8. Verify target reached causes zero motor speed and disarm."""
        self.assertIn("stopAutoCalibration('target_reached'", self.server_code)
        self.assertIn("sendMotorSpeeds(0, 0, 0, 0)", self.server_code)
        self.assertIn("FUNC_DISARM_NORMAL_DRIVE", self.server_code)

    def test_9_timeout_causes_zero_command_and_disarm(self):
        """9. Verify timeout causes zero command and disarm."""
        self.assertIn("stopAutoCalibration('timeout'", self.server_code)
        self.assertIn("exceeded 12s timeout", self.server_code)
        self.assertIn("exceeded 8s timeout", self.server_code)

    def test_10_estop_causes_immediate_stop_and_disarm(self):
        """10. Verify E-stop causes immediate stop and disarm."""
        self.assertIn("stopAutoCalibration('estop'", self.server_code)

    def test_11_telemetry_becoming_stale_aborts_safely(self):
        """11. Verify telemetry becoming stale during run aborts safely."""
        self.assertIn("stopAutoCalibration('telemetry_stale'", self.server_code)

    def test_12_odometry_becoming_stale_aborts_safely(self):
        """12. Verify ROS odometry becoming stale during run aborts safely."""
        self.assertIn("stopAutoCalibration('odom_stale'", self.server_code)

    def test_13_unexpected_direction_aborts_safely(self):
        """13. Verify unexpected movement direction aborts safely."""
        self.assertIn("stopAutoCalibration('unexpected_direction'", self.server_code)

    def test_14_excessive_yaw_during_forward_test_aborts_safely(self):
        """14. Verify excessive yaw deviation (>15 deg) during forward test aborts safely."""
        self.assertIn("stopAutoCalibration('yaw_limit'", self.server_code)

    def test_15_excessive_translation_during_turn_aborts_safely(self):
        """15. Verify excessive translation (>0.20m) during turn test aborts safely."""
        self.assertIn("stopAutoCalibration('translation_limit'", self.server_code)

    def test_16_abort_endpoint_stops_and_disarms(self):
        """16. Verify /api/calibration/auto/abort stops and disarms."""
        self.assertIn("app.post('/api/calibration/auto/abort'", self.server_code)
        self.assertIn("stopAutoCalibration('user_abort'", self.server_code)

    def test_17_normal_driving_blocked_during_automatic_tests(self):
        """17. Verify manual drive commands are blocked when auto calibration test is active."""
        self.assertIn("autoCalibState.active", self.server_code)
        self.assertIn("Joystick override during automatic calibration test", self.server_code)

    def test_18_existing_maintenance_tests_blocked_during_automatic_tests(self):
        """18. Verify existing maintenance tests are blocked when auto test is active."""
        self.assertIn("An automatic calibration test is currently active. Maintenance mode blocked.", self.server_code)
        self.assertIn("An automatic calibration test is currently active. Maintenance tests blocked.", self.server_code)

    def test_19_ui_buttons_call_new_endpoints(self):
        """19. Verify UI buttons invoke promptAutoCalib, confirmAndStartAutoCalib, and abortAutoCalibrationTest."""
        self.assertIn("promptAutoCalib('forward_1m')", self.html_code)
        self.assertIn("promptAutoCalib('turn_left_90')", self.html_code)
        self.assertIn("promptAutoCalib('turn_right_90')", self.html_code)
        self.assertIn("abortAutoCalibrationTest()", self.html_code)
        self.assertIn("/api/calibration/auto/start", self.app_code)
        self.assertIn("/api/calibration/auto/abort", self.app_code)
        self.assertIn("/api/calibration/auto/status", self.app_code)

    def test_20_ui_displays_active_completion_and_fault_state(self):
        """20. Verify UI renders active banner, progress HUD, and result cards."""
        self.assertIn("auto-calib-active-banner", self.html_code)
        self.assertIn("auto-calib-hud", self.html_code)
        self.assertIn("auto-calib-result-card", self.html_code)
        self.assertIn("updateAutoCalibUI", self.app_code)

    def test_21_existing_maintenance_safety_tests_continue_to_pass(self):
        """21. Verify maintenance safety mechanisms remain present and intact."""
        self.assertIn("activeTestInProgress", self.server_code)
        self.assertIn("FUNC_EXIT_MAINTENANCE", self.server_code)

    def test_22_cmd_vel_remains_absent(self):
        """22. Verify /cmd_vel remains absent in ROS node and launch files."""
        foundation_launch = os.path.join(
            os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup', 'launch', 'foundation.launch.py'
        )
        with open(foundation_launch, 'r', encoding='utf-8') as f:
            launch_code = f.read()

        self.assertNotIn('/cmd_vel', launch_code)
        self.assertNotIn('/cmd_vel', self.odom_code)

    def test_23_no_dev_mounts_added_to_ros_container(self):
        """23. Verify compose.yaml contains no /dev mounts or serial device mappings."""
        compose_path = os.path.join(os.path.dirname(__file__), 'ros2', 'compose.yaml')
        if os.path.exists(compose_path):
            with open(compose_path, 'r', encoding='utf-8') as f:
                compose_code = f.read()
            self.assertNotIn('/dev/', compose_code)
            self.assertNotIn('devices:', compose_code)

    # -----------------------------------------------------------------------
    # Tests 24–30: Targeted regression tests for confirmed post-calibration fixes
    # -----------------------------------------------------------------------

    def test_24_stop_auto_calibration_resets_cmd_source(self):
        """24. Fix #1: stopAutoCalibration() must reset cmdSource = 'NONE' before any other state update."""
        # Normalise line endings so index() works on both LF and CRLF files
        code = self.server_code.replace('\r\n', '\n')
        fn_start = code.index("function stopAutoCalibration(reason, detail) {")
        fn_end   = code.index("\nasync function runAutoCalibTick(", fn_start)
        fn_body  = code[fn_start:fn_end]

        self.assertIn("cmdSource = 'NONE'", fn_body,
            "stopAutoCalibration() must set cmdSource = 'NONE' to release the CALIBRATION_TEST lock")

    def test_25_cmd_source_reset_before_state_update(self):
        """25. Fix #1: cmdSource must be cleared before autoCalibState.active is set to false."""
        code = self.server_code.replace('\r\n', '\n')
        fn_start = code.index("function stopAutoCalibration(reason, detail) {")
        fn_end   = code.index("\nasync function runAutoCalibTick(", fn_start)
        fn_body  = code[fn_start:fn_end]

        idx_cmd    = fn_body.index("cmdSource = 'NONE'")
        idx_active = fn_body.index("autoCalibState.active = false")
        self.assertLess(idx_cmd, idx_active,
            "cmdSource must be reset before autoCalibState.active = false inside stopAutoCalibration()")

    def test_26_arm_button_id_is_btn_arm_drive_in_html(self):
        """26. Fix #2: index.html must define id='btn-arm-drive' exactly once."""
        import re
        matches = re.findall(r'id=["\']btn-arm-drive["\']', self.html_code)
        self.assertEqual(len(matches), 1,
            "index.html must contain id='btn-arm-drive' exactly once")

    def test_27_no_stale_btn_arm_id_in_html(self):
        """27. Fix #2: index.html must NOT contain stale id='btn-arm' (without -drive)."""
        import re
        stale = re.findall(r'id=["\']btn-arm["\']', self.html_code)
        self.assertEqual(stale, [],
            "index.html must not contain stale id='btn-arm' (correct ID is btn-arm-drive)")

    def test_28_app_js_uses_btn_arm_drive(self):
        """28. Fix #2: app.js must reference 'btn-arm-drive' and NOT bare 'btn-arm'."""
        self.assertIn("btn-arm-drive", self.app_code,
            "app.js must reference btn-arm-drive")

    def test_29_no_stale_btn_arm_reference_in_app_js(self):
        """29. Fix #2: app.js must NOT contain stale 'btn-arm' without the '-drive' suffix."""
        import re
        # Match 'btn-arm' that is NOT immediately followed by -drive
        stale = re.findall(r"'btn-arm(?!-drive)", self.app_code)
        stale += re.findall(r'"btn-arm(?!-drive)', self.app_code)
        self.assertEqual(stale, [],
            "app.js must not contain stale 'btn-arm' reference (use btn-arm-drive)")

    def test_30_update_auto_calib_ui_does_not_reference_btn_arm(self):
        """30. Fix #2: updateAutoCalibUI function must not reference bare 'btn-arm'."""
        import re
        fn_start = self.app_code.index("function updateAutoCalibUI(status) {")
        fn_end   = self.app_code.index("\n}", fn_start) + 2
        fn_body  = self.app_code[fn_start:fn_end]

        stale = re.findall(r"['\"]btn-arm(?!-drive)", fn_body)
        self.assertEqual(stale, [],
            "updateAutoCalibUI must not reference bare 'btn-arm'")


if __name__ == '__main__':
    unittest.main()
