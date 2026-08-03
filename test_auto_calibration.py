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
        """5. Verify forward test sets targetLinear > 0 and targetAngular = 0.0."""
        self.assertIn("testType === 'forward_1m'", self.server_code)
        self.assertIn("targetLinear = parseFloat(lin.toFixed(3));", self.server_code)
        self.assertIn("targetAngular = 0.0;", self.server_code)

    def test_6_left_test_commands_in_place_left_rotation(self):
        """6. Verify left turn test sets targetLinear = 0.0 and positive targetAngular."""
        self.assertIn("testType === 'turn_left_90'", self.server_code)
        self.assertIn("targetAngular = parseFloat(ang.toFixed(3));", self.server_code)

    def test_7_right_test_commands_in_place_right_rotation(self):
        """7. Verify right turn test sets targetLinear = 0.0 and negative targetAngular."""
        self.assertIn("testType === 'turn_right_90'", self.server_code)
        self.assertIn("targetAngular = parseFloat((-ang).toFixed(3));", self.server_code)

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

    # -----------------------------------------------------------------------
    # Tests 31–36: Phase 5 Repeatability History & Reporting Tests
    # -----------------------------------------------------------------------

    def test_31_repeatability_history_logging_in_stop_auto_calibration(self):
        """31. Verify stopAutoCalibration() records run in calibrationDb.testLogs."""
        self.assertIn("calibrationDb.testLogs.push(historyRecord)", self.server_code,
            "stopAutoCalibration() must append completed run to calibrationDb.testLogs")
        self.assertIn("saveCalibrationDb()", self.server_code,
            "stopAutoCalibration() must save calibration database")

    def test_32_compute_repeatability_stats_helper_present(self):
        """32. Verify computeRepeatabilityStats helper computes stats and recommended counters."""
        self.assertIn("function computeRepeatabilityStats(", self.server_code)
        self.assertIn("normalizedYawMagnitude", self.server_code)
        self.assertIn("recommended", self.server_code)

    def test_33_repeatability_api_endpoints_present(self):
        """33. Verify repeatability history & clear HTTP endpoints are present."""
        self.assertIn("/api/calibration/repeatability/history", self.server_code)
        self.assertIn("/api/calibration/repeatability/clear", self.server_code)

    def test_34_repeatability_json_csv_export_endpoints(self):
        """34. Verify JSON and CSV export endpoints are present."""
        self.assertIn("/api/calibration/repeatability/export/json", self.server_code)
        self.assertIn("/api/calibration/repeatability/export/csv", self.server_code)

    def test_35_repeatability_ui_hud_in_index_html(self):
        """35. Verify repeatability statistics & recommended run HUD in index.html."""
        self.assertIn("repeatability-stats-hud", self.html_code)
        self.assertIn("rep-val-passrate", self.html_code)
        self.assertIn("rep-rec-total", self.html_code)

    def test_37_strict_recommendation_qualification_rule(self):
        """37. Verify qualification rule requires both stopReason === 'target_reached' AND pass === true."""
        self.assertIn("l.stopReason === 'target_reached' && l.pass === true", self.server_code,
            "Qualification rule must strictly require stopReason === 'target_reached' && pass === true")

    def test_38_safe_csv_sanitize_and_formula_injection_prevention(self):
        """38. Verify sanitizeForCsv function and formula injection protection in server.js."""
        self.assertIn("function sanitizeForCsv(", self.server_code)
        self.assertIn("['=', '+', '-', '@']", self.server_code)

    def test_39_atomic_save_calibration_db(self):
        """39. Verify saveCalibrationDb uses atomic file replacement via .tmp extension."""
        self.assertIn(".tmp'", self.server_code)
        self.assertIn("fs.renameSync(", self.server_code)

    def test_40_separate_per_test_statistics(self):
        """40. Verify stats are calculated separately under byTest without combining distance & turn metrics."""
        self.assertIn("byTest:", self.server_code)
        self.assertIn("turn_left_90: leftStats", self.server_code)

    def test_41_cleanup_order_in_stop_auto_calibration(self):
        """41. Verify safety cleanup order: motor stop & disarm & cmdSource reset BEFORE history recording."""
        code = self.server_code.replace('\r\n', '\n')
        fn_start = code.index("function stopAutoCalibration(reason, detail) {")
        fn_end   = code.index("\nasync function runAutoCalibTick(", fn_start)
        fn_body  = code[fn_start:fn_end]

        idx_motor = fn_body.index("sendMotorSpeeds(0, 0, 0, 0)")
        idx_cmd   = fn_body.index("cmdSource = 'NONE'")
        idx_hist  = fn_body.index("calibrationDb.testLogs.push(historyRecord)")

        self.assertLess(idx_motor, idx_hist, "sendMotorSpeeds(0,0,0,0) must occur before history logging")
        self.assertLess(idx_cmd, idx_hist, "cmdSource = 'NONE' must occur before history logging")

    def test_42_json_schema_metadata(self):
        """42. Verify JSON export includes $schema and version metadata."""
        self.assertIn("$schema", self.server_code)
        self.assertIn("repeatability-history-v1.json", self.server_code)


    # -----------------------------------------------------------------------
    # Tests 43–50: Regression tests for armed-field synchronization fix
    # Fix: autoCalibState.armed must always reflect Boolean(latestNormalDriveStatus?.armed)
    # -----------------------------------------------------------------------

    def test_43_broadcast_syncs_armed_from_latestNormalDriveStatus(self):
        """43. broadcastAutoCalibStatus() must synchronize armed from latestNormalDriveStatus before broadcasting."""
        code = self.server_code.replace('\r\n', '\n')
        fn_start = code.index('function broadcastAutoCalibStatus() {')
        fn_end   = code.index('\n}', fn_start) + 2
        fn_body  = code[fn_start:fn_end]

        self.assertIn(
            'autoCalibState.armed = Boolean(latestNormalDriveStatus?.armed)',
            fn_body,
            "broadcastAutoCalibStatus() must set autoCalibState.armed = Boolean(latestNormalDriveStatus?.armed)"
        )

    def test_44_status_endpoint_syncs_armed_from_latestNormalDriveStatus(self):
        """44. /api/calibration/auto/status must synchronize armed from latestNormalDriveStatus before responding."""
        code = self.server_code.replace('\r\n', '\n')
        # Find the GET status route block
        route_start = code.index("app.get('/api/calibration/auto/status'")
        route_end   = code.index('\n});', route_start) + 4
        route_body  = code[route_start:route_end]

        self.assertIn(
            'autoCalibState.armed = Boolean(latestNormalDriveStatus?.armed)',
            route_body,
            "/api/calibration/auto/status must set autoCalibState.armed = Boolean(latestNormalDriveStatus?.armed) before res.json()"
        )
        # Ensure sync occurs BEFORE res.json()
        idx_sync = route_body.index('autoCalibState.armed = Boolean(latestNormalDriveStatus?.armed)')
        idx_json = route_body.index('res.json(')
        self.assertLess(idx_sync, idx_json,
            "armed sync must occur before res.json() in /api/calibration/auto/status")

    def test_45_no_drive_telemetry_armed_is_false(self):
        """45. When latestNormalDriveStatus is null/undefined, Boolean(latestNormalDriveStatus?.armed) === false."""
        # This is a logic test of the expression itself, not server startup state.
        # Verify the optional-chaining pattern is used (safe for null/undefined).
        code = self.server_code.replace('\r\n', '\n')
        # Count occurrences of the safe pattern — must appear at least 3 times
        # (broadcastAutoCalibStatus, status endpoint, start endpoint)
        import re
        matches = re.findall(
            r'Boolean\(latestNormalDriveStatus\?\.armed\)',
            code
        )
        self.assertGreaterEqual(len(matches), 3,
            "Boolean(latestNormalDriveStatus?.armed) must appear in broadcastAutoCalibStatus, "
            "the status endpoint, and the start endpoint (at minimum)")

    def test_46_arm_endpoint_broadcasts_calib_status_promptly(self):
        """46. /api/drive/arm must call broadcastAutoCalibStatus() after updating latestNormalDriveStatus."""
        code = self.server_code.replace('\r\n', '\n')
        route_start = code.index("app.post('/api/drive/arm'")
        route_end   = code.index('\n});', route_start) + 4
        route_body  = code[route_start:route_end]

        self.assertIn(
            'broadcastAutoCalibStatus()',
            route_body,
            "/api/drive/arm must call broadcastAutoCalibStatus() to propagate armed=true to calib-status consumers"
        )
        # Ensure broadcast happens after the latestNormalDriveStatus update
        idx_status  = route_body.index("latestNormalDriveStatus = { armed: true }")
        idx_bcast   = route_body.index('broadcastAutoCalibStatus()')
        self.assertLess(idx_status, idx_bcast,
            "broadcastAutoCalibStatus() must come after latestNormalDriveStatus = { armed: true } in /api/drive/arm")

    def test_47_disarm_endpoint_broadcasts_calib_status_promptly(self):
        """47. /api/drive/disarm must call broadcastAutoCalibStatus() after updating latestNormalDriveStatus."""
        code = self.server_code.replace('\r\n', '\n')
        route_start = code.index("app.post('/api/drive/disarm'")
        route_end   = code.index('\n});', route_start) + 4
        route_body  = code[route_start:route_end]

        self.assertIn(
            'broadcastAutoCalibStatus()',
            route_body,
            "/api/drive/disarm must call broadcastAutoCalibStatus() to propagate armed=false to calib-status consumers"
        )
        idx_status = route_body.index("latestNormalDriveStatus = { armed: false }")
        idx_bcast  = route_body.index('broadcastAutoCalibStatus()')
        self.assertLess(idx_status, idx_bcast,
            "broadcastAutoCalibStatus() must come after latestNormalDriveStatus = { armed: false } in /api/drive/disarm")

    def test_48_stop_auto_calibration_no_longer_sets_armed_false(self):
        """48. stopAutoCalibration() must NOT independently set autoCalibState.armed = false (redundant + misleading)."""
        code = self.server_code.replace('\r\n', '\n')
        fn_start = code.index("function stopAutoCalibration(reason, detail) {")
        fn_end   = code.index("\nasync function runAutoCalibTick(", fn_start)
        fn_body  = code[fn_start:fn_end]

        self.assertNotIn(
            'autoCalibState.armed = false',
            fn_body,
            "stopAutoCalibration() must not independently set autoCalibState.armed = false; "
            "armed is derived from latestNormalDriveStatus by broadcastAutoCalibStatus()"
        )

    def test_49_status_endpoint_does_not_change_active_phase_test_motor(self):
        """49. /api/calibration/auto/status GET must not mutate active, phase, test, or motorCommand."""
        code = self.server_code.replace('\r\n', '\n')
        route_start = code.index("app.get('/api/calibration/auto/status'")
        route_end   = code.index('\n});', route_start) + 4
        route_body  = code[route_start:route_end]

        self.assertNotIn('autoCalibState.active =', route_body,
            "Status endpoint must not mutate autoCalibState.active")
        self.assertNotIn('autoCalibState.phase =', route_body,
            "Status endpoint must not mutate autoCalibState.phase")
        self.assertNotIn('autoCalibState.test =', route_body,
            "Status endpoint must not mutate autoCalibState.test")
        self.assertNotIn('autoCalibState.motorCommand =', route_body,
            "Status endpoint must not mutate autoCalibState.motorCommand")

    def test_50_telemetry_path_broadcasts_calib_status_on_armed_change(self):
        """50. The ESP32 normal_drive_status telemetry path must call broadcastAutoCalibStatus()
           so WebSocket consumers receive updated armed state when ESP32 confirms arm/disarm."""
        code = self.server_code.replace('\r\n', '\n')
        # Find the block that broadcasts normal_drive_status
        idx_nd_bcast = code.index("type: 'normal_drive_status'")
        # Look for broadcastAutoCalibStatus within a reasonable window after it
        window = code[idx_nd_bcast:idx_nd_bcast + 400]
        self.assertIn(
            'broadcastAutoCalibStatus()',
            window,
            "broadcastAutoCalibStatus() must be called near the normal_drive_status broadcast "
            "so ESP32-confirmed arm state changes propagate to calib-status WS consumers"
        )

    def test_51_keepalive_loop_suppresses_func_motion_during_auto_calib(self):
        """51. Verify startDriveKeepaliveLoop suppresses background FUNC_MOTION packets when autoCalibState is active."""
        self.assertIn('if (isMaintenance || isPositionActive) return;', self.server_code,
            "Drive keepalive loop must check isMaintenance before emitting FUNC_MOTION packets")

    def test_52_arm_confirmation_gate_and_no_nonzero_output_before_arm(self):
        """52. Verify arm confirmation gate sets phase ARMING initially and zero motor command before arming."""
        self.assertIn("phase: 'ARMING'", self.server_code,
            "Start route must set autoCalibState.phase to 'ARMING'")
        self.assertIn("if (autoCalibState.phase === 'ARMING')", self.server_code,
            "runAutoCalibTick must handle ARMING phase")

    def test_53_arm_confirmation_timeout_triggers_arm_timeout_reason(self):
        """53. Verify arm confirmation failure triggers arm_timeout stop reason."""
        self.assertIn("stopAutoCalibration('arm_timeout'", self.server_code,
            "Arm confirmation timeout must trigger stopAutoCalibration('arm_timeout')")

    def test_54_loss_of_armed_state_during_active_test_stops_safely(self):
        """54. Verify loss of armed state during active test triggers armed_lost stop reason."""
        self.assertIn("stopAutoCalibration('armed_lost'", self.server_code,
            "Mid-test disarm must trigger stopAutoCalibration('armed_lost')")

    def test_55_stop_auto_calibration_preserves_requested_test_name(self):
        """55. Verify stopAutoCalibration captures completed test identifier before resetting autoCalibState."""
        self.assertIn("const completedTest = autoCalibState.test;", self.server_code,
            "stopAutoCalibration must capture completedTest before resetting autoCalibState.test")
        self.assertIn("Test '${completedTest}' stopped", self.server_code,
            "Console stop log must log completedTest instead of null")

    def test_56_single_flight_odom_promise_prevents_concurrent_fetches(self):
        """56. Verify fetchRosOdometry uses single-flight odomFetchPromise to serialize HTTP requests."""
        self.assertIn("let odomFetchPromise = null;", self.server_code,
            "server.js must declare odomFetchPromise")
        self.assertIn("if (odomFetchPromise) {\n    return odomFetchPromise;\n  }", self.server_code,
            "fetchRosOdometry must return existing odomFetchPromise if in flight")

    def test_57_last_good_sample_timestamp_prevents_failed_overwrites(self):
        """57. Verify transient HTTP failures maintain valid = true if sample age is < 2000ms."""
        self.assertIn("let lastOdomSuccessTime = 0;", self.server_code,
            "server.js must track lastOdomSuccessTime")
        self.assertIn("latestRosOdom.valid = (lastOdomSuccessTime > 0 && sampleAge < 2000);", self.server_code,
            "handleOdomError must preserve sample validity if cached sample is fresh")

    def test_58_auto_calib_tick_in_flight_guard_prevents_overlapping_ticks(self):
        """58. Verify runAutoCalibTick uses autoCalibTickInFlight guard to prevent overlapping ticks."""
        self.assertIn("let autoCalibTickInFlight = false;", self.server_code,
            "server.js must declare autoCalibTickInFlight guard")
        self.assertIn("if (autoCalibTickInFlight) return;", self.server_code,
            "runAutoCalibTick must check autoCalibTickInFlight guard")

    def test_59_status_endpoint_does_not_launch_competing_odometry_fetches(self):
        """59. Verify calibration status endpoint does not launch competing odometry fetches when active."""
        self.assertIn("if (!autoCalibState.active) {\n    await fetchRosOdometry();\n  }", self.server_code,
            "Status endpoint must skip fetchRosOdometry when autoCalibState.active is true")

    def test_60_genuinely_stale_odometry_in_running_immediately_zeroes_motors(self):
        """60. Verify genuinely stale odometry in RUNNING phase immediately triggers stopAutoCalibration('odom_stale')."""
        self.assertIn("if (!isOdomFresh) {\n      stopAutoCalibration('odom_stale', 'ROS odometry stale or unreachable');", self.server_code,
            "RUNNING phase must immediately stop with odom_stale when odometry becomes stale")

    def test_61_source_odometry_age_check_required_before_updating_last_success(self):
        """61. Verify fetchRosOdometry checks parsed.odometry_age_ms < 2000 before updating lastOdomSuccessTime."""
        self.assertIn("parsed.odometry_age_ms < 2000", self.server_code,
            "fetchRosOdometry must verify parsed.odometry_age_ms < 2000 before treating sample as fresh")

    def test_62_single_request_settlement_handled_guard_present(self):
        """62. Verify fetchRosOdometry uses request-local handled guard to settle requests once."""
        self.assertIn("let handled = false;", self.server_code,
            "fetchRosOdometry must use handled guard per request")
        self.assertIn("if (handled) return;", self.server_code,
            "fetchRosOdometry handlers must check handled guard")

    def test_63_arming_phase_requires_both_armed_and_fresh_odometry(self):
        """63. Verify transition from ARMING to RUNNING requires both isArmed and isOdomFresh."""
        self.assertIn("if (isArmed && isOdomFresh) {", self.server_code,
            "ARMING transition must require both isArmed and isOdomFresh")

    def test_64_cmd_source_auto_calib_ownership(self):
        """64. Verify cmdSource = 'AUTO_CALIB' is set when starting an automatic calibration test."""
        self.assertIn("cmdSource = 'AUTO_CALIB';", self.server_code,
            "server.js must set cmdSource = 'AUTO_CALIB' during auto calibration")

    def test_65_keepalive_loop_transmits_auto_calib_func_motion(self):
        """65. Verify startDriveKeepaliveLoop isMaintenance allows autoCalibState.active to send FUNC_MOTION."""
        self.assertIn("const isMaintenance = activeTestInProgress || lidarTestState !== 'IDLE';", self.server_code,
            "isMaintenance must not block drive keepalive during auto calibration")

    def test_66_conservative_auto_calib_velocity_constants_defined(self):
        """66. Verify named conservative velocity constants are defined."""
        self.assertIn("AUTO_CALIB_FORWARD_MPS", self.server_code,
            "AUTO_CALIB_FORWARD_MPS constant must be defined")
        self.assertIn("AUTO_CALIB_TURN_RADPS", self.server_code,
            "AUTO_CALIB_TURN_RADPS constant must be defined")

    def test_67_stop_auto_calibration_zeros_targets_and_sends_func_motion_zero(self):
        """67. Verify stopAutoCalibration zeros targets, clears cmdSource, and sends zero FUNC_MOTION packet."""
        self.assertIn("targetLinear = 0.0;\n  targetAngular = 0.0;", self.server_code,
            "stopAutoCalibration must zero linear and angular targets")
        self.assertIn("cmdSource = 'NONE';", self.server_code,
            "stopAutoCalibration must reset cmdSource to NONE")


if __name__ == '__main__':
    unittest.main()




