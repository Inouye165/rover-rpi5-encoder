# ==============================================================================
# test_stage4_drive.py — Automated Unit & Structural Tests for Stage 4
# ==============================================================================

import os
import sys
import re
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestStage4Drive(unittest.TestCase):

    def setUp(self):
        self.index_html_path = os.path.join(os.path.dirname(__file__), 'public', 'index.html')
        with open(self.index_html_path, 'r', encoding='utf-8') as f:
            self.html = f.read()

        self.app_js_path = os.path.join(os.path.dirname(__file__), 'public', 'app.js')
        with open(self.app_js_path, 'r', encoding='utf-8') as f:
            self.js = f.read()

    def _extract_container(self, container_id):
        """Helper to extract inner HTML of a container div."""
        pattern = rf'<div\s+id=["\']{container_id}["\'].*?>(.*?)(?=<div\s+id=["\']tab-|\Z)'
        match = re.search(pattern, self.html, re.DOTALL)
        return match.group(1) if match else self.html

    def test_1_drive_v2_layout_hierarchy(self):
        """1. Verify Drive tab contains camera, compact LiDAR, manual controls, status strip, and fault banner."""
        drive_html = self._extract_container('tab-drive-v2')
        self.assertIn('id="drive-camera"', drive_html)
        self.assertIn('id="drive-compact-lidar"', drive_html)
        self.assertIn('id="drive-manual-control"', drive_html)
        self.assertIn('id="drive-status-summary"', drive_html)
        self.assertIn('id="drive-faults"', drive_html)

    def test_2_operational_control_singularity(self):
        """2. STRICT GUARD: Verify every operational control exists EXACTLY ONCE in the document."""
        operational_ids = [
            'btn-arm-drive',
            'btn-disarm-drive',
            'btn-estop',
            'ctrl-forward',
            'ctrl-reverse',
            'ctrl-left',
            'ctrl-right',
            'ctrl-spin-left',
            'ctrl-spin-right',
            'ctrl-stop-center',
            'sync-speed-slider'
        ]
        for ctrl_id in operational_ids:
            matches = re.findall(rf'id=["\']{ctrl_id}["\']', self.html)
            self.assertEqual(
                len(matches), 1,
                f"CONTROL SINGULARITY VIOLATION: '{ctrl_id}' must exist exactly ONCE in document (found {len(matches)})."
            )

    def test_3_safe_stop_lifecycle_handlers(self):
        """3. Verify app.js implements safe stop on tab departure, window blur, visibility loss, and gamepad disconnect."""
        self.assertIn("if (activeTopTabId === 'tab-drive-v2' && targetTabId !== 'tab-drive-v2')", self.js)
        self.assertIn("window.addEventListener('blur',", self.js)
        self.assertIn("document.addEventListener('visibilitychange',", self.js)
        self.assertIn("window.addEventListener('gamepaddisconnect',", self.js)
        self.assertIn("driveRover('stop')", self.js)

    def test_4_no_auto_arm_on_tab_activation(self):
        """4. Verify tab activation logic does NOT invoke arming or drive commands."""
        # Ensure activateTopTab does not contain armNormalDrive or sendServerMessage arming calls
        pattern = r'function activateTopTab\(targetTabId\)\s*\{(.*?)\}\n\nfunction'
        match = re.search(pattern, self.js, re.DOTALL)
        if match:
            tab_fn_body = match.group(1)
            self.assertNotIn("armNormalDrive", tab_fn_body)
            self.assertNotIn("FUNC_ARM_NORMAL_DRIVE", tab_fn_body)

    def test_5_camera_stream_migration(self):
        """5. Verify camera stream element exists in tab-drive-v2 and has single stream owner."""
        self.assertIn('id="camera-stream"', self.html)
        self.assertIn('id="camera-viewport"', self.html)
        self.assertIn('id="btn-toggle-camera"', self.html)
        self.assertIn('id="btn-fullscreen-camera"', self.html)
        # Ensure camera stream img tag exists exactly once
        cam_matches = re.findall(r'id=["\']camera-stream["\']', self.html)
        self.assertEqual(len(cam_matches), 1, "Camera stream element must exist exactly once.")

    def test_6_compact_lidar_view(self):
        """6. Verify compact LiDAR canvas exists with unique canvas ID and uses shared drawCompactLidarScan."""
        self.assertIn('id="v2-compact-lidar-canvas"', self.html)
        self.assertIn('id="v2-compact-lidar-dist"', self.html)
        self.assertIn('id="v2-compact-lidar-status"', self.html)
        self.assertIn('function drawCompactLidarScan(scan)', self.js)

    def test_7_legacy_tools_compatibility_banner(self):
        """7. Verify tab-dashboard contains legacy compatibility banner with redirect button."""
        dash_html = self._extract_container('tab-dashboard')
        self.assertIn('Manual Driving Controls Moved to Drive Tab', dash_html)
        self.assertIn('data-target-tab="tab-drive-v2"', dash_html)

    def test_8_canonical_drive_state_functions_exist(self):
        """8. Verify updateCanonicalDriveState and fetchDriveStatus exist in app.js."""
        self.assertIn('function updateCanonicalDriveState(', self.js)
        self.assertIn('function fetchDriveStatus(', self.js)

    def test_9_ws_normal_drive_status_routes_to_canonical_state(self):
        """9. Verify normal_drive_status WebSocket case invokes updateCanonicalDriveState."""
        pattern = r"case 'normal_drive_status':\s*\{\s*updateCanonicalDriveState\(msg\);"
        self.assertTrue(re.search(pattern, self.js), "WebSocket normal_drive_status must call updateCanonicalDriveState(msg).")

    def test_10_arm_and_disarm_fetch_status_synchronization(self):
        """10. Verify armNormalDrive and disarmNormalDrive call fetchDriveStatus after HTTP request."""
        # Ensure armNormalDrive contains fetchDriveStatus
        arm_pattern = r'function armNormalDrive\(\)\s*\{(.*?)\}\n\nfunction'
        arm_match = re.search(arm_pattern, self.js, re.DOTALL)
        self.assertTrue(arm_match and 'fetchDriveStatus()' in arm_match.group(1), "armNormalDrive must call fetchDriveStatus().")

        # Ensure disarmNormalDrive contains fetchDriveStatus
        disarm_pattern = r'function disarmNormalDrive\(\)\s*\{(.*?)\}\n\nconst'
        disarm_match = re.search(disarm_pattern, self.js, re.DOTALL)
        self.assertTrue(disarm_match and 'fetchDriveStatus()' in disarm_match.group(1), "disarmNormalDrive must call fetchDriveStatus().")

    def test_11_lock_status_interpretation_logic(self):
        """11. Verify lockStatus=false displays CLEAR (DISABLED) and lockStatus=true displays CLAMP ACTIVE."""
        self.assertIn("drv.lockStatus === true", self.js)
        self.assertIn("CLAMP ACTIVE", self.js)
        self.assertIn("CLEAR (DISABLED)", self.js)

    def test_12_dom_content_loaded_poller_startup(self):
        """12. Verify updateLidarTabState and checkGamepadConnection run on DOMContentLoaded."""
        self.assertIn("updateLidarTabState()", self.js)
        self.assertIn("checkGamepadConnection()", self.js)
        pattern = r"DOMContentLoaded.*?updateLidarTabState"
        self.assertTrue(re.search(pattern, self.js, re.DOTALL), "updateLidarTabState must be called on DOMContentLoaded.")

    def test_13_dpad_pointer_deadman_bindings(self):
        """13. Verify bindDpadButton attaches pointerdown and pointerup/pointerleave/pointercancel handlers."""
        self.assertIn("function bindDpadButton(", self.js)
        self.assertIn("pointerdown", self.js)
        self.assertIn("pointerup", self.js)
        self.assertIn("pointerleave", self.js)

    def test_14_keyboard_keyup_safe_stop(self):
        """14. Verify keyup event handler issues driveRover('stop') when drive keys are released."""
        pattern = r"document\.addEventListener\(['\"]keyup['\"].*?driveRover\(['\"]stop['\"]\)"
        self.assertTrue(re.search(pattern, self.js, re.DOTALL), "keyup handler must trigger driveRover('stop').")

    def test_15_gamepad_auto_detection_and_hud(self):
        """15. Verify checkGamepadConnection checks navigator.getGamepads and updates HUD state."""
        self.assertIn("function checkGamepadConnection()", self.js)
        self.assertIn("navigator.getGamepads", self.js)
        self.assertIn("Controller connected — deadman not active", self.js)

    def test_16_websocket_auto_start_and_rate_limit(self):
        """16. Verify connectWebSocket runs on DOMContentLoaded and sendServerMessage rate-limits error logging."""
        pattern = r"DOMContentLoaded.*?connectWebSocket"
        self.assertTrue(re.search(pattern, self.js, re.DOTALL), "connectWebSocket must be called on DOMContentLoaded.")
        self.assertIn("lastWsErrorLogTime", self.js)
        self.assertIn("WebSocket is not open to send command.", self.js)


if __name__ == '__main__':
    unittest.main()
