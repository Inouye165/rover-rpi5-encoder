# ==============================================================================
# test_stage2_shell.py — Unit & Structural Contract Tests for Stage 2 Shell
# ==============================================================================

import os
import sys
import re
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestStage2ShellStructure(unittest.TestCase):

    def setUp(self):
        self.index_html_path = os.path.join(os.path.dirname(__file__), 'public', 'index.html')
        with open(self.index_html_path, 'r', encoding='utf-8') as f:
            self.html = f.read()

        self.app_js_path = os.path.join(os.path.dirname(__file__), 'public', 'app.js')
        with open(self.app_js_path, 'r', encoding='utf-8') as f:
            self.js = f.read()

        self.server_js_path = os.path.join(os.path.dirname(__file__), 'server.js')
        with open(self.server_js_path, 'r', encoding='utf-8') as f:
            self.server = f.read()

    def test_1_five_primary_nav_buttons_exist_once(self):
        """1. Verify five new primary navigation buttons exist exactly once."""
        primary_btn_ids = ['nav-drive', 'nav-autonomy', 'nav-sensors', 'nav-calibration', 'nav-diagnostics']
        for btn_id in primary_btn_ids:
            matches = re.findall(rf'id=["\']{btn_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Nav button '{btn_id}' must exist exactly once in HTML.")

    def test_2_temporary_legacy_nav_button_exists_once(self):
        """2. Verify temporary Legacy Tools navigation button exists exactly once."""
        matches = re.findall(r'id=["\']nav-legacy["\']', self.html)
        self.assertEqual(len(matches), 1, "Nav button 'nav-legacy' must exist exactly once in HTML.")

    def test_3_five_destination_containers_exist_once(self):
        """3. Verify all five destination tab containers exist exactly once."""
        v2_tab_ids = ['tab-drive-v2', 'tab-autonomy-v2', 'tab-sensors-v2', 'tab-calibration-v2', 'tab-diagnostics-v2']
        for tab_id in v2_tab_ids:
            matches = re.findall(rf'id=["\']{tab_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Destination tab '{tab_id}' must exist exactly once in HTML.")

    def test_4_all_28_placeholder_ids_exist_once(self):
        """4. Verify all 28 required placeholder IDs exist exactly once."""
        required_placeholders = [
            # DRIVE
            'drive-status-summary', 'drive-manual-control', 'drive-camera', 'drive-faults',
            # AUTONOMY
            'autonomy-stack-health', 'autonomy-localization', 'autonomy-slam', 'autonomy-nav2', 'autonomy-goal', 'autonomy-foxglove',
            # SENSORS
            'sensors-summary', 'sensors-lidar', 'sensors-imu', 'sensors-odometry', 'sensors-future',
            # CALIBRATION
            'calibration-readiness', 'calibration-primary-tests', 'calibration-live-progress', 'calibration-latest-result', 'calibration-repeatability', 'calibration-advanced',
            # DIAGNOSTICS
            'diagnostics-services', 'diagnostics-firmware', 'diagnostics-serial', 'diagnostics-ros', 'diagnostics-logs', 'diagnostics-developer'
        ]
        for p_id in required_placeholders:
            matches = re.findall(rf'id=["\']{p_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Placeholder ID '{p_id}' must exist exactly once in HTML.")

    def test_5_all_seven_legacy_tab_containers_exist(self):
        """5. Verify all seven legacy tab containers exist."""
        legacy_tab_ids = ['tab-dashboard', 'tab-imu', 'tab-encoder', 'tab-ros2', 'tab-calibrate', 'tab-motion-cal', 'tab-lidar']
        for tab_id in legacy_tab_ids:
            matches = re.findall(rf'id=["\']{tab_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Legacy tab container '{tab_id}' must exist exactly once in HTML.")

    def test_6_safety_controls_not_duplicated(self):
        """6. Verify key safety and command controls were not duplicated."""
        unique_controls = [
            'btn-estop',
            'btn-arm-drive',
            'btn-disarm-drive',
            'btn-auto-fwd-1m',
            'btn-auto-turn-left',
            'btn-auto-turn-right',
            'btn-stop-all-maint',
            'btn-m1-fwd',
            'btn-m1-rev'
        ]
        for ctrl_id in unique_controls:
            matches = re.findall(rf'id=["\']{ctrl_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Control ID '{ctrl_id}' must exist exactly once to prevent command ambiguity.")

    def test_7_default_tab_is_drive(self):
        """7. Verify default active top-level tab is Drive (tab-drive-v2)."""
        self.assertIn('<button class="tab-btn active" id="nav-drive"', self.html)
        self.assertIn('<div id="tab-drive-v2" class="tab-content active"', self.html)

    def test_8_aria_accessibility_attributes_present(self):
        """8. Verify ARIA tab roles and tabindex attributes are present."""
        self.assertIn('role="tablist"', self.html)
        self.assertIn('role="tab"', self.html)
        self.assertIn('role="tabpanel"', self.html)
        self.assertIn('aria-selected="true"', self.html)
        self.assertIn('tabindex="0"', self.html)

    def test_9_no_backend_route_changes(self):
        """9. Verify no backend routes were added or modified in server.js."""
        expected_endpoints = [
            '/api/drive/arm',
            '/api/drive/disarm',
            '/api/stop',
            '/api/maintenance/run_test',
            '/api/calibration/auto/start'
        ]
        for ep in expected_endpoints:
            self.assertIn(ep, self.server)

    def test_10_lidar_tab_activation_refresh_hook(self):
        """10. Verify legacy LiDAR tab activation invokes updateLidarTabState and pollLidar/drawPolarScan hook."""
        self.assertIn('function updateLidarTabState()', self.js)
        self.assertIn('function activateLegacySubTab(', self.js)
        # Verify updateLidarTabState is referenced inside app.js and inside activateLegacySubTab
        sub_tab_idx = self.js.find('function activateLegacySubTab(')
        self.assertNotEqual(sub_tab_idx, -1)
        sub_tab_code = self.js[sub_tab_idx:sub_tab_idx + 1800]
        self.assertIn('updateLidarTabState()', sub_tab_code)

    def test_11_lidar_canvas_render_and_status_endpoint_binding(self):
        """11. Verify LiDAR status is driven by /api/lidar/status and canvas draw uses requestAnimationFrame after layout."""
        self.assertIn('/api/lidar/status', self.js)
        self.assertIn('/api/lidar/scan', self.js)
        self.assertIn('requestAnimationFrame', self.js)
        self.assertIn('drawPolarScan(latestLidarScan)', self.js)

    def test_12_single_lidar_polling_interval_guard(self):
        """12. Verify startLidarPolling guards against creating duplicate interval timers."""
        start_fn = re.search(r'function startLidarPolling\(\)\s*\{(.*?)\}', self.js, re.DOTALL)
        self.assertIsNotNone(start_fn)
        self.assertIn('if (lidarPollTimer) return;', start_fn.group(1))
        self.assertIn('setInterval(pollLidar,', start_fn.group(1))

    def test_13_all_seven_legacy_tabs_reachable_in_sub_nav(self):
        """13. Verify all seven legacy tabs are referenced in the legacy sub-nav bar."""
        legacy_tabs = ['tab-dashboard', 'tab-imu', 'tab-encoder', 'tab-ros2', 'tab-calibrate', 'tab-motion-cal', 'tab-lidar']
        for tab in legacy_tabs:
            self.assertIn(f'data-legacy-tab="{tab}"', self.html)


if __name__ == '__main__':
    unittest.main()

