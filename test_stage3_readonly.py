# ==============================================================================
# test_stage3_readonly.py — Automated Unit & Structural Tests for Stage 3
# ==============================================================================

import os
import sys
import re
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestStage3ReadonlyStructure(unittest.TestCase):

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

    def _extract_container(self, container_id):
        """Helper to extract inner HTML of a container div."""
        pattern = rf'<div\s+id=["\']{container_id}["\'].*?>(.*?)(?=<div\s+id=["\']tab-|\Z)'
        match = re.search(pattern, self.html, re.DOTALL)
        return match.group(1) if match else ""

    def test_1_drive_v2_readonly_content(self):
        """1. Verify Drive V2 contains read-only telemetry strip and fault banner."""
        self.assertIn('id="drive-status-summary"', self.html)
        self.assertIn('id="drive-faults"', self.html)
        self.assertIn('v2-drive-val-armed', self.html)
        self.assertIn('v2-drive-val-estop', self.html)
        self.assertIn('v2-drive-val-ws', self.html)
        self.assertIn('v2-drive-val-serial', self.html)

    def test_2_prohibited_control_guard_in_v2_containers(self):
        """2. STRICT GUARD: Verify NO prohibited motor/calibration/serial control IDs exist in read-only V2 containers."""
        v2_container_ids = ['tab-autonomy-v2', 'tab-sensors-v2', 'tab-diagnostics-v2']
        prohibited_control_ids = [
            'btn-arm-drive',
            'btn-disarm-drive',
            'btn-estop',
            'btn-auto-fwd-1m',
            'btn-auto-turn-left',
            'btn-auto-turn-right',
            'btn-m1-fwd',
            'btn-m1-rev',
            'btn-m2-fwd',
            'btn-m2-rev',
            'btn-m3-fwd',
            'btn-m3-rev',
            'btn-m4-fwd',
            'btn-m4-rev',
            'btn-stop-all-maint',
            'btn-send-raw-command'
        ]

        for container_id in v2_container_ids:
            container_html = self._extract_container(container_id)
            for p_id in prohibited_control_ids:
                self.assertNotIn(
                    f'id="{p_id}"',
                    container_html,
                    f"PROHIBITED CONTROL GUARD VIOLATION: '{p_id}' must NOT exist inside '{container_id}'."
                )

    def test_3_autonomy_v2_ros_health_and_localization(self):
        """3. Verify Autonomy V2 renders ROS health, Foxglove, and odometry-only localization label."""
        self.assertIn('id="autonomy-stack-health"', self.html)
        self.assertIn('id="autonomy-localization"', self.html)
        self.assertIn('id="autonomy-foxglove"', self.html)
        self.assertIn('Wheel Odometry Only (Skid-Steer Kinematics)', self.html)
        self.assertIn('ws://10.0.0.246:8765', self.html)

    def test_4_autonomy_v2_honest_planned_states(self):
        """4. Verify SLAM, Nav2, and Goal controls render honest 'Not installed' state labels."""
        self.assertIn('id="autonomy-slam"', self.html)
        self.assertIn('id="autonomy-nav2"', self.html)
        self.assertIn('id="autonomy-goal"', self.html)
        self.assertIn('SLAM map building service is <strong>Not installed</strong>', self.html)
        self.assertIn('Nav2 stack planner and controller servers are <strong>Not installed</strong>', self.html)
        self.assertIn('Navigation goal controls will be enabled after Nav2 integration', self.html)

    def test_5_sensors_v2_lidar_imu_odom_containers(self):
        """5. Verify Sensors V2 contains summary, LiDAR canvas, IMU 3D canvas, and Odometry canvas containers."""
        self.assertIn('id="sensors-summary"', self.html)
        self.assertIn('id="sensors-lidar"', self.html)
        self.assertIn('id="sensors-imu"', self.html)
        self.assertIn('id="sensors-odometry"', self.html)
        self.assertIn('id="sensors-future"', self.html)
        self.assertIn('id="v2-lidar-polar-canvas"', self.html)
        self.assertIn('id="v2-imu-model-canvas"', self.html)
        self.assertIn('id="v2-odom-traj-canvas"', self.html)
        self.assertIn('Ultrasonic Range Sensors: <strong>Not installed</strong>', self.html)

    def test_6_diagnostics_v2_services_firmware_serial_ros(self):
        """6. Verify Diagnostics V2 contains service status, firmware, serial, and ROS diagnostic cards."""
        self.assertIn('id="diagnostics-services"', self.html)
        self.assertIn('id="diagnostics-firmware"', self.html)
        self.assertIn('id="diagnostics-serial"', self.html)
        self.assertIn('id="diagnostics-ros"', self.html)
        self.assertIn('v2-fw-val-board', self.html)
        self.assertIn('v2-serial-val-dev', self.html)

    def test_7_canonical_shared_state_and_render_functions(self):
        """7. Verify app.js defines window.roverState and Stage 3 rendering functions."""
        self.assertIn('window.roverState =', self.js)
        self.assertIn('function renderStage3V2Panels()', self.js)
        self.assertIn('function renderDriveV2Status()', self.js)
        self.assertIn('function renderAutonomyV2()', self.js)
        self.assertIn('function renderSensorsV2Summary()', self.js)
        self.assertIn('function renderDiagnosticsV2()', self.js)

    def test_8_control_singularity_preserved_in_legacy_tabs(self):
        """8. Verify all original operational controls exist exactly once in the document."""
        operational_controls = [
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
        for ctrl_id in operational_controls:
            matches = re.findall(rf'id=["\']{ctrl_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Operational control '{ctrl_id}' must exist exactly once in document.")

    def test_9_all_seven_legacy_tabs_reachable(self):
        """9. Verify all seven legacy tabs are removed in Stage 7."""
        legacy_tabs = ['tab-dashboard', 'tab-imu', 'tab-encoder', 'tab-ros2', 'tab-calibrate', 'tab-motion-cal', 'tab-lidar']
        for tab in legacy_tabs:
            self.assertNotIn(f'data-legacy-tab="{tab}"', self.html)
            self.assertNotIn(f'id="{tab}"', self.html)


if __name__ == '__main__':
    unittest.main()
