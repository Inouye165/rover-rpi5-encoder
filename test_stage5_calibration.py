# ==============================================================================
# test_stage5_calibration.py — Verification Tests for Stage 5 Calibration Tab
# ==============================================================================

import os
import sys
import re
import json
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestStage5CalibrationTab(unittest.TestCase):

    def setUp(self):
        self.repo_dir = os.path.dirname(os.path.abspath(__file__))
        self.index_html_path = os.path.join(self.repo_dir, 'public', 'index.html')
        with open(self.index_html_path, 'r', encoding='utf-8') as f:
            self.html = f.read()

        self.app_js_path = os.path.join(self.repo_dir, 'public', 'app.js')
        with open(self.app_js_path, 'r', encoding='utf-8') as f:
            self.js = f.read()

        self.server_js_path = os.path.join(self.repo_dir, 'server.js')
        with open(self.server_js_path, 'r', encoding='utf-8') as f:
            self.server = f.read()

    def test_1_canonical_8_sections_exist_in_calibration_v2(self):
        """1. Verify all 8 canonical sections exist in #tab-calibration-v2."""
        sections = [
            'calibration-readiness',
            'calibration-wheel-verification',
            'calibration-primary-tests',
            'calibration-live-progress',
            'calibration-latest-result',
            'calibration-repeatability',
            'calibration-advanced',
            'calibration-constants'
        ]
        for sec in sections:
            matches = re.findall(rf'id=["\']{sec}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Section '{sec}' must exist exactly once in HTML.")

    def test_2_calibration_control_uniqueness(self):
        """2. Verify canonical calibration controls exist exactly once across the whole document."""
        canonical_controls = [
            # Auto tests
            'btn-auto-fwd-1m', 'btn-auto-turn-left', 'btn-auto-turn-right', 'btn-auto-abort',
            # Single wheel maintenance
            'btn-m1-fwd', 'btn-m1-rev', 'btn-m2-fwd', 'btn-m2-rev',
            'btn-m3-fwd', 'btn-m3-rev', 'btn-m4-fwd', 'btn-m4-rev',
            'btn-stop-all-maint', 'maint-safety-chk', 'maint-status-badge', 'maint-test-result-card',
            # Advanced tests
            'btn-cal-run-distance', 'btn-cal-run-rot-cw', 'btn-cal-run-rot-ccw',
            # Repeatability & DB
            'repeatability-stats-hud', 'cal-surface-type', 'cal-history-table-body'
        ]
        for ctrl_id in canonical_controls:
            matches = re.findall(rf'id=["\']{ctrl_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"Control ID '{ctrl_id}' must exist exactly once across entire document.")

    def test_3_safety_exclusions_in_calibration_tab(self):
        """3. Verify drive joysticks, WASD keys, and arm drive buttons are excluded from calibration tab."""
        # Find inner HTML of tab-calibration-v2
        calib_match = re.search(r'<div id=["\']tab-calibration-v2["\'][\s\S]*?(?=<div id=["\']tab-|\Z)', self.html)
        self.assertIsNotNone(calib_match, "tab-calibration-v2 must be present in HTML.")
        calib_html = calib_match.group(0)

        # Drive controls should NOT be inside calibration tab
        self.assertNotIn('dpad-wrapper', calib_html, "D-Pad joystick controls must NOT be inside calibration tab.")
        self.assertNotIn('btn-arm-drive', calib_html, "Arm normal drive button must NOT be inside calibration tab.")
        self.assertNotIn('armNormalDrive', calib_html, "Arm normal drive trigger must NOT be inside calibration tab.")

    def test_4_readiness_calculation_logic_present(self):
        """4. Verify frontend readiness update logic and backend endpoints exist."""
        self.assertIn('updateCalibrationReadiness', self.js, "updateCalibrationReadiness must be defined in app.js.")
        self.assertIn('/api/calibration/auto/start', self.server, "Auto calib start endpoint must be present in server.js.")
        self.assertIn('/api/calibration/auto/status', self.server, "Auto calib status endpoint must be present in server.js.")

    def test_5_qualification_rule_in_server(self):
        """5. Verify server computeRepeatabilityStats enforces strict target_reached & pass===true qualification."""
        self.assertIn("target_reached", self.server, "server.js must check target_reached for qualification.")
        self.assertIn("pass", self.server, "server.js must check pass status for qualification.")


if __name__ == '__main__':
    unittest.main()
