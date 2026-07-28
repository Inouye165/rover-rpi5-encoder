# ==============================================================================
# test_stage6_diagnostics.py — Stage 6 Diagnostics Consolidation Test Suite
# ==============================================================================

import os
import sys
import re
import json
import unittest
from pathlib import Path

LOCAL_ROOT = Path(__file__).parent.resolve()


class TestStage6Diagnostics(unittest.TestCase):

    def setUp(self):
        self.index_html_path = LOCAL_ROOT / "public" / "index.html"
        with open(self.index_html_path, "r", encoding="utf-8") as f:
            self.html = f.read()

        self.app_js_path = LOCAL_ROOT / "public" / "app.js"
        with open(self.app_js_path, "r", encoding="utf-8") as f:
            self.js = f.read()

        self.server_js_path = LOCAL_ROOT / "server.js"
        with open(self.server_js_path, "r", encoding="utf-8") as f:
            self.server_js = f.read()

    def _extract_container(self, container_id):
        """Helper to extract inner HTML of a container div."""
        pattern = rf'<div\s+id=["\']{container_id}["\'].*?>(.*?)(?=<div\s+id=["\']tab-|\Z)'
        match = re.search(pattern, self.html, re.DOTALL)
        return match.group(1) if match else ""

    def test_1_layout_sections_exist(self):
        """1. Verify all 9 Stage 6 Diagnostics sections exist in tab-diagnostics-v2."""
        diagnostics_html = self._extract_container("tab-diagnostics-v2")
        self.assertTrue(len(diagnostics_html) > 0, "tab-diagnostics-v2 must exist in index.html")

        required_sections = [
            "diagnostics-overall-health",
            "diagnostics-drive-safety",
            "diagnostics-services",
            "diagnostics-ros2-nav",
            "diagnostics-sensor-health",
            "diagnostics-logs",
            "diagnostics-raw-telemetry",
            "diagnostics-developer",
            "diagnostics-export-bundle"
        ]

        for sec in required_sections:
            self.assertIn(
                f'id="{sec}"',
                diagnostics_html,
                f"Required Section '{sec}' missing in tab-diagnostics-v2"
            )

    def test_2_safety_exclusions_in_diagnostics_tab(self):
        """2. Verify NO movement/calibration/firmware-flash/unrestricted-write controls exist in tab-diagnostics-v2."""
        diagnostics_html = self._extract_container("tab-diagnostics-v2")
        
        prohibited_ids = [
            "btn-arm-drive",
            "btn-disarm-drive",
            "btn-estop",
            "btn-auto-fwd-1m",
            "btn-auto-turn-left",
            "btn-auto-turn-right",
            "btn-m1-fwd",
            "btn-m1-rev",
            "btn-m2-fwd",
            "btn-m2-rev",
            "btn-m3-fwd",
            "btn-m3-rev",
            "btn-m4-fwd",
            "btn-m4-rev",
            "btn-stop-all-maint",
            "btn-send-raw-command"
        ]

        for p_id in prohibited_ids:
            self.assertNotIn(
                f'id="{p_id}"',
                diagnostics_html,
                f"SAFETY GUARD VIOLATION: '{p_id}' must NOT exist inside tab-diagnostics-v2!"
            )

    def test_3_health_calculation_logic_in_app_js(self):
        """3. Verify app.js defines calculateOverallSystemHealth and handles HEALTHY, DEGRADED, FAULT states."""
        self.assertIn("function calculateOverallSystemHealth()", self.js)
        self.assertIn("function renderDiagnosticsV2()", self.js)
        self.assertIn("v2-health-val-overall", self.js)
        self.assertIn("v2-health-val-reason", self.js)

    def test_4_bounded_log_capacity(self):
        """4. Verify app.js enforces maximum bounded log capacity (500 entries max)."""
        self.assertIn("maxLogEntries = 500", self.js)
        self.assertIn("container.removeChild(container.firstChild)", self.js)

    def test_5_diagnostics_bundle_export(self):
        """5. Verify app.js defines generateDiagnosticsBundle and excludes secrets."""
        self.assertIn("function generateDiagnosticsBundle()", self.js)
        self.assertIn("delete st.tokens", self.js)
        self.assertIn("delete st.passwords", self.js)
        self.assertIn("delete st.keys", self.js)

    def test_6_legacy_redirection_notices(self):
        """6. Verify legacy tab containers were removed in Stage 7."""
        self.assertNotIn('id="tab-ros2"', self.html)

    def test_7_honest_unimplemented_feature_labels(self):
        """7. Verify planned features render honest 'Not implemented' state labels."""
        diagnostics_html = self._extract_container("tab-diagnostics-v2")
        self.assertIn("Nav2 Stack:", diagnostics_html)
        self.assertIn("SLAM Mapping:", diagnostics_html)
        self.assertIn("Not implemented", diagnostics_html)

    def test_8_dom_ids_singular_and_valid(self):
        """8. Verify terminal console exists once and browser raw command write input is removed in Stage 7."""
        critical_ids = [
            "terminal-console",
            "btn-clear-logs",
            "btn-copy-logs"
        ]
        for c_id in critical_ids:
            matches = re.findall(rf'id=["\']{c_id}["\']', self.html)
            self.assertEqual(len(matches), 1, f"ID '{c_id}' must exist exactly once in document!")

        # Raw serial command write box must be removed from browser UI in Stage 7
        self.assertNotIn('id="terminal-command-input"', self.html)
        self.assertNotIn('id="btn-send-raw-command"', self.html)

    def test_9_raw_log_filtering_and_verbose_toggle(self):
        """9. Verify raw/loop statistics packets are filtered out of normal logs and directed to verbose container."""
        self.assertIn("function isVerboseLog(text)", self.js)
        self.assertIn("chk-show-verbose-logs", self.html)
        self.assertIn("terminal-verbose-console", self.html)
        self.assertIn("v2-diag-verbose-logs-details", self.html)
        self.assertIn("logVerbose", self.js)

    def test_10_live_firmware_identity_parsing(self):
        """10. Verify app.js dynamically updates identity card from live telemetry and defaults to Awaiting."""
        self.assertIn("Awaiting firmware identity...", self.html)
        self.assertIn("case 'firmware_info':", self.js)
        self.assertNotIn("'1.0.0'", self.html, "Stale hardcoded version '1.0.0' must be removed from HTML defaults")

    def test_11_no_automatic_set_upload_and_maker_esp32_protocol_text(self):
        """11. Verify set_upload is not sent automatically and server references Maker ESP32 binary protocol."""
        # ws.onopen block in app.js must not call sendUploadConfig()
        onopen_match = re.search(r'ws\.onopen\s*=\s*\(\)\s*=>\s*\{(.*?)\};', self.js, re.DOTALL)
        if onopen_match:
            onopen_code = onopen_match.group(1)
            self.assertNotIn("sendUploadConfig()", onopen_code, "sendUploadConfig must NOT be called inside ws.onopen!")

        self.assertNotIn("ROS Expansion Board V3.0", self.server_js, "Misleading ROS Expansion Board V3.0 text must be removed from server.js")
        self.assertIn("Maker ESP32 binary protocol", self.server_js)

    def test_12_websocket_connecting_state_error_suppression(self):
        """12. Verify WebSocket CONNECTING state suppresses false startup errors and drops movement commands."""
        self.assertIn("ws.readyState === WebSocket.CONNECTING", self.js)
        self.assertNotIn("⚠️ Error: WebSocket is not open to send command.", self.js)

    def test_13_command_source_and_motion_clarity(self):
        """13. Verify Command Source 2 displays Gamepad label and motion request status renders Zero when idle."""
        self.assertIn("Gamepad (Source 2)", self.js)
        self.assertIn("v2-diag-val-motion-status", self.html)
        self.assertIn("Zero (No Movement Requested)", self.js)


if __name__ == "__main__":
    unittest.main()
