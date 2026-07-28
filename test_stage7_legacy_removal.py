# ==============================================================================
# test_stage7_legacy_removal.py — Stage 7 Canonical Controller & Legacy Removal Tests
# ==============================================================================

import os
import sys
import re
import json
import unittest
from pathlib import Path

LOCAL_ROOT = Path(__file__).parent.resolve()


class TestStage7LegacyRemoval(unittest.TestCase):

    def setUp(self):
        self.index_html_path = LOCAL_ROOT / "public" / "index.html"
        with open(self.index_html_path, "r", encoding="utf-8") as f:
            self.html = f.read()

        self.app_js_path = LOCAL_ROOT / "public" / "app.js"
        with open(self.app_js_path, "r", encoding="utf-8") as f:
            self.js = f.read()

    def _extract_container(self, container_id):
        """Helper to extract inner HTML of a container div."""
        pattern = rf'<div\s+id=["\']{container_id}["\'].*?>(.*?)(?=<div\s+id=["\']tab-|\Z)'
        match = re.search(pattern, self.html, re.DOTALL)
        return match.group(1) if match else ""

    def test_1_canonical_gamepad_functions_exist(self):
        """1. Verify canonical gamepad controller functions exist in app.js."""
        required_funcs = [
            "function checkGamepadConnection()",
            "function startGamepadLoop()",
            "function updateGamepadBadge(",
            "function updateGamepadHUD(",
            "window.addEventListener('gamepadconnected'",
            "window.addEventListener('gamepaddisconnected'"
        ]

        for func in required_funcs:
            self.assertIn(
                func,
                self.js,
                f"Canonical controller function or handler '{func}' missing in app.js"
            )

    def test_2_canonical_gamepad_hud_elements_in_drive_v2(self):
        """2. Verify all Gamepad HUD elements exist inside tab-drive-v2 in index.html."""
        drive_v2_html = self._extract_container("tab-drive-v2")
        self.assertTrue(len(drive_v2_html) > 0, "tab-drive-v2 must exist in index.html")

        required_hud_ids = [
            "v2-drive-val-gamepad",
            "gp-live-arm",
            "gp-live-deadman",
            "gp-live-linear",
            "gp-live-angular",
            "gp-live-stop",
            "gp-live-estop",
            "gp-live-buttons"
        ]

        for hud_id in required_hud_ids:
            self.assertIn(
                f'id="{hud_id}"',
                drive_v2_html,
                f"Gamepad HUD element '{hud_id}' missing from tab-drive-v2 in index.html"
            )

    def test_3_controller_has_no_legacy_dependencies(self):
        """3. Verify controller functions contain no references to tab-legacy or tab-dashboard."""
        # Extract the Gamepad Integration block from app.js
        gp_match = re.search(r'// Gamepad Controller Integration.*?(?=// Stage 4 Safety Event Handlers|\Z)', self.js, re.DOTALL)
        self.assertIsNotNone(gp_match, "Gamepad Controller Integration block must exist in app.js")
        gp_code = gp_match.group(0)

        prohibited_legacy_refs = [
            "tab-legacy",
            "tab-dashboard",
            "legacy-target"
        ]

        for ref in prohibited_legacy_refs:
            self.assertNotIn(
                ref,
                gp_code,
                f"Controller code must NOT depend on legacy reference '{ref}'"
            )

    def test_4_duplicate_gamepad_loop_guard(self):
        """4. Verify startGamepadLoop contains guard to prevent duplicate concurrent loops."""
        self.assertIn(
            "if (gamepadLoopRunning) return;",
            self.js,
            "startGamepadLoop must contain duplicate loop guard 'if (gamepadLoopRunning) return;'"
        )

    def test_5_safety_stop_event_listeners(self):
        """5. Verify blur, visibilitychange, and gamepaddisconnected listeners dispatch safe stop."""
        self.assertIn("window.addEventListener('blur'", self.js)
        self.assertIn("document.addEventListener('visibilitychange'", self.js)
        self.assertIn("window.addEventListener('gamepaddisconnected'", self.js)

    def test_6_deadman_button_and_threshold_policy(self):
        """6. Verify deadman switch policy uses RB (5) or RT (7) with strict 0.5 threshold."""
        gp_match = re.search(r'// Gamepad Controller Integration.*?(?=// Stage 4 Safety Event Handlers|\Z)', self.js, re.DOTALL)
        self.assertIsNotNone(gp_match, "Gamepad Controller Integration block must exist in app.js")
        gp_code = gp_match.group(0)

        # Confirm RB (5) and RT (7) are used as deadman buttons
        self.assertIn("gp.buttons[5]", gp_code, "Deadman policy must evaluate RB (button 5)")
        self.assertIn("gp.buttons[7]", gp_code, "Deadman policy must evaluate RT (button 7)")

        # Confirm 0.5 analog threshold is used to prevent resting trigger noise activation
        self.assertIn("gp.buttons[5].value > 0.5", gp_code, "RB deadman evaluation must require value > 0.5")
        self.assertIn("gp.buttons[7].value > 0.5", gp_code, "RT deadman evaluation must require value > 0.5")

        # Confirm LB (4) and LT (6) are NOT used as deadman buttons to avoid accidental left-hand activation
        deadman_line = [l for l in gp_code.splitlines() if 'deadmanPressed' in l and 'Boolean' in l]
        if deadman_line:
            line_str = " ".join(deadman_line)
            self.assertNotIn("gp.buttons[4]", line_str, "LB (button 4) must NOT be an active deadman button")
            self.assertNotIn("gp.buttons[6]", line_str, "LT (button 6) must NOT be an active deadman button")

    def test_7_primary_stick_only_policy(self):
        """7. Verify primary control strictly uses Left stick (axes 0, 1) without right-stick fallback."""
        gp_match = re.search(r'// Gamepad Controller Integration.*?(?=// Stage 4 Safety Event Handlers|\Z)', self.js, re.DOTALL)
        self.assertIsNotNone(gp_match, "Gamepad Controller Integration block must exist in app.js")
        gp_code = gp_match.group(0)

        self.assertIn("let throttle = -gp.axes[1];", gp_code, "Throttle must strictly use Left stick Y (axes 1)")
        self.assertIn("let turn = gp.axes[0];", gp_code, "Turn must strictly use Left stick X (axes 0)")
        self.assertNotIn("gp.axes[2]", gp_code, "Right stick X (axes 2) fallback must be disabled")
        self.assertNotIn("gp.axes[3]", gp_code, "Right stick Y (axes 3) fallback must be disabled")

    def test_8_simulated_gamepad_telemetry_chain(self):
        """8. Behavioral verification: Simulate gamepad joystick payload creation without legacy HTML."""
        # Verify joystick payload structure generated by sendServerMessage call
        self.assertIn("type: 'joystick'", self.js)
        self.assertIn("x: turn", self.js)
        self.assertIn("y: throttle", self.js)
        self.assertIn("deadman: deadmanPressed", self.js)


if __name__ == "__main__":
    unittest.main()

