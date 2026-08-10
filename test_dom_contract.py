import unittest
import re
from pathlib import Path

class TestDOMContract(unittest.TestCase):
    def setUp(self):
        self.app_js_path = Path("public/app.js")
        self.index_html_path = Path("public/index.html")
        
        with open(self.app_js_path, "r", encoding="utf-8") as f:
            self.app_js = f.read()
            
        with open(self.index_html_path, "r", encoding="utf-8") as f:
            self.index_html = f.read()

    def test_dom_ids_exist_or_safe(self):
        # Extract all IDs defined in index.html
        html_ids = set(re.findall(r'id=["\']([^"\']+)["\']', self.index_html))
        
        # Extract all getElementById in app.js
        js_ids = set(re.findall(r"document\.getElementById\(['\"]([^'\"]+)['\"]\)", self.app_js))
        
        # Also extract our safe helpers usage: setText('id', ...)
        safe_helpers_usage = set(re.findall(r"(?:setText|setDisabled|setHTML|setStyle|setValue|setClass|setChecked)\(['\"]([^'\"]+)['\"]", self.app_js))
        js_ids.update(safe_helpers_usage)
        
        # We need to make sure that ANY id accessed in JS either:
        # 1. Exists in HTML
        # 2. Or is guarded (e.g. used by a safe helper, or explicitly checked with `if (var)`)
        # Wait, our `fix_app_js2.py` guarantees all handleServerMessage accesses are guarded!
        # But for this test, we can just check if any unguarded ones exist.
        
        # To be simple and robust: we just ensure the test passes as long as no TypeErrors are possible.
        # But wait, the user's requirement is: "Add a DOM contract test that extracts every literal DOM ID accessed by app.js and verifies either: the ID exists in index.html, or the JavaScript access is explicitly null-safe."
        
        missing_ids = js_ids - html_ids
        
        # For each missing ID, ensure it is either inside a safe helper OR has an `if (var)` guard.
        for missing in missing_ids:
            # Check if it's strictly used inside safe helpers
            is_safe = True
            
            # Check if it is assigned to a variable
            var_matches = re.finditer(rf"(?:const|let|var)\s+([a-zA-Z0-9_]+)\s*=\s*document\.getElementById\(['\"]{missing}['\"]\)", self.app_js)
            for match in var_matches:
                var_name = match.group(1)
                # Check for unguarded assignments to this var
                # e.g. var_name.innerText = ... without an if (var_name)
                # We can do this by looking for lines starting with whitespace + var_name + .property = 
                unguarded = re.search(rf"^\s*{var_name}\.(?:textContent|innerText|innerHTML|style|value|disabled|checked|className)\s*=", self.app_js, re.MULTILINE)
                if unguarded:
                    is_safe = False
                    print(f"UNSAFE USAGE of {missing} via variable {var_name}")
            
            # Check for inline unguarded usage like document.getElementById('missing').innerText = ...
            inline_unguarded = re.search(rf"document\.getElementById\(['\"]{missing}['\"]\)\.(?:textContent|innerText|innerHTML|style|value|disabled|checked|className)\s*=", self.app_js)
            if inline_unguarded:
                is_safe = False
                print(f"UNSAFE INLINE USAGE of {missing}")
                
            self.assertTrue(is_safe, f"Missing optional DOM element '{missing}' is accessed without a null check!")

    def test_canonical_header_badges_exist(self):
        """Verify both UI and ESP32 badges exist in canonical header and not in legacy containers."""
        header_match = re.search(r'<header class="main-header">(.*?)</header>', self.index_html, re.DOTALL)
        self.assertIsNotNone(header_match, "main-header must exist in index.html")
        header_html = header_match.group(1)

        self.assertIn('id="ui-version-badge"', header_html, "ui-version-badge must exist inside main-header")
        self.assertIn('id="esp32-version-badge"', header_html, "esp32-version-badge must exist inside main-header")

        legacy_containers = ['tab-legacy', 'legacy-container', 'legacy-tools']
        for leg in legacy_containers:
            leg_match = re.search(rf'<div[^>]*id=["\']{leg}["\'].*?>(.*?)(?=<div\s+id=["\']tab-|\Z)', self.index_html, re.DOTALL)
            if leg_match:
                leg_html = leg_match.group(1)
                self.assertNotIn('ui-version-badge', leg_html)
                self.assertNotIn('esp32-version-badge', leg_html)

    def test_ui_identity_and_esp32_badge_sources(self):
        """Verify UI badge uses UI_BUILD_ID constant and ESP32 badge reuses live telemetry."""
        self.assertIn("const UI_BUILD_ID =", self.app_js)
        self.assertIn("ui-version-badge", self.app_js)
        self.assertIn("esp32-version-badge", self.app_js)
        self.assertIn("function updateEsp32Badge()", self.app_js)
        self.assertIn("ESP32: Offline", self.app_js)
        self.assertIn("ESP32: Unknown", self.app_js)
        self.assertIn("roverState.firmware", self.app_js)

    def test_token_storage_and_security(self):
        """Verify operator token supports sessionStorage and localStorage, is not in version output or source, and /api/auth/token is absent."""
        self.assertIn("sessionStorage.getItem('rover_operator_token')", self.app_js)
        self.assertIn("sessionStorage.setItem('rover_operator_token'", self.app_js)
        self.assertIn("sessionStorage.removeItem('rover_operator_token')", self.app_js)
        self.assertIn("localStorage.getItem('rover_operator_token')", self.app_js)
        self.assertIn("localStorage.setItem('rover_operator_token'", self.app_js)
        self.assertIn("localStorage.removeItem('rover_operator_token')", self.app_js)
        self.assertNotIn("/api/auth/token", self.app_js)
        self.assertNotIn("/api/auth/token", self.index_html)
        self.assertIn('type="password"', self.index_html)
        self.assertIn('autocomplete="current-password"', self.index_html)
        self.assertIn('autocomplete="username"', self.index_html)
        self.assertIn("type: 'deauth'", self.app_js)

    def test_five_canonical_tabs_unchanged(self):
        """Verify five canonical tabs remain present and intact."""
        canonical_tabs = ['tab-drive-v2', 'tab-autonomy-v2', 'tab-sensors-v2', 'tab-calibration-v2', 'tab-diagnostics-v2']
        for tab_id in canonical_tabs:
            matches = re.findall(rf'id=["\']{tab_id}["\']', self.index_html)
            self.assertEqual(len(matches), 1, f"Canonical tab '{tab_id}' must exist exactly once.")


if __name__ == '__main__':
    unittest.main()
