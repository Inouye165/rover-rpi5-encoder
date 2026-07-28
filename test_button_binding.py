import unittest
import re

class TestAutoCalibrationUIFlow(unittest.TestCase):
    def setUp(self):
        with open('public/index.html', 'r', encoding='utf-8') as f:
            self.html = f.read()
        with open('public/app.js', 'r', encoding='utf-8') as f:
            self.js = f.read()

    def test_auto_1m_forward_invokes_confirmation_flow(self):
        # Find the button
        btn_match = re.search(r'<button id="btn-auto-fwd-1m"[^>]*onclick="([^"]+)"', self.html)
        self.assertIsNotNone(btn_match, "Auto 1 m Forward button is missing.")
        onclick_handler = btn_match.group(1)
        self.assertEqual(onclick_handler, "promptAutoCalib('forward_1m')", "Button does not invoke correct flow with right ID.")

        # Ensure modal exists
        self.assertIn('id="modal-auto-calib-confirm"', self.html, "Confirmation modal is missing from HTML.")

    def test_confirm_flow_constructs_exact_post(self):
        # Extract confirmAndStartAutoCalib function
        match = re.search(r'async function confirmAndStartAutoCalib\(\) \{(.*?)\n\}', self.js, re.DOTALL)
        self.assertIsNotNone(match, "confirmAndStartAutoCalib function missing.")
        func_body = match.group(1)

        # Ensure exact POST endpoint is used
        self.assertIn("fetch('/api/calibration/auto/start', {", func_body)
        self.assertIn("method: 'POST'", func_body)
        self.assertIn("body: JSON.stringify({ test: testType })", func_body)

        # Ensure no motor/movement endpoint is invoked
        self.assertNotIn("/api/motor", func_body)
        self.assertNotIn("/api/cmd_vel", func_body)

    def test_cancel_sends_no_request(self):
        # Extract closeAutoCalibModal function
        match = re.search(r'function closeAutoCalibModal\(\) \{(.*?)\n\}', self.js, re.DOTALL)
        self.assertIsNotNone(match, "closeAutoCalibModal function missing.")
        func_body = match.group(1)

        # Ensure it only hides the modal and resets the pending test
        self.assertIn("pendingAutoCalibTest = null;", func_body)
        self.assertIn("modal.style.display = 'none';", func_body)
        self.assertNotIn("fetch", func_body) # No fetch requests

if __name__ == '__main__':
    unittest.main()
