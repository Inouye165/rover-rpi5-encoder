# ==============================================================================
# test_localization_safety.py — Python runner for localization safety tests
# ==============================================================================

import os
import sys
import subprocess
import unittest

class TestLocalizationSafety(unittest.TestCase):

    def test_localization_safety_suite(self):
        """Execute the end-to-end Node.js localization safety test suite."""
        test_script = os.path.join(os.path.dirname(__file__), 'test_localization_safety.js')
        res = subprocess.run(['node', test_script], capture_output=True, text=True)
        print(res.stdout)
        if res.stderr:
            print(res.stderr, file=sys.stderr)
        self.assertEqual(res.returncode, 0, f"Localization safety test failed with exit code {res.returncode}")

if __name__ == '__main__':
    unittest.main()
