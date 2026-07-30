# ==============================================================================
# test_cmd_vel_safety.py — Behavioral & Runtime Verification for ROS 2 /cmd_vel
# ==============================================================================

import os
import sys
import subprocess
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class TestCmdVelSafety(unittest.TestCase):

    def test_behavioral_node_suite(self):
        """Execute the end-to-end Node.js behavioral verification test suite."""
        test_script = os.path.join(os.path.dirname(__file__), 'test_cmd_vel_behavior.js')
        res = subprocess.run(['node', test_script], capture_output=True, text=True)
        print(res.stdout)
        if res.stderr:
            print(res.stderr, file=sys.stderr)
        self.assertEqual(res.returncode, 0, f"Behavioral test suite failed with exit code {res.returncode}")


if __name__ == '__main__':
    unittest.main()
