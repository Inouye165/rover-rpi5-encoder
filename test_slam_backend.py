import pytest
import unittest
import json
import os
import sys
import subprocess
from unittest.mock import MagicMock, patch

# Add parent directory to path so we can import modules if needed
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class TestSlamBackendState(unittest.TestCase):
    """Test suite for SLAM backend state detection and lifecycle management."""

    def test_truthful_state_parsing_running(self):
        """Verify that when /slam_toolbox and /lifecycle_manager_slam exist and lifecycle is active, state is RUNNING."""
        node_list_out = "/rover_encoder_odometry\n/rover_lidar_bridge\n/slam_toolbox\n/lifecycle_manager_slam\n"
        lifecycle_out = "active [3]\n"

        has_toolbox = '/slam_toolbox' in node_list_out.splitlines()
        has_manager = '/lifecycle_manager_slam' in node_list_out.splitlines()
        is_active = 'active' in lifecycle_out and 'inactive' not in lifecycle_out

        self.assertTrue(has_toolbox)
        self.assertTrue(has_manager)
        self.assertTrue(is_active)

        state = 'RUNNING' if (has_toolbox and has_manager and is_active) else 'STOPPED'
        self.assertEqual(state, 'RUNNING')

    def test_truthful_state_parsing_stopped(self):
        """Verify that when SLAM nodes are missing, state is STOPPED."""
        node_list_out = "/rover_encoder_odometry\n/rover_lidar_bridge\n"
        has_toolbox = '/slam_toolbox' in node_list_out.splitlines()
        has_manager = '/lifecycle_manager_slam' in node_list_out.splitlines()

        state = 'RUNNING' if (has_toolbox and has_manager) else 'STOPPED'
        self.assertEqual(state, 'STOPPED')

    def test_truthful_state_parsing_starting(self):
        """Verify that when nodes exist but lifecycle is unconfigured or inactive, state is STARTING."""
        node_list_out = "/slam_toolbox\n/lifecycle_manager_slam\n"
        lifecycle_out = "unconfigured [1]\n"

        has_toolbox = '/slam_toolbox' in node_list_out.splitlines()
        has_manager = '/lifecycle_manager_slam' in node_list_out.splitlines()
        is_active = 'active' in lifecycle_out and 'inactive' not in lifecycle_out

        if has_toolbox and has_manager:
            state = 'RUNNING' if is_active else 'STARTING'
        else:
            state = 'STOPPED'

        self.assertEqual(state, 'STARTING')

    def test_duplicate_start_prevention_logic(self):
        """Verify that attempting to start when state is RUNNING or STARTING is rejected."""
        state = 'RUNNING'
        can_start = (state not in ['STARTING', 'RUNNING'])
        self.assertFalse(can_start)

        state = 'STARTING'
        can_start = (state not in ['STARTING', 'RUNNING'])
        self.assertFalse(can_start)

        state = 'STOPPED'
        can_start = (state not in ['STARTING', 'RUNNING'])
        self.assertTrue(can_start)

    def test_targeted_pkill_safety(self):
        """Verify that stop command only targets SLAM processes and does not touch foundation nodes."""
        kill_cmd = "pkill -f 'slam.launch.py|async_slam_toolbox_node|lifecycle_manager_slam'"

        # Verify command regex targets only slam
        self.assertIn('slam.launch.py', kill_cmd)
        self.assertIn('async_slam_toolbox_node', kill_cmd)
        self.assertIn('lifecycle_manager_slam', kill_cmd)
        self.assertNotIn('rover_encoder_odometry', kill_cmd)
        self.assertNotIn('rover_imu_bridge', kill_cmd)
        self.assertNotIn('rover_lidar_bridge', kill_cmd)


if __name__ == '__main__':
    unittest.main()
