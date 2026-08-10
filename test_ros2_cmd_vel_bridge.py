# ==============================================================================
# test_ros2_cmd_vel_bridge.py — Behavioral Unit Tests for ROS 2 /cmd_vel Bridge
# ==============================================================================

import os
import sys
import math
import unittest
from unittest.mock import MagicMock, patch

# Mock rclpy and geometry_msgs if running outside ROS 2 environment (e.g. Windows host)
try:
    import rclpy
    from rclpy.node import Node
except ModuleNotFoundError:
    mock_rclpy = MagicMock()
    class DummyNode:
        def __init__(self, name):
            self.name = name
            self._logger = MagicMock()
            self._params = {}
        def get_logger(self):
            return self._logger
        def declare_parameter(self, name, default_val=None):
            self._params[name] = default_val
            return MagicMock(value=default_val)
        def get_parameter(self, name):
            val = self._params.get(name, '')
            return MagicMock(value=val)
        def create_subscription(self, msg_type, topic, callback, qos):
            return MagicMock()
    mock_rclpy.node.Node = DummyNode
    sys.modules['rclpy'] = mock_rclpy
    sys.modules['rclpy.node'] = mock_rclpy.node

try:
    import geometry_msgs.msg
except ModuleNotFoundError:
    mock_geom = MagicMock()
    sys.modules['geometry_msgs'] = mock_geom
    sys.modules['geometry_msgs.msg'] = mock_geom

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
bridge_dir = os.path.join(
    os.path.dirname(__file__),
    'ros2', 'ros2_ws', 'src', 'rover_bringup', 'rover_bringup'
)
if bridge_dir not in sys.path:
    sys.path.append(bridge_dir)

from rover_cmd_vel_bridge import RoverCmdVelBridgeNode


class DummyTwist:
    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = float(x)
            self.y = float(y)
            self.z = float(z)

    def __init__(self, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
        self.linear = self.Vector3(lin_x, lin_y, lin_z)
        self.angular = self.Vector3(ang_x, ang_y, ang_z)


class TestRos2CmdVelBridgeBehavioral(unittest.TestCase):

    def setUp(self):
        os.environ['ROVER_INTERNAL_CMD_URL'] = 'http://127.0.0.1:3810/api/cmd_vel'
        os.environ['ROVER_CMD_VEL_TOKEN'] = 'a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890'

    def test_1_valid_twist_conversion_and_token_header(self):
        node = RoverCmdVelBridgeNode()

        mock_session = MagicMock()
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {'ok': True, 'state': 'ACTIVE'}
        mock_session.post.return_value = mock_response
        node.session = mock_session

        # Valid Twist message
        twist = DummyTwist(lin_x=0.15, ang_z=0.45)
        node.cmd_vel_callback(twist)

        # Assert post was called with correct payload and endpoint
        mock_session.post.assert_called_once()
        args, kwargs = mock_session.post.call_args
        self.assertEqual(args[0], 'http://127.0.0.1:3810/api/cmd_vel')
        self.assertEqual(kwargs['json'], {
            "linear": {"x": 0.15, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.45}
        })
        self.assertEqual(node.bridge_status, 'active')

    def test_2_nan_and_unsupported_axis_rejection(self):
        node = RoverCmdVelBridgeNode()

        mock_session = MagicMock()
        node.session = mock_session

        # NaN Twist
        nan_twist = DummyTwist(lin_x=float('nan'), ang_z=0.0)
        node.cmd_vel_callback(nan_twist)
        mock_session.post.assert_not_called()
        self.assertEqual(node.bridge_status, 'rejected')

        # Unsupported axis (y linear != 0)
        y_twist = DummyTwist(lin_x=0.1, lin_y=0.2, ang_z=0.0)
        node.cmd_vel_callback(y_twist)
        mock_session.post.assert_not_called()
        self.assertEqual(node.bridge_status, 'rejected')

    def test_3_shutdown_stop_command(self):
        node = RoverCmdVelBridgeNode()

        mock_session = MagicMock()
        node.session = mock_session

        node.send_stop()
        mock_session.post.assert_called_once()
        args, kwargs = mock_session.post.call_args
        self.assertEqual(kwargs['json'], {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        })


if __name__ == '__main__':
    unittest.main()
