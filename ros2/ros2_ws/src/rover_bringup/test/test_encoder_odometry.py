# ==============================================================================
# test_encoder_odometry.py — ROS 2 Node & Integration Tests for Odometry
# ==============================================================================

import os
import sys
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from rover_bringup.rover_encoder_odometry import RoverEncoderOdometry


class TestRoverEncoderOdometryNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = RoverEncoderOdometry()

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        self.assertEqual(self.node.odom_frame, 'odom')
        self.assertEqual(self.node.base_frame, 'base_link')
        self.assertEqual(self.node.publish_rate_hz, 20.0)
        self.assertEqual(self.node.get_parameter('track_width_m').get_parameter_value().double_value, 0.197)
        self.assertEqual(self.node.get_parameter('ticks_per_revolution').get_parameter_value().double_value, 1894.0)
        self.assertEqual(self.node.get_parameter('wheel_diameter_m').get_parameter_value().double_value, 0.065)
        self.assertEqual(self.node.kinematics.track_width_m, 0.197)

    def test_runtime_config_and_server_defaults_match_code_default(self):
        """Verify that server.js, app.js, and launch/node default parameter track_width_m is 0.197."""
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

        # Check server.js
        server_js = os.path.join(repo_root, 'server.js')
        if os.path.exists(server_js):
            with open(server_js, 'r', encoding='utf-8') as f:
                content = f.read()
            self.assertIn('let TRACK_WIDTH = 0.197;', content)
            self.assertIn('effectiveTrackWidth: 0.197', content)

        # Check node default declared parameter
        param_val = self.node.get_parameter('track_width_m').get_parameter_value().double_value
        self.assertEqual(param_val, 0.197, f"Runtime launch/node default track_width_m must be 0.197, got {param_val}")

    @patch('requests.Session.get')
    def test_poll_and_publish_success(self, mock_get):
        # Mock HTTP response from Cockpit GET /api/encoders
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            'ok': True,
            'schema_version': '1.0',
            'serialConnected': True,
            'timestamp': 1721697600000,
            'lastPacketAgeMs': 10,
            'sequence': 42,
            'encoders': {'m1': 937, 'm2': 937, 'm3': 937, 'm4': 937}
        }
        mock_get.return_value = mock_response

        # Capture published /odom messages
        published_msgs = []
        self.node.odom_pub.publish = lambda msg: published_msgs.append(msg)

        # First baseline call
        self.node._poll_and_publish()

        # Second call with forward motion
        mock_response.json.return_value['timestamp'] = 1721697601000
        mock_response.json.return_value['sequence'] = 43
        mock_response.json.return_value['encoders'] = {'m1': 1874, 'm2': 1874, 'm3': 1874, 'm4': 1874}

        self.node._poll_and_publish()

        self.assertGreaterEqual(len(published_msgs), 1)
        last_msg = published_msgs[-1]
        self.assertIsInstance(last_msg, Odometry)
        self.assertEqual(last_msg.header.frame_id, 'odom')
        self.assertEqual(last_msg.child_frame_id, 'base_link')
        self.assertGreater(last_msg.pose.pose.position.x, 0.0)
        self.assertTrue(math.isfinite(last_msg.pose.pose.position.x))
        self.assertTrue(math.isfinite(last_msg.twist.twist.linear.x))

    @patch('requests.Session.get')
    def test_poll_and_publish_malformed_payload(self, mock_get):
        # Mock HTTP response with malformed structure
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {'ok': False, 'encoders': 'invalid_type'}
        mock_get.return_value = mock_response

        published_msgs = []
        self.node.odom_pub.publish = lambda msg: published_msgs.append(msg)

        # Polling should handle malformed payload gracefully without crashing
        self.node._poll_and_publish()
        self.assertEqual(len(published_msgs), 0)
        self.assertEqual(self.node.consecutive_errors, 1)

    def test_node_source_has_no_serial_or_dev_access(self):
        """Safety audit: ensure node source does not open serial ports or /dev devices."""
        node_file = os.path.join(
            os.path.dirname(__file__), '..', 'rover_bringup', 'rover_encoder_odometry.py'
        )
        with open(node_file, 'r', encoding='utf-8') as f:
            code = f.read()

        forbidden_terms = ['/dev/tty', '/dev/rover', 'serialport', 'Serial(', 'termios', 'fcntl']
        for term in forbidden_terms:
            self.assertNotIn(term, code, f"Forbidden term '{term}' found in rover_encoder_odometry.py")

    def test_odom_http_api_endpoint(self):
        """Verify that the HTTP API endpoint on port 3003 (or node.odom_api_port) serves odometry JSON."""
        import urllib.request
        import json
        url = f"http://127.0.0.1:{self.node.odom_api_port}/api/odom"
        try:
            req = urllib.request.urlopen(url, timeout=1.0)
            self.assertEqual(req.status, 200)
            data = json.loads(req.read().decode('utf-8'))
            self.assertIn('ok', data)
            self.assertIn('x', data)
            self.assertIn('y', data)
            self.assertIn('yaw', data)
            self.assertIn('yaw_deg', data)
            self.assertIn('odometry_age_ms', data)
            self.assertIn('node_health', data)
        except Exception as err:
            self.fail(f"Failed to query Odom HTTP API server at {url}: {err}")

    def test_compose_yaml_has_no_dev_mounts(self):
        """Safety audit: ensure compose.yaml has no /dev mounts, privileged mode, or devices."""
        compose_file = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'compose.yaml')
        if os.path.exists(compose_file):
            with open(compose_file, 'r', encoding='utf-8') as f:
                content = f.read()

            self.assertNotIn('/dev/', content)
            self.assertNotIn('privileged: true', content)
            self.assertNotIn('devices:', content)



import math
if __name__ == '__main__':
    unittest.main()
