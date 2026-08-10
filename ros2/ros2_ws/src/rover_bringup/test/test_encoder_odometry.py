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
        self.assertAlmostEqual(self.node.get_parameter('track_width_m').get_parameter_value().double_value, 0.3408575433, places=6)
        self.assertEqual(self.node.get_parameter('physical_track_width_m').get_parameter_value().double_value, 0.197)
        self.assertAlmostEqual(self.node.get_parameter('ticks_per_revolution').get_parameter_value().double_value, 1974.1666666667, places=4)
        self.assertEqual(self.node.get_parameter('wheel_diameter_m').get_parameter_value().double_value, 0.065)
        self.assertAlmostEqual(self.node.kinematics.track_width_m, 0.3408575433, places=6)
        self.assertEqual(self.node.kinematics.physical_track_width_m, 0.197)

    def test_runtime_config_and_server_defaults_match_code_default(self):
        """Verify that server.js, app.js, and launch/node default parameter track_width_m is 0.3408575433."""
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

        # Check server.js
        server_js = os.path.join(repo_root, 'server.js')
        if os.path.exists(server_js):
            with open(server_js, 'r', encoding='utf-8') as f:
                content = f.read()
            self.assertIn('let TRACK_WIDTH = 0.3408575433;', content)
            self.assertIn('effectiveTrackWidth = 0.3408575433', content)
            self.assertIn('const PHYSICAL_TRACK_WIDTH_M = 0.197;', content)

        # Check node default declared parameter
        param_val = self.node.get_parameter('track_width_m').get_parameter_value().double_value
        self.assertAlmostEqual(param_val, 0.3408575433, places=6)

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

    @patch('requests.Session.get')
    def test_poll_and_publish_duplicate_cached_sequence(self, mock_get):
        """Verify that unchanged cached sequence publishes odom/TF with zero velocity, 0 errors, and no double integration."""
        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {
            'ok': True,
            'schema_version': '1.0',
            'serialConnected': True,
            'timestamp': 1721697600000,
            'sequence': 50,
            'encoders': {'m1': 1000, 'm2': 1000, 'm3': 1000, 'm4': 1000}
        }
        mock_get.return_value = mock_response

        published_msgs = []
        self.node.odom_pub.publish = lambda msg: published_msgs.append(msg)

        # 1. Baseline initialization
        self.node._poll_and_publish()
        self.assertEqual(len(published_msgs), 1)

        # 2. First motion step (seq 51)
        mock_response.json.return_value['sequence'] = 51
        mock_response.json.return_value['encoders'] = {'m1': 2000, 'm2': 2000, 'm3': 2000, 'm4': 2000}
        self.node._poll_and_publish()
        self.assertEqual(len(published_msgs), 2)
        pose_x_step1 = published_msgs[-1].pose.pose.position.x
        self.assertGreater(pose_x_step1, 0.0)

        # 3. Duplicate cached sequence poll (seq 51 repeated)
        mock_response.json.return_value['sequence'] = 51
        mock_response.json.return_value['encoders'] = {'m1': 2000, 'm2': 2000, 'm3': 2000, 'm4': 2000}
        self.node._poll_and_publish()

        # Must still publish odom message (count=3)
        self.assertEqual(len(published_msgs), 3)
        msg_dup = published_msgs[-1]

        # Pose must NOT move (no double integration)
        self.assertAlmostEqual(msg_dup.pose.pose.position.x, pose_x_step1, places=6)

        # Published linear & angular velocities must be 0.0
        self.assertEqual(msg_dup.twist.twist.linear.x, 0.0)
        self.assertEqual(msg_dup.twist.twist.angular.z, 0.0)

        # Error counter must NOT increment
        self.assertEqual(self.node.consecutive_errors, 0)

        # 4. Resume with fresh sequence (seq 52)
        mock_response.json.return_value['sequence'] = 52
        mock_response.json.return_value['encoders'] = {'m1': 3000, 'm2': 3000, 'm3': 3000, 'm4': 3000}
        self.node._poll_and_publish()
        self.assertEqual(len(published_msgs), 4)
        pose_x_step3 = published_msgs[-1].pose.pose.position.x

        # Pose advances further cleanly
        self.assertGreater(pose_x_step3, pose_x_step1)

    def test_fresh_sample_dt_not_shortened_by_cached_polls(self):
        """Verify that velocity dt on a fresh sample is calculated from the PREVIOUS FRESH sample, not intermediate cached polls."""
        kin = self.node.kinematics
        kin.reset_pose()

        # 1. Fresh baseline at T=0.000
        ok, msg = kin.update([1000, 1000, 1000, 1000], timestamp_sec=0.000, sequence=10)
        self.assertTrue(ok)

        # 2. Intermediate cached poll at T=0.050 (seq 10 repeated)
        ok, msg = kin.update([1000, 1000, 1000, 1000], timestamp_sec=0.050, sequence=10)
        self.assertTrue(ok)
        self.assertEqual(msg, "NO_NEW_SAMPLE")

        # 3. Fresh sample at T=0.100 (seq 11) with +974 ticks (~0.102m)
        ok, msg = kin.update([1974, 1974, 1974, 1974], timestamp_sec=0.100, sequence=11)
        self.assertTrue(ok)

        # Distance ~0.102m over 0.100s -> v_x ~ 1.02 m/s. (If dt incorrectly used 0.050s, v_x would be ~2.04 m/s)
        self.assertAlmostEqual(kin.v_x, 1.02206, places=2)

    def test_velocity_retained_during_cached_polls_prevents_flicker(self):
        """Verify that velocity is retained during duplicate cached polls to eliminate 20 Hz flicker."""
        kin = self.node.kinematics
        kin.reset_pose()

        # Baseline
        kin.update([1000, 1000, 1000, 1000], timestamp_sec=0.000, sequence=10)

        # Fresh motion step -> v_x = 1.02 m/s
        kin.update([1974, 1974, 1974, 1974], timestamp_sec=0.100, sequence=11)
        v_fresh = kin.v_x
        self.assertGreater(v_fresh, 0.5)

        # Cached duplicate poll -> velocity must retain v_fresh (no flicker to 0.0)
        kin.update([1974, 1974, 1974, 1974], timestamp_sec=0.150, sequence=11)
        self.assertEqual(kin.v_x, v_fresh)

    def test_external_imu_accumulator_preserved_across_cached_polls(self):
        """Verify that external IMU yaw accumulator accumulates across cached polls and applies to next fresh sample."""
        kin = self.node.kinematics
        kin.reset_pose()

        # Baseline fresh sample at T=0.000
        kin.update([1000, 1000, 1000, 1000], timestamp_sec=0.000, sequence=10)

        # Simulate external IMU accumulator
        accumulated_gyro_yaw = 0.0

        # Sub-interval 1: Gyro accumulates +0.035 rad (+2 deg)
        accumulated_gyro_yaw += 0.035

        # Cached duplicate poll at T=0.050 -> NO_NEW_SAMPLE. Accumulator is NOT reset!
        ok, msg = kin.update([1000, 1000, 1000, 1000], timestamp_sec=0.050, sequence=10)
        self.assertEqual(msg, "NO_NEW_SAMPLE")
        self.assertAlmostEqual(accumulated_gyro_yaw, 0.035, places=6)

        # Sub-interval 2: Gyro accumulates another +0.035 rad (+2 deg)
        accumulated_gyro_yaw += 0.035
        self.assertAlmostEqual(accumulated_gyro_yaw, 0.070, places=6)

        # Fresh sample N+1 at T=0.100: pass full accumulated external_d_yaw (+0.070 rad)
        ok, msg = kin.update([1050, 1050, 1050, 1050], timestamp_sec=0.100, sequence=11, external_d_yaw=accumulated_gyro_yaw)
        self.assertTrue(ok)

        # Confirm full accumulated gyro yaw (+0.070 rad) was applied exactly once to pose yaw
        self.assertAlmostEqual(kin.yaw, 0.070, places=5)

        # Reset external accumulator after fresh sample consume
        accumulated_gyro_yaw = 0.0
        self.assertEqual(accumulated_gyro_yaw, 0.0)

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

    def test_compute_integrated_gyro_yaw_math_and_timing(self):
        """Verify trapezoidal integration of gyro Z samples over an encoder update interval [t_prev, t_curr]."""
        # Feed 50Hz IMU samples: gz = 0.5 rad/s from T=100.00 to T=100.10
        for i in range(6):
            t = 100.00 + i * 0.02  # 100.00, 100.02, 100.04, 100.06, 100.08, 100.10
            msg = Imu()
            msg.header.stamp.sec = int(t)
            msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
            msg.angular_velocity.z = 0.5
            self.node._imu_callback(msg)

        # Integrate over [100.00, 100.10] -> expected 0.5 rad/s * 0.10s = 0.050 rad
        d_yaw = self.node._compute_integrated_gyro_yaw(100.00, 100.10)
        self.assertIsNotNone(d_yaw)
        self.assertAlmostEqual(d_yaw, 0.050, places=5)

    def test_gyro_sign_convention(self):
        """Verify positive gyro Z produces positive CCW yaw delta, negative gyro Z produces negative CW yaw delta."""
        # Positive Z gyro (+1.0 rad/s)
        msg_pos = Imu()
        msg_pos.header.stamp.sec = 200
        msg_pos.header.stamp.nanosec = 0
        msg_pos.angular_velocity.z = 1.0
        self.node._imu_callback(msg_pos)

        msg_pos2 = Imu()
        msg_pos2.header.stamp.sec = 200
        msg_pos2.header.stamp.nanosec = 100000000  # 0.1s
        msg_pos2.angular_velocity.z = 1.0
        self.node._imu_callback(msg_pos2)

        d_yaw_pos = self.node._compute_integrated_gyro_yaw(200.0, 200.1)
        self.assertGreater(d_yaw_pos, 0.0)
        self.assertAlmostEqual(d_yaw_pos, 0.1, places=4)

        # Clear buffer
        with self.node._imu_lock:
            self.node._imu_gyro_buffer.clear()

        # Negative Z gyro (-1.0 rad/s)
        msg_neg = Imu()
        msg_neg.header.stamp.sec = 300
        msg_neg.header.stamp.nanosec = 0
        msg_neg.angular_velocity.z = -1.0
        self.node._imu_callback(msg_neg)

        msg_neg2 = Imu()
        msg_neg2.header.stamp.sec = 300
        msg_neg2.header.stamp.nanosec = 100000000  # 0.1s
        msg_neg2.angular_velocity.z = -1.0
        self.node._imu_callback(msg_neg2)

        d_yaw_neg = self.node._compute_integrated_gyro_yaw(300.0, 300.1)
        self.assertLess(d_yaw_neg, 0.0)
        self.assertAlmostEqual(d_yaw_neg, -0.1, places=4)

    def test_imu_stale_or_missing_fallback(self):
        """Verify that when IMU samples are stale or missing, _compute_integrated_gyro_yaw returns None."""
        # Case 1: Empty buffer
        self.assertIsNone(self.node._compute_integrated_gyro_yaw(100.0, 100.05))

        # Case 2: Buffer timestamp too old for window
        msg = Imu()
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = 0
        msg.angular_velocity.z = 0.5
        self.node._imu_callback(msg)

        msg2 = Imu()
        msg2.header.stamp.sec = 10
        msg2.header.stamp.nanosec = 50000000
        msg2.angular_velocity.z = 0.5
        self.node._imu_callback(msg2)

        # Querying interval [100.0, 100.05] when buffer only has t=10.0s -> None
        self.assertIsNone(self.node._compute_integrated_gyro_yaw(100.0, 100.05))

        # Case 3: Stale wall clock (>0.5s since last IMU callback)
        msg_stale = Imu()
        msg_stale.header.stamp.sec = 500
        msg_stale.header.stamp.nanosec = 0
        msg_stale.angular_velocity.z = 0.5
        self.node._imu_callback(msg_stale)
        self.node._last_imu_recv_time_sec = time.time() - 2.0  # Simulate 2.0s age

        self.assertIsNone(self.node._compute_integrated_gyro_yaw(500.0, 500.05))


if __name__ == '__main__':
    unittest.main()

