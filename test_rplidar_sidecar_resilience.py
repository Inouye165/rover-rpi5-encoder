# test_rplidar_sidecar_resilience.py
# Unit tests for rplidar_sidecar.py HTTP disconnect exception handling and parser log rate-limiting.

import io
import json
import socket
import sys
import unittest
from unittest.mock import MagicMock, patch

import rplidar_sidecar
from rplidar_sidecar import LiDARHTTPHandler, StreamRateLimiter, http_disconnect_counter, realign_counter


class TestLidarSidecarResilience(unittest.TestCase):

    def setUp(self):
        # Reset counters before each test
        rplidar_sidecar.http_disconnect_counter = 0
        rplidar_sidecar.realign_counter = 0

    def test_broken_pipe_error_during_send_does_not_propagate(self):
        """Test A: BrokenPipeError during _send() is handled cleanly without exception propagation."""
        handler = object.__new__(LiDARHTTPHandler)
        handler.wfile = MagicMock()
        handler.wfile.write.side_effect = BrokenPipeError("Broken pipe")
        handler.send_response = MagicMock()
        handler.send_header = MagicMock()
        handler.end_headers = MagicMock()

        # Should not raise exception
        handler._send(200, {"status": "ok"})
        self.assertGreater(rplidar_sidecar.http_disconnect_counter, 0, "Disconnect counter incremented")
        print(" -> PASS A: BrokenPipeError during _send() handled quietly")

    def test_connection_reset_error_does_not_propagate(self):
        """Test B: ConnectionResetError during _send() is handled cleanly."""
        handler = object.__new__(LiDARHTTPHandler)
        handler.wfile = MagicMock()
        handler.wfile.write.side_effect = ConnectionResetError("Connection reset by peer")
        handler.send_response = MagicMock()
        handler.send_header = MagicMock()
        handler.end_headers = MagicMock()

        # Should not raise exception
        handler._send(200, {"status": "ok"})
        self.assertGreater(rplidar_sidecar.http_disconnect_counter, 0, "Disconnect counter incremented")
        print(" -> PASS B: ConnectionResetError during _send() handled quietly")

    def test_no_traceback_printed_for_normal_client_disconnect(self):
        """Test C: handle_error ignores disconnect errors without printing tracebacks."""
        handler = object.__new__(LiDARHTTPHandler)
        mock_stderr = io.StringIO()

        with patch("sys.stderr", mock_stderr), patch("sys.exc_info") as mock_exc_info:
            mock_exc_info.return_value = (BrokenPipeError, BrokenPipeError("Broken pipe"), None)
            handler.handle_error(None, ("127.0.0.1", 12345))

        err_output = mock_stderr.getvalue()
        self.assertEqual(err_output, "", "No traceback output printed for BrokenPipeError")
        print(" -> PASS C: No traceback output for client disconnect")

    def test_unrelated_exceptions_are_not_swallowed(self):
        """Test D: Unrelated exceptions (KeyError, TypeError) raise normally and are not swallowed."""
        handler = object.__new__(LiDARHTTPHandler)
        handler.wfile = MagicMock()
        handler.wfile.write.side_effect = TypeError("Invalid payload type")
        handler.send_response = MagicMock()
        handler.send_header = MagicMock()
        handler.end_headers = MagicMock()

        with self.assertRaises(TypeError):
            handler._send(200, {"status": "ok"})
        print(" -> PASS D: Unrelated exceptions (TypeError) raise normally and are not swallowed")

    def test_parser_realignment_failures_are_rate_limited(self):
        """Test E: Repeated exact parser realignment logs are rate-limited, while generic words & methods pass through."""
        mock_output = MagicMock()
        mock_output.write = MagicMock()
        mock_output.isatty = MagicMock(return_value=True)
        limiter = StreamRateLimiter(mock_output)

        # 1. Attribute delegation via __getattr__
        self.assertEqual(limiter.isatty(), True, "__getattr__ transparently delegates isatty()")

        # 2. Generic messages containing partial words pass through transparently
        limiter.write("Token verification failed\n")
        limiter.write("Realigning motor controller\n")
        self.assertEqual(mock_output.write.call_count, 2, "Generic messages pass through without interception")
        self.assertEqual(rplidar_sidecar.realign_counter, 0, "Realign counter not incremented for generic words")

        # 3. Exact RPLIDAR parser messages are intercepted and rate-limited
        limiter.write("C bit verification failed. Realigning.\n")
        limiter.write("Verification bytes not matching\n")
        self.assertEqual(rplidar_sidecar.realign_counter, 2, "Exact RPLIDAR parser messages counted")
        print(" -> PASS E: Exact parser logs rate-limited; generic words and stream methods transparently delegated")

    def test_normal_http_response_path_works(self):
        """Test F: Normal HTTP response path succeeds and writes JSON body."""
        handler = object.__new__(LiDARHTTPHandler)
        output_buffer = io.BytesIO()
        handler.wfile = output_buffer
        handler.send_response = MagicMock()
        handler.send_header = MagicMock()
        handler.end_headers = MagicMock()

        payload = {"status": "ok", "scanHz": 7.1}
        handler._send(200, payload)

        written_data = output_buffer.getvalue().decode('utf-8')
        parsed = json.loads(written_data)
        self.assertEqual(parsed["status"], "ok")
        self.assertEqual(parsed["scanHz"], 7.1)
        print(" -> PASS F: Normal HTTP response path succeeds with valid JSON output")

    def test_lidar_offsets_match_ros_tf(self):
        """Test G: Verify sidecar default LiDAR offsets match verified ROS 2 static TF (x=0.03175, y=0.0)."""
        self.assertEqual(rplidar_sidecar.test_config["lidar_x_offset"], 0.03175)
        self.assertEqual(rplidar_sidecar.test_config["lidar_y_offset"], 0.0)
        print(" -> PASS G: Sidecar default LiDAR offsets match ROS static TF (x=0.03175, y=0.0)")

    def test_self_mask_math_and_rejection(self):
        """Test H: Verify self-masking math correctly rejects points inside chassis and preserves points outside."""
        # Config using default verified offsets (x=0.03175, y=0.0)
        config = dict(rplidar_sidecar.test_config)
        
        # 1. Point at 0 deg (front, +X) at 0.05m distance:
        # laser frame: x_l = 0.05, y_l = 0.0
        # body frame: x_r = 0.05 + 0.03175 = 0.08175m (inside chassis half-length 0.1343m) -> REJECTED
        pt_inside = {"angleDeg": 0.0, "distanceMm": 50, "quality": 15}
        
        # 2. Point at 0 deg (front, +X) at 0.50m distance:
        # laser frame: x_l = 0.50, y_l = 0.0
        # body frame: x_r = 0.50 + 0.03175 = 0.53175m (outside chassis half-length 0.1343m) -> PRESERVED
        pt_outside_fwd = {"angleDeg": 0.0, "distanceMm": 500, "quality": 15}
        pt_outside_fwd_neighbor = {"angleDeg": 1.0, "distanceMm": 501, "quality": 15} # neighbor for KDTree
        
        # 3. Point at 90 deg (native RPLIDAR CW 90 deg = ROS CCW 270 deg = -Y = right):
        # laser frame: x_l = 0.0, y_l = -0.50
        # body frame: x_r = 0.03175, y_r = -0.50 (outside chassis half-width 0.1311m) -> PRESERVED
        pt_outside_right = {"angleDeg": 90.0, "distanceMm": 500, "quality": 15}
        pt_outside_right_neighbor = {"angleDeg": 91.0, "distanceMm": 501, "quality": 15}

        # Test inside point rejection
        pts_in = [pt_inside]
        res_in = rplidar_sidecar.filter_and_process_scan(pts_in, config)
        self.assertEqual(len(res_in), 0, "Point inside chassis self-mask must be rejected")

        # Test outside point preservation and coordinate signs
        pts_out = [pt_outside_fwd, pt_outside_fwd_neighbor, pt_outside_right, pt_outside_right_neighbor]
        res_out = rplidar_sidecar.filter_and_process_scan(pts_out, config)
        self.assertGreaterEqual(len(res_out), 2, "Points outside chassis self-mask must be preserved")

        # Check forward (+X) point coordinates
        fwd_pts = [p for p in res_out if p[0] > 0.4]
        self.assertGreater(len(fwd_pts), 0)
        self.assertAlmostEqual(fwd_pts[0][0], 0.50, places=2, msg="Forward point +X laser coordinate")

        # Check right (-Y) point coordinates for 90 deg RPLIDAR CW scan
        right_pts = [p for p in res_out if p[1] < -0.4]
        self.assertGreater(len(right_pts), 0)
        self.assertAlmostEqual(right_pts[0][1], -0.50, places=2, msg="Right point -Y laser coordinate")
        print(" -> PASS H: Self-mask math correctly rejects points inside chassis and preserves points outside")


if __name__ == "__main__":
    unittest.main()

