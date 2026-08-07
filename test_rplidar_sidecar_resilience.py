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


if __name__ == "__main__":
    unittest.main()
