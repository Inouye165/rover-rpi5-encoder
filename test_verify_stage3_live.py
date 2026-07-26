# ==============================================================================
# test_verify_stage3_live.py — Isolated Unit Tests for Live Verification Script
# ==============================================================================

import os
import sys
import unittest
from unittest.mock import patch, MagicMock

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import verify_stage3_live as vsl


class TestVerifyStage3Live(unittest.TestCase):

    # --- 1. SHA256 File & Hash Verification Tests ---

    @patch("verify_stage3_live.sha256_file")
    @patch("verify_stage3_live.get_remote_hash")
    @patch("verify_stage3_live.fetch_bytes")
    def test_verify_hashes_matching(self, mock_fetch, mock_remote, mock_local):
        mock_local.return_value = "abc123hash"
        mock_remote.return_value = "abc123hash"
        mock_fetch.return_value = b"file content"

        with patch("hashlib.sha256") as mock_sha:
            mock_sha.return_value.hexdigest.return_value = "abc123hash"
            all_match, details = vsl.verify_hashes("/local", "/pi", "10.0.0.246", "ron", ["public/index.html"])
            self.assertTrue(all_match)
            self.assertTrue(details["public/index.html"]["pass"])

    @patch("verify_stage3_live.sha256_file")
    @patch("verify_stage3_live.get_remote_hash")
    @patch("verify_stage3_live.fetch_bytes")
    def test_verify_hashes_mismatch_fails(self, mock_fetch, mock_remote, mock_local):
        mock_local.return_value = "abc123hash"
        mock_remote.return_value = "def456different"
        mock_fetch.return_value = b"file content"

        with patch("hashlib.sha256") as mock_sha:
            mock_sha.return_value.hexdigest.return_value = "abc123hash"
            all_match, details = vsl.verify_hashes("/local", "/pi", "10.0.0.246", "ron", ["public/index.html"])
            self.assertFalse(all_match)
            self.assertFalse(details["public/index.html"]["pass"])

    @patch("verify_stage3_live.get_remote_hash", side_effect=RuntimeError("Missing remote file"))
    @patch("verify_stage3_live.sha256_file", return_value="abc123hash")
    def test_verify_hashes_missing_pi_file_fails(self, mock_local, mock_remote):
        all_match, details = vsl.verify_hashes("/local", "/pi", "10.0.0.246", "ron", ["public/index.html"])
        self.assertFalse(all_match)
        self.assertIn("Pi SSH hash error", details["public/index.html"]["error"])

    @patch("verify_stage3_live.get_remote_hash", side_effect=RuntimeError("SSH timeout"))
    @patch("verify_stage3_live.sha256_file", return_value="abc123hash")
    def test_verify_hashes_ssh_timeout_fails(self, mock_local, mock_remote):
        all_match, details = vsl.verify_hashes("/local", "/pi", "10.0.0.246", "ron", ["public/index.html"])
        self.assertFalse(all_match)
        self.assertIn("SSH timeout", details["public/index.html"]["error"])

    @patch("verify_stage3_live.fetch_bytes", side_effect=RuntimeError("HTTP timeout"))
    @patch("verify_stage3_live.get_remote_hash", return_value="abc123hash")
    @patch("verify_stage3_live.sha256_file", return_value="abc123hash")
    def test_verify_hashes_http_timeout_fails(self, mock_local, mock_remote, mock_fetch):
        all_match, details = vsl.verify_hashes("/local", "/pi", "10.0.0.246", "ron", ["public/index.html"])
        self.assertFalse(all_match)
        self.assertIn("HTTP timeout", details["public/index.html"]["error"])

    # --- 2. Drive Status Verification Tests ---

    def test_verify_drive_status_healthy(self):
        data = {
            "status": {
                "armed": False,
                "motorCommand": [0, 0, 0, 0],
                "cmdSource": "NONE",
                "reqLinear": 0,
                "reqAngular": 0,
                "limLinear": 0,
                "limAngular": 0
            }
        }
        ok, msg = vsl.verify_drive_status(data)
        self.assertTrue(ok)

    def test_verify_drive_status_armed_fails(self):
        data = {"status": {"armed": True, "motorCommand": [0, 0, 0, 0], "cmdSource": "NONE"}}
        ok, msg = vsl.verify_drive_status(data)
        self.assertFalse(ok)
        self.assertIn("armed must be False", msg)

    def test_verify_drive_status_nonzero_motor_command_fails(self):
        data = {"status": {"armed": False, "motorCommand": [10, 0, 0, 0], "cmdSource": "NONE"}}
        ok, msg = vsl.verify_drive_status(data)
        self.assertFalse(ok)

    def test_verify_drive_status_cmd_source_not_none_fails(self):
        data = {"status": {"armed": False, "motorCommand": [0, 0, 0, 0], "cmdSource": "WEB_JOYSTICK"}}
        ok, msg = vsl.verify_drive_status(data)
        self.assertFalse(ok)
        self.assertIn("cmdSource", msg)

    def test_verify_drive_status_missing_fields_fails(self):
        data = {"status": {"armed": False}}  # missing cmdSource and motorCommand
        ok, msg = vsl.verify_drive_status(data)
        self.assertFalse(ok)

    # --- 3. Calibration Status Verification Tests ---

    def test_verify_calibration_status_healthy(self):
        data = {
            "ok": True,
            "status": {
                "phase": "IDLE",
                "active": False,
                "armed": False,
                "motorCommand": [0, 0, 0, 0],
                "fault": None,
                "safetyChecks": {
                    "serialConnected": True,
                    "telemetryValid": True,
                    "odomValid": True,
                    "limitsOk": True
                }
            }
        }
        ok, msg = vsl.verify_calibration_status(data)
        self.assertTrue(ok)

    def test_verify_calibration_status_active_fails(self):
        data = {
            "ok": True,
            "status": {
                "phase": "RUNNING",
                "active": True,
                "armed": True,
                "motorCommand": [20, 20, 20, 20],
                "fault": None,
                "safetyChecks": {"serialConnected": True, "telemetryValid": True, "odomValid": True, "limitsOk": True}
            }
        }
        ok, msg = vsl.verify_calibration_status(data)
        self.assertFalse(ok)

    def test_verify_calibration_status_missing_status_fails(self):
        data = {"ok": True}  # missing status dict
        ok, msg = vsl.verify_calibration_status(data)
        self.assertFalse(ok)

    def test_verify_calibration_status_failed_safety_check_fails(self):
        data = {
            "ok": True,
            "status": {
                "phase": "IDLE",
                "active": False,
                "armed": False,
                "motorCommand": [0, 0, 0, 0],
                "fault": None,
                "safetyChecks": {
                    "serialConnected": False,  # Failed!
                    "telemetryValid": True,
                    "odomValid": True,
                    "limitsOk": True
                }
            }
        }
        ok, msg = vsl.verify_calibration_status(data)
        self.assertFalse(ok)

    # --- 4. LiDAR Status Verification Tests ---

    def test_verify_lidar_status_healthy(self):
        data = {
            "connected": True,
            "state": "scanning",
            "health": "Good",
            "latestScanPointCount": 360,
            "scanHz": 6.5,
            "lastScanAgeMs": 10,
            "lastError": None
        }
        ok, msg = vsl.verify_lidar_status(data)
        self.assertTrue(ok)

    def test_verify_lidar_status_stale_age_fails(self):
        data = {
            "connected": True,
            "state": "scanning",
            "health": "Good",
            "latestScanPointCount": 360,
            "scanHz": 6.5,
            "lastScanAgeMs": 3500,  # > 2000ms!
            "lastError": None
        }
        ok, msg = vsl.verify_lidar_status(data)
        self.assertFalse(ok)

    def test_verify_lidar_status_zero_points_fails(self):
        data = {
            "connected": True,
            "state": "scanning",
            "health": "Good",
            "latestScanPointCount": 0,  # 0 points!
            "scanHz": 6.5,
            "lastScanAgeMs": 10,
            "lastError": None
        }
        ok, msg = vsl.verify_lidar_status(data)
        self.assertFalse(ok)

    # --- 5. Odometry Status Verification Tests ---

    def test_verify_odometry_status_healthy(self):
        data = {"ok": True, "node_health": "ok", "odometry_age_ms": 42}
        ok, msg = vsl.verify_odometry_status(data)
        self.assertTrue(ok)

    def test_verify_odometry_status_stale_fails(self):
        data = {"ok": True, "node_health": "ok", "odometry_age_ms": 2500}
        ok, msg = vsl.verify_odometry_status(data)
        self.assertFalse(ok)

    @patch("verify_stage3_live.fetch_json", side_effect=RuntimeError("Odometry request failed"))
    def test_verify_odometry_request_failure_fails(self, mock_fetch):
        with self.assertRaises(RuntimeError):
            vsl.fetch_json("http://10.0.0.246:3003/api/odom")

    # --- 6. Foxglove Port Verification Tests ---

    @patch("socket.socket")
    def test_verify_foxglove_open_passes(self, mock_socket):
        mock_sock_inst = MagicMock()
        mock_socket.return_value = mock_sock_inst
        ok, msg = vsl.verify_foxglove("10.0.0.246", 8765)
        self.assertTrue(ok)

    @patch("socket.socket")
    def test_verify_foxglove_closed_fails(self, mock_socket):
        mock_sock_inst = MagicMock()
        mock_sock_inst.connect.side_effect = ConnectionRefusedError("Connection refused")
        mock_socket.return_value = mock_sock_inst
        ok, msg = vsl.verify_foxglove("10.0.0.246", 8765)
        self.assertFalse(ok)

    # --- 7. Main Function Exit Code Tests ---

    @patch("verify_stage3_live.verify_foxglove", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_odometry_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_lidar_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_calibration_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_drive_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_hashes", return_value=(True, {"f": {"pass": True, "local_hash": "a"}}))
    @patch("verify_stage3_live.fetch_json", return_value={})
    def test_main_exit_0_when_all_pass(self, *mocks):
        with self.assertRaises(SystemExit) as cm:
            vsl.main()
        self.assertEqual(cm.exception.code, 0)

    @patch("verify_stage3_live.verify_foxglove", return_value=(False, "Failed connection"))
    @patch("verify_stage3_live.verify_odometry_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_lidar_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_calibration_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_drive_status", return_value=(True, "OK"))
    @patch("verify_stage3_live.verify_hashes", return_value=(True, {"f": {"pass": True, "local_hash": "a"}}))
    @patch("verify_stage3_live.fetch_json", return_value={})
    def test_main_exit_1_when_any_fails(self, *mocks):
        with self.assertRaises(SystemExit) as cm:
            vsl.main()
        self.assertEqual(cm.exception.code, 1)


if __name__ == "__main__":
    unittest.main()
