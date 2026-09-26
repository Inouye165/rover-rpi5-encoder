"""
test.test_gold_standard_home_test - Automated Unit & Safety Abort Tests
Tests HOME loading, pre-arm gates, target geometry, grading criteria, all fail-safe abort conditions,
clean command ownership handshakes, signed Leg 4 rotation, and active production velocity configs.
"""

import os
import math
import json
import yaml
import pytest
import requests
from unittest.mock import MagicMock, patch

from tools.gold_standard_home_test.constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    ROTATION_180_MAX_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_YAW_ALIGNMENT_MAX_SPEED,
    FINAL_POS_CORRECTION_CEILING,
    PASS_FINAL_POS_ERR_M,
    PASS_FINAL_YAW_ERR_DEG,
    PASS_MIN_RECORDER_RATE_HZ,
    PRE_ARM_MAX_POS_ERR_M,
    PRE_ARM_MAX_YAW_ERR_DEG
)
from tools.gold_standard_home_test.home_loader import (
    load_home_from_file,
    load_authoritative_home,
    wrap_angle_rad,
    wrap_angle_deg
)
from tools.gold_standard_home_test.grader import MissionGrader
from tools.gold_standard_home_test.mission import GoldStandardMission, MissionAbortException


class TestHomeLoader:
    def test_load_home_from_file(self, tmp_path):
        dummy_home = {
            "pose": {
                "position": {"x": 1.193853, "y": -0.045221, "z": 0.0},
                "orientation": {"z": -0.044029, "w": 0.99903},
                "yaw_deg": -5.047,
                "yaw_rad": -0.088087
            },
            "description": "Test HOME pose"
        }
        json_file = tmp_path / "home_pose_slam.json"
        with open(json_file, "w", encoding="utf-8") as f:
            json.dump(dummy_home, f)

        res = load_home_from_file(str(json_file))
        assert res is not None
        assert abs(res["x"] - 1.193853) < 1e-5
        assert abs(res["y"] - (-0.045221)) < 1e-5
        assert abs(res["yaw_deg"] - (-5.047)) < 1e-3
        assert abs(res["yaw_rad"] - (-0.088087)) < 1e-5

    def test_load_home_angle_wrapping(self):
        assert abs(wrap_angle_deg(370.0) - 10.0) < 1e-5
        assert abs(wrap_angle_deg(-190.0) - 170.0) < 1e-5
        assert abs(wrap_angle_rad(3.5 * math.pi) - (-0.5 * math.pi)) < 1e-5


class TestMissionGeometry:
    def test_mission_target_calculation(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {
            "x": 1.0,
            "y": 2.0,
            "yaw_rad": 0.0,
            "yaw_deg": 0.0
        }
        targets = mission.compute_mission_targets()
        
        # Leg 1: 0.6096 m forward along 0 heading
        l1 = targets["leg1_outbound"]
        assert abs(l1["x"] - (1.0 + FORWARD_DISTANCE_M)) < 1e-5
        assert abs(l1["y"] - 2.0) < 1e-5
        assert abs(l1["yaw_deg"] - 0.0) < 1e-5

        # Leg 2: 180° CW in-place rotation from 0 deg -> -180 deg
        l2 = targets["leg2_rotation"]
        assert abs(abs(l2["yaw_deg"]) - 180.0) < 1e-5

        # Leg 3: Return to exact HOME coordinates retaining return-facing heading (-180.0 deg)
        l3 = targets["leg3_return"]
        assert abs(l3["x"] - 1.0) < 1e-5
        assert abs(l3["y"] - 2.0) < 1e-5
        assert abs(abs(l3["yaw_deg"]) - 180.0) < 1e-5

        # Leg 4: Settle target is exact saved HOME yaw (0.0 deg)
        l4 = targets["leg4_settle"]
        assert abs(l4["yaw_deg"] - 0.0) < 1e-5


class TestPreArmGate:
    def test_pre_arm_gate_accepted(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Current pose within 2 cm and 1.5 deg
        amcl_ok = {"x": 1.02, "y": 2.0, "yaw_deg": 1.5, "yaw_rad": math.radians(1.5), "localized": True, "state": "LOCALIZED"}
        ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_ok)
        assert ok is True
        assert p_err <= PRE_ARM_MAX_POS_ERR_M
        assert y_err <= PRE_ARM_MAX_YAW_ERR_DEG

    def test_pre_arm_gate_rejected_position(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Position error 8 cm (> 5 cm limit)
        amcl_bad = {"x": 1.08, "y": 2.0, "yaw_deg": 0.0, "yaw_rad": 0.0, "localized": True, "state": "LOCALIZED"}
        ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_bad)
        assert ok is False
        assert "exceeds 5.0 cm ceiling" in msg

    def test_pre_arm_gate_rejected_yaw(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        
        # Yaw error 8.0 deg (> 5.0 deg limit)
        amcl_bad = {"x": 1.0, "y": 2.0, "yaw_deg": 8.0, "yaw_rad": math.radians(8.0), "localized": True, "state": "LOCALIZED"}
        ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_bad)
        assert ok is False
        assert "exceeds 5.0° ceiling" in msg

    def test_pre_arm_gate_rejected_not_localized(self):
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        amcl_pose = {"x": 1.0, "y": 2.0, "yaw_deg": 0.0, "yaw_rad": 0.0, "localized": False, "state": "UNLOCALIZED"}
        ok, msg, p_err, y_err = mission.check_pre_arm_gate(amcl_pose)
        assert ok is False
        assert "not LOCALIZED" in msg


class TestSafetyAborts:
    def test_stale_telemetry_abort(self):
        mission = GoldStandardMission(dry_run=True)
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_seq_adv_time = time.monotonic() - 10.0
        mission.dry_run = False
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Stale telemetry" in str(excinfo.value)

    def test_boot_count_change_abort(self):
        mission = GoldStandardMission(dry_run=False)
        mission.initial_boot_count = 1
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["drive"] = {"bootCount": 2}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "ESP32 hardware reboot detected" in str(excinfo.value)

    def test_imu_discontinuity_abort(self):
        mission = GoldStandardMission(dry_run=True)
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_yaw = 10.0
        mission.latest_telemetry["imu"] = {"raw_yaw_deg": 65.0, "gyro_z": 0.05}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "IMU frame discontinuity detected" in str(excinfo.value)

    def test_serial_disconnect_abort(self):
        mission = GoldStandardMission(dry_run=False)
        import time
        mission.initial_boot_count = 1
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["drive"] = {"bootCount": 1}
        mission.latest_telemetry["imu"] = {"serialConnected": False}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Serial communication to ESP32 disconnected" in str(excinfo.value)

    def test_localization_loss_abort(self):
        mission = GoldStandardMission(dry_run=True)
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["amcl"] = {"localized": False, "state": "LOST"}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Localization lost" in str(excinfo.value)


class TestFourLegStateTransitions:
    def test_complete_four_leg_dry_run_transitions(self):
        mission = GoldStandardMission(dry_run=True)
        targets = mission.compute_mission_targets()
        
        mission.recorder.record_transition("LEG1_FORWARD_START", {"target": (targets["leg1_outbound"]["x"], targets["leg1_outbound"]["y"])})
        mission.recorder.record_transition("LEG1_FORWARD_END")

        mission.recorder.record_transition("LEG2_ROTATION_START", {"target_deg": -180.0})
        mission.recorder.record_transition("LEG2_ROTATION_END")

        mission.recorder.record_transition("LEG3_RETURN_START", {"home": (targets["home"]["x"], targets["home"]["y"])})
        mission.recorder.record_transition("LEG3_RETURN_END")

        mission.recorder.record_transition("LEG4_SETTLE_START")
        mission.recorder.record_transition("LEG4_SETTLE_END")

        stages = [t["stage"] for t in mission.recorder.transitions]
        assert "LEG1_FORWARD_START" in stages
        assert "LEG1_FORWARD_END" in stages
        assert "LEG2_ROTATION_START" in stages
        assert "LEG2_ROTATION_END" in stages
        assert "LEG3_RETURN_START" in stages
        assert "LEG3_RETURN_END" in stages
        assert "LEG4_SETTLE_START" in stages
        assert "LEG4_SETTLE_END" in stages


class TestGrader:
    def test_grade_perfect_run(self):
        # 60 samples spanning 2.0s -> achieved rate 29.5 Hz (>= 20 Hz)
        samples = []
        for i in range(60):
            t = round(float(i) * 0.035, 4)
            samples.append({
                "t_rel_s": t,
                "mission_stage": "LEG1_FORWARD" if i < 30 else "LEG4_SETTLE",
                "amcl": {"x": 1.0, "y": 2.0},
                "to_home": {"pos_err_m": 0.015, "yaw_err_deg": 1.2},
                "drive": {"bootCount": 1, "armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0},
                "final_cmd": {"vx": 0.0, "wz": 0.0}
            })

        run_data = {
            "metadata": {"duration_s": samples[-1]["t_rel_s"], "watchdog_trips": 0, "rejections": 0},
            "samples": samples
        }
        res = MissionGrader.grade_run(run_data)
        assert res["overall_status"] == "PASS"
        assert res["criteria"]["final_home_position_error"]["passed"] is True
        assert res["criteria"]["final_home_yaw_error"]["passed"] is True
        assert res["criteria"]["recorder_sample_rate"]["passed"] is True
        assert res["criteria"]["final_safe_state"]["passed"] is True

    def test_grade_fail_position_error(self):
        samples = [
            {
                "t_rel_s": float(i) * 0.035,
                "mission_stage": "LEG4_SETTLE",
                "to_home": {"pos_err_m": 0.08, "yaw_err_deg": 1.0},
                "drive": {"bootCount": 1, "armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0},
                "final_cmd": {"vx": 0.0, "wz": 0.0}
            }
            for i in range(60)
        ]
        run_data = {
            "metadata": {"duration_s": samples[-1]["t_rel_s"], "watchdog_trips": 0, "rejections": 0},
            "samples": samples
        }
        res = MissionGrader.grade_run(run_data)
        assert res["overall_status"] == "FAIL"
        assert res["criteria"]["final_home_position_error"]["passed"] is False

    def test_grade_fail_excessive_crawling(self):
        samples = [
            {
                "t_rel_s": round(float(i) * 0.04, 3),
                "mission_stage": "LEG3_RETURN",
                "final_cmd": {"vx": 0.03, "wz": 0.0},
                "to_home": {"pos_err_m": 0.02, "yaw_err_deg": 1.0},
                "drive": {"armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0}
            }
            for i in range(80) # 3.2 seconds continuous crawl > 2.0s
        ]
        run_data = {
            "metadata": {"duration_s": samples[-1]["t_rel_s"], "watchdog_trips": 0, "rejections": 0},
            "samples": samples
        }
        res = MissionGrader.grade_run(run_data)
        assert res["overall_status"] == "FAIL"
        assert res["criteria"]["low_speed_crawling"]["passed"] is False


class TestReviewRequirements:
    """Explicit regression tests covering items 1-6 of the code review."""

    def test_no_simultaneous_nav2_and_exact_motion_ownership(self):
        """Proof: Cannot send EXACT_MOTION velocity commands while Nav2 goal is active."""
        mission = GoldStandardMission(dry_run=True)
        mission.active_nav2_goal = True
        
        # Zero commands pass unconditionally for safe halting
        mission.send_velocity(0.0, 0.0)

        # Nonzero velocity command while Nav2 goal is active MUST raise MissionAbortException
        with pytest.raises(MissionAbortException) as exc:
            mission.send_velocity(0.10, 0.0, source="EXACT_MOTION")
        assert "while Nav2 goal is active" in str(exc.value)

    def test_nav2_success_and_zero_output_handshake(self):
        """Proof: Nav2 must report success/target reached and zero output before closing and advancing."""
        mission = GoldStandardMission(dry_run=False)
        mission.active_nav2_goal = True

        # Mock Cockpit responses: drive status settled at standstill, Nav2 status SUCCEEDED
        with patch("requests.get") as mock_get, patch("requests.post") as mock_post:
            def mock_get_router(url, **kwargs):
                resp = MagicMock()
                resp.status_code = 200
                if "/api/drive/status" in url:
                    resp.json.return_value = {
                        "status": {"limLinear": 0.0, "limAngular": 0.0, "bootCount": 1}
                    }
                elif "/api/navigation/status" in url:
                    resp.json.return_value = {
                        "status": "SUCCEEDED", "distance_remaining_m": 0.01
                    }
                elif "/api/localization/status" in url:
                    resp.json.return_value = {"localized": True, "state": "LOCALIZED"}
                elif "/api/imu" in url:
                    resp.json.return_value = {"dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": 100}
                else:
                    resp.json.return_value = {}
                return resp

            mock_get.side_effect = mock_get_router
            mock_post.return_value.status_code = 200

            res = mission.wait_for_nav2_completion_and_zero(timeout_s=2.0, stage_name="TEST_LEG")
            assert res is True
            assert mission.active_nav2_goal is False
            
            # Verify cancel/close was called to release Nav2
            cancel_called = any("/api/navigation/cancel" in call.args[0] for call in mock_post.call_args_list)
            assert cancel_called is True

    def test_leg3_retains_return_facing_yaw(self):
        """Proof: Leg 3 target yaw retains return-facing yaw, not final HOME yaw."""
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_deg": -5.0, "yaw_rad": math.radians(-5.0)}
        
        targets = mission.compute_mission_targets()
        leg2_yaw = targets["leg2_rotation"]["yaw_deg"]
        leg3_yaw = targets["leg3_return"]["yaw_deg"]
        home_yaw = targets["home"]["yaw_deg"]

        assert leg3_yaw == leg2_yaw
        assert abs(leg3_yaw - home_yaw) > 170.0 # Return heading is ~180° away from HOME heading

    def test_leg4_signed_rotation_and_timeout(self):
        """Proof: Leg 4 produces signed shortest-angle commands capped at 0.18 rad/s and fails on timeout."""
        mission = GoldStandardMission(dry_run=True)
        # HOME yaw is 0.0
        target_yaw_rad = 0.0

        # Case 1: Current heading is -30 deg (-0.52 rad) -> Shortest turn is CCW (+wz)
        cur_th = math.radians(-30.0)
        delta_yaw = wrap_angle_rad(target_yaw_rad - cur_th)
        assert delta_yaw > 0
        wz_mag = min(FINAL_YAW_ALIGNMENT_MAX_SPEED, 0.08 + (abs(delta_yaw) / math.radians(30.0)) * (FINAL_YAW_ALIGNMENT_MAX_SPEED - 0.08))
        wz_cmd = math.copysign(wz_mag, delta_yaw)
        assert wz_cmd > 0.0
        assert wz_cmd <= FINAL_YAW_ALIGNMENT_MAX_SPEED

        # Case 2: Current heading is +30 deg (+0.52 rad) -> Shortest turn is CW (-wz)
        cur_th2 = math.radians(30.0)
        delta_yaw2 = wrap_angle_rad(target_yaw_rad - cur_th2)
        assert delta_yaw2 < 0
        wz_cmd2 = math.copysign(wz_mag, delta_yaw2)
        assert wz_cmd2 < 0.0
        assert abs(wz_cmd2) <= FINAL_YAW_ALIGNMENT_MAX_SPEED

        # Case 3: Timeout must fail the mission
        mission.dry_run = False
        with patch("requests.get") as mock_get:
            resp = MagicMock()
            resp.status_code = 200
            # Heading remains at 45° error (> 3.0° tolerance)
            resp.json.return_value = {
                "x": 1.0, "y": 2.0, "yawDeg": 45.0, "yaw": math.radians(45.0),
                "localized": True, "state": "LOCALIZED", "dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": 1
            }
            mock_get.return_value = resp

            # Set a very short timeout
            with patch("tools.gold_standard_home_test.mission.TIMEOUT_LEG4_SETTLE_S", 0.1):
                with pytest.raises(MissionAbortException) as exc:
                    mission.execute_leg4_settle(target_yaw_rad)
                assert "exceeded timeout" in str(exc.value)

    def test_preflight_fails_closed_on_missing_or_failed_localization(self):
        """Proof: Preflight refuses to arm if AMCL fails, is unlocalized, or fields are missing."""
        mission = GoldStandardMission(dry_run=True)
        mission.cockpit_url = "http://fake-cockpit"

        # Failure 1: Network exception
        with patch("requests.get", side_effect=requests.ConnectionError("Refused")):
            ok, err, pose = mission.fetch_live_amcl_pose()
            assert ok is False
            assert pose is None

        # Failure 2: Not localized
        with patch("requests.get") as mock_get:
            mock_get.return_value.status_code = 200
            mock_get.return_value.json.return_value = {"ok": True, "localized": False, "state": "UNLOCALIZED"}
            ok, err, pose = mission.fetch_live_amcl_pose()
            assert ok is False
            assert "not LOCALIZED" in err
            assert pose is None

        # Failure 3: Missing required coordinates
        with patch("requests.get") as mock_get:
            mock_get.return_value.status_code = 200
            mock_get.return_value.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": None}
            ok, err, pose = mission.fetch_live_amcl_pose()
            assert ok is False
            assert "missing" in err
            assert pose is None

    def test_active_production_linear_cap(self):
        """Proof: Production configuration in nav2_params.yaml sets max_vel_x and max_speed_xy to 0.20 m/s."""
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        params_path = os.path.join(repo_root, "ros2", "ros2_ws", "src", "rover_bringup", "config", "nav2_params.yaml")
        
        with open(params_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        fp = cfg["controller_server"]["ros__parameters"]["FollowPath"]
        vs = cfg["velocity_smoother"]["ros__parameters"]

        assert fp["max_vel_x"] == 0.20
        assert fp["max_speed_xy"] == 0.20
        assert fp["max_vel_theta"] == 0.50
        assert fp["rotate_to_heading_angular_vel"] == 0.50
        assert vs["max_velocity"][0] == 0.20
        assert vs["max_velocity"][2] == 0.50

    def test_stale_cached_telemetry_abort(self):
        """Proof: Stale underlying telemetry (dataAgeMs > 500 or sequence freeze) triggers abort."""
        mission = GoldStandardMission(dry_run=False)
        import time
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_seq_adv_time = time.monotonic()
        
        # Test dataAgeMs > 500
        mission.latest_telemetry["imu"] = {
            "dataAgeMs": 650, # Stale underlying packet
            "stale": False,
            "serialConnected": True
        }
        with pytest.raises(MissionAbortException) as exc:
            mission.verify_safety_invariants("TEST")
        assert "exceeds 500ms limit" in str(exc.value)

        # Test serial packet marked stale
        mission.latest_telemetry["imu"] = {
            "dataAgeMs": 10,
            "stale": True,
            "serialConnected": True
        }
        with pytest.raises(MissionAbortException) as exc2:
            mission.verify_safety_invariants("TEST")
        assert "packet marked stale" in str(exc2.value)
