"""
test.test_gold_standard_home_test - Automated Unit & Safety Abort Tests
Tests HOME loading, pre-arm gates, target geometry, grading criteria, all fail-safe abort conditions,
clean command ownership handshakes, signed Leg 4 rotation, active production velocity configs,
and comprehensive mocked integration tests for defect corrections.
"""

import os
import math
import time
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


@pytest.fixture(autouse=True)
def prevent_unmocked_network(monkeypatch):
    """Hermetic fixture ensuring unit tests never make unmocked HTTP calls to external hardware."""
    def dummy_post(url, *args, **kwargs):
        resp = MagicMock()
        resp.status_code = 200
        source = kwargs.get("json", {}).get("source", "NONE") if isinstance(kwargs.get("json"), dict) else "NONE"
        resp.json.return_value = {"ok": True, "status": "ok", "cmdSource": source}
        return resp

    def dummy_get(url, *args, **kwargs):
        resp = MagicMock()
        resp.status_code = 200
        if "/api/drive/status" in url:
            resp.json.return_value = {
                "ok": True,
                "status": {
                    "armed": True, "mode": 3, "bootCount": 1,
                    "reqLinear": 0.0, "reqAngular": 0.0,
                    "limLinear": 0.0, "limAngular": 0.0,
                    "cmdSource": "CALIBRATION_TEST", "seq": 1
                }
            }
        elif "/api/navigation/dispatch" in url:
            resp.json.return_value = {
                "ok": True,
                "goal_id": "mock_goal_001",
                "dispatch_meta": {"goal_id": "mock_goal_001", "dispatched_at": 1000.0}
            }
        elif "/api/navigation/status" in url:
            resp.json.return_value = {
                "ok": True, "status": "SUCCEEDED", "goal_id": "mock_goal_001", "distance_remaining_m": 0.0
            }
        elif "/api/localization/status" in url:
            resp.json.return_value = {
                "ok": True, "localized": True, "state": "LOCALIZED",
                "x": 1.0, "y": 2.0, "yawDeg": 0.0, "yaw": 0.0, "ageMs": 50
            }
        elif "/api/imu" in url:
            resp.json.return_value = {
                "ok": True, "serialConnected": True, "dataAgeMs": 20,
                "stale": False, "sequence": 1, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}
            }
        elif "/api/encoders" in url:
            resp.json.return_value = {
                "ok": True,
                "schema_version": "1.0",
                "serialConnected": True,
                "timestamp": 12345678,
                "lastPacketAgeMs": 15,
                "sequence": 100,
                "encoders": {"m1": 1000, "m2": 1000, "m3": 1000, "m4": 1000}
            }
        elif "/api/odom" in url:
            resp.json.return_value = {
                "ok": True,
                "timestamp": 12345678,
                "x": 0.0, "y": 0.0, "yaw": 0.0, "yaw_deg": 0.0,
                "v_x": 0.0, "w_z": 0.0, "odometry_age_ms": 25,
                "raw_d_left_m": 0.0, "raw_d_right_m": 0.0,
                "node_health": "ok"
            }
        elif "/api/clearance" in url:
            resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
        else:
            resp.json.return_value = {"ok": True}
        return resp

    monkeypatch.setattr(requests, "post", dummy_post)
    monkeypatch.setattr(requests, "get", dummy_get)


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
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_seq_adv_time = time.monotonic() - 10.0
        mission.dry_run = False
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Stale telemetry" in str(excinfo.value)

    def test_boot_count_change_abort(self):
        mission = GoldStandardMission(dry_run=False)
        mission.initial_boot_count = 1
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["drive"] = {"bootCount": 2}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "ESP32 hardware reboot detected" in str(excinfo.value)

    def test_imu_discontinuity_abort(self):
        mission = GoldStandardMission(dry_run=True)
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_yaw = 10.0
        mission.latest_telemetry["imu"] = {"raw_yaw_deg": 65.0, "gyro_z": 0.05}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "IMU frame discontinuity detected" in str(excinfo.value)

    def test_serial_disconnect_abort(self):
        mission = GoldStandardMission(dry_run=False)
        mission.initial_boot_count = 1
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.latest_telemetry["drive"] = {"bootCount": 1}
        mission.latest_telemetry["imu"] = {"serialConnected": False}
        with pytest.raises(MissionAbortException) as excinfo:
            mission.verify_safety_invariants("TEST_STAGE")
        assert "Serial communication to ESP32 disconnected" in str(excinfo.value)

    def test_localization_loss_abort(self):
        mission = GoldStandardMission(dry_run=True)
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
            for i in range(80)
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
        """Proof: Cannot send direct velocity commands while Nav2 goal is active."""
        mission = GoldStandardMission(dry_run=True)
        mission.active_nav2_goal = True
        
        # Zero commands pass unconditionally for safe halting
        mission.send_velocity(0.0, 0.0)

        # Nonzero velocity command while Nav2 goal is active MUST raise MissionAbortException
        with pytest.raises(MissionAbortException) as exc:
            mission.send_velocity(0.10, 0.0, source="CALIBRATION_TEST")
        assert "while Nav2 goal is active" in str(exc.value)

    def test_nav2_success_and_zero_output_handshake(self):
        """Proof: Nav2 must report success/target reached and zero output before advancing."""
        mission = GoldStandardMission(dry_run=False)
        mission.active_nav2_goal = True

        status_calls = 0
        nav_calls = 0
        with patch("requests.get") as mock_get, patch("requests.post") as mock_post:
            def mock_get_router(url, **kwargs):
                nonlocal status_calls, nav_calls
                status_calls += 1
                resp = MagicMock()
                resp.status_code = 200
                if "/api/drive/status" in url:
                    resp.json.return_value = {
                        "ok": True,
                        "status": {
                            "armed": False, "mode": 0, "cmdSource": "NONE",
                            "reqLinear": 0.0, "reqAngular": 0.0,
                            "limLinear": 0.0, "limAngular": 0.0,
                            "bootCount": 1, "seq": status_calls
                        }
                    }
                elif "/api/navigation/status" in url:
                    nav_calls += 1
                    # Transition: EXECUTING -> SUCCEEDED
                    st = "EXECUTING" if nav_calls < 3 else "SUCCEEDED"
                    resp.json.return_value = {
                        "status": st, "goal_id": "test_goal", "distance_remaining_m": 0.01
                    }
                elif "/api/localization/status" in url:
                    resp.json.return_value = {"localized": True, "state": "LOCALIZED"}
                elif "/api/imu" in url:
                    resp.json.return_value = {"dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": status_calls}
                else:
                    resp.json.return_value = {}
                return resp

            mock_get.side_effect = mock_get_router
            mock_post.return_value.status_code = 200

            res = mission.wait_for_nav2_completion_and_zero(timeout_s=2.0, stage_name="TEST_LEG", goal_id="test_goal")
            assert res is True
            assert mission.active_nav2_goal is False

    def test_leg3_retains_return_facing_yaw(self):
        """Proof: Leg 3 target yaw retains return-facing yaw, not final HOME yaw."""
        mission = GoldStandardMission(dry_run=True)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_deg": -5.0, "yaw_rad": math.radians(-5.0)}
        
        targets = mission.compute_mission_targets()
        leg2_yaw = targets["leg2_rotation"]["yaw_deg"]
        leg3_yaw = targets["leg3_return"]["yaw_deg"]
        home_yaw = targets["home"]["yaw_deg"]

        assert leg3_yaw == leg2_yaw
        assert abs(leg3_yaw - home_yaw) > 170.0

    def test_leg4_signed_rotation_and_timeout(self):
        """Proof: Leg 4 produces signed shortest-angle commands capped at 0.18 rad/s and fails on timeout."""
        mission = GoldStandardMission(dry_run=True)
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
        seq_c = 0
        with patch("requests.get") as mock_get, patch("requests.post") as mock_post:
            def mock_get_router(url, **kwargs):
                nonlocal seq_c
                seq_c += 1
                resp = MagicMock()
                resp.status_code = 200
                if "/api/drive/status" in url:
                    resp.json.return_value = {
                        "ok": True,
                        "status": {"armed": True, "mode": 3, "bootCount": 1, "reqLinear": 0.0, "reqAngular": 0.0, "limLinear": 0.0, "limAngular": 0.0, "cmdSource": "CALIBRATION_TEST", "seq": seq_c}
                    }
                elif "/api/localization/status" in url:
                    resp.json.return_value = {
                        "ok": True, "x": 1.0, "y": 2.0, "yawDeg": 45.0, "yaw": math.radians(45.0),
                        "localized": True, "state": "LOCALIZED", "ageMs": 10
                    }
                elif "/api/imu" in url:
                    resp.json.return_value = {
                        "ok": True, "dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": seq_c, "raw_yaw_deg": 45.0, "gyro": {"z": 0.0}
                    }
                else:
                    resp.json.return_value = {"ok": True}
                return resp

            def mock_post_router(url, **kwargs):
                resp = MagicMock()
                resp.status_code = 200
                src = kwargs.get("json", {}).get("source", "CALIBRATION_TEST") if isinstance(kwargs.get("json"), dict) else "CALIBRATION_TEST"
                resp.json.return_value = {"ok": True, "cmdSource": src}
                return resp

            mock_get.side_effect = mock_get_router
            mock_post.side_effect = mock_post_router

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
        mission.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        mission.last_imu_seq_adv_time = time.monotonic()
        
        # Test dataAgeMs > 500
        mission.latest_telemetry["imu"] = {
            "dataAgeMs": 650,
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


class TestDefectCorrectionsAndIntegration:
    """Mocked integration tests reproducing defect corrections and verified transitions."""

    def test_nav_cancel_causes_disarm(self):
        """1. /api/navigation/cancel disarms the rover. Runner handles disarm properly."""
        mission = GoldStandardMission(dry_run=False)
        posted_urls = []

        def tracking_post(url, *args, **kwargs):
            posted_urls.append(url)
            resp = MagicMock()
            resp.status_code = 200
            resp.json.return_value = {"ok": True, "status": "ok"}
            return resp

        with patch("requests.post", side_effect=tracking_post):
            mission.disarm_and_stop()

        # Confirm cancel and disarm endpoints were called
        assert any("/api/navigation/cancel" in u for u in posted_urls)
        assert any("/api/drive/disarm" in u for u in posted_urls)
        assert mission.active_nav2_goal is False

    def test_initial_idle_status_rejected_before_goal_starts(self):
        """2. Initial IDLE status is rejected before goal becomes ACTIVE; only then can SUCCEEDED complete."""
        mission = GoldStandardMission(dry_run=False)
        mission.active_nav2_goal = True

        status_sequence = ["IDLE", "IDLE", "EXECUTING", "SUCCEEDED"]
        call_idx = 0

        def mock_get(url, **kwargs):
            nonlocal call_idx
            resp = MagicMock()
            resp.status_code = 200
            if "/api/navigation/status" in url:
                st = status_sequence[min(call_idx, len(status_sequence) - 1)]
                call_idx += 1
                resp.json.return_value = {"status": st, "goal_id": "goal_123", "distance_remaining_m": 0.01}
            elif "/api/drive/status" in url:
                resp.json.return_value = {
                    "status": {
                        "armed": False, "mode": 0, "cmdSource": "NONE",
                        "reqLinear": 0.0, "reqAngular": 0.0,
                        "limLinear": 0.0, "limAngular": 0.0,
                        "bootCount": 1, "seq": call_idx
                    }
                }
            elif "/api/localization/status" in url:
                resp.json.return_value = {"localized": True, "state": "LOCALIZED"}
            elif "/api/imu" in url:
                resp.json.return_value = {"dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": call_idx}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_get):
            res = mission.wait_for_nav2_completion_and_zero(timeout_s=2.0, stage_name="TEST_LEG", goal_id="goal_123")
            assert res is True
            # Verified we polled past IDLE into EXECUTING before SUCCEEDED was accepted
            assert call_idx >= 4

    def test_nav2_stays_idle_times_out(self):
        """Proof: If Nav2 stays IDLE and never becomes active, it must not complete and must abort on timeout."""
        mission = GoldStandardMission(dry_run=False)
        mission.active_nav2_goal = True

        def mock_get(url, **kwargs):
            resp = MagicMock()
            resp.status_code = 200
            if "/api/navigation/status" in url:
                resp.json.return_value = {"status": "IDLE", "distance_remaining_m": 0.0}
            elif "/api/drive/status" in url:
                resp.json.return_value = {"status": {"limLinear": 0.0, "limAngular": 0.0, "bootCount": 1, "seq": 1}}
            elif "/api/localization/status" in url:
                resp.json.return_value = {"localized": True, "state": "LOCALIZED"}
            elif "/api/imu" in url:
                resp.json.return_value = {"dataAgeMs": 10, "stale": False, "serialConnected": True, "sequence": 1}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_get):
            with pytest.raises(MissionAbortException) as exc:
                mission.wait_for_nav2_completion_and_zero(timeout_s=0.2, stage_name="TEST_LEG", goal_id="goal_123")
            assert "exceeded timeout" in str(exc.value)

    def test_rejected_direct_velocity_command(self):
        """3. A rejected direct velocity command immediately commands zero, disarms, and raises MissionAbortException."""
        mission = GoldStandardMission(dry_run=False)
        disarm_called = False

        def mock_post(url, *args, **kwargs):
            nonlocal disarm_called
            resp = MagicMock()
            if "/api/cmd_vel" in url:
                resp.status_code = 500
                resp.text = "Internal Bridge Error"
                resp.json.return_value = {"ok": False, "error": "Hardware fault"}
            elif "/api/drive/disarm" in url:
                disarm_called = True
                resp.status_code = 200
                resp.json.return_value = {"ok": True}
            else:
                resp.status_code = 200
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.post", side_effect=mock_post):
            with pytest.raises(MissionAbortException) as exc:
                mission.send_velocity(0.10, 0.0, source="CALIBRATION_TEST")
            assert "Velocity command rejected" in str(exc.value)
            assert disarm_called is True

    def test_failed_command_source_transition(self):
        """4. A failed command source transition immediately commands zero, disarms, and raises MissionAbortException."""
        mission = GoldStandardMission(dry_run=False)
        disarm_called = False

        def mock_post(url, *args, **kwargs):
            nonlocal disarm_called
            resp = MagicMock()
            if "/api/command-source" in url:
                resp.status_code = 403
                resp.text = "Forbidden"
                resp.json.return_value = {"ok": False, "error": "Source locked"}
            elif "/api/drive/disarm" in url:
                disarm_called = True
                resp.status_code = 200
                resp.json.return_value = {"ok": True}
            else:
                resp.status_code = 200
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.post", side_effect=mock_post):
            with pytest.raises(MissionAbortException) as exc:
                mission.set_command_source("CALIBRATION_TEST")
            assert "Command source transition to CALIBRATION_TEST rejected" in str(exc.value)
            assert disarm_called is True

    def test_zero_output_handshake_verification(self):
        """7. Verify real zero-output handshake: advancing non-null sequence, >= 50ms apart, 3 consecutive samples."""
        mission = GoldStandardMission(dry_run=False)
        seq_counter = 10
        mono_time = 100.0

        def mock_get(url, **kwargs):
            nonlocal seq_counter
            resp = MagicMock()
            resp.status_code = 200
            if "/api/drive/status" in url:
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": False, "mode": 0, "bootCount": 1,
                        "reqLinear": 0.0, "reqAngular": 0.0,
                        "limLinear": 0.0, "limAngular": 0.0,
                        "cmdSource": "NONE", "seq": seq_counter
                    }
                }
            elif "/api/imu" in url:
                resp.json.return_value = {
                    "ok": True, "serialConnected": True, "dataAgeMs": 10,
                    "stale": False, "sequence": seq_counter, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}
                }
            elif "/api/localization/status" in url:
                resp.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": 1.0, "y": 2.0, "yawDeg": 0.0, "yaw": 0.0, "ageMs": 10}
            elif "/api/encoders" in url:
                resp.json.return_value = {
                    "ok": True, "schema_version": "1.0", "serialConnected": True,
                    "sequence": seq_counter, "encoders": {"m1": 10, "m2": 10, "m3": 10, "m4": 10}
                }
            elif "/api/odom" in url:
                resp.json.return_value = {"ok": True, "x": 0.0, "y": 0.0, "v_x": 0.0, "w_z": 0.0}
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            return resp

        def advancing_mono():
            nonlocal mono_time, seq_counter
            # Advance monotonic time by 60ms and seq by 1 per poll cycle
            mono_time += 0.060
            seq_counter += 1
            return mono_time

        with patch("requests.get", side_effect=mock_get),              patch("time.monotonic", side_effect=advancing_mono),              patch("time.sleep", return_value=None):
            res = mission.verify_zero_handshake(expected_cmd_source="NONE", timeout_s=1.0)
            assert res is True

    def test_successful_rearming_and_ownership_transfer_all_four_legs(self):
        """5. Successful rearming and ownership transfer between all four legs."""
        mission = GoldStandardMission(dry_run=False)
        mission.home_pose = {"x": 1.0, "y": 2.0, "yaw_deg": 0.0, "yaw_rad": 0.0}

        actions_log = []
        seq_num = 100
        leg2_yaw = 0.0

        def mock_post(url, *args, **kwargs):
            nonlocal seq_num
            seq_num += 1
            resp = MagicMock()
            resp.status_code = 200
            
            if "/api/drive/arm" in url:
                actions_log.append("ARM")
                resp.json.return_value = {"ok": True, "armed": True, "mode": 3}
            elif "/api/drive/disarm" in url:
                actions_log.append("DISARM")
                resp.json.return_value = {"ok": True, "armed": False, "mode": 0}
            elif "/api/command-source" in url:
                src = kwargs.get("json", {}).get("source", "UNKNOWN")
                actions_log.append(f"SOURCE:{src}")
                resp.json.return_value = {"ok": True, "cmdSource": src}
            elif "/api/navigation/dispatch" in url:
                actions_log.append("DISPATCH_NAV2")
                gid = f"leg_goal_{len(actions_log)}"
                resp.json.return_value = {"ok": True, "goal_id": gid, "dispatch_meta": {"goal_id": gid, "dispatched_at": time.time()}}
            elif "/api/cmd_vel" in url:
                src = kwargs.get("json", {}).get("source", "UNKNOWN")
                actions_log.append(f"CMD_VEL:{src}")
                resp.json.return_value = {"ok": True}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        nav_call_count = 0

        def mock_get(url, **kwargs):
            nonlocal seq_num, nav_call_count, leg2_yaw
            seq_num += 1
            resp = MagicMock()
            resp.status_code = 200
            
            current_armed = ("ARM" in actions_log and (actions_log[-1] == "ARM" or "DISARM" not in actions_log[actions_log.index("ARM"):]))
            mode = 3 if current_armed else 0

            sources = [a.split(":")[1] for a in actions_log if a.startswith("SOURCE:")]
            curr_src = sources[-1] if sources else "NONE"

            if "/api/drive/status" in url:
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": current_armed, "mode": mode, "bootCount": 1,
                        "reqLinear": 0.0, "reqAngular": 0.0,
                        "limLinear": 0.0, "limAngular": 0.0,
                        "cmdSource": curr_src, "seq": seq_num
                    }
                }
            elif "/api/navigation/status" in url:
                nav_call_count += 1
                st = "EXECUTING" if nav_call_count % 3 != 0 else "SUCCEEDED"
                # Echo active goal_id matching the latest dispatch
                last_gids = [f"leg_goal_{i+1}" for i, a in enumerate(actions_log) if a == "DISPATCH_NAV2"]
                active_gid = last_gids[-1] if last_gids else "leg_goal_1"
                resp.json.return_value = {"ok": True, "status": st, "goal_id": active_gid, "distance_remaining_m": 0.01}
            elif "/api/localization/status" in url:
                resp.json.return_value = {
                    "ok": True, "localized": True, "state": "LOCALIZED",
                    "x": 1.0, "y": 2.0, "yawDeg": leg2_yaw, "yaw": math.radians(leg2_yaw), "ageMs": 10
                }
            elif "/api/imu" in url:
                # During Leg 2, reach 180° in smooth 15° increments once turning starts
                if "LEG2" in actions_log and mission.latest_exact_cmd.get("wz", 0.0) < -0.1:
                    leg2_yaw = max(-180.0, leg2_yaw - 15.0)
                elif "LEG4" in actions_log and abs(mission.latest_exact_cmd.get("wz", 0.0)) > 0.05:
                    leg2_yaw = min(0.0, leg2_yaw + 15.0)
                gyro_val = 0.005 if (leg2_yaw >= -1.0 and "LEG4" in actions_log) else -0.4
                resp.json.return_value = {
                    "ok": True, "serialConnected": True, "dataAgeMs": 10,
                    "stale": False, "sequence": seq_num, "raw_yaw_deg": leg2_yaw, "gyro": {"z": gyro_val}
                }
            elif "/api/encoders" in url:
                resp.json.return_value = {
                    "ok": True, "schema_version": "1.0", "serialConnected": True,
                    "sequence": seq_num, "encoders": {"m1": 100, "m2": 100, "m3": 100, "m4": 100}
                }
            elif "/api/odom" in url:
                resp.json.return_value = {
                    "ok": True, "x": 0.0, "y": 0.0, "yaw_deg": leg2_yaw, "v_x": 0.0, "w_z": 0.0
                }
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        # Smooth advancing monotonic clock
        curr_m_time = 100.0
        def advancing_monotonic():
            nonlocal curr_m_time
            curr_m_time += 0.060  # >= 50ms per poll
            return curr_m_time

        with patch("requests.post", side_effect=mock_post), \
             patch("requests.get", side_effect=mock_get), \
             patch("time.monotonic", side_effect=advancing_monotonic), \
             patch("time.sleep", return_value=None):
            
            # Leg 1: Starts disarmed, Nav2 dispatch -> SUCCEEDED (disarms in Cockpit)
            mission.execute_leg1_forward(1.6096, 2.0, 0.0)
            assert "DISPATCH_NAV2" in actions_log

            # Leg 2: Zero handshake, Arm Mode 3, CALIBRATION_TEST, execute rotation, Disarm Mode 0, Source NONE
            actions_log.append("LEG2")
            mission.execute_leg2_rotation(180.0)
            assert "ARM" in actions_log
            assert "SOURCE:CALIBRATION_TEST" in actions_log
            assert any(a == "CMD_VEL:CALIBRATION_TEST" for a in actions_log)
            assert "DISARM" in actions_log
            assert actions_log[-1] == "SOURCE:NONE"

            # Leg 3: Zero handshake, Nav2 dispatch return -> SUCCEEDED (disarms in Cockpit)
            mission.execute_leg3_return(1.0, 2.0, math.radians(-180.0))
            assert actions_log.count("DISPATCH_NAV2") == 2

            # Leg 4: Zero handshake, Arm Mode 3, CALIBRATION_TEST, align, settle, Disarm Mode 0, Source NONE
            actions_log.append("LEG4")
            # Advance monotonic time smoothly while settling
            settle_samples = 0
            def settle_advancing_monotonic():
                nonlocal curr_m_time, settle_samples
                if leg2_yaw >= -1.0: # In tolerance
                    settle_samples += 1
                    # After 5 ticks, advance monotonic by 0.2s each tick
                    curr_m_time += 0.2
                else:
                    curr_m_time += 0.02
                return curr_m_time

            with patch("time.monotonic", side_effect=settle_advancing_monotonic):
                mission.execute_leg4_settle(0.0)

            assert actions_log.count("ARM") == 2
            assert actions_log.count("DISARM") == 2
            assert actions_log[-1] == "SOURCE:NONE"


class TestContractSchemasAndEdgeCases:
    def test_real_encoders_and_odom_schemas(self):
        """Contract: /api/encoders returns raw ticks; /api/odom returns actual kinematics."""
        mission = GoldStandardMission(dry_run=False)

        real_encoders_payload = {
            "ok": True,
            "schema_version": "1.0",
            "serialConnected": True,
            "timestamp": 1790450000000,
            "lastPacketAgeMs": 12,
            "sequence": 5432,
            "parserStats": {"errors": 0},
            "encoders": {
                "m1": 12345,
                "m2": -12340,
                "m3": 12342,
                "m4": -12338
            }
        }

        real_odom_payload = {
            "ok": True,
            "timestamp": 1790450000.123,
            "x": 0.1234,
            "y": -0.0567,
            "yaw": -0.088,
            "yaw_deg": -5.042,
            "v_x": 0.15,
            "w_z": -0.02,
            "odometry_age_ms": 42,
            "node_health": "ok",
            "raw_d_left_m": 0.12,
            "raw_d_right_m": 0.12
        }

        def mock_get(url, **kwargs):
            resp = MagicMock()
            resp.status_code = 200
            if "/api/encoders" in url:
                resp.json.return_value = real_encoders_payload
            elif "/api/odom" in url:
                resp.json.return_value = real_odom_payload
            else:
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_get):
            mission.poll_all_telemetry()
            
            # Encoders must reflect raw ticks, not false kinematics
            enc = mission.latest_telemetry.get("encoders")
            assert enc is not None
            assert enc["m1"] == 12345
            assert enc["sequence"] == 5432

            # Odom must reflect kinematics from /api/odom
            od = mission.latest_telemetry.get("odom")
            assert od is not None
            assert abs(od["x"] - 0.1234) < 1e-4
            assert abs(od["vx"] - 0.15) < 1e-4
            assert abs(od["wz"] - (-0.02)) < 1e-4

    def test_prevent_recursive_abort_when_cmd_vel_fails(self):
        """Contract: When /api/cmd_vel is unavailable, disarm_and_stop does not recurse."""
        mission = GoldStandardMission(dry_run=False)
        cmd_vel_calls = 0

        def failing_post(url, *args, **kwargs):
            nonlocal cmd_vel_calls
            if "/api/cmd_vel" in url:
                cmd_vel_calls += 1
                raise requests.exceptions.ConnectionError("Connection refused by bridge")
            resp = MagicMock()
            resp.status_code = 200
            resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.post", side_effect=failing_post):
            # Calling disarm_and_stop must not raise recursion or uncaught error
            mission.disarm_and_stop()
            # cmd_vel was attempted with force_zero, failed safely, no recursion
            assert cmd_vel_calls == 1
            assert mission._in_disarm_and_stop is False

    def test_stage_attribution_does_not_duplicate_fake_stages(self):
        """Contract: smoothed_cmd is None unless a distinct smoother stage is polled."""
        mission = GoldStandardMission(dry_run=True)
        mission.latest_exact_cmd = {"vx": 0.10, "wz": 0.20}
        mission.latest_telemetry["drive"] = {
            "reqLinear": 0.10, "reqAngular": 0.20,
            "limLinear": 0.08, "limAngular": 0.15,
            "armed": True, "mode": 3
        }

        # Calibration test tick
        mission.active_nav2_goal = False
        mission.record_tick("LEG2_ROTATION")
        last_frame = mission.recorder.frames[-1]
        assert last_frame["cmd_source"] == "CALIBRATION_TEST"
        assert last_frame["raw_cmd"]["vx"] == 0.10
        assert last_frame["smoothed_cmd"] is None  # Must NOT be duplicated
        assert last_frame["final_cmd"]["vx"] == 0.08

        # Nav2 autonomy tick
        mission.active_nav2_goal = True
        mission.record_tick("LEG1_FORWARD")
        nav_frame = mission.recorder.frames[-1]
        assert nav_frame["cmd_source"] == "ROS_AUTONOMY"
        assert nav_frame["raw_cmd"]["vx"] == 0.10
        assert nav_frame["smoothed_cmd"] is None  # Must NOT be duplicated
        assert nav_frame["final_cmd"]["vx"] == 0.08

    def test_nav2_completion_requires_matching_goal_id(self):
        """Contract: Nav2 completion rejects mismatched or stale prior goal IDs."""
        mission = GoldStandardMission(dry_run=False)
        nav_poll_count = 0
        status_seq = 0

        def mock_status_get(url, **kwargs):
            nonlocal nav_poll_count, status_seq
            status_seq += 1
            resp = MagicMock()
            resp.status_code = 200
            if "/api/navigation/status" in url:
                nav_poll_count += 1
                if nav_poll_count == 1:
                    resp.json.return_value = {"ok": True, "status": "SUCCEEDED", "goal_id": "prior_stale_goal"}
                elif nav_poll_count == 2:
                    resp.json.return_value = {"ok": True, "status": "EXECUTING", "goal_id": "target_goal_99"}
                else:
                    resp.json.return_value = {"ok": True, "status": "SUCCEEDED", "goal_id": "target_goal_99"}
            elif "/api/localization/status" in url:
                resp.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": 1.0, "y": 2.0, "yawDeg": 0.0, "ageMs": 10}
            elif "/api/imu" in url:
                resp.json.return_value = {"ok": True, "serialConnected": True, "dataAgeMs": 10, "stale": False, "sequence": status_seq, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}}
            elif "/api/drive/status" in url:
                # Disarms on SUCCEEDED (nav_poll_count >= 3)
                is_armed = (nav_poll_count < 3)
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": is_armed, "mode": 3 if is_armed else 0,
                        "bootCount": 1, "reqLinear": 0.1 if is_armed else 0.0, "reqAngular": 0,
                        "limLinear": 0.1 if is_armed else 0.0, "limAngular": 0,
                        "cmdSource": "ROS_AUTONOMY" if is_armed else "NONE",
                        "seq": status_seq
                    }
                }
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_status_get), \
             patch("time.sleep", return_value=None):
            res = mission.wait_for_nav2_completion_and_zero(timeout_s=2.0, stage_name="TEST_STAGE", goal_id="target_goal_99")
            assert res is True
            assert nav_poll_count >= 3

    def test_leg4_settle_requires_both_yaw_and_wz_within_tolerance(self):
        """Contract: Final HOME settling requires yaw error <= 3° and |wz| <= 0.02 rad/s for 1.5s."""
        mission = GoldStandardMission(dry_run=False)
        m_time = 100.0
        poll_step = 0
        disarmed = False

        def dynamic_post(url, *args, **kwargs):
            nonlocal disarmed
            resp = MagicMock()
            resp.status_code = 200
            if "/api/drive/disarm" in url:
                disarmed = True
                resp.json.return_value = {"ok": True, "armed": False, "mode": 0}
            elif "/api/command-source" in url:
                src = kwargs.get("json", {}).get("source", "CALIBRATION_TEST") if isinstance(kwargs.get("json"), dict) else "CALIBRATION_TEST"
                resp.json.return_value = {"ok": True, "cmdSource": src}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        def dynamic_get(url, **kwargs):
            nonlocal poll_step, disarmed
            poll_step += 1
            resp = MagicMock()
            resp.status_code = 200
            if "/api/drive/status" in url:
                resp.json.return_value = {
                    "ok": True, "status": {
                        "armed": not disarmed,
                        "mode": 0 if disarmed else 3,
                        "bootCount": 1,
                        "reqLinear": 0, "reqAngular": 0,
                        "limLinear": 0, "limAngular": 0,
                        "cmdSource": "NONE" if disarmed else "CALIBRATION_TEST",
                        "seq": poll_step
                    }
                }
            elif "/api/localization/status" in url:
                resp.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": 1.0, "y": 2.0, "yawDeg": 0.0, "yaw": 0.0, "ageMs": 10}
            elif "/api/imu" in url:
                wz_val = 0.05 if poll_step <= 6 else 0.005
                resp.json.return_value = {"ok": True, "serialConnected": True, "dataAgeMs": 10, "stale": False, "sequence": poll_step, "raw_yaw_deg": 0.0, "gyro": {"z": wz_val}}
            elif "/api/odom" in url:
                resp.json.return_value = {"ok": True, "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "v_x": 0.0, "w_z": 0.0}
            elif "/api/encoders" in url:
                resp.json.return_value = {"ok": True, "sequence": poll_step, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            return resp

        def dynamic_mono():
            nonlocal m_time
            m_time += 0.015
            return m_time

        def mock_sleep(s):
            nonlocal m_time
            m_time += 0.060

        with patch("requests.post", side_effect=dynamic_post), \
             patch("requests.get", side_effect=dynamic_get), \
             patch("time.monotonic", side_effect=dynamic_mono), \
             patch("time.sleep", side_effect=mock_sleep):
            mission.execute_leg4_settle(target_home_yaw_rad=0.0)
            assert poll_step > 6
            assert disarmed is True

    def test_nav2_active_to_succeeded_disarms_to_mode_0(self):
        """Contract: ACTIVE -> SUCCEEDED transition verifies matching goal_id, zero velocities, and Mode 0 disarm."""
        mission = GoldStandardMission(dry_run=False)
        target_goal_id = "dispatch_unique_999"
        nav_poll_count = 0
        drive_poll_count = 0
        status_seq = 0

        def mock_get(url, **kwargs):
            nonlocal nav_poll_count, drive_poll_count, status_seq
            status_seq += 1
            resp = MagicMock()
            resp.status_code = 200
            if "/api/navigation/status" in url:
                nav_poll_count += 1
                if nav_poll_count <= 2:
                    # ACTIVE / EXECUTING on matching goal
                    resp.json.return_value = {
                        "ok": True, "status": "ACTIVE", "goal_id": target_goal_id, "distance_remaining_m": 0.30
                    }
                else:
                    # SUCCEEDED on matching goal
                    resp.json.return_value = {
                        "ok": True, "status": "SUCCEEDED", "goal_id": target_goal_id, "distance_remaining_m": 0.0
                    }
            elif "/api/drive/status" in url:
                drive_poll_count += 1
                # Status remains armed immediately after SUCCEEDED until simulated ESP32 packet confirms disarm
                # (drive_poll_count <= 4 simulates ESP32 packet delay; drive_poll_count >= 5 confirms disarm)
                is_armed = (drive_poll_count <= 4)
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "bootCount": 1,
                        "reqLinear": 0.20 if is_armed else 0.0,
                        "reqAngular": 0.0,
                        "limLinear": 0.20 if is_armed else 0.0,
                        "limAngular": 0.0,
                        "cmdSource": "ROS_AUTONOMY" if is_armed else "NONE",
                        "seq": status_seq
                    }
                }
            elif "/api/localization/status" in url:
                resp.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": 1.0, "y": 2.0, "yawDeg": 0.0, "ageMs": 10}
            elif "/api/imu" in url:
                resp.json.return_value = {"ok": True, "serialConnected": True, "dataAgeMs": 10, "stale": False, "sequence": status_seq, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}}
            elif "/api/odom" in url:
                resp.json.return_value = {"ok": True, "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "v_x": 0.0, "w_z": 0.0}
            elif "/api/encoders" in url:
                resp.json.return_value = {"ok": True, "sequence": status_seq, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_get), \
             patch("time.sleep", return_value=None):
            res = mission.wait_for_nav2_completion_and_zero(timeout_s=2.0, stage_name="TEST_LEG", goal_id=target_goal_id)
            assert res is True
            assert drive_poll_count >= 5  # Proves runner waited through initial armed samples for ESP32 confirmation
            assert mission.active_nav2_goal is False
            assert mission.current_cmd_source == "NONE"

    def test_nav2_disarm_timeout_fails_closed_if_esp32_never_confirms(self):
        """Contract: If ESP32 telemetry never confirms disarm after Nav2 SUCCEEDED, verification times out and fails closed."""
        mission = GoldStandardMission(dry_run=False)
        target_goal_id = "dispatch_unconfirmed_999"
        status_seq = 0
        m_time = 100.0

        nav_polls = 0
        def mock_get(url, **kwargs):
            nonlocal status_seq, nav_polls
            status_seq += 1
            resp = MagicMock()
            resp.status_code = 200
            if "/api/navigation/status" in url:
                nav_polls += 1
                if nav_polls == 1:
                    resp.json.return_value = {
                        "ok": True, "status": "ACTIVE", "goal_id": target_goal_id, "distance_remaining_m": 0.30
                    }
                else:
                    resp.json.return_value = {
                        "ok": True, "status": "SUCCEEDED", "goal_id": target_goal_id, "distance_remaining_m": 0.0
                    }
            elif "/api/drive/status" in url:
                # ESP32 NEVER confirms disarm: remains armed Mode 3
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": True,
                        "mode": 3,
                        "bootCount": 1,
                        "reqLinear": 0.0,
                        "reqAngular": 0.0,
                        "limLinear": 0.0,
                        "limAngular": 0.0,
                        "cmdSource": "ROS_AUTONOMY",
                        "seq": status_seq
                    }
                }
            elif "/api/localization/status" in url:
                resp.json.return_value = {"ok": True, "localized": True, "state": "LOCALIZED", "x": 1.0, "y": 2.0, "yawDeg": 0.0, "ageMs": 10}
            elif "/api/imu" in url:
                resp.json.return_value = {"ok": True, "serialConnected": True, "dataAgeMs": 10, "stale": False, "sequence": status_seq, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}}
            elif "/api/odom" in url:
                resp.json.return_value = {"ok": True, "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "v_x": 0.0, "w_z": 0.0}
            elif "/api/encoders" in url:
                resp.json.return_value = {"ok": True, "sequence": status_seq, "encoders": {"m1": 0, "m2": 0, "m3": 0, "m4": 0}}
            elif "/api/clearance" in url:
                resp.json.return_value = {"ok": True, "piComputed": {"minFwdMm": 1000}, "espConfirmed": {"clearanceMask": 3}}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        def mock_time():
            nonlocal m_time
            m_time += 0.20  # Advance time so 3.0s verify window expires
            return m_time

        with patch("requests.get", side_effect=mock_get), \
             patch("time.monotonic", side_effect=mock_time), \
             patch("time.sleep", return_value=None), \
             patch.object(mission, "disarm_and_stop") as mock_disarm:
            with pytest.raises(MissionAbortException) as exc:
                mission.wait_for_nav2_completion_and_zero(timeout_s=5.0, stage_name="TEST_LEG", goal_id=target_goal_id)
            assert "post-goal check failed" in str(exc.value)
            mock_disarm.assert_called_once()

    def test_missing_or_empty_goal_id_strictly_rejected(self):
        """Contract: Exact nonempty goal_id is required; missing or empty IDs fail closed immediately."""
        mission = GoldStandardMission(dry_run=False)
        with pytest.raises(MissionAbortException) as exc1:
            mission.wait_for_nav2_completion_and_zero(timeout_s=1.0, stage_name="TEST_LEG", goal_id="")
        assert "empty or missing" in str(exc1.value)

        with pytest.raises(MissionAbortException) as exc2:
            mission.wait_for_nav2_completion_and_zero(timeout_s=1.0, stage_name="TEST_LEG", goal_id="   ")
        assert "empty or missing" in str(exc2.value)

    def test_telemetry_records_real_limited_output_for_angular(self):
        """Contract: Real limited output is used whenever either limLinear OR limAngular is nonzero."""
        mission = GoldStandardMission(dry_run=True)
        mission.latest_exact_cmd = {"vx": 0.0, "wz": 0.40}
        # In-place turn: limLinear is 0.0, limAngular is 0.35
        mission.latest_telemetry["drive"] = {
            "reqLinear": 0.0, "reqAngular": 0.40,
            "limLinear": 0.0, "limAngular": 0.35,
            "armed": True, "mode": 3
        }

        mission.active_nav2_goal = False
        mission.record_tick("LEG2_ROTATION")
        last_frame = mission.recorder.frames[-1]
        assert last_frame["cmd_source"] == "CALIBRATION_TEST"
        assert last_frame["raw_cmd"]["vx"] == 0.0
        assert last_frame["raw_cmd"]["wz"] == 0.40
        assert last_frame["smoothed_cmd"] is None
        assert last_frame["final_cmd"]["vx"] == 0.0
        assert last_frame["final_cmd"]["wz"] == 0.35  # Must capture real limited angular velocity!

    def test_delayed_esp32_disarm_confirmation_retries_and_succeeds(self):
        """Contract: disarm() does not abort on immediate stale sample; retries and waits for ESP32 confirmation."""
        mission = GoldStandardMission(dry_run=False)
        poll_count = 0
        status_seq = 0

        def mock_get(url, **kwargs):
            nonlocal poll_count, status_seq
            status_seq += 1
            resp = MagicMock()
            resp.status_code = 200
            if "/api/drive/status" in url:
                poll_count += 1
                # Polls 1-3: ESP32 has not processed disarm yet; still reports armed Mode 3
                # Poll 4+: ESP32 telemetry packet confirms disarmed Mode 0
                is_armed = (poll_count <= 3)
                resp.json.return_value = {
                    "ok": True,
                    "status": {
                        "armed": is_armed,
                        "mode": 3 if is_armed else 0,
                        "bootCount": 1,
                        "reqLinear": 0.0,
                        "reqAngular": 0.0,
                        "limLinear": 0.0,
                        "limAngular": 0.0,
                        "cmdSource": "ROS_AUTONOMY" if is_armed else "NONE",
                        "seq": status_seq
                    }
                }
            elif "/api/imu" in url:
                resp.json.return_value = {"ok": True, "serialConnected": True, "dataAgeMs": 10, "stale": False, "sequence": status_seq, "raw_yaw_deg": 0.0, "gyro": {"z": 0.0}}
            else:
                resp.json.return_value = {"ok": True}
            return resp

        post_disarm_count = 0
        def mock_post(url, **kwargs):
            nonlocal post_disarm_count
            if "/api/drive/disarm" in url:
                post_disarm_count += 1
            resp = MagicMock()
            resp.status_code = 200
            resp.json.return_value = {"ok": True}
            return resp

        with patch("requests.get", side_effect=mock_get), \
             patch("requests.post", side_effect=mock_post), \
             patch("time.sleep", return_value=None):
            res = mission.disarm(timeout_s=3.0)
            assert res is True
            assert poll_count >= 4
            assert post_disarm_count >= 1

    def test_leg1_measures_distance_from_offset_start_pose_along_home_heading(self):
        """Contract: Leg 1 measures 0.6096 m from fresh preflight AMCL start pose along saved HOME heading, while Leg 3 returns to saved HOME."""
        mission = GoldStandardMission(dry_run=True)
        h = mission.home_pose
        # Start pose offset by +3.0 cm X, -2.0 cm Y from saved HOME (hypot = 3.6 cm <= 5.0 cm allowed)
        offset_start = {
            "x": h["x"] + 0.030,
            "y": h["y"] - 0.020,
            "yaw_deg": h["yaw_deg"],
            "yaw_rad": h["yaw_rad"],
            "localized": True,
            "state": "LOCALIZED"
        }
        gate_ok, gate_msg, pos_err, yaw_err = mission.check_pre_arm_gate(offset_start)
        assert gate_ok is True
        assert pos_err < 0.05

        targets = mission.compute_mission_targets(start_pose=offset_start)

        # Leg 1 outbound starts from offset_start along saved HOME heading
        expected_l1_x = offset_start["x"] + FORWARD_DISTANCE_M * math.cos(h["yaw_rad"])
        expected_l1_y = offset_start["y"] + FORWARD_DISTANCE_M * math.sin(h["yaw_rad"])
        assert math.isclose(targets["leg1_outbound"]["x"], expected_l1_x, abs_tol=1e-4)
        assert math.isclose(targets["leg1_outbound"]["y"], expected_l1_y, abs_tol=1e-4)
        assert math.isclose(targets["leg1_outbound"]["yaw_rad"], h["yaw_rad"], abs_tol=1e-4)

        # Leg 3 return MUST still target exact authoritative saved HOME coordinates (not offset start!)
        assert math.isclose(targets["leg3_return"]["x"], h["x"], abs_tol=1e-4)
        assert math.isclose(targets["leg3_return"]["y"], h["y"], abs_tol=1e-4)

    def test_baseline_telemetry_frame_captured_and_graded(self):
        """Contract: MissionGrader grades outbound distance relative to BASELINE telemetry frame."""
        h_x, h_y = 1.0, 2.0
        start_x, start_y = 1.03, 1.98 # Offset start pose
        end_l1_x = start_x + FORWARD_DISTANCE_M
        end_l1_y = start_y

        run_data = {
            "metadata": {"total_frames": 3, "sample_rate_hz": 25.0},
            "transitions": [],
            "samples": [
                {
                    "t_rel_s": 0.0, "mission_stage": "BASELINE",
                    "amcl": {"x": start_x, "y": start_y, "yaw_deg": 0.0, "localized": True},
                    "drive": {"armed": False, "mode": 0, "reqLinear": 0, "reqAngular": 0, "limLinear": 0, "limAngular": 0, "cmdSource": "NONE"},
                    "to_home": {"pos_err_m": 0.036, "yaw_err_deg": 0.0}
                },
                {
                    "t_rel_s": 1.0, "mission_stage": "LEG1_FORWARD",
                    "amcl": {"x": end_l1_x, "y": end_l1_y, "yaw_deg": 0.0, "localized": True},
                    "drive": {"armed": True, "mode": 3, "reqLinear": 0.20, "reqAngular": 0, "limLinear": 0.20, "limAngular": 0, "cmdSource": "ROS_AUTONOMY"},
                    "to_home": {"pos_err_m": 0.64, "yaw_err_deg": 0.0}
                },
                {
                    "t_rel_s": 2.0, "mission_stage": "FINAL_DISARMED",
                    "amcl": {"x": h_x, "y": h_y, "yaw_deg": 0.0, "localized": True},
                    "drive": {"armed": False, "mode": 0, "reqLinear": 0, "reqAngular": 0, "limLinear": 0, "limAngular": 0, "cmdSource": "NONE"},
                    "to_home": {"pos_err_m": 0.0, "yaw_err_deg": 0.0}
                }
            ]
        }
        grade = MissionGrader.grade_run(run_data)
        assert math.isclose(grade["metrics"]["outbound_distance_m"], FORWARD_DISTANCE_M, abs_tol=1e-4)
        assert math.isclose(grade["metrics"]["outbound_distance_error_m"], 0.0, abs_tol=1e-4)

    def test_delayed_stationary_telemetry_waits_and_dispatches(self):
        """Contract: Runner waits on delayed is_stationary telemetry until production predicate passes, then proceeds."""
        mission = GoldStandardMission(dry_run=False)
        loc_calls = 0

        with patch("requests.get") as mock_get, patch("requests.post") as mock_post:
            def mock_get_router(url, **kwargs):
                nonlocal loc_calls
                resp = MagicMock()
                resp.status_code = 200
                if "/api/localization/status" in url:
                    loc_calls += 1
                    # Delayed: first 4 calls return is_stationary=False, then True
                    is_stat = (loc_calls >= 5)
                    resp.json.return_value = {
                        "ok": True, "localized": True, "state": "LOCALIZED",
                        "is_stationary": is_stat, "isStationary": is_stat,
                        "seq": loc_calls, "ageMs": 50, "x": 1.0, "y": 2.0, "yawDeg": 0.0
                    }
                elif "/api/drive/status" in url:
                    resp.json.return_value = {
                        "ok": True,
                        "status": {
                            "armed": False, "mode": 0, "cmdSource": "NONE",
                            "reqLinear": 0.0, "reqAngular": 0.0,
                            "limLinear": 0.0, "limAngular": 0.0,
                            "seq": loc_calls
                        }
                    }
                elif "/api/imu" in url:
                    resp.json.return_value = {"raw_yaw_deg": 0.0, "gyro_z": 0.0, "serialConnected": True, "sequence": loc_calls}
                else:
                    resp.json.return_value = {}
                return resp

            mock_get.side_effect = mock_get_router
            mock_post.return_value.status_code = 200

            res = mission.wait_for_stationary_at_rest(timeout_s=3.0)
            assert res is True
            assert loc_calls >= 5

    def test_stationary_telemetry_timeout_fails_closed_and_disarms(self):
        """Contract: When is_stationary is never confirmed, wait_for_stationary_at_rest times out and disarms."""
        mission = GoldStandardMission(dry_run=False)

        with patch("requests.get") as mock_get, patch("requests.post") as mock_post:
            def mock_get_router(url, **kwargs):
                resp = MagicMock()
                resp.status_code = 200
                if "/api/localization/status" in url:
                    # Persistently not stationary
                    resp.json.return_value = {
                        "ok": True, "localized": True, "state": "LOCALIZED",
                        "is_stationary": False, "isStationary": False,
                        "seq": 100, "ageMs": 50
                    }
                elif "/api/drive/status" in url:
                    resp.json.return_value = {
                        "ok": True,
                        "status": {"armed": False, "mode": 0, "cmdSource": "NONE", "reqLinear": 0, "reqAngular": 0, "limLinear": 0, "limAngular": 0}
                    }
                elif "/api/imu" in url:
                    resp.json.return_value = {"raw_yaw_deg": 0.0, "gyro_z": 0.0, "serialConnected": True, "sequence": 100}
                else:
                    resp.json.return_value = {}
                return resp

            mock_get.side_effect = mock_get_router
            mock_post.return_value.status_code = 200

            with pytest.raises(MissionAbortException) as exc_info:
                mission.wait_for_stationary_at_rest(timeout_s=0.2)
            assert "waiting for Cockpit stationary-at-rest confirmation" in str(exc_info.value)

    def test_grader_separately_reports_linear_and_angular_settling_reversals(self):
        """Contract: MissionGrader separately reports linear reversals and angular settling reversals with settling time."""
        run_data = {
            "metadata": {"total_frames": 10, "duration_s": 10.0, "sample_rate_hz": 25.0},
            "transitions": [],
            "samples": [
                # BASELINE
                {"t_rel_s": 0.0, "mission_stage": "BASELINE", "amcl": {"x": 1.0, "y": 2.0, "yaw_deg": 0.0}, "drive": {"armed": False, "mode": 0}},
                # Leg 1: contains 1 linear command reversal (+0.20 -> -0.05)
                {"t_rel_s": 1.0, "mission_stage": "LEG1_FORWARD", "final_cmd": {"vx": 0.20, "wz": 0.0}, "amcl": {"x": 1.3, "y": 2.0}, "drive": {"armed": True, "mode": 3}},
                {"t_rel_s": 2.0, "mission_stage": "LEG1_FORWARD", "final_cmd": {"vx": -0.05, "wz": 0.0}, "amcl": {"x": 1.6, "y": 2.0}, "drive": {"armed": True, "mode": 3}},
                # Leg 2: starts rotation from 0°, crosses 180° at t=4.0, then reverses angular command (-0.18 -> +0.10)
                {"t_rel_s": 3.0, "mission_stage": "LEG2_ROTATION", "imu": {"raw_yaw_deg": 0.0}, "final_cmd": {"vx": 0.0, "wz": -0.40}, "drive": {"armed": True, "mode": 3}},
                {"t_rel_s": 4.0, "mission_stage": "LEG2_ROTATION", "imu": {"raw_yaw_deg": -180.0}, "final_cmd": {"vx": 0.0, "wz": -0.18}, "drive": {"armed": True, "mode": 3}},
                {"t_rel_s": 5.0, "mission_stage": "LEG2_ROTATION", "imu": {"raw_yaw_deg": -182.0}, "final_cmd": {"vx": 0.0, "wz": 0.10}, "drive": {"armed": True, "mode": 3}},
                # FINAL_DISARMED
                {"t_rel_s": 6.0, "mission_stage": "FINAL_DISARMED", "to_home": {"pos_err_m": 0.0, "yaw_err_deg": 0.0}, "drive": {"armed": False, "mode": 0, "reqLinear": 0, "reqAngular": 0}}
            ]
        }

        grade = MissionGrader.grade_run(run_data)
        crit_rev = grade["criteria"]["corrective_reversals"]
        perf = grade["performance_metrics"]

        assert crit_rev["linear_reversals"] == 1
        assert crit_rev["angular_settling_reversals"] == 1
        assert crit_rev["count"] == 2
        assert crit_rev["settling_time_after_crossing_s"] == 1.0 # 5.0s - 4.0s
        assert perf["linear_reversals"] == 1
        assert perf["angular_settling_reversals"] == 1
        assert perf["settling_time_after_target_crossing_s"] == 1.0


    def test_leg1_forward_dispatches_with_position_goal_checker(self):
        """Prove Leg 1 dispatches to Nav2 with position_goal_checker to prevent terminal yaw reversal."""
        mission = GoldStandardMission(dry_run=False)
        dispatched_payloads = []

        def mock_post(url, json=None, headers=None, timeout=None):
            class MockResp:
                status_code = 200
                text = '{"ok": true, "goal_id": "test_goal_123"}'
                def json(self):
                    return {"ok": True, "goal_id": "test_goal_123"}
            if "/api/navigation/dispatch" in url:
                dispatched_payloads.append(json)
            return MockResp()

        nav_call_count = [0]
        def mock_get(url, headers=None, timeout=None):
            class MockGetResp:
                status_code = 200
                def json(self):
                    if "/api/navigation/status" in url:
                        nav_call_count[0] += 1
                        st = "EXECUTING" if nav_call_count[0] == 1 else "SUCCEEDED"
                        return {"ok": True, "status": st, "goal_id": "test_goal_123"}
                    elif "/api/drive/status" in url:
                        return {"armed": False, "mode": 0, "reqLinear": 0, "reqAngular": 0, "limLinear": 0, "limAngular": 0, "cmdSource": "NONE"}
                    return {"ok": True}
            return MockGetResp()

        mission._http_post = mock_post
        mission._http_get = mock_get
        # Short-circuit poll_all_telemetry and verify_safety_invariants for this unit test
        mission.poll_all_telemetry = lambda: None
        mission.verify_safety_invariants = lambda stage: None
        mission.record_tick = lambda stage: None
        mission.latest_telemetry["drive"] = {"armed": False, "mode": 0, "reqLinear": 0, "reqAngular": 0, "limLinear": 0, "limAngular": 0, "cmdSource": "NONE"}

        mission.execute_leg1_forward(target_x=1.7993, target_y=-0.1012, target_yaw=-0.0881)

        assert len(dispatched_payloads) == 1
        payload = dispatched_payloads[0]
        assert payload["target_x"] == 1.7993
        assert payload["target_y"] == -0.1012
        assert payload["target_yaw"] == -0.0881
        assert payload.get("goal_checker") == "position_goal_checker"

    def test_sampling_rate_honesty_active_motion_vs_whole_run_coverage(self):
        """
        Prove the honest distinction between:
        - Active motion sample rate: (N-1) / (t_last - t_first) = (437-1)/19.305 = 22.59 Hz
        - Whole-run coverage rate: N / duration_s = 437 / 25.81 = 16.93 Hz
        """
        # 437 samples spanning from t_rel_s = 6.505 to 25.810 over duration 25.81s
        samples = []
        n_samples = 437
        t_start = 6.5049
        t_end = 25.8097
        dt = (t_end - t_start) / (n_samples - 1)
        for i in range(n_samples):
            t = t_start + i * dt
            samples.append({
                "t_rel_s": t,
                "mission_stage": "LEG1_FORWARD" if i < 200 else "LEG2_ROTATION",
                "final_cmd": {"vx": 0.0, "wz": 0.0},
                "amcl": {"x": 1.0, "y": 2.0, "yaw_deg": 0.0},
                "drive": {"armed": False, "mode": 0, "bootCount": 1},
                "to_home": {"pos_err_m": 0.0, "yaw_err_deg": 0.0}
            })

        run_data = {
            "metadata": {
                "duration_s": 25.81,
                "total_frames": 437,
                "sample_rate_hz": 22.59,
                "active_motion_rate_hz": 22.59,
                "whole_run_coverage_hz": 16.93
            },
            "samples": samples
        }

        grade = MissionGrader.grade_run(run_data)
        rate_crit = grade["criteria"]["recorder_sample_rate"]
        perf = grade["performance_metrics"]

        assert rate_crit["active_motion_rate_hz"] == 22.59
        assert rate_crit["whole_run_coverage_hz"] == 16.93
        assert rate_crit["active_span_s"] == round(t_end - t_start, 2)
        assert rate_crit["total_duration_s"] == 25.81
        assert rate_crit["passed"] is True  # 22.59 >= 20.0 Hz threshold

        assert perf["active_motion_rate_hz"] == 22.59
        assert perf["whole_run_coverage_hz"] == 16.93

    def test_telemetry_recorder_save_records_honest_sampling_metrics(self, tmp_path):
        """Verify TelemetryRecorder saves both active-motion rate and whole-run coverage."""
        from tools.gold_standard_home_test.telemetry import TelemetryRecorder, TelemetryFrame
        rec = TelemetryRecorder(output_dir=str(tmp_path))
        rec.t0 = 100.0
        # Frame 0 at t=106.5s (after 6.5s preflight), Frame 1 at t=107.5s
        frame0 = TelemetryFrame(
            t_rel_s=6.5, timestamp_epoch=1700000006.5, mission_stage="BASELINE",
            raw_cmd={"vx": 0, "wz": 0}, smoothed_cmd=None, final_cmd={"vx": 0, "wz": 0},
            cmd_source="NONE", amcl={}, odom={}, encoders=None, imu={}, drive={},
            collision_monitor={}, to_home={}
        )
        frame1 = TelemetryFrame(
            t_rel_s=7.5, timestamp_epoch=1700000007.5, mission_stage="LEG1_FORWARD",
            raw_cmd={"vx": 0.2, "wz": 0}, smoothed_cmd=None, final_cmd={"vx": 0.2, "wz": 0},
            cmd_source="ROS_AUTONOMY", amcl={}, odom={}, encoders=None, imu={}, drive={},
            collision_monitor={}, to_home={}
        )
        rec.record_frame(frame0)
        rec.record_frame(frame1)

        import time
        orig_monotonic = time.monotonic
        try:
            # Simulate total elapsed time of 10.0 seconds from rec.t0
            time.monotonic = lambda: 110.0
            filepath = rec.save(run_id="test_sampling_honesty")
        finally:
            time.monotonic = orig_monotonic

        import json
        with open(filepath, "r", encoding="utf-8") as f:
            saved = json.load(f)

        meta = saved["metadata"]
        assert meta["total_frames"] == 2
        assert meta["duration_s"] == 10.0
        assert meta["active_span_s"] == 1.0  # 7.5s - 6.5s
        assert meta["active_motion_rate_hz"] == 1.0  # (2-1)/1.0s = 1.0 Hz
        assert meta["whole_run_coverage_hz"] == 0.2  # 2 / 10.0s = 0.2 Hz
