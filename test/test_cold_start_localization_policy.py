import os
import sys
import json
import yaml
import pytest
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2", "ros2_ws", "src", "rover_bringup"))

from rover_bringup.scan_validator import validate_laserscan_msg, validate_scan_to_map

MAP_YAML_PATH = os.path.join(REPO_ROOT, "ros2", "maps", "production", "house_slam_2026-09-23_candidate_hallway.yaml")
MAP_PGM_PATH = os.path.join(REPO_ROOT, "ros2", "maps", "production", "house_slam_2026-09-23_candidate_hallway.pgm")
HOME_JSON_PATH = os.path.join(REPO_ROOT, "ros2", "maps", "production", "home_pose_slam.json")
FIXTURE_SCAN_PATH = os.path.join(REPO_ROOT, "test", "fixtures", "home_reference_scan.json")


@pytest.fixture(scope="module")
def production_map_dict():
    assert os.path.isfile(MAP_YAML_PATH), f"Map YAML missing: {MAP_YAML_PATH}"
    assert os.path.isfile(MAP_PGM_PATH), f"Map PGM missing: {MAP_PGM_PATH}"

    with open(MAP_YAML_PATH, "r", encoding="utf-8") as f:
        ydata = yaml.safe_load(f)

    with open(MAP_PGM_PATH, "rb") as f:
        buf = f.read()

    newlines = 0
    header_end = 0
    for i in range(len(buf)):
        if buf[i] == 0x0A:
            newlines += 1
            if newlines == 3:
                header_end = i + 1
                break

    header_str = buf[:header_end].decode("ascii", errors="ignore")
    lines = [l.strip() for l in header_str.splitlines() if l.strip() and not l.startswith("#")]
    w, h = map(int, lines[1].split())
    pixels = np.frombuffer(buf[header_end:], dtype=np.uint8).reshape((h, w))

    return {
        "width": w,
        "height": h,
        "resolution": float(ydata.get("resolution", 0.05)),
        "origin": ydata.get("origin", [-4.202, -5.1, 0.0]),
        "pixels": pixels
    }


@pytest.fixture(scope="module")
def authoritative_home_dict():
    assert os.path.isfile(HOME_JSON_PATH), f"Home JSON missing: {HOME_JSON_PATH}"
    with open(HOME_JSON_PATH, "r", encoding="utf-8") as f:
        hj = json.load(f)

    pos = hj.get("pose", {}).get("position", {})
    ori = hj.get("pose", {})
    return {
        "x": float(pos.get("x", 1.193853)),
        "y": float(pos.get("y", -0.045221)),
        "yaw_deg": float(ori.get("yaw_deg", -5.047)),
        "yaw_rad": float(ori.get("yaw_rad", -0.088087))
    }


@pytest.fixture(scope="module")
def reference_scan_msg():
    assert os.path.isfile(FIXTURE_SCAN_PATH), f"Scan fixture missing: {FIXTURE_SCAN_PATH}"
    with open(FIXTURE_SCAN_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def test_verified_home_case(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 1: Rover physically placed at verified HOME tape mark.
    Scan-to-map validation MUST strongly confirm HOME with >= 75% overlap.
    """
    now = reference_scan_msg["timestamp_sec"] + 0.1
    res = validate_laserscan_msg(
        scan_msg=reference_scan_msg,
        map_dict=production_map_dict,
        home_dict=authoritative_home_dict,
        now_sec=now,
        max_age_sec=0.75,
        min_overlap=0.75
    )

    assert res["ok"] is True, f"Expected verified HOME, got: {res}"
    assert res["reason"] == "VERIFIED_HOME"
    assert res["overlap"] >= 0.75, f"Expected overlap >= 0.75, got {res['overlap']}"
    assert res["hits"] > 200


def test_scan_mismatch_displaced_y(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 2A: Rover displaced by +25cm in lateral Y from HOME.
    Validation MUST reject with SCAN_MISMATCH (overlap < 75%) and block /initialpose.
    """
    displaced_home = dict(authoritative_home_dict)
    displaced_home["y"] += 0.25

    now = reference_scan_msg["timestamp_sec"] + 0.1
    res = validate_laserscan_msg(
        scan_msg=reference_scan_msg,
        map_dict=production_map_dict,
        home_dict=displaced_home,
        now_sec=now,
        max_age_sec=0.75,
        min_overlap=0.75
    )

    assert res["ok"] is False, f"Expected rejection, got: {res}"
    assert res["reason"] == "SCAN_MISMATCH"
    assert res["overlap"] < 0.75


def test_scan_mismatch_rotated_yaw(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 2B: Rover rotated by 25° from HOME orientation.
    Validation MUST reject with SCAN_MISMATCH and block /initialpose.
    """
    rotated_home = dict(authoritative_home_dict)
    rotated_home["yaw_deg"] += 25.0
    import math
    rotated_home["yaw_rad"] = math.radians(rotated_home["yaw_deg"])

    now = reference_scan_msg["timestamp_sec"] + 0.1
    res = validate_laserscan_msg(
        scan_msg=reference_scan_msg,
        map_dict=production_map_dict,
        home_dict=rotated_home,
        now_sec=now,
        max_age_sec=0.75,
        min_overlap=0.75
    )

    assert res["ok"] is False, f"Expected rejection, got: {res}"
    assert res["reason"] == "SCAN_MISMATCH"
    assert res["overlap"] < 0.75


def test_scan_mismatch_different_room(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 2C: Rover located in corridor/bedroom rather than office HOME.
    Validation MUST reject with SCAN_MISMATCH.
    """
    diff_room_home = dict(authoritative_home_dict)
    diff_room_home["x"] = 2.50
    diff_room_home["y"] = 0.50
    diff_room_home["yaw_deg"] = 90.0
    import math
    diff_room_home["yaw_rad"] = math.radians(90.0)

    now = reference_scan_msg["timestamp_sec"] + 0.1
    res = validate_laserscan_msg(
        scan_msg=reference_scan_msg,
        map_dict=production_map_dict,
        home_dict=diff_room_home,
        now_sec=now,
        max_age_sec=0.75,
        min_overlap=0.75
    )

    assert res["ok"] is False, f"Expected rejection, got: {res}"
    assert res["reason"] == "SCAN_MISMATCH"
    assert res["overlap"] < 0.50


def test_missing_scan_case(production_map_dict, authoritative_home_dict):
    """
    Case 3: No LiDAR scan available at startup (e.g. sensor startup delay or failure).
    Validation MUST reject with MISSING_SCAN and never inject /initialpose.
    """
    res_none = validate_laserscan_msg(
        scan_msg=None,
        map_dict=production_map_dict,
        home_dict=authoritative_home_dict
    )
    assert res_none["ok"] is False
    assert res_none["reason"] == "MISSING_SCAN"

    empty_scan = {
        "timestamp_sec": 1000.0,
        "ranges": []
    }
    res_empty = validate_laserscan_msg(
        scan_msg=empty_scan,
        map_dict=production_map_dict,
        home_dict=authoritative_home_dict
    )
    assert res_empty["ok"] is False
    assert res_empty["reason"] == "MISSING_SCAN"


def test_stale_scan_case(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 4: Scan timestamp is stale (> 0.75s old).
    Validation MUST reject with STALE_SCAN to prevent using outdated sensor data.
    """
    stale_now = reference_scan_msg["timestamp_sec"] + 5.0
    res = validate_laserscan_msg(
        scan_msg=reference_scan_msg,
        map_dict=production_map_dict,
        home_dict=authoritative_home_dict,
        now_sec=stale_now,
        max_age_sec=0.75,
        min_overlap=0.75
    )

    assert res["ok"] is False
    assert res["reason"] == "STALE_SCAN"
    assert "exceeds freshness threshold" in res["details"]


def test_insufficient_beams_case(production_map_dict, authoritative_home_dict, reference_scan_msg):
    """
    Case 5: LiDAR returns mostly obstructed or out-of-range (< 100 valid beams).
    Validation MUST reject with INSUFFICIENT_BEAMS.
    """
    few_beams_scan = dict(reference_scan_msg)
    # Mask all but 15 beams to inf
    sparse_ranges = [float("inf")] * len(reference_scan_msg["ranges"])
    for i in range(15):
        sparse_ranges[i * 20] = 1.5
    few_beams_scan["ranges"] = sparse_ranges

    now = reference_scan_msg["timestamp_sec"] + 0.1
    res = validate_laserscan_msg(
        scan_msg=few_beams_scan,
        map_dict=production_map_dict,
        home_dict=authoritative_home_dict,
        now_sec=now,
        max_age_sec=0.75,
        min_overlap=0.75,
        min_beams=100
    )

    assert res["ok"] is False
    assert res["reason"] == "INSUFFICIENT_BEAMS"
