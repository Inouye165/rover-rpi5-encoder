# ==============================================================================
# test_encoder_kinematics.py — Comprehensive Unit Tests for Encoder Kinematics
# ==============================================================================

import math
import pytest

from rover_bringup.encoder_kinematics import EncoderKinematics, compute_ticks_delta, normalize_angle


def test_normalize_angle():
    assert abs(normalize_angle(0.0)) < 1e-6
    assert abs(normalize_angle(2 * math.pi)) < 1e-6
    assert abs(normalize_angle(3 * math.pi) - math.pi) < 1e-5
    assert abs(normalize_angle(-3 * math.pi) - (-math.pi)) < 1e-5


def test_compute_ticks_delta_basic():
    # Initial / None previous
    assert compute_ticks_delta(100, None) == 0

    # Normal delta
    assert compute_ticks_delta(150, 100) == 50
    assert compute_ticks_delta(80, 100) == -20


def test_compute_ticks_delta_rollover():
    # Positive overflow: 2^31 - 1 -> -2^31
    prev = 2147483640
    curr = -2147483640
    delta = compute_ticks_delta(curr, prev)
    assert delta == 16  # 2147483647 - 2147483640 + 1 + (curr - (-2147483648))

    # Negative underflow: -2^31 -> 2^31 - 1
    prev = -2147483640
    curr = 2147483640
    delta = compute_ticks_delta(curr, prev)
    assert delta == -16


test_compute_ticks_delta_rollover()


def test_compute_ticks_delta_reset():
    # Counter reset after reboot (e.g. from 500,000 back to 0)
    assert compute_ticks_delta(0, 500000, reset_threshold=100000) == 0


def test_straight_movement():
    kin = EncoderKinematics(wheel_radius_m=0.0325, track_width_m=0.718, ticks_per_revolution=1894.0)
    # Baseline
    kin.update([0, 0, 0, 0], 100.0)

    # 1 rev forward on all wheels = 1894.0 ticks
    ticks_1rev = 1894.0
    ok, msg = kin.update([ticks_1rev, ticks_1rev, ticks_1rev, ticks_1rev], 101.0)
    assert ok
    # Distance = 2 * pi * 0.0325 = 0.2042035m
    expected_dist = 2.0 * math.pi * 0.0325
    assert abs(kin.x - expected_dist) < 1e-3
    assert abs(kin.y) < 1e-4
    assert abs(kin.yaw) < 1e-4
    assert abs(kin.v_x - expected_dist) < 1e-3
    assert abs(kin.w_z) < 1e-4


def test_reverse_movement():
    kin = EncoderKinematics(wheel_radius_m=0.0325, track_width_m=0.718, ticks_per_revolution=1894.0)
    kin.update([1000, 1000, 1000, 1000], 100.0)

    # Move backwards by 1 rev
    ticks_1rev = 1894.0
    ok, msg = kin.update([1000 - ticks_1rev, 1000 - ticks_1rev, 1000 - ticks_1rev, 1000 - ticks_1rev], 101.0)
    assert ok
    expected_dist = -(2.0 * math.pi * 0.0325)
    assert abs(kin.x - expected_dist) < 1e-3
    assert kin.x < 0.0
    assert abs(kin.y) < 1e-4
    assert abs(kin.yaw) < 1e-4


def test_in_place_ccw_turn():
    kin = EncoderKinematics(wheel_radius_m=0.0325, track_width_m=0.718, ticks_per_revolution=1894.0)
    kin.update([0, 0, 0, 0], 100.0)

    # CCW turn: Left backward (-500 ticks), Right forward (+500 ticks)
    ok, msg = kin.update([-500, 500, -500, 500], 101.0)
    assert ok
    assert kin.yaw > 0.0  # Positive yaw
    assert kin.w_z > 0.0
    assert abs(kin.x) < 0.01  # In-place turn stays near origin


def test_in_place_cw_turn():
    kin = EncoderKinematics(wheel_radius_m=0.0325, track_width_m=0.718, ticks_per_revolution=1894.0)
    kin.update([0, 0, 0, 0], 100.0)

    # CW turn: Left forward (+500 ticks), Right backward (-500 ticks)
    ok, msg = kin.update([500, -500, 500, -500], 101.0)
    assert ok
    assert kin.yaw < 0.0  # Negative yaw
    assert kin.w_z < 0.0


def test_curved_motion():
    kin = EncoderKinematics(wheel_radius_m=0.0325, track_width_m=0.718, ticks_per_revolution=1894.0)
    kin.update([0, 0, 0, 0], 100.0)

    # Left side moves slower than right side
    ok, msg = kin.update([200, 400, 200, 400], 101.0)
    assert ok
    assert kin.x > 0.0
    assert kin.y > 0.0
    assert kin.yaw > 0.0


def test_effective_track_width_rotation_kinematics():
    """Verify that equal and opposite wheel travel of ~2.256m per side yields 2pi radians (360 deg) rotation with 0.718m effective width."""
    track_width_m = 0.718
    physical_track_width_m = 0.197
    wheel_radius_m = 0.0325
    ticks_per_rev = 1894.0

    kin = EncoderKinematics(wheel_radius_m=wheel_radius_m, track_width_m=track_width_m, ticks_per_revolution=ticks_per_rev)
    kin.update([0, 0, 0, 0], 100.0)

    # Wheel distance s = pi * track_width_m = 2.25566359... ~ 2.256 m
    s_wheel = math.pi * track_width_m
    ticks_per_meter = ticks_per_rev / (2.0 * math.pi * wheel_radius_m)
    ticks_delta = s_wheel * ticks_per_meter

    # Left backward (-ticks_delta), Right forward (+ticks_delta)
    ok, msg = kin.update([-ticks_delta, ticks_delta, -ticks_delta, ticks_delta], 101.0)
    assert ok
    # Rotation delta_theta = (s_right - s_left) / W = 2 * pi rad -> normalizes to 0
    assert abs(normalize_angle(kin.yaw)) < 1e-3
    assert abs(s_wheel - 2.25566) < 0.001
    assert abs(physical_track_width_m - 0.197) < 1e-5


def test_four_wheel_grouping_and_disagreement():
    kin = EncoderKinematics(disagreement_threshold_ticks=50)
    kin.update([0, 0, 0, 0], 100.0)

    # Front and rear on left match (100, 100), front and rear on right mismatch (100 vs 300)
    ok, msg = kin.update([100, 100, 100, 300], 101.0)
    assert ok
    assert kin.disagreement_warning is True
    assert "left_diff=0, right_diff=200" in kin.disagreement_details


def test_sign_normalization_and_reversal():
    # If M1 was inverted (-1.0 sign)
    kin = EncoderKinematics(m1_sign=-1.0)
    kin.update([0, 0, 0, 0], 100.0)

    # M1 ticks decreased by 100, which with m1_sign=-1 yields +100 normalized ticks
    ok, msg = kin.update([-100, 100, 100, 100], 101.0)
    assert ok
    assert kin.x > 0.0


def test_rejection_duplicate_telemetry():
    kin = EncoderKinematics()
    kin.update([10, 10, 10, 10], 100.0, sequence=1)

    # Same timestamp / sequence
    ok, msg = kin.update([10, 10, 10, 10], 100.0, sequence=1)
    assert ok is False
    assert "Non-positive time delta" in msg or "Duplicate sequence" in msg


def test_rejection_stale_telemetry():
    kin = EncoderKinematics(stale_timeout_sec=2.0)
    kin.update([10, 10, 10, 10], 100.0)

    # 5 second jump > 2s timeout
    ok, msg = kin.update([20, 20, 20, 20], 105.0)
    assert ok is False
    assert "Stale telemetry gap" in msg


def test_rejection_impossible_count_jump():
    kin = EncoderKinematics(max_plausible_wheel_speed_mps=2.0)
    kin.update([0, 0, 0, 0], 100.0)

    # 100,000 count jump in 0.1s is impossible physically
    ok, msg = kin.update([100000, 100000, 100000, 100000], 100.1)
    assert ok is False
    assert "Impossible count jump" in msg


def test_finite_odometry_pose_and_twist():
    kin = EncoderKinematics()
    kin.update([0, 0, 0, 0], 100.0)
    kin.update([500, 500, 500, 500], 101.0)

    assert math.isfinite(kin.x)
    assert math.isfinite(kin.y)
    assert math.isfinite(kin.yaw)
    assert math.isfinite(kin.v_x)
    assert math.isfinite(kin.w_z)
