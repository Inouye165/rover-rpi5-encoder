# ==============================================================================
# test_encoder_slip_gating.py — Unit Tests for Odometry Slip Gating (Phase 4)
# ==============================================================================

import math
import pytest
from rover_bringup.encoder_kinematics import EncoderKinematics, signed_min_magnitude


def create_kinematics():
    """Helper to instantiate clean EncoderKinematics engine."""
    return EncoderKinematics(
        wheel_radius_m=0.0325,
        track_width_m=0.3408575433,
        ticks_per_revolution=1974.1666666667,
        stale_timeout_sec=2.0,
        max_plausible_wheel_speed_mps=2.5,
    )


def test_signed_min_magnitude():
    # Equal positive
    assert signed_min_magnitude(0.05, 0.05) == 0.05
    # Unequal positive
    assert signed_min_magnitude(0.20, 0.01) == 0.01
    assert signed_min_magnitude(0.01, 0.20) == 0.01
    # Equal negative
    assert signed_min_magnitude(-0.05, -0.05) == -0.05
    # Unequal negative
    assert signed_min_magnitude(-0.20, -0.01) == -0.01
    # Opposite signs (in-place turn)
    assert signed_min_magnitude(0.10, -0.10) == 0.0


def test_scenario_a_straight_forward_equal_wheels():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # 50ms later, both sides advance 100 ticks
    success, msg = k.update([1100, 1100, 1100, 1100], timestamp_sec=100.05, sequence=2)
    assert success is True
    assert k.slip_gate_active is False
    assert k.last_gated_d_center_m > 0.009
    assert abs(k.last_ungated_d_center_m - k.last_gated_d_center_m) < 1e-6


def test_scenario_b_straight_reverse_equal_wheels():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # 50ms later, both sides reverse 100 ticks
    success, msg = k.update([900, 900, 900, 900], timestamp_sec=100.05, sequence=2)
    assert success is True
    assert k.slip_gate_active is False
    assert k.last_gated_d_center_m < -0.009


def test_scenario_c_normal_gentle_turn():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Left wheel advances 150 ticks, right wheel advances 100 ticks
    # Wheel yaw delta = (0.01035 - 0.01553) / 0.340857 = -0.0152 rad
    external_yaw_delta = -0.0152
    success, msg = k.update([1150, 1100, 1150, 1100], timestamp_sec=100.05, sequence=2, external_d_yaw=external_yaw_delta)
    
    assert success is True
    # In a normal gentle turn, IMU matches wheel rotation, so slip gate stays INACTIVE
    assert k.slip_gate_active is False


def test_scenario_d_inplace_turn():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Left forward +100 ticks, Right reverse -100 ticks
    # Wheel yaw delta = (-0.01035 - 0.01035) / 0.340857 = -0.0607 rad
    external_yaw_delta = -0.0607
    success, msg = k.update([1100, 900, 1100, 900], timestamp_sec=100.05, sequence=2, external_d_yaw=external_yaw_delta)
    
    assert success is True
    assert k.slip_gate_active is False
    assert abs(k.last_gated_d_center_m) < 1e-6


def test_scenario_e_severe_left_wheel_spin():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Severe slip: Left spins +308 ticks (~0.0319m), Right moves +5 ticks (~0.0005m)
    # IMU reports negligible yaw delta (~0.001 rad)
    success, msg = k.update([1308, 1005, 1308, 1005], timestamp_sec=100.05, sequence=2, external_d_yaw=0.001)
    
    assert success is True
    assert k.slip_gate_active is True
    assert k.slip_event_count == 1
    # Gated translation should be small (~0.0005m from right wheel) instead of ~0.0162m (ungated)
    assert k.last_gated_d_center_m < 0.001
    assert k.last_ungated_d_center_m > 0.015


def test_scenario_f_severe_right_wheel_spin():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Severe slip: Right spins +308 ticks, Left moves +5 ticks
    success, msg = k.update([1005, 1308, 1005, 1308], timestamp_sec=100.05, sequence=2, external_d_yaw=0.001)
    
    assert success is True
    assert k.slip_gate_active is True
    assert k.last_gated_d_center_m < 0.001


def test_scenario_g_reverse_direction_slip():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Reverse slip: Left spins -308 ticks, Right moves -5 ticks
    success, msg = k.update([692, 995, 692, 995], timestamp_sec=100.05, sequence=2, external_d_yaw=-0.001)
    
    assert success is True
    assert k.slip_gate_active is True
    # Gated translation should be conservative negative step (~ -0.0005m)
    assert -0.001 < k.last_gated_d_center_m < 0.0


def test_scenario_h_implausibly_huge_deltas_sanity_cap():
    # Instantiate engine with 1.5 m/s max speed (max single wheel = 14491 ticks/sec -> 724 ticks per 50ms)
    k = EncoderKinematics(
        wheel_radius_m=0.0325,
        track_width_m=0.3408575433,
        ticks_per_revolution=1974.1666666667,
        max_plausible_wheel_speed_mps=1.5,
    )
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # 600 ticks in 50ms = 12000 ticks/sec (under 14491 single-wheel threshold), ungated translation = 0.0621m
    # Sanity cap = 1.5 m/s * 0.05s * 1.2 = 0.090m
    success, msg = k.update([1600, 1600, 1600, 1600], timestamp_sec=100.05, sequence=2)
    assert success is True
    assert k.last_gated_d_center_m <= 0.090


def test_scenario_i_gyro_yaw_unaffected_by_translation_gate():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    external_yaw_delta = 0.05
    k.update([1308, 1005, 1308, 1005], timestamp_sec=100.05, sequence=2, external_d_yaw=external_yaw_delta)
    
    # Yaw integration remains equal to external_d_yaw
    assert abs(k.yaw - external_yaw_delta) < 1e-4


def test_failed_floor_sample_regression():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Reproduction of actual failed floor sample: left_diff = 128 ticks (13.2mm), right_diff = 8 ticks (0.8mm)
    # IMU yaw delta = 0.001 rad (negligible body rotation)
    success, msg = k.update([1128, 1008, 1128, 1008], timestamp_sec=100.05, sequence=2, external_d_yaw=0.001)
    
    assert success is True
    assert k.slip_gate_active is True
    assert k.slip_event_count == 1
    # Gated translation should equal smaller right wheel movement (~0.0008m) instead of ungated average (~0.0070m)
    assert abs(k.last_gated_d_center_m - 0.000828) < 1e-4
    assert k.last_ungated_d_center_m > 0.0069


def test_boundary_disparity_7mm():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # 7mm disparity (7.2mm vs 0.8mm -> 70 ticks vs 8 ticks = 6.4mm disparity < 8.0mm threshold)
    success, msg = k.update([1070, 1008, 1070, 1008], timestamp_sec=100.05, sequence=2, external_d_yaw=0.001)
    
    assert success is True
    assert k.slip_gate_active is False


def test_boundary_disparity_9mm():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # 9.3mm disparity (98 ticks vs 8 ticks = 9.3mm disparity > 8.0mm threshold), IMU yaw near zero
    success, msg = k.update([1098, 1008, 1098, 1008], timestamp_sec=100.05, sequence=2, external_d_yaw=0.001)
    
    assert success is True
    assert k.slip_gate_active is True


def test_legitimate_skid_steer_turn_with_large_disparity():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Legitimate turn: left_diff = 120 ticks (12.4mm), right_diff = 30 ticks (3.1mm) -> Disparity = 9.3mm > 8.0mm
    # Kinematic wheel yaw = (0.0031 - 0.0124) / 0.340857 = -0.0273 rad
    # IMU gyro measures matching physical rotation (external_d_yaw = -0.0270 rad)
    success, msg = k.update([1120, 1030, 1120, 1030], timestamp_sec=100.05, sequence=2, external_d_yaw=-0.0270)
    
    assert success is True
    # PROOF: Even though disparity (9.3mm) > 8mm, IMU agrees with wheel rotation -> Gate remains INACTIVE
    assert k.slip_gate_active is False


def test_fast_arc_turn_compatibility():
    k = create_kinematics()
    k.update([1000, 1000, 1000, 1000], timestamp_sec=100.0, sequence=1)
    
    # Fast arc turn: left_diff = 180 ticks (18.6mm), right_diff = 60 ticks (6.2mm) -> Disparity = 12.4mm > 8.0mm
    # Kinematic wheel yaw = (0.0062 - 0.0186) / 0.340857 = -0.0364 rad
    # IMU gyro measures matching physical rotation (external_d_yaw = -0.0360 rad)
    success, msg = k.update([1180, 1060, 1180, 1060], timestamp_sec=100.05, sequence=2, external_d_yaw=-0.0360)
    
    assert success is True
    assert k.slip_gate_active is False
