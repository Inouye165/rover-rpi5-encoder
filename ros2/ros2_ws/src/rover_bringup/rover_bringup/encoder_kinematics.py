# ==============================================================================
# encoder_kinematics.py — Rover Encoder Kinematics & Odometry Math
# ==============================================================================
"""
Pure Python module for 4-wheel skid-steer encoder odometry integration.

Converts raw cumulative encoder counts (M1..M4) into 2D differential-drive pose
(x, y, yaw) and velocities (v_x, w_z) with robust counter rollover handling,
reset detection, impossible jump rejection, and front/rear wheel disagreement
diagnostics.

Encoder to wheel mapping:
  M1 (encoders.m1 / index 0): Left Front (LF)
  M2 (encoders.m2 / index 1): Right Front (RF)
  M3 (encoders.m3 / index 2): Left Rear (LR)
  M4 (encoders.m4 / index 3): Right Rear (RR)

Sign convention:
  Forward movement produces POSITIVE count increases on all 4 encoders
  (M2 and M4 right-side signs are negated in ESP32 firmware).
"""

import math
from typing import Dict, List, Optional, Tuple


def normalize_angle(angle_rad: float) -> float:
    """Normalize angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def signed_min_magnitude(a: float, b: float) -> float:
    """
    Return signed value with smaller absolute magnitude.
    If signs differ (turning in place), return 0.0.
    """
    if a * b < 0.0:
        return 0.0
    if abs(a) < abs(b):
        return a
    else:
        return b


def compute_ticks_delta(current: int, previous: Optional[int], reset_threshold: int = 100000) -> int:
    """
    Compute delta between two 32-bit signed integer encoder counts.
    Handles 32-bit signed rollover (-2^31 to 2^31 - 1) and detects resets.
    Returns 0 if previous is None or if a counter reset is detected.
    """
    if previous is None:
        return 0

    delta = current - previous

    # Handle 32-bit signed integer rollover
    if delta > 2147483647:
        delta -= 4294967296
    elif delta < -2147483648:
        delta += 4294967296

    # Counter reset detection (e.g. ESP32 reboot / PCNT clear)
    if abs(delta) > reset_threshold:
        return 0

    return delta


class EncoderKinematics:
    """
    Stateful odometry integrator for 4-wheel skid-steer rover.
    """

    def __init__(
        self,
        wheel_radius_m: float = 0.0325,
        track_width_m: float = 0.3408575433,
        physical_track_width_m: float = 0.197,
        ticks_per_revolution: float = 1974.1666666667,
        m1_sign: float = 1.0,
        m2_sign: float = 1.0,
        m3_sign: float = 1.0,
        m4_sign: float = 1.0,
        reset_threshold_ticks: int = 100000,
        disagreement_threshold_ticks: int = 100,
        max_plausible_wheel_speed_mps: float = 2.5,
        stale_timeout_sec: float = 2.0,
    ):
        self.wheel_radius_m = float(wheel_radius_m)
        self.track_width_m = float(track_width_m)
        self.physical_track_width_m = float(physical_track_width_m)
        self.ticks_per_revolution = float(ticks_per_revolution)
        self.m_signs = [float(m1_sign), float(m2_sign), float(m3_sign), float(m4_sign)]

        self.reset_threshold_ticks = int(reset_threshold_ticks)
        self.disagreement_threshold_ticks = int(disagreement_threshold_ticks)
        self.max_plausible_wheel_speed_mps = float(max_plausible_wheel_speed_mps)
        self.stale_timeout_sec = float(stale_timeout_sec)

        # Pose state (odom frame)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Velocity state
        self.v_x = 0.0
        self.w_z = 0.0

        # Accumulated distance
        self.accum_left_dist_m = 0.0
        self.accum_right_dist_m = 0.0

        # History tracking
        self.last_ticks: Optional[List[int]] = None
        self.last_timestamp_sec: Optional[float] = None
        self.last_fresh_timestamp_sec: Optional[float] = None

        # Diagnostics & slip telemetry flags
        self.last_sequence: Optional[int] = None
        self.disagreement_warning: bool = False
        self.disagreement_details: str = ""

        self.slip_gate_active: bool = False
        self.slip_event_count: int = 0
        self.last_raw_d_left_m: float = 0.0
        self.last_raw_d_right_m: float = 0.0
        self.last_wheel_disparity_m: float = 0.0
        self.last_d_yaw_wheel_rad: float = 0.0
        self.last_external_d_yaw_rad: Optional[float] = None
        self.last_yaw_disagreement_rad: float = 0.0
        self.last_imu_yaw_valid: bool = False
        self.last_ratio_fallback_used: bool = False
        self.last_ungated_d_center_m: float = 0.0
        self.last_gated_d_center_m: float = 0.0
        self.last_slip_reason: str = ""

    def reset_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Reset pose integration back to (x, y, yaw)."""
        self.x = float(x)
        self.y = float(y)
        self.yaw = normalize_angle(yaw)
        self.v_x = 0.0
        self.w_z = 0.0
        self.accum_left_dist_m = 0.0
        self.accum_right_dist_m = 0.0
        self.last_ticks = None
        self.last_timestamp_sec = None
        self.last_fresh_timestamp_sec = None
        self.slip_gate_active = False
        self.slip_event_count = 0
        self.last_raw_d_left_m = 0.0
        self.last_raw_d_right_m = 0.0
        self.last_wheel_disparity_m = 0.0
        self.last_d_yaw_wheel_rad = 0.0
        self.last_external_d_yaw_rad = None
        self.last_yaw_disagreement_rad = 0.0
        self.last_imu_yaw_valid = False
        self.last_ratio_fallback_used = False
        self.last_ungated_d_center_m = 0.0
        self.last_gated_d_center_m = 0.0
        self.last_slip_reason = ""

    def update(
        self,
        ticks: List[int],
        timestamp_sec: float,
        sequence: Optional[int] = None,
        external_d_yaw: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Process a new telemetry frame [m1, m2, m3, m4] at timestamp_sec.

        Returns (success: bool, message: str).
        """
        if len(ticks) < 4:
            return False, "Invalid telemetry length"

        current_ticks = [int(ticks[0]), int(ticks[1]), int(ticks[2]), int(ticks[3])]

        # Initial sample baseline
        if self.last_ticks is None or self.last_timestamp_sec is None or self.last_fresh_timestamp_sec is None:
            self.last_ticks = current_ticks
            self.last_timestamp_sec = float(timestamp_sec)
            self.last_fresh_timestamp_sec = float(timestamp_sec)
            self.last_sequence = sequence
            return True, "Baseline initialized"

        dt_total = float(timestamp_sec) - self.last_timestamp_sec

        # Duplicate sample or zero/negative time delta check
        if dt_total <= 0.0:
            return False, f"Non-positive time delta (dt={dt_total:.4f}s)"

        # Stale timestamp check
        if dt_total > self.stale_timeout_sec:
            self.last_ticks = current_ticks
            self.last_timestamp_sec = float(timestamp_sec)
            self.last_fresh_timestamp_sec = float(timestamp_sec)
            self.last_sequence = sequence
            self.v_x = 0.0
            self.w_z = 0.0
            return False, f"Stale telemetry gap ({dt_total:.2f}s > {self.stale_timeout_sec}s)"

        # Unchanged cached sequence check
        if sequence is not None and self.last_sequence is not None and sequence == self.last_sequence:
            self.last_timestamp_sec = float(timestamp_sec)
            self.disagreement_warning = False
            self.disagreement_details = ""
            # Retain current pose and velocity (no pose motion, no double integration, no 20Hz flicker)
            return True, "NO_NEW_SAMPLE"

        # Fresh sample: compute dt relative to the last accepted FRESH sample
        dt = float(timestamp_sec) - self.last_fresh_timestamp_sec
        if dt <= 0.0:
            dt = dt_total  # fallback safeguard if timestamps coincide

        # Compute per-wheel tick deltas with rollover & sign normalization
        dm1 = compute_ticks_delta(current_ticks[0], self.last_ticks[0], self.reset_threshold_ticks) * self.m_signs[0]
        dm2 = compute_ticks_delta(current_ticks[1], self.last_ticks[1], self.reset_threshold_ticks) * self.m_signs[1]
        dm3 = compute_ticks_delta(current_ticks[2], self.last_ticks[2], self.reset_threshold_ticks) * self.m_signs[2]
        dm4 = compute_ticks_delta(current_ticks[3], self.last_ticks[3], self.reset_threshold_ticks) * self.m_signs[3]

        # Sanity check: max wheel speed limit
        m_per_tick = (2.0 * math.pi * self.wheel_radius_m) / self.ticks_per_revolution
        max_ticks_per_sec = self.max_plausible_wheel_speed_mps / m_per_tick

        for idx, (name, dm) in enumerate([("M1", dm1), ("M2", dm2), ("M3", dm3), ("M4", dm4)]):
            speed_ticks_per_sec = abs(dm) / dt
            if speed_ticks_per_sec > max_ticks_per_sec:
                # Reject sample due to impossible jump
                self.last_ticks = current_ticks
                self.last_timestamp_sec = float(timestamp_sec)
                self.last_fresh_timestamp_sec = float(timestamp_sec)
                self.last_sequence = sequence
                return False, f"Impossible count jump on {name}: {abs(dm)} ticks in {dt:.3f}s"

        # Front / rear encoder disagreement check
        left_diff = abs(dm1 - dm3)
        right_diff = abs(dm2 - dm4)

        self.disagreement_warning = (
            left_diff > self.disagreement_threshold_ticks or right_diff > self.disagreement_threshold_ticks
        )
        if self.disagreement_warning:
            self.disagreement_details = f"Wheel disagreement detected: left_diff={int(left_diff)}, right_diff={int(right_diff)}"
        else:
            self.disagreement_details = ""

        # Average front and rear wheel deltas for left and right sides
        d_left_ticks = (dm1 + dm3) / 2.0
        d_right_ticks = (dm2 + dm4) / 2.0

        d_left_m = d_left_ticks * m_per_tick
        d_right_m = d_right_ticks * m_per_tick

        self.accum_left_dist_m += d_left_m
        self.accum_right_dist_m += d_right_m

        # Raw ungated center translation and wheel-based yaw
        ungated_d_center_m = (d_left_m + d_right_m) / 2.0
        d_yaw_wheel = (d_right_m - d_left_m) / self.track_width_m

        # Effective yaw delta (prefer external gyro IMU yaw if provided)
        effective_d_yaw = float(external_d_yaw) if external_d_yaw is not None else d_yaw_wheel

        # Slip detection rule:
        # 1. Severe wheel speed disparity: >0.008m per tick (~0.16 m/s speed diff)
        # 2. Kinematics vs IMU rotation disagreement: |d_yaw_wheel - external_d_yaw| > 0.025 rad (~1.4 deg)
        wheel_disparity_m = abs(d_left_m - d_right_m)
        yaw_disagreement_rad = abs(d_yaw_wheel - effective_d_yaw)

        imu_valid = external_d_yaw is not None
        is_severe_disparity = wheel_disparity_m > 0.008
        is_yaw_disagree = yaw_disagreement_rad > 0.025 if imu_valid else (
            (max(abs(d_left_m), abs(d_right_m)) / (min(abs(d_left_m), abs(d_right_m)) + 1e-6)) > 3.0
        )

        prev_slip_active = self.slip_gate_active
        self.slip_gate_active = is_severe_disparity and is_yaw_disagree

        if self.slip_gate_active:
            if not prev_slip_active:
                self.slip_event_count += 1
            mode_str = "IMU_YAW_DISAGREE" if imu_valid else "RATIO_FALLBACK"
            self.last_slip_reason = (
                f"Wheel slip detected [{mode_str}]: left={d_left_m*1000:.1f}mm, right={d_right_m*1000:.1f}mm, "
                f"wheel_yaw={math.degrees(d_yaw_wheel):.1f}deg, imu_yaw={math.degrees(effective_d_yaw):.1f}deg, "
                f"yaw_err={math.degrees(yaw_disagreement_rad):.1f}deg"
            )
            d_center_m = signed_min_magnitude(d_left_m, d_right_m)
        else:
            self.last_slip_reason = ""
            d_center_m = ungated_d_center_m

        # Per-update sanity cap: 0.15m max translation step per 50ms sample (equivalent to 3.0 m/s max speed)
        max_step_m = self.max_plausible_wheel_speed_mps * dt * 1.2
        if abs(d_center_m) > max_step_m:
            d_center_m = math.copysign(max_step_m, d_center_m)

        # Update telemetry attributes
        self.last_raw_d_left_m = d_left_m
        self.last_raw_d_right_m = d_right_m
        self.last_wheel_disparity_m = wheel_disparity_m
        self.last_d_yaw_wheel_rad = d_yaw_wheel
        self.last_external_d_yaw_rad = float(external_d_yaw) if imu_valid else None
        self.last_yaw_disagreement_rad = yaw_disagreement_rad
        self.last_imu_yaw_valid = imu_valid
        self.last_ratio_fallback_used = not imu_valid
        self.last_ungated_d_center_m = ungated_d_center_m
        self.last_gated_d_center_m = d_center_m

        # Integrate pose using midpoint arc approximation
        yaw_mid = self.yaw + effective_d_yaw / 2.0
        self.x += d_center_m * math.cos(yaw_mid)
        self.y += d_center_m * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + effective_d_yaw)

        # Compute body velocities
        self.v_x = d_center_m / dt
        self.w_z = effective_d_yaw / dt

        # Update history
        self.last_ticks = current_ticks
        self.last_timestamp_sec = float(timestamp_sec)
        self.last_fresh_timestamp_sec = float(timestamp_sec)
        self.last_sequence = sequence

        return True, "Success"
