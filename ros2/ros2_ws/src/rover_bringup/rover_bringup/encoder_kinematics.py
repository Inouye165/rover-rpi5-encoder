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
        track_width_m: float = 0.170,
        ticks_per_revolution: float = 937.2,
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

        # Diagnostics flags
        self.last_sequence: Optional[int] = None
        self.disagreement_warning: bool = False
        self.disagreement_details: str = ""

    def reset_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        """Reset pose integration back to (x, y, yaw)."""
        self.x = float(x)
        self.y = float(y)
        self.yaw = normalize_angle(yaw)
        self.v_x = 0.0
        self.w_z = 0.0
        self.accum_left_dist_m = 0.0
        self.accum_right_dist_m = 0.0

    def update(
        self,
        ticks: List[int],
        timestamp_sec: float,
        sequence: Optional[int] = None,
    ) -> Tuple[bool, str]:
        """
        Process a new telemetry frame [m1, m2, m3, m4] at timestamp_sec.

        Returns (success: bool, message: str).
        """
        if len(ticks) < 4:
            return False, "Invalid telemetry length"

        current_ticks = [int(ticks[0]), int(ticks[1]), int(ticks[2]), int(ticks[3])]

        # Initial sample baseline
        if self.last_ticks is None or self.last_timestamp_sec is None:
            self.last_ticks = current_ticks
            self.last_timestamp_sec = float(timestamp_sec)
            self.last_sequence = sequence
            return True, "Baseline initialized"

        dt = float(timestamp_sec) - self.last_timestamp_sec

        # Duplicate sample or zero/negative time delta check
        if dt <= 0.0:
            return False, f"Non-positive time delta (dt={dt:.4f}s)"

        # Stale timestamp check
        if dt > self.stale_timeout_sec:
            self.last_ticks = current_ticks
            self.last_timestamp_sec = float(timestamp_sec)
            self.last_sequence = sequence
            self.v_x = 0.0
            self.w_z = 0.0
            return False, f"Stale telemetry gap ({dt:.2f}s > {self.stale_timeout_sec}s)"

        # Duplicate sequence check
        if sequence is not None and self.last_sequence is not None and sequence == self.last_sequence:
            return False, f"Duplicate sequence payload ({sequence})"

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

        d_center_m = (d_left_m + d_right_m) / 2.0
        d_yaw = (d_right_m - d_left_m) / self.track_width_m

        # Integrate pose using midpoint arc approximation
        yaw_mid = self.yaw + d_yaw / 2.0
        self.x += d_center_m * math.cos(yaw_mid)
        self.y += d_center_m * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + d_yaw)

        # Compute body velocities
        self.v_x = d_center_m / dt
        self.w_z = d_yaw / dt

        # Update history
        self.last_ticks = current_ticks
        self.last_timestamp_sec = float(timestamp_sec)
        self.last_sequence = sequence

        return True, "Success"
