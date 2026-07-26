#!/usr/bin/env python3
# ==============================================================================
# record_odom_calibration.py - Read-Only Odometry Calibration Helper (Phase 3)
# ==============================================================================
#
# Listens to /odom topic and records starting pose, ending pose, reported
# distance, reported yaw change, and timestamps for physical calibration tests.
#
# Requirements met:
#   - Read-only: does not send any motor commands or armed states.
#   - Records starting pose, ending pose, distance, yaw, timestamp.
# ==============================================================================

import argparse
import datetime
import json
import math
import os
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion (x, y, z, w) to 2D yaw angle in radians [-pi, pi]."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class OdomCalibrationHelper(Node):
    def __init__(self):
        super().__init__('record_odom_calibration')
        self.latest_odom: Optional[Odometry] = None
        self.sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def get_current_pose(self, timeout_sec: float = 3.0) -> Tuple[float, float, float, float]:
        """
        Wait for a fresh /odom message and return (x, y, yaw, timestamp_sec).
        """
        start_t = time.time()
        while time.time() - start_t < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_odom is not None:
                msg = self.latest_odom
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
                stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                return p.x, p.y, yaw, stamp_sec
        raise RuntimeError("Timeout waiting for /odom message! Is rover_encoder_odometry running?")


def run_calibration_test(test_name: str, duration_sec: Optional[float] = None, log_file: Optional[str] = None):
    rclpy.init()
    helper = OdomCalibrationHelper()

    print(f"\n==================================================")
    print(f"   ODOMETRY CALIBRATION HELPER: [{test_name}]")
    print(f"==================================================")
    print("Fetching initial baseline pose from /odom...")

    try:
        x0, y0, yaw0, t0 = helper.get_current_pose()
    except Exception as e:
        print(f"ERROR: {e}")
        helper.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    iso_t0 = datetime.datetime.fromtimestamp(t0, datetime.timezone.utc).isoformat()
    print(f"\n[START POSE RECORDED]")
    print(f"  Timestamp : {iso_t0} ({t0:.3f} s)")
    print(f"  X         : {x0:+.4f} m")
    print(f"  Y         : {y0:+.4f} m")
    print(f"  Yaw       : {yaw0:+.4f} rad ({math.degrees(yaw0):+.2f} deg)")

    if duration_sec is not None and duration_sec > 0:
        print(f"\nWaiting {duration_sec:.1f} seconds for test completion...")
        time.sleep(duration_sec)
    else:
        try:
            input("\n>>> Perform test (manually move/turn rover), then press ENTER to record ending pose...")
        except EOFError:
            print("\nNon-interactive shell detected. Waiting 10 seconds default...")
            time.sleep(10)

    print("\nFetching final pose from /odom...")
    helper.latest_odom = None
    try:
        x1, y1, yaw1, t1 = helper.get_current_pose()
    except Exception as e:
        print(f"ERROR: {e}")
        helper.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    iso_t1 = datetime.datetime.fromtimestamp(t1, datetime.timezone.utc).isoformat()

    dx = x1 - x0
    dy = y1 - y0
    distance = math.hypot(dx, dy)
    dyaw = normalize_angle(yaw1 - yaw0)
    dyaw_deg = math.degrees(dyaw)
    dt = t1 - t0

    print(f"\n[END POSE RECORDED]")
    print(f"  Timestamp : {iso_t1} ({t1:.3f} s)")
    print(f"  X         : {x1:+.4f} m")
    print(f"  Y         : {y1:+.4f} m")
    print(f"  Yaw       : {yaw1:+.4f} rad ({math.degrees(yaw1):+.2f} deg)")

    print(f"\n[CALIBRATION SUMMARY]")
    print(f"  Test Name       : {test_name}")
    print(f"  Elapsed Time    : {dt:.2f} s")
    print(f"  Delta X         : {dx:+.4f} m")
    print(f"  Delta Y         : {dy:+.4f} m")
    print(f"  Reported Dist   : {distance:.4f} m")
    print(f"  Reported Yaw d  : {dyaw:+.4f} rad ({dyaw_deg:+.2f} deg)")

    record = {
        "test_name": test_name,
        "start_timestamp_iso": iso_t0,
        "end_timestamp_iso": iso_t1,
        "start_pose": {"x": x0, "y": y0, "yaw_rad": yaw0, "yaw_deg": math.degrees(yaw0)},
        "end_pose": {"x": x1, "y": y1, "yaw_rad": yaw1, "yaw_deg": math.degrees(yaw1)},
        "delta": {
            "dx_m": dx,
            "dy_m": dy,
            "reported_distance_m": distance,
            "reported_yaw_rad": dyaw,
            "reported_yaw_deg": dyaw_deg,
            "elapsed_sec": dt,
        }
    }

    if log_file:
        os.makedirs(os.path.dirname(os.path.abspath(log_file)), exist_ok=True)
        logs = []
        if os.path.exists(log_file):
            try:
                with open(log_file, "r") as f:
                    logs = json.load(f)
            except Exception:
                logs = []
        logs.append(record)
        with open(log_file, "w") as f:
            json.dump(logs, f, indent=2)
        print(f"\n[LOG SAVED] Append to '{log_file}'")

    print(f"==================================================\n")

    helper.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Read-only ROS 2 Odometry Calibration Helper")
    parser.add_argument("--test", type=str, default="1m_forward", help="Test name (e.g. 1m_forward, reverse_to_start, 90deg_turn)")
    parser.add_argument("--duration", type=float, default=None, help="Optional duration in seconds (if non-interactive)")
    parser.add_argument("--log-file", type=str, default="/ros2_ws/logs/odom_calibration.json", help="Path to save JSON log results")

    args = parser.parse_args()
    run_calibration_test(test_name=args.test, duration_sec=args.duration, log_file=args.log_file)


if __name__ == "__main__":
    main()
