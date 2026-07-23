#!/usr/bin/env python3
# ==============================================================================
# rover_lidar_bridge.py - ROS 2 LiDAR Bridge Node (Phase 2)
# ==============================================================================
#
# Polls the host LiDAR sidecar HTTP API (GET /scan) and republishes scan data
# as sensor_msgs/msg/LaserScan on /scan.
#
# The physical LiDAR device (/dev/rover-lidar) is owned exclusively by the host
# rover-lidar.service systemd service.  This node does NOT open any serial
# device and requires no /dev mounts in Docker.
#
# Sidecar API contract (from rplidar_sidecar.py):
#   GET /scan  →  200 OK
#   {
#     "timestamp":  "2026-07-20T15:30:00Z",  // ISO-8601 UTC; null until first scan
#     "sequence":   42,                       // monotonic int; primary staleness key
#     "scanHz":     7.1,                      // float, rotations per second
#     "pointCount": 360,                      // int, number of valid points served
#     "points": [
#       { "angleDeg": 0.0, "distanceMm": 1050, "quality": 31 }
#       // sorted ascending by angleDeg ∈ [0, 360), CCW-positive
#       // distanceMm already validated: > 0, finite (sidecar strips bad readings)
#     ]
#   }
#   503  { "error": "No scan data available" }  when not yet ready
#
# REP-117: missing angular bins are represented as float('inf'), never fabricated.
# ==============================================================================

import math
import os
import time

import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# Number of angular bins in the output LaserScan (1° resolution, full circle).
_NUM_BINS = 360
_TWO_PI = 2.0 * math.pi


def _iso_to_stamp(ts_str, fallback_sec):
    """
    Parse an ISO-8601 UTC string like '2026-07-20T15:30:00Z' into
    (seconds, nanoseconds) integers.  Returns fallback on any parse error.
    """
    if not ts_str:
        return fallback_sec, 0
    try:
        import calendar
        t = time.strptime(ts_str, "%Y-%m-%dT%H:%M:%SZ")
        epoch_sec = int(calendar.timegm(t))
        return epoch_sec, 0
    except Exception:
        return fallback_sec, 0


def build_laserscan(
    points,
    stamp_sec,
    stamp_nanosec,
    frame_id,
    scan_hz,
    range_min_m,
    range_max_m,
    angle_offset_deg,
    reverse_scan,
):
    """
    Convert a list of sidecar point dicts into a sensor_msgs/msg/LaserScan.

    Parameters
    ----------
    points : list[dict]
        Each dict has keys: angleDeg (float), distanceMm (int), quality (int).
        angleDeg ∈ [0, 360), sorted ascending, CCW-positive, 0 = forward.
    stamp_sec / stamp_nanosec : int
        Header timestamp.
    frame_id : str
        TF frame for the LaserScan header.
    scan_hz : float
        Scan rotation rate reported by the sidecar (for scan_time / time_increment).
    range_min_m / range_max_m : float
        Inclusive range window for valid readings; outside → float('inf').
    angle_offset_deg : float
        Degrees to add to all incoming angleDeg values before binning.
        Use to correct for physical mounting orientation.
    reverse_scan : bool
        When True, publish with angle_min = 2π, angle_max = 0 (CW convention).

    Returns
    -------
    sensor_msgs.msg.LaserScan
    """
    msg = LaserScan()

    # --- Header ---
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nanosec
    msg.header.frame_id = frame_id

    # --- Angular geometry ---
    # One bin per degree (360 bins for a full revolution).
    angle_increment_rad = _TWO_PI / _NUM_BINS  # ≈ 0.01745 rad

    if not reverse_scan:
        msg.angle_min = 0.0
        msg.angle_max = _TWO_PI - angle_increment_rad
        msg.angle_increment = angle_increment_rad
    else:
        msg.angle_min = _TWO_PI - angle_increment_rad
        msg.angle_max = 0.0
        msg.angle_increment = -angle_increment_rad

    # --- Timing ---
    scan_time = 1.0 / scan_hz if scan_hz > 0.0 else 0.0
    msg.scan_time = float(scan_time)
    msg.time_increment = float(scan_time / _NUM_BINS) if _NUM_BINS > 0 else 0.0

    # --- Range bounds ---
    msg.range_min = float(range_min_m)
    msg.range_max = float(range_max_m)

    # --- Build fixed-size bins (1° each), filled with inf by default (REP-117) ---
    ranges = [float("inf")] * _NUM_BINS
    intensities = [0.0] * _NUM_BINS

    for pt in points:
        try:
            raw_deg = float(pt["angleDeg"])
            dist_mm = float(pt["distanceMm"])
            quality = float(pt.get("quality", 0))
        except (KeyError, TypeError, ValueError):
            continue  # skip malformed entries

        # Validate raw values from the sidecar
        if not math.isfinite(raw_deg) or not math.isfinite(dist_mm) or dist_mm <= 0:
            continue

        # Apply angle offset and normalise to [0, 360)
        adj_deg = (raw_deg + angle_offset_deg) % 360.0
        if adj_deg < 0.0:
            adj_deg += 360.0

        # Convert distance
        dist_m = dist_mm / 1000.0

        # Range gate
        if dist_m < range_min_m or dist_m > range_max_m:
            continue

        # Bin index: floor(angleDeg) clipped to [0, NUM_BINS-1]
        bin_idx = int(adj_deg) % _NUM_BINS

        # Keep the closest reading if multiple points fall in the same bin
        if dist_m < ranges[bin_idx] or not math.isfinite(ranges[bin_idx]):
            ranges[bin_idx] = dist_m
            intensities[bin_idx] = float(quality)

    # Reverse order if requested (CW scan direction)
    if reverse_scan:
        ranges = ranges[::-1]
        intensities = intensities[::-1]

    msg.ranges = ranges
    msg.intensities = intensities
    return msg


class RoverLidarBridgeNode(Node):
    """
    Polls the host LiDAR sidecar at ${ROVER_LIDAR_URL}/scan and publishes
    sensor_msgs/msg/LaserScan on /scan.

    ROS Parameters
    --------------
    lidar_url          : str   – Base URL for the LiDAR sidecar (default from env).
    frame_id           : str   – TF frame id for the scan header (default: laser_frame).
    poll_rate_hz       : float – HTTP polling rate (default: 10.0).
    range_min_m        : float – Minimum valid range in metres (default: 0.10).
    range_max_m        : float – Maximum valid range in metres (default: 12.0).
    angle_offset_deg   : float – Degrees added to all angleDeg values (default: 0.0).
    reverse_scan       : bool  – Publish CW instead of CCW if True (default: False).
    """

    def __init__(self):
        super().__init__("rover_lidar_bridge")

        # Declare parameters
        default_lidar_url = os.environ.get("ROVER_LIDAR_URL", "http://127.0.0.1:3002")

        self.declare_parameter("lidar_url", default_lidar_url)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("range_min_m", 0.10)
        self.declare_parameter("range_max_m", 12.0)
        self.declare_parameter("angle_offset_deg", 0.0)
        self.declare_parameter("reverse_scan", False)

        self.lidar_url = self.get_parameter("lidar_url").value
        self.frame_id = self.get_parameter("frame_id").value
        self.poll_rate_hz = self.get_parameter("poll_rate_hz").value
        self.range_min_m = self.get_parameter("range_min_m").value
        self.range_max_m = self.get_parameter("range_max_m").value
        self.angle_offset_deg = self.get_parameter("angle_offset_deg").value
        self.reverse_scan = self.get_parameter("reverse_scan").value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        # Deduplication state
        self._last_sequence = None   # last published sequence number
        self._api_error_logged = False  # throttle repeated error logs

        # Polling timer
        period = 1.0 / max(self.poll_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._poll)

        self.get_logger().info(
            f"rover_lidar_bridge initialised.\n"
            f"  Sidecar URL : {self.lidar_url}\n"
            f"  frame_id    : {self.frame_id}\n"
            f"  poll_rate   : {self.poll_rate_hz} Hz\n"
            f"  range       : [{self.range_min_m}, {self.range_max_m}] m\n"
            f"  angle_offset: {self.angle_offset_deg} °\n"
            f"  reverse_scan: {self.reverse_scan}"
        )

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _poll(self):
        """Fetch /scan from the sidecar and publish if the data is new."""
        scan_url = f"{self.lidar_url}/scan"
        try:
            resp = requests.get(scan_url, timeout=1.0)
        except Exception as exc:
            if not self._api_error_logged:
                self.get_logger().warn(
                    f"LiDAR sidecar unreachable at {scan_url}: {exc}. "
                    "Will retry silently until it recovers."
                )
                self._api_error_logged = True
            return

        # Successful contact — clear the error-logged flag for next failure
        self._api_error_logged = False

        if resp.status_code == 503:
            # Sidecar is up but scan not yet ready — silent, not an error
            return

        if resp.status_code != 200:
            self.get_logger().warn(
                f"LiDAR sidecar returned HTTP {resp.status_code} for {scan_url}"
            )
            return

        try:
            data = resp.json()
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse /scan JSON: {exc}")
            return

        # ---- Duplicate / stale detection ----
        sequence = data.get("sequence")
        if sequence is not None:
            if sequence == self._last_sequence:
                # Same sequence number → sidecar hasn't produced a new scan yet
                return
            self._last_sequence = sequence

        # ---- Extract payload ----
        points = data.get("points")
        if not isinstance(points, list):
            self.get_logger().warn("/scan response missing 'points' list; skipping.")
            return

        timestamp_str = data.get("timestamp")
        scan_hz = float(data.get("scanHz", 0.0))

        # Compute header stamp
        now_sec = int(time.time())
        stamp_sec, stamp_nanosec = _iso_to_stamp(timestamp_str, now_sec)

        # ---- Build and publish LaserScan ----
        msg = build_laserscan(
            points=points,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            frame_id=self.frame_id,
            scan_hz=scan_hz,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            angle_offset_deg=self.angle_offset_deg,
            reverse_scan=self.reverse_scan,
        )

        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverLidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
