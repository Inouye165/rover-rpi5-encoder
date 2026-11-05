#!/usr/bin/env python3
# ==============================================================================
# rover_tof_bridge.py - ROS 2 Time-of-Flight Bridge Node
# ==============================================================================
#
# Polls the Cockpit API (GET ${ROVER_SERVER_URL}/api/tof-sensors), which exposes
# the ESP32-S3 TCA9548A bench state, and republishes the two front-corner
# VL53L7CX 8x8 sensors as:
#   /tof/front_left/points   sensor_msgs/PointCloud2  (frame tof_front_left_link)
#   /tof/front_left/range    sensor_msgs/Range        (nearest valid zone)
#   /tof/front_right/points  sensor_msgs/PointCloud2  (frame tof_front_right_link)
#   /tof/front_right/range   sensor_msgs/Range
#
# The VL53L4CD on mux channel 2 is intentionally NOT published yet.
# This node never opens a serial device and never commands motion.
# ==============================================================================

import math
import os
import struct

import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Range

ZONE_PITCH_DEG = 45.0 / 8.0  # VL53L7CX 45 deg x 45 deg FoV over an 8x8 grid
FOV_RAD = math.radians(45.0)

SLOTS = ("front_left_l7cx", "front_right_l7cx")


def display_to_source_index(z_disp, rot):
    """Map a display zone (row-major, row 0 = top, col 0 = left as seen from the
    rover) to the sensor's raw zone index. Mirrors public/sensors.html."""
    r, c = divmod(z_disp, 8)
    if rot == "90_ccw":
        return (7 - c) * 8 + (7 - r)
    if rot == "90_ccw_noflip":
        return c * 8 + (7 - r)
    if rot == "90_cw":
        return (7 - c) * 8 + r
    if rot == "180":
        return 63 - z_disp
    return z_disp


def grid_to_points(grid_mm, rot, range_min_m=0.02, range_max_m=4.0):
    """Convert a raw 64-zone grid (mm, -1 = no target) into (x, y, z) points in
    the sensor frame (x forward, y left, z up). Returns a list of tuples."""
    pts = []
    if not isinstance(grid_mm, (list, tuple)) or len(grid_mm) < 64:
        return pts
    for z_disp in range(64):
        d_mm = grid_mm[display_to_source_index(z_disp, rot)]
        if d_mm is None or d_mm < 0:
            continue
        d = d_mm / 1000.0
        if not (range_min_m <= d <= range_max_m):
            continue
        r, c = divmod(z_disp, 8)
        yaw = math.radians((3.5 - c) * ZONE_PITCH_DEG)    # left positive
        pitch = math.radians((3.5 - r) * ZONE_PITCH_DEG)  # up positive
        pts.append((
            d * math.cos(pitch) * math.cos(yaw),
            d * math.cos(pitch) * math.sin(yaw),
            d * math.sin(pitch),
        ))
    return pts


def build_pointcloud2(points, frame_id, sec, nanosec):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


class RoverTofBridgeNode(Node):
    def __init__(self):
        super().__init__("rover_tof_bridge")
        default_url = os.environ.get("ROVER_SERVER_URL", "http://127.0.0.1:3000")
        self.declare_parameter("server_url", default_url)
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("range_min_m", 0.02)
        self.declare_parameter("range_max_m", 4.0)
        self.declare_parameter("max_age_ms", 1000)
        self.url = self.get_parameter("server_url").value.rstrip("/") + "/api/tof-sensors"
        self.range_min = float(self.get_parameter("range_min_m").value)
        self.range_max = float(self.get_parameter("range_max_m").value)
        self.max_age_ms = int(self.get_parameter("max_age_ms").value)

        self.pubs = {}
        for slot, name in (("front_left_l7cx", "front_left"), ("front_right_l7cx", "front_right")):
            self.pubs[slot] = (
                name,
                self.create_publisher(PointCloud2, f"/tof/{name}/points", 5),
                self.create_publisher(Range, f"/tof/{name}/range", 5),
            )
        self.create_timer(1.0 / float(self.get_parameter("poll_rate_hz").value), self._poll)
        self._warned = False

    def _poll(self):
        try:
            payload = requests.get(self.url, timeout=1.0).json()
        except Exception as exc:  # noqa: BLE001
            if not self._warned:
                self.get_logger().warn(f"ToF API unreachable at {self.url}: {exc}")
                self._warned = True
            return
        self._warned = False
        data = payload.get("data") or {}
        if payload.get("stale") or not data:
            return
        mapping = payload.get("mapping") or {}
        channels = {c.get("ch"): c for c in data.get("channels", [])}
        now = self.get_clock().now().to_msg()

        for slot, (name, pc_pub, range_pub) in self.pubs.items():
            ch_idx = mapping.get(slot)
            if not isinstance(ch_idx, int):
                continue  # only explicitly mapped channels; never guess
            ch = channels.get(ch_idx)
            if not ch or not ch.get("present") or not ch.get("init"):
                continue
            if ch.get("age_ms", 10**9) > self.max_age_ms:
                continue
            rot = str(mapping.get(f"{slot}_rot", "90_ccw"))
            frame = f"tof_{name}_link"
            pts = grid_to_points(ch.get("grid_mm"), rot, self.range_min, self.range_max)
            pc_pub.publish(build_pointcloud2(pts, frame, now.sec, now.nanosec))

            rng = Range()
            rng.header.frame_id = frame
            rng.header.stamp = now
            rng.radiation_type = Range.INFRARED
            rng.field_of_view = FOV_RAD
            rng.min_range = float(self.range_min)
            rng.max_range = float(self.range_max)
            min_mm = ch.get("min_mm", -1)
            rng.range = (min_mm / 1000.0) if min_mm is not None and min_mm >= 0 else float("inf")
            range_pub.publish(rng)


def main(args=None):
    rclpy.init(args=args)
    node = RoverTofBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

