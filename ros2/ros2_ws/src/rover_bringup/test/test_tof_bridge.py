"""
test_tof_bridge.py - Unit tests for rover_tof_bridge (VL53L7CX 8x8 front ToF ROS 2 bridge)

Tests run without a live ROS 2 daemon or hardware.
"""

import importlib.util
import math
import os
import struct
import sys
from unittest.mock import MagicMock, patch

# --- Minimal rclpy & sensor_msgs stubs ---
rclpy_stub = MagicMock()
rclpy_stub.ok.return_value = True
sys.modules.setdefault("rclpy", rclpy_stub)
sys.modules.setdefault("rclpy.node", MagicMock())


class _PointField:
    FLOAT32 = 7

    def __init__(self, name="", offset=0, datatype=7, count=1):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = count


class _PointCloud2:
    def __init__(self):
        self.header = MagicMock()
        self.height = 0
        self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = 0
        self.row_step = 0
        self.is_dense = True
        self.data = b""


class _Range:
    INFRARED = 1

    def __init__(self):
        self.header = MagicMock()
        self.radiation_type = 0
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0


sensor_msgs_stub = MagicMock()
sensor_msgs_stub.msg.PointCloud2 = _PointCloud2
sensor_msgs_stub.msg.PointField = _PointField
sensor_msgs_stub.msg.Range = _Range

with patch.dict(
    "sys.modules",
    {
        "rclpy": rclpy_stub,
        "rclpy.node": MagicMock(),
        "sensor_msgs": sensor_msgs_stub,
        "sensor_msgs.msg": sensor_msgs_stub.msg,
    },
):
    _pkg_dir = os.path.join(os.path.dirname(__file__), "..", "rover_bringup")
    _bridge_path = os.path.join(_pkg_dir, "rover_tof_bridge.py")
    spec = importlib.util.spec_from_file_location("rover_tof_bridge", _bridge_path)
    _tof_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_tof_mod)

display_to_source_index = _tof_mod.display_to_source_index
grid_to_points = _tof_mod.grid_to_points
build_pointcloud2 = _tof_mod.build_pointcloud2


def test_display_to_source_index_identity():
    assert display_to_source_index(0, "none") == 0
    assert display_to_source_index(7, "none") == 7
    assert display_to_source_index(63, "none") == 63


def test_display_to_source_index_rotations():
    # 90_ccw maps z_disp=0 (r=0, c=0) -> (7-0)*8 + (7-0) = 63
    assert display_to_source_index(0, "90_ccw") == 63
    # 90_cw maps z_disp=0 (r=0, c=0) -> (7-0)*8 + 0 = 56
    assert display_to_source_index(0, "90_cw") == 56
    # 180 maps z_disp=0 -> 63
    assert display_to_source_index(0, "180") == 63


def test_grid_to_points_filters_invalid_and_projects_fov():
    grid = [-1] * 64
    # Top-left display zone (z_disp=0: row=0, col=0) -> +y (left), +z (up)
    grid[0] = 500  # 0.50 m
    # Bottom-right display zone (z_disp=63: row=7, col=7) -> -y (right), -z (down)
    grid[63] = 1000  # 1.00 m

    pts = grid_to_points(grid, rot="none", range_min_m=0.02, range_max_m=3.50)
    assert len(pts) == 2

    x0, y0, z0 = pts[0]
    assert x0 > 0.40
    assert y0 > 0.0  # col 0 is left (+Y)
    assert z0 > 0.0  # row 0 is top (+Z)

    x1, y1, z1 = pts[1]
    assert x1 > 0.80
    assert y1 < 0.0  # col 7 is right (-Y)
    assert z1 < 0.0  # row 7 is bottom (-Z)


def test_grid_to_points_respects_range_bounds():
    grid = [-1] * 64
    grid[0] = 10     # 0.01 m (< range_min_m 0.02)
    grid[1] = 4000   # 4.00 m (> range_max_m 3.50)
    grid[2] = 250    # 0.25 m (valid)

    pts = grid_to_points(grid, rot="none", range_min_m=0.02, range_max_m=3.50)
    assert len(pts) == 1
    r0 = math.sqrt(pts[0][0] ** 2 + pts[0][1] ** 2 + pts[0][2] ** 2)
    assert math.isclose(r0, 0.25, rel_tol=1e-6)


def test_grid_to_points_preserves_close_stop_and_crawl_zone_readings_by_default():
    grid = [-1] * 64
    grid[0] = 8    # 8 mm (inside <=30 mm stop zone)
    grid[1] = 45   # 45 mm (inside <=75 mm crawl zone)
    pts = grid_to_points(grid, rot="none")
    assert len(pts) == 2
    r0 = math.sqrt(pts[0][0] ** 2 + pts[0][1] ** 2 + pts[0][2] ** 2)
    r1 = math.sqrt(pts[1][0] ** 2 + pts[1][1] ** 2 + pts[1][2] ** 2)
    assert math.isclose(r0, 0.008, rel_tol=1e-6)
    assert math.isclose(r1, 0.045, rel_tol=1e-6)


def test_build_pointcloud2_packs_xyz_float32():
    pts = [(0.25, 0.05, -0.02), (0.80, -0.10, 0.04)]
    msg = build_pointcloud2(pts, "tof_front_left_link", 123, 456000)

    assert msg.header.frame_id == "tof_front_left_link"
    assert msg.header.stamp.sec == 123
    assert msg.header.stamp.nanosec == 456000
    assert msg.height == 1
    assert msg.width == 2
    assert msg.point_step == 12
    assert msg.row_step == 24
    assert len(msg.data) == 24

    u0 = struct.unpack_from("<fff", msg.data, 0)
    u1 = struct.unpack_from("<fff", msg.data, 12)
    assert math.isclose(u0[0], 0.25, rel_tol=1e-6)
    assert math.isclose(u0[1], 0.05, rel_tol=1e-6)
    assert math.isclose(u1[0], 0.80, rel_tol=1e-6)
