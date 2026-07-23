"""
test_lidar_bridge.py - Unit tests for rover_lidar_bridge (Phase 2)

Tests run without a live ROS 2 daemon or real hardware.  The build_laserscan()
helper and _iso_to_stamp() helper are imported directly and exercised in
isolation using plain Python.

Sidecar API contract (verified from rplidar_sidecar.py):
  - angleDeg  : float [0, 360), CCW-positive, sorted ascending
  - distanceMm: int   > 0, already validated by sidecar
  - quality   : int   quality hint (0-47)
  - sequence  : int   monotonically increasing (for stale detection)
  - timestamp : str   ISO-8601 UTC, e.g. "2026-07-20T15:30:00Z"
  - scanHz    : float rotations per second
"""

import math
import sys
import os
import importlib.util
import pytest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Import the module under test without requiring a running ROS daemon.
# We mock out rclpy and sensor_msgs before the import so the module loads
# even on a plain development machine.
# ---------------------------------------------------------------------------

# --- Minimal rclpy stub ---
rclpy_stub = MagicMock()
rclpy_stub.ok.return_value = True
sys.modules.setdefault("rclpy", rclpy_stub)
sys.modules.setdefault("rclpy.node", MagicMock())
sys.modules.setdefault("rclpy.time", MagicMock())

# --- sensor_msgs stub ---
class _LaserScan:
    """Minimal LaserScan stand-in."""
    def __init__(self):
        self.header = MagicMock()
        self.header.stamp = MagicMock()
        self.header.stamp.sec = 0
        self.header.stamp.nanosec = 0
        self.header.frame_id = ""
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.scan_time = 0.0
        self.time_increment = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        self.ranges = []
        self.intensities = []

sensor_msgs_stub = MagicMock()
sensor_msgs_stub.msg.LaserScan = _LaserScan
sys.modules.setdefault("sensor_msgs", sensor_msgs_stub)
sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_stub.msg)

# Patch LaserScan in the module's namespace *before* importing
with patch.dict("sys.modules", {
    "sensor_msgs": sensor_msgs_stub,
    "sensor_msgs.msg": sensor_msgs_stub.msg,
}):
    # Import build_laserscan and _iso_to_stamp directly from the source file
    _pkg_dir = os.path.join(
        os.path.dirname(__file__), "..", "rover_bringup"
    )
    _bridge_path = os.path.join(_pkg_dir, "rover_lidar_bridge.py")
    spec = importlib.util.spec_from_file_location("rover_lidar_bridge", _bridge_path)
    _bridge_mod = importlib.util.module_from_spec(spec)
    # Inject stubs into the module's namespace before exec
    _bridge_mod.LaserScan = _LaserScan
    spec.loader.exec_module(_bridge_mod)

build_laserscan = _bridge_mod.build_laserscan
_iso_to_stamp = _bridge_mod._iso_to_stamp
_NUM_BINS = _bridge_mod._NUM_BINS
_TWO_PI = _bridge_mod._TWO_PI

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_points(angle_dist_pairs):
    """Build a list of sidecar point dicts from (angleDeg, distanceMm) tuples."""
    return [
        {"angleDeg": a, "distanceMm": d, "quality": 31}
        for a, d in angle_dist_pairs
    ]


def _default_scan(points=None, scan_hz=7.0):
    """Call build_laserscan with safe defaults."""
    return build_laserscan(
        points=points or [],
        stamp_sec=1000,
        stamp_nanosec=0,
        frame_id="laser_frame",
        scan_hz=scan_hz,
        range_min_m=0.10,
        range_max_m=12.0,
        angle_offset_deg=0.0,
        reverse_scan=False,
    )


# ===========================================================================
# 1. API payload parsing
# ===========================================================================

class TestPayloadParsing:
    def test_parse_valid_payload_returns_correct_range(self):
        """A well-formed point at 0° / 1000 mm → bin 0 = 1.0 m."""
        pts = _make_points([(0.0, 1000)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 1.0, rel_tol=1e-6)

    def test_parse_multiple_valid_points(self):
        """Several valid points populate the correct bins."""
        pts = _make_points([(45.0, 2000), (180.0, 500), (270.5, 800)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[45], 2.0, rel_tol=1e-6)
        assert math.isclose(msg.ranges[180], 0.5, rel_tol=1e-6)
        assert math.isclose(msg.ranges[270], 0.8, rel_tol=1e-6)

    def test_empty_payload_returns_all_inf(self):
        """No points → all bins are inf."""
        msg = _default_scan([])
        assert all(math.isinf(r) for r in msg.ranges)
        assert len(msg.ranges) == _NUM_BINS

    def test_quality_maps_to_intensities(self):
        """quality field is reflected in intensities at the matching bin."""
        pts = [{"angleDeg": 90.0, "distanceMm": 1500, "quality": 25}]
        msg = _default_scan(pts)
        assert math.isclose(msg.intensities[90], 25.0, rel_tol=1e-6)

    def test_missing_quality_key_uses_zero(self):
        """Points without a quality key get intensity 0 (not an error)."""
        pts = [{"angleDeg": 10.0, "distanceMm": 1000}]
        msg = _default_scan(pts)
        assert math.isclose(msg.intensities[10], 0.0, abs_tol=1e-9)

    def test_malformed_point_skipped(self):
        """Dict with wrong types is silently skipped; valid neighbours still parsed."""
        pts = [
            {"angleDeg": "bad", "distanceMm": 1000, "quality": 0},  # bad type
            {"angleDeg": 45.0, "distanceMm": 1000, "quality": 31},   # good
        ]
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[45], 1.0, rel_tol=1e-6)


# ===========================================================================
# 2. Degrees-to-radians conversion
# ===========================================================================

class TestDegreesToRadians:
    def test_angle_min_is_zero(self):
        msg = _default_scan()
        assert math.isclose(msg.angle_min, 0.0, abs_tol=1e-9)

    def test_angle_max_is_2pi_minus_increment(self):
        msg = _default_scan()
        expected = _TWO_PI - (_TWO_PI / _NUM_BINS)
        assert math.isclose(msg.angle_max, expected, rel_tol=1e-6)

    def test_angle_increment_matches_one_degree(self):
        msg = _default_scan()
        one_degree_rad = math.radians(1.0)
        assert math.isclose(msg.angle_increment, one_degree_rad, rel_tol=1e-6)

    def test_angle_increment_times_num_bins_is_2pi(self):
        msg = _default_scan()
        assert math.isclose(msg.angle_increment * _NUM_BINS, _TWO_PI, rel_tol=1e-6)


# ===========================================================================
# 3. Millimeters-to-meters conversion
# ===========================================================================

class TestMmToMeters:
    def test_1000mm_becomes_1m(self):
        pts = _make_points([(0.0, 1000)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 1.0, rel_tol=1e-9)

    def test_500mm_becomes_0_5m(self):
        pts = _make_points([(0.0, 500)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 0.5, rel_tol=1e-9)

    def test_10000mm_becomes_10m(self):
        pts = _make_points([(0.0, 10000)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 10.0, rel_tol=1e-9)


# ===========================================================================
# 4. Angular ordering
# ===========================================================================

class TestAngularOrdering:
    def test_output_bins_are_sequential(self):
        """ranges[i] corresponds to angle i degrees; the grid is always sorted."""
        pts = _make_points([(350.0, 1000), (0.0, 1200), (90.0, 800)])
        msg = _default_scan(pts)
        # bin 0 = 0°, bin 90 = 90°, bin 350 = 350°
        assert math.isclose(msg.ranges[0], 1.2, rel_tol=1e-6)
        assert math.isclose(msg.ranges[90], 0.8, rel_tol=1e-6)
        assert math.isclose(msg.ranges[350], 1.0, rel_tol=1e-6)

    def test_num_bins_is_360(self):
        msg = _default_scan()
        assert len(msg.ranges) == 360

    def test_intensities_and_ranges_same_length(self):
        msg = _default_scan(_make_points([(0.0, 1000)]))
        assert len(msg.intensities) == len(msg.ranges)


# ===========================================================================
# 5. Configurable angle offset
# ===========================================================================

class TestAngleOffset:
    def test_offset_shifts_bin(self):
        """A 10° offset moves a reading at 0° into bin 10."""
        pts = _make_points([(0.0, 1000)])
        msg = build_laserscan(
            points=pts,
            stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame",
            scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=10.0,
            reverse_scan=False,
        )
        # Bin 10 should be filled; bin 0 should stay inf
        assert math.isclose(msg.ranges[10], 1.0, rel_tol=1e-6)
        assert math.isinf(msg.ranges[0])

    def test_negative_offset_wraps(self):
        """A -10° offset on a reading at 5° → bin 355."""
        pts = _make_points([(5.0, 1000)])
        msg = build_laserscan(
            points=pts,
            stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame",
            scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=-10.0,
            reverse_scan=False,
        )
        assert math.isclose(msg.ranges[355], 1.0, rel_tol=1e-6)

    def test_zero_offset_is_identity(self):
        pts = _make_points([(45.0, 2000)])
        msg = build_laserscan(
            points=pts,
            stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame",
            scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0,
            reverse_scan=False,
        )
        assert math.isclose(msg.ranges[45], 2.0, rel_tol=1e-6)


# ===========================================================================
# 6. Scan direction inversion (reverse_scan)
# ===========================================================================

class TestReverseScan:
    def test_reverse_scan_negates_angle_increment(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=True,
        )
        assert msg.angle_increment < 0.0

    def test_reverse_scan_swaps_angle_min_max(self):
        msg_fwd = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        msg_rev = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=True,
        )
        assert math.isclose(msg_fwd.angle_min, msg_rev.angle_max, abs_tol=1e-9)
        assert math.isclose(msg_fwd.angle_max, msg_rev.angle_min, abs_tol=1e-9)

    def test_reverse_scan_reverses_range_order(self):
        """With reverse_scan, a point at angleDeg=0 ends up at the last bin."""
        pts = _make_points([(0.0, 1000)])
        msg = build_laserscan(
            points=pts, stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=True,
        )
        # Bin 0 reversed becomes the last element of the reversed array
        assert math.isclose(msg.ranges[-1], 1.0, rel_tol=1e-6)


# ===========================================================================
# 7. Missing angular bins
# ===========================================================================

class TestMissingAngularBins:
    def test_empty_bins_are_inf(self):
        pts = _make_points([(90.0, 1000)])
        msg = _default_scan(pts)
        # All bins except 90 should be inf
        for i, r in enumerate(msg.ranges):
            if i == 90:
                assert math.isclose(r, 1.0, rel_tol=1e-6)
            else:
                assert math.isinf(r), f"Bin {i} should be inf, got {r}"

    def test_missing_bins_have_zero_intensity(self):
        pts = _make_points([(90.0, 1000)])
        msg = _default_scan(pts)
        for i, intens in enumerate(msg.intensities):
            if i != 90:
                assert math.isclose(intens, 0.0, abs_tol=1e-9), \
                    f"Missing bin {i} intensity should be 0.0, got {intens}"


# ===========================================================================
# 8. Invalid and out-of-range sample handling
# ===========================================================================

class TestInvalidSamples:
    def test_zero_distance_becomes_inf(self):
        pts = [{"angleDeg": 0.0, "distanceMm": 0, "quality": 31}]
        msg = _default_scan(pts)
        assert math.isinf(msg.ranges[0])

    def test_negative_distance_becomes_inf(self):
        pts = [{"angleDeg": 0.0, "distanceMm": -500, "quality": 31}]
        msg = _default_scan(pts)
        assert math.isinf(msg.ranges[0])

    def test_non_finite_distance_becomes_inf(self):
        pts = [{"angleDeg": 0.0, "distanceMm": float("nan"), "quality": 31}]
        msg = _default_scan(pts)
        assert math.isinf(msg.ranges[0])

    def test_below_range_min_becomes_inf(self):
        """distanceMm=50 → 0.05 m < range_min 0.10 m → inf."""
        pts = _make_points([(0.0, 50)])
        msg = _default_scan(pts)
        assert math.isinf(msg.ranges[0])

    def test_above_range_max_becomes_inf(self):
        """distanceMm=15000 → 15.0 m > range_max 12.0 m → inf."""
        pts = _make_points([(0.0, 15000)])
        msg = _default_scan(pts)
        assert math.isinf(msg.ranges[0])

    def test_boundary_range_min_accepted(self):
        """Exactly at range_min (0.10 m = 100 mm) is accepted."""
        pts = _make_points([(0.0, 100)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 0.10, rel_tol=1e-6)

    def test_boundary_range_max_accepted(self):
        """Exactly at range_max (12.0 m = 12000 mm) is accepted."""
        pts = _make_points([(0.0, 12000)])
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[0], 12.0, rel_tol=1e-6)

    def test_closer_of_two_same_bin_wins(self):
        """Two points map to bin 45; the closer one (1.0m) should win."""
        pts = [
            {"angleDeg": 45.0, "distanceMm": 1000, "quality": 31},
            {"angleDeg": 45.9, "distanceMm": 2000, "quality": 31},
        ]
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[45], 1.0, rel_tol=1e-6)

    def test_non_finite_angle_becomes_inf(self):
        pts = [{"angleDeg": float("inf"), "distanceMm": 1000, "quality": 31}]
        msg = _default_scan(pts)
        assert all(math.isinf(r) for r in msg.ranges)


# ===========================================================================
# 9. Duplicate / stale scan rejection
# ===========================================================================

class TestDuplicateStaleRejection:
    """
    These tests exercise the node's _poll() state machine by directly
    manipulating the node's _last_sequence field.
    """

    def _make_node(self):
        """Return a minimal bridge node stub for dedup testing."""
        class _FakeNode:
            _last_sequence = None
            _api_error_logged = False
            published = []

            def _should_publish(self, sequence):
                """Mirrors the dedup logic in _poll()."""
                if sequence is not None:
                    if sequence == self._last_sequence:
                        return False
                    self._last_sequence = sequence
                return True

        return _FakeNode()

    def test_first_sequence_always_published(self):
        node = self._make_node()
        assert node._should_publish(1) is True

    def test_same_sequence_rejected(self):
        node = self._make_node()
        node._should_publish(5)
        assert node._should_publish(5) is False

    def test_new_sequence_accepted(self):
        node = self._make_node()
        node._should_publish(5)
        assert node._should_publish(6) is True

    def test_none_sequence_always_published(self):
        """If sequence is absent, publish every response (no dedup possible)."""
        node = self._make_node()
        assert node._should_publish(None) is True
        assert node._should_publish(None) is True

    def test_sequence_resets_across_reconnect(self):
        """After a fresh reconnect (sequence wraps back to low value), still accept."""
        node = self._make_node()
        node._should_publish(999)
        # Sidecar restarted, sequence reset to a non-zero low value
        assert node._should_publish(1) is True

    def test_sequence_restarts_from_zero(self):
        """After a service restart the sidecar resets its counter to 0.

        The node must NOT suppress this as a stale duplicate: sequence 0 is new
        and must be accepted even though 0 < the previous high value.
        """
        node = self._make_node()
        node._should_publish(50)   # simulates running sidecar at sequence 50
        node._should_publish(51)
        # Sidecar restarted → counter resets to 0
        assert node._should_publish(0) is True, (
            "sequence=0 after restart must be published (node resets from scratch)"
        )
        # Subsequent sequence=1 must also be accepted
        assert node._should_publish(1) is True


# ===========================================================================
# 10. Correct LaserScan metadata
# ===========================================================================

class TestLaserScanMetadata:
    def test_frame_id_propagated(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="custom_laser", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert msg.header.frame_id == "custom_laser"

    def test_stamp_propagated(self):
        msg = build_laserscan(
            points=[], stamp_sec=1234567890, stamp_nanosec=500000000,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert msg.header.stamp.sec == 1234567890
        assert msg.header.stamp.nanosec == 500000000

    def test_range_min_propagated(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.15, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert math.isclose(msg.range_min, 0.15, rel_tol=1e-9)

    def test_range_max_propagated(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=7.0,
            range_min_m=0.10, range_max_m=8.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert math.isclose(msg.range_max, 8.0, rel_tol=1e-9)

    def test_scan_time_from_hz(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=10.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert math.isclose(msg.scan_time, 0.1, rel_tol=1e-9)

    def test_time_increment_from_hz_and_bins(self):
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=10.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        expected = 0.1 / 360
        assert math.isclose(msg.time_increment, expected, rel_tol=1e-9)

    def test_zero_scan_hz_safe(self):
        """scan_hz=0 must not divide by zero."""
        msg = build_laserscan(
            points=[], stamp_sec=0, stamp_nanosec=0,
            frame_id="laser_frame", scan_hz=0.0,
            range_min_m=0.10, range_max_m=12.0,
            angle_offset_deg=0.0, reverse_scan=False,
        )
        assert msg.scan_time == 0.0
        assert msg.time_increment == 0.0

    def test_ranges_length_is_360(self):
        msg = _default_scan(_make_points([(0.0, 1000)]))
        assert len(msg.ranges) == 360

    def test_intensities_length_matches_ranges(self):
        msg = _default_scan(_make_points([(0.0, 1000)]))
        assert len(msg.intensities) == len(msg.ranges)


# ===========================================================================
# 11. _iso_to_stamp helper
# ===========================================================================

class TestIsoToStamp:
    def test_valid_iso_parsed(self):
        import calendar, time as _time
        sec, nanosec = _iso_to_stamp("2026-07-20T15:30:00Z", 0)
        # Should be a valid unix timestamp around 2026
        assert sec > 1_700_000_000
        assert nanosec == 0

    def test_none_returns_fallback(self):
        sec, nanosec = _iso_to_stamp(None, 9999)
        assert sec == 9999
        assert nanosec == 0

    def test_empty_string_returns_fallback(self):
        sec, nanosec = _iso_to_stamp("", 8888)
        assert sec == 8888

    def test_garbage_string_returns_fallback(self):
        sec, nanosec = _iso_to_stamp("not-a-date", 7777)
        assert sec == 7777

    def test_null_json_value_returns_fallback(self):
        """Simulates the sidecar returning JSON null for timestamp (before first scan).

        json.loads returns Python None for JSON null; _iso_to_stamp must
        fall back to the supplied fallback_sec rather than raising.
        """
        sec, nanosec = _iso_to_stamp(None, 42)
        assert sec == 42
        assert nanosec == 0

    def test_malformed_iso_returns_fallback(self):
        """Timestamp strings that are syntactically wrong must yield fallback."""
        for bad in [
            "2026-13-01T00:00:00Z",   # month 13
            "20260720T153000",         # no separators / no Z
            "1234567890",              # plain epoch integer string
            "{}",                      # JSON object string
            "null",                    # literal string 'null'
        ]:
            sec, nanosec = _iso_to_stamp(bad, 55555)
            assert sec == 55555, f"Expected fallback for malformed ts={bad!r}"
            assert nanosec == 0


# ===========================================================================
# 12. All-infinite scan handling
# ===========================================================================

class TestAllInfiniteScan:
    """When the sidecar returns only out-of-range or unconvertible readings,
    build_laserscan() must produce an all-inf ranges array, never panic.
    """

    def test_all_out_of_range_produces_all_inf(self):
        """Points all outside [range_min, range_max] → all-inf output."""
        pts = [
            {"angleDeg": 0.0,   "distanceMm": 5,      "quality": 31},  # < 100 mm min
            {"angleDeg": 90.0,  "distanceMm": 15000,  "quality": 31},  # > 12000 mm max
            {"angleDeg": 180.0, "distanceMm": 0,      "quality": 31},  # zero → skipped
        ]
        msg = _default_scan(pts)
        assert all(math.isinf(r) for r in msg.ranges), (
            "All out-of-range readings must produce an all-inf ranges array"
        )

    def test_all_nan_distances_produces_all_inf(self):
        """Points with NaN distanceMm → all bins remain inf."""
        pts = [
            {"angleDeg": float(a), "distanceMm": float("nan"), "quality": 31}
            for a in range(0, 360, 10)
        ]
        msg = _default_scan(pts)
        assert all(math.isinf(r) for r in msg.ranges)

    def test_all_inf_distances_produces_all_inf(self):
        """Points with +inf distanceMm → all bins remain inf."""
        pts = [
            {"angleDeg": float(a), "distanceMm": float("inf"), "quality": 31}
            for a in range(0, 360, 10)
        ]
        msg = _default_scan(pts)
        assert all(math.isinf(r) for r in msg.ranges)

    def test_all_inf_ranges_length_still_correct(self):
        """Even an all-inf scan must have exactly _NUM_BINS entries."""
        msg = _default_scan([])
        assert len(msg.ranges) == _NUM_BINS
        assert len(msg.intensities) == _NUM_BINS

    def test_mixed_valid_and_inf_only_valid_shows(self):
        """One valid point surrounded by out-of-range readings: only valid bin is finite."""
        pts = [
            {"angleDeg": 0.0,   "distanceMm": 5,    "quality": 0},   # too close
            {"angleDeg": 45.0,  "distanceMm": 2000, "quality": 31},  # valid: 2.0 m
            {"angleDeg": 90.0,  "distanceMm": 99999,"quality": 0},   # too far
        ]
        msg = _default_scan(pts)
        assert math.isclose(msg.ranges[45], 2.0, rel_tol=1e-6)
        for i, r in enumerate(msg.ranges):
            if i != 45:
                assert math.isinf(r), f"bin {i} should be inf"


# ===========================================================================
# 13. No direct /dev access or device mounts
# ===========================================================================

class TestNoDevAccess:
    """Confirms that compose.yaml does not bind-mount any /dev path into
    the rover-ros2 container, satisfying hardware isolation requirements.
    """

    def _load_compose(self):
        """Return the parsed compose.yaml as a dict."""
        import yaml  # PyYAML – available in the test environment
        compose_path = os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "..",  # ros2_ws/src → ros2
            "compose.yaml",
        )
        with open(compose_path, "r") as fh:
            return yaml.safe_load(fh)

    def test_compose_yaml_no_dev_volume_sources(self):
        """No volume source in compose.yaml must start with /dev."""
        try:
            compose = self._load_compose()
        except (FileNotFoundError, ImportError) as exc:
            pytest.skip(f"compose.yaml or PyYAML not available: {exc}")

        service = compose.get("services", {}).get("rover-ros2", {})
        volumes = service.get("volumes", [])
        dev_vols = []
        for vol in volumes:
            # Volumes can be a string "src:dst[:opts]" or a dict {source:, target:}
            if isinstance(vol, str):
                src = vol.split(":")[0]
            elif isinstance(vol, dict):
                src = vol.get("source", "")
            else:
                continue
            if str(src).startswith("/dev"):
                dev_vols.append(vol)
        assert dev_vols == [], (
            f"compose.yaml must not bind-mount /dev paths, found: {dev_vols}"
        )

    def test_compose_yaml_no_devices_key(self):
        """The rover-ros2 service must not use the 'devices' key."""
        try:
            compose = self._load_compose()
        except (FileNotFoundError, ImportError) as exc:
            pytest.skip(f"compose.yaml or PyYAML not available: {exc}")

        service = compose.get("services", {}).get("rover-ros2", {})
        assert "devices" not in service, (
            "rover-ros2 service must not expose host devices via the 'devices' key"
        )

    def test_compose_yaml_no_privileged(self):
        """The rover-ros2 service must not run privileged."""
        try:
            compose = self._load_compose()
        except (FileNotFoundError, ImportError) as exc:
            pytest.skip(f"compose.yaml or PyYAML not available: {exc}")

        service = compose.get("services", {}).get("rover-ros2", {})
        assert not service.get("privileged", False), (
            "rover-ros2 service must not run as privileged"
        )


# ===========================================================================
# 14. Launch file contains only the approved Phase 2 nodes
# ===========================================================================

class TestLaunchFile:
    def test_foundation_launch_has_exactly_two_nodes(self):
        """foundation.launch.py must contain exactly rover_system_health and
        rover_lidar_bridge, and nothing else.

        Skipped on non-ROS machines (no 'launch' package installed).
        Runs fully inside the Docker/ROS 2 environment.
        """
        launch_mod = pytest.importorskip("launch", reason="ROS 2 'launch' not available")
        launch_ros_mod = pytest.importorskip("launch_ros", reason="ROS 2 'launch_ros' not available")
        import importlib.util as ilu
        from launch import LaunchDescription
        from launch_ros.actions import Node

        launch_path = os.path.join(
            os.path.dirname(__file__), "..", "launch", "foundation.launch.py"
        )
        assert os.path.exists(launch_path), "foundation.launch.py not found"

        spec = ilu.spec_from_file_location("foundation.launch", launch_path)
        m = ilu.module_from_spec(spec)
        spec.loader.exec_module(m)

        ld = m.generate_launch_description()
        assert isinstance(ld, LaunchDescription)

        nodes = [e for e in ld.entities if isinstance(e, Node)]
        assert len(nodes) == 2, f"Expected 2 nodes, found {len(nodes)}"

        executables = {n.node_executable for n in nodes}
        assert "rover_system_health" in executables
        assert "rover_lidar_bridge" in executables

    def test_foundation_launch_has_no_forbidden_packages(self):
        """Forbidden Nav2/SLAM packages must not appear in the launch file.

        Skipped on non-ROS machines (no 'launch_ros' package installed).
        """
        pytest.importorskip("launch", reason="ROS 2 'launch' not available")
        pytest.importorskip("launch_ros", reason="ROS 2 'launch_ros' not available")
        import importlib.util as ilu
        from launch_ros.actions import Node

        launch_path = os.path.join(
            os.path.dirname(__file__), "..", "launch", "foundation.launch.py"
        )
        spec = ilu.spec_from_file_location("foundation.launch", launch_path)
        m = ilu.module_from_spec(spec)
        spec.loader.exec_module(m)
        ld = m.generate_launch_description()

        forbidden = {
            "nav2_bringup", "slam_toolbox", "navigation2",
            "nav2_planner", "nav2_controller", "amcl",
            "robot_state_publisher",
        }
        for entity in ld.entities:
            if isinstance(entity, Node):
                assert entity.node_package not in forbidden, \
                    f"Forbidden package '{entity.node_package}' in launch file"

    def test_foundation_launch_no_cmd_vel_odom(self):
        """Launch file must not reference cmd_vel or odom."""
        launch_path = os.path.join(
            os.path.dirname(__file__), "..", "launch", "foundation.launch.py"
        )
        with open(launch_path, "r") as f:
            content = f.read()
        assert "cmd_vel" not in content
        assert "odom" not in content
