#!/usr/bin/env python3
# ==============================================================================
# rover_encoder_odometry.py - ROS 2 Encoder Odometry & Dynamic TF Node (Phase 3)
# ==============================================================================
#
# Reads encoder telemetry from the host Cockpit HTTP API (GET /api/encoders) and
# publishes nav_msgs/msg/Odometry on /odom and dynamic TF odom -> base_link.
#
# The physical ESP32 device is owned exclusively by the host
# rover-server.service. This node does NOT open any serial device and requires
# no /dev mounts in Docker.
# ==============================================================================

import json
import math
import os
import threading
import time
from typing import Optional, Tuple, List
from http.server import HTTPServer, BaseHTTPRequestHandler

from geometry_msgs.msg import TransformStamped, PolygonStamped, Point32, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
import requests
from tf2_ros import TransformBroadcaster

from rover_bringup.encoder_kinematics import EncoderKinematics, normalize_angle


class OdomAPIHandler(BaseHTTPRequestHandler):
    node_ref = None

    def log_message(self, format, *args):
        pass  # Suppress HTTP access logging in ROS logs

    def do_GET(self):
        try:
            if self.path in ['/api/odom', '/odom', '/status']:
                if OdomAPIHandler.node_ref is None:
                    body = json.dumps({"ok": False, "error": "Node uninitialized"}).encode('utf-8')
                    self.send_response(503)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                node = OdomAPIHandler.node_ref
                now_ts = time.time()
                last_ts = getattr(node.kinematics, 'last_timestamp_sec', None)
                if last_ts is not None and last_ts > 0:
                    odom_age_ms = int((now_ts - last_ts) * 1000.0)
                else:
                    odom_age_ms = 0

                consec_errs = getattr(node, 'consecutive_errors', 0)
                is_ok = (consec_errs == 0) and (odom_age_ms < 2000)
                health = "ok" if is_ok else ("degraded" if odom_age_ms < 5000 else "stale")

                data = {
                    "ok": is_ok,
                    "timestamp": now_ts,
                    "x": float(getattr(node.kinematics, 'x', 0.0)),
                    "y": float(getattr(node.kinematics, 'y', 0.0)),
                    "yaw": float(getattr(node.kinematics, 'yaw', 0.0)),
                    "yaw_deg": float(math.degrees(getattr(node.kinematics, 'yaw', 0.0))),
                    "v_x": float(getattr(node.kinematics, 'v_x', 0.0)),
                    "w_z": float(getattr(node.kinematics, 'w_z', 0.0)),
                    "odometry_age_ms": odom_age_ms,
                    "node_health": health,
                    "consecutive_errors": consec_errs,
                    "slip_gate_active": bool(getattr(node.kinematics, 'slip_gate_active', False)),
                    "slip_event_count": int(getattr(node.kinematics, 'slip_event_count', 0)),
                    "raw_d_left_m": float(getattr(node.kinematics, 'last_raw_d_left_m', 0.0)),
                    "raw_d_right_m": float(getattr(node.kinematics, 'last_raw_d_right_m', 0.0)),
                    "wheel_disparity_m": float(getattr(node.kinematics, 'last_wheel_disparity_m', 0.0)),
                    "d_yaw_wheel_rad": float(getattr(node.kinematics, 'last_d_yaw_wheel_rad', 0.0)),
                    "external_d_yaw_rad": getattr(node.kinematics, 'last_external_d_yaw_rad', None),
                    "yaw_disagreement_rad": float(getattr(node.kinematics, 'last_yaw_disagreement_rad', 0.0)),
                    "imu_yaw_valid": bool(getattr(node.kinematics, 'last_imu_yaw_valid', False)),
                    "ratio_fallback_used": bool(getattr(node.kinematics, 'last_ratio_fallback_used', False)),
                    "ungated_d_center_m": float(getattr(node.kinematics, 'last_ungated_d_center_m', 0.0)),
                    "gated_d_center_m": float(getattr(node.kinematics, 'last_gated_d_center_m', 0.0)),
                    "slip_reason": str(getattr(node.kinematics, 'last_slip_reason', '')),
                }
                body = json.dumps(data).encode('utf-8')
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()
        except Exception as err:
            try:
                body = json.dumps({"ok": False, "error": str(err)}).encode('utf-8')
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception:
                pass




class RoverEncoderOdometry(Node):
    """
    ROS 2 node that polls read-only encoder telemetry from the host Cockpit API,
    integrates skid-steer odometry, subscribes to /imu/data to integrate gyro Z,
    publishes /odom, and broadcasts odom -> base_link TF.
    """

    def __init__(self):
        super().__init__('rover_encoder_odometry')

        # Environment variable fallback for telemetry URL
        host_server_url = os.environ.get('ROVER_SERVER_URL', 'http://127.0.0.1:3000')
        default_telemetry_url = f"{host_server_url.rstrip('/')}/api/encoders"

        # Parameters
        self.declare_parameter('telemetry_url', default_telemetry_url)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('wheel_diameter_m', 0.065)
        self.declare_parameter('track_width_m', 0.3408575433)
        self.declare_parameter('physical_track_width_m', 0.197)
        self.declare_parameter('ticks_per_revolution', 1974.1666666667)
        self.declare_parameter('m1_sign', 1.0)
        self.declare_parameter('m2_sign', 1.0)
        self.declare_parameter('m3_sign', 1.0)
        self.declare_parameter('m4_sign', 1.0)
        self.declare_parameter('wheel_mapping', ['m1', 'm2', 'm3', 'm4'])
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('reset_threshold_ticks', 100000)
        self.declare_parameter('disagreement_threshold_ticks', 100)
        self.declare_parameter('max_plausible_wheel_speed_mps', 2.5)

        # Retrieve parameter values
        self.telemetry_url = self.get_parameter('telemetry_url').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        wheel_diameter = self.get_parameter('wheel_diameter_m').get_parameter_value().double_value
        track_width = self.get_parameter('track_width_m').get_parameter_value().double_value
        ticks_per_rev = self.get_parameter('ticks_per_revolution').get_parameter_value().double_value

        self.wheel_mapping = list(self.get_parameter('wheel_mapping').get_parameter_value().string_array_value)
        if len(self.wheel_mapping) < 4:
            self.wheel_mapping = ['m1', 'm2', 'm3', 'm4']

        m1_sign = self.get_parameter('m1_sign').get_parameter_value().double_value
        m2_sign = self.get_parameter('m2_sign').get_parameter_value().double_value
        m3_sign = self.get_parameter('m3_sign').get_parameter_value().double_value
        m4_sign = self.get_parameter('m4_sign').get_parameter_value().double_value
        stale_timeout = self.get_parameter('stale_timeout_sec').get_parameter_value().double_value
        reset_thresh = self.get_parameter('reset_threshold_ticks').get_parameter_value().integer_value
        disagree_thresh = self.get_parameter('disagreement_threshold_ticks').get_parameter_value().integer_value
        max_speed = self.get_parameter('max_plausible_wheel_speed_mps').get_parameter_value().double_value

        default_odom_api_port = int(os.environ.get('ROVER_ODOM_HTTP_PORT', '3003'))
        self.declare_parameter('odom_api_port', default_odom_api_port)
        self.odom_api_port = self.get_parameter('odom_api_port').get_parameter_value().integer_value

        # Kinematics engine initialization
        self.kinematics = EncoderKinematics(
            wheel_radius_m=wheel_diameter / 2.0,
            track_width_m=track_width,
            ticks_per_revolution=ticks_per_rev,
            m1_sign=m1_sign,
            m2_sign=m2_sign,
            m3_sign=m3_sign,
            m4_sign=m4_sign,
            reset_threshold_ticks=reset_thresh,
            disagreement_threshold_ticks=disagree_thresh,
            max_plausible_wheel_speed_mps=max_speed,
            stale_timeout_sec=stale_timeout,
        )

        # Publishers & TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.footprint_pub = self.create_publisher(PolygonStamped, '/footprint', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rover_footprint_marker', 10)
        self.vis_marker_pub = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # IMU subscription and gyro integration state
        self._imu_gyro_buffer = []  # List of (timestamp_sec: float, gz_rad_per_sec: float)
        self._imu_lock = threading.Lock()
        self._last_imu_recv_time_sec = 0.0

        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, 10
        )

        # HTTP session setup
        self.http_session = requests.Session()

        # Connection / Error state
        self.consecutive_errors = 0

        # Start HTTP API server for host odometry queries
        OdomAPIHandler.node_ref = self
        self._start_http_server(self.odom_api_port)

        # Timer setup
        timer_period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(timer_period, self._poll_and_publish)

        self.get_logger().info(
            f"rover_encoder_odometry initialized. Polling '{self.telemetry_url}' at {self.publish_rate_hz} Hz. "
            f"Subscribed to IMU topic '{self.imu_topic}'. "
            f"HTTP Odom API serving on port {self.odom_api_port}."
        )

    def _start_http_server(self, port: int):
        try:
            httpd = HTTPServer(('0.0.0.0', port), OdomAPIHandler)
            thread = threading.Thread(target=httpd.serve_forever, daemon=True)
            thread.start()
            self.get_logger().info(f"Odom HTTP API server listening on 0.0.0.0:{port}")
        except Exception as err:
            self.get_logger().error(f"Failed to start Odom HTTP API server on port {port}: {err}")

    def _imu_callback(self, msg: Imu):
        """Receive incoming BNO08x 50Hz sensor_msgs/Imu and buffer gyro Z samples."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z

        if math.isfinite(now_sec) and math.isfinite(gz):
            with self._imu_lock:
                self._last_imu_recv_time_sec = time.time()
                self._imu_gyro_buffer.append((now_sec, gz))

                # Keep up to 5.0 seconds of gyro data in buffer
                cutoff = now_sec - 5.0
                while self._imu_gyro_buffer and self._imu_gyro_buffer[0][0] < cutoff:
                    self._imu_gyro_buffer.pop(0)

    def _compute_integrated_gyro_yaw(self, t_prev: Optional[float], t_curr: float) -> Optional[float]:
        """
        Integrate gyro Z (angular_velocity.z in rad/s) over interval [t_prev, t_curr].
        Returns float (radians) if valid IMU samples cover [t_prev, t_curr], else None.
        """
        if t_prev is None or t_curr is None or t_curr <= t_prev:
            return None

        now_wall = time.time()
        with self._imu_lock:
            # Check staleness: if no IMU msg received in last 0.5s
            if self._last_imu_recv_time_sec == 0.0 or (now_wall - self._last_imu_recv_time_sec > 0.5):
                return None

            if len(self._imu_gyro_buffer) < 2:
                return None

            buf = self._imu_gyro_buffer
            # If buffer newest is older than t_prev or buffer oldest is newer than t_curr
            if buf[-1][0] < t_prev or buf[0][0] > t_curr:
                return None

            # Extract samples within 0.2s padding of the encoder window
            samples = [s for s in buf if (t_prev - 0.2) <= s[0] <= (t_curr + 0.2)]
            if len(samples) < 2:
                return None

            def interp_gz(target_t: float) -> Optional[float]:
                for t, gz in samples:
                    if abs(t - target_t) < 1e-9:
                        return gz
                for i in range(len(samples) - 1):
                    t0, g0 = samples[i]
                    t1, g1 = samples[i + 1]
                    if t0 <= target_t <= t1:
                        if t1 == t0:
                            return g0
                        ratio = (target_t - t0) / (t1 - t0)
                        return g0 + ratio * (g1 - g0)
                if target_t < samples[0][0] and (samples[0][0] - target_t) <= 0.05:
                    return samples[0][1]
                if target_t > samples[-1][0] and (target_t - samples[-1][0]) <= 0.05:
                    return samples[-1][1]
                return None

            g_start = interp_gz(t_prev)
            g_end = interp_gz(t_curr)
            if g_start is None or g_end is None:
                return None

            pts = [(t_prev, g_start)]
            for t, gz in samples:
                if t_prev < t < t_curr:
                    pts.append((t, gz))
            pts.append((t_curr, g_end))

            pts.sort(key=lambda x: x[0])

            d_yaw = 0.0
            for i in range(len(pts) - 1):
                dt = pts[i + 1][0] - pts[i][0]
                if dt <= 0:
                    continue
                if dt > 0.2:  # Data gap inside interval > 200ms
                    return None
                d_yaw += 0.5 * (pts[i][1] + pts[i + 1][1]) * dt

            return d_yaw

    def _poll_and_publish(self):
        """Timer callback: poll GET /api/encoders, update kinematics, publish /odom & TF."""
        now_ros = self.get_clock().now()
        timestamp_sec = now_ros.nanoseconds * 1e-9

        try:
            resp = self.http_session.get(self.telemetry_url, timeout=0.25)
            if resp.status_code != 200:
                self._handle_degraded(f"HTTP GET returned status {resp.status_code}")
                return

            payload = resp.json()
            if not payload.get('ok', False):
                self._handle_degraded(f"API payload indicated not ok: {payload}")
                return

            encoders_data = payload.get('encoders', {})
            if isinstance(encoders_data, dict):
                ticks = [
                    encoders_data.get(self.wheel_mapping[0], 0),
                    encoders_data.get(self.wheel_mapping[1], 0),
                    encoders_data.get(self.wheel_mapping[2], 0),
                    encoders_data.get(self.wheel_mapping[3], 0),
                ]
            elif isinstance(encoders_data, list) and len(encoders_data) >= 4:
                ticks = [int(encoders_data[0]), int(encoders_data[1]), int(encoders_data[2]), int(encoders_data[3])]
            else:
                self._handle_degraded("Malformed encoders payload structure")
                return

            sequence = payload.get('sequence', None)
            source_ts_ms = payload.get('timestamp', None)
            if source_ts_ms is not None and source_ts_ms > 0:
                sample_time_sec = float(source_ts_ms) / 1000.0
            else:
                sample_time_sec = timestamp_sec

            # Calculate integrated gyro yaw over [last_fresh_timestamp_sec, sample_time_sec]
            t_prev = self.kinematics.last_fresh_timestamp_sec
            external_d_yaw = self._compute_integrated_gyro_yaw(t_prev, sample_time_sec)

            # Update kinematics
            success, msg = self.kinematics.update(ticks, sample_time_sec, sequence, external_d_yaw=external_d_yaw)
            if not success:
                self._handle_degraded(f"Kinematics update rejected sample: {msg}")
                return

            # Check wheel disagreement warning
            if self.kinematics.disagreement_warning:
                self.get_logger().warn(f"Wheel encoder disagreement: {self.kinematics.disagreement_details}", throttle_duration_sec=1.0)

            # Check slip gate active warning
            if self.kinematics.slip_gate_active:
                self.get_logger().warn(
                    f"TRANSLATION SLIP GATE ACTIVE (Event #{self.kinematics.slip_event_count}): {self.kinematics.last_slip_reason}",
                    throttle_duration_sec=1.0
                )

            self.consecutive_errors = 0

            # Publish odometry and broadcast TF
            self._publish_odom_and_tf(now_ros)

        except requests.exceptions.RequestException as err:
            self._handle_degraded(f"HTTP request error connecting to host Cockpit: {err}")
        except Exception as err:
            self._handle_degraded(f"Unexpected error in odometry loop: {err}")

    def _handle_degraded(self, reason: str):
        """Log degraded telemetry conditions without spamming."""
        self.consecutive_errors += 1
        if self.consecutive_errors == 1 or self.consecutive_errors % 50 == 0:
            self.get_logger().warn(f"Encoder odometry degraded ({self.consecutive_errors} errors): {reason}")

    def _publish_odom_and_tf(self, now_ros):
        """Construct and publish Odometry message and TF transform."""
        x = self.kinematics.x
        y = self.kinematics.y
        yaw = self.kinematics.yaw
        v_x = self.kinematics.v_x
        w_z = self.kinematics.w_z

        # Orientation quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        stamp_msg = now_ros.to_msg()

        # 1. Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp_msg
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        # Pose Covariance (6x6 matrix flattened)
        # Standard planar covariance estimates
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.005   # x
        pose_cov[7] = 0.005   # y
        pose_cov[14] = 999.0  # z
        pose_cov[21] = 999.0  # roll
        pose_cov[28] = 999.0  # pitch
        pose_cov[35] = 0.02   # yaw
        odom_msg.pose.covariance = pose_cov

        # Twist
        odom_msg.twist.twist.linear.x = float(v_x)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(w_z)

        # Twist Covariance
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.005   # vx
        twist_cov[7] = 999.0   # vy
        twist_cov[14] = 999.0  # vz
        twist_cov[21] = 999.0  # wx
        twist_cov[28] = 999.0  # wy
        twist_cov[35] = 0.02   # wz
        odom_msg.twist.covariance = twist_cov

        self.odom_pub.publish(odom_msg)

        # 2. Dynamic Transform (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

        # 3. Footprint Polygon Message (10 in x 9 in = 0.254m x 0.2286m in base_link frame)
        footprint_msg = PolygonStamped()
        footprint_msg.header.stamp = stamp_msg
        footprint_msg.header.frame_id = self.base_frame
        half_l = 0.254 / 2.0   # 0.127 m
        half_w = 0.2286 / 2.0  # 0.1143 m
        footprint_msg.polygon.points = [
            Point32(x=half_l, y=half_w, z=0.0),    # Front Left
            Point32(x=half_l, y=-half_w, z=0.0),   # Front Right
            Point32(x=-half_l, y=-half_w, z=0.0),  # Rear Right
            Point32(x=-half_l, y=half_w, z=0.0),   # Rear Left
        ]
        self.footprint_pub.publish(footprint_msg)

        # 4. Rover Footprint MarkerArray Message (10 in x 9 in = 0.254m x 0.2286m in base_link frame)
        marker_array = MarkerArray()

        # 4a. Semi-transparent Cyan Body (CUBE)
        m_body = Marker()
        m_body.header.stamp = stamp_msg
        m_body.header.frame_id = self.base_frame
        m_body.ns = "footprint_body"
        m_body.id = 0
        m_body.type = Marker.CUBE
        m_body.action = Marker.ADD
        m_body.pose.position.x = 0.0
        m_body.pose.position.y = 0.0
        m_body.pose.position.z = 0.0
        m_body.pose.orientation.w = 1.0
        m_body.scale.x = 0.254   # 10 inches (length)
        m_body.scale.y = 0.2286  # 9 inches (width)
        m_body.scale.z = 0.01    # thin chassis slab
        m_body.color.r = 0.0
        m_body.color.g = 0.95
        m_body.color.b = 1.0
        m_body.color.a = 0.20    # light semi-transparent cyan fill
        marker_array.markers.append(m_body)

        # 4b. Thin Cyan Outline (LINE_STRIP)
        m_outline = Marker()
        m_outline.header.stamp = stamp_msg
        m_outline.header.frame_id = self.base_frame
        m_outline.ns = "footprint_outline"
        m_outline.id = 1
        m_outline.type = Marker.LINE_STRIP
        m_outline.action = Marker.ADD
        m_outline.scale.x = 0.012  # line thickness
        m_outline.color.r = 0.0
        m_outline.color.g = 0.95
        m_outline.color.b = 1.0
        m_outline.color.a = 0.90   # bright cyan outline
        m_outline.points = [
            Point(x=half_l, y=half_w, z=0.01),
            Point(x=half_l, y=-half_w, z=0.01),
            Point(x=-half_l, y=-half_w, z=0.01),
            Point(x=-half_l, y=half_w, z=0.01),
            Point(x=half_l, y=half_w, z=0.01),
        ]
        marker_array.markers.append(m_outline)

        # 4c. Red Front/Nose Indicator (ARROW)
        m_nose = Marker()
        m_nose.header.stamp = stamp_msg
        m_nose.header.frame_id = self.base_frame
        m_nose.ns = "footprint_nose"
        m_nose.id = 2
        m_nose.type = Marker.ARROW
        m_nose.action = Marker.ADD
        m_nose.scale.x = 0.015  # shaft diameter
        m_nose.scale.y = 0.035  # head diameter
        m_nose.scale.z = 0.030  # head length
        m_nose.color.r = 1.0
        m_nose.color.g = 0.0
        m_nose.color.b = 0.33
        m_nose.color.a = 1.0    # solid red
        m_nose.points = [
            Point(x=0.04, y=0.0, z=0.015),
            Point(x=0.15, y=0.0, z=0.015),
        ]
        marker_array.markers.append(m_nose)

        self.marker_pub.publish(marker_array)
        self.vis_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RoverEncoderOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
