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

import math
import os
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import requests
from tf2_ros import TransformBroadcaster

from rover_bringup.encoder_kinematics import EncoderKinematics, normalize_angle


class RoverEncoderOdometry(Node):
    """
    ROS 2 node that polls read-only encoder telemetry from the host Cockpit API,
    integrates skid-steer odometry, publishes /odom, and broadcasts odom -> base_link TF.
    """

    def __init__(self):
        super().__init__('rover_encoder_odometry')

        # Environment variable fallback for telemetry URL
        host_server_url = os.environ.get('ROVER_SERVER_URL', 'http://127.0.0.1:3000')
        default_telemetry_url = f"{host_server_url.rstrip('/')}/api/encoders"

        # Parameters
        self.declare_parameter('telemetry_url', default_telemetry_url)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('wheel_diameter_m', 0.065)
        self.declare_parameter('track_width_m', 0.170)
        self.declare_parameter('ticks_per_revolution', 937.2)
        self.declare_parameter('m1_sign', 1.0)
        self.declare_parameter('m2_sign', 1.0)
        self.declare_parameter('m3_sign', 1.0)
        self.declare_parameter('m4_sign', 1.0)
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('reset_threshold_ticks', 100000)
        self.declare_parameter('disagreement_threshold_ticks', 100)
        self.declare_parameter('max_plausible_wheel_speed_mps', 2.5)

        # Retrieve parameter values
        self.telemetry_url = self.get_parameter('telemetry_url').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        wheel_diameter = self.get_parameter('wheel_diameter_m').get_parameter_value().double_value
        track_width = self.get_parameter('track_width_m').get_parameter_value().double_value
        ticks_per_rev = self.get_parameter('ticks_per_revolution').get_parameter_value().double_value

        m1_sign = self.get_parameter('m1_sign').get_parameter_value().double_value
        m2_sign = self.get_parameter('m2_sign').get_parameter_value().double_value
        m3_sign = self.get_parameter('m3_sign').get_parameter_value().double_value
        m4_sign = self.get_parameter('m4_sign').get_parameter_value().double_value
        stale_timeout = self.get_parameter('stale_timeout_sec').get_parameter_value().double_value
        reset_thresh = self.get_parameter('reset_threshold_ticks').get_parameter_value().integer_value
        disagree_thresh = self.get_parameter('disagreement_threshold_ticks').get_parameter_value().integer_value
        max_speed = self.get_parameter('max_plausible_wheel_speed_mps').get_parameter_value().double_value

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
        self.tf_broadcaster = TransformBroadcaster(self)

        # HTTP session setup
        self.http_session = requests.Session()

        # Connection / Error state
        self.consecutive_errors = 0

        # Timer setup
        timer_period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(timer_period, self._poll_and_publish)

        self.get_logger().info(
            f"rover_encoder_odometry initialized. Polling '{self.telemetry_url}' at {self.publish_rate_hz} Hz. "
            f"Wheel diameter: {wheel_diameter}m, track width: {track_width}m, ticks/rev: {ticks_per_rev} (provisional)."
        )

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
                    encoders_data.get('m1', 0),
                    encoders_data.get('m2', 0),
                    encoders_data.get('m3', 0),
                    encoders_data.get('m4', 0),
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

            # Update kinematics
            success, msg = self.kinematics.update(ticks, sample_time_sec, sequence)
            if not success:
                self._handle_degraded(f"Kinematics update rejected sample: {msg}")
                return

            # Check wheel disagreement warning
            if self.kinematics.disagreement_warning:
                self.get_logger().warn(f"Wheel encoder disagreement: {self.kinematics.disagreement_details}")

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
