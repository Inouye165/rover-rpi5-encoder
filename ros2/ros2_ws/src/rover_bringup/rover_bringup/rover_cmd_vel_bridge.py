#!/usr/bin/env python3
# ==============================================================================
# rover_cmd_vel_bridge.py - Hardened ROS 2 /cmd_vel Motor Command Bridge Node
# ==============================================================================

import os
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests


class RoverCmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('rover_cmd_vel_bridge')

        default_internal_url = os.environ.get(
            'ROVER_INTERNAL_CMD_URL',
            f"http://{os.environ.get('ROVER_INTERNAL_CMD_HOST', '127.0.0.1')}:{os.environ.get('ROVER_INTERNAL_CMD_PORT', '3010')}/api/cmd_vel"
        )
        self.declare_parameter('cmd_endpoint', default_internal_url)
        self.declare_parameter('connect_timeout', 0.2)
        self.declare_parameter('read_timeout', 0.2)

        self.cmd_endpoint = self.get_parameter('cmd_endpoint').value
        self.connect_timeout = self.get_parameter('connect_timeout').value
        self.read_timeout = self.get_parameter('read_timeout').value
        self.timeout = (self.connect_timeout, self.read_timeout)

        # Bridge Token authentication (never log or expose token value)
        self.bridge_token = os.environ.get('ROVER_CMD_VEL_TOKEN', '')

        self.session = requests.Session()
        if self.bridge_token:
            self.session.headers.update({'X-Rover-Bridge-Token': self.bridge_token})

        # Log throttling state to prevent log floods on repeated errors
        self.last_log_times = {}
        self.log_throttle_interval = 2.0  # seconds

        # Status tracking
        self.bridge_status = 'connected'

        # Subscribe strictly to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            f"Rover /cmd_vel Bridge Node initialized.\n"
            f"  Subscribed to: /cmd_vel\n"
            f"  Internal Command Endpoint: {self.cmd_endpoint}\n"
            f"  Timeouts: connect={self.connect_timeout}s, read={self.read_timeout}s\n"
            f"  Token Configured: {'YES' if self.bridge_token else 'NO'}"
        )

    def _should_log(self, key: str) -> bool:
        now = time.time()
        last = self.last_log_times.get(key, 0.0)
        if (now - last) >= self.log_throttle_interval:
            self.last_log_times[key] = now
            return True
        return False

    def cmd_vel_callback(self, msg: Twist):
        lin_x = msg.linear.x
        lin_y = msg.linear.y
        lin_z = msg.linear.z
        ang_x = msg.angular.x
        ang_y = msg.angular.y
        ang_z = msg.angular.z

        # 1. Validate numeric sanity (reject NaN / Inf)
        all_vals = [lin_x, lin_y, lin_z, ang_x, ang_y, ang_z]
        if any(math.isnan(v) or math.isinf(v) for v in all_vals):
            if self._should_log('nan_inf'):
                self.get_logger().error(f"Rejected /cmd_vel with NaN or Inf: lin=({lin_x},{lin_y},{lin_z}), ang=({ang_x},{ang_y},{ang_z})")
            self.bridge_status = 'rejected'
            return

        # 2. Reject unsupported non-zero axes (y/z for linear, x/y for angular)
        if abs(lin_y) > 1e-4 or abs(lin_z) > 1e-4 or abs(ang_x) > 1e-4 or abs(ang_y) > 1e-4:
            if self._should_log('unsupported_axis'):
                self.get_logger().warn(f"Rejected /cmd_vel requesting unsupported axes: lin=({lin_x},{lin_y},{lin_z}), ang=({ang_x},{ang_y},{ang_z})")
            self.bridge_status = 'rejected'
            return

        payload = {
            "linear": {"x": float(lin_x), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(ang_z)}
        }

        try:
            r = self.session.post(self.cmd_endpoint, json=payload, timeout=self.timeout)
            if r.status_code == 200:
                resp_json = r.json() if r.content else {}
                server_state = resp_json.get('state', 'ACTIVE')
                self.bridge_status = server_state.lower()
            else:
                log_key = f"http_{r.status_code}"
                if self._should_log(log_key):
                    self.get_logger().warn(f"Cockpit API rejected /cmd_vel (HTTP {r.status_code}): {r.text[:120]}")
                if r.status_code in (401, 403):
                    self.bridge_status = 'autonomy_disabled'
                else:
                    self.bridge_status = 'rejected'
        except Exception as e:
            if self._should_log('connection_error'):
                self.get_logger().error(f"Failed to post /cmd_vel to Cockpit API: {str(e)}")
            self.bridge_status = 'fault'

    def send_stop(self):
        """Send a best-effort zero command on orderly shutdown."""
        try:
            payload = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
            self.session.post(self.cmd_endpoint, json=payload, timeout=(0.2, 0.5))
            self.get_logger().info("Sent best-effort safe zero command on node shutdown.")
        except Exception as e:
            self.get_logger().warn(f"Could not send stop command on shutdown: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = RoverCmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
