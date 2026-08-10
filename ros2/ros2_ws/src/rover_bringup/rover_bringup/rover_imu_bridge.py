#!/usr/bin/env python3
# ==============================================================================
# rover_imu_bridge.py - ROS 2 BNO08x Streaming IMU Bridge Node
# ==============================================================================
#
# Connects to the host rover-server WebSocket stream (ws://127.0.0.1:3000) and
# streams 50 Hz production BNO08x IMU telemetry frames (0x3A).
# Publishes sensor_msgs/msg/Imu on /imu/data.
#
# REP-145 Compliant: linear_acceleration incorporates gravity (SH2_ACCELEROMETER).
# Frame ID: imu_link.
# Covariances: all zero matrices (provided with unknown covariance).
# Absolute orientation is NOT fused into robot_localization initially.
# ==============================================================================

import base64
import json
import math
import os
import socket
import struct
import threading
import time

from sensor_msgs.msg import Imu
import rclpy
from rclpy.node import Node


class RoverImuBridge(Node):
    """
    ROS 2 node streaming BNO08x telemetry from rover-server WebSocket
    and publishing sensor_msgs/msg/Imu on /imu/data.
    """

    def __init__(self):
        super().__init__('rover_imu_bridge')

        host_server_url = os.environ.get('ROVER_SERVER_URL', 'http://127.0.0.1:3000')
        # Deriving WebSocket host/port from ROVER_SERVER_URL
        ws_url_default = host_server_url.replace('http://', 'ws://').replace('https://', 'wss://')

        self.declare_parameter('ws_url', ws_url_default)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('stale_timeout_sec', 0.20)

        self.ws_url = self.get_parameter('ws_url').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.stale_timeout_sec = self.get_parameter('stale_timeout_sec').get_parameter_value().double_value

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Internal state & diagnostic counters
        self.last_seq = None
        self.sequence_gaps = 0
        self.last_esp_timestamp_us = 0
        self.last_recv_time = 0.0
        self.imu_sample_count = 0
        self.running = True

        # Start WebSocket client background thread
        self.ws_thread = threading.Thread(target=self._ws_client_loop, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(
            f"rover_imu_bridge initialized. Streaming from '{self.ws_url}'. "
            f"Publishing /imu/data with frame_id '{self.imu_frame}' (REP-145 compliant)."
        )

    def stop(self):
        self.running = False

    def _ws_client_loop(self):
        """Background thread connecting to WebSocket server with automatic reconnect backoff."""
        # Extract host and port
        url_clean = self.ws_url.replace('ws://', '').replace('wss://', '').split('/')[0]
        if ':' in url_clean:
            host, port_str = url_clean.split(':')
            port = int(port_str)
        else:
            host = url_clean
            port = 3000

        retry_delay = 2.0

        while self.running and rclpy.ok():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((host, port))

                # Handshake
                sec_key = base64.b64encode(os.urandom(16)).decode('utf-8')
                req = (
                    f"GET / HTTP/1.1\r\n"
                    f"Host: {host}:{port}\r\n"
                    f"Upgrade: websocket\r\n"
                    f"Connection: Upgrade\r\n"
                    f"Sec-WebSocket-Key: {sec_key}\r\n"
                    f"Sec-WebSocket-Version: 13\r\n\r\n"
                )
                sock.sendall(req.encode('utf-8'))

                # Read handshake response
                resp = b""
                while b"\r\n\r\n" not in resp:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    resp += chunk

                if b"101" not in resp:
                    sock.close()
                    time.sleep(retry_delay)
                    continue

                sock.settimeout(2.0)
                retry_delay = 2.0  # Reset delay on success

                # Frame processing loop
                buf = b""
                while self.running and rclpy.ok():
                    try:
                        chunk = sock.recv(4096)
                        if not chunk:
                            break
                        buf += chunk

                        while len(buf) >= 2:
                            fin_opcode = buf[0]
                            opcode = fin_opcode & 0x0F
                            payload_len = buf[1] & 0x7F
                            offset = 2

                            if payload_len == 126:
                                if len(buf) < 4:
                                    break
                                payload_len = struct.unpack("!H", buf[2:4])[0]
                                offset = 4
                            elif payload_len == 127:
                                if len(buf) < 10:
                                    break
                                payload_len = struct.unpack("!Q", buf[2:10])[0]
                                offset = 10

                            total_frame_len = offset + payload_len
                            if len(buf) < total_frame_len:
                                break

                            frame_payload = buf[offset:total_frame_len]
                            buf = buf[total_frame_len:]

                            if opcode == 0x01:  # Text frame
                                try:
                                    msg_json = json.loads(frame_payload.decode('utf-8'))
                                    if msg_json.get('type') == 'bno08x_imu':
                                        self._process_imu_sample(msg_json)
                                except Exception:
                                    pass
                            elif opcode == 0x09:  # Ping frame
                                try:
                                    sock.sendall(b"\x8a\x00")  # Send Pong frame
                                except Exception:
                                    pass
                            elif opcode == 0x0A:  # Pong frame
                                pass
                            elif opcode == 0x08:  # Close frame
                                sock.close()
                                break
                    except socket.timeout:
                        continue
            except Exception:
                pass
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass
            time.sleep(retry_delay)

    def _process_imu_sample(self, data: dict):
        """Processes incoming production 0x3A BNO08x IMU telemetry sample."""
        now_ts = time.time()
        self.last_recv_time = now_ts
        self.imu_sample_count += 1

        seq = data.get('sequence', 0)
        flags = data.get('flags', 0)

        # Sequence gap detection (uint32 rollover aware)
        if self.last_seq is not None:
            expected_seq = (self.last_seq + 1) & 0xFFFFFFFF
            if seq != expected_seq:
                gap = (seq - expected_seq) & 0xFFFFFFFF
                self.sequence_gaps += gap
        self.last_seq = seq

        # Decode status bits
        hw_init = (flags & (1 << 0)) != 0
        in_reset = (flags & (1 << 1)) != 0
        rot_valid = (flags & (1 << 2)) != 0
        gyro_valid = (flags & (1 << 3)) != 0
        accel_valid = (flags & (1 << 4)) != 0

        # Enforce locked v1 ROS publication policy:
        # Publish ONLY if hardware is initialized, not in reset recovery, and ALL 3 reports are valid/fresh.
        if not (hw_init and not in_reset and rot_valid and gyro_valid and accel_valid):
            return

        # Orientation (quaternion w, x, y, z)
        orient = data.get('orientation', {})
        qw = float(orient.get('w', 1.0))
        qx = float(orient.get('x', 0.0))
        qy = float(orient.get('y', 0.0))
        qz = float(orient.get('z', 0.0))

        if not all(math.isfinite(v) for v in (qw, qx, qy, qz)):
            return

        # Gyroscope (angular velocity rad/s)
        gyro = data.get('gyro', {})
        gx = float(gyro.get('x', 0.0))
        gy = float(gyro.get('y', 0.0))
        gz = float(gyro.get('z', 0.0))

        if not all(math.isfinite(v) for v in (gx, gy, gz)):
            return

        # Accelerometer (linear acceleration m/s^2, gravity INCLUDED per REP-145)
        accel = data.get('accel', {})
        ax = float(accel.get('x', 0.0))
        ay = float(accel.get('y', 0.0))
        az = float(accel.get('z', 0.0))

        if not all(math.isfinite(v) for v in (ax, ay, az)):
            return

        # Construct sensor_msgs/msg/Imu
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Covariance matrices: set to all zeros (provided with unknown covariance)
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
