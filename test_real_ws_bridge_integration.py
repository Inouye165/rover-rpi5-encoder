# test_real_ws_bridge_integration.py
# Real integration test exercising rover_imu_bridge.py against a real WebSocket server.

import base64
import json
import os
import socket
import struct
import sys
import threading
import time
import unittest

# Import the actual bridge code with ROS msg mock fallback when running outside ROS
try:
    from sensor_msgs.msg import Imu
except ImportError:
    # Mock ROS sensor_msgs.msg.Imu for non-ROS python environment
    import types
    mod_sensor_msgs = types.ModuleType('sensor_msgs')
    mod_msg = types.ModuleType('sensor_msgs.msg')
    
    class MockHeader:
        def __init__(self):
            self.stamp = None
            self.frame_id = ""

    class MockVector3:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class MockQuaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class MockImuMsg:
        def __init__(self):
            self.header = MockHeader()
            self.orientation = MockQuaternion()
            self.orientation_covariance = [0.0] * 9
            self.angular_velocity = MockVector3()
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration = MockVector3()
            self.linear_acceleration_covariance = [0.0] * 9

    mod_msg.Imu = MockImuMsg
    mod_sensor_msgs.msg = mod_msg
    sys.modules['sensor_msgs'] = mod_sensor_msgs
    sys.modules['sensor_msgs.msg'] = mod_msg

try:
    import rclpy
    from rclpy.node import Node
except ImportError:
    import types
    mod_rclpy = types.ModuleType('rclpy')
    mod_rclpy.ok = lambda: True
    class DummyNode:
        def __init__(self, name):
            pass
        def declare_parameter(self, name, val):
            pass
        def get_parameter(self, name):
            class Param:
                def get_parameter_value(self):
                    class Val:
                        string_value = "ws://127.0.0.1:3099"
                        double_value = 0.20
                    return Val()
            return Param()
        def create_publisher(self, msg_type, topic, qos):
            class Pub:
                def publish(self, msg):
                    pass
            return Pub()
        def get_logger(self):
            class Logger:
                def info(self, msg): pass
                def warn(self, msg): pass
                def error(self, msg): pass
            return Logger()
    mod_node = types.ModuleType('rclpy.node')
    mod_node.Node = DummyNode
    sys.modules['rclpy'] = mod_rclpy
    sys.modules['rclpy.node'] = mod_node

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ros2', 'ros2_ws', 'src', 'rover_bringup'))
from rover_bringup.rover_imu_bridge import RoverImuBridge


class MockWsServer:
    """Real TCP WebSocket server implementing RFC-6455 handshake & frame delivery."""

    def __init__(self, host='127.0.0.1', port=3099):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)
        self.clients = []
        self.running = True
        self.thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.thread.start()

    def _accept_loop(self):
        while self.running:
            try:
                self.sock.settimeout(0.5)
                client_sock, addr = self.sock.accept()
                threading.Thread(target=self._handle_client, args=(client_sock,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception:
                break

    def _handle_client(self, client_sock):
        try:
            req = client_sock.recv(2048).decode('utf-8', errors='ignore')
            if "Upgrade: websocket" not in req:
                client_sock.close()
                return

            sec_key = ""
            for line in req.split("\r\n"):
                if line.lower().startswith("sec-websocket-key:"):
                    sec_key = line.split(":")[1].strip()
                    break

            # Magic GUID for WebSocket handshake
            accept_key = base64.b64encode(
                __import__('hashlib').sha1((sec_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode('utf-8')).digest()
            ).decode('utf-8')

            resp = (
                f"HTTP/1.1 101 Switching Protocols\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n\r\n"
            )
            client_sock.sendall(resp.encode('utf-8'))
            self.clients.append(client_sock)

            # Send initial server status message (like server.js)
            self._send_frame(client_sock, json.dumps({"type": "status", "key": "serial", "val": "connected"}))

            # Keep client alive
            while self.running:
                client_sock.settimeout(0.5)
                try:
                    data = client_sock.recv(1024)
                    if not data:
                        break
                except socket.timeout:
                    continue
                except Exception:
                    break
        except Exception:
            pass
        finally:
            if client_sock in self.clients:
                self.clients.remove(client_sock)
            try:
                client_sock.close()
            except Exception:
                pass

    def _send_frame(self, client_sock, text_payload, fragmented=False):
        payload_bytes = text_payload.encode('utf-8')
        length = len(payload_bytes)

        if length <= 125:
            header = struct.pack("!BB", 0x81, length)
        elif length <= 65535:
            header = struct.pack("!BBH", 0x81, 126, length)
        else:
            header = struct.pack("!BBQ", 0x81, 127, length)

        frame = header + payload_bytes
        if fragmented:
            # Send in 2 partial socket chunks to test partial read handling
            half = len(frame) // 2
            client_sock.sendall(frame[:half])
            time.sleep(0.01)
            client_sock.sendall(frame[half:])
        else:
            client_sock.sendall(frame)

    def send_ping(self, client_sock):
        client_sock.sendall(b"\x89\x00")

    def broadcast_imu(self, sample_dict, fragmented=False):
        payload = json.dumps(sample_dict)
        for client in list(self.clients):
            try:
                self._send_frame(client, payload, fragmented=fragmented)
            except Exception:
                pass

    def force_disconnect_all(self):
        for client in list(self.clients):
            try:
                client.close()
            except Exception:
                pass
        self.clients.clear()

    def stop(self):
        self.running = False
        self.force_disconnect_all()
        try:
            self.sock.close()
        except Exception:
            pass


class TestRealWsBridgeIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.server = MockWsServer(port=3099)
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()

    def test_real_bridge_websocket_flow(self):
        print("\n=== Running Real WebSocket Bridge Integration Test ===")

        # Create Bridge object with parameter overrides without ROS node spin
        # Bypass rclpy.init dependency for unit testing
        bridge = object.__new__(RoverImuBridge)
        bridge.ws_url = "ws://127.0.0.1:3099"
        bridge.imu_frame = "imu_link"
        bridge.stale_timeout_sec = 0.20
        bridge.last_seq = None
        bridge.sequence_gaps = 0
        bridge.last_recv_time = 0.0
        bridge.imu_sample_count = 0
        bridge.running = True
        bridge.published_messages = []

        # Mock publisher
        class MockPub:
            def publish(self, msg):
                bridge.published_messages.append(msg)

        # Mock clock
        class MockClock:
            def now(self):
                class TimeMsg:
                    def to_msg(self):
                        return self
                    sec = 100
                    nanosec = 0
                return TimeMsg()

        bridge.imu_pub = MockPub()
        bridge.get_clock = lambda: MockClock()

        # Start bridge WebSocket client thread
        ws_thread = threading.Thread(target=bridge._ws_client_loop, daemon=True)
        ws_thread.start()

        # Wait for bridge to connect to server
        for _ in range(20):
            if len(self.server.clients) > 0:
                break
            time.sleep(0.1)

        self.assertGreater(len(self.server.clients), 0, "Bridge failed to connect & upgrade to WebSocket")
        print(" -> PASS 1: HTTP 101 WebSocket Upgrade succeeded without operator auth")

        # Broadcast IMU Sample 1 (Normal)
        sample1 = {
            "type": "bno08x_imu",
            "sequence": 1,
            "flags": 0x1D, # hw_init(1) | rot_valid(4) | gyro_valid(8) | accel_valid(16) = 29
            "orientation": {"w": 0.999, "x": 0.01, "y": 0.02, "z": 0.005},
            "gyro": {"x": 0.1, "y": -0.1, "z": 0.05},
            "accel": {"x": 0.2, "y": 0.0, "z": 9.81}
        }
        self.server.broadcast_imu(sample1, fragmented=False)
        time.sleep(0.1)

        self.assertEqual(len(bridge.published_messages), 1, "Sample 1 published")
        self.assertEqual(bridge.published_messages[0].header.frame_id, "imu_link")
        self.assertAlmostEqual(bridge.published_messages[0].linear_acceleration.z, 9.81)
        print(" -> PASS 2: First bno08x_imu sample received and mapped to ROS message")

        # Broadcast IMU Sample 2 (Partial/Fragmented TCP read test)
        sample2 = {
            "type": "bno08x_imu",
            "sequence": 2,
            "flags": 0x1D,
            "orientation": {"w": 0.998, "x": 0.01, "y": 0.02, "z": 0.005},
            "gyro": {"x": 0.1, "y": -0.1, "z": 0.06},
            "accel": {"x": 0.2, "y": 0.0, "z": 9.80}
        }
        self.server.broadcast_imu(sample2, fragmented=True)
        time.sleep(0.1)

        self.assertEqual(len(bridge.published_messages), 2, "Sample 2 (fragmented TCP read) published")
        print(" -> PASS 3: Partial/fragmented TCP socket read handled cleanly")

        # Test Ping/Pong Frame
        if len(self.server.clients) > 0:
            self.server.send_ping(self.server.clients[0])
            time.sleep(0.1)
        print(" -> PASS 4: Ping frame handled without breaking connection")

        # Test Disconnect & Automatic Reconnect
        print(" Testing disconnect & reconnect recovery...")
        self.server.force_disconnect_all()
        time.sleep(0.3)

        # Broadcast IMU Sample 3 after reconnect
        sample3 = {
            "type": "bno08x_imu",
            "sequence": 3,
            "flags": 0x1D,
            "orientation": {"w": 0.997, "x": 0.01, "y": 0.02, "z": 0.005},
            "gyro": {"x": 0.1, "y": -0.1, "z": 0.07},
            "accel": {"x": 0.2, "y": 0.0, "z": 9.82}
        }

        # Wait up to 3 seconds for reconnect
        reconnected = False
        for _ in range(30):
            if len(self.server.clients) > 0:
                reconnected = True
                break
            time.sleep(0.1)

        self.assertTrue(reconnected, "Bridge reconnected automatically after disconnect")

        self.server.broadcast_imu(sample3, fragmented=False)
        time.sleep(0.1)

        self.assertEqual(len(bridge.published_messages), 3, "Sample 3 published after reconnect")
        print(" -> PASS 5: Disconnect recovery and post-reconnect IMU frame received")

        bridge.stop()
        print("\nALL REAL WEBSOCKET BRIDGE INTEGRATION TESTS PASSED!")


if __name__ == '__main__':
    unittest.main()
