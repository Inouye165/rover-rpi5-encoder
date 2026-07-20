#!/usr/bin/env python3
# ==============================================================================
# rover_system_health.py - ROS 2 Diagnostic Health Node
# ==============================================================================

import os
import rclpy
from rclpy.node import Node
import requests
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class RoverSystemHealthNode(Node):
    def __init__(self):
        super().__init__('rover_system_health')
        
        # Declare parameters with fallbacks to environment variables
        default_server = os.environ.get('ROVER_SERVER_URL', 'http://127.0.0.1:3000')
        default_lidar = os.environ.get('ROVER_LIDAR_URL', 'http://127.0.0.1:3002')
        
        self.declare_parameter('server_url', default_server)
        self.declare_parameter('lidar_url', default_lidar)
        self.declare_parameter('check_interval', 1.0)
        
        self.server_url = self.get_parameter('server_url').value
        self.lidar_url = self.get_parameter('lidar_url').value
        self.check_interval = self.get_parameter('check_interval').value
        
        # Publish standard ROS diagnostics
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Period timer
        self.timer = self.create_timer(self.check_interval, self.check_health)
        
        self.get_logger().info(
            f"Rover Health Node initialized.\n"
            f"  Cockpit Server API: {self.server_url}\n"
            f"  LiDAR Sidecar API: {self.lidar_url}\n"
            f"  Interval: {self.check_interval}s"
        )

    def check_health(self):
        # Default nominal states
        cockpit_api_available = False
        lidar_api_available = False
        
        esp32_connected = False
        esp32_telemetry_fresh = False
        esp32_last_packet_age_ms = -1.0
        esp32_armed = False
        
        lidar_connected = False
        lidar_health = "Unknown"
        lidar_scan_hz = 0.0

        # 1. Query Cockpit / ESP32 status
        try:
            r = requests.get(f"{self.server_url}/api/status", timeout=0.5)
            if r.status_code == 200:
                data = r.json()
                cockpit_api_available = True
                esp32_connected = data.get('serialConnected', False)
                age = data.get('lastPacketAgeMs')
                if age is not None:
                    esp32_last_packet_age_ms = float(age)
                    esp32_telemetry_fresh = esp32_last_packet_age_ms < 2000.0
                esp32_armed = data.get('armed', False)
            else:
                self.get_logger().warn(f"Cockpit API returned HTTP status code {r.status_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to query Cockpit API status: {str(e)}")

        # 2. Query LiDAR status
        try:
            r = requests.get(f"{self.lidar_url}/status", timeout=0.5)
            if r.status_code == 200:
                data = r.json()
                lidar_api_available = True
                lidar_connected = data.get('connected', False)
                lidar_health = data.get('health', 'Unknown')
                lidar_scan_hz = float(data.get('scanHz', 0.0))
            else:
                self.get_logger().warn(f"LiDAR Sidecar API returned HTTP status code {r.status_code}")
        except Exception as e:
            self.get_logger().error(f"Failed to query LiDAR API status: {str(e)}")

        # 3. Compute overall status level & message
        level = DiagnosticStatus.OK
        msg = "All systems nominal"

        if not cockpit_api_available or not lidar_api_available:
            level = DiagnosticStatus.ERROR
            msg = "API connection lost"
        elif not esp32_connected or not lidar_connected:
            level = DiagnosticStatus.ERROR
            msg = "Hardware disconnected"
        elif not esp32_telemetry_fresh:
            level = DiagnosticStatus.ERROR
            msg = "ESP32 telemetry stale"
        elif lidar_health not in ("Good", "OK"):
            level = DiagnosticStatus.ERROR
            msg = "LiDAR health error"

        # 4. Construct DiagnosticArray message
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'rover_system_health'
        status.level = level
        status.message = msg
        status.hardware_id = 'rover_rpi5'
        
        status.values = [
            KeyValue(key="cockpit_api_available", value=str(cockpit_api_available)),
            KeyValue(key="lidar_api_available", value=str(lidar_api_available)),
            KeyValue(key="esp32_connected", value=str(esp32_connected)),
            KeyValue(key="esp32_telemetry_fresh", value=str(esp32_telemetry_fresh)),
            KeyValue(key="esp32_last_packet_age_ms", value=f"{esp32_last_packet_age_ms:.1f}"),
            KeyValue(key="esp32_armed", value=str(esp32_armed)),
            KeyValue(key="lidar_connected", value=str(lidar_connected)),
            KeyValue(key="lidar_health", value=str(lidar_health)),
            KeyValue(key="lidar_scan_hz", value=f"{lidar_scan_hz:.2f}"),
        ]
        
        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    node = RoverSystemHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
