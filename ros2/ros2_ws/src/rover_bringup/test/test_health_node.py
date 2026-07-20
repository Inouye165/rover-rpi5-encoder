import sys
from unittest.mock import MagicMock, patch
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rover_bringup.rover_system_health import RoverSystemHealthNode
import requests

@pytest.fixture(scope='module', autouse=True)
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

def test_node_initialization():
    node = RoverSystemHealthNode()
    assert node.get_name() == 'rover_system_health'
    
    # Verify no cmd_vel, scan or odom publishers were created
    for p in node.publishers:
        assert p.topic_name != '/cmd_vel'
        assert p.topic_name != '/odom'
        assert p.topic_name != '/scan'
    
    node.destroy_node()

@patch('requests.get')
def test_health_node_handles_unavailable_api(mock_get):
    # Simulate API timeout/failure
    mock_get.side_effect = requests.exceptions.Timeout("Connection timed out")
    
    node = RoverSystemHealthNode()
    node.diag_pub.publish = MagicMock()
    
    node.check_health()
    
    assert node.diag_pub.publish.called
    diag_array = node.diag_pub.publish.call_args[0][0]
    
    status = diag_array.status[0]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "API connection lost"
    
    kv_dict = {kv.key: kv.value for kv in status.values}
    assert kv_dict["cockpit_api_available"] == "False"
    assert kv_dict["lidar_api_available"] == "False"
    
    node.destroy_node()

@patch('requests.get')
def test_health_node_reports_stale_telemetry(mock_get):
    # Mock Cockpit API returning stale telemetry, and LiDAR nominal
    def side_effect(url, timeout=None):
        mock_response = MagicMock()
        mock_response.status_code = 200
        if 'status' in url and '3000' in url:
            mock_response.json.return_value = {
                'serialConnected': True,
                'lastPacketAgeMs': 2500,
                'armed': False
            }
        else:
            mock_response.json.return_value = {
                'connected': True,
                'health': 'Good',
                'scanHz': 7.2
            }
        return mock_response
        
    mock_get.side_effect = side_effect
    
    node = RoverSystemHealthNode()
    node.diag_pub.publish = MagicMock()
    
    node.check_health()
    
    assert node.diag_pub.publish.called
    diag_array = node.diag_pub.publish.call_args[0][0]
    status = diag_array.status[0]
    
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "ESP32 telemetry stale"
    
    kv_dict = {kv.key: kv.value for kv in status.values}
    assert kv_dict["esp32_telemetry_fresh"] == "False"
    assert kv_dict["esp32_last_packet_age_ms"] == "2500.0"
    
    node.destroy_node()

@patch('requests.get')
def test_health_node_reports_disconnected_lidar(mock_get):
    # Mock Cockpit API nominal, LiDAR disconnected
    def side_effect(url, timeout=None):
        mock_response = MagicMock()
        mock_response.status_code = 200
        if 'status' in url and '3000' in url:
            mock_response.json.return_value = {
                'serialConnected': True,
                'lastPacketAgeMs': 100,
                'armed': True
            }
        else:
            mock_response.json.return_value = {
                'connected': False,
                'health': 'Error',
                'scanHz': 0.0
            }
        return mock_response
        
    mock_get.side_effect = side_effect
    
    node = RoverSystemHealthNode()
    node.diag_pub.publish = MagicMock()
    
    node.check_health()
    
    assert node.diag_pub.publish.called
    diag_array = node.diag_pub.publish.call_args[0][0]
    status = diag_array.status[0]
    
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "Hardware disconnected"
    
    kv_dict = {kv.key: kv.value for kv in status.values}
    assert kv_dict["lidar_connected"] == "False"
    
    node.destroy_node()
