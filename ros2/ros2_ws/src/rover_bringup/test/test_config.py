import os
import pytest

def find_file(filename):
    search_paths = [
        os.path.join('/ros2_ws', filename),
        os.path.join(os.path.dirname(__file__), '..', '..', '..', filename),
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', filename),
        os.path.join(os.path.dirname(__file__), filename)
    ]
    for path in search_paths:
        if os.path.exists(path):
            return path
    raise FileNotFoundError(f"Could not find {filename}")

def test_dockerfile_base_image():
    path = find_file('Dockerfile')
    with open(path, 'r') as f:
        content = f.read()
    
    # Assert correct base image
    assert 'FROM ros:jazzy-ros-base-noble' in content

def test_compose_security_and_network():
    path = find_file('compose.yaml')
    with open(path, 'r') as f:
        content = f.read()
    
    # Assert host networking is configured
    assert 'network_mode: host' in content
    
    # Assert no privileged mode is set
    assert 'privileged: true' not in content.lower()
    
    # Assert no /dev mounts or serial-device references
    assert '/dev' not in content
    assert 'ttyUSB' not in content
    assert 'rover-esp32' not in content
    assert 'rover-lidar' not in content
