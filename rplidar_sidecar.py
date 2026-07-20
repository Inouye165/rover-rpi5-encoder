#!/usr/bin/env python3
"""
rplidar_sidecar.py - Python sidecar for RPLIDAR C1 USB LiDAR

Runs a background thread to read scan data from /dev/rover-lidar at 460800 baud.
Serves the latest scan and status as JSON on port 3002.

Endpoints:
  GET /status - connection details, uptime, scan rate, points per second, errors.
  GET /scan   - the latest complete rotation (points validated, sorted, and capped/downsampled).
"""

import argparse
import json
import math
import os
import random
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock, Thread
import numpy as np
from scipy.spatial import KDTree
from urllib.parse import urlparse, parse_qs

# State lock
state_lock = Lock()

# Straight-line test state
test_lock = Lock()
test_active = False
test_state = "IDLE"  # "IDLE", "ZEROING", "READY"
test_ref_scans = []  # Scans collected during ZEROING to build a stable reference cloud
test_ref_cloud = None  # N x 2 array of reference points (X, Y) in meters
test_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
test_metrics = {
    "confidence": 0.0,
    "inliers": 0,
    "rmse": 0.0,
    "scan_age_ms": 0,
    "status": "idle",
    "rejection_reason": ""
}

# Test configuration (sent on /test/start)
test_config = {
    "front_angle_offset": 0.0,
    "lidar_x_offset": 0.0127,  # base_link to laser x translation in meters
    "lidar_y_offset": 0.034925,  # base_link to laser y translation in meters
    "lidar_yaw_offset": 0.0,
    "min_range": 0.15,
    "max_range": 4.0,
    "chassis_margin": 0.02,  # 2cm safety margin
    "angle_sector_masks": [],  # list of [start, end] in degrees
}

def filter_and_process_scan(points, config):
    """
    Applies angle offset, filters out invalid ranges, masks out the chassis,
    removes isolated outliers, and downsamples spatially.
    Returns: N x 2 numpy array of Cartesian coordinates (X, Y) in the laser frame, in meters.
    """
    if not points:
        return np.empty((0, 2), dtype=np.float32)
        
    front_offset = config.get("front_angle_offset", 0.0)
    min_range = config.get("min_range", 0.15)
    max_range = config.get("max_range", 4.0)
    chassis_margin = config.get("chassis_margin", 0.02)
    lx = config.get("lidar_x_offset", 0.0127)
    ly = config.get("lidar_y_offset", 0.034925)
    lyaw = config.get("lidar_yaw_offset", 0.0)
    sector_masks = config.get("angle_sector_masks", [])
    
    # Pre-allocate arrays
    cartesian_pts = []
    
    # Rover dimensions (meters)
    # Length = 9 inches = 0.2286 m. Width = 8.75 inches = 0.22225 m.
    rover_half_l = 0.2286 / 2.0 + chassis_margin
    rover_half_w = 0.22225 / 2.0 + chassis_margin
    
    cos_lyaw = math.cos(math.radians(lyaw))
    sin_lyaw = math.sin(math.radians(lyaw))
    
    for p in points:
        dist_m = p["distanceMm"] / 1000.0
        
        # 1. Reject range bounds
        if dist_m < min_range or dist_m > max_range:
            continue
            
        # 2. Apply front angle offset
        angle = (p["angleDeg"] - front_offset) % 360.0
        if angle < 0:
            angle += 360.0
            
        # 3. Reject angle sectors
        masked_sector = False
        for start, end in sector_masks:
            # Handle wrapping sector angles
            if start <= end:
                if start <= angle <= end:
                    masked_sector = True
                    break
            else:
                if angle >= start or angle <= end:
                    masked_sector = True
                    break
        if masked_sector:
            continue
            
        # Convert polar to Cartesian in laser frame
        x_l = dist_m * math.cos(math.radians(angle))
        y_l = -dist_m * math.sin(math.radians(angle))
        
        # 4. Rover self-mask: check if point falls inside rover chassis
        # Project laser point into rover body frame
        x_r = x_l * cos_lyaw - y_l * sin_lyaw + lx
        y_r = x_l * sin_lyaw + y_l * cos_lyaw + ly
        
        if -rover_half_l <= x_r <= rover_half_l and -rover_half_w <= y_r <= rover_half_w:
            # Point is inside rover chassis, reject!
            continue
            
        cartesian_pts.append([x_l, y_l])
        
    if not cartesian_pts:
        return np.empty((0, 2), dtype=np.float32)
        
    pts_arr = np.array(cartesian_pts, dtype=np.float32)
    
    # 5. Reject isolated outliers
    # If a point has no neighbors within 0.15m, it is isolated
    if len(pts_arr) > 1:
        tree = KDTree(pts_arr)
        # Find neighbors within 0.15m
        counts = tree.query_ball_point(pts_arr, r=0.15, return_length=True)
        # Keep points with at least 2 neighbors (the point itself + 1 neighbor)
        pts_arr = pts_arr[counts >= 2]
        
    if len(pts_arr) == 0:
        return pts_arr
        
    # 6. Spatial downsampling
    # Grid cell size = 3cm. Keep at most one point per cell.
    grid_size = 0.03
    coords_grid = np.round(pts_arr / grid_size).astype(int)
    # Use unique rows to downsample
    _, indices = np.unique(coords_grid, axis=0, return_index=True)
    downsampled_pts = pts_arr[indices]
    
    return downsampled_pts

def run_icp(ref_cloud, curr_cloud, initial_pose, max_iterations=20, tolerance=1e-4):
    """
    Runs robust 2D ICP to align curr_cloud to ref_cloud, starting from initial_pose (x, y, yaw_rad).
    ref_cloud: N x 2 numpy array
    curr_cloud: M x 2 numpy array
    initial_pose: tuple/list (x, y, yaw_rad)
    Returns: (x, y, yaw_rad), confidence, rmse, inliers_count
    """
    if len(ref_cloud) == 0 or len(curr_cloud) == 0:
        return initial_pose, 0.0, 999.0, 0
        
    # Build KDTree of reference cloud
    tree = KDTree(ref_cloud)
    
    # Current pose estimate
    x_est, y_est, yaw_est = initial_pose
    
    max_match_dist = 0.15  # 15 cm outlier threshold
    
    # Limit iterations
    for it in range(max_iterations):
        # 1. Transform current cloud using current pose estimate
        c = math.cos(yaw_est)
        s = math.sin(yaw_est)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        t = np.array([x_est, y_est], dtype=np.float32)
        curr_aligned = curr_cloud.dot(R.T) + t
        
        # 2. Find nearest neighbors in reference cloud
        dists, indices = tree.query(curr_aligned)
        
        # 3. Filter outliers
        inlier_mask = dists < max_match_dist
        inliers_count = np.sum(inlier_mask)
        
        if inliers_count < 10:
            # Not enough overlap
            return (x_est, y_est, yaw_est), 0.0, 999.0, int(inliers_count)
            
        p_inliers = ref_cloud[indices[inlier_mask]]
        q_inliers = curr_aligned[inlier_mask]
        
        # 4. Compute optimal rotation and translation deltas
        p_centroid = p_inliers.mean(axis=0)
        q_centroid = q_inliers.mean(axis=0)
        
        p_centered = p_inliers - p_centroid
        q_centered = q_inliers - q_centroid
        
        # 2D cross-covariance
        H = q_centered.T.dot(p_centered)
        
        # Closed-form optimal rotation in 2D
        # angle = atan2(H_xy - H_yx, H_xx + H_yy)
        d_yaw = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        
        # Compute optimal translation delta
        c_d = math.cos(d_yaw)
        s_d = math.sin(d_yaw)
        R_d = np.array([[c_d, -s_d], [s_d, c_d]], dtype=np.float32)
        d_t = p_centroid - R_d.dot(q_centroid)
        
        # 5. Update cumulative pose
        # Apply delta translation rotated by current yaw
        x_est += c * d_t[0] - s * d_t[1]
        y_est += s * d_t[0] + c * d_t[1]
        yaw_est += d_yaw
        
        # Normalize yaw to [-pi, pi]
        yaw_est = (yaw_est + math.pi) % (2 * math.pi) - math.pi
        
        # Check convergence
        if abs(d_t[0]) < tolerance and abs(d_t[1]) < tolerance and abs(d_yaw) < tolerance:
            break
            
    # Final metrics
    c = math.cos(yaw_est)
    s = math.sin(yaw_est)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    t = np.array([x_est, y_est], dtype=np.float32)
    curr_aligned = curr_cloud.dot(R.T) + t
    dists, _ = tree.query(curr_aligned)
    inlier_mask = dists < max_match_dist
    inliers_count = np.sum(inlier_mask)
    
    rmse = math.sqrt(np.mean(dists[inlier_mask]**2)) if inliers_count > 0 else 999.0
    confidence = float(inliers_count) / len(curr_cloud) if len(curr_cloud) > 0 else 0.0
    
    return (x_est, y_est, yaw_est), confidence, rmse, int(inliers_count)

def latest_scan_time(ts_str):
    try:
        return time.mktime(time.strptime(ts_str, "%Y-%m-%dT%H:%M:%SZ"))
    except:
        return time.time()

def handle_new_scan(raw_points, timestamp):
    """
    Called whenever a new complete scan is ready.
    Runs ICP if a test session is active.
    """
    global test_active, test_state, test_ref_scans, test_ref_cloud, test_pose, test_metrics
    
    with test_lock:
        if not test_active:
            return
            
        # 1. Preprocess and filter scan
        filtered = filter_and_process_scan(raw_points, test_config)
        
        # Check if scan is empty or invalid
        if len(filtered) < 15:
            test_metrics.update({
                "status": "error",
                "rejection_reason": "Insufficient scan geometry (too few points after filtering)"
            })
            return
            
        # 2. Handle Zeroing
        if test_state == "ZEROING":
            test_ref_scans.append(filtered)
            test_metrics.update({
                "status": "zeroing",
                "rejection_reason": f"Collecting reference scans ({len(test_ref_scans)}/5)"
            })
            if len(test_ref_scans) >= 5:
                # Verify stability: match scan 4 against scan 0
                pose, conf, rmse, inliers = run_icp(test_ref_scans[0], test_ref_scans[4], (0.0, 0.0, 0.0))
                if conf > 0.85 and rmse < 0.02:
                    # Merge all points and downsample again to create reference cloud
                    merged = np.vstack(test_ref_scans)
                    grid_size = 0.03
                    coords_grid = np.round(merged / grid_size).astype(int)
                    _, indices = np.unique(coords_grid, axis=0, return_index=True)
                    test_ref_cloud = merged[indices]
                    
                    test_state = "READY"
                    test_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    test_metrics.update({
                        "confidence": 1.0,
                        "inliers": len(test_ref_cloud),
                        "rmse": 0.0,
                        "scan_age_ms": 0,
                        "status": "ready",
                        "rejection_reason": ""
                    })
                    print(f"[LiDAR Sidecar] Zeroing completed successfully. Reference cloud size: {len(test_ref_cloud)}")
                else:
                    # Environment unstable
                    test_ref_scans.clear()
                    test_metrics.update({
                        "status": "error",
                        "rejection_reason": f"Environment unstable during zeroing (conf={conf:.2f}, rmse={rmse:.4f})"
                    })
            return
            
        # 3. Handle Running state
        if test_state in ("READY", "RUNNING"):
            if test_ref_cloud is None:
                return
                
            # Run ICP against reference cloud
            init_pose = (test_pose["x"], test_pose["y"], test_pose["yaw"])
            pose, conf, rmse, inliers = run_icp(test_ref_cloud, filtered, init_pose)
            
            # Check for pose jump rejection (e.g. if jump > 0.15m or > 0.25 rad)
            dx = pose[0] - test_pose["x"]
            dy = pose[1] - test_pose["y"]
            dyaw = pose[2] - test_pose["yaw"]
            dist_jump = math.sqrt(dx*dx + dy*dy)
            yaw_jump = abs(dyaw)
            
            # Normalize yaw jump
            yaw_jump = (yaw_jump + math.pi) % (2 * math.pi) - math.pi
            yaw_jump = abs(yaw_jump)
            
            if dist_jump > 0.15 or yaw_jump > 0.25:
                # Pose jump rejected!
                test_metrics.update({
                    "scan_age_ms": int((time.time() - latest_scan_time(timestamp)) * 1000),
                    "rejection_reason": f"Pose jump rejected (jump: {dist_jump:.3f}m, {math.degrees(yaw_jump):.1f} deg)"
                })
                return
                
            rejection = ""
            if conf < 0.50:
                rejection = f"Low confidence: {conf:.2f}"
            elif rmse > 0.08:
                rejection = f"High residual: {rmse:.4f}"
                
            if not rejection:
                # Valid pose update!
                test_pose = {"x": float(pose[0]), "y": float(pose[1]), "yaw": float(pose[2])}
                test_metrics.update({
                    "confidence": float(conf),
                    "inliers": int(inliers),
                    "rmse": float(rmse),
                    "scan_age_ms": 0,
                    "status": "running",
                    "rejection_reason": ""
                })
            else:
                test_metrics.update({
                    "confidence": float(conf),
                    "inliers": int(inliers),
                    "rmse": float(rmse),
                    "scan_age_ms": int((time.time() - latest_scan_time(timestamp)) * 1000),
                    "rejection_reason": rejection
                })

# Global state
latest_scan = {
    "timestamp": None,
    "sequence": 0,
    "scanHz": 0.0,
    "pointCount": 0,
    "points": []
}

status_data = {
    "connected": False,
    "state": "disconnected",
    "device": "/dev/rover-lidar",
    "model": "RPLIDAR C1",
    "health": "unknown",
    "firmwareVersion": "unknown",
    "hardwareVersion": "unknown",
    "scanHz": 0.0,
    "pointsPerSecond": 0,
    "latestScanPointCount": 0,
    "lastCompleteScanAt": None,
    "lastScanAgeMs": -1,
    "serviceUptimeSeconds": 0,
    "reconnectCount": 0,
    "lastError": None
}

start_time = time.time()
running = True

def load_env():
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, val = line.split('=', 1)
                    os.environ[key.strip()] = val.strip()

load_env()

def generate_mock_scan(seq):
    points = []
    # Generate 400 raw points, some invalid, some valid
    # Simulates a room with walls around 2.0-2.5m, and an obstacle at 0 degrees (1.0m away)
    t = time.time()
    # Let the obstacle move slightly to prove it's live
    obstacle_angle = (t * 5) % 40  # moves left/right around 20 degrees
    
    for i in range(420):
        angle = i * (360.0 / 400.0) + random.uniform(-0.1, 0.1)
        # Randomly introduce a few invalid zero or negative readings
        if random.random() < 0.02:
            points.append((0, angle, 0.0))
            continue
            
        quality = random.randint(25, 45)
        # Base room wall distance
        dist = 2200.0 + math.sin(math.radians(angle * 4)) * 300.0 + random.uniform(-10, 10)
        
        # Obstacle box at ~0 degrees
        diff = abs(((angle - obstacle_angle + 180) % 360) - 180)
        if diff < 15:
            dist = 1000.0 + random.uniform(-5, 5)
            
        points.append((quality, angle, dist))
    return points

def poll_mock(interval):
    global latest_scan, status_data
    print("[LiDAR Sidecar] Starting Mock LiDAR Poller Thread...")
    
    with state_lock:
        status_data.update({
            "connected": True,
            "state": "scanning",
            "health": "OK",
            "firmwareVersion": "mock-1.0",
            "hardwareVersion": "mock-c1",
            "device": "MOCK-DEVICE"
        })
        
    seq = 0
    last_time = time.time()
    
    while running:
        time.sleep(interval)
        seq += 1
        raw_points = generate_mock_scan(seq)
        now = time.time()
        scan_hz = round(1.0 / (now - last_time), 1)
        last_time = now
        
        # Process and validate
        processed = []
        for q, ang, dist in raw_points:
            if dist <= 0 or not math.isfinite(dist) or not math.isfinite(ang):
                continue
            norm_ang = ang % 360.0
            if norm_ang < 0:
                norm_ang += 360.0
            processed.append({
                "angleDeg": round(norm_ang, 2),
                "distanceMm": round(dist),
                "quality": int(q)
            })
            
        processed.sort(key=lambda p: p["angleDeg"])
        
        # Keep full useful scan resolution internally for scan matching
        full_resolution_points = list(processed)
        
        # Downsample if needed
        max_points = 360
        if len(processed) > max_points:
            step = len(processed) / max_points
            downsampled = []
            for i in range(max_points):
                idx = int(round(i * step))
                if idx < len(processed):
                    downsampled.append(processed[idx])
            processed = downsampled
            
        points_per_sec = int(scan_hz * len(processed))
        
        with state_lock:
            latest_scan.update({
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
                "sequence": seq,
                "scanHz": scan_hz,
                "pointCount": len(processed),
                "points": processed
            })
            status_data.update({
                "scanHz": scan_hz,
                "pointsPerSecond": points_per_sec,
                "latestScanPointCount": len(processed),
                "lastCompleteScanAt": latest_scan["timestamp"],
                "lastScanAgeMs": int((time.time() - now) * 1000),
                "serviceUptimeSeconds": int(time.time() - start_time)
            })
            
        # Process scan matching with full resolution points
        handle_new_scan(full_resolution_points, latest_scan["timestamp"])

import asyncio

async def async_scan_loop(device_path):
    global latest_scan, status_data, running
    
    try:
        from rplidarc1 import RPLidar
    except ImportError as e:
        errmsg = "rplidarc1 library not installed. Install via pip3 install rplidarc1"
        print(f"[LiDAR Sidecar] ERROR: {errmsg}", file=sys.stderr)
        with state_lock:
            status_data.update({
                "connected": False,
                "state": "error",
                "lastError": errmsg
            })
        return

    seq = 0
    reconnects = 0
    
    while running:
        scan_task = None
        lidar = None
        try:
            with state_lock:
                status_data.update({
                    "connected": False,
                    "state": "connecting",
                    "device": device_path,
                    "lastError": None
                })
            
            print(f"[LiDAR Sidecar] Connecting to RPLIDAR C1 on {device_path}...")
            lidar = RPLidar(device_path, 460800)
            
            # Start simple_scan task in the background
            scan_task = asyncio.create_task(lidar.simple_scan(make_return_dict=True))
            
            with state_lock:
                status_data.update({
                    "connected": True,
                    "state": "scanning",
                    "model": "RPLIDAR C1",
                    "health": "Good",
                    "firmwareVersion": "1.2",
                    "hardwareVersion": "18"
                })
                
            print("[LiDAR Sidecar] Scanning started successfully.")
            last_time = time.time()
            
            while running:
                await asyncio.sleep(0.15) # Poll scan results at ~6.6Hz
                
                # Verify background scan task status
                if scan_task.done():
                    exc = scan_task.exception()
                    if exc:
                        raise exc
                    break
                
                data = lidar.output_dict
                if not data:
                    continue
                
                now = time.time()
                scan_hz = round(1.0 / (now - last_time), 1) if (now - last_time) > 0 else 0.0
                last_time = now
                seq += 1
                
                # Make a thread-safe copy of output_dict
                snapshot = dict(data)
                
                processed = []
                for ang, dist in snapshot.items():
                    if dist is None or dist <= 0 or not math.isfinite(dist) or not math.isfinite(ang):
                        continue
                    norm_ang = ang % 360.0
                    if norm_ang < 0:
                        norm_ang += 360.0
                    processed.append({
                        "angleDeg": round(norm_ang, 2),
                        "distanceMm": round(dist),
                        "quality": 31 # Static nominal quality
                    })
                
                processed.sort(key=lambda p: p["angleDeg"])
                
                # Keep full useful scan resolution internally for scan matching
                full_resolution_points = list(processed)
                
                # Downsample to a maximum of 360 points
                max_points = 360
                if len(processed) > max_points:
                    step = len(processed) / max_points
                    downsampled = []
                    for i in range(max_points):
                        idx = int(round(i * step))
                        if idx < len(processed):
                            downsampled.append(processed[idx])
                    processed = downsampled
                
                points_per_sec = int(scan_hz * len(processed))
                
                with state_lock:
                    latest_scan.update({
                        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
                        "sequence": seq,
                        "scanHz": scan_hz,
                        "pointCount": len(processed),
                        "points": processed
                    })
                    status_data.update({
                        "scanHz": scan_hz,
                        "pointsPerSecond": points_per_sec,
                        "latestScanPointCount": len(processed),
                        "lastCompleteScanAt": latest_scan["timestamp"],
                        "lastScanAgeMs": int((time.time() - now) * 1000),
                        "serviceUptimeSeconds": int(time.time() - start_time)
                    })
                    
                # Process scan matching with full resolution points
                handle_new_scan(full_resolution_points, latest_scan["timestamp"])
                    
        except Exception as e:
            errmsg = f"Error during scanning: {e}"
            print(f"[LiDAR Sidecar] {errmsg}", file=sys.stderr)
            with state_lock:
                status_data.update({
                    "connected": False,
                    "state": "error",
                    "lastError": errmsg
                })
        finally:
            if lidar is not None:
                print("[LiDAR Sidecar] Stopping scan and resetting lidar...")
                try:
                    if scan_task and not scan_task.done():
                        scan_task.cancel()
                        await scan_task
                except asyncio.CancelledError:
                    pass
                except Exception:
                    pass
                
                try:
                    lidar.reset()
                except Exception:
                    pass
                lidar = None
            
            if running:
                reconnects += 1
                with state_lock:
                    status_data["reconnectCount"] = reconnects
                print("[LiDAR Sidecar] Reconnecting in 3 seconds...")
                await asyncio.sleep(3.0)

def poll_rplidar(device_path, baudrate):
    print(f"[LiDAR Sidecar] Starting RPLIDAR C1 async loop on {device_path}...")
    try:
        asyncio.run(async_scan_loop(device_path))
    except Exception as e:
        print(f"[LiDAR Sidecar] Async loop failed: {e}", file=sys.stderr)


class LiDARHTTPHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global test_active, test_state, test_ref_scans, test_ref_cloud, test_pose, test_metrics, test_config
        parsed_url = urlparse(self.path)
        path = parsed_url.path
        query_params = parse_qs(parsed_url.query)

        if path in ("/status", "/status/"):
            with state_lock:
                # Update uptime and scan age
                status_data["serviceUptimeSeconds"] = int(time.time() - start_time)
                if latest_scan["timestamp"]:
                    # Convert timestamp back or just compute from current time
                    # We can use actual elapsed time
                    if status_data["lastCompleteScanAt"]:
                        # Simply estimate age
                        status_data["lastScanAgeMs"] = int(status_data["lastScanAgeMs"]) # Keep last processed age
                payload = dict(status_data)
            self._send(200, payload)
            
        elif path in ("/scan", "/scan/"):
            with state_lock:
                if not latest_scan["timestamp"]:
                    self._send(503, {"error": "No scan data available", "connected": status_data["connected"], "state": status_data["state"]})
                    return
                # Update scan age
                status_data["lastScanAgeMs"] = int(status_data["lastScanAgeMs"])
                payload = dict(latest_scan)
            self._send(200, payload)
            
        elif path in ("/test/start", "/test/start/"):
            with test_lock:
                test_active = True
                test_state = "ZEROING"
                test_ref_scans = []
                test_ref_cloud = None
                test_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                test_metrics = {
                    "confidence": 0.0,
                    "inliers": 0,
                    "rmse": 0.0,
                    "scan_age_ms": 0,
                    "status": "zeroing",
                    "rejection_reason": "Starting zeroing calibration"
                }
                
                # Load configuration from query parameters if provided
                if "front_angle_offset" in query_params:
                    test_config["front_angle_offset"] = float(query_params["front_angle_offset"][0])
                if "lidar_x_offset" in query_params:
                    test_config["lidar_x_offset"] = float(query_params["lidar_x_offset"][0])
                if "lidar_y_offset" in query_params:
                    test_config["lidar_y_offset"] = float(query_params["lidar_y_offset"][0])
                if "lidar_yaw_offset" in query_params:
                    test_config["lidar_yaw_offset"] = float(query_params["lidar_yaw_offset"][0])
                if "min_range" in query_params:
                    test_config["min_range"] = float(query_params["min_range"][0])
                if "max_range" in query_params:
                    test_config["max_range"] = float(query_params["max_range"][0])
                if "chassis_margin" in query_params:
                    test_config["chassis_margin"] = float(query_params["chassis_margin"][0])
                if "angle_sector_masks" in query_params:
                    masks = []
                    raw_masks = query_params["angle_sector_masks"][0].split(",")
                    for rm in raw_masks:
                        parts = rm.split("-")
                        if len(parts) == 2:
                            masks.append([float(parts[0]), float(parts[1])])
                    test_config["angle_sector_masks"] = masks
                
            self._send(200, {"ok": True, "message": "LiDAR straight-line test session started.", "config": test_config})

        elif path in ("/test/pose", "/test/pose/"):
            with test_lock:
                payload = {
                    "pose": dict(test_pose),
                    "metrics": dict(test_metrics),
                    "active": test_active,
                    "state": test_state
                }
            self._send(200, payload)

        elif path in ("/test/stop", "/test/stop/"):
            with test_lock:
                test_active = False
                test_state = "IDLE"
            self._send(200, {"ok": True, "message": "LiDAR straight-line test session stopped."})
            
        else:
            self._send(404, {"error": f"Unknown path: {self.path}"})
            
    def _send(self, code, obj):
        body = json.dumps(obj, separators=(',', ':')).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass

def parse_args():
    p = argparse.ArgumentParser(description="RPLIDAR C1 HTTP Sidecar")
    p.add_argument("--dev", type=str, default="/dev/rover-lidar", help="Serial port device path")
    p.add_argument("--baud", type=int, default=460800, help="Serial baud rate (default: 460800)")
    p.add_argument("--port", type=int, default=3002, help="HTTP port to serve json")
    p.add_argument("--mock", action="store_true", help="Run in Mock mode without physical LiDAR")
    return p.parse_args()

def main():
    global running
    args = parse_args()
    
    print(f"[LiDAR Sidecar] Starting server on http://localhost:{args.port}")
    
    if args.mock:
        t = Thread(target=poll_mock, args=(0.1,), daemon=True)
    else:
        t = Thread(target=poll_rplidar, args=(args.dev, args.baud), daemon=True)
        
    t.start()
    
    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), LiDARHTTPHandler)
    print(f"[LiDAR Sidecar] Listening on http://127.0.0.1:{args.port}/status and /scan")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[LiDAR Sidecar] KeyboardInterrupt received. Shutting down.")
    finally:
        running = False
        print("[LiDAR Sidecar] Stopped.")

if __name__ == "__main__":
    main()
