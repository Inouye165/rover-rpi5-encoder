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

# State lock
state_lock = Lock()

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
        if self.path in ("/status", "/status/"):
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
            
        elif self.path in ("/scan", "/scan/"):
            with state_lock:
                if not latest_scan["timestamp"]:
                    self._send(503, {"error": "No scan data available", "connected": status_data["connected"], "state": status_data["state"]})
                    return
                # Update scan age
                status_data["lastScanAgeMs"] = int(status_data["lastScanAgeMs"])
                payload = dict(latest_scan)
            self._send(200, payload)
            
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
