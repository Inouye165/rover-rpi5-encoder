#!/usr/bin/env python3
# ==============================================================================
# rover_nav_bridge.py - Lightweight Nav2 HTTP Navigation Bridge Node
# Provides clean HTTP endpoints on port 3005 for Cockpit:
#   - GET  /api/nav/map       -> Static map metadata & occupancy pixels
#   - POST /api/nav/plan      -> Disarmed Smac2D collision-free path preview
#   - POST /api/nav/dispatch  -> NavigateToPose action dispatch
#   - POST /api/nav/cancel    -> Goal cancellation & safe stop
#   - GET  /api/nav/status    -> Active goal, global path & local DWB trajectory
# ==============================================================================

import os
import json
import time
import math
import subprocess
import threading
import yaml
from http.server import HTTPServer, BaseHTTPRequestHandler
import requests

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from nav_msgs.msg import Path

DEFAULT_PORT = 3005
ODOM_API_URL = "http://127.0.0.1:3003/api/odom"
MAP_YAML_PATH = "/ros2_ws/maps/house_slam_2026-09-23_candidate_hallway.yaml"
MAP_PGM_PATH = "/ros2_ws/maps/house_slam_2026-09-23_candidate_hallway.pgm"

class RoverNavBridge(Node):
    def __init__(self):
        super().__init__('rover_nav_bridge')

        self.compute_path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')

        self.latest_global_plan = []
        self.latest_local_plan = []
        self.active_goal_handle = None
        self.active_goal_status = "IDLE"
        self.active_target = None

        self.sub_plan = self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.sub_local_plan = self.create_subscription(Path, '/local_plan', self._local_plan_cb, 10)

        # Pre-load and cache map
        self.map_cache = self._load_map()

        # Warm up action clients in background so first request is instant
        threading.Thread(target=self._warm_up_clients, daemon=True).start()

        self.get_logger().info("RoverNavBridge initialized. Subscribed to /plan, /local_plan.")

    def _warm_up_clients(self):
        try:
            self.compute_path_client.wait_for_server(timeout_sec=2.0)
            self.nav_client.wait_for_server(timeout_sec=2.0)
        except Exception:
            pass

    def _plan_cb(self, msg: Path):
        self.latest_global_plan = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]

    def _local_plan_cb(self, msg: Path):
        self.latest_local_plan = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]

    def _get_active_home_pose_path(self):
        pointer_files = [
            "/ros2_ws/maps/home_pose_slam_2026-09-23_candidate_hallway.json",
            "/ros2_ws/maps/home_pose_slam.json",
            "/ros2_ws/maps/home_pose_slam_2026-08-23_final.json",
            "/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam_2026-09-23_candidate_hallway.json",
            "/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam.json",
            "/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam_2026-08-23_final.json",
            "/home/ron/yahboom-encoder/home_pose_slam.json",
        ]
        for pf in pointer_files:
            if os.path.exists(pf):
                return pf
        return "/ros2_ws/maps/home_pose_slam.json"

    def _get_active_yaml_path(self):
        pointer_files = [
            "/ros2_ws/maps/active_map_path.txt",
            "/tmp/active_map_path.txt",
            "/home/ron/yahboom-encoder/ros2/volumes/maps/active_map_path.txt"
        ]
        for pf in pointer_files:
            if os.path.exists(pf):
                try:
                    with open(pf, "r") as f:
                        val = f.read().strip()
                        if val and os.path.exists(val):
                            return val
                except Exception:
                    pass
        return os.environ.get("ROVER_ACTIVE_MAP_YAML", MAP_YAML_PATH)

    def _load_map(self):
        try:
            yaml_path = self._get_active_yaml_path()
            if not os.path.exists(yaml_path):
                self.get_logger().warn(f"Map YAML not found at {yaml_path}")
                return getattr(self, 'map_cache', None)

            mtime = os.path.getmtime(yaml_path)
            if getattr(self, 'active_yaml_path', None) == yaml_path and getattr(self, 'active_yaml_mtime', None) == mtime and getattr(self, 'map_cache', None):
                return self.map_cache

            with open(yaml_path, 'r') as f:
                ydata = yaml.safe_load(f)

            res = float(ydata.get('resolution', 0.05))
            origin = [float(x) for x in ydata.get('origin', [-0.746, -5.035, 0.0])]
            img_rel = ydata.get('image', '')
            pgm_path = os.path.join(os.path.dirname(yaml_path), img_rel)
            if not os.path.exists(pgm_path):
                self.get_logger().warn(f"Map PGM not found at {pgm_path}")
                return getattr(self, 'map_cache', None)

            with open(pgm_path, "rb") as f:
                buf = f.read()

            newlines = 0
            header_end = 0
            for i in range(len(buf)):
                if buf[i] == 0x0A:
                    newlines += 1
                    if newlines == 3:
                        header_end = i + 1
                        break

            header_str = buf[:header_end].decode('ascii', errors='ignore')
            lines = [l.strip() for l in header_str.splitlines() if l.strip() and not l.startswith('#')]
            w, h = map(int, lines[1].split())
            pixels = list(buf[header_end:])

            self.active_yaml_path = yaml_path
            self.active_yaml_mtime = mtime
            self.map_cache = {
                "ok": True,
                "map_name": os.path.basename(yaml_path),
                "width": w,
                "height": h,
                "resolution": res,
                "origin": origin,
                "pixels": pixels
            }
            self.get_logger().info(f"Loaded active map from {yaml_path}: {w}x{h} @ {res}m/px, origin={origin}")
            return self.map_cache
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return getattr(self, 'map_cache', None)

    def compute_plan(self, start_x, start_y, start_yaw, target_x, target_y, target_yaw):
        t_req_start = time.perf_counter()

        t_wait_server_start = time.perf_counter()
        if not self.compute_path_client.server_is_ready():
            if not self.compute_path_client.wait_for_server(timeout_sec=1.0):
                return {"ok": False, "error": "/compute_path_to_pose action server unavailable"}
        t_wait_server_end = time.perf_counter()

        goal = ComputePathToPose.Goal()
        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = float(start_x)
        goal.start.pose.position.y = float(start_y)
        goal.start.pose.orientation.z = math.sin(start_yaw / 2.0)
        goal.start.pose.orientation.w = math.cos(start_yaw / 2.0)
        goal.use_start = True

        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(target_x)
        goal.goal.pose.position.y = float(target_y)
        goal.goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(target_yaw / 2.0)
        goal.planner_id = 'Smac2D'

        t_send_goal_start = time.perf_counter()
        future = self.compute_path_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.001)

        if not future.done() or not future.result().accepted:
            return {"ok": False, "error": "Plan goal was rejected or timed out by Smac2D planner"}
        t_goal_accept_end = time.perf_counter()

        handle = future.result()
        t_compute_start = time.perf_counter()
        res_future = handle.get_result_async()
        t0 = time.time()
        while not res_future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.001)

        if not res_future.done():
            return {"ok": False, "error": "Plan computation timed out"}
        t_compute_end = time.perf_counter()

        t_extract_start = time.perf_counter()
        result = res_future.result().result
        waypoints = []
        total_len = 0.0
        for i, p in enumerate(result.path.poses):
            px, py = p.pose.position.x, p.pose.position.y
            waypoints.append([round(px, 3), round(py, 3)])
            if i > 0:
                prev_p = result.path.poses[i-1].pose.position
                total_len += math.hypot(px - prev_p.x, py - prev_p.y)

        straight_len = 0.0
        if waypoints:
            straight_len = math.hypot(waypoints[-1][0] - waypoints[0][0], waypoints[-1][1] - waypoints[0][1])
        t_extract_end = time.perf_counter()

        t_req_end = time.perf_counter()
        timing_bridge = {
            "wait_server_ms": round((t_wait_server_end - t_wait_server_start) * 1000, 2),
            "goal_accept_ms": round((t_goal_accept_end - t_send_goal_start) * 1000, 2),
            "computepath_action_ms": round((t_compute_end - t_compute_start) * 1000, 2),
            "extract_waypoints_ms": round((t_extract_end - t_extract_start) * 1000, 2),
            "total_bridge_ms": round((t_req_end - t_req_start) * 1000, 2)
        }

        return {
            "ok": True,
            "waypoints": waypoints,
            "first_waypoint": waypoints[0] if waypoints else None,
            "last_waypoint": waypoints[-1] if waypoints else None,
            "cumulative_length_m": round(total_len, 3),
            "straight_distance_m": round(straight_len, 3),
            "count": len(waypoints),
            "timing_bridge": timing_bridge
        }

    def dispatch_goal(self, target_x, target_y, target_yaw):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return {"ok": False, "error": "/navigate_to_pose action server unavailable"}

        # Cancel any previous goal
        self.cancel_goal()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target_x)
        goal.pose.pose.position.y = float(target_y)
        goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        future = self.nav_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.02)

        if not future.done() or not future.result().accepted:
            return {"ok": False, "error": "Goal was rejected by /navigate_to_pose action server"}

        self.active_goal_handle = future.result()
        self.active_goal_status = "EXECUTING"
        self.active_target = {"x": target_x, "y": target_y, "yaw": target_yaw}

        def _on_done(f):
            try:
                res = f.result()
                status = res.status
                # action_msgs/msg/GoalStatus: 4=STATUS_SUCCEEDED, 5=STATUS_CANCELED, 6=STATUS_ABORTED
                if status == 4:
                    self.active_goal_status = "SUCCEEDED"
                elif status == 5:
                    self.active_goal_status = "CANCELLED"
                else:
                    self.active_goal_status = f"STOPPED_STATUS_{status}"
            except Exception:
                self.active_goal_status = "FINISHED"
            self.active_goal_handle = None

        res_fut = self.active_goal_handle.get_result_async()
        res_fut.add_done_callback(_on_done)

        return {"ok": True, "status": "EXECUTING", "target": self.active_target}

    def request_nomotion_update(self, timeout_sec=1.5):
        """Requests an instantaneous AMCL no-motion particle filter update."""
        try:
            if not self.nomotion_client.service_is_ready():
                if not self.nomotion_client.wait_for_service(timeout_sec=0.5):
                    return {"ok": False, "error": "AMCL /request_nomotion_update service not ready"}
            future = self.nomotion_client.call_async(Empty.Request())
            t0 = time.time()
            while not future.done() and (time.time() - t0) < timeout_sec:
                time.sleep(0.01)
            if not future.done():
                return {"ok": False, "error": f"AMCL /request_nomotion_update service call timed out after {timeout_sec}s"}
            return {"ok": True, "message": "AMCL no-motion update requested"}
        except Exception as err:
            return {"ok": False, "error": f"Error calling nomotion service: {err}"}

    def cancel_goal(self):
        cancelled = False
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
                cancelled = True
            except Exception:
                pass
            self.active_goal_handle = None

        try:
            if self.nav_client.server_is_ready():
                self.nav_client.cancel_all_goals()
                cancelled = True
        except Exception:
            pass

        self.active_goal_status = "CANCELLED"
        self.latest_local_plan = []
        return {"ok": True, "status": "CANCELLED", "cancelled": cancelled}


bridge_node = None

class NavHTTPHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type, X-Rover-Operator-Token')
        self.end_headers()

    def do_GET(self):
        if bridge_node is None:
            self._send_json(503, {"ok": False, "error": "Bridge node uninitialized"})
            return

        if self.path.startswith('/api/nav/map'):
            map_data = bridge_node._load_map()
            if map_data:
                self._send_json(200, map_data)
            else:
                self._send_json(404, {"ok": False, "error": "Map not loaded"})

        elif self.path == '/api/nav/home':
            home_path = bridge_node._get_active_home_pose_path()
            if os.path.exists(home_path):
                try:
                    with open(home_path, "r") as f:
                        home_json = json.load(f)
                    pos = home_json.get("pose", {}).get("position", {})
                    ori = home_json.get("pose", {})
                    self._send_json(200, {
                        "ok": True,
                        "home": {
                            "x": pos.get("x", 1.193853),
                            "y": pos.get("y", -0.045221),
                            "yaw_deg": ori.get("yaw_deg", -5.047),
                            "yaw_rad": ori.get("yaw_rad", -0.088087),
                            "description": home_json.get("description", "Permanent verified HOME pose settled on floor at x=1.193853, y=-0.045221, yaw=-5.047 deg")
                        }
                    })
                    return
                except Exception as err:
                    self._send_json(500, {"ok": False, "error": str(err)})
                    return
            self._send_json(404, {"ok": False, "error": "Home pose file not found"})

        elif self.path == '/api/nav/status':
            # Query odometry node for current AMCL pose to compute distance remaining
            cur_x, cur_y = 0.0, 0.0
            try:
                r = requests.get(ODOM_API_URL, timeout=0.25).json()
                loc = r.get('localization', {}).get('pose', {})
                cur_x = loc.get('x', 0.0)
                cur_y = loc.get('y', 0.0)
            except Exception:
                pass

            dist_rem = 0.0
            if bridge_node.active_target:
                dist_rem = math.hypot(bridge_node.active_target['x'] - cur_x, bridge_node.active_target['y'] - cur_y)

            data = {
                "ok": True,
                "status": bridge_node.active_goal_status,
                "target": bridge_node.active_target,
                "distance_remaining_m": round(dist_rem, 3),
                "global_path": bridge_node.latest_global_plan,
                "local_path": bridge_node.latest_local_plan
            }
            self._send_json(200, data)
        else:
            self._send_json(404, {"ok": False, "error": "Endpoint not found"})

    def do_POST(self):
        if bridge_node is None:
            self._send_json(503, {"ok": False, "error": "Bridge node uninitialized"})
            return

        content_len = int(self.headers.get('Content-Length', 0))
        post_body = self.rfile.read(content_len).decode('utf-8')
        try:
            req_json = json.loads(post_body) if post_body else {}
        except Exception:
            req_json = {}

        if self.path == '/api/nav/plan':
            sx = req_json.get('start_x')
            sy = req_json.get('start_y')
            syaw = req_json.get('start_yaw')

            if sx is None or sy is None:
                try:
                    r = requests.get(ODOM_API_URL, timeout=0.50).json()
                    loc = r.get('localization', {}).get('pose', {})
                    sx = loc.get('x', 1.133)
                    sy = loc.get('y', -0.075)
                    syaw = loc.get('yaw', 0.0)
                except Exception:
                    sx, sy, syaw = 1.133, -0.075, 0.0

            tx = float(req_json.get('target_x', req_json.get('x', 2.154)))
            ty = float(req_json.get('target_y', req_json.get('y', -0.235)))
            tyaw = float(req_json.get('target_yaw', req_json.get('yaw', -0.073)))

            plan_res = bridge_node.compute_plan(sx, sy, syaw, tx, ty, tyaw)
            self._send_json(200 if plan_res.get('ok') else 400, plan_res)

        elif self.path == '/api/nav/dispatch':
            tx = float(req_json.get('target_x', 2.154))
            ty = float(req_json.get('target_y', -0.235))
            tyaw = float(req_json.get('target_yaw', -0.073))

            dispatch_res = bridge_node.dispatch_goal(tx, ty, tyaw)
            self._send_json(200 if dispatch_res.get('ok') else 400, dispatch_res)

        elif self.path == '/api/nav/cancel':
            cancel_res = bridge_node.cancel_goal()
            self._send_json(200, cancel_res)

        elif self.path in ['/api/nav/nomotion_update', '/api/nav/refresh_localization']:
            nomotion_res = bridge_node.request_nomotion_update()
            self._send_json(200 if nomotion_res.get('ok') else 503, nomotion_res)

        else:
            self._send_json(404, {"ok": False, "error": "Endpoint not found"})

    def _send_json(self, code, data):
        resp_bytes = json.dumps(data).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(resp_bytes)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(resp_bytes)


def main():
    global bridge_node
    rclpy.init()
    bridge_node = RoverNavBridge()

    server = HTTPServer(('0.0.0.0', DEFAULT_PORT), NavHTTPHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    bridge_node.get_logger().info(f"RoverNavBridge HTTP server running on 0.0.0.0:{DEFAULT_PORT}")

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
