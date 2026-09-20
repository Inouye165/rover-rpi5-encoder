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
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import requests

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DEFAULT_PORT = 3005
ODOM_API_URL = "http://127.0.0.1:3003/api/odom"
MAP_YAML_PATH = "/ros2_ws/maps/house_slam_2026-08-23_final.yaml"
MAP_PGM_PATH = "/ros2_ws/maps/house_slam_2026-08-23_final.pgm"

class RoverNavBridge(Node):
    def __init__(self):
        super().__init__('rover_nav_bridge')

        self.compute_path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.latest_global_plan = []
        self.latest_local_plan = []
        self.active_goal_handle = None
        self.active_goal_status = "IDLE"
        self.active_target = None

        self.sub_plan = self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.sub_local_plan = self.create_subscription(Path, '/local_plan', self._local_plan_cb, 10)

        # Pre-load and cache map
        self.map_cache = self._load_map()

        self.get_logger().info("RoverNavBridge initialized. Subscribed to /plan, /local_plan.")

    def _plan_cb(self, msg: Path):
        self.latest_global_plan = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]

    def _local_plan_cb(self, msg: Path):
        self.latest_local_plan = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]

    def _load_map(self):
        try:
            pgm_path = MAP_PGM_PATH if os.path.exists(MAP_PGM_PATH) else "/home/ron/yahboom-encoder/ros2/volumes/maps/house_slam_2026-08-23_final.pgm"
            if not os.path.exists(pgm_path):
                self.get_logger().warn(f"Map PGM file not found at {pgm_path}")
                return None

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
            lines = [l.strip() for l in header_str.split('\n') if l.strip() and not l.startswith('#')]
            w, h = map(int, lines[1].split())
            pixels = list(buf[header_end:])

            return {
                "ok": True,
                "width": w,
                "height": h,
                "resolution": 0.050,
                "origin": [-0.746, -5.035, 0.0],
                "pixels": pixels
            }
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return None

    def compute_plan(self, start_x, start_y, start_yaw, target_x, target_y, target_yaw):
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            return {"ok": False, "error": "/compute_path_to_pose action server unavailable"}

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

        future = self.compute_path_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.02)

        if not future.done() or not future.result().accepted:
            return {"ok": False, "error": "Plan goal was rejected or timed out by Smac2D planner"}

        handle = future.result()
        res_future = handle.get_result_async()
        t0 = time.time()
        while not res_future.done() and (time.time() - t0) < 3.0:
            time.sleep(0.02)

        if not res_future.done():
            return {"ok": False, "error": "Plan computation timed out"}

        result = res_future.result().result
        waypoints = []
        total_len = 0.0
        for i, p in enumerate(result.path.poses):
            px, py = p.pose.position.x, p.pose.position.y
            waypoints.append([round(px, 3), round(py, 3)])
            if i > 0:
                prev_p = result.path.poses[i-1].pose.position
                total_len += math.hypot(px - prev_p.x, py - prev_p.y)

        return {
            "ok": True,
            "waypoints": waypoints,
            "length_m": round(total_len, 3),
            "count": len(waypoints)
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

        if self.path == '/api/nav/map':
            if bridge_node.map_cache:
                self._send_json(200, bridge_node.map_cache)
            else:
                self._send_json(404, {"ok": False, "error": "Map not loaded"})

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
            # Get current AMCL pose
            try:
                r = requests.get(ODOM_API_URL, timeout=0.50).json()
                loc = r.get('localization', {}).get('pose', {})
                sx = loc.get('x', 1.442)
                sy = loc.get('y', -0.059)
                syaw = loc.get('yaw', -0.048)
            except Exception:
                sx, sy, syaw = 1.442, -0.059, -0.048

            tx = float(req_json.get('target_x', 2.154))
            ty = float(req_json.get('target_y', -0.235))
            tyaw = float(req_json.get('target_yaw', -0.073))

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
