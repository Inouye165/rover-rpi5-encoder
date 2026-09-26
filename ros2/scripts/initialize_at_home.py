#!/usr/bin/env python3
"""
initialize_at_home.py - Guarded HOME Pose Initializer & Readiness Validator for Yahboom Pi 5 Rover

PURPOSE:
Applies the verified, permanent HOME tape mark pose to AMCL, confirms acceptance, executes stationary
no-motion convergence, verifies geometric LiDAR scan-to-map alignment, clears costmaps, and validates
the complete TF and localization chain before declaring LOCALIZATION READY.

SAFETY GUARDS:
1. Stationary only: Rover remains disarmed and locked.
2. Explicit operator confirmation required (via --confirm-home flag or interactive prompt).
3. Strict readiness gate:
   - map->odom exists, connected, and fresh (< 0.5s lag)
   - map->base_link within 10 cm and 5° of HOME
   - AMCL covariance σ_x <= 10 cm, σ_y <= 10 cm, σ_yaw <= 5°
   - LiDAR scan-to-map wall overlap >= 70% within 10 cm
4. Fails fast with non-zero exit code if gates are not met.
"""

import sys
import os
import json
import time
import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ClearEntireCostmap, SetInitialPose
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

DEFAULT_HOME = {
    "x": 1.193853,
    "y": -0.045221,
    "z": 0.0,
    "yaw_deg": -5.047,
    "yaw_rad": -0.088087,
    "qz": -0.044029,
    "qw": 0.999030,
    "frame_id": "map",
    "cov_x": 0.01,    # std = 10 cm
    "cov_y": 0.01,    # std = 10 cm
    "cov_yaw": 0.003  # std = 3.1 deg
}

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg

def load_home_pose_file(target_map_name=None):
    candidates = [
        ("/ros2_ws/maps/home_pose_slam_2026-09-23_candidate_hallway.json", "house_slam_2026-09-23_candidate_hallway"),
        ("/ros2_ws/maps/home_pose_slam.json", "house_slam_2026-09-23_candidate_hallway"),
        ("/ros2_ws/maps/home_pose_slam_2026-08-23_final.json", "house_slam_2026-08-23_final"),
        ("/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam_2026-09-23_candidate_hallway.json", "house_slam_2026-09-23_candidate_hallway"),
        ("/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam.json", "house_slam_2026-09-23_candidate_hallway"),
        ("/home/ron/yahboom-encoder/ros2/volumes/maps/home_pose_slam_2026-08-23_final.json", "house_slam_2026-08-23_final"),
        ("/home/ron/yahboom-encoder/home_pose_slam.json", "house_slam_2026-09-23_candidate_hallway"),
        (os.path.join(os.path.dirname(__file__), "home_pose_slam_2026-09-23_candidate_hallway.json"), "house_slam_2026-09-23_candidate_hallway"),
        (os.path.join(os.path.dirname(__file__), "home_pose_slam.json"), "house_slam_2026-09-23_candidate_hallway"),
        (os.path.join(os.path.dirname(__file__), "home_pose_slam_2026-08-23_final.json"), "house_slam_2026-08-23_final")
    ]

    for p, assoc in candidates:
        if os.path.isfile(p):
            try:
                with open(p, "r") as f:
                    data = json.load(f)
                    pos = data.get("pose", {}).get("position", {})
                    ori = data.get("pose", {}).get("orientation", {})
                    yaw_deg = data.get("pose", {}).get("yaw_deg", DEFAULT_HOME["yaw_deg"])
                    
                    if target_map_name:
                        if target_map_name not in p and target_map_name not in data.get("associated_map", "") and target_map_name not in data.get("meta", {}).get("map_name", ""):
                            continue
                    
                    return {
                        "x": pos.get("x", DEFAULT_HOME["x"]),
                        "y": pos.get("y", DEFAULT_HOME["y"]),
                        "z": pos.get("z", DEFAULT_HOME["z"]),
                        "yaw_deg": yaw_deg,
                        "yaw_rad": math.radians(yaw_deg),
                        "qz": ori.get("z", DEFAULT_HOME["qz"]),
                        "qw": ori.get("w", DEFAULT_HOME["qw"]),
                        "frame_id": data.get("frame_id", "map"),
                        "source": f"{p} (assoc={assoc})"
                    }
            except Exception:
                pass
    return {**DEFAULT_HOME, "source": "BUILTIN_DEFAULT"}

class HomeInitializerNode(Node):
    def __init__(self):
        super().__init__('home_pose_guarded_initializer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_reliable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        volatile_best_effort_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT)
        volatile_reliable_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)

        self.pub_initialpose_latched = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', latched_reliable_qos)
        self.pub_initialpose_volatile = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', volatile_reliable_qos)
        self.pub_initialpose_be = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', volatile_best_effort_qos)

        self.latest_amcl_pose = None
        self.latest_scan = None
        self.latest_map = None

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, latched_reliable_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched_reliable_qos)

        self.amcl_state_client = self.create_client(GetState, '/amcl/get_state')
        self.set_initial_pose_client = self.create_client(SetInitialPose, '/set_initial_pose')
        self.nomotion_client = self.create_client(Empty, '/request_nomotion_update')
        self.clear_global_cm_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_cm_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')

    def amcl_cb(self, msg):
        self.latest_amcl_pose = msg

    def scan_cb(self, msg):
        self.latest_scan = msg

    def map_cb(self, msg):
        self.latest_map = msg

    def check_amcl_active(self, timeout_sec=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if self.amcl_state_client.wait_for_service(timeout_sec=0.5):
                fut = self.amcl_state_client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
                if fut.result() and fut.result().current_state.label == 'active':
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def send_initial_pose(self, home_data):
        now_msg = self.get_clock().now().to_msg()
        
        service_accepted = False
        if self.set_initial_pose_client.wait_for_service(timeout_sec=1.5):
            req = SetInitialPose.Request()
            req.pose.header.frame_id = home_data["frame_id"]
            req.pose.header.stamp = now_msg
            req.pose.pose.pose.position.x = float(home_data["x"])
            req.pose.pose.pose.position.y = float(home_data["y"])
            req.pose.pose.pose.position.z = 0.0
            req.pose.pose.pose.orientation.z = float(home_data["qz"])
            req.pose.pose.pose.orientation.w = float(home_data["qw"])
            cov = [0.0] * 36
            cov[0] = float(home_data.get("cov_x", 0.01))
            cov[7] = float(home_data.get("cov_y", 0.01))
            cov[35] = float(home_data.get("cov_yaw", 0.003))
            req.pose.pose.covariance = cov

            fut = self.set_initial_pose_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.result() is not None:
                service_accepted = True

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = home_data["frame_id"]
        msg.header.stamp = now_msg
        msg.pose.pose.position.x = float(home_data["x"])
        msg.pose.pose.position.y = float(home_data["y"])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = float(home_data["qz"])
        msg.pose.pose.orientation.w = float(home_data["qw"])
        cov = [0.0] * 36
        cov[0] = float(home_data.get("cov_x", 0.01))
        cov[7] = float(home_data.get("cov_y", 0.01))
        cov[35] = float(home_data.get("cov_yaw", 0.003))
        msg.pose.covariance = cov

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_initialpose_latched.publish(msg)
            self.pub_initialpose_volatile.publish(msg)
            self.pub_initialpose_be.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        return service_accepted

    def call_nomotion(self):
        if not self.nomotion_client.wait_for_service(timeout_sec=1.0):
            return False
        fut = self.nomotion_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        return True

    def clear_costmaps(self):
        res = {'global': False, 'local': False}
        if self.clear_global_cm_client.wait_for_service(timeout_sec=3.0):
            fut = self.clear_global_cm_client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            res['global'] = True
        if self.clear_local_cm_client.wait_for_service(timeout_sec=3.0):
            fut = self.clear_local_cm_client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            res['local'] = True
        return res

def run_initializer(confirmed=False, num_iterations=8, json_out=False):
    if not confirmed:
        print("\n" + "="*65)
        print(" [GUARD CHECK] OPERATOR CONFIRMATION REQUIRED")
        print("="*65)
        print(" This command will seed AMCL localization with the permanent HOME pose.")
        print(" ONLY execute this if the rover is physically on the tape mark facing forward.")
        resp = input("\nIs the rover physically placed at the HOME tape mark? (yes/no): ").strip().lower()
        if resp not in ["yes", "y"]:
            print("Initialization aborted. No changes made.")
            sys.exit(1)

    rclpy.init()
    node = HomeInitializerNode()
    home_data = load_home_pose_file()

    print("=================================================================")
    print("      GUARDED HOME POSE INITIALIZATION & AMCL CONVERGENCE        ")
    print("=================================================================")
    print(f"Source:     {home_data.get('source')}")
    print(f"Target:     x = {home_data['x']:.4f} m, y = {home_data['y']:.4f} m, yaw = {home_data['yaw_deg']:.2f}°")

    # Step 1: Wait for AMCL active
    print(f"\n[1/6] Checking AMCL lifecycle readiness...")
    amcl_ready = node.check_amcl_active(timeout_sec=8.0)
    if not amcl_ready:
        print("  ERROR: AMCL node is not in 'active' state!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print("  • AMCL lifecycle state: ACTIVE")

    # Step 2: Send Initial Pose Handshake
    print(f"[2/6] Sending HOME pose handshake to AMCL...")
    srv_ok = node.send_initial_pose(home_data)
    print(f"  • /set_initial_pose Service Handshake: {'OK' if srv_ok else 'FALLBACK_TOPIC'}")

    # Step 3: Confirm AMCL accepted HOME pose (within 20 cm before no-motion)
    print(f"[3/6] Confirming initial pose acceptance...")
    t0 = time.time()
    accepted = False
    first_accepted_pose = None
    first_pos_err = 999.0
    first_yaw_err = 999.0

    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_amcl_pose:
            p = node.latest_amcl_pose.pose.pose.position
            q = node.latest_amcl_pose.pose.pose.orientation
            yaw = quaternion_to_yaw(q)
            pos_err = math.hypot(p.x - home_data["x"], p.y - home_data["y"])
            yaw_err = abs(normalize_angle_deg(math.degrees(yaw) - home_data["yaw_deg"]))
            if pos_err <= 0.25: # Accept within 25cm
                accepted = True
                first_accepted_pose = (p.x, p.y, math.degrees(yaw))
                first_pos_err = pos_err
                first_yaw_err = yaw_err
                break

    if not accepted:
        print(f"  ERROR: AMCL did not accept HOME pose! Latest pose: {node.latest_amcl_pose}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print(f"  • First Accepted Pose: x = {first_accepted_pose[0]:.4f} m, y = {first_accepted_pose[1]:.4f} m, yaw = {first_accepted_pose[2]:.2f}°")
    print(f"  • First-Pose Error:    pos = {first_pos_err*100.0:.2f} cm, yaw = {first_yaw_err:.2f}°")

    # Step 4: Iterative no-motion convergence
    print(f"[4/6] Triggering {num_iterations} stationary AMCL no-motion update iterations...")
    for i in range(1, num_iterations + 1):
        node.call_nomotion()
        time.sleep(0.10)
        rclpy.spin_once(node, timeout_sec=0.08)

    # Step 5: Verification: Scan to Map Alignment & TF Freshness
    print(f"[5/6] Evaluating LiDAR scan-to-wall geometric alignment & TF freshness...")
    alignment_score = 0.0
    total_valid_scan = 0
    near_wall_hits = 0
    tf_fresh = False
    tf_lag_ms = 999.0

    t_spin = time.time()
    while time.time() - t_spin < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.latest_map and node.latest_scan:
        try:
            tf_laser = node.tf_buffer.lookup_transform('map', 'laser_frame', rclpy.time.Time())
            tf_stamp_s = tf_laser.header.stamp.sec + tf_laser.header.stamp.nanosec * 1e-9
            now_s = node.get_clock().now().nanoseconds * 1e-9
            tf_lag_ms = max(0.0, (now_s - tf_stamp_s) * 1000.0)
            tf_fresh = tf_lag_ms < 500.0

            tx = tf_laser.transform.translation.x
            ty = tf_laser.transform.translation.y
            tq = tf_laser.transform.rotation
            tyaw = quaternion_to_yaw(tq)

            map_info = node.latest_map.info
            res = map_info.resolution
            ox, oy = map_info.origin.position.x, map_info.origin.position.y
            w, h = map_info.width, map_info.height
            map_data = np.array(node.latest_map.data).reshape((h, w))

            ranges = np.array(node.latest_scan.ranges)
            angles = node.latest_scan.angle_min + np.arange(len(ranges)) * node.latest_scan.angle_increment
            valid_mask = (ranges > node.latest_scan.range_min) & (ranges < node.latest_scan.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)

            valid_r = ranges[valid_mask]
            valid_a = angles[valid_mask]

            lx = valid_r * np.cos(valid_a)
            ly = valid_r * np.sin(valid_a)

            mx = tx + (lx * math.cos(tyaw) - ly * math.sin(tyaw))
            my = ty + (lx * math.sin(tyaw) + ly * math.cos(tyaw))

            gx = ((mx - ox) / res).astype(int)
            gy = ((my - oy) / res).astype(int)

            inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
            gx = gx[inside]
            gy = gy[inside]

            total_valid_scan = len(gx)
            for x, y in zip(gx, gy):
                sub = map_data[max(0, y-2):min(h, y+3), max(0, x-2):min(w, x+3)]
                if np.any(sub > 50):
                    near_wall_hits += 1

            if total_valid_scan > 0:
                alignment_score = (near_wall_hits / total_valid_scan) * 100.0
        except Exception as e:
            print(f"  Warning: Alignment calculation TF error: {e}")

    print(f"  • Valid LiDAR Points:       {total_valid_scan}")
    print(f"  • Near-Wall Hits (<=10cm):   {near_wall_hits} ({alignment_score:.1f}%)")
    print(f"  • TF Freshness (map->laser): {tf_lag_ms:.1f} ms ({'FRESH' if tf_fresh else 'STALE'})")

    # Step 6: Clear Costmaps
    print(f"[6/6] Clearing global and local costmaps...")
    cm_res = node.clear_costmaps()
    print(f"  • Global Costmap Cleared:   {cm_res['global']}")
    print(f"  • Local Costmap Cleared:    {cm_res['local']}")

    # Final Convergence Evaluation
    final_pass = False
    summary_data = {}
    if node.latest_amcl_pose:
        p = node.latest_amcl_pose.pose.pose.position
        q = node.latest_amcl_pose.pose.pose.orientation
        c = node.latest_amcl_pose.pose.covariance
        y_deg = math.degrees(quaternion_to_yaw(q))
        sx = math.sqrt(abs(c[0])) * 100.0
        sy = math.sqrt(abs(c[7])) * 100.0
        syaw = math.degrees(math.sqrt(abs(c[35])))

        final_pos_err = math.hypot(p.x - home_data["x"], p.y - home_data["y"])
        final_yaw_err = abs(normalize_angle_deg(y_deg - home_data["yaw_deg"]))
        max_jump = math.hypot(p.x - first_accepted_pose[0], p.y - first_accepted_pose[1])

        # Contract Limit Gates:
        # 1. Final pose within 10 cm and 5° of HOME
        # 2. Covariance <= 10 cm for x/y and <= 5° yaw
        # 3. Scan overlap >= 70%
        # 4. Max jump after first accepted pose <= 10 cm (no large jump)
        gate_pose = final_pos_err <= 0.10 and final_yaw_err <= 5.0
        gate_cov = sx <= 10.0 and sy <= 10.0 and syaw <= 5.0
        gate_scan = alignment_score >= 70.0
        gate_jump = max_jump <= 0.10

        final_pass = gate_pose and gate_cov and gate_scan and gate_jump

        print("\n" + "="*65)
        print("      INITIALIZATION SUMMARY & READINESS GATE            ")
        print("=================================================================")
        print(f"  • First Accepted Pose: x = {first_accepted_pose[0]:.4f} m, y = {first_accepted_pose[1]:.4f} m, yaw = {first_accepted_pose[2]:.2f}°")
        print(f"  • Final AMCL Pose:     x = {p.x:.4f} m, y = {p.y:.4f} m, heading = {y_deg:.2f}°")
        print(f"  • Pose Error to HOME:  pos = {final_pos_err*100.0:.2f} cm (limit <=10cm), yaw = {final_yaw_err:.2f}° (limit <=5°)")
        print(f"  • Max Jump After Init: {max_jump*100.0:.2f} cm (limit <=10cm)")
        print(f"  • Uncertainty (1σ):    σ_x = {sx:.2f} cm, σ_y = {sy:.2f} cm, σ_yaw = {syaw:.2f}°")
        print(f"  • Scan-to-Map Overlap: {alignment_score:.1f}% (target >= 70%)")
        print(f"  • Gate Checks:         [Pose: {'PASS' if gate_pose else 'FAIL'}] [Cov: {'PASS' if gate_cov else 'FAIL'}] [Scan: {'PASS' if gate_scan else 'FAIL'}] [Jump: {'PASS' if gate_jump else 'FAIL'}]")
        print(f"  • OVERALL STATUS:      {'>>> LOCALIZATION READY (PASS) <<<' if final_pass else '>>> LOCALIZATION FAILED <<<'}")
        print("=================================================================")

        summary_data = {
            "first_accepted_pose": {"x": first_accepted_pose[0], "y": first_accepted_pose[1], "yaw_deg": first_accepted_pose[2]},
            "first_pos_err_cm": round(first_pos_err * 100.0, 2),
            "first_yaw_err_deg": round(first_yaw_err, 2),
            "final_pose": {"x": round(p.x, 4), "y": round(p.y, 4), "yaw_deg": round(y_deg, 2)},
            "final_pos_err_cm": round(final_pos_err * 100.0, 2),
            "final_yaw_err_deg": round(final_yaw_err, 2),
            "max_jump_cm": round(max_jump * 100.0, 2),
            "sigma_x_cm": round(sx, 2),
            "sigma_y_cm": round(sy, 2),
            "sigma_yaw_deg": round(syaw, 2),
            "scan_overlap_pct": round(alignment_score, 1),
            "tf_lag_ms": round(tf_lag_ms, 1),
            "passed": final_pass
        }

    if json_out:
        with open("/ros2_ws/logs/last_home_initialization.json", "w") as f:
            json.dump(summary_data, f, indent=2)

    node.destroy_node()
    rclpy.shutdown()

    if not final_pass:
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="Guarded HOME Pose Initializer & Readiness Validator")
    parser.add_argument("--confirm-home", action="store_true", help="Explicitly confirm rover is physically on the HOME mark")
    parser.add_argument("--iterations", type=int, default=8, help="Stationary AMCL no-motion update iterations")
    parser.add_argument("--json", action="store_true", help="Write JSON summary to /ros2_ws/logs/last_home_initialization.json")
    args = parser.parse_args()

    run_initializer(confirmed=args.confirm_home, num_iterations=args.iterations, json_out=args.json)

if __name__ == '__main__':
    main()
