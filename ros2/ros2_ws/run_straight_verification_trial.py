import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import socket
import base64
import struct
import urllib.request
import json
import math
import time
import os
import sys

# Ensure local and container path discovery for LinearApproachController
sys.path.insert(0, '/ros2_ws')
sys.path.insert(0, '/ros2_ws/scratch')
sys.path.insert(0, os.path.dirname(__file__))

from linear_approach_controller import LinearApproachController, LinearApproachPhase

OPERATOR_TOKEN = "787f1b987d6295357ff3f664e08b0c96984f4f82a7b1edc17adef2793e64a168"
WHEEL_DIAMETER_M = 0.06695
TICKS_PER_REV = 1974.1666666667
METERS_PER_TICK = (math.pi * WHEEL_DIAMETER_M) / TICKS_PER_REV

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class NativeWSClient:
    def __init__(self, host="127.0.0.1", port=3000, path="/ws"):
        self.host = host
        self.port = port
        self.path = path
        self.sock = None
        self.connected = False

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            key = base64.b64encode(os.urandom(16)).decode('utf-8')
            handshake = (
                f"GET {self.path} HTTP/1.1\r\n"
                f"Host: {self.host}:{self.port}\r\n"
                f"Upgrade: websocket\r\n"
                f"Connection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Sec-WebSocket-Version: 13\r\n\r\n"
            )
            self.sock.sendall(handshake.encode('utf-8'))
            resp = self.sock.recv(1024)
            if b"101 Switching Protocols" in resp:
                self.connected = True
                self.sock.settimeout(0.002)
                return True
        except Exception:
            self.connected = False
        return False

    def close(self):
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

class StraightTrialRunner(Node):
    def __init__(self):
        super().__init__('straight_trial_runner')
        self.cmd_pub_nav = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.cmd_pub_direct = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_odom = {'msg': None, 'time': None}
        self.latest_imu = {'msg': None, 'time': None}
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

    def odom_cb(self, msg):
        self.latest_odom = {'msg': msg, 'time': time.time()}

    def imu_cb(self, msg):
        self.latest_imu = {'msg': msg, 'time': time.time()}

    def cockpit_post(self, endpoint, data):
        url = f"http://127.0.0.1:3000{endpoint}"
        payload = json.dumps(data).encode('utf-8')
        headers = {
            'Content-Type': 'application/json',
            'User-Agent': 'Mozilla/5.0',
            'x-rover-operator-token': OPERATOR_TOKEN
        }
        try:
            req = urllib.request.Request(url, data=payload, headers=headers)
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            return {'error': str(e)}

    def cockpit_get(self, endpoint):
        url = f"http://127.0.0.1:3000{endpoint}"
        headers = {
            'User-Agent': 'Mozilla/5.0',
            'x-rover-operator-token': OPERATOR_TOKEN
        }
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                return json.loads(resp.read().decode())
        except Exception as e:
            return {'error': str(e), 'status': 'UNAVAILABLE'}

    def publish_twist(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub_nav.publish(t)
        self.cmd_pub_direct.publish(t)

def run_trial(trial_num=1, target_dist=1.000, speed=0.20):
    rclpy.init()
    node = StraightTrialRunner()

    print(f"\n=================================================================")
    print(f"      PHYSICAL STRAIGHT VERIFICATION TRIAL #{trial_num} ({target_dist:.3f} m)")
    print(f"=================================================================")

    ws = NativeWSClient()
    if ws.connect():
        print("  [OK] WebSocket connected to Cockpit")

    # Step 1: Pre-test Disarmed Handshake & Enable Autonomy
    print("\n[Step 1] Enabling Autonomy & Handshake...")
    node.cockpit_post("/api/autonomy/enable", {})
    for _ in range(25):
        node.publish_twist(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.03)
        time.sleep(0.04)

    # Step 2: Arm Drivetrain
    print("\n[Step 2] Arming Normal Drive...")
    node.cockpit_post("/api/drive/arm", {})
    time.sleep(0.3)
    st = {}
    for _ in range(15):
        st = node.cockpit_get("/api/status")
        if st.get('armed'):
            break
        time.sleep(0.05)

    print(f"  Armed Status: {st.get('armed')} (Autonomy: {st.get('autonomyState')})")
    if not st.get('armed'):
        print(f"  [ERROR] Drivetrain failed to arm! Aborting Trial #{trial_num}.")
        ws.close()
        node.destroy_node()
        rclpy.shutdown()
        return None

    # Record Initial Poses and Encoder Ticks
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.02)

    init_odom_x = node.latest_odom['msg'].pose.pose.position.x if node.latest_odom['msg'] else 0.0
    init_odom_y = node.latest_odom['msg'].pose.pose.position.y if node.latest_odom['msg'] else 0.0
    init_imu_yaw = quaternion_to_yaw(node.latest_imu['msg'].orientation) if node.latest_imu['msg'] else 0.0

    enc_resp = node.cockpit_get("/api/encoders")
    init_ticks = enc_resp.get('encoders', {})
    t1_0 = init_ticks.get('m1', 0)
    t2_0 = init_ticks.get('m2', 0)
    t3_0 = init_ticks.get('m3', 0)
    t4_0 = init_ticks.get('m4', 0)

    print(f"\nInitial Starting State:")
    print(f"  • Odom Pose: x = {init_odom_x:.4f} m, y = {init_odom_y:.4f} m, yaw = {math.degrees(init_imu_yaw):.2f}°")
    print(f"  • Raw Ticks: M1={t1_0}, M2={t2_0}, M3={t3_0}, M4={t4_0}")

    # Step 3: Drive Forward Using LinearApproachController
    print(f"\n--- Driving Forward {target_dist:.3f} m (Cruise={speed:.2f} m/s, Creep=0.05 m/s, Zone=0.15 m) ---")
    controller = LinearApproachController(cruise_speed_mps=speed, creep_speed_mps=0.05, approach_zone_m=0.15)
    t_start = time.time()
    controller.reset(target_dist_m=target_dist, start_time=t_start)

    traveled = 0.0
    last_print_time = 0

    while time.time() - t_start < 15.0:
        now = time.time()
        rclpy.spin_once(node, timeout_sec=0.002)

        odom_msg = node.latest_odom['msg']
        imu_msg = node.latest_imu['msg']

        cur_x = odom_msg.pose.pose.position.x if odom_msg else 0.0
        cur_y = odom_msg.pose.pose.position.y if odom_msg else 0.0
        cur_yaw = quaternion_to_yaw(imu_msg.orientation) if imu_msg else 0.0

        dx = cur_x - init_odom_x
        dy = cur_y - init_odom_y
        traveled = math.hypot(dx, dy)

        heading_err = normalize_angle(cur_yaw - init_imu_yaw)
        wz_corr = max(-0.3, min(0.3, -1.2 * heading_err))

        cmd_vx = controller.update(current_dist_m=traveled, current_time=now)
        node.publish_twist(cmd_vx, wz_corr)

        if controller.phase == LinearApproachPhase.ZERO:
            print(f"  [TARGET REACHED] Target crossed! Traveled {traveled:.4f} m (Target: {target_dist:.3f} m)")
            break

        time.sleep(0.02)

        if now - last_print_time > 0.4:
            rem = target_dist - traveled
            print(f"  [{controller.phase}] Traveled: {traveled:.3f} m / {target_dist:.3f} m (Rem: {rem*1000.0:+.1f} mm) | Cmd Vx: {cmd_vx:.3f} m/s | Heading Err: {math.degrees(heading_err):+.2f}°")
            last_print_time = now

    odom_dist_at_cross = traveled

    # Stop command & settle
    node.publish_twist(0.0, 0.0)
    t_settle_start = time.time()
    while time.time() - t_settle_start < 1.5:
        node.publish_twist(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Step 4: Final Settled State & Ticks
    final_odom_x = node.latest_odom['msg'].pose.pose.position.x if node.latest_odom['msg'] else 0.0
    final_odom_y = node.latest_odom['msg'].pose.pose.position.y if node.latest_odom['msg'] else 0.0
    settled_odom_dist = math.hypot(final_odom_x - init_odom_x, final_odom_y - init_odom_y)
    post_zero_movement = settled_odom_dist - odom_dist_at_cross

    controller.mark_settled(settled_dist_m=settled_odom_dist, current_time=time.time())
    approach_summary = controller.get_telemetry_summary()

    enc_resp_final = node.cockpit_get("/api/encoders")
    final_ticks = enc_resp_final.get('encoders', {})
    t1_f = final_ticks.get('m1', 0)
    t2_f = final_ticks.get('m2', 0)
    t3_f = final_ticks.get('m3', 0)
    t4_f = final_ticks.get('m4', 0)

    dt1 = t1_f - t1_0
    dt2 = t2_f - t2_0
    dt3 = t3_f - t3_0
    dt4 = t4_f - t4_0

    d_m1 = dt1 * METERS_PER_TICK
    d_m2 = dt2 * METERS_PER_TICK
    d_m3 = dt3 * METERS_PER_TICK
    d_m4 = dt4 * METERS_PER_TICK
    d_wheels_avg = (d_m1 + d_m2 + d_m3 + d_m4) / 4.0

    # Step 5: Stop, Disarm, and Lock
    print("\n[Step 5] Disarming and Locking Rover...")
    for _ in range(5):
        node.publish_twist(0.0, 0.0)
        time.sleep(0.02)

    node.cockpit_post("/api/drive/disarm", {})
    node.cockpit_post("/api/autonomy/disable", {})

    print(f"\n=================================================================")
    print(f"      TRIAL #{trial_num} COMPLETE — SETTLED TELEMETRY")
    print(f"=================================================================")
    print(f"  • Slowdown-Entry Distance:   {approach_summary.get('creep_entry_dist_m', 0.0):.4f} m")
    print(f"  • Target-Crossing Distance:  {odom_dist_at_cross:.4f} m")
    print(f"  • Zero-Command Distance:     {approach_summary.get('threshold_crossing_dist_m', 0.0):.4f} m")
    print(f"  • Post-Zero Coast Movement:  {post_zero_movement*1000.0:+.1f} mm")
    print(f"  • Settled ROS /odom Distance:{settled_odom_dist:.4f} m ({settled_odom_dist*1000.0:.1f} mm)")
    print(f"  • Raw Tick Deltas:           M1={dt1}, M2={dt2}, M3={dt3}, M4={dt4} (Avg={(dt1+dt2+dt3+dt4)/4.0:.1f} ticks)")
    print(f"  • Per-Wheel Calculated Dist: M1={d_m1*1000.0:.1f}mm, M2={d_m2*1000.0:.1f}mm, M3={d_m3*1000.0:.1f}mm, M4={d_m4*1000.0:.1f}mm (Avg={d_wheels_avg*1000.0:.1f}mm)")
    print(f"=================================================================")

    trial_data = {
        'trial_num': trial_num,
        'timestamp': time.time(),
        'target_dist_m': target_dist,
        'speed_mps': speed,
        'wheel_diameter_m': WHEEL_DIAMETER_M,
        'ticks_per_rev': TICKS_PER_REV,
        'meters_per_tick': METERS_PER_TICK,
        'approach_summary': approach_summary,
        'initial': {
            'odom_x': init_odom_x,
            'odom_y': init_odom_y,
            'imu_yaw_deg': math.degrees(init_imu_yaw),
            'ticks': {'m1': t1_0, 'm2': t2_0, 'm3': t3_0, 'm4': t4_0}
        },
        'final': {
            'odom_x': final_odom_x,
            'odom_y': final_odom_y,
            'settled_odom_dist_m': settled_odom_dist,
            'odom_dist_at_cross_m': odom_dist_at_cross,
            'post_zero_movement_m': post_zero_movement,
            'ticks': {'m1': t1_f, 'm2': t2_f, 'm3': t3_f, 'm4': t4_f},
            'tick_deltas': {'m1': dt1, 'm2': dt2, 'm3': dt3, 'm4': dt4},
            'wheel_distances_m': {'m1': d_m1, 'm2': d_m2, 'm3': d_m3, 'm4': d_m4, 'avg': d_wheels_avg}
        }
    }

    report_filename = f"/ros2_ws/straight_trial_{trial_num}_{int(time.time())}.json"
    with open(report_filename, "w") as f:
        json.dump(trial_data, f, indent=2)

    ws.close()
    node.destroy_node()
    rclpy.shutdown()
    return trial_data

if __name__ == '__main__':
    trial_num = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    run_trial(trial_num)
