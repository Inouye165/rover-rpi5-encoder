#!/usr/bin/env python3
import os
import sys
import time
import math
import json
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

COCKPIT_URL = "http://127.0.0.1:3000"
INTERNAL_CMD_URL = "http://127.0.0.1:3010/api/cmd_vel"

def load_env(env_path=None):
    if env_path is None:
        if os.path.exists("/ros2_ws/.env"):
            env_path = "/ros2_ws/.env"
        else:
            env_path = "/home/ron/yahboom-encoder/.env"
    env = {}
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    env[k.strip()] = v.strip().strip('"').strip("'")
    return env

def main():
    env = load_env()
    op_token = env.get("ROVER_OPERATOR_TOKEN", "")
    cmd_token = env.get("ROVER_CMD_VEL_TOKEN", "")
    if not op_token:
        print("ERROR: ROVER_OPERATOR_TOKEN not found in .env")
        return 1

    headers_op = {
        "x-operator-token": op_token,
        "X-Rover-Operator-Token": op_token,
        "Content-Type": "application/json"
    }
    headers_cmd = {"X-Rover-Bridge-Token": cmd_token, "Content-Type": "application/json"}

    # Target destination (aligned with costmap cell 58, 96):
    target_x = 2.154
    target_y = -0.235
    target_yaw_deg = -8.34
    target_qz = -0.0727
    target_qw = 0.9974

    print("==================================================")
    print(" Yahboom Rover Nav2 Autonomous Execution Sequence")
    print(f" Target: x={target_x:.3f}m, y={target_y:.3f}m, heading={target_yaw_deg:.2f} deg")
    print("==================================================")

    # 1. Pre-check Cockpit Status & Localization
    try:
        r = requests.get(f"{COCKPIT_URL}/api/status", timeout=2.0)
        status = r.json()
        loc = status.get("localization", {})
        nav = status.get("navigation", {})
        print(f"Current State: armed={status.get('armed')}, mode={status.get('mode')}, autonomy={status.get('autonomyState')}")
        print(f"Localization: localized={loc.get('localized')}, state={loc.get('state')}, fresh={loc.get('freshValidationOk')}, age={loc.get('ageMs')}ms")
        print(f"Navigation Lifecycle: ready={nav.get('ready')}, state={nav.get('state')}, details={nav.get('details')}")
        print(f"Current Pose: x={loc.get('x'):.3f}, y={loc.get('y'):.3f}, yaw={loc.get('yawDeg'):.2f} deg")
        
        if not loc.get("localized") or loc.get("state") != "LOCALIZED":
            print("ERROR: Robot is not localized. Aborting.")
            return 1
        if not nav.get("ready"):
            print(f"ERROR: Navigation stack is not ready: {nav.get('details')}. Aborting.")
            return 1

        start_x = loc.get("x")
        start_y = loc.get("y")
        start_yaw = loc.get("yawDeg")
    except Exception as e:
        print(f"ERROR: Failed to query cockpit status: {e}")
        return 1

    # 2. Initialize ROS 2 Node & Action Client
    rclpy.init()
    node = Node("nav2_mission_runner")
    nav_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

    print("Connecting to /navigate_to_pose action server...")
    if not nav_client.wait_for_server(timeout_sec=5.0):
        print("ERROR: /navigate_to_pose action server unavailable.")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("Connected to Nav2 action server.")

    goal_handle = None
    result_future = None
    exit_code = 0
    reason = "UNKNOWN"
    start_time = time.time()

    try:
        # 3. Enable Autonomy Intake
        print("\n[Step 1/4] Enabling Autonomy (POST /api/autonomy/enable)...")
        r = requests.post(f"{COCKPIT_URL}/api/autonomy/enable", json={"token": op_token}, headers=headers_op, timeout=2.0)
        res = r.json()
        if not res.get("ok"):
            raise RuntimeError(f"Failed to enable autonomy: {res}")
        print(f"Autonomy enabled: state={res.get('status', {}).get('state')}")

        # 4. Zero Command Handshake (3 zero packets to /api/cmd_vel)
        print("\n[Step 2/4] Sending 3 zero-motion packets for handshake...")
        zero_payload = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        for i in range(3):
            r = requests.post(INTERNAL_CMD_URL, json=zero_payload, headers=headers_cmd, timeout=1.0)
            time.sleep(0.05)
        
        # Verify transition to READY_DISARMED
        r = requests.get(f"{COCKPIT_URL}/api/autonomy/status", timeout=1.0)
        auton_stat = r.json()
        print(f"Autonomy state after zero handshake: {auton_stat.get('state')}")
        if auton_stat.get("state") != "READY_DISARMED":
            raise RuntimeError(f"Expected READY_DISARMED but got {auton_stat.get('state')}")

        # Ensure fresh validation and navigation readiness before arming:
        print("\nWaiting for fresh validation & navigation readiness...")
        from std_srvs.srv import Empty
        nomotion_client = node.create_client(Empty, '/request_nomotion_update')
        for wait_attempt in range(20):
            r = requests.get(f"{COCKPIT_URL}/api/status", timeout=0.5)
            st = r.json()
            loc_st = st.get("localization", {})
            nav_st = st.get("navigation", {})
            fresh_ok = loc_st.get("freshValidationOk") or (loc_st.get("ageMs") is not None and loc_st.get("ageMs") <= 1500)
            nav_ok = nav_st.get("ready", False)
            if fresh_ok and nav_ok:
                print(f"Fresh validation & navigation confirmed! age={loc_st.get('ageMs')}ms, nav={nav_st.get('state')}")
                break
            if not fresh_ok and nomotion_client.wait_for_service(timeout_sec=0.1):
                nomotion_client.call_async(Empty.Request())
            time.sleep(0.2)

        # 5. Hardware Arming for Autonomy (POST /api/drive/arm with autonomy: true)
        print("\n[Step 3/4] Arming rover hardware for autonomy (POST /api/drive/arm)...")
        r = requests.post(f"{COCKPIT_URL}/api/drive/arm", json={"token": op_token, "autonomy": True}, headers=headers_op, timeout=3.0)
        arm_res = r.json()
        if not arm_res.get("ok"):
            raise RuntimeError(f"Failed to arm rover: {arm_res}")
        print(f"Hardware armed confirmed: status={arm_res.get('status')}, mode={arm_res.get('mode')}")

        # Verify state is READY_ARMED
        r = requests.get(f"{COCKPIT_URL}/api/autonomy/status", timeout=1.0)
        auton_stat = r.json()
        print(f"Autonomy state: {auton_stat.get('state')}")
        if auton_stat.get("state") != "READY_ARMED":
            raise RuntimeError(f"Expected READY_ARMED but got {auton_stat.get('state')}")

        # 6. Dispatch Nav2 Goal
        print(f"\n[Step 4/4] Dispatching Nav2 Goal to ({target_x:.3f}, {target_y:.3f}, {target_yaw_deg:.2f} deg)...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = float(target_qz)
        goal_msg.pose.pose.orientation.w = float(target_qw)

        send_future = nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the goal!")
        print("Nav2 goal ACCEPTED. Executing trajectory...")

        result_future = goal_handle.get_result_async()

        # 7. Active Monitoring Loop
        timeout_sec = 25.0
        last_log_time = time.time()
        start_time = time.time()

        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.time()
            elapsed = now - start_time

            if elapsed > timeout_sec:
                print(f"\n[WATCHDOG] Execution exceeded maximum duration of {timeout_sec}s! Cancelling goal...")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=2.0)
                reason = "TIMEOUT"
                break

            if (now - last_log_time) >= 0.5:
                last_log_time = now
                try:
                    s_res = requests.get(f"{COCKPIT_URL}/api/status", timeout=0.2)
                    s_data = s_res.json()
                    l_data = s_data.get("localization", {})
                    cx = l_data.get("x", 0.0)
                    cy = l_data.get("y", 0.0)
                    cyaw = l_data.get("yawDeg", 0.0)
                    dist_to_goal = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
                    auton_s = s_data.get("autonomyState", "UNKNOWN")
                    armed_s = s_data.get("armed", False)
                    print(f"[{elapsed:4.1f}s] Pose: ({cx:.3f}, {cy:.3f}, {cyaw:5.1f} deg) | Dist to goal: {dist_to_goal:.3f}m | Auton: {auton_s} | Armed: {armed_s}")
                except Exception:
                    pass

        if result_future.done():
            res_obj = result_future.result()
            status_code = res_obj.status
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                print("\n>>> Nav2 Goal SUCCEEDED! Destination reached within tolerance.")
                reason = "GOAL_REACHED"
                exit_code = 0
            elif status_code == GoalStatus.STATUS_ABORTED:
                print(f"\n>>> Nav2 Goal ABORTED by controller/planner (status={status_code})")
                reason = "NAV2_ABORTED"
                exit_code = 2
            elif status_code == GoalStatus.STATUS_CANCELED:
                print(f"\n>>> Nav2 Goal CANCELED (status={status_code})")
                reason = "NAV2_CANCELED"
                exit_code = 3
            else:
                print(f"\n>>> Nav2 Goal Finished with status code: {status_code}")
                reason = f"STATUS_{status_code}"
                exit_code = status_code

    except Exception as e:
        print(f"\nCRITICAL EXCEPTION during autonomous execution: {e}")
        reason = f"EXCEPTION: {e}"
        exit_code = 1
        # Cancel any active goal handle on exception
        if goal_handle is not None and result_future is not None and not result_future.done():
            print("Cancelling active goal handle due to exception...")
            try:
                cf = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cf, timeout_sec=2.0)
            except Exception as ce:
                print(f"Warning: Failed to cancel goal handle: {ce}")

    finally:
        # 8. Fail-Safe Securing Sequence (Always runs!)
        print("\n==================================================")
        print(" Post-Mission Safety Securing Sequence")
        print("==================================================")
        
        # a) Send zero motion immediately
        try:
            zero_payload = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
            requests.post(INTERNAL_CMD_URL, json=zero_payload, headers=headers_cmd, timeout=1.0)
        except Exception as e:
            print(f"Warning: Failed to send zero cmd_vel: {e}")

        # b) Hardware Disarm via canonical POST /api/drive/disarm
        disarm_req_ok = False
        try:
            print("Sending hardware disarm (POST /api/drive/disarm)...")
            r_disarm = requests.post(f"{COCKPIT_URL}/api/drive/disarm", json={"token": op_token}, headers=headers_op, timeout=2.0)
            if r_disarm.status_code == 200 and r_disarm.json().get("ok"):
                disarm_req_ok = True
                print("Disarm command accepted by Cockpit server.")
            else:
                print(f"Warning: Disarm request returned HTTP {r_disarm.status_code}: {r_disarm.text}")
        except Exception as e:
            print(f"Warning: Disarm request failed: {e}")

        # c) Disable Autonomy
        try:
            print("Disabling autonomy (POST /api/autonomy/disable)...")
            requests.post(f"{COCKPIT_URL}/api/autonomy/disable", json={"token": op_token}, headers=headers_op, timeout=2.0)
        except Exception as e:
            print(f"Warning: Disable autonomy request failed: {e}")

        # d) Verify confirmed hardware disarm from live telemetry
        disarm_confirmed = False
        for _ in range(15):
            try:
                r = requests.get(f"{COCKPIT_URL}/api/status", timeout=0.5)
                st = r.json()
                if st.get("armed") is False and st.get("mode") == 0:
                    disarm_confirmed = True
                    break
            except Exception:
                pass
            time.sleep(0.1)

        if not disarm_confirmed:
            print("CRITICAL WARNING: Hardware disarm was NOT confirmed by ESP32 telemetry within timeout!")
        else:
            print("Hardware confirmed STOPPED, DISARMED, and in Mode 0.")

        # e) Print final telemetry report
        try:
            r = requests.get(f"{COCKPIT_URL}/api/status", timeout=2.0)
            final_status = r.json()
            final_loc = final_status.get("localization", {})
            fx = final_loc.get("x", 0.0)
            fy = final_loc.get("y", 0.0)
            fyaw = final_loc.get("yawDeg", 0.0)
            
            dx = fx - start_x
            dy = fy - start_y
            total_disp = math.sqrt(dx*dx + dy*dy)
            total_duration = time.time() - start_time

            print("\nFinal Vehicle State:")
            print(f"  Armed:            {final_status.get('armed')} ({'CONFIRMED DISARMED' if disarm_confirmed else 'UNCONFIRMED'})")
            print(f"  Mode:             {final_status.get('mode')} (Mode 0: Locked)")
            print(f"  Autonomy State:   {final_status.get('autonomyState')} (DISABLED)")
            print(f"  Command Source:   {final_status.get('cmdSource')}")
            print(f"  Final AMCL Pose:  x={fx:.3f}m, y={fy:.3f}m, yaw={fyaw:.2f} deg")
            print(f"  Net Translation:  {total_disp:.3f}m ({total_disp*3.28084:.2f} ft)")
            print(f"  Mission Duration: {total_duration:.2f}s")
            print(f"  Termination Reason: {reason}")
        except Exception as e:
            print(f"Warning: Could not fetch final telemetry: {e}")

        node.destroy_node()
        rclpy.shutdown()

    return exit_code

if __name__ == '__main__':
    sys.exit(main())
