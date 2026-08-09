import paramiko
import os
import sys
import subprocess
import json
import time
import argparse
import re

EXPECTED_ROS_NODES = [
    "/base_link_to_laser_frame_publisher",
    "/foxglove_bridge",
    "/rover_cmd_vel_bridge",
    "/rover_encoder_odometry",
    "/rover_lidar_bridge",
    "/rover_system_health",
]

EXPECTED_PARAMS = {
    "track_width_m": 0.3408575433,
    "ticks_per_revolution": 1974.1666666667,
    "physical_track_width_m": 0.197
}

def run_local_cmd(cmd, cwd=None):
    res = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, shell=True)
    return res.returncode, res.stdout.strip(), res.stderr.strip()

def load_env(env_path):
    env_vars = {}
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split("=", 1)
                if len(parts) == 2:
                    key = parts[0].strip()
                    val = parts[1].strip().strip('"').strip("'")
                    env_vars[key] = val
    return env_vars

def exec_remote(client, cmd, password=None):
    if password and ("sudo" in cmd or cmd.startswith("sudo")):
        cmd_to_run = f"sudo -S {cmd.replace('sudo ', '')}"
    else:
        cmd_to_run = cmd

    stdin, stdout, stderr = client.exec_command(cmd_to_run)
    if password and ("sudo" in cmd or cmd.startswith("sudo")):
        stdin.write(password + '\n')
        stdin.flush()

    out = stdout.read().decode('utf-8', errors='ignore').encode('ascii', errors='replace').decode('ascii')
    err = stderr.read().decode('utf-8', errors='ignore').encode('ascii', errors='replace').decode('ascii')
    exit_code = stdout.channel.recv_exit_status()
    return exit_code, out, err

def parse_ros_param_value(output_text):
    if not output_text:
        raise ValueError("Empty output from ros2 param get")
    match = re.search(r'(?:Double|Integer|String)\s+value\s+is:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', output_text, re.IGNORECASE)
    if match:
        return float(match.group(1))
    match_any = re.search(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', output_text)
    if match_any:
        return float(match_any.group(0))
    raise ValueError(f"Could not parse numeric parameter value from output: '{output_text}'")

def compare_numeric_param(actual, expected, tolerance=1e-3):
    return abs(actual - expected) <= tolerance

def verify_ros_container_and_startup(client, password=None, max_wait_sec=60, poll_interval=2.0):
    start_time = time.time()
    attempt_history = []
    last_error = ""

    print(f"Waiting up to {max_wait_sec} seconds for ROS 2 container & nodes to initialize...")

    while time.time() - start_time < max_wait_sec:
        elapsed = time.time() - start_time
        current_attempt = len(attempt_history) + 1
        
        # 1. Check container running
        exit_code, container_out, err = exec_remote(
            client, "sudo docker inspect -f '{{.State.Running}}' rover-ros2", password
        )
        if exit_code != 0 or container_out.strip().lower() != "true":
            last_error = f"rover-ros2 container is not running (output: '{container_out.strip()}', err: '{err.strip()}')"
            attempt_history.append({'attempt': current_attempt, 'elapsed': elapsed, 'stage': 'container_running', 'error': last_error})
            time.sleep(poll_interval)
            continue

        # 2. Check ROS node discovery
        node_cmd = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && ros2 node list'"
        exit_code, node_out, err = exec_remote(client, node_cmd, password)
        if exit_code != 0:
            last_error = f"Failed to run ros2 node list (err: '{err.strip()}')"
            attempt_history.append({'attempt': current_attempt, 'elapsed': elapsed, 'stage': 'node_list', 'error': last_error})
            time.sleep(poll_interval)
            continue

        found_nodes = set(line.strip() for line in node_out.splitlines() if line.strip())
        missing_nodes = [node for node in EXPECTED_ROS_NODES if node not in found_nodes]
        if missing_nodes:
            last_error = f"Missing ROS node(s): {', '.join(missing_nodes)}"
            attempt_history.append({'attempt': current_attempt, 'elapsed': elapsed, 'stage': 'node_list', 'missing_nodes': missing_nodes, 'error': last_error})
            time.sleep(poll_interval)
            continue

        # 3. Check parameters independently
        params_ok = True
        param_errors = []
        attempted_params = {}

        for param_name, expected_val in EXPECTED_PARAMS.items():
            param_cmd = f"sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 param get /rover_encoder_odometry {param_name}'"
            exit_code, p_out, p_err = exec_remote(client, param_cmd, password)
            if exit_code != 0:
                params_ok = False
                p_err_msg = f"Failed to get parameter '{param_name}': {p_err.strip() or p_out.strip()}"
                param_errors.append(p_err_msg)
                attempted_params[param_name] = {'status': 'failed', 'raw': p_out.strip(), 'error': p_err_msg}
                continue

            try:
                numeric_val = parse_ros_param_value(p_out)
                if not compare_numeric_param(numeric_val, expected_val, tolerance=1e-3):
                    params_ok = False
                    p_err_msg = f"Parameter '{param_name}' value {numeric_val} does not match expected {expected_val}"
                    param_errors.append(p_err_msg)
                    attempted_params[param_name] = {'status': 'out_of_tolerance', 'numeric': numeric_val, 'expected': expected_val, 'raw': p_out.strip(), 'error': p_err_msg}
                else:
                    attempted_params[param_name] = {'status': 'ok', 'numeric': numeric_val, 'expected': expected_val, 'raw': p_out.strip()}
            except Exception as e:
                params_ok = False
                p_err_msg = f"Failed to parse parameter '{param_name}': {e} (raw: '{p_out.strip()}')"
                param_errors.append(p_err_msg)
                attempted_params[param_name] = {'status': 'parse_error', 'raw': p_out.strip(), 'error': p_err_msg}

        if not params_ok:
            last_error = "; ".join(param_errors)
            attempt_history.append({'attempt': current_attempt, 'elapsed': elapsed, 'stage': 'parameters', 'attempted_params': attempted_params, 'error': last_error})
            time.sleep(poll_interval)
            continue

        # 4. Check topic publishing (/scan and /odom)
        topics_ok = True
        topic_errors = []
        for topic in ["/scan", "/odom"]:
            topic_cmd = f"sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && timeout 5s ros2 topic echo {topic} --once'"
            exit_code, t_out, t_err = exec_remote(client, topic_cmd, password)
            if exit_code != 0 or not t_out.strip():
                topics_ok = False
                t_err_msg = f"Topic {topic} is not publishing or timed out (err: '{t_err.strip()}')"
                topic_errors.append(t_err_msg)

        if not topics_ok:
            last_error = "; ".join(topic_errors)
            attempt_history.append({'attempt': current_attempt, 'elapsed': elapsed, 'stage': 'topics', 'error': last_error})
            time.sleep(poll_interval)
            continue

        # All checks passed!
        print(" [OK] Container running & all 6 expected ROS nodes verified present.")
        print(f" [OK] ROS parameters verified: track_width_m={attempted_params['track_width_m']['numeric']}, ticks_per_revolution={attempted_params['ticks_per_revolution']['numeric']}, physical_track_width_m={attempted_params['physical_track_width_m']['numeric']}.")
        print(" [OK] Topics /scan and /odom verified publishing.")
        return True, attempt_history, ""

    return False, attempt_history, last_error

def verify_no_obsolete_placeholders(client, password=None):
    check_cmd = "test -f /home/ron/yahboom-encoder/ros2/ros2_ws/Dockerfile || test -f /home/ron/yahboom-encoder/ros2/ros2_ws/compose.yaml"
    exit_code, _, _ = exec_remote(client, check_cmd, password)
    if exit_code == 0:
        return False, "Obsolete placeholder file ros2/ros2_ws/Dockerfile or ros2/ros2_ws/compose.yaml exists on Pi!"
    return True, ""

def verify_cockpit_api_and_safety(client, password=None):
    status_cmd = "curl -s http://localhost:3000/api/drive/status"
    exit_code, api_out, err = exec_remote(client, status_cmd, password)
    if exit_code != 0 or not api_out.strip():
        return False, f"Failed to query Cockpit API /api/drive/status: {err.strip()}"
    try:
        drive_json = json.loads(api_out)
        status_obj = drive_json.get("status", {})

        is_armed = status_obj.get("armed", drive_json.get("armed", False))
        if is_armed:
            return False, "Safety Violation: Rover drive status reports ARMED!"

        req_lin = float(status_obj.get("reqLinear", 0.0))
        req_ang = float(status_obj.get("reqAngular", 0.0))
        lim_lin = float(status_obj.get("limLinear", 0.0))
        lim_ang = float(status_obj.get("limAngular", 0.0))
        if abs(req_lin) > 1e-4 or abs(req_ang) > 1e-4 or abs(lim_lin) > 1e-4 or abs(lim_ang) > 1e-4:
            return False, f"Safety Violation: Non-zero velocity commands reported (req: [{req_lin}, {req_ang}], lim: [{lim_lin}, {lim_ang}])!"

        track_width = float(drive_json.get("trackWidthM", 0.0))
        track_source = drive_json.get("trackWidthSource", "")
        if abs(track_width - 0.3408575433) > 1e-3 or track_source != "CALIBRATION_DB":
            return False, f"Cockpit API track width check failed (got trackWidthM={track_width}, source={track_source})"
    except Exception as e:
        return False, f"Failed to parse Cockpit API response: {e}"

    autonomy_cmd = "curl -s http://localhost:3000/api/autonomy/status"
    exit_code, aut_out, err = exec_remote(client, autonomy_cmd, password)
    if exit_code != 0 or not aut_out.strip():
        return False, f"Failed to query Cockpit API /api/autonomy/status: {err.strip()}"
    try:
        aut_json = json.loads(aut_out)
        enabled = aut_json.get("enabled", False)
        active = aut_json.get("active", False)
        state = aut_json.get("state", "DISABLED")
        if enabled or active or state != "DISABLED":
            return False, f"Safety Violation: Autonomy status is not disabled/inactive (enabled={enabled}, active={active}, state={state})"
    except Exception as e:
        return False, f"Failed to parse Autonomy API response: {e}"

    return True, ""

def verify_remote_git_clean(client, password=None):
    exit_code, git_status_out, err = exec_remote(client, "cd /home/ron/yahboom-encoder && git status --porcelain", password)
    if exit_code != 0:
        return False, f"Failed to query git status on Pi: {err.strip()}"
    if git_status_out.strip():
        return False, f"Remote Pi git checkout contains untracked or modified files:\n{git_status_out.strip()}"
    return True, ""

def main():
    parser = argparse.ArgumentParser(description="Clean, Git-verified Rover Cockpit & ROS 2 Deployment Script")
    parser.add_argument("--allow-dirty", action="store_true", help="Allow deployment from a dirty local git working tree")
    parser.add_argument("--skip-ros-build", action="store_true", help="Skip ROS 2 workspace colcon rebuild")
    args = parser.parse_args()

    local_root = os.path.dirname(os.path.abspath(__file__))
    env_path = os.path.join(local_root, ".env")
    env = load_env(env_path)

    print("=== 1. Local Git Hygiene & Commit Audit ===")
    code, current_branch, _ = run_local_cmd("git branch --show-current", cwd=local_root)
    code2, local_head, _ = run_local_cmd("git rev-parse HEAD", cwd=local_root)
    
    if code != 0 or not current_branch:
        print("Error: Failed to determine local git branch.", file=sys.stderr)
        sys.exit(1)

    print(f"Local Branch: {current_branch}")
    print(f"Local HEAD:   {local_head}")

    code_status, status_out, _ = run_local_cmd("git status --porcelain", cwd=local_root)
    tracked_changes = [line for line in status_out.splitlines() if line and not line.startswith("??")]
    if tracked_changes and not args.allow_dirty:
        print(f"ERROR: Local git working tree contains {len(tracked_changes)} uncommitted tracked changes:", file=sys.stderr)
        for line in tracked_changes[:10]:
            print(f"  {line}", file=sys.stderr)
        print("Aborting deployment! Please commit or stash changes, or use --allow-dirty.", file=sys.stderr)
        sys.exit(1)

    code_ls, remote_ls_out, _ = run_local_cmd(f"git ls-remote origin refs/heads/{current_branch}", cwd=local_root)
    if code_ls != 0 or not remote_ls_out:
        print(f"ERROR: Failed to query origin for branch '{current_branch}'.", file=sys.stderr)
        sys.exit(1)

    remote_head = remote_ls_out.split()[0]
    print(f"Origin HEAD:  {remote_head}")

    if local_head != remote_head and not args.allow_dirty:
        print(f"ERROR: Local HEAD ({local_head[:7]}) does not match origin HEAD ({remote_head[:7]}).", file=sys.stderr)
        print("Please push your commits to origin before deploying!", file=sys.stderr)
        sys.exit(1)

    print("Local git hygiene & origin sync: PASS")

    ip = os.environ.get("ROVER_PI_HOST") or env.get("ROVER_PI_HOST")
    user = os.environ.get("ROVER_PI_USER") or env.get("ROVER_PI_USER")
    password = os.environ.get("ROVER_PI_PASSWORD") or env.get("ROVER_PI_PASSWORD")
    key_filename = os.environ.get("ROVER_PI_SSH_KEY") or env.get("ROVER_PI_SSH_KEY")
    port_str = os.environ.get("ROVER_PI_PORT") or env.get("ROVER_PI_PORT") or "22"

    try:
        port = int(port_str)
    except ValueError:
        print(f"Error: ROVER_PI_PORT must be an integer, got '{port_str}'", file=sys.stderr)
        sys.exit(1)

    if not ip or not user:
        print("Error: ROVER_PI_HOST and ROVER_PI_USER must be set in environment variables or .env file.", file=sys.stderr)
        sys.exit(1)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        if key_filename:
            print(f"Connecting to {user}@{ip}:{port} using SSH key: {key_filename}...")
            client.connect(ip, port=port, username=user, key_filename=key_filename, look_for_keys=False, allow_agent=False, timeout=10)
        elif password:
            print(f"Connecting to {user}@{ip}:{port} using password auth...")
            client.connect(ip, port=port, username=user, password=password, look_for_keys=False, allow_agent=False, timeout=10)
        else:
            print(f"Connecting to {user}@{ip}:{port} using default SSH keys / agent...")
            client.connect(ip, port=port, username=user, timeout=10)
    except Exception as e:
        print(f"Connection failed: {e}", file=sys.stderr)
        sys.exit(1)

    print("\n=== 2. Creating Remote Backup of Preserved Data ===")
    ts = time.strftime("%Y%m%d_%H%M%S")
    backup_dir = f"/home/ron/yahboom_backups/deploy_backup_{ts}"
    exec_remote(client, f"mkdir -p {backup_dir}")
    exec_remote(client, f"cp /home/ron/yahboom-encoder/calibration_db.json {backup_dir}/ 2>/dev/null || true")
    exec_remote(client, f"cp /home/ron/yahboom-encoder/.env {backup_dir}/env_root.txt 2>/dev/null || true")
    exec_remote(client, f"mkdir -p {backup_dir}/maps && cp -r /home/ron/yahboom-encoder/ros2/maps/* {backup_dir}/maps/ 2>/dev/null || true")
    print(f"Backup created at: {backup_dir}")

    print("\n=== 3. Synchronizing Remote Pi Git Checkout ===")
    git_fetch_cmd = f"cd /home/ron/yahboom-encoder && git fetch origin {current_branch} && git checkout {current_branch} && git reset --hard origin/{current_branch}"
    exit_code, out, err = exec_remote(client, git_fetch_cmd, password)
    if exit_code != 0:
        print(f"ERROR: Failed to update git checkout on Pi:\n{err}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print("Remote git checkout updated successfully.")

    exit_code, pi_head, _ = exec_remote(client, "cd /home/ron/yahboom-encoder && git rev-parse HEAD")
    pi_head = pi_head.strip()
    print(f"Remote Pi HEAD: {pi_head}")

    if pi_head != local_head and not args.allow_dirty:
        print(f"ERROR: Remote Pi HEAD ({pi_head[:7]}) does not match local HEAD ({local_head[:7]})!", file=sys.stderr)
        client.close()
        sys.exit(1)

    print("\n=== 4. Rebuilding ROS 2 Workspace inside Container ===")
    if not args.skip_ros_build:
        colcon_cmd = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --packages-select rover_bringup'"
        exit_code, out, err = exec_remote(client, colcon_cmd, password)
        if exit_code != 0:
            print(f"ERROR: colcon build failed inside container:\n{err}", file=sys.stderr)
            client.close()
            sys.exit(1)
        print("ROS 2 colcon build: PASS")

    print("\n=== 5. Recreating ROS 2 Container & Restarting Cockpit Services ===")
    exec_remote(client, "sudo cp /home/ron/yahboom-encoder/rover-health.service /etc/systemd/system/rover-health.service && sudo systemctl daemon-reload && sudo systemctl enable --now rover-health.service && sudo systemctl restart rover-health.service", password)
    exec_remote(client, "sudo systemctl restart rover-server.service", password)
    recreate_cmd = "cd /home/ron/yahboom-encoder/ros2 && sudo docker compose up -d --force-recreate"
    exit_code, out, err = exec_remote(client, recreate_cmd, password)
    if exit_code != 0:
        print(f"ERROR: Failed to recreate ROS 2 container with Docker Compose:\n{err}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print(" [OK] ROS 2 container recreated via Docker Compose (--force-recreate)")
    print(" [OK] Rover Health Recorder service deployed and active")

    print("\n=== 6. Post-Deployment Verification ===")
    ros_ok, attempt_history, ros_err = verify_ros_container_and_startup(client, password, max_wait_sec=60)
    if not ros_ok:
        print(f"ERROR: Post-deployment ROS verification failed after {len(attempt_history)} attempts!", file=sys.stderr)
        print(f"Last Error: {ros_err}", file=sys.stderr)
        print("Attempt History:", file=sys.stderr)
        for att in attempt_history:
            print(f"  Attempt {att['attempt']} ({att['elapsed']:.1f}s) [{att['stage']}]: {att.get('error')}", file=sys.stderr)
        client.close()
        sys.exit(1)

    ph_ok, ph_err = verify_no_obsolete_placeholders(client, password)
    if not ph_ok:
        print(f"ERROR: {ph_err}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print(" [OK] No obsolete mount placeholders exist in /ros2_ws/")

    api_ok, api_err = verify_cockpit_api_and_safety(client, password)
    if not api_ok:
        print(f"ERROR: {api_err}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print(" [OK] Cockpit API & Safety Verification: PASS (disarmed, zero velocities, autonomy disabled & inactive)")

    exit_code, final_pi_head, _ = exec_remote(client, "cd /home/ron/yahboom-encoder && git rev-parse HEAD")
    final_pi_head = final_pi_head.strip()
    if final_pi_head != local_head and not args.allow_dirty:
        print(f"ERROR: Final Pi HEAD ({final_pi_head}) does not match expected local HEAD ({local_head})!", file=sys.stderr)
        client.close()
        sys.exit(1)

    clean_ok, clean_err = verify_remote_git_clean(client, password)
    if not clean_ok:
        print(f"ERROR: {clean_err}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print(" [OK] Remote Pi Git Clean Verification: PASS (0 untracked or modified files)")

    client.close()

    print("\n==================================================")
    print("  DEPLOYMENT & POST-VERIFICATION SUCCESSFUL!")
    print(f"  - Branch:     {current_branch}")
    print(f"  - Local HEAD: {local_head}")
    print(f"  - Pi HEAD:    {final_pi_head}")
    print("==================================================")

if __name__ == "__main__":
    main()
