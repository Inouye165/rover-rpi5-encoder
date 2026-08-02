import paramiko
import os
import sys
import subprocess
import json
import time
import argparse

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

    # Check for uncommitted tracked changes
    code_status, status_out, _ = run_local_cmd("git status --porcelain", cwd=local_root)
    tracked_changes = [line for line in status_out.splitlines() if line and not line.startswith("??")]
    if tracked_changes and not args.allow_dirty:
        print(f"ERROR: Local git working tree contains {len(tracked_changes)} uncommitted tracked changes:", file=sys.stderr)
        for line in tracked_changes[:10]:
            print(f"  {line}", file=sys.stderr)
        print("Aborting deployment! Please commit or stash changes, or use --allow-dirty.", file=sys.stderr)
        sys.exit(1)

    # Verify local HEAD is pushed to origin
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

    # SSH credentials
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

    # Verify remote Pi HEAD
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

    print("\n=== 5. Restarting Cockpit & ROS Services ===")
    exec_remote(client, "sudo systemctl restart rover-server.service", password)
    exec_remote(client, "sudo docker restart rover-ros2", password)

    print("Waiting 5 seconds for services to initialize...")
    time.sleep(5)

    print("\n=== 6. Post-Deployment Verification ===")
    # 1. Active ROS 2 track_width_m parameter check
    ros_param_cmd = "sudo docker exec rover-ros2 bash -c 'source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 param get /rover_encoder_odometry track_width_m'"
    exit_code, param_out, _ = exec_remote(client, ros_param_cmd, password)
    print(f"Active ROS 2 Parameter Output: {param_out.strip()}")
    if "0.718" not in param_out:
        print(f"ERROR: Active ROS parameter verification failed! Expected 0.718, got: {param_out.strip()}", file=sys.stderr)
        client.close()
        sys.exit(1)
    print("✓ Active ROS 2 Parameter Verification: PASS (track_width_m = 0.718)")

    # 2. Cockpit API status check
    status_cmd = "curl -s http://localhost:3000/api/drive/status"
    exit_code, api_out, _ = exec_remote(client, status_cmd)
    try:
        status_json = json.loads(api_out)
        track_width_api = status_json.get("trackWidthM")
        track_source_api = status_json.get("trackWidthSource")
        print(f"Cockpit API Report: trackWidthM={track_width_api}, source={track_source_api}")
        if abs(float(track_width_api) - 0.718) > 1e-4 or track_source_api != "CALIBRATION_DB":
            print(f"ERROR: Cockpit API verification failed! Got trackWidthM={track_width_api}, source={track_source_api}", file=sys.stderr)
            client.close()
            sys.exit(1)
        print("✓ Cockpit API Verification: PASS (trackWidthM = 0.718, source = CALIBRATION_DB)")
    except Exception as e:
        print(f"ERROR: Failed to parse Cockpit API response: {e}\nRaw output: {api_out}", file=sys.stderr)
        client.close()
        sys.exit(1)

    # 3. Final Pi HEAD check
    exit_code, final_pi_head, _ = exec_remote(client, "cd /home/ron/yahboom-encoder && git rev-parse HEAD")
    final_pi_head = final_pi_head.strip()
    if final_pi_head != local_head and not args.allow_dirty:
        print(f"ERROR: Final Pi HEAD ({final_pi_head}) does not match expected local HEAD ({local_head})!", file=sys.stderr)
        client.close()
        sys.exit(1)

    client.close()

    print("\n==================================================")
    print("  DEPLOYMENT & POST-VERIFICATION SUCCESSFUL!")
    print(f"  - Branch:     {current_branch}")
    print(f"  - Local HEAD: {local_head}")
    print(f"  - Pi HEAD:    {final_pi_head}")
    print("==================================================")

if __name__ == "__main__":
    main()
