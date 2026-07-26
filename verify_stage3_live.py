# ==============================================================================
# verify_stage3_live.py — Hardened Live Read-Only Verification Script
# ==============================================================================

import os
import sys
import json
import socket
import urllib.request
import urllib.error
import hashlib
import subprocess

ROVER_HOST = "10.0.0.246"
ROVER_USER = "ron"
COCKPIT_PORT = 3000
ODOM_PORT = 3003
FOXGLOVE_PORT = 8765

LOCAL_ROOT = r"C:\Users\Ron\electronic_projects\yahboom-encoder"
PI_ROOT = "/home/ron/yahboom-encoder"
VERIFY_FILES = [
    "public/index.html",
    "public/app.js",
    "public/style.css"
]


def sha256_file(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Local file not found: {filepath}")
    with open(filepath, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def fetch_bytes(url, timeout=5):
    req = urllib.request.Request(url, headers={"User-Agent": "RoverVerification/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status != 200:
                raise urllib.error.HTTPError(url, resp.status, f"HTTP Status {resp.status}", resp.headers, None)
            return resp.read()
    except Exception as e:
        raise RuntimeError(f"HTTP fetch failed for {url}: {e}") from e


def fetch_json(url, timeout=5):
    raw_data = fetch_bytes(url, timeout=timeout)
    try:
        parsed = json.loads(raw_data.decode("utf-8"))
        if not isinstance(parsed, dict):
            raise ValueError(f"Expected JSON object (dict), got {type(parsed).__name__}")
        return parsed
    except Exception as e:
        raise ValueError(f"Invalid JSON response from {url}: {e}") from e


def get_remote_hash(host, user, remote_path, timeout=5):
    cmd = [
        "ssh",
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=5",
        f"{user}@{host}",
        f"sha256sum {remote_path}"
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=True)
        out = proc.stdout.strip()
        parts = out.split()
        if not parts:
            raise RuntimeError(f"Empty output from ssh sha256sum on {remote_path}")
        return parts[0]
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"SSH timeout connecting to {user}@{host}")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"SSH command failed ({e.returncode}): {e.stderr.strip()}")
    except Exception as e:
        raise RuntimeError(f"SSH execution error: {e}")


def verify_hashes(local_root, pi_root, host, user, file_list, timeout=5):
    results = {}
    all_match = True

    for rel_path in file_list:
        local_path = os.path.join(local_root, rel_path)
        pi_path = f"{pi_root}/{rel_path.replace(os.sep, '/')}"
        http_url = f"http://{host}:{COCKPIT_PORT}/{rel_path.replace('public/', '').replace(os.sep, '/')}"

        # 1. Local Hash
        try:
            local_hash = sha256_file(local_path)
        except Exception as e:
            results[rel_path] = {"pass": False, "error": f"Local file error: {e}"}
            all_match = False
            continue

        # 2. Remote Pi Hash
        try:
            pi_hash = get_remote_hash(host, user, pi_path, timeout=timeout)
        except Exception as e:
            results[rel_path] = {"pass": False, "error": f"Pi SSH hash error: {e}"}
            all_match = False
            continue

        # 3. HTTP Hash
        try:
            http_bytes = fetch_bytes(http_url, timeout=timeout)
            http_hash = hashlib.sha256(http_bytes).hexdigest()
        except Exception as e:
            results[rel_path] = {"pass": False, "error": f"HTTP fetch error: {e}"}
            all_match = False
            continue

        is_match = (local_hash == pi_hash == http_hash)
        if not is_match:
            all_match = False

        results[rel_path] = {
            "pass": is_match,
            "local_hash": local_hash,
            "pi_hash": pi_hash,
            "http_hash": http_hash,
            "error": None if is_match else "Hash mismatch across local, Pi, or HTTP"
        }

    return all_match, results


def verify_drive_status(data):
    if not isinstance(data, dict):
        return False, "Data is not a dict"

    # Extract status object
    st = data.get("status")
    if not isinstance(st, dict):
        st = data

    # 1. Check armed
    armed = st.get("armed")
    if armed is not False:
        return False, f"armed must be False (got {armed})"

    # 2. Check motorCommand / velocities
    if "motorCommand" in st:
        mc = st.get("motorCommand")
        if mc != [0, 0, 0, 0]:
            return False, f"motorCommand must be [0,0,0,0] (got {mc})"
    else:
        req_lin = st.get("reqLinear", 0)
        req_ang = st.get("reqAngular", 0)
        lim_lin = st.get("limLinear", 0)
        lim_ang = st.get("limAngular", 0)
        if req_lin != 0 or req_ang != 0 or lim_lin != 0 or lim_ang != 0:
            return False, f"Nonzero velocities: req=({req_lin},{req_ang}), lim=({lim_lin},{lim_ang})"

    # 3. Check cmdSource / source
    if "cmdSource" in st:
        src = st.get("cmdSource")
        if src != "NONE":
            return False, f"cmdSource must be 'NONE' (got '{src}')"
    elif "source" in st:
        src = st.get("source")
        if src != 0 and src != "NONE":
            return False, f"source must be 0/'NONE' (got {src})"
    else:
        return False, "Missing cmdSource/source field in drive status"

    return True, "Nominal: Disarmed, motor commands zeroed, cmdSource NONE"


def verify_calibration_status(data):
    if not isinstance(data, dict):
        return False, "Response is not a dict"

    if data.get("ok") is not True:
        return False, f"Response 'ok' field must be True (got {data.get('ok')})"

    st = data.get("status")
    if not isinstance(st, dict):
        return False, "Response missing nested 'status' dict"

    if st.get("phase") != "IDLE":
        return False, f"phase must be 'IDLE' (got '{st.get('phase')}')"

    if st.get("active") is not False:
        return False, f"active must be False (got {st.get('active')})"

    if st.get("armed") is not False:
        return False, f"armed must be False (got {st.get('armed')})"

    if st.get("motorCommand") != [0, 0, 0, 0]:
        return False, f"motorCommand must be [0,0,0,0] (got {st.get('motorCommand')})"

    if st.get("fault") is not None:
        return False, f"fault must be None (got {st.get('fault')})"

    sc = st.get("safetyChecks")
    if not isinstance(sc, dict):
        return False, "Missing nested 'safetyChecks' dict"

    required_checks = ["serialConnected", "telemetryValid", "odomValid", "limitsOk"]
    for check_key in required_checks:
        if sc.get(check_key) is not True:
            return False, f"safetyCheck '{check_key}' must be True (got {sc.get(check_key)})"

    return True, "Nominal: Auto-calibration IDLE, armed=False, motorCommand=[0,0,0,0], safety checks True"


def verify_lidar_status(data):
    if not isinstance(data, dict):
        return False, "Data is not a dict"

    if data.get("connected") is not True:
        return False, f"connected must be True (got {data.get('connected')})"

    if data.get("state") != "scanning":
        return False, f"state must be 'scanning' (got '{data.get('state')}')"

    if data.get("health") not in ["Good", "OK"]:
        return False, f"health must be 'Good' or 'OK' (got '{data.get('health')}')"

    pt_count = data.get("latestScanPointCount")
    if not isinstance(pt_count, int) or pt_count <= 0:
        return False, f"latestScanPointCount must be positive int (got {pt_count})"

    scan_hz = data.get("scanHz")
    if not isinstance(scan_hz, (int, float)) or scan_hz <= 0:
        return False, f"scanHz must be positive number (got {scan_hz})"

    scan_age = data.get("lastScanAgeMs")
    if not isinstance(scan_age, (int, float)) or scan_age < 0 or scan_age >= 2000:
        return False, f"lastScanAgeMs must be numeric and < 2000 ms (got {scan_age})"

    if data.get("lastError") is not None:
        return False, f"lastError must be None (got '{data.get('lastError')}')"

    return True, f"Nominal: LiDAR scanning at {scan_hz} Hz, {pt_count} points, age {scan_age} ms"


def verify_odometry_status(data):
    if not isinstance(data, dict):
        return False, "Data is not a dict"

    if data.get("ok") is not True:
        return False, f"ok must be True (got {data.get('ok')})"

    if data.get("node_health") != "ok":
        return False, f"node_health must be 'ok' (got '{data.get('node_health')}')"

    odom_age = data.get("odometry_age_ms")
    if not isinstance(odom_age, (int, float)) or odom_age < 0 or odom_age >= 2000:
        return False, f"odometry_age_ms must be numeric and < 2000 ms (got {odom_age})"

    return True, f"Nominal: Odometry node_health='ok', age {odom_age} ms"


def verify_foxglove(host, port, timeout=3):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((host, port))
        s.close()
        return True, f"Foxglove TCP port {port} open and responsive"
    except Exception as e:
        return False, f"Foxglove TCP port {port} failed connection: {e}"


def main():
    print("==================================================================")
    print("      STAGE 3 HARDENED LIVE COCKPIT READ-ONLY VERIFICATION       ")
    print("==================================================================")

    checks = {}
    overall_pass = True

    # 1. Hashes Verification
    print("\n[1/6] Verifying SHA-256 Hashes (Local vs Pi SSH vs HTTP)...")
    hash_pass, hash_details = verify_hashes(LOCAL_ROOT, PI_ROOT, ROVER_HOST, ROVER_USER, VERIFY_FILES)
    checks["hashes"] = (hash_pass, hash_details)
    if not hash_pass:
        overall_pass = False
    for fname, details in hash_details.items():
        if details["pass"]:
            print(f"  [PASS] {fname}: Hash MATCH ({details['local_hash'][:16]}...)")
        else:
            print(f"  [FAIL] {fname}: FAIL — {details['error']}")

    # 2. Drive Status Verification
    print("\n[2/6] Verifying Drive Status (/api/drive/status)...")
    try:
        drive_json = fetch_json(f"http://{ROVER_HOST}:{COCKPIT_PORT}/api/drive/status")
        drive_pass, drive_msg = verify_drive_status(drive_json)
    except Exception as e:
        drive_pass, drive_msg = False, f"Fetch/Parse Error: {e}"

    checks["drive"] = (drive_pass, drive_msg)
    if not drive_pass:
        overall_pass = False
    print(f"  [{'PASS' if drive_pass else 'FAIL'}]: {drive_msg}")

    # 3. Calibration Status Verification
    print("\n[3/6] Verifying Calibration Status (/api/calibration/auto/status)...")
    try:
        cal_json = fetch_json(f"http://{ROVER_HOST}:{COCKPIT_PORT}/api/calibration/auto/status")
        cal_pass, cal_msg = verify_calibration_status(cal_json)
    except Exception as e:
        cal_pass, cal_msg = False, f"Fetch/Parse Error: {e}"

    checks["calibration"] = (cal_pass, cal_msg)
    if not cal_pass:
        overall_pass = False
    print(f"  [{'PASS' if cal_pass else 'FAIL'}]: {cal_msg}")

    # 4. LiDAR Status Verification
    print("\n[4/6] Verifying LiDAR Status (/api/lidar/status)...")
    try:
        lidar_json = fetch_json(f"http://{ROVER_HOST}:{COCKPIT_PORT}/api/lidar/status")
        lidar_pass, lidar_msg = verify_lidar_status(lidar_json)
    except Exception as e:
        lidar_pass, lidar_msg = False, f"Fetch/Parse Error: {e}"

    checks["lidar"] = (lidar_pass, lidar_msg)
    if not lidar_pass:
        overall_pass = False
    print(f"  [{'PASS' if lidar_pass else 'FAIL'}]: {lidar_msg}")

    # 5. Odometry Node Status Verification
    print("\n[5/6] Verifying Odometry Node (http://10.0.0.246:3003/api/odom)...")
    try:
        odom_json = fetch_json(f"http://{ROVER_HOST}:{ODOM_PORT}/api/odom")
        odom_pass, odom_msg = verify_odometry_status(odom_json)
    except Exception as e:
        odom_pass, odom_msg = False, f"Fetch/Parse Error: {e}"

    checks["odometry"] = (odom_pass, odom_msg)
    if not odom_pass:
        overall_pass = False
    print(f"  [{'PASS' if odom_pass else 'FAIL'}]: {odom_msg}")

    # 6. Foxglove Port Verification
    print("\n[6/6] Verifying Foxglove TCP Port (10.0.0.246:8765)...")
    fox_pass, fox_msg = verify_foxglove(ROVER_HOST, FOXGLOVE_PORT)
    checks["foxglove"] = (fox_pass, fox_msg)
    if not fox_pass:
        overall_pass = False
    print(f"  [{'PASS' if fox_pass else 'FAIL'}]: {fox_msg}")

    print("\n==================================================================")
    print("                     VERIFICATION SUMMARY TABLE                   ")
    print("==================================================================")
    print(f"  1. File Hash Parity:     {'PASS' if hash_pass else 'FAIL'}")
    print(f"  2. Drive Safety State:   {'PASS' if drive_pass else 'FAIL'}")
    print(f"  3. Calibration Idle:     {'PASS' if cal_pass else 'FAIL'}")
    print(f"  4. LiDAR Status:         {'PASS' if lidar_pass else 'FAIL'}")
    print(f"  5. Odometry Health:      {'PASS' if odom_pass else 'FAIL'}")
    print(f"  6. Foxglove Port 8765:   {'PASS' if fox_pass else 'FAIL'}")
    print("------------------------------------------------------------------")

    if overall_pass:
        print("RESULT: ALL STAGE 3 HARDENED VERIFICATION CHECKS PASSED (EXIT 0)")
        sys.exit(0)
    else:
        print("RESULT: VERIFICATION FAILED (EXIT 1)")
        sys.exit(1)


if __name__ == "__main__":
    main()
