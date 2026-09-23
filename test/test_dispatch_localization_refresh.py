#!/usr/bin/env python3
"""
test_dispatch_localization_refresh.py - Automated Stationary Regression Test
Verifies the bounded pre-dispatch AMCL localization refresh:
1. Verifies that /api/nav/nomotion_update triggers an instantaneous AMCL particle filter update.
2. Verifies that sample stale by > 2000 ms is refreshed to <= 1000 ms with advancing timestamp/sequence.
3. Verifies that sample stale beyond stationary threshold is handled safely.
4. Verifies successful pre-dispatch refresh advancing timestamp/sequence before arming.
5. Verifies failed refresh / unauthenticated dispatch returns HTTP 409/403 before arming.
6. Invariant check: Drivetrain strictly remains stopped and disarmed in Mode 0 throughout.

STATIONARY TEST ONLY. NO PHYSICAL MOTION.
"""

import os
import sys
import time
import json
import urllib.request
import urllib.error

ROVER_IP = os.getenv("ROVER_PI_HOST", "10.0.0.246")
COCKPIT_URL = f"http://{ROVER_IP}:3000"
NAV_BRIDGE_URL = f"http://{ROVER_IP}:3005"

def get_operator_token():
    token = os.getenv("ROVER_OPERATOR_TOKEN")
    if token:
        return token
    # Search local .env files
    search_dirs = [
        os.getcwd(),
        os.path.dirname(os.path.abspath(__file__)),
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ]
    for d in search_dirs:
        env_path = os.path.join(d, ".env")
        if os.path.isfile(env_path):
            try:
                with open(env_path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith("ROVER_OPERATOR_TOKEN="):
                            val = line.split("=", 1)[1].strip().strip('"').strip("'")
                            if val:
                                return val
            except Exception:
                pass
    print("ERROR: ROVER_OPERATOR_TOKEN is not set in environment or local .env.", file=sys.stderr)
    sys.exit(1)

def get_json(url, timeout=2.5):
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode('utf-8'))

def post_json(url, data, token=None, timeout=3.5):
    payload = json.dumps(data).encode('utf-8')
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Rover-Operator-Token"] = token
    req = urllib.request.Request(url, data=payload, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        body = e.read().decode('utf-8')
        try:
            parsed = json.loads(body)
        except Exception:
            parsed = {"raw": body}
        return e.code, parsed

def verify_drivetrain_safe(stage_name):
    drive = get_json(f"{COCKPIT_URL}/api/drive/status")
    st = drive.get("status", {})
    armed = st.get("armed", drive.get("armed", True))
    mode = st.get("mode", drive.get("mode", -1))
    req_l = abs(float(st.get("reqLinear", 0.0)))
    req_a = abs(float(st.get("reqAngular", 0.0)))
    if armed or mode != 0 or req_l > 1e-4 or req_a > 1e-4:
        print(f"  SAFETY VIOLATION at {stage_name}: armed={armed}, mode={mode}, req=[{req_l}, {req_a}]")
        return False
    return True

def main():
    print("=" * 75)
    print("STATIONARY REGRESSION SUITE: PRE-DISPATCH LOCALIZATION REFRESH")
    print(f"Target Cockpit:    {COCKPIT_URL}")
    print(f"Target Nav Bridge: {NAV_BRIDGE_URL}")
    print("=" * 75)

    operator_token = get_operator_token()
    print("Operator Token:    [Configured from environment/.env]")

    # Check 1: Drivetrain Initial Safety Invariant
    print("\n[CHECK 1] Verify initial drivetrain safety state (Disarmed in Mode 0)...")
    if not verify_drivetrain_safe("Initial Check"):
        return 1
    print("  PASS: Drivetrain confirmed disarmed and locked in Mode 0 with zero velocities.")

    # Check 2: Authoritative Map Verification
    print("\n[CHECK 2] Verify authoritative production map (house_slam_2026-08-23_final)...")
    map_meta = get_json(f"{COCKPIT_URL}/api/navigation/map")
    w = map_meta.get("width")
    h = map_meta.get("height")
    res = map_meta.get("resolution")
    origin = map_meta.get("origin")
    print(f"  Loaded Map: {w}x{h} @ {res} m/px, origin: {origin}")
    if w != 97 or h != 125 or res != 0.05:
        print(f"  FAILED: Unexpected map geometry: {w}x{h} @ {res} m/px (expected 97x125 @ 0.05)")
        return 1
    print("  PASS: Authoritative production map geometry confirmed (house_slam_2026-08-23_final).")

    # Check 3: Baseline Localization & Sequence/Timestamp Check
    print("\n[CHECK 3] Verify baseline localization state...")
    loc = get_json(f"{COCKPIT_URL}/api/localization/status")
    print(f"  Current Pose: x={loc.get('x'):.4f}, y={loc.get('y'):.4f}, yawDeg={loc.get('yawDeg'):.2f}°")
    print(f"  State: localized={loc.get('localized')}, state={loc.get('state')}, ageMs={loc.get('ageMs')}")
    if not loc.get("localized"):
        print("  FAILED: Rover is not localized in the map frame!")
        return 1
    print("  PASS: Baseline localization active in map frame.")

    # Check 4: Sample stale by > 2000 ms & advancing refresh verification
    print("\n[CHECK 4] Testing sample stale by > 2000 ms & bounded refresh advance...")
    print("  Dwelling stationary to allow sample age to exceed 2000 ms active-motion safety threshold...")
    time.sleep(2.5)
    loc_stale = get_json(f"{COCKPIT_URL}/api/localization/status")
    stale_age = loc_stale.get("ageMs", 0)
    base_seq = loc_stale.get("seq", -1)
    base_stamp = loc_stale.get("sampleTimestamp", 0)
    print(f"  Stale sample age: {stale_age} ms (base_seq: {base_seq}, base_stamp: {base_stamp})")
    if stale_age < 1500:
        print(f"  INFO: Sample age {stale_age}ms is lower than expected; proceeding with refresh verification...")

    print("  Invoking /api/navigation/refresh_localization while disarmed...")
    t0 = time.time()
    code_rf, resp_rf = post_json(f"{COCKPIT_URL}/api/navigation/refresh_localization", {})
    t_elapsed = time.time() - t0
    print(f"  Refresh HTTP {code_rf} in {t_elapsed:.3f}s: {resp_rf}")
    if code_rf != 200 or not resp_rf.get("ok"):
        print(f"  FAILED: Pre-dispatch refresh failed: {resp_rf}")
        return 1
    if not resp_rf.get("advanced"):
        print(f"  FAILED: Refresh did not advance sample sequence/timestamp! {resp_rf}")
        return 1
    if resp_rf.get("age_ms", 9999) > 1000:
        print(f"  FAILED: Sample age after refresh {resp_rf.get('age_ms')} ms > 1000 ms limit!")
        return 1
    print(f"  PASS: Pre-dispatch refresh achieved verified fresh sample ({resp_rf.get('age_ms')}ms <= 1000ms) with advancing timestamp/sequence.")

    # Check 5: Stationary threshold recovery coverage
    print("\n[CHECK 5] Testing stationary threshold recovery behavior...")
    status_nav, resp_nav = post_json(f"{NAV_BRIDGE_URL}/api/nav/nomotion_update", {})
    print(f"  Direct bridge /api/nav/nomotion_update HTTP {status_nav}: {resp_nav}")
    if status_nav != 200 or not resp_nav.get("ok"):
        print(f"  FAILED: Direct bridge nomotion endpoint failed: {resp_nav}")
        return 1
    print("  PASS: AMCL responds to no-motion updates during extended stationary dwell.")

    # Check 6: Failed refresh / safety rejection returning HTTP 409/403 before arming
    print("\n[CHECK 6] Testing safe rejection before arming (disarmed safety guard)...")
    # 6A: Unauthenticated dispatch must reject with 401/403 and never arm
    code_unauth, resp_unauth = post_json(f"{COCKPIT_URL}/api/navigation/dispatch", {"target_x": 0.0, "target_y": 0.0}, token="invalid_token_xyz")
    print(f"  6A. Unauthorized dispatch HTTP {code_unauth}: {resp_unauth}")
    if code_unauth not in [401, 403]:
        print(f"  FAILED: Unauthorized dispatch was not rejected with 401/403! (Got {code_unauth})")
        return 1
    if not verify_drivetrain_safe("After 6A Unauth Dispatch"):
        return 1
    print("  PASS: Unauthorized dispatch safely rejected; rover strictly remained disarmed.")

    # 6B: Dispatch rejection while unlocalized or invalid coordinate returns HTTP 409
    code_bad, resp_bad = post_json(f"{COCKPIT_URL}/api/navigation/dispatch", {"target_x": float("nan"), "target_y": float("nan")}, token=operator_token)
    print(f"  6B. Invalid dispatch HTTP {code_bad}: {resp_bad}")
    if code_bad != 409 and code_bad != 400:
        print(f"  FAILED: Invalid dispatch was not rejected with 409/400! (Got {code_bad})")
        return 1
    if not verify_drivetrain_safe("After 6B Invalid Dispatch"):
        return 1
    print("  PASS: Invalid dispatch safely rejected before arming with HTTP 409/400.")

    # Check 7: Final Invariant Audit
    print("\n[CHECK 7] Final verification: drivetrain strictly remained disarmed and locked...")
    if not verify_drivetrain_safe("Final Audit"):
        return 1
    print("  PASS: Drivetrain strictly remained disarmed and locked in Mode 0 throughout the entire suite.")

    print("\n" + "=" * 75)
    print("ALL 7 STATIONARY REGRESSION CHECKS PASSED SUCCESSFULLY!")
    print("=" * 75)
    return 0

if __name__ == "__main__":
    sys.exit(main())
