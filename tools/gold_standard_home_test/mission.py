"""
tools.gold_standard_home_test.mission - Gold Standard HOME Acceptance Test Orchestrator
Deterministic 4-leg mission with strict pre-arm validation, production API dispatch, and fail-closed safety watchdogs.
"""

import os
import sys
import time
import math
import requests
from typing import Dict, Any, Optional, Tuple, Callable

from .constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    NORMAL_LINEAR_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_POS_CORRECTION_CEILING,
    FINAL_YAW_CORRECTION_CEILING,
    PRE_ARM_MAX_POS_ERR_M,
    PRE_ARM_MAX_YAW_ERR_DEG,
    PASS_FINAL_POS_ERR_M,
    PASS_FINAL_YAW_ERR_DEG,
    TIMEOUT_LEG1_FORWARD_S,
    TIMEOUT_LEG2_ROTATION_S,
    TIMEOUT_LEG3_RETURN_S,
    TIMEOUT_LEG4_SETTLE_S,
    TELEMETRY_STALE_TIMEOUT_S,
    COCKPIT_DEFAULT_URL,
    BRIDGE_DEFAULT_URL
)
from .home_loader import load_authoritative_home, wrap_angle_rad, wrap_angle_deg
from .telemetry import TelemetryRecorder, TelemetryFrame
from .grader import MissionGrader

def get_env_token(var_name: str) -> str:
    """Reads credential from supported environment mechanism without copying files."""
    return os.environ.get(var_name, "").strip()

class MissionAbortException(Exception):
    """Raised when a safety invariant or watchdog triggers an immediate abort."""
    pass

class ContinuousYawTracker:
    def __init__(self):
        self.initial_yaw: Optional[float] = None
        self.last_raw_yaw: Optional[float] = None
        self.accumulated_yaw_rad: float = 0.0

    def reset(self):
        self.initial_yaw = None
        self.last_raw_yaw = None
        self.accumulated_yaw_rad = 0.0

    def update(self, raw_yaw_rad: float) -> float:
        if self.initial_yaw is None:
            self.initial_yaw = raw_yaw_rad
            self.last_raw_yaw = raw_yaw_rad
            self.accumulated_yaw_rad = 0.0
            return 0.0

        diff = raw_yaw_rad - self.last_raw_yaw
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi

        self.accumulated_yaw_rad += diff
        self.last_raw_yaw = raw_yaw_rad
        return self.accumulated_yaw_rad

    @property
    def relative_yaw_deg(self) -> float:
        return math.degrees(self.accumulated_yaw_rad)

class GoldStandardMission:
    def __init__(
        self,
        cockpit_url: str = COCKPIT_DEFAULT_URL,
        bridge_url: str = BRIDGE_DEFAULT_URL,
        dry_run: bool = False,
        output_dir: str = "reports/gold_standard_home_test"
    ):
        self.cockpit_url = cockpit_url.rstrip("/")
        self.bridge_url = bridge_url.rstrip("/")
        self.dry_run = dry_run
        self.output_dir = output_dir

        self.op_token = get_env_token("ROVER_OPERATOR_TOKEN")
        self.cmd_token = get_env_token("ROVER_CMD_VEL_TOKEN")

        self.home_pose = load_authoritative_home(cockpit_url=self.cockpit_url)
        self.recorder = TelemetryRecorder(output_dir=self.output_dir)
        self.recorder.metadata["home_pose"] = self.home_pose
        self.recorder.metadata["dry_run"] = self.dry_run

        self.initial_boot_count: Optional[int] = None
        self.last_imu_yaw: Optional[float] = None
        self.initial_checksum_errors: Optional[int] = None
        self.yaw_tracker = ContinuousYawTracker()
        
        self.latest_telemetry: Dict[str, Any] = {
            "amcl": None,
            "odom": None,
            "imu": None,
            "drive": None,
            "cm": None,
            "cmd_raw": {"vx": 0.0, "wz": 0.0},
            "cmd_smooth": {"vx": 0.0, "wz": 0.0},
            "cmd_final": {"vx": 0.0, "wz": 0.0},
            "cmd_source": "NONE",
            "last_packet_monotonic": time.monotonic()
        }

    def compute_mission_targets(self) -> Dict[str, Any]:
        """Calculates exact deterministic coordinates for each leg."""
        x0 = self.home_pose["x"]
        y0 = self.home_pose["y"]
        th0 = self.home_pose["yaw_rad"]

        # Leg 1: Outbound forward along saved HOME heading
        x1 = x0 + FORWARD_DISTANCE_M * math.cos(th0)
        y1 = y0 + FORWARD_DISTANCE_M * math.sin(th0)
        th1 = th0 # Same heading

        # Leg 2: In-place 180° CLOCKWISE rotation (CW reduces yaw by pi)
        th2 = wrap_angle_rad(th1 - math.pi)

        # Leg 3: Return to saved HOME coordinates and finish at exact saved HOME yaw
        x3 = x0
        y3 = y0
        th3 = th0

        return {
            "home": {"x": x0, "y": y0, "yaw_rad": th0, "yaw_deg": self.home_pose["yaw_deg"]},
            "leg1_outbound": {"x": x1, "y": y1, "yaw_rad": th1, "yaw_deg": wrap_angle_deg(math.degrees(th1))},
            "leg2_rotation": {"x": x1, "y": y1, "yaw_rad": th2, "yaw_deg": wrap_angle_deg(math.degrees(th2))},
            "leg3_return": {"x": x3, "y": y3, "yaw_rad": th3, "yaw_deg": wrap_angle_deg(math.degrees(th3))}
        }

    def update_drive_status(self) -> Dict[str, Any]:
        """Polls Cockpit for latest ESP32 drive status."""
        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        r = requests.get(f"{self.cockpit_url}/api/drive/status", headers=headers, timeout=1.0)
        r.raise_for_status()
        data = r.json()
        status = data.get("status", {})
        self.latest_telemetry["drive"] = status
        self.latest_telemetry["cmd_source"] = data.get("cmdSource", "NONE")
        self.latest_telemetry["odom"] = data.get("odom", {})
        self.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        return status

    def poll_all_telemetry(self):
        """Polls Cockpit endpoints to populate 20+ Hz telemetry channels."""
        now = time.monotonic()
        # 1. Drive & Odom
        try:
            self.update_drive_status()
        except Exception:
            pass

        # 2. Localization
        try:
            r_loc = requests.get(f"{self.cockpit_url}/api/localization/status", timeout=0.2)
            if r_loc.status_code == 200:
                loc = r_loc.json()
                self.latest_telemetry["amcl"] = {
                    "x": loc.get("x", 0.0),
                    "y": loc.get("y", 0.0),
                    "yaw_deg": loc.get("yawDeg", 0.0),
                    "yaw_rad": loc.get("yaw", 0.0),
                    "cov_x": loc.get("sigmaX", 0.0),
                    "cov_y": loc.get("sigmaY", 0.0),
                    "cov_yaw": loc.get("sigmaYaw", 0.0),
                    "localized": loc.get("localized", False),
                    "state": loc.get("state", "UNKNOWN")
                }
        except Exception:
            pass

        # 3. IMU
        try:
            r_imu = requests.get(f"{self.cockpit_url}/api/imu", timeout=0.2)
            if r_imu.status_code == 200:
                imu = r_imu.json()
                q = imu.get("orientation", {})
                raw_yaw = 2.0 * math.atan2(q.get("z", 0.0), q.get("w", 1.0))
                self.yaw_tracker.update(raw_yaw)
                self.latest_telemetry["imu"] = {
                    "raw_yaw_deg": math.degrees(raw_yaw),
                    "rel_yaw_deg": self.yaw_tracker.relative_yaw_deg,
                    "gyro_z": imu.get("gyro", {}).get("z", 0.0),
                    "non_magnetic_valid": imu.get("rotVecValid", False),
                    "serialConnected": imu.get("serialConnected", True),
                    "checksumErrors": imu.get("parserStats", {}).get("checksumErrors", 0)
                }
        except Exception:
            pass

        # 4. Clearance / Collision Monitor
        try:
            r_clr = requests.get(f"{self.cockpit_url}/api/clearance", timeout=0.2)
            if r_clr.status_code == 200:
                clr = r_clr.json().get("piComputed", {})
                self.latest_telemetry["cm"] = {
                    "minFwdMm": clr.get("minFwdMm", 0),
                    "minRevMm": clr.get("minRevMm", 0),
                    "clearanceMask": clr.get("clearanceMask", 0)
                }
        except Exception:
            pass

    def check_pre_arm_gate(self, amcl_pose: Dict[str, Any]) -> Tuple[bool, str, float, float]:
        """
        Validates:
        1. AMCL is localized.
        2. Starting pose is within 0.05m and 5° of saved HOME.
        """
        if not amcl_pose:
            return False, "No AMCL pose telemetry received", 999.0, 180.0

        x0 = self.home_pose["x"]
        y0 = self.home_pose["y"]
        th0 = self.home_pose["yaw_rad"]

        cur_x = amcl_pose.get("x", 0.0)
        cur_y = amcl_pose.get("y", 0.0)
        cur_th = amcl_pose.get("yaw_rad", math.radians(amcl_pose.get("yaw_deg", 0.0)))

        pos_err_m = math.hypot(cur_x - x0, cur_y - y0)
        yaw_err_deg = abs(wrap_angle_deg(math.degrees(cur_th - th0)))

        # Check localization state
        try:
            r = requests.get(f"{self.cockpit_url}/api/localization/status", timeout=1.0)
            if r.status_code == 200:
                loc_data = r.json()
                if not loc_data.get("localized", False) or loc_data.get("state") != "LOCALIZED":
                    return False, f"Cockpit localization state is not LOCALIZED ({loc_data.get('state')})", pos_err_m, yaw_err_deg
        except Exception:
            pass

        if pos_err_m > PRE_ARM_MAX_POS_ERR_M:
            return False, f"Initial position error ({pos_err_m*100:.1f} cm) exceeds 5.0 cm ceiling", pos_err_m, yaw_err_deg

        if yaw_err_deg > PRE_ARM_MAX_YAW_ERR_DEG:
            return False, f"Initial yaw error ({yaw_err_deg:.1f}°) exceeds 5.0° ceiling", pos_err_m, yaw_err_deg

        return True, "Pre-arm gate passed", pos_err_m, yaw_err_deg

    def arm(self) -> bool:
        """Arms the drivetrain immediately before mission execution."""
        if self.dry_run:
            print("[DRY-RUN] Drivetrain arming simulated (Hardware remained disarmed).")
            return True

        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        r = requests.post(f"{self.cockpit_url}/api/drive/arm", json={"autonomy": False}, headers=headers, timeout=2.0)
        if r.status_code != 200:
            print(f"[FAIL] Hardware arming rejected: HTTP {r.status_code} {r.text}")
            return False

        # Verify armed mode 3
        stat = self.update_drive_status()
        if not (stat.get("armed") is True and stat.get("mode") == 3):
            print(f"[FAIL] Armed state not confirmed by ESP32: {stat}")
            return False

        self.initial_boot_count = stat.get("bootCount", 1)
        # Record initial checksum errors
        self.poll_all_telemetry()
        imu_stat = self.latest_telemetry.get("imu") or {}
        self.initial_checksum_errors = imu_stat.get("checksumErrors", 0)
        print(f"[OK] Rover armed in Mode 3 (ESP32 boot count: {self.initial_boot_count}).")
        return True

    def disarm_and_stop(self):
        """Universal safety fail-safe: stops all velocity, cancels goals, disarms to Mode 0."""
        print("[SAFETY] Disarming drivetrain, zeroing commands, and locking Mode 0...")
        try:
            self.send_velocity(0.0, 0.0)
        except Exception:
            pass

        try:
            headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
            requests.post(f"{self.cockpit_url}/api/navigation/cancel", headers=headers, timeout=1.0)
        except Exception:
            pass

        try:
            headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
            requests.post(f"{self.cockpit_url}/api/drive/disarm", headers=headers, timeout=1.5)
            requests.post(f"{self.cockpit_url}/api/command-source", json={"source": "NONE"}, headers=headers, timeout=1.0)
        except Exception:
            pass

    def send_velocity(self, vx: float, wz: float, source: str = "EXACT_MOTION"):
        """Sends velocity packet through the authenticated bridge."""
        # Enforce frozen ceilings
        vx_clamped = max(-NORMAL_LINEAR_SPEED, min(NORMAL_LINEAR_SPEED, vx))
        wz_clamped = max(-NORMAL_ANGULAR_SPEED, min(NORMAL_ANGULAR_SPEED, wz))

        if self.dry_run:
            self.latest_telemetry["cmd_final"] = {"vx": vx_clamped, "wz": wz_clamped}
            return

        payload = {
            "linear": {"x": float(vx_clamped), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz_clamped)},
            "source": source
        }
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        requests.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=0.1)
        self.latest_telemetry["cmd_final"] = {"vx": vx_clamped, "wz": wz_clamped}

    def verify_safety_invariants(self, current_stage: str):
        """
        Continuous safety monitor. Trips immediately on:
        - Boot count change (ESP reboot)
        - IMU discontinuity / reset
        - Stale telemetry (> 500ms)
        - Localization loss
        - Serial communication restart
        """
        now = time.monotonic()
        if now - self.latest_telemetry["last_packet_monotonic"] > TELEMETRY_STALE_TIMEOUT_S:
            raise MissionAbortException(f"Stale telemetry: no status packet for > {TELEMETRY_STALE_TIMEOUT_S}s")

        drive_stat = self.latest_telemetry.get("drive") or {}
        if not self.dry_run and self.initial_boot_count is not None:
            curr_bc = drive_stat.get("bootCount")
            if curr_bc is not None and curr_bc != self.initial_boot_count:
                raise MissionAbortException(f"ESP32 hardware reboot detected! (BootCount: {self.initial_boot_count} -> {curr_bc})")

        imu_stat = self.latest_telemetry.get("imu") or {}
        if not self.dry_run:
            if imu_stat.get("serialConnected") is False:
                raise MissionAbortException("Serial communication to ESP32 disconnected!")

        raw_yaw = imu_stat.get("raw_yaw_deg")
        if raw_yaw is not None:
            if self.last_imu_yaw is not None:
                step_delta = abs(wrap_angle_deg(raw_yaw - self.last_imu_yaw))
                if step_delta > 45.0 and abs(imu_stat.get("gyro_z", 0.0)) < 1.0:
                    raise MissionAbortException(f"IMU frame discontinuity detected ({step_delta:.1f}° jump at low gyro)")
            self.last_imu_yaw = raw_yaw

        amcl_stat = self.latest_telemetry.get("amcl") or {}
        if amcl_stat.get("localized") is False or amcl_stat.get("state") not in ("LOCALIZED", "UNKNOWN"):
            raise MissionAbortException(f"Localization lost during {current_stage} (state={amcl_stat.get('state')})")

    def record_tick(self, stage: str):
        """Captures a 20+ Hz synchronized telemetry frame."""
        now = time.monotonic()
        t_rel = round(now - self.recorder.t0, 4)
        
        amcl = self.latest_telemetry.get("amcl") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "cov_x": 0.0, "cov_y": 0.0, "cov_yaw": 0.0, "localized": True}
        odom = self.latest_telemetry.get("odom") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "vx": 0.0, "wz": 0.0, "left_dist": 0.0, "right_dist": 0.0}
        imu = self.latest_telemetry.get("imu") or {"raw_yaw_deg": 0.0, "rel_yaw_deg": 0.0, "gyro_z": 0.0, "non_magnetic_valid": True}
        drive = self.latest_telemetry.get("drive") or {"armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0, "limLinear": 0.0, "limAngular": 0.0, "bootCount": 1, "resetReason": "NONE", "rtcResetReason": "NONE", "lockStatus": False}
        cm = self.latest_telemetry.get("cm") or {"minFwdMm": 9999, "minRevMm": 9999, "clearanceMask": 3}

        pos_err_m = math.hypot(amcl.get("x", 0.0) - self.home_pose["x"], amcl.get("y", 0.0) - self.home_pose["y"])
        yaw_err_deg = wrap_angle_deg(amcl.get("yaw_deg", 0.0) - self.home_pose["yaw_deg"])

        # Motor targets / PWM if available
        cmd_raw = {
            "vx": drive.get("reqLinear", 0.0),
            "wz": drive.get("reqAngular", 0.0)
        }
        cmd_smooth = {
            "vx": drive.get("limLinear", 0.0),
            "wz": drive.get("limAngular", 0.0)
        }

        frame = TelemetryFrame(
            t_rel_s=t_rel,
            timestamp_epoch=time.time(),
            mission_stage=stage,
            raw_cmd=cmd_raw,
            smoothed_cmd=cmd_smooth,
            final_cmd=self.latest_telemetry["cmd_final"],
            cmd_source=self.latest_telemetry["cmd_source"],
            amcl=amcl,
            odom=odom,
            imu=imu,
            drive=drive,
            collision_monitor=cm,
            to_home={
                "pos_err_m": round(pos_err_m, 4),
                "pos_err_cm": round(pos_err_m * 100.0, 2),
                "yaw_err_deg": round(yaw_err_deg, 2)
            }
        )
        self.recorder.record_frame(frame)

    def dispatch_nav2_goal(self, target_x: float, target_y: float, target_yaw: float) -> bool:
        """Dispatches goal to production Nav2 via Cockpit /api/navigation/dispatch."""
        if self.dry_run:
            return True
        payload = {
            "target_x": target_x,
            "target_y": target_y,
            "target_yaw": target_yaw
        }
        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        r = requests.post(f"{self.cockpit_url}/api/navigation/dispatch", json=payload, headers=headers, timeout=5.0)
        return r.status_code == 200

    def execute_leg1_forward(self, target_x: float, target_y: float, target_yaw: float):
        """Leg 1: Drive forward exactly 2.000 ft (0.6096 m) along saved HOME heading."""
        print(f"\n[LEG 1] Dispatching forward 2.000 ft ({FORWARD_DISTANCE_M:.4f} m) -> ({target_x:.4f}, {target_y:.4f})...")
        self.recorder.record_transition("LEG1_FORWARD_START", {"target": (target_x, target_y, target_yaw)})
        
        if not self.dispatch_nav2_goal(target_x, target_y, target_yaw):
            raise MissionAbortException("Leg 1 dispatch rejected by Nav2!")

        t_start = time.monotonic()
        while time.monotonic() - t_start < TIMEOUT_LEG1_FORWARD_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG1_FORWARD")
            self.record_tick("LEG1_FORWARD")

            amcl = self.latest_telemetry.get("amcl") or {}
            cur_dist = math.hypot(amcl.get("x", 0.0) - target_x, amcl.get("y", 0.0) - target_y)
            drive = self.latest_telemetry.get("drive") or {}
            vx = abs(drive.get("limLinear", 0.0))
            wz = abs(drive.get("limAngular", 0.0))

            # Verified completion: within 0.04m and settled to standstill
            if cur_dist <= 0.04 and vx < 0.02 and wz < 0.03:
                print(f"[LEG 1 COMPLETE] Reached turnaround point in {time.monotonic() - t_start:.2f}s (dist err: {cur_dist*100:.2f} cm).")
                self.recorder.record_transition("LEG1_FORWARD_END")
                return

            time.sleep(0.04)

        raise MissionAbortException(f"Leg 1 exceeded {TIMEOUT_LEG1_FORWARD_S}s timeout!")

    def execute_leg2_rotation(self, target_cw_deg: float = 180.0):
        """Leg 2: Perform an explicit 180° CLOCKWISE in-place rotation."""
        print(f"\n[LEG 2] Executing explicit 180.0° CLOCKWISE in-place rotation...")
        self.recorder.record_transition("LEG2_ROTATION_START", {"target_deg": -target_cw_deg})
        self.yaw_tracker.reset()

        t_start = time.monotonic()
        while time.monotonic() - t_start < TIMEOUT_LEG2_ROTATION_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG2_ROTATION")

            # In CW rotation, relative yaw decreases (becomes negative)
            turned_cw = -self.yaw_tracker.relative_yaw_deg
            remaining_deg = target_cw_deg - turned_cw

            if remaining_deg <= 1.0: # Reached
                self.send_velocity(0.0, 0.0)
                print(f"[LEG 2 COMPLETE] 180° CW rotation completed in {time.monotonic() - t_start:.2f}s (Traveled: {turned_cw:.1f}°).")
                self.recorder.record_transition("LEG2_ROTATION_END")
                time.sleep(0.5)
                return

            # Angular approach controller (Cruise -> Creep ceiling)
            if remaining_deg > 30.0:
                wz_cmd = -NORMAL_ANGULAR_SPEED # -0.40 rad/s
            else:
                alpha = max(0.0, min(1.0, remaining_deg / 30.0))
                # Creep ceiling 0.18 rad/s
                wz_cmd = -(FINAL_YAW_CORRECTION_CEILING + alpha * (NORMAL_ANGULAR_SPEED - FINAL_YAW_CORRECTION_CEILING))

            self.send_velocity(0.0, wz_cmd, source="EXACT_MOTION")
            self.record_tick("LEG2_ROTATION")
            time.sleep(0.04)

        raise MissionAbortException(f"Leg 2 exceeded {TIMEOUT_LEG2_ROTATION_S}s timeout!")

    def execute_leg3_return(self, home_x: float, home_y: float, home_yaw: float):
        """Leg 3: Return to the authoritative saved HOME coordinates."""
        print(f"\n[LEG 3] Dispatching return to HOME -> ({home_x:.4f}, {home_y:.4f}, {math.degrees(home_yaw):+.2f}°)...")
        self.recorder.record_transition("LEG3_RETURN_START", {"home": (home_x, home_y, home_yaw)})

        if not self.dispatch_nav2_goal(home_x, home_y, home_yaw):
            raise MissionAbortException("Leg 3 return dispatch rejected by Nav2!")

        t_start = time.monotonic()
        while time.monotonic() - t_start < TIMEOUT_LEG3_RETURN_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG3_RETURN")

            amcl = self.latest_telemetry.get("amcl") or {}
            dist_to_home = math.hypot(amcl.get("x", 0.0) - home_x, amcl.get("y", 0.0) - home_y)
            drive = self.latest_telemetry.get("drive") or {}
            vx = abs(drive.get("limLinear", 0.0))
            wz = abs(drive.get("limAngular", 0.0))

            # Enforce final-position correction ceiling (0.10 m/s) when within 0.25m of HOME
            if dist_to_home <= 0.25 and vx > FINAL_POS_CORRECTION_CEILING:
                self.send_velocity(FINAL_POS_CORRECTION_CEILING, drive.get("limAngular", 0.0))

            self.record_tick("LEG3_RETURN")

            # Verified completion: within 0.04m of HOME coordinates and settled
            if dist_to_home <= PASS_FINAL_POS_ERR_M and vx < 0.02 and wz < 0.03:
                print(f"[LEG 3 COMPLETE] Returned to HOME in {time.monotonic() - t_start:.2f}s (dist err: {dist_to_home*100:.2f} cm).")
                self.recorder.record_transition("LEG3_RETURN_END")
                return

            time.sleep(0.04)

        raise MissionAbortException(f"Leg 3 exceeded {TIMEOUT_LEG3_RETURN_S}s timeout!")

    def execute_leg4_settle(self, home_yaw_rad: float):
        """Leg 4: Final yaw alignment, verification, stop, cancel, and disarm to Mode 0."""
        print(f"\n[LEG 4] Finalizing HOME yaw alignment and settling...")
        self.recorder.record_transition("LEG4_SETTLE_START")

        t_start = time.monotonic()
        while time.monotonic() - t_start < TIMEOUT_LEG4_SETTLE_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG4_SETTLE")

            amcl = self.latest_telemetry.get("amcl") or {}
            cur_th = amcl.get("yaw_rad", math.radians(amcl.get("yaw_deg", 0.0)))
            yaw_err_deg = abs(wrap_angle_deg(math.degrees(cur_th - home_yaw_rad)))

            # Command zero and settle
            self.send_velocity(0.0, 0.0)
            self.record_tick("LEG4_SETTLE")

            if yaw_err_deg <= PASS_FINAL_YAW_ERR_DEG and time.monotonic() - t_start >= 1.5:
                print(f"[LEG 4 COMPLETE] Settled at HOME with yaw error {yaw_err_deg:.2f}° (Limit <= 3.0°).")
                self.recorder.record_transition("LEG4_SETTLE_END")
                return

            time.sleep(0.04)

        print("[LEG 4 WARNING] Settle window expired without reaching yaw tolerance.")
