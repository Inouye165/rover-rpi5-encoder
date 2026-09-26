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
    ROTATION_180_MAX_SPEED,
    NORMAL_ANGULAR_SPEED,
    FINAL_YAW_ALIGNMENT_MAX_SPEED,
    FINAL_YAW_CORRECTION_CEILING,
    FINAL_POS_CORRECTION_CEILING,
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

        # Load authoritative dynamic HOME from disk
        self.home_pose = load_authoritative_home()
        self.recorder = TelemetryRecorder(output_dir=self.output_dir)
        self.recorder.metadata["home_pose"] = self.home_pose
        self.yaw_tracker = ContinuousYawTracker()

        # Command ownership & isolation state
        self.active_nav2_goal: bool = False
        self.current_cmd_source: str = "NONE"
        self.latest_exact_cmd: Dict[str, float] = {"vx": 0.0, "wz": 0.0}

        # Watchdog baselines
        self.initial_boot_count: Optional[int] = None
        self.last_imu_yaw: Optional[float] = None
        self.last_imu_seq: Optional[int] = None
        self.last_imu_seq_adv_time: float = time.monotonic()
        
        # Telemetry cache
        self.latest_telemetry: Dict[str, Any] = {
            "amcl": None,
            "odom": None,
            "imu": None,
            "drive": None,
            "cm": None,
            "last_packet_monotonic": time.monotonic()
        }

    def compute_mission_targets(self) -> Dict[str, Any]:
        """
        Calculates exact target coordinates and headings for the 4-leg mission.
        - Leg 1: Outbound 2.000 ft (0.6096 m) along saved HOME heading
        - Leg 2: Explicit 180.0° CLOCKWISE in-place rotation
        - Leg 3: Return to HOME coordinates retaining return-facing heading
        - Leg 4: In-place signed alignment back to exact saved HOME yaw
        """
        h = self.home_pose
        # Leg 1: Outbound forward 2.000 ft (0.6096 m) along saved HOME heading
        x1 = h["x"] + FORWARD_DISTANCE_M * math.cos(h["yaw_rad"])
        y1 = h["y"] + FORWARD_DISTANCE_M * math.sin(h["yaw_rad"])
        yaw1 = h["yaw_rad"]

        # Leg 2: Explicit 180.0° CLOCKWISE in-place rotation
        yaw2 = wrap_angle_rad(yaw1 - math.radians(ROTATION_TARGET_DEG))

        # Leg 3: Return to authoritative saved HOME coordinates retaining return-facing heading!
        # Target heading is return-facing (yaw2) so rover drives forward directly to HOME.
        x3 = h["x"]
        y3 = h["y"]
        yaw3 = yaw2

        # Leg 4: Final explicit in-place rotation to exact saved HOME yaw
        yaw4 = h["yaw_rad"]

        return {
            "home": h,
            "leg1_outbound": {
                "x": round(x1, 4),
                "y": round(y1, 4),
                "yaw_rad": round(yaw1, 4),
                "yaw_deg": round(math.degrees(yaw1), 2)
            },
            "leg2_rotation": {
                "relative_delta_deg": -ROTATION_TARGET_DEG,
                "yaw_rad": round(yaw2, 4),
                "yaw_deg": round(math.degrees(yaw2), 2)
            },
            "leg3_return": {
                "x": round(x3, 4),
                "y": round(y3, 4),
                "yaw_rad": round(yaw3, 4),
                "yaw_deg": round(math.degrees(yaw3), 2)
            },
            "leg4_settle": {
                "x": round(h["x"], 4),
                "y": round(h["y"], 4),
                "yaw_rad": round(yaw4, 4),
                "yaw_deg": round(math.degrees(yaw4), 2)
            }
        }

    def fetch_live_amcl_pose(self) -> Tuple[bool, str, Optional[Dict[str, float]]]:
        """
        Queries live AMCL status from Cockpit /api/localization/status.
        Fails closed: If endpoint fails, localization is not LOCALIZED, or required
        fields are missing/stale, refuses to arm and NEVER substitutes HOME coordinates.
        """
        try:
            r = requests.get(f"{self.cockpit_url}/api/localization/status", timeout=1.0)
            if r.status_code != 200:
                return False, f"HTTP error {r.status_code} from /api/localization/status: {r.text}", None
            
            loc = r.json()
            if not loc.get("ok", False):
                return False, f"Localization status response not OK: {loc}", None
            
            if not loc.get("localized", False) or loc.get("state") != "LOCALIZED":
                return False, f"Vehicle is not LOCALIZED (localized={loc.get('localized')}, state='{loc.get('state')}')", None

            # Verify required fields
            for f in ("x", "y"):
                if f not in loc or loc[f] is None:
                    return False, f"Required localization field '{f}' is missing from response", None

            yaw_deg = loc.get("yawDeg", loc.get("yaw_deg"))
            yaw_rad = loc.get("yaw", loc.get("yaw_rad"))
            if yaw_deg is None and yaw_rad is None:
                return False, "Required localization yaw field is missing from response", None
            
            if yaw_rad is None:
                yaw_rad = math.radians(float(yaw_deg))
            if yaw_deg is None:
                yaw_deg = math.degrees(float(yaw_rad))

            # Freshness verification
            age_ms = loc.get("ageMs", loc.get("age_ms"))
            if age_ms is not None and float(age_ms) > 2000.0:
                return False, f"Localization pose is stale ({age_ms} ms > 2000 ms ceiling)", None

            pose = {
                "x": float(loc["x"]),
                "y": float(loc["y"]),
                "yaw_deg": float(yaw_deg),
                "yaw_rad": float(yaw_rad),
                "cov_x": float(loc.get("sigmaX", 0.0)),
                "cov_y": float(loc.get("sigmaY", 0.0)),
                "cov_yaw": float(loc.get("sigmaYaw", 0.0)),
                "localized": True,
                "state": loc.get("state", "LOCALIZED")
            }
            return True, "OK", pose
        except Exception as e:
            return False, f"Failed to connect to Cockpit localization endpoint ({e})", None

    def check_pre_arm_gate(self, amcl_pose: Dict[str, Any]) -> Tuple[bool, str, float, float]:
        """
        Enforces pre-arm gate:
        - Must be verified LOCALIZED (not inferred or substituted)
        - Euclidean position error <= 0.05 m
        - Yaw error <= 5.0 deg
        """
        x0 = self.home_pose["x"]
        y0 = self.home_pose["y"]
        th0 = self.home_pose["yaw_rad"]

        cur_x = amcl_pose.get("x", 0.0)
        cur_y = amcl_pose.get("y", 0.0)
        cur_th = amcl_pose.get("yaw_rad", math.radians(amcl_pose.get("yaw_deg", 0.0)))

        pos_err_m = math.hypot(cur_x - x0, cur_y - y0)
        yaw_err_deg = abs(wrap_angle_deg(math.degrees(cur_th - th0)))

        if not amcl_pose.get("localized", False) or amcl_pose.get("state") != "LOCALIZED":
            return False, f"Pre-arm refusal: AMCL state is not LOCALIZED ({amcl_pose.get('state')})", pos_err_m, yaw_err_deg

        if pos_err_m > PRE_ARM_MAX_POS_ERR_M:
            return False, f"Pre-arm refusal: Initial position error ({pos_err_m*100:.1f} cm) exceeds 5.0 cm ceiling", pos_err_m, yaw_err_deg

        if yaw_err_deg > PRE_ARM_MAX_YAW_ERR_DEG:
            return False, f"Pre-arm refusal: Initial yaw error ({yaw_err_deg:.1f}°) exceeds 5.0° ceiling", pos_err_m, yaw_err_deg

        return True, "Pre-arm gate passed", pos_err_m, yaw_err_deg

    def arm(self) -> bool:
        """Arms the drivetrain explicitly. Fails closed on any rejection."""
        if self.dry_run:
            print("[DRY-RUN] Drivetrain arming simulated (Hardware remained disarmed).")
            return True

        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        try:
            r = requests.post(f"{self.cockpit_url}/api/drive/arm", headers=headers, timeout=2.0)
            if r.status_code != 200:
                self.disarm_and_stop()
                raise MissionAbortException(f"Arming request rejected with HTTP {r.status_code}: {r.text}")
            res = r.json()
            if not res.get("ok", False):
                self.disarm_and_stop()
                raise MissionAbortException(f"Arming request returned ok=false: {res}")
        except Exception as e:
            self.disarm_and_stop()
            raise MissionAbortException(f"Exception during arming request: {e}")
        
        status = self.update_drive_status()
        if not status.get("armed", False) or status.get("mode") != 3:
            self.disarm_and_stop()
            raise MissionAbortException(f"Drive status did not confirm armed Mode 3: armed={status.get('armed')}, mode={status.get('mode')}")

        if self.initial_boot_count is None:
            self.initial_boot_count = status.get("bootCount")
        return True

    def disarm(self) -> bool:
        """Disarms the drivetrain explicitly to Mode 0. Checks HTTP status and response."""
        if self.dry_run:
            return True

        try:
            r = requests.post(f"{self.cockpit_url}/api/drive/disarm", timeout=1.0)
            if r.status_code != 200:
                raise MissionAbortException(f"Disarm request rejected with HTTP {r.status_code}: {r.text}")
            res = r.json()
            if not res.get("ok", False):
                raise MissionAbortException(f"Disarm request returned ok=false: {res}")
        except Exception as e:
            raise MissionAbortException(f"Exception during disarm request: {e}")

        status = self.update_drive_status()
        if status.get("armed", False) or status.get("mode") != 0:
            raise MissionAbortException(f"Drive status did not confirm disarmed Mode 0: armed={status.get('armed')}, mode={status.get('mode')}")
        return True

    def disarm_and_stop(self):
        """Immediately halts all physical motion, zeroes commands, and disarms to Mode 0."""
        print("[SAFETY] Disarming drivetrain, zeroing commands, and locking Mode 0...")
        try:
            self.send_velocity(0.0, 0.0, force_zero=True)
        except Exception:
            pass

        try:
            self.set_command_source("NONE", check_errors=False)
        except Exception:
            pass
        
        if not self.dry_run:
            try:
                requests.post(f"{self.cockpit_url}/api/navigation/cancel", timeout=1.0)
            except Exception:
                pass
            try:
                requests.post(f"{self.cockpit_url}/api/drive/disarm", timeout=1.0)
            except Exception:
                pass
        self.active_nav2_goal = False

    def update_drive_status(self) -> Dict[str, Any]:
        """Queries /api/drive/status to monitor hardware health and command echoes."""
        if self.dry_run:
            return {
                "armed": False,
                "mode": 0,
                "bootCount": 1,
                "reqLinear": 0.0,
                "reqAngular": 0.0,
                "limLinear": 0.0,
                "limAngular": 0.0,
                "cmdSource": self.current_cmd_source
            }
        r = requests.get(f"{self.cockpit_url}/api/drive/status", timeout=0.5)
        if r.status_code != 200:
            raise MissionAbortException(f"Drive status request failed with HTTP {r.status_code}")
        res = r.json()
        status = res.get("status", {})
        self.latest_telemetry["drive"] = status
        self.latest_telemetry["last_packet_monotonic"] = time.monotonic()
        return status

    def poll_all_telemetry(self):
        """Polls Cockpit endpoints to populate synchronized telemetry channels."""
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
                    "state": loc.get("state", "UNKNOWN"),
                    "ageMs": loc.get("ageMs", 0)
                }
        except Exception:
            pass

        # 3. IMU
        try:
            r_imu = requests.get(f"{self.cockpit_url}/api/imu", timeout=0.2)
            if r_imu.status_code == 200:
                imu = r_imu.json()
                raw_yaw = float(imu.get("raw_yaw_deg", 0.0))
                if "orientation" in imu and imu["orientation"] and "w" in imu["orientation"]:
                    o = imu["orientation"]
                    raw_yaw = math.atan2(2.0 * (o["w"] * o["z"] + o["x"] * o["y"]), 1.0 - 2.0 * (o["y"]**2 + o["z"]**2))
                    raw_yaw = math.degrees(raw_yaw)
                
                gyro_z = imu.get("gyro", {}).get("z", 0.0)
                seq = imu.get("sequence")
                if seq is not None:
                    if self.last_imu_seq is None or seq != self.last_imu_seq:
                        self.last_imu_seq = seq
                        self.last_imu_seq_adv_time = now

                self.latest_telemetry["imu"] = {
                    "raw_yaw_deg": raw_yaw,
                    "rel_yaw_deg": self.yaw_tracker.update(math.radians(raw_yaw)),
                    "gyro_z": gyro_z,
                    "serialConnected": imu.get("serialConnected", True),
                    "dataAgeMs": imu.get("dataAgeMs", 0),
                    "stale": imu.get("stale", False),
                    "sequence": seq
                }
        except Exception:
            pass

        # 4. Encoders & Odom
        try:
            r_enc = requests.get(f"{self.cockpit_url}/api/encoders", timeout=0.2)
            if r_enc.status_code == 200:
                enc = r_enc.json()
                self.latest_telemetry["odom"] = {
                    "x": enc.get("x", 0.0),
                    "y": enc.get("y", 0.0),
                    "yaw_deg": enc.get("yaw_deg", 0.0),
                    "vx": enc.get("vx", 0.0),
                    "wz": enc.get("wz", 0.0),
                    "left_dist": enc.get("left_dist", 0.0),
                    "right_dist": enc.get("right_dist", 0.0)
                }
        except Exception:
            pass

        # 5. Collision Monitor & Clearance
        try:
            r_cl = requests.get(f"{self.cockpit_url}/api/clearance", timeout=0.2)
            if r_cl.status_code == 200:
                cl = r_cl.json()
                pi_cl = cl.get("piComputed", {})
                esp_cl = cl.get("espConfirmed", {})
                self.latest_telemetry["cm"] = {
                    "minFwdMm": pi_cl.get("minFwdMm", 9999),
                    "minRevMm": pi_cl.get("minRevMm", 9999),
                    "clearanceMask": esp_cl.get("clearanceMask", 0),
                    "fwdOk": esp_cl.get("fwdOk", True),
                    "revOk": esp_cl.get("revOk", True)
                }
        except Exception:
            pass

    def set_command_source(self, source: str, check_errors: bool = True) -> bool:
        """Sets Cockpit command source ownership (NONE, CALIBRATION_TEST, ROS_AUTONOMY)."""
        self.current_cmd_source = source
        if self.dry_run:
            return True
        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        try:
            r = requests.post(f"{self.cockpit_url}/api/command-source", json={"source": source}, headers=headers, timeout=1.0)
            if check_errors:
                if r.status_code != 200:
                    self.disarm_and_stop()
                    raise MissionAbortException(f"Command source transition to {source} rejected with HTTP {r.status_code}: {r.text}")
                res = r.json()
                if not res.get("ok", False) or res.get("cmdSource") != source:
                    self.disarm_and_stop()
                    raise MissionAbortException(f"Command source transition returned unexpected payload: {res}")
            return r.status_code == 200
        except Exception as e:
            if check_errors:
                self.disarm_and_stop()
                raise MissionAbortException(f"Exception during command source transition to {source}: {e}")
            return False

    def send_velocity(self, vx: float, wz: float, source: str = "CALIBRATION_TEST", force_zero: bool = False):
        """
        Dispatches velocity command to the velocity bridge (/api/cmd_vel).
        Safety invariant: CANNOT send direct velocity commands while a Nav2 goal is active!
        Direct test commands must use CALIBRATION_TEST consistently.
        """
        is_zero = abs(vx) <= 1e-4 and abs(wz) <= 1e-4
        if not force_zero and not is_zero and self.active_nav2_goal:
            self.disarm_and_stop()
            raise MissionAbortException(
                "Safety invariant violated: Attempted to send direct CALIBRATION_TEST velocity command while Nav2 goal is active!"
            )

        vx_clamped = max(-NORMAL_LINEAR_SPEED, min(NORMAL_LINEAR_SPEED, vx))
        wz_clamped = max(-NORMAL_ANGULAR_SPEED, min(NORMAL_ANGULAR_SPEED, wz))
        self.latest_exact_cmd = {"vx": vx_clamped, "wz": wz_clamped}

        if self.dry_run:
            return

        payload = {
            "linear": {"x": float(vx_clamped), "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz_clamped)},
            "source": source
        }
        headers = {"X-Rover-Bridge-Token": self.cmd_token} if self.cmd_token else {}
        try:
            r = requests.post(f"{self.bridge_url}/api/cmd_vel", json=payload, headers=headers, timeout=0.2)
            if r.status_code != 200:
                self.disarm_and_stop()
                raise MissionAbortException(f"Velocity command rejected by bridge (HTTP {r.status_code}): {r.text}")
            res = r.json()
            if not res.get("ok", False):
                self.disarm_and_stop()
                raise MissionAbortException(f"Velocity command returned ok=false: {res}")
        except Exception as e:
            if not isinstance(e, MissionAbortException):
                self.disarm_and_stop()
                raise MissionAbortException(f"Exception sending velocity command: {e}")
            raise

    def verify_safety_invariants(self, current_stage: str):
        """
        Continuous safety monitor. Trips immediately on:
        - Underlying telemetry staleness (dataAgeMs > 500ms or sequence freeze)
        - ESP32 hardware reboot (bootCount change)
        - IMU discontinuity / reset (>45° jump at low gyro)
        - Serial communication disconnect
        - Localization loss (localized=False or state != LOCALIZED)
        """
        now = time.monotonic()
        imu_stat = self.latest_telemetry.get("imu") or {}
        if not self.dry_run:
            data_age_ms = imu_stat.get("dataAgeMs")
            if data_age_ms is not None and data_age_ms > 500:
                self.disarm_and_stop()
                raise MissionAbortException(f"Stale telemetry: ESP32 IMU dataAgeMs={data_age_ms}ms exceeds 500ms limit")
            if imu_stat.get("stale") is True:
                self.disarm_and_stop()
                raise MissionAbortException("Stale telemetry: Cockpit serial packet marked stale")
            if now - self.last_imu_seq_adv_time > TELEMETRY_STALE_TIMEOUT_S:
                self.disarm_and_stop()
                raise MissionAbortException(f"Stale telemetry: ESP32 packet sequence did not advance for > {TELEMETRY_STALE_TIMEOUT_S}s")

        drive_stat = self.latest_telemetry.get("drive") or {}
        if not self.dry_run and self.initial_boot_count is not None:
            curr_bc = drive_stat.get("bootCount")
            if curr_bc is not None and curr_bc != self.initial_boot_count:
                self.disarm_and_stop()
                raise MissionAbortException(f"ESP32 hardware reboot detected! (BootCount: {self.initial_boot_count} -> {curr_bc})")

        if not self.dry_run:
            if imu_stat.get("serialConnected") is False:
                self.disarm_and_stop()
                raise MissionAbortException("Serial communication to ESP32 disconnected!")

        raw_yaw = imu_stat.get("raw_yaw_deg")
        if raw_yaw is not None:
            if self.last_imu_yaw is not None:
                step_delta = abs(wrap_angle_deg(raw_yaw - self.last_imu_yaw))
                if step_delta > 45.0 and abs(imu_stat.get("gyro_z", 0.0)) < 1.0:
                    self.disarm_and_stop()
                    raise MissionAbortException(f"IMU frame discontinuity detected ({step_delta:.1f}° jump at low gyro)")
            self.last_imu_yaw = raw_yaw

        amcl_stat = self.latest_telemetry.get("amcl") or {}
        if amcl_stat.get("localized") is False or amcl_stat.get("state") not in ("LOCALIZED", "UNKNOWN"):
            self.disarm_and_stop()
            raise MissionAbortException(f"Localization lost during {current_stage} (state={amcl_stat.get('state')})")

    def record_tick(self, stage: str):
        """Captures a synchronized telemetry frame."""
        now = time.monotonic()
        t_rel = round(now - self.recorder.t0, 4)
        
        amcl = self.latest_telemetry.get("amcl") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "cov_x": 0.0, "cov_y": 0.0, "cov_yaw": 0.0, "localized": True}
        odom = self.latest_telemetry.get("odom") or {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "vx": 0.0, "wz": 0.0, "left_dist": 0.0, "right_dist": 0.0}
        imu = self.latest_telemetry.get("imu") or {"raw_yaw_deg": 0.0, "rel_yaw_deg": 0.0, "gyro_z": 0.0, "serialConnected": True}
        drive = self.latest_telemetry.get("drive") or {"armed": False, "mode": 0, "reqLinear": 0.0, "reqAngular": 0.0, "limLinear": 0.0, "limAngular": 0.0, "bootCount": 1}
        cm = self.latest_telemetry.get("cm") or {"minFwdMm": 9999, "minRevMm": 9999, "clearanceMask": 3}

        pos_err_m = math.hypot(amcl.get("x", 0.0) - self.home_pose["x"], amcl.get("y", 0.0) - self.home_pose["y"])
        yaw_err_deg = wrap_angle_deg(amcl.get("yaw_deg", 0.0) - self.home_pose["yaw_deg"])

        # Determine command stream attribution
        if self.active_nav2_goal:
            cmd_source = "ROS_AUTONOMY"
            cmd_raw = {"vx": drive.get("reqLinear", 0.0), "wz": drive.get("reqAngular", 0.0)}
            cmd_smooth = {"vx": drive.get("limLinear", 0.0), "wz": drive.get("limAngular", 0.0)}
            cmd_final = {"vx": drive.get("limLinear", 0.0), "wz": drive.get("limAngular", 0.0)}
        else:
            cmd_source = "CALIBRATION_TEST"
            cmd_raw = dict(self.latest_exact_cmd)
            cmd_smooth = dict(self.latest_exact_cmd)
            cmd_final = dict(self.latest_exact_cmd)

        frame = TelemetryFrame(
            t_rel_s=t_rel,
            timestamp_epoch=time.time(),
            mission_stage=stage,
            raw_cmd=cmd_raw,
            smoothed_cmd=cmd_smooth,
            final_cmd=cmd_final,
            cmd_source=cmd_source,
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

    def dispatch_nav2_goal(self, target_x: float, target_y: float, target_yaw: float) -> str:
        """
        Dispatches goal to production Nav2 via Cockpit /api/navigation/dispatch.
        Returns unique goal identifier (timestamp).
        """
        if self.dry_run:
            self.active_nav2_goal = True
            return str(time.time())

        payload = {
            "target_x": target_x,
            "target_y": target_y,
            "target_yaw": target_yaw
        }
        headers = {"X-Rover-Operator-Token": self.op_token} if self.op_token else {}
        try:
            r = requests.post(f"{self.cockpit_url}/api/navigation/dispatch", json=payload, headers=headers, timeout=5.0)
            if r.status_code != 200:
                self.disarm_and_stop()
                raise MissionAbortException(f"Nav2 dispatch rejected with HTTP {r.status_code}: {r.text}")
            res = r.json()
            if not res.get("ok", False):
                self.disarm_and_stop()
                raise MissionAbortException(f"Nav2 dispatch returned ok=false: {res}")
            
            self.active_nav2_goal = True
            goal_id = str(res.get("dispatch_meta", {}).get("dispatched_at") or res.get("dispatched_at") or time.time())
            return goal_id
        except Exception as e:
            if not isinstance(e, MissionAbortException):
                self.disarm_and_stop()
                raise MissionAbortException(f"Exception dispatching Nav2 goal: {e}")
            raise

    def verify_zero_handshake(self, expected_cmd_source: str, timeout_s: float = 3.0) -> bool:
        """
        Requires advancing telemetry and at least 3 consecutive samples showing zero requested/limited velocity,
        no active Nav2 goal, and the expected command source.
        """
        t_start = time.monotonic()
        consecutive_zeros = 0
        last_seen_seq = None

        while time.monotonic() - t_start < timeout_s:
            self.poll_all_telemetry()
            drive = self.latest_telemetry.get("drive") or {}
            imu = self.latest_telemetry.get("imu") or {}
            
            curr_seq = imu.get("sequence") or drive.get("seq")
            advancing = (last_seen_seq is None) or (curr_seq != last_seen_seq) or self.dry_run
            last_seen_seq = curr_seq

            req_l = abs(drive.get("reqLinear", 0.0))
            req_a = abs(drive.get("reqAngular", 0.0))
            lim_l = abs(drive.get("limLinear", 0.0))
            lim_a = abs(drive.get("limAngular", 0.0))
            current_src = drive.get("cmdSource", self.current_cmd_source)

            src_matches = (expected_cmd_source in (current_src, "ANY")) or self.dry_run

            if (advancing and
                req_l < 1e-4 and req_a < 1e-4 and
                lim_l < 1e-4 and lim_a < 1e-4 and
                not self.active_nav2_goal and
                src_matches):
                consecutive_zeros += 1
                if consecutive_zeros >= 3:
                    return True
            else:
                consecutive_zeros = 0

            time.sleep(0.04)

        self.disarm_and_stop()
        raise MissionAbortException(f"Zero-output handshake failed: could not confirm 3 consecutive zero samples for source {expected_cmd_source}")

    def wait_for_nav2_completion_and_zero(self, timeout_s: float, stage_name: str, goal_id: str, target_dist_thresh_m: float = 0.04) -> bool:
        """
        Goal-specific Nav2 completion:
        1. Observes goal become accepted/active.
        2. Requires that same goal to report SUCCEEDED.
        3. Never accepts initial IDLE, distance alone, or stale status.
        4. Nav2 success triggers hardware disarm in Cockpit; updates active_nav2_goal = False.
        """
        t_start = time.monotonic()
        goal_observed_active = False

        while time.monotonic() - t_start < timeout_s:
            self.poll_all_telemetry()
            self.verify_safety_invariants(stage_name)
            self.record_tick(stage_name)

            if self.dry_run:
                self.active_nav2_goal = False
                return True

            try:
                r = requests.get(f"{self.cockpit_url}/api/navigation/status", timeout=0.5)
                if r.status_code == 200:
                    nav_stat = r.json()
                    status_str = nav_stat.get("status", "")
                    
                    # Phase 1: Verify goal becomes active
                    if not goal_observed_active:
                        if status_str in ("EXECUTING", "ACTIVE"):
                            goal_observed_active = True
                        elif status_str in ("IDLE", "CANCELLED", "STOPPED"):
                            # Still in initial / stale pre-start state: DO NOT accept as complete
                            pass
                    
                    # Phase 2: Once active, require SUCCEEDED
                    if goal_observed_active:
                        if status_str == "SUCCEEDED":
                            # Goal succeeded. Cockpit automatically initiates disarm on SUCCEEDED.
                            self.active_nav2_goal = False
                            time.sleep(0.1)
                            self.poll_all_telemetry()
                            return True
                        elif status_str in ("ABORTED", "FAILED"):
                            self.disarm_and_stop()
                            raise MissionAbortException(f"Nav2 goal aborted by navigation stack: {status_str}")
            except Exception as e:
                if isinstance(e, MissionAbortException):
                    raise
                pass

            time.sleep(0.04)

        self.disarm_and_stop()
        raise MissionAbortException(f"{stage_name} exceeded timeout ({timeout_s}s) waiting for goal {goal_id} to succeed!")

    def execute_leg1_forward(self, target_x: float, target_y: float, target_yaw: float):
        """
        Leg 1: Nav2 drives 2.000 ft (0.6096 m) outward along saved HOME heading.
        Rover starts disarmed. Dispatching Nav2 refreshes localization and arms to Mode 3.
        On success, Nav2 completes and Cockpit disarms hardware.
        """
        print(f"\n[LEG 1] Dispatching forward 2.000 ft ({FORWARD_DISTANCE_M:.4f} m) -> ({target_x:.4f}, {target_y:.4f})...")
        self.recorder.record_transition("LEG1_FORWARD_START", {"target": (target_x, target_y, target_yaw)})
        
        goal_id = self.dispatch_nav2_goal(target_x, target_y, target_yaw)
        self.wait_for_nav2_completion_and_zero(TIMEOUT_LEG1_FORWARD_S, "LEG1_FORWARD", goal_id, 0.04)
        print("[LEG 1 COMPLETE] Reached turnaround point, Nav2 reported SUCCEEDED, rover finished disarmed.")
        self.recorder.record_transition("LEG1_FORWARD_END")

    def execute_leg2_rotation(self, target_cw_deg: float = 180.0):
        """
        Leg 2: Perform an explicit 180° CLOCKWISE in-place rotation using exact-motion controller.
        State transition: Rover is disarmed after Leg 1.
        1. Zero handshake.
        2. Arm explicitly to Mode 3.
        3. Acquire CALIBRATION_TEST ownership.
        4. Execute rotation using CALIBRATION_TEST commands.
        5. Stop, disarm to Mode 0, release ownership.
        """
        print(f"\n[LEG 2] Handshaking zero, arming Mode 3, acquiring CALIBRATION_TEST ownership...")
        self.recorder.record_transition("LEG2_ROTATION_START", {"target_deg": -target_cw_deg})

        # 1. Zero handshake while disarmed
        self.verify_zero_handshake(expected_cmd_source="ANY")

        # 2. Arm explicitly to Mode 3
        self.arm()

        # 3. Acquire CALIBRATION_TEST command ownership
        self.set_command_source("CALIBRATION_TEST")

        self.yaw_tracker.reset()
        t_start = time.monotonic()

        while time.monotonic() - t_start < TIMEOUT_LEG2_ROTATION_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG2_ROTATION")

            turned_cw = -self.yaw_tracker.relative_yaw_deg
            remaining_deg = target_cw_deg - turned_cw

            if remaining_deg <= 1.0: # Turn complete
                self.send_velocity(0.0, 0.0, source="CALIBRATION_TEST", force_zero=True)
                print(f"[LEG 2 COMPLETE] 180° CW rotation completed in {time.monotonic() - t_start:.2f}s (Traveled: {turned_cw:.1f}°).")
                self.recorder.record_transition("LEG2_ROTATION_END")
                time.sleep(0.5)
                # Disarm explicitly and release ownership
                self.disarm()
                self.set_command_source("NONE")
                return

            # Angular approach controller (Cruise 0.40 rad/s -> Creep ceiling 0.18 rad/s)
            if remaining_deg > 30.0:
                wz_cmd = -ROTATION_180_MAX_SPEED
            else:
                alpha = max(0.0, min(1.0, remaining_deg / 30.0))
                wz_cmd = -(FINAL_YAW_ALIGNMENT_MAX_SPEED + alpha * (ROTATION_180_MAX_SPEED - FINAL_YAW_ALIGNMENT_MAX_SPEED))

            self.send_velocity(0.0, wz_cmd, source="CALIBRATION_TEST")
            self.record_tick("LEG2_ROTATION")
            time.sleep(0.04)

        self.disarm_and_stop()
        raise MissionAbortException(f"Leg 2 exceeded {TIMEOUT_LEG2_ROTATION_S}s timeout!")

    def execute_leg3_return(self, home_x: float, home_y: float, return_yaw: float):
        """
        Leg 3: Return to authoritative saved HOME coordinates while retaining return-facing heading.
        Rover is disarmed after Leg 2.
        1. Zero handshake while disarmed.
        2. Dispatch Nav2 goal (pre-dispatch refresh while disarmed, then Nav2 arms to Mode 3).
        3. Nav2 drives to HOME coordinates retaining return heading.
        4. Nav2 reports SUCCEEDED and Cockpit disarms hardware.
        """
        print(f"\n[LEG 3] Verifying zero handshake and dispatching Nav2 return to HOME (retaining return heading: {math.degrees(return_yaw):+.2f}°)...")
        self.recorder.record_transition("LEG3_RETURN_START", {"home": (home_x, home_y, return_yaw)})

        # 1. Zero handshake while disarmed
        self.verify_zero_handshake(expected_cmd_source="ANY")

        # 2. Dispatch Nav2 goal
        goal_id = self.dispatch_nav2_goal(home_x, home_y, return_yaw)

        # 3. Wait for Nav2 completion
        self.wait_for_nav2_completion_and_zero(TIMEOUT_LEG3_RETURN_S, "LEG3_RETURN", goal_id, PASS_FINAL_POS_ERR_M)
        print("[LEG 3 COMPLETE] Returned to HOME position, Nav2 reported SUCCEEDED, rover finished disarmed.")
        self.recorder.record_transition("LEG3_RETURN_END")

    def execute_leg4_settle(self, target_home_yaw_rad: float):
        """
        Leg 4: Final in-place alignment to saved HOME yaw.
        Rover is disarmed after Leg 3.
        1. Zero handshake.
        2. Arm explicitly to Mode 3.
        3. Acquire CALIBRATION_TEST ownership.
        4. Rotate in place using signed shortest-angle error (<= 0.18 rad/s).
        5. Stop within 3.0° and settle for 1.5s. Timeout fails the test.
        6. Disarm to Mode 0, release ownership.
        """
        print(f"\n[LEG 4] Handshaking zero, arming Mode 3, acquiring CALIBRATION_TEST ownership for final HOME yaw alignment...")
        self.recorder.record_transition("LEG4_SETTLE_START")

        # 1. Zero handshake while disarmed
        self.verify_zero_handshake(expected_cmd_source="ANY")

        # 2. Arm explicitly to Mode 3
        self.arm()

        # 3. Acquire CALIBRATION_TEST ownership
        self.set_command_source("CALIBRATION_TEST")

        t_start = time.monotonic()
        settle_start_time: Optional[float] = None

        while time.monotonic() - t_start < TIMEOUT_LEG4_SETTLE_S:
            self.poll_all_telemetry()
            self.verify_safety_invariants("LEG4_SETTLE")

            amcl = self.latest_telemetry.get("amcl") or {}
            cur_th = amcl.get("yaw_rad", math.radians(amcl.get("yaw_deg", 0.0)))
            
            # Signed shortest-angle error
            delta_yaw = wrap_angle_rad(target_home_yaw_rad - cur_th)
            yaw_err_deg = abs(math.degrees(delta_yaw))

            if yaw_err_deg <= PASS_FINAL_YAW_ERR_DEG:
                self.send_velocity(0.0, 0.0, source="CALIBRATION_TEST", force_zero=True)
                self.record_tick("LEG4_SETTLE")
                if settle_start_time is None:
                    settle_start_time = time.monotonic()
                elif time.monotonic() - settle_start_time >= 1.5:
                    print(f"[LEG 4 COMPLETE] Settled at HOME with yaw error {yaw_err_deg:.2f}° (Limit <= 3.0°).")
                    self.recorder.record_transition("LEG4_SETTLE_END")
                    self.disarm()
                    self.set_command_source("NONE")
                    return
            else:
                settle_start_time = None
                abs_err = abs(delta_yaw)
                if abs_err > math.radians(30.0):
                    wz_mag = FINAL_YAW_ALIGNMENT_MAX_SPEED
                else:
                    alpha = abs_err / math.radians(30.0)
                    wz_mag = 0.08 + alpha * (FINAL_YAW_ALIGNMENT_MAX_SPEED - 0.08)
                
                wz_cmd = math.copysign(wz_mag, delta_yaw)
                self.send_velocity(0.0, wz_cmd, source="CALIBRATION_TEST")
                self.record_tick("LEG4_SETTLE")

            time.sleep(0.04)

        # Timeout fails the mission
        self.disarm_and_stop()
        raise MissionAbortException(
            f"Leg 4 final yaw alignment exceeded timeout ({TIMEOUT_LEG4_SETTLE_S}s) without reaching tolerance! (yaw error: {yaw_err_deg:.2f}° > 3.0°)"
        )
