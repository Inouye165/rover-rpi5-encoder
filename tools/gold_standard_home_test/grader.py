"""
tools.gold_standard_home_test.grader - Acceptance Test Grading Engine
Computes objective pass/fail metrics against deterministic criteria.
"""

import math
from typing import Dict, Any, List, Tuple
from .constants import (
    FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    PASS_FINAL_POS_ERR_M,
    PASS_FINAL_YAW_ERR_DEG,
    PASS_MAX_CRAWL_WINDOW_SEC,
    PASS_MAX_CORRECTIVE_REVERSALS,
    PASS_MIN_RECORDER_RATE_HZ,
    CRAWL_LINEAR_THRESHOLD,
    CRAWL_ANGULAR_THRESHOLD,
    STANDSTILL_LINEAR_EPSILON,
    STANDSTILL_ANGULAR_EPSILON
)

class MissionGrader:
    @staticmethod
    def grade_run(run_data: Dict[str, Any]) -> Dict[str, Any]:
        samples = run_data.get("samples", [])
        transitions = run_data.get("transitions", [])
        metadata = run_data.get("metadata", {})
        
        # 1. Total Mission Time
        total_time_s = metadata.get("duration_s", 0.0)
        if total_time_s == 0.0 and samples:
            total_time_s = samples[-1].get("t_rel_s", 0.0) - samples[0].get("t_rel_s", 0.0)

        # 2. Identify Key Transition Samples
        stage_samples: Dict[str, List[Dict[str, Any]]] = {}
        for s in samples:
            st = s.get("mission_stage", "UNKNOWN")
            stage_samples.setdefault(st, []).append(s)

        # Start pose: use BASELINE sample if captured, otherwise first sample
        base_samples = stage_samples.get("BASELINE", [])
        if base_samples:
            base_amcl = base_samples[0].get("amcl", {})
            start_x = base_amcl.get("x", samples[0].get("amcl", {}).get("x", 0.0) if samples else 0.0)
            start_y = base_amcl.get("y", samples[0].get("amcl", {}).get("y", 0.0) if samples else 0.0)
        elif samples:
            start_amcl = samples[0].get("amcl", {})
            start_x = start_amcl.get("x", 0.0)
            start_y = start_amcl.get("y", 0.0)
        else:
            start_x = 0.0
            start_y = 0.0

        # Leg 1 Outbound Distance
        leg1_samples = stage_samples.get("LEG1_FORWARD", [])
        if leg1_samples:
            end_leg1 = leg1_samples[-1].get("amcl", {})
            actual_outbound_dist = math.hypot(end_leg1.get("x", 0.0) - start_x, end_leg1.get("y", 0.0) - start_y)
        else:
            actual_outbound_dist = 0.0
        outbound_dist_err_m = abs(actual_outbound_dist - FORWARD_DISTANCE_M)

        # Leg 2 180Â° CW Rotation
        leg2_samples = stage_samples.get("LEG2_ROTATION", [])
        if leg2_samples:
            start_rot = leg2_samples[0].get("imu", {}).get("raw_yaw_deg", 0.0)
            end_rot = leg2_samples[-1].get("imu", {}).get("raw_yaw_deg", 0.0)
            delta_rot = (start_rot - end_rot) % 360.0
            rot_180_err_deg = abs(delta_rot - ROTATION_TARGET_DEG)
        else:
            rot_180_err_deg = 180.0

        # Final HOME Error
        last_sample = samples[-1] if samples else {}
        final_to_home = last_sample.get("to_home", {})
        final_pos_err_m = final_to_home.get("pos_err_m", 999.0)
        final_yaw_err_deg = abs(final_to_home.get("yaw_err_deg", 180.0))

        # Final Hardware State
        final_drive = last_sample.get("drive", {})
        final_armed = final_drive.get("armed", True)
        final_mode = final_drive.get("mode", -1)
        final_req_lin = final_drive.get("reqLinear", 99.0)
        final_req_ang = final_drive.get("reqAngular", 99.0)
        final_state_safe = (not final_armed) and (final_mode == 0) and (abs(final_req_lin) < 0.001) and (abs(final_req_ang) < 0.001)

        # 3. Analyze Crawling & Low-Speed Duration
        max_continuous_crawl_s = 0.0
        current_crawl_s = 0.0
        last_t = None

        for s in samples:
            t = s.get("t_rel_s", 0.0)
            dt = (t - last_t) if last_t is not None else 0.0
            last_t = t
            
            cmd = s.get("final_cmd", {})
            vx = abs(cmd.get("vx", 0.0))
            wz = abs(cmd.get("wz", 0.0))

            is_active = (vx > STANDSTILL_LINEAR_EPSILON) or (wz > STANDSTILL_ANGULAR_EPSILON)
            is_crawling = is_active and (vx < CRAWL_LINEAR_THRESHOLD) and (wz < CRAWL_ANGULAR_THRESHOLD)
            
            if is_crawling and dt < 0.2:
                current_crawl_s += dt
                if current_crawl_s > max_continuous_crawl_s:
                    max_continuous_crawl_s = current_crawl_s
            else:
                current_crawl_s = 0.0

        # 4. Count Direction Reversals (Linear and Angular Settling)
        # Linear reversals across translation stages (LEG1_FORWARD and LEG3_RETURN)
        linear_reversals = 0
        for st_name in ("LEG1_FORWARD", "LEG3_RETURN"):
            samples_st = stage_samples.get(st_name, [])
            last_sign_lin = 0
            for s in samples_st:
                vx = s.get("final_cmd", {}).get("vx", 0.0)
                if abs(vx) > 0.01:
                    curr_sign = 1 if vx > 0 else -1
                    if last_sign_lin != 0 and curr_sign != last_sign_lin:
                        linear_reversals += 1
                    last_sign_lin = curr_sign

        # Angular settling reversals and settling time after first target crossing
        # In Leg 2: target is 180° CW from start. Target crossing happens when turned_cw >= 180.0°.
        # In Leg 4: target is saved HOME yaw. Target crossing happens when yaw error crosses 0°.
        angular_settling_reversals = 0
        total_settling_time_after_crossing_s = 0.0

        # Leg 2 analysis
        leg2_samples = stage_samples.get("LEG2_ROTATION", [])
        if leg2_samples:
            start_rot = leg2_samples[0].get("imu", {}).get("raw_yaw_deg", 0.0)
            crossed_t = None
            last_sign_ang = 0
            for s in leg2_samples:
                cur_rot = s.get("imu", {}).get("raw_yaw_deg", 0.0)
                turned_cw = (start_rot - cur_rot) % 360.0
                t = s.get("t_rel_s", 0.0)
                wz = s.get("final_cmd", {}).get("wz", 0.0)
                curr_sign = (1 if wz > 0 else -1) if abs(wz) > 0.01 else 0

                if crossed_t is None and turned_cw >= 180.0:
                    crossed_t = t

                if crossed_t is not None and curr_sign != 0:
                    if last_sign_ang != 0 and curr_sign != last_sign_ang:
                        angular_settling_reversals += 1

                if curr_sign != 0:
                    last_sign_ang = curr_sign

            if crossed_t is not None:
                total_settling_time_after_crossing_s += max(0.0, leg2_samples[-1].get("t_rel_s", 0.0) - crossed_t)

        # Leg 4 analysis
        leg4_samples = stage_samples.get("LEG4_SETTLE", [])
        if leg4_samples:
            crossed_t = None
            last_sign_ang = 0
            prev_yaw_err = None
            for s in leg4_samples:
                yaw_err = s.get("to_home", {}).get("yaw_err_deg", 0.0)
                t = s.get("t_rel_s", 0.0)
                wz = s.get("final_cmd", {}).get("wz", 0.0)
                curr_sign = (1 if wz > 0 else -1) if abs(wz) > 0.01 else 0

                if crossed_t is None:
                    if prev_yaw_err is not None and ((prev_yaw_err < 0 and yaw_err >= 0) or (prev_yaw_err > 0 and yaw_err <= 0)):
                        crossed_t = t
                    prev_yaw_err = yaw_err

                if crossed_t is not None and curr_sign != 0:
                    if last_sign_ang != 0 and curr_sign != last_sign_ang:
                        angular_settling_reversals += 1

                if curr_sign != 0:
                    last_sign_ang = curr_sign

            if crossed_t is not None:
                total_settling_time_after_crossing_s += max(0.0, leg4_samples[-1].get("t_rel_s", 0.0) - crossed_t)

        reversal_count = linear_reversals + angular_settling_reversals

        # 5. Safety Interventions & Resets
        first_sample = samples[0] if samples else {}
        initial_boot_count = first_sample.get("drive", {}).get("bootCount", 1)
        resets_detected = 0
        for s in samples:
            bc = s.get("drive", {}).get("bootCount", initial_boot_count)
            if bc != initial_boot_count:
                resets_detected += 1
                break

        watchdog_trips = metadata.get("watchdog_trips", 0)
        rejections = metadata.get("rejections", 0)
        safety_interventions = resets_detected + watchdog_trips + rejections

        # 6. Sample Rate Verification
        if len(samples) >= 2:
            dt_span = samples[-1].get("t_rel_s", 0.0) - samples[0].get("t_rel_s", 0.0)
            achieved_rate_hz = round((len(samples) - 1) / dt_span, 2) if dt_span > 0 else 0.0
        else:
            achieved_rate_hz = 0.0

        crit_final_pos = final_pos_err_m <= PASS_FINAL_POS_ERR_M
        crit_final_yaw = final_yaw_err_deg <= PASS_FINAL_YAW_ERR_DEG
        crit_resets = (resets_detected == 0) and (safety_interventions == 0)
        crit_reversals = reversal_count <= PASS_MAX_CORRECTIVE_REVERSALS
        crit_crawling = max_continuous_crawl_s <= PASS_MAX_CRAWL_WINDOW_SEC
        crit_safe_stop = final_state_safe
        crit_rate = achieved_rate_hz >= PASS_MIN_RECORDER_RATE_HZ

        all_passed = (
            crit_final_pos and
            crit_final_yaw and
            crit_resets and
            crit_reversals and
            crit_crawling and
            crit_safe_stop and
            crit_rate
        )

        return {
            "overall_status": "PASS" if all_passed else "FAIL",
            "criteria": {
                "final_home_position_error": {
                    "measured_m": round(final_pos_err_m, 4),
                    "measured_cm": round(final_pos_err_m * 100.0, 2),
                    "threshold_m": PASS_FINAL_POS_ERR_M,
                    "passed": crit_final_pos
                },
                "final_home_yaw_error": {
                    "measured_deg": round(final_yaw_err_deg, 2),
                    "threshold_deg": PASS_FINAL_YAW_ERR_DEG,
                    "passed": crit_final_yaw
                },
                "no_reset_or_discontinuity": {
                    "resets_detected": resets_detected,
                    "safety_interventions": safety_interventions,
                    "passed": crit_resets
                },
                "corrective_reversals": {
                    "count": reversal_count,
                    "linear_reversals": linear_reversals,
                    "angular_settling_reversals": angular_settling_reversals,
                    "settling_time_after_crossing_s": round(total_settling_time_after_crossing_s, 2),
                    "max_allowed": PASS_MAX_CORRECTIVE_REVERSALS,
                    "passed": crit_reversals
                },
                "low_speed_crawling": {
                    "max_continuous_s": round(max_continuous_crawl_s, 2),
                    "max_allowed_s": PASS_MAX_CRAWL_WINDOW_SEC,
                    "passed": crit_crawling
                },
                "recorder_sample_rate": {
                    "achieved_rate_hz": achieved_rate_hz,
                    "threshold_hz": PASS_MIN_RECORDER_RATE_HZ,
                    "passed": crit_rate
                },
                "final_safe_state": {
                    "armed": final_armed,
                    "mode": final_mode,
                    "passed": crit_safe_stop
                }
            },
            "metrics": {
                "outbound_distance_m": round(actual_outbound_dist, 4),
                "outbound_distance_error_m": round(outbound_dist_err_m, 4),
                "rotation_180_error_deg": round(rot_180_err_deg, 2),
                "linear_reversals": linear_reversals,
                "angular_settling_reversals": angular_settling_reversals,
                "settling_time_after_target_crossing_s": round(total_settling_time_after_crossing_s, 2),
            },
            "performance_metrics": {
                "outbound_distance_m": round(actual_outbound_dist, 4),
                "outbound_distance_error_m": round(outbound_dist_err_m, 4),
                "rotation_180_error_deg": round(rot_180_err_deg, 2),
                "linear_reversals": linear_reversals,
                "angular_settling_reversals": angular_settling_reversals,
                "settling_time_after_target_crossing_s": round(total_settling_time_after_crossing_s, 2),
                "total_mission_time_s": round(total_time_s, 2),
                "achieved_rate_hz": achieved_rate_hz
            }
        }
