"""
tools.gold_standard_home_test.grader - Acceptance Test Grading Engine
Computes objective pass/fail metrics against deterministic criteria.
"""

import math
from typing import Dict, Any, List, Tuple
from .constants import (
    FORWARD_DISTANCE_M,
    PRODUCTION_CHECK_FORWARD_DISTANCE_M,
    ROTATION_TARGET_DEG,
    PRE_ARM_MAX_POS_ERR_M,
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
            base_amcl = base_samples[-1].get("amcl", {})
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
        is_prod_check = metadata.get("mission_type") == "PRODUCTION_CHECK"
        expected_fwd_dist = PRODUCTION_CHECK_FORWARD_DISTANCE_M if is_prod_check else FORWARD_DISTANCE_M
        leg1_samples = stage_samples.get("LEG1_FORWARD", [])
        if leg1_samples:
            end_leg1 = leg1_samples[-1].get("amcl", {})
            actual_outbound_dist = math.hypot(end_leg1.get("x", 0.0) - start_x, end_leg1.get("y", 0.0) - start_y)
        else:
            actual_outbound_dist = 0.0
        outbound_dist_err_m = abs(actual_outbound_dist - expected_fwd_dist)

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
        # If target was approached without crossing, report "no crossing" distinctly with no fabricated settling time.
        angular_settling_reversals = 0
        total_settling_time_after_crossing_s = 0.0
        leg2_target_crossed = False
        leg4_target_crossed = False

        # Leg 2 analysis
        leg2_samples = stage_samples.get("LEG2_ROTATION", [])
        if leg2_samples:
            start_rot = leg2_samples[0].get("imu", {}).get("raw_yaw_deg", 0.0)
            crossed_t = None
            last_sign_ang = 0
            for s in leg2_samples:
                imu = s.get("imu", {})
                if "rel_yaw_deg" in imu and imu["rel_yaw_deg"] is not None:
                    turned_cw = -float(imu["rel_yaw_deg"])
                else:
                    cur_rot = float(imu.get("raw_yaw_deg", 0.0))
                    diff = (start_rot - cur_rot) % 360.0
                    turned_cw = diff if diff < 300.0 else 0.0

                t = s.get("t_rel_s", 0.0)
                wz = s.get("final_cmd", {}).get("wz", 0.0)
                curr_sign = (1 if wz > 0 else -1) if abs(wz) > 0.01 else 0

                if crossed_t is None and turned_cw >= ROTATION_TARGET_DEG:
                    crossed_t = t
                    leg2_target_crossed = True

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
                    if prev_yaw_err is not None and ((prev_yaw_err < -0.1 and yaw_err >= 0.0) or (prev_yaw_err > 0.1 and yaw_err <= 0.0)):
                        crossed_t = t
                        leg4_target_crossed = True
                    prev_yaw_err = yaw_err

                if crossed_t is not None and curr_sign != 0:
                    if last_sign_ang != 0 and curr_sign != last_sign_ang:
                        angular_settling_reversals += 1

                if curr_sign != 0:
                    last_sign_ang = curr_sign

            if crossed_t is not None:
                total_settling_time_after_crossing_s += max(0.0, leg4_samples[-1].get("t_rel_s", 0.0) - crossed_t)

        any_target_crossed = leg2_target_crossed or leg4_target_crossed

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
        total_time_s = metadata.get("duration_s", 0.0)
        total_frames = len(samples)

        active_motion_rate_hz = metadata.get("active_motion_rate_hz")
        if active_motion_rate_hz is None:
            if total_frames >= 2:
                active_span_s = samples[-1].get("t_rel_s", 0.0) - samples[0].get("t_rel_s", 0.0)
                active_motion_rate_hz = round((total_frames - 1) / max(0.001, active_span_s), 2) if active_span_s > 0 else 0.0
            else:
                active_span_s = 0.0
                active_motion_rate_hz = 0.0
        else:
            active_span_s = metadata.get("active_span_s", samples[-1].get("t_rel_s", 0.0) - samples[0].get("t_rel_s", 0.0) if total_frames >= 2 else 0.0)

        whole_run_coverage_hz = metadata.get("whole_run_coverage_hz")
        if whole_run_coverage_hz is None:
            whole_run_coverage_hz = round(total_frames / max(0.001, total_time_s), 2) if total_time_s > 0 else 0.0
        achieved_rate_hz = active_motion_rate_hz

        crit_final_pos = final_pos_err_m <= PASS_FINAL_POS_ERR_M
        crit_final_yaw = final_yaw_err_deg <= PASS_FINAL_YAW_ERR_DEG
        crit_resets = (resets_detected == 0) and (safety_interventions == 0)
        crit_reversals = reversal_count <= PASS_MAX_CORRECTIVE_REVERSALS
        crit_crawling = max_continuous_crawl_s <= PASS_MAX_CRAWL_WINDOW_SEC
        crit_safe_stop = final_state_safe
        # Require BOTH active-motion sampling and baseline-to-final whole-run coverage to be at least 20 Hz; 11.1 Hz must not pass
        crit_rate = (active_motion_rate_hz >= PASS_MIN_RECORDER_RATE_HZ) and (whole_run_coverage_hz >= PASS_MIN_RECORDER_RATE_HZ)

        # Leg 3 Return Position Error from reconciliation (if Leg 3 was run)
        leg3_pos_err_cm = None
        for t in transitions:
            if t.get("stage") == "LEG3_RETURN_END":
                recon = t.get("details", {}).get("reconciliation", {}) or {}
                leg3_pos_err_cm = recon.get("settled_amcl_error_to_target_cm")
                if leg3_pos_err_cm is None:
                    leg3_pos_err_cm = recon.get("amcl_error_to_target_cm")
                break

        crit_leg3_pos = True
        if leg3_pos_err_cm is not None:
            crit_leg3_pos = (leg3_pos_err_cm <= round(PASS_FINAL_POS_ERR_M * 100.0, 2))

        # Final HOME LiDAR scan agreement at rest (if captured in metadata)
        final_scan_val = metadata.get("final_home_scan_validation")
        crit_scan_agreement = True
        if final_scan_val and isinstance(final_scan_val, dict) and "overlap" in final_scan_val:
            crit_scan_agreement = bool(final_scan_val.get("ok", False) or final_scan_val.get("overlap", 0.0) >= 0.75)

        # Extract Leg 2 rotation details from transitions
        leg2_start = {}
        leg2_zero = {}
        leg2_end = {}
        for t in transitions:
            stage = t.get("stage")
            if stage == "LEG2_ROTATION_START":
                leg2_start = t.get("details", {})
            elif stage == "LEG2_ZERO_COMMANDED":
                leg2_zero = t.get("details", {})
            elif stage == "LEG2_ROTATION_END":
                leg2_end = t.get("details", {})

        turn_executed = bool(leg2_end)
        if turn_executed:
            settled_heading_err = leg2_end.get("settled_error_to_reciprocal_deg", leg2_end.get("settled_error_deg", 0.0))
            crit_settled_heading = settled_heading_err <= PASS_FINAL_YAW_ERR_DEG
            l2_status = "PASS" if crit_settled_heading else "FAIL"
            l2_details_str = f"Settled: {leg2_end.get('settled_heading_deg', 0.0):+.2f}° vs Target: {leg2_end.get('target_reciprocal_yaw_deg', 0.0):+.2f}° (Error: {settled_heading_err:.2f}° | Threshold <= {PASS_FINAL_YAW_ERR_DEG:.1f}°)"
        else:
            settled_heading_err = None
            crit_settled_heading = False
            l2_status = "NOT RUN"
            l2_details_str = "NOT RUN (Turn maneuver was not executed or aborted before completion)"

        # Check mission type
        is_production_check = (metadata.get("mission_type") == "PRODUCTION_CHECK")
        is_forward_and_turn = (
            metadata.get("mission_type") in ("FORWARD_AND_TURN", "SHORTENED_TEST") or
            metadata.get("stop_after_leg2") is True
        )

        crit_leg1_dist = outbound_dist_err_m <= PRE_ARM_MAX_POS_ERR_M

        if is_production_check:
            all_passed = (
                crit_leg1_dist and
                crit_final_pos and
                crit_final_yaw and
                crit_resets and
                crit_reversals and
                crit_crawling and
                crit_safe_stop and
                crit_rate
            )
            criteria_dict = {
                "request1_forward_distance": {
                    "measured_m": round(actual_outbound_dist, 4),
                    "measured_cm": round(actual_outbound_dist * 100.0, 2),
                    "error_cm": round(outbound_dist_err_m * 100.0, 2),
                    "threshold_cm": round(PRE_ARM_MAX_POS_ERR_M * 100.0, 1),
                    "passed": crit_leg1_dist
                },
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
                    "active_motion_rate_hz": active_motion_rate_hz,
                    "whole_run_coverage_hz": whole_run_coverage_hz,
                    "threshold_hz": PASS_MIN_RECORDER_RATE_HZ,
                    "passed": crit_rate
                },
                "final_safe_state": {
                    "armed": final_armed,
                    "mode": final_mode,
                    "passed": crit_safe_stop
                }
            }
            if final_scan_val and isinstance(final_scan_val, dict) and "overlap" in final_scan_val:
                criteria_dict["final_home_scan_agreement"] = {
                    "overlap_pct": round(final_scan_val.get("overlap", 0.0) * 100.0, 1),
                    "threshold_pct": 75.0,
                    "passed": crit_scan_agreement
                }
        elif is_forward_and_turn:
            all_passed = (
                crit_leg1_dist and
                turn_executed and
                crit_settled_heading and
                crit_resets and
                crit_reversals and
                crit_crawling and
                crit_safe_stop and
                crit_rate
            )
            criteria_dict = {
                "leg1_outbound_distance": {
                    "measured_m": round(actual_outbound_dist, 4),
                    "measured_cm": round(actual_outbound_dist * 100.0, 2),
                    "error_cm": round(outbound_dist_err_m * 100.0, 2),
                    "threshold_cm": round(PRE_ARM_MAX_POS_ERR_M * 100.0, 1),
                    "passed": crit_leg1_dist
                },
                "leg2_settled_reciprocal_heading": {
                    "status": l2_status,
                    "measured_err_deg": round(settled_heading_err, 2) if settled_heading_err is not None else None,
                    "settled_heading_deg": leg2_end.get("settled_heading_deg"),
                    "target_reciprocal_yaw_deg": leg2_end.get("target_reciprocal_yaw_deg") or leg2_start.get("target_reciprocal_yaw_deg"),
                    "zero_cmd_heading_deg": leg2_end.get("zero_commanded_yaw_deg") or leg2_end.get("zero_cmd_heading_deg") or leg2_zero.get("zero_commanded_yaw_deg") or leg2_zero.get("zero_cmd_heading_deg"),
                    "rotation_after_zero_deg": leg2_end.get("rotation_after_zero_deg"),
                    "threshold_deg": PASS_FINAL_YAW_ERR_DEG,
                    "passed": crit_settled_heading,
                    "details": l2_details_str
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
                    "target_crossed": any_target_crossed,
                    "target_crossing_status": "crossed" if any_target_crossed else "no crossing",
                    "settling_time_after_crossing_s": round(total_settling_time_after_crossing_s, 2) if any_target_crossed else None,
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
                    "active_motion_rate_hz": active_motion_rate_hz,
                    "whole_run_coverage_hz": whole_run_coverage_hz,
                    "active_span_s": round(active_span_s, 2),
                    "total_duration_s": round(total_time_s, 2),
                    "threshold_hz": PASS_MIN_RECORDER_RATE_HZ,
                    "passed": crit_rate
                },
                "final_safe_state": {
                    "armed": final_armed,
                    "mode": final_mode,
                    "passed": crit_safe_stop
                }
            }
        else:
            all_passed = (
                crit_final_pos and
                crit_final_yaw and
                crit_resets and
                crit_reversals and
                crit_crawling and
                crit_safe_stop and
                crit_rate and
                crit_leg3_pos and
                crit_scan_agreement
            )

            criteria_dict = {
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
                    "target_crossed": any_target_crossed,
                    "target_crossing_status": "crossed" if any_target_crossed else "no crossing",
                    "settling_time_after_crossing_s": round(total_settling_time_after_crossing_s, 2) if any_target_crossed else None,
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
                    "active_motion_rate_hz": active_motion_rate_hz,
                    "whole_run_coverage_hz": whole_run_coverage_hz,
                    "active_span_s": round(active_span_s, 2),
                    "total_duration_s": round(total_time_s, 2),
                    "threshold_hz": PASS_MIN_RECORDER_RATE_HZ,
                    "passed": crit_rate
                },
                "final_safe_state": {
                    "armed": final_armed,
                    "mode": final_mode,
                    "passed": crit_safe_stop
                }
            }

            if leg3_pos_err_cm is not None:
                criteria_dict["leg3_return_position_error"] = {
                    "measured_cm": leg3_pos_err_cm,
                    "threshold_cm": round(PASS_FINAL_POS_ERR_M * 100.0, 2),
                    "passed": crit_leg3_pos
                }

            if final_scan_val and isinstance(final_scan_val, dict) and "overlap" in final_scan_val:
                criteria_dict["final_home_scan_agreement"] = {
                    "overlap_pct": round(final_scan_val.get("overlap", 0.0) * 100.0, 1),
                    "threshold_pct": 75.0,
                    "passed": crit_scan_agreement
                }

        metrics = {
            "mission_type": "PRODUCTION_CHECK" if is_production_check else ("FORWARD_AND_TURN" if is_forward_and_turn else "FULL_HOME_MISSION"),
            "outbound_distance_m": round(actual_outbound_dist, 4),
            "outbound_distance_error_m": round(outbound_dist_err_m, 4),
            "rotation_180_error_deg": round(rot_180_err_deg, 2),
            "linear_reversals": linear_reversals,
            "angular_settling_reversals": angular_settling_reversals,
            "target_crossing_status": "crossed" if any_target_crossed else "no crossing",
            "settling_time_after_target_crossing_s": round(total_settling_time_after_crossing_s, 2) if any_target_crossed else None,
            "total_mission_time_s": round(total_time_s, 2),
            "achieved_rate_hz": achieved_rate_hz,
            "active_motion_rate_hz": active_motion_rate_hz,
            "whole_run_coverage_hz": whole_run_coverage_hz,
            "starting_heading_deg": leg2_end.get("starting_heading_deg") or leg2_start.get("starting_heading_deg") or (metadata.get("home_pose", {}).get("yaw_deg") if metadata else None),
            "turn_start_heading_deg": leg2_end.get("turn_start_heading_deg") or leg2_start.get("turn_start_heading_deg"),
            "zero_cmd_heading_deg": leg2_end.get("zero_commanded_yaw_deg") or leg2_end.get("zero_cmd_heading_deg") or leg2_zero.get("zero_commanded_yaw_deg") or leg2_zero.get("zero_cmd_heading_deg"),
            "settled_heading_deg": leg2_end.get("settled_heading_deg"),
            "target_reciprocal_yaw_deg": leg2_end.get("target_reciprocal_yaw_deg") or leg2_start.get("target_reciprocal_yaw_deg"),
            "settled_error_deg": round(settled_heading_err, 2) if settled_heading_err is not None else None,
            "rotation_after_zero_deg": leg2_end.get("rotation_after_zero_deg"),
            "sensor_disagreement": leg2_end.get("sensor_disagreement") or {},
            "starting_baseline_heading_deg": leg2_end.get("starting_heading_deg") or leg2_start.get("starting_heading_deg") or (metadata.get("home_pose", {}).get("yaw_deg") if metadata else None),
            "target_reciprocal_heading_deg": leg2_end.get("target_reciprocal_yaw_deg") or leg2_start.get("target_reciprocal_yaw_deg"),
            "settled_reciprocal_error_deg": round(settled_heading_err, 2) if settled_heading_err is not None else None,
            "post_zero_drift_deg": leg2_end.get("rotation_after_zero_deg")
        }

        return {
            "overall_status": "PASS" if all_passed else "FAIL",
            "criteria": criteria_dict,
            "metrics": metrics,
            "performance_metrics": dict(metrics)
        }
