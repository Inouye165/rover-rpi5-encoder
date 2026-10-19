import os
import math
import numpy as np

def validate_scan_to_map(
    ranges,
    angle_min,
    angle_increment,
    map_pixels,
    map_width,
    map_height,
    map_resolution,
    map_origin_x,
    map_origin_y,
    home_x,
    home_y,
    home_yaw_rad,
    base_to_laser=(0.03175, 0.0, 0.0),
    window_size=3,
    min_overlap=0.75,
    min_beams=100,
    range_min=0.12,
    range_max=8.0
):
    """
    Independently compares LiDAR scan beams against occupancy grid map
    under the hypothesis that the rover is positioned at (home_x, home_y, home_yaw_rad).

    Returns:
        dict: {
            "ok": bool,
            "overlap": float,
            "hits": int,
            "total_valid": int,
            "reason": str,
            "details": str
        }
    """
    if map_pixels is None or map_width <= 0 or map_height <= 0:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": 0,
            "total_valid": 0,
            "reason": "INVALID_MAP",
            "details": "Map pixels or dimensions are invalid"
        }

    if ranges is None or len(ranges) == 0:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": 0,
            "total_valid": 0,
            "reason": "MISSING_SCAN",
            "details": "Scan ranges are empty or None"
        }

    # Sensor frame position in map frame under HOME hypothesis
    laser_x = home_x + base_to_laser[0] * math.cos(home_yaw_rad) - base_to_laser[1] * math.sin(home_yaw_rad)
    laser_y = home_y + base_to_laser[0] * math.sin(home_yaw_rad) + base_to_laser[1] * math.cos(home_yaw_rad)
    laser_yaw = home_yaw_rad + base_to_laser[2]

    # Half window for occupancy hit check
    w_r = window_size // 2

    total_valid = 0
    hits = 0

    ranges_arr = np.asarray(ranges, dtype=np.float32)
    num_beams = len(ranges_arr)

    for i in range(num_beams):
        r = float(ranges_arr[i])
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue

        total_valid += 1
        angle_rad = angle_min + i * angle_increment

        # Point in laser_frame (ROS REP-103 convention: x forward, y left)
        lx = r * math.cos(angle_rad)
        ly = r * math.sin(angle_rad)

        # Transform to map coordinates
        mx = laser_x + (lx * math.cos(laser_yaw) - ly * math.sin(laser_yaw))
        my = laser_y + (lx * math.sin(laser_yaw) + ly * math.cos(laser_yaw))

        # Map grid cell
        gx = int((mx - map_origin_x) / map_resolution)
        gy = int((my - map_origin_y) / map_resolution)
        # Flip Y for PGM pixel storage (row 0 is top)
        py = (map_height - 1 - gy)

        if 0 <= gx < map_width and 0 <= py < map_height:
            sub = map_pixels[max(0, py - w_r):min(map_height, py + w_r + 1),
                             max(0, gx - w_r):min(map_width, gx + w_r + 1)]
            # In standard ROS PGM, occupied cells have low pixel intensity (< 50)
            if np.any(sub < 50):
                hits += 1

    if total_valid < min_beams:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": hits,
            "total_valid": total_valid,
            "reason": "INSUFFICIENT_BEAMS",
            "details": f"Only {total_valid} valid beams found (minimum required: {min_beams})"
        }

    overlap = float(hits) / float(total_valid)
    if overlap >= min_overlap:
        return {
            "ok": True,
            "overlap": round(overlap, 4),
            "hits": hits,
            "total_valid": total_valid,
            "reason": "VERIFIED_HOME",
            "details": f"Strong confirmation: {overlap*100:.1f}% scan-to-map overlap (threshold {min_overlap*100:.1f}%)"
        }
    else:
        return {
            "ok": False,
            "overlap": round(overlap, 4),
            "hits": hits,
            "total_valid": total_valid,
            "reason": "SCAN_MISMATCH",
            "details": f"Scan overlap {overlap*100:.1f}% below required {min_overlap*100:.1f}% threshold"
        }


def validate_laserscan_msg(
    scan_msg,
    map_dict,
    home_dict,
    now_sec=None,
    max_age_sec=0.75,
    min_overlap=0.75,
    min_beams=100
):
    """
    High-level validator for LaserScan message or dict, map metadata/pixels, and home pose.
    Handles staleness, missing data, and geometric comparison.
    """
    if scan_msg is None:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": 0,
            "total_valid": 0,
            "reason": "MISSING_SCAN",
            "details": "No scan message provided (None)"
        }

    # Extract stamp and ranges depending on msg type (ROS msg or dict)
    if hasattr(scan_msg, "header"):
        stamp_sec = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
    elif isinstance(scan_msg, dict):
        stamp_sec = scan_msg.get("timestamp_sec", 0.0)
        ranges = scan_msg.get("ranges", [])
        angle_min = scan_msg.get("angle_min", 0.0)
        angle_increment = scan_msg.get("angle_increment", 0.01745)
    else:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": 0,
            "total_valid": 0,
            "reason": "INVALID_SCAN",
            "details": "Unrecognized scan message format"
        }

    # Freshness check
    if now_sec is not None and stamp_sec > 0.0:
        age_sec = now_sec - stamp_sec
        if age_sec > max_age_sec:
            return {
                "ok": False,
                "overlap": 0.0,
                "hits": 0,
                "total_valid": 0,
                "reason": "STALE_SCAN",
                "details": f"Scan age {age_sec:.3f}s exceeds freshness threshold of {max_age_sec:.3f}s"
            }

    # Extract map data
    w = map_dict.get("width", 0)
    h = map_dict.get("height", 0)
    map_res = float(map_dict.get("resolution", 0.05))
    origin = map_dict.get("origin", [0.0, 0.0, 0.0])
    ox, oy = float(origin[0]), float(origin[1])

    pixels = map_dict.get("pixels")
    if isinstance(pixels, list):
        map_pixels = np.array(pixels, dtype=np.uint8).reshape((h, w))
    elif isinstance(pixels, np.ndarray):
        map_pixels = pixels
    else:
        return {
            "ok": False,
            "overlap": 0.0,
            "hits": 0,
            "total_valid": 0,
            "reason": "INVALID_MAP",
            "details": "Map pixels missing or invalid"
        }

    # Extract home pose
    hx = float(home_dict.get("x", 1.193853))
    hy = float(home_dict.get("y", -0.045221))
    hyaw_rad = float(home_dict.get("yaw_rad", math.radians(float(home_dict.get("yaw_deg", -5.047)))))

    val_res = validate_scan_to_map(
        ranges=ranges,
        angle_min=angle_min,
        angle_increment=angle_increment,
        map_pixels=map_pixels,
        map_width=w,
        map_height=h,
        map_resolution=map_res,
        map_origin_x=ox,
        map_origin_y=oy,
        home_x=hx,
        home_y=hy,
        home_yaw_rad=hyaw_rad,
        window_size=3,
        min_overlap=min_overlap,
        min_beams=min_beams
    )
    if val_res.get("ok") or val_res.get("total_valid", 0) >= min_beams:
        val_res["estimated_pose"] = estimate_scan_to_map_pose(
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            map_pixels=map_pixels,
            map_width=w,
            map_height=h,
            map_resolution=map_res,
            map_origin_x=ox,
            map_origin_y=oy,
            center_x=hx,
            center_y=hy,
            center_yaw_rad=hyaw_rad,
            window_size=3
        )
    return val_res


def estimate_scan_to_map_pose(
    ranges,
    angle_min,
    angle_increment,
    map_pixels,
    map_width,
    map_height,
    map_resolution,
    map_origin_x,
    map_origin_y,
    center_x,
    center_y,
    center_yaw_rad,
    search_radius_m=0.06,
    step_m=0.015,
    base_to_laser=(0.03175, 0.0, 0.0),
    window_size=3
):
    """
    Evaluates scan overlap on a local 2D grid around (center_x, center_y) to find
    the peak scan-to-map position estimate.
    """
    best_x = center_x
    best_y = center_y
    best_overlap = -1.0
    best_res = None

    dx_range = np.arange(-search_radius_m, search_radius_m + 1e-4, step_m)
    dy_range = np.arange(-search_radius_m, search_radius_m + 1e-4, step_m)

    for dx in dx_range:
        for dy in dy_range:
            cand_x = center_x + dx
            cand_y = center_y + dy
            res = validate_scan_to_map(
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                map_pixels=map_pixels,
                map_width=map_width,
                map_height=map_height,
                map_resolution=map_resolution,
                map_origin_x=map_origin_x,
                map_origin_y=map_origin_y,
                home_x=cand_x,
                home_y=cand_y,
                home_yaw_rad=center_yaw_rad,
                base_to_laser=base_to_laser,
                window_size=window_size
            )
            ov = res.get("overlap", 0.0)
            if ov > best_overlap:
                best_overlap = ov
                best_x = cand_x
                best_y = cand_y
                best_res = res

    return {
        "x": round(float(best_x), 4),
        "y": round(float(best_y), 4),
        "yaw_rad": round(float(center_yaw_rad), 4),
        "yaw_deg": round(float(math.degrees(center_yaw_rad)), 2),
        "overlap": round(float(best_overlap), 4),
        "error_to_center_m": round(float(math.hypot(best_x - center_x, best_y - center_y)), 4),
        "hits": best_res.get("hits", 0) if best_res else 0,
        "total_valid": best_res.get("total_valid", 0) if best_res else 0
    }
