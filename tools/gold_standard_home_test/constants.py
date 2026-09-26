"""
tools.gold_standard_home_test.constants - Frozen Settings & Pass Criteria
Deterministic parameters governing the Gold Standard HOME Acceptance Test.
"""

# Mission Kinematic Ceilings & Settings (Frozen)
FORWARD_DISTANCE_M = 0.6096       # Exactly 2.000 ft
ROTATION_TARGET_DEG = 180.0       # Explicit 180.0 deg CLOCKWISE

# Velocity limits & distinctions:
# 1. Active production linear cap for Nav2 travel and approach
NORMAL_LINEAR_SPEED = 0.20        # m/s (nav2_params.yaml max_vel_x and max_speed_xy)
# 2. Nav2 autonomous rotation limit in nav2_params.yaml is 0.50 rad/s (max_vel_theta)
NAV2_MAX_ANGULAR_SPEED = 0.50     # rad/s
# 3. Explicit 180° test rotation maximum angular speed
ROTATION_180_MAX_SPEED = 0.40     # rad/s (governed approach controller)
NORMAL_ANGULAR_SPEED = ROTATION_180_MAX_SPEED # alias for backward compatibility
# 4. Final explicit HOME alignment maximum angular speed
FINAL_YAW_ALIGNMENT_MAX_SPEED = 0.18  # rad/s (signed shortest-angle in-place alignment)
FINAL_YAW_CORRECTION_CEILING = FINAL_YAW_ALIGNMENT_MAX_SPEED # alias
FINAL_POS_CORRECTION_CEILING = 0.10  # m/s

# Pre-Arm Verification Thresholds
PRE_ARM_MAX_POS_ERR_M = 0.05      # 0.05 m (50 mm)
PRE_ARM_MAX_YAW_ERR_DEG = 5.0     # 5.0 deg

# Mission Pass Criteria
PASS_FINAL_POS_ERR_M = 0.04       # <= 0.04 m (40 mm)
PASS_FINAL_YAW_ERR_DEG = 3.0      # <= 3.0 deg
PASS_MAX_CRAWL_WINDOW_SEC = 2.0   # <= 2.0s continuous crawling below thresholds
PASS_MAX_CORRECTIVE_REVERSALS = 1 # <= 1 reversal
PASS_MIN_RECORDER_RATE_HZ = 20.0  # >= 20.0 Hz achieved sample rate

# Low-speed crawl detection thresholds (excluding normal accel/decel)
CRAWL_LINEAR_THRESHOLD = 0.05     # m/s
CRAWL_ANGULAR_THRESHOLD = 0.12    # rad/s
STANDSTILL_LINEAR_EPSILON = 0.005 # m/s
STANDSTILL_ANGULAR_EPSILON = 0.02 # rad/s

# Leg Timeout Limits (Seconds)
TIMEOUT_LEG1_FORWARD_S = 25.0
TIMEOUT_LEG2_ROTATION_S = 15.0
TIMEOUT_LEG3_RETURN_S = 35.0
TIMEOUT_LEG4_SETTLE_S = 25.0      # In-place rotation from ~180° away at 0.18 rad/s takes ~18s + 1.5s settle

# Telemetry
TELEMETRY_RATE_HZ = 25.0          # Target rate >= 20 Hz
TELEMETRY_STALE_TIMEOUT_S = 0.5   # 500 ms stale detection

# Network Endpoints
COCKPIT_DEFAULT_URL = "http://127.0.0.1:3000"
BRIDGE_DEFAULT_URL = "http://127.0.0.1:3010"
