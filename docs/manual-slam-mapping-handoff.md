# Manual SLAM Mapping & Nav2 Next-Phase Handoff

## 1. Executive Summary

This document records the empirical discovery, configuration state, and next-phase implementation plan for SLAM (Simultaneous Localization and Mapping) and Nav2 autonomous navigation on the Yahboom RPi5 Rover.

---

## 2. Environment & Package Audit (Confirmed Facts)

* **Cockpit UI Status**: The Web Cockpit currently reports SLAM and Nav2 as `NOT INSTALLED`.
* **Audit Finding**: That Cockpit message is **stale or incorrect**. The active ROS 2 environment contains fully built and functional SLAM and Nav2 installations inside the `rover-ros2` Docker container on the Raspberry Pi 5.

### Container & Package Inventory

1. **SLAM Packages & Executables**:
   - Package: `slam_toolbox`
   - Executables available: `async_slam_toolbox_node` and `sync_slam_toolbox_node`

2. **Nav2 Navigation Stack**:
   - Installed packages: `nav2_bringup`, `nav2_amcl`, `nav2_controller`, `nav2_map_server`, and `nav2_velocity_smoother`

3. **Launch & Configuration Paths**:
   - **Installed SLAM Launch File**: `/ros2_ws/install/rover_bringup/share/rover_bringup/launch/slam.launch.py`
   - **Repository Source Launch File**: [ros2/ros2_ws/src/rover_bringup/launch/slam.launch.py](file:///C:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/launch/slam.launch.py)
   - **SLAM Parameter File**: [ros2/ros2_ws/src/rover_bringup/config/mapper_params_online_async.yaml](file:///C:/Users/Ron/electronic_projects/yahboom-encoder/ros2/ros2_ws/src/rover_bringup/config/mapper_params_online_async.yaml)
   - **Existing Workflow Script**: [ros2/scripts/slam_map_workflow.sh](file:///C:/Users/Ron/electronic_projects/yahboom-encoder/ros2/scripts/slam_map_workflow.sh)

---

## 3. Empirical Manual SLAM Testing & Discovery

1. **Prerequisite Topics**:
   - Live telemetry and transform streams were validated on `/scan`, `/odom`, `/tf`, and `/tf_static`.

2. **Manual Execution**:
   - SLAM was manually launched inside the `rover-ros2` container via `slam.launch.py`.

3. **Visualization & Mapping**:
   - `/map` topic occupancy grid became visible in Foxglove Studio connected to `ws://10.0.0.246:8765`.
   - The rover was manually teleoperated while the occupancy grid accumulated.

4. **Observed Distortion & Open Diagnostic Hypotheses**:
   - The experimental map later appeared distorted and seemed to retain only the local area.
   - The root cause was not determined during baseline testing. Potential underlying factors to investigate in the next phase include:
     - Foxglove camera / 2D frame view visualization settings
     - Unintended SLAM node process restart
     - Reduced map dimensions or boundary truncation in `mapper_params_online_async.yaml`
     - Discontinuous `map` to `odom` frame transform jumps
     - Lost laser scan matching alignment
     - Erroneous or failed loop closure
     - Wheel-slip or odometry integration drift
     - Transform lookup or message header timestamp synchronization problems

5. **Map Asset Status**:
   - The experimental map was **intentionally not treated as a validated saved map** and was not committed to version control.

---

## 4. Next-Phase Scope & Implementation Handoff

The upcoming SLAM/Nav2 implementation phase should begin with:
1. **Real Cockpit SLAM-State Detection**: Replace the stale `NOT INSTALLED` placeholder with real container process/topic inspection.
2. **Cockpit Lifecycle Controls**: Implement web API endpoints for Start, Stop, and Save Map.
3. **Structured Logging**: Capture ROS 2 process output in designated log files.
4. **Controlled One-Room Test**: Perform a baseline indoor mapping session to calibrate scan matching and verify loop closure before proceeding to autonomous path navigation.

*Note: No SLAM UI controls, API handlers, or active mapping features were implemented in this finalization task.*
