# Rover 1 — Containerized ROS 2 Jazzy & Nav2 Stack (`ros2/`)

This directory houses the containerized **ROS 2 Jazzy** and **Nav2** environment (`rover-ros2`) for **Rover 1** on the Raspberry Pi 5 (`ron@10.0.0.246:/home/ron/yahboom-encoder/ros2`).

---

## 1. Hardware Isolation & Ownership Architecture

> **The `rover-ros2` Docker container has NO `/dev` mounts and never opens serial ports directly.**
>
> - `/dev/rover-esp32` (`1a86:7523` Maker-ESP32-Pro) is owned exclusively by `rover-server.service` (`:3000`).
> - `/dev/rover-lidar` (`10c4:ea60` RPLiDAR C1) is owned exclusively by `rover-lidar.service` (`:3002`).
> - `/dev/ttyAMA0` (Lonely Binary ESP32-S3 + `TCA9548A` ToF mux) is owned exclusively by `esp32s3-status-led.service` (`/tmp/esp32s3_tof_state.json` → `:3000/api/tof-sensors`).
>
> All ROS 2 bridge nodes communicate with the host exclusively via localhost HTTP APIs (`ROVER_SERVER_URL=http://127.0.0.1:3000`, `ROVER_LIDAR_URL=http://127.0.0.1:3002`).

```text
┌────────────────────────────────────────────────────────────────────────────┐
│  Host (Raspberry Pi 5)                                                     │
│                                                                            │
│  rover-server.service (:3000)  ◄──► /dev/rover-esp32 (Maker-ESP32-Pro)     │
│  esp32s3-status-led.service    ◄──► /dev/ttyAMA0     (ESP32-S3 + ToF Mux)  │
│  rover-lidar.service  (:3002)  ◄──► /dev/rover-lidar (RPLiDAR C1)          │
│            │                                  │                            │
│            └──────────────┬───────────────────┘                            │
│                           ▼  HTTP (localhost :3000 / :3002)                │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │  Docker container: rover-ros2 (ROS 2 Jazzy, network_mode: host)      │  │
│  │  Entrypoint: ros2 launch rover_bringup bringup.launch.py             │  │
│  │                                                                      │  │
│  │  1. foundation.launch.py (11 nodes):                                 │  │
│  │     - rover_system_health      ──► /diagnostics                      │  │
│  │     - rover_lidar_bridge       ──► /scan (LaserScan, laser_frame)    │  │
│  │     - rover_tof_bridge         ──► /tof/front_{left,right}/{points,range} │
│  │     - rover_encoder_odometry   ──► /odom + TF (odom -> base_link)    │  │
│  │     - rover_imu_bridge         ──► /imu/data, /imu/game_rotation_vector │
│  │     - rover_cmd_vel_bridge     ◄── /cmd_vel (POST /api/cmd_vel)      │  │
│  │     - rover_nav_bridge (:3010) ◄─► Nav2 Actions, AMCL, HOME Guard    │  │
│  │     - foxglove_bridge (:8765)  ──► Foxglove WebSocket                │  │
│  │     - 4x static_transform_publisher:                                 │  │
│  │         base_link -> laser_frame          (0.03175, 0.0, 0.17145)    │  │
│  │         base_link -> imu_link             (-0.0254, -0.0254, 0.14605)│  │
│  │         base_link -> tof_front_left_link  (0.14, +0.08, 0.08, +8°)   │  │
│  │         base_link -> tof_front_right_link (0.14, -0.08, 0.08, -8°)   │  │
│  │                                                                      │  │
│  │  2. navigation.launch.py:                                            │  │
│  │     - map_server, amcl, planner_server (Smac2D),                     │  │
│  │       controller_server (RotationShim + DWB), bt_navigator,          │  │
│  │       velocity_smoother, collision_monitor (PolygonSlow/PolygonStop) │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Bridge Node Reference

### A. `rover_lidar_bridge` (`/scan`)
Polls `GET ${ROVER_LIDAR_URL}/scan` (`:3002`) at `10 Hz` and publishes `sensor_msgs/msg/LaserScan` in `laser_frame` (`360` 1-degree bins, missing bins set to `float('inf')` per REP-117).

### B. `rover_tof_bridge` (`/tof/front_left/*`, `/tof/front_right/*`)
Polls `GET ${ROVER_SERVER_URL}/api/tof-sensors` (`:3000`) at `10 Hz` via a persistent `requests.Session()` and publishes:
- `/tof/front_left/points` (`sensor_msgs/msg/PointCloud2`) & `/tof/front_left/range` (`sensor_msgs/msg/Range`, `tof_front_left_link`, $+8^\circ$ outward yaw)
- `/tof/front_right/points` (`sensor_msgs/msg/PointCloud2`) & `/tof/front_right/range` (`sensor_msgs/msg/Range`, `tof_front_right_link`, $-8^\circ$ outward yaw)
*(Only explicitly mapped `front_left_l7cx` and `front_right_l7cx` 8×8 channels are published; `CH 2` `VL53L4CD` is intentionally excluded until physically mounted/mapped.)*

### C. `rover_encoder_odometry` (`/odom` & `odom -> base_link` TF)
Polls wheel encoder telemetry from `rover-server` using calibrated effective wheel diameter **`0.06695 m`** (`1974.1667` ticks/rev) and effective differential track width **`0.3408575433 m`**.

### D. `rover_imu_bridge` (`/imu/data` & `/imu/game_rotation_vector`)
Publishes BNO08x non-magnetic orientation (`SH2_GAME_ROTATION_VECTOR_NON_MAGNETIC`, report `0x08`) and calibrated gyro rates in `imu_link`.

### E. `rover_cmd_vel_bridge` (`/cmd_vel` $\to$ `POST /api/cmd_vel`)
Forwards `collision_monitor`-approved `/cmd_vel` commands to `rover-server.service` (which enforces arming state, mode gates, and the `500 ms` autonomy watchdog).

### F. `rover_nav_bridge` (HTTP `:3010`)
Provides the HTTP bridge between `server.js` (`:3000`) and ROS 2 / Nav2:
- Guarded cold-start HOME scan-to-map validation (`>= 75%` wall overlap required before injecting `/initialpose`).
- Non-blocking `ComputePathToPose` path previews (`~17 ms` end-to-end) and `NavigateToPose` goal dispatch/cancel.

---

## 3. Container Management & Validation Commands

```bash
# Build workspace inside container and restart rover-ros2
cd /home/ron/yahboom-encoder/ros2
docker exec rover-ros2 bash -lc "cd /ros2_ws && colcon build --packages-select rover_bringup"
docker compose restart rover-ros2

# Verify active nodes inside rover-ros2
docker exec rover-ros2 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list"

# Verify Front ToF topics and static TF frames
docker exec rover-ros2 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list | grep tof"
docker exec rover-ros2 bash -lc "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 run tf2_ros tf2_echo base_link tof_front_left_link"
```
