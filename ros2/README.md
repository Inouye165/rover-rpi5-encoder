# Rover ROS 2 — Phase 2: LiDAR /scan Bridge

This directory houses the containerized ROS 2 Jazzy environment for the Yahboom Pi 5 Rover.

---

## Architecture

### Hardware Ownership

> **ROS 2 does NOT own or open the LiDAR serial device.**
>
> The physical RPLIDAR C1 (`/dev/rover-lidar`) is owned exclusively by the host
> `rover-lidar.service` systemd unit.  The Docker container has **no `/dev`
> mounts** and cannot access any serial device.  This design ensures that the
> sidecar remains the single authoritative owner of the LiDAR, preventing
> concurrent-access failures and keeping the ROS layer fully restartable without
> touching hardware.

### Phase 2 Node Graph

```
┌─────────────────────────────────────────────────────────────┐
│  Host (Raspberry Pi 5)                                      │
│                                                             │
│  rover-lidar.service (/dev/rover-lidar → RPLIDAR C1)        │
│    └─► rplidar_sidecar.py  HTTP :3002                       │
│                │                                            │
│                │  GET /scan  (JSON, ~7 Hz)                  │
│                ▼                                            │
│  ┌─────────────────────────────────────────────────┐        │
│  │  Docker container: rover-ros2 (ROS 2 Jazzy)     │        │
│  │                                                 │        │
│  │  rover_system_health ──► /diagnostics           │        │
│  │                                                 │        │
│  │  rover_lidar_bridge  ──► /scan                  │        │
│  │    (sensor_msgs/msg/LaserScan)                  │        │
│  └─────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

### Sidecar-to-ROS Data Flow

| Sidecar field    | Type            | Conversion                     | ROS field             |
|------------------|-----------------|--------------------------------|-----------------------|
| `angleDeg`       | float [0, 360)  | × π/180 → radians, 1° bins     | `ranges[bin_index]`   |
| `distanceMm`     | int mm          | ÷ 1000 → metres                | `ranges[i]`           |
| `quality`        | int 0-47        | cast to float                  | `intensities[i]`      |
| `timestamp`      | ISO-8601 UTC    | `calendar.timegm()` → sec/ns   | `header.stamp`        |
| `sequence`       | int             | compared to last published seq | (staleness gate)      |
| `scanHz`         | float           | `1/scanHz` → scan_time         | `scan_time`           |

**Missing angular bins** (the sidecar may return sparse points) are filled with
`float('inf')` per [REP-117](https://ros.org/reps/rep-0117.html), never fabricated.

### Configurable ROS Parameters (rover_lidar_bridge)

| Parameter         | Default                      | Description                                   |
|-------------------|------------------------------|-----------------------------------------------|
| `lidar_url`       | `$ROVER_LIDAR_URL` or `:3002`| HTTP base URL of the LiDAR sidecar            |
| `frame_id`        | `laser_frame`                | TF frame ID in the LaserScan header           |
| `poll_rate_hz`    | `10.0`                       | Polling frequency (Hz)                        |
| `range_min_m`     | `0.10`                       | Minimum valid range (metres)                  |
| `range_max_m`     | `12.0`                       | Maximum valid range (metres)                  |
| `angle_offset_deg`| `0.0`                        | Rotation correction for physical mounting (°) |
| `reverse_scan`    | `False`                      | Publish CW instead of CCW (invert scan)       |

---

## Environment Setup

1. Copy `.env.example` to `.env`:
   ```bash
   cp .env.example .env
   ```

2. Make the scripts executable:
   ```bash
   chmod +x scripts/*.sh
   ```

---

## Usage

| Script                    | Description                                              |
|---------------------------|----------------------------------------------------------|
| `./scripts/build.sh`      | Build Docker image and compile the ROS 2 workspace       |
| `./scripts/up.sh`         | Pre-checks → start container → post-checks               |
| `./scripts/down.sh`       | Stop the container                                       |
| `./scripts/shell.sh`      | Open an interactive ROS 2 shell inside the container     |
| `./scripts/doctor.sh`     | Phase 2 diagnostic audit (hardware isolation + /scan)    |

---

## Launch Commands

The container automatically runs:

```bash
ros2 launch rover_bringup foundation.launch.py
```

This brings up **two nodes**:
- `rover_system_health` — publishes `/diagnostics` (Phase 1)
- `rover_lidar_bridge` — publishes `/scan` as `sensor_msgs/msg/LaserScan` (Phase 2)

To launch manually inside the container shell:

```bash
# Inside: ./scripts/shell.sh
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch rover_bringup foundation.launch.py
```

---

## Validation Commands

Run these inside the container (`./scripts/shell.sh`) or via `docker exec`:

```bash
# List running nodes
ros2 node list
# Expected: /rover_system_health  /rover_lidar_bridge

# List topics
ros2 topic list
# Expected: /diagnostics  /scan  (no /odom or /cmd_vel)

# Verify /scan message type
ros2 topic type /scan
# Expected: sensor_msgs/msg/LaserScan

# Receive one scan
ros2 topic echo /scan --once

# Check publish rate
ros2 topic hz /scan
# Expected: ~7–10 Hz

# Run the Phase 2 doctor (on the Pi host)
./scripts/doctor.sh
```

---

## Safety Guarantees

- No motor commands: `/cmd_vel` and `/odom` are **not published**.
- No SLAM or Nav2 is launched.
- `restart: "no"` in `compose.yaml` — container does not auto-restart.
- No `/dev` mounts — serial devices are never visible inside the container.
- The host `rover-lidar.service` remains the sole owner of `/dev/rover-lidar`.
