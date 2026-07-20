# Rover ROS 2 - Phase 1: Foundation

This directory houses the containerized ROS 2 Jazzy environment for the Yahboom Pi 5 Rover.

## Architecture

To maintain reliability, the ROS 2 container does **not** directly claim or communicate with the hardware serial interfaces (`/dev/rover-esp32`, `/dev/rover-lidar`). The existing host systemd services remain the sole hardware owners.

The container runs on host networking and retrieves telemetry and sensor diagnostics by communicating with the local host APIs:
- Cockpit Server API: `http://127.0.0.1:3000`
- LiDAR Sidecar API: `http://127.0.0.1:3002`

## Environment Setup

1. Copy `.env.example` to `.env`:
   ```bash
   cp .env.example .env
   ```

2. Make the scripts executable:
   ```bash
   chmod +x scripts/*.sh
   ```

## Usage

- **Build**: `./scripts/build.sh` (builds Docker image and compiles the workspace)
- **Start**: `./scripts/up.sh` (executes host pre-checks, starts the container, runs host post-checks)
- **Stop**: `./scripts/down.sh`
- **Shell**: `./scripts/shell.sh` (opens an interactive ROS 2 terminal)
- **Diagnostic Doctor**: `./scripts/doctor.sh` (runs diagnostic audits on hardware isolation and ROS 2 status)
