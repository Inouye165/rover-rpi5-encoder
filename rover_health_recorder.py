#!/usr/bin/env python3
"""
rover_health_recorder.py - Persistent Boot/Crash Tracker & Flight Recorder Daemon

Runs as a systemd background service on the RPi5 host system.
1. Maintains /var/lib/rover-health/boot-history.jsonl tracking clean vs unclean reboots.
2. Maintains a rolling log /var/log/rover-health/flight-recorder.log sampling every 5 seconds.
"""

import os
import sys
import time
import json
import socket
import signal
import subprocess
import glob
from datetime import datetime, timezone

DATA_DIR = "/var/lib/rover-health"
LOG_DIR = "/var/log/rover-health"
BOOT_HISTORY_FILE = os.path.join(DATA_DIR, "boot-history.jsonl")
SHUTDOWN_FLAG_FILE = os.path.join(DATA_DIR, "clean_shutdown.flag")
LAST_METRICS_FILE = os.path.join(DATA_DIR, "last-recorder-state.json")
RECORDER_LOG_FILE = os.path.join(LOG_DIR, "flight-recorder.log")

MAX_LOG_BYTES = 5 * 1024 * 1024  # 5 MB per log file
MAX_ROTATIONS = 3  # Keep 3 backups (15 MB max total storage)
SAMPLE_INTERVAL_SEC = 5.0

running = True

def signal_handler(signum, frame):
    global running
    print(f"[RoverHealth] Received signal {signum}. Writing clean shutdown marker...")
    try:
        os.makedirs(DATA_DIR, exist_ok=True)
        with open(SHUTDOWN_FLAG_FILE, "w") as f:
            f.write(f"CLEAN_SHUTDOWN at {datetime.now(timezone.utc).isoformat()}\n")
    except Exception as e:
        print(f"[RoverHealth] Error writing shutdown marker: {e}", file=sys.stderr)
    running = False

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

def get_boot_id():
    try:
        with open("/proc/sys/kernel/random/boot_id", "r") as f:
            return f.read().strip()
    except Exception:
        return "unknown"

def get_uptime_sec():
    try:
        with open("/proc/uptime", "r") as f:
            return float(f.read().split()[0])
    except Exception:
        return 0.0

def record_boot_event():
    os.makedirs(DATA_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

    current_boot_id = get_boot_id()
    current_time = datetime.now(timezone.utc).isoformat()

    # Read previous recorder state if available
    prev_state = {}
    if os.path.exists(LAST_METRICS_FILE):
        try:
            with open(LAST_METRICS_FILE, "r") as f:
                prev_state = json.load(f)
        except Exception:
            pass

    clean_flag_exists = os.path.exists(SHUTDOWN_FLAG_FILE)
    shutdown_status = "CLEAN_SHUTDOWN" if clean_flag_exists else "UNCLEAN_SHUTDOWN"
    
    incident_id = None
    if not clean_flag_exists:
        ts_compact = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
        incident_id = f"INCIDENT-{ts_compact}"

    # Remove clean shutdown flag for the current boot session
    if clean_flag_exists:
        try:
            os.remove(SHUTDOWN_FLAG_FILE)
        except Exception:
            pass

    boot_record = {
        "event": "BOOT",
        "timestamp": current_time,
        "boot_id": current_boot_id,
        "shutdown_status": shutdown_status,
        "incident_id": incident_id,
        "prev_boot_id": prev_state.get("boot_id"),
        "prev_boot_last_recorder_ts": prev_state.get("timestamp"),
        "prev_uptime_s": prev_state.get("uptime_s")
    }

    try:
        with open(BOOT_HISTORY_FILE, "a") as f:
            f.write(json.dumps(boot_record) + "\n")
        print(f"[RoverHealth] Boot logged: status={shutdown_status}, incident={incident_id}")
    except Exception as e:
        print(f"[RoverHealth] Error recording boot event: {e}", file=sys.stderr)

def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(float(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None

def get_throttled():
    try:
        res = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1)
        if res.returncode == 0 and "=" in res.stdout:
            return res.stdout.strip().split("=")[1]
    except Exception:
        pass
    return "0x0"

def get_mem_info():
    avail_mb = 0
    swap_free_mb = 0
    swap_total_mb = 0
    try:
        with open("/proc/meminfo", "r") as f:
            for line in f:
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == "MemAvailable:":
                    avail_mb = round(float(parts[1]) / 1024.0, 1)
                elif parts[0] == "SwapTotal:":
                    swap_total_mb = round(float(parts[1]) / 1024.0, 1)
                elif parts[0] == "SwapFree:":
                    swap_free_mb = round(float(parts[1]) / 1024.0, 1)
    except Exception:
        pass
    swap_used_mb = round(max(0.0, swap_total_mb - swap_free_mb), 1)
    return avail_mb, swap_used_mb

def get_load_avg():
    try:
        with open("/proc/loadavg", "r") as f:
            parts = f.read().split()
            return [float(parts[0]), float(parts[1]), float(parts[2])]
    except Exception:
        return [0.0, 0.0, 0.0]

def find_pid_by_cmdline(pattern):
    try:
        for p in glob.glob("/proc/[0-9]*/cmdline"):
            try:
                with open(p, "rb") as f:
                    cmd = f.read().replace(b"\x00", b" ").decode("utf-8", errors="ignore")
                    if pattern in cmd:
                        pid = p.split("/")[2]
                        return int(pid)
            except Exception:
                continue
    except Exception:
        pass
    return None

def get_process_rss_mb(pid):
    if not pid:
        return 0.0
    try:
        with open(f"/proc/{pid}/status", "r") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    parts = line.split()
                    return round(float(parts[1]) / 1024.0, 1)
    except Exception:
        pass
    return 0.0

def check_tcp_port(host, port, timeout_s=0.5):
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except Exception:
        return False

def rotate_log_if_needed():
    if not os.path.exists(RECORDER_LOG_FILE):
        return
    try:
        size = os.path.getsize(RECORDER_LOG_FILE)
        if size >= MAX_LOG_BYTES:
            for i in range(MAX_ROTATIONS - 1, 0, -1):
                src = f"{RECORDER_LOG_FILE}.{i}"
                dst = f"{RECORDER_LOG_FILE}.{i+1}"
                if os.path.exists(src):
                    os.rename(src, dst)
            os.rename(RECORDER_LOG_FILE, f"{RECORDER_LOG_FILE}.1")
    except Exception as e:
        print(f"[RoverHealth] Log rotation error: {e}", file=sys.stderr)

def main():
    print("[RoverHealth] Starting persistent health recorder service...")
    record_boot_event()

    boot_id = get_boot_id()
    prev_idle = 0
    prev_total = 0

    while running:
        t0 = time.time()
        now_iso = datetime.now(timezone.utc).isoformat()
        uptime_s = get_uptime_sec()

        # CPU % calculation from /proc/stat
        cpu_pct = 0.0
        try:
            with open("/proc/stat", "r") as f:
                fields = [float(x) for x in f.readline().split()[1:8]]
                idle = fields[3] + fields[4]
                total = sum(fields)
                if prev_total > 0:
                    diff_idle = idle - prev_idle
                    diff_total = total - prev_total
                    if diff_total > 0:
                        cpu_pct = round(100.0 * (1.0 - diff_idle / diff_total), 1)
                prev_idle = idle
                prev_total = total
        except Exception:
            pass

        temp_c = get_cpu_temp()
        throttled = get_throttled()
        ram_avail_mb, swap_used_mb = get_mem_info()
        load1, load5, load15 = get_load_avg()

        # Service PIDs & RSS
        sidecar_pid = find_pid_by_cmdline("rplidar_sidecar.py")
        sidecar_rss_mb = get_process_rss_mb(sidecar_pid)

        server_pid = find_pid_by_cmdline("server.js")
        server_rss_mb = get_process_rss_mb(server_pid)

        # Port pings (cheap TCP)
        cockpit_alive = check_tcp_port("127.0.0.1", 3000, 0.5)
        lidar_sidecar_alive = check_tcp_port("127.0.0.1", 3002, 0.5)

        # SLAM process check via pgrep pattern
        slam_pid = find_pid_by_cmdline("async_slam_toolbox_node")
        slam_alive = slam_pid is not None

        # Container check
        ros2_container_alive = find_pid_by_cmdline("rover-ros2") is not None or find_pid_by_cmdline("rover_encoder_odometry") is not None

        metrics = {
            "timestamp": now_iso,
            "boot_id": boot_id,
            "uptime_s": round(uptime_s, 1),
            "cpu_temp_c": temp_c,
            "throttled": throttled,
            "ram_avail_mb": ram_avail_mb,
            "swap_used_mb": swap_used_mb,
            "load_1m": load1,
            "load_5m": load5,
            "cpu_pct": cpu_pct,
            "rplidar_sidecar": {"alive": sidecar_pid is not None, "rss_mb": sidecar_rss_mb},
            "rover_server": {"alive": cockpit_alive, "rss_mb": server_rss_mb},
            "lidar_sidecar_alive": lidar_sidecar_alive,
            "ros2_container_alive": ros2_container_alive,
            "slam_alive": slam_alive
        }

        # Save latest state for next boot unclean check
        try:
            with open(LAST_METRICS_FILE, "w") as f:
                json.dump(metrics, f)
        except Exception:
            pass

        # Write metric record to flight recorder log
        try:
            rotate_log_if_needed()
            with open(RECORDER_LOG_FILE, "a") as f:
                f.write(json.dumps(metrics) + "\n")
        except Exception as e:
            print(f"[RoverHealth] Log write error: {e}", file=sys.stderr)

        elapsed = time.time() - t0
        sleep_dur = max(0.1, SAMPLE_INTERVAL_SEC - elapsed)
        time.sleep(sleep_dur)

    print("[RoverHealth] Service stopped.")

if __name__ == "__main__":
    main()
