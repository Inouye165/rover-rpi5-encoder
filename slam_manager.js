const { spawn, exec } = require('child_process');

class SlamManager {
  constructor() {
    this.state = 'STOPPED'; // STOPPED, STARTING, RUNNING, STOPPING, ERROR
    this.lastError = null;
    this.managedProcess = null;
    this.lastCheckTime = 0;
    this.cachedStatus = { state: 'STOPPED', nodes: [], lifecycle: null, error: null };
  }

  // Execute shell command asynchronously returning stdout/stderr promise
  runCommand(cmd, timeoutMs = 5000) {
    return new Promise((resolve) => {
      exec(cmd, { timeout: timeoutMs }, (err, stdout, stderr) => {
        if (err) {
          resolve({ ok: false, error: err.message, stdout: stdout || '', stderr: stderr || '' });
        } else {
          resolve({ ok: true, stdout: stdout || '', stderr: stderr || '' });
        }
      });
    });
  }

  // Truthfully check ROS nodes and lifecycle state inside rover-ros2 Docker container
  async checkTruthfulState() {
    const nodeCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null" || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null"`;
    const res = await this.runCommand(nodeCmd, 6000);

    if (!res.ok) {
      return {
        state: 'ERROR',
        nodes: [],
        lifecycle: null,
        error: `Failed to query ROS 2 nodes inside container: ${res.error}`
      };
    }

    const nodeLines = res.stdout.split('\n').map(s => s.trim()).filter(Boolean);
    const hasSlamToolbox = nodeLines.includes('/slam_toolbox');
    const hasLifecycleManager = nodeLines.includes('/lifecycle_manager_slam');

    if (!hasSlamToolbox || !hasLifecycleManager) {
      return {
        state: 'STOPPED',
        nodes: nodeLines.filter(n => n.includes('slam')),
        lifecycle: null,
        error: null
      };
    }

    // Check slam_toolbox lifecycle state
    const lcCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /slam_toolbox 2>/dev/null" || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /slam_toolbox 2>/dev/null"`;
    const lcRes = await this.runCommand(lcCmd, 6000);

    const lcOutput = (lcRes.stdout || '').toLowerCase();
    const isActive = lcOutput.includes('active') && !lcOutput.includes('inactive');

    if (isActive) {
      return {
        state: 'RUNNING',
        nodes: ['/slam_toolbox', '/lifecycle_manager_slam'],
        lifecycle: 'active',
        error: null
      };
    } else if (this.state === 'STOPPING') {
      return {
        state: 'STOPPING',
        nodes: nodeLines.filter(n => n.includes('slam')),
        lifecycle: lcOutput.trim() || 'unconfigured/inactive',
        error: null
      };
    } else {
      return {
        state: 'STARTING',
        nodes: nodeLines.filter(n => n.includes('slam')),
        lifecycle: lcOutput.trim() || 'unconfigured/inactive',
        error: null
      };
    }
  }

  // Get current SLAM status, checking truthful ROS state if not in fast transition
  async getStatus() {
    const now = Date.now();
    // Cache for 2 seconds unless currently in STARTING or STOPPING state
    if (now - this.lastCheckTime < 2000 && this.state !== 'STARTING' && this.state !== 'STOPPING') {
      return {
        ok: true,
        state: this.state,
        nodes: this.cachedStatus.nodes,
        lifecycle: this.cachedStatus.lifecycle,
        error: this.lastError
      };
    }

    // Don't override transient STARTING/STOPPING states during quick polling
    if (this.state === 'STARTING' || this.state === 'STOPPING') {
      return {
        ok: true,
        state: this.state,
        nodes: this.cachedStatus.nodes,
        lifecycle: this.cachedStatus.lifecycle,
        error: this.lastError
      };
    }

    const verified = await this.checkTruthfulState();
    this.lastCheckTime = now;
    this.cachedStatus = verified;

    if (verified.state === 'ERROR' && this.state !== 'ERROR') {
      // Container/ROS error
      this.lastError = verified.error;
    } else if (verified.state !== 'ERROR') {
      this.state = verified.state;
      if (this.state === 'RUNNING' || this.state === 'STOPPED') {
        this.lastError = null;
      }
    }

    return {
      ok: true,
      state: this.state,
      nodes: verified.nodes,
      lifecycle: verified.lifecycle,
      error: this.lastError
    };
  }

  // Start SLAM launch inside rover-ros2 container
  async startSlam() {
    if (this.state === 'STARTING' || this.state === 'RUNNING') {
      return {
        ok: false,
        error: `Cannot start SLAM: Current state is ${this.state}`,
        state: this.state
      };
    }

    // Verify actual ROS state first
    const truthful = await this.checkTruthfulState();
    if (truthful.state === 'RUNNING') {
      this.state = 'RUNNING';
      return {
        ok: false,
        error: 'SLAM is already running on ROS 2 stack.',
        state: 'RUNNING'
      };
    }

    this.state = 'STARTING';
    this.lastError = null;

    const launchCmd = 'docker exec -d -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rover_bringup slam.launch.py" || sudo -n docker exec -d -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rover_bringup slam.launch.py"';
    await this.runCommand(launchCmd, 8000);

    // Poll for verification up to 30 seconds (Ceres solver initialization on Pi takes 15-22 seconds)
    const maxPollMs = 30000;
    const startTime = Date.now();

    while (Date.now() - startTime < maxPollMs) {
      await new Promise(r => setTimeout(r, 1000));
      const check = await this.checkTruthfulState();
      this.cachedStatus = check;

      if (check.state === 'RUNNING') {
        this.state = 'RUNNING';
        this.lastError = null;
        return {
          ok: true,
          state: 'RUNNING',
          nodes: check.nodes,
          lifecycle: check.lifecycle
        };
      }
    }

    // Timeout reached without active RUNNING state
    this.state = 'ERROR';
    this.lastError = 'SLAM start timed out: slam_toolbox lifecycle node did not reach active state within 30 seconds.';
    return {
      ok: false,
      error: this.lastError,
      state: 'ERROR'
    };
  }

  // Stop SLAM processes inside rover-ros2 container without stopping container or foundation nodes
  async stopSlam() {
    if (this.state === 'STOPPING' || this.state === 'STOPPED') {
      // Double check truthful state
      const check = await this.checkTruthfulState();
      if (check.state === 'STOPPED') {
        this.state = 'STOPPED';
        return { ok: true, state: 'STOPPED' };
      }
    }

    this.state = 'STOPPING';

    // Run targeted pkill inside container targeting ONLY SLAM processes
    const killCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "pkill -9 -f 'slam.launch.py|async_slam_toolbox_node|nav2_lifecycle_manager' || true" 2>/dev/null || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "pkill -9 -f 'slam.launch.py|async_slam_toolbox_node|nav2_lifecycle_manager' || true" 2>/dev/null`;
    await this.runCommand(killCmd, 5000);

    // Poll for verification up to 10 seconds
    const maxPollMs = 10000;
    const startTime = Date.now();

    while (Date.now() - startTime < maxPollMs) {
      await new Promise(r => setTimeout(r, 1000));
      const check = await this.checkTruthfulState();
      this.cachedStatus = check;

      if (check.state === 'STOPPED') {
        this.state = 'STOPPED';
        this.lastError = null;
        return {
          ok: true,
          state: 'STOPPED'
        };
      }
    }

    // Force kill if still stopping
    const forceKillCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "pkill -9 -f 'slam.launch.py|async_slam_toolbox_node|nav2_lifecycle_manager' || true" 2>/dev/null`;
    await this.runCommand(forceKillCmd, 5000);

    this.state = 'STOPPED';
    this.lastError = null;
    return {
      ok: true,
      state: 'STOPPED'
    };
  }
}

const slamManager = new SlamManager();
module.exports = slamManager;
