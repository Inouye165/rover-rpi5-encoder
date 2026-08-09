const { spawn, exec } = require('child_process');

class SlamManager {
  constructor() {
    this.state = 'STOPPED'; // STOPPED, STARTING, RUNNING, STOPPING, ERROR
    this.lastError = null;
    this.lastCheckTime = 0;
    this.cachedStatus = { state: 'STOPPED', nodes: [], lifecycle: null, error: null };
    this.activeCheckPromise = null;
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

  // Fast pre-check: inspect if SLAM node process is running via pgrep (< 50ms vs > 1000ms for ROS 2 CLI)
  async checkProcessAlive() {
    const pgrepCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 pgrep -f async_slam_toolbox_node 2>/dev/null || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 pgrep -f async_slam_toolbox_node 2>/dev/null`;
    const res = await this.runCommand(pgrepCmd, 3000);
    return res.ok && res.stdout.trim().length > 0;
  }

  // Check ROS nodes and lifecycle state with single in-flight promise collapsing
  checkTruthfulState() {
    if (this.activeCheckPromise) {
      return this.activeCheckPromise;
    }

    this.activeCheckPromise = (async () => {
      try {
        const isProcessAlive = await this.checkProcessAlive();

        if (!isProcessAlive) {
          return {
            state: 'STOPPED',
            nodes: [],
            lifecycle: null,
            error: null
          };
        }

        // SLAM process exists, check detailed node & lifecycle status
        const lcCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /slam_toolbox 2>/dev/null" || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /slam_toolbox 2>/dev/null"`;
        const lcRes = await this.runCommand(lcCmd, 5000);

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
            nodes: ['/slam_toolbox'],
            lifecycle: lcOutput.trim() || 'unconfigured/inactive',
            error: null
          };
        } else {
          return {
            state: 'STARTING',
            nodes: ['/slam_toolbox'],
            lifecycle: lcOutput.trim() || 'unconfigured/inactive',
            error: null
          };
        }
      } catch (err) {
        return {
          state: 'ERROR',
          nodes: [],
          lifecycle: null,
          error: `Error checking SLAM state: ${err.message}`
        };
      } finally {
        this.activeCheckPromise = null;
      }
    })();

    return this.activeCheckPromise;
  }

  // Get current SLAM status with brief caching and promise deduplication
  async getStatus() {
    const now = Date.now();
    const cacheTtlMs = 3000;

    if (now - this.lastCheckTime < cacheTtlMs && this.state !== 'STARTING' && this.state !== 'STOPPING') {
      return {
        ok: true,
        state: this.state,
        nodes: this.cachedStatus.nodes,
        lifecycle: this.cachedStatus.lifecycle,
        error: this.lastError
      };
    }

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

  // Start SLAM launch inside rover-ros2 container with state locking and lightweight polling
  async startSlam() {
    if (this.state === 'STARTING' || this.state === 'RUNNING') {
      return {
        ok: false,
        error: `Cannot start SLAM: Current state is ${this.state}`,
        state: this.state
      };
    }

    this.state = 'STARTING';
    this.lastError = null;

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

    const launchCmd = 'docker exec -d -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rover_bringup slam.launch.py" || sudo -n docker exec -d -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch rover_bringup slam.launch.py"';
    await this.runCommand(launchCmd, 8000);

    // Poll for verification up to 30 seconds using 2s lightweight pgrep polling
    const maxPollMs = 30000;
    const startTime = Date.now();

    while (Date.now() - startTime < maxPollMs) {
      await new Promise(r => setTimeout(r, 2000));
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

  // Stop SLAM processes inside rover-ros2 container
  async stopSlam() {
    if (this.state === 'STOPPING' || this.state === 'STOPPED') {
      const check = await this.checkTruthfulState();
      if (check.state === 'STOPPED') {
        this.state = 'STOPPED';
        return { ok: true, state: 'STOPPED' };
      }
    }

    this.state = 'STOPPING';

    const killCmd = `docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "pkill -9 -f 'slam.launch.py|async_slam_toolbox_node|nav2_lifecycle_manager' || true" 2>/dev/null || sudo -n docker exec -e ROS_DOMAIN_ID=42 rover-ros2 bash -c "pkill -9 -f 'slam.launch.py|async_slam_toolbox_node|nav2_lifecycle_manager' || true" 2>/dev/null`;
    await this.runCommand(killCmd, 5000);

    const maxPollMs = 10000;
    const startTime = Date.now();

    while (Date.now() - startTime < maxPollMs) {
      await new Promise(r => setTimeout(r, 1500));
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
