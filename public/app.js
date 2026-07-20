let ws = null;
let reconnectTimer = null;
let reconnectInterval = 1000;
const maxReconnectInterval = 16000;
let driveArmed = false;
let realOdomActive = false;
let gpDeadmanPressed = false;

// Calibration variables
let currentWheelDiameter = 0.065; // synchronized from ESP32 parameters
let currentTrackWidth = 0.170;    // synchronized from ESP32 parameters
let activeTest = null;            // 'distance', 'rotation_cw', 'rotation_ccw', 'out_and_back'
let testTimer = null;
let testStartOdom = null;
let calibrationDatabase = null;
let straightDriveLocked = false;
let straightTestOffsets = null;
let autoTestActive = false;
let autotestStartTicks = [0, 0, 0, 0];
let autotestInitialized = false;
let visualX = 0.0;
let visualY = 0.0;
let visualYaw = 0.0;
let prevLeftTicks = 0.0;
let prevRightTicks = 0.0;
let clientTestLogs = [];
let currentVisualStep = 0;
let lastProcessedStep = 0;
let currentLegMaxDrift = 0;
let currentLegMaxMismatch = 0;
let legResults = {};

// LiDAR straight-line test globals
let lidarOdomPath = [];
let lidarPosePath = [];
let calibPathHistory = []; // array of { tier: 'SLOW'|'MED'|'FAST', odom: [...], lidar: [...] }
let lastTelemetrySpeedTier = 'SLOW';
let testScanPollInterval = null;
let lastLidarScanForTest = null;

function startTestScanPolling() {
  if (testScanPollInterval) return;
  testScanPollInterval = setInterval(() => {
    fetch('/api/lidar/scan')
      .then(r => r.json())
      .then(data => {
        if (data && data.points) {
          lastLidarScanForTest = data;
          drawLidarTestCanvas();
        }
      })
      .catch(err => console.error('Failed to poll test scan:', err));
  }, 200);
}

function stopTestScanPolling() {
  if (testScanPollInterval) {
    clearInterval(testScanPollInterval);
    testScanPollInterval = null;
  }
  lastLidarScanForTest = null;
  drawLidarTestCanvas();
}

function saveCurrentPathToHistory(tier) {
  if (lidarOdomPath.length > 0 || lidarPosePath.length > 0) {
    calibPathHistory.push({
      tier: tier || 'SLOW',
      odom: [...lidarOdomPath],
      lidar: [...lidarPosePath]
    });
  }
}
let orientationStep = 1;
let orientationVerified = localStorage.getItem('lidar_orientation_verified') === 'true';
let wizardPollInterval = null;

// Track Interference global variables for drawing & UI
let closestFrontObstacle = null; // { x, y, dist }
let closestLeftObstacle = null;  // { x, y, dist }
let closestRightObstacle = null; // { x, y, dist }
let monitoredTrackWidth = 0.60;  // default track width in meters
let lidarTestState = 'IDLE';     // local copy of current calibration state


// Odom and IMU State variables
let m1Speed = 0, m2Speed = 0, m3Speed = 0, m4Speed = 0;
let odomX = 0, odomY = 0, odomTheta = 0; // x, y (mm), theta (rad)
let realIMUActive = false;
let imuPitch = 0, imuRoll = 0, imuYaw = 0;
let pathHistory = []; // list of {x, y} coordinates for tracing
const maxPathPoints = 800;
const trackWidth = 160; // distance between wheels in mm

// UI Elements
const wsStatus = document.getElementById('ws-status');
const serialStatus = document.getElementById('serial-status');
const comPortInput = document.getElementById('com-port-input');
const btnChangePort = document.getElementById('btn-change-port');

const batteryFill = document.getElementById('battery-fill');
const batteryValue = document.getElementById('battery-value');
const batteryContainer = document.getElementById('battery-container');

// Driving Controls Elements
const ctrlForward = document.getElementById('ctrl-forward');
const ctrlLeft = document.getElementById('ctrl-left');
const ctrlStopCenter = document.getElementById('ctrl-stop-center');
const ctrlRight = document.getElementById('ctrl-right');
const ctrlReverse = document.getElementById('ctrl-reverse');
const ctrlSpinLeft = document.getElementById('ctrl-spin-left');
const ctrlSpinRight = document.getElementById('ctrl-spin-right');

const syncSpeedSlider = document.getElementById('sync-speed-slider');
const syncSpeedReadout = document.getElementById('sync-speed-readout');

const sliderM1 = document.getElementById('speed-m1');
const sliderM2 = document.getElementById('speed-m2');
const sliderM3 = document.getElementById('speed-m3');
const sliderM4 = document.getElementById('speed-m4');
const readoutM1 = document.getElementById('readout-m1');
const readoutM2 = document.getElementById('readout-m2');
const readoutM3 = document.getElementById('readout-m3');
const readoutM4 = document.getElementById('readout-m4');
const btnEstop = document.getElementById('btn-estop');
const btnMotorProof = document.getElementById('btn-motor-proof');
const encoderActivity = document.getElementById('encoder-activity');

// Camera Elements and State
const cameraStream = document.getElementById('camera-stream');
const cameraPlaceholder = document.getElementById('camera-placeholder');
const btnToggleCamera = document.getElementById('btn-toggle-camera');
const btnFullscreenCamera = document.getElementById('btn-fullscreen-camera');
const cameraStatusDot = document.getElementById('camera-status-dot');
const cameraStatusText = document.getElementById('camera-status-text');
const cameraViewport = document.getElementById('camera-viewport');
let isCameraStreaming = false;

// Telemetry Elements
const streamTotal = document.getElementById('stream-total');
const streamRealtime = document.getElementById('stream-realtime');
const streamSpeed = document.getElementById('stream-speed');

const speedValM1 = document.getElementById('telemetry-speed-m1');
const speedValM2 = document.getElementById('telemetry-speed-m2');
const speedValM3 = document.getElementById('telemetry-speed-m3');
const speedValM4 = document.getElementById('telemetry-speed-m4');

const realValM1 = document.getElementById('telemetry-real-m1');
const realValM2 = document.getElementById('telemetry-real-m2');
const realValM3 = document.getElementById('telemetry-real-m3');
const realValM4 = document.getElementById('telemetry-real-m4');

const totalValM1 = document.getElementById('telemetry-total-m1');
const totalValM2 = document.getElementById('telemetry-total-m2');
const totalValM3 = document.getElementById('telemetry-total-m3');
const totalValM4 = document.getElementById('telemetry-total-m4');

const cardM1 = document.getElementById('card-m1');
const cardM2 = document.getElementById('card-m2');
const cardM3 = document.getElementById('card-m3');
const cardM4 = document.getElementById('card-m4');

// Config Elements
const configForm = document.getElementById('config-form');
const motorType = document.getElementById('motor-type');
const deadband = document.getElementById('deadband');
const phaseLines = document.getElementById('phase-lines');
const reductionRatio = document.getElementById('reduction-ratio');
const wheelDiameter = document.getElementById('wheel-diameter');
const pidP = document.getElementById('pid-p');
const pidI = document.getElementById('pid-i');
const pidD = document.getElementById('pid-d');
const btnReadFlash = document.getElementById('btn-read-flash');
const btnResetFlash = document.getElementById('btn-reset-flash');

// Logs Elements
const terminalConsole = document.getElementById('terminal-console');
const terminalCommandInput = document.getElementById('terminal-command-input');
const btnSendRawCommand = document.getElementById('btn-send-raw-command');
const btnClearLogs = document.getElementById('btn-clear-logs');

// Encoder calculations for RPM and MPH
let prevEncoderTime = null;
let prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;

// Connect WebSocket
function connectWebSocket() {
  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${wsProtocol}//${window.location.host}`;
  
  logSystem(`Connecting to server WebSocket at ${wsUrl}...`);
  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    logSystem('WebSocket connected successfully.');
    updateBadge(wsStatus, 'ok', 'WS: Connected');
    reconnectInterval = 1000; // Reset reconnect timeout backoff
    if (reconnectTimer) clearTimeout(reconnectTimer);
    
    // Sync telemetry checkbox state with server
    sendUploadConfig();
    
    // Automatically query firmware identity from ESP32
    fetch('/api/firmware').catch(err => console.error('Firmware query failed:', err));

    // Request current calibration database
    sendServerMessage({ type: 'get_calibration_db' });
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleServerMessage(msg);
    } catch (e) {
      console.error('Error parsing WS message:', e);
    }
  };

  ws.onclose = () => {
    updateBadge(wsStatus, 'alert', 'WS: Disconnected');
    logSystem('WebSocket connection lost. Retrying...');
    scheduleReconnect();
  };

  ws.onerror = (err) => {
    console.error('WebSocket Error:', err);
    updateBadge(wsStatus, 'alert', 'WS: Connection Error');
  };
}

function scheduleReconnect() {
  if (reconnectTimer) clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => {
    reconnectInterval = Math.min(reconnectInterval * 2, maxReconnectInterval);
    connectWebSocket();
  }, reconnectInterval);
}

// Update UI Badge Status safely without duplicating text nodes
function updateBadge(badgeElement, state, text) {
  if (!badgeElement) return;
  const indicator = badgeElement.querySelector('.status-indicator');
  if (indicator) {
    indicator.className = 'status-indicator ' + state;
  }
  
  // Remove all child nodes except the indicator span
  Array.from(badgeElement.childNodes).forEach(node => {
    if (node !== indicator) {
      badgeElement.removeChild(node);
    }
  });
  
  // Append the updated status text
  badgeElement.appendChild(document.createTextNode(' ' + text));
}

// Log Terminal Helpers
function logSystem(msg) {
  addLogLine(msg, 'system-line');
}

function logSerialIn(msg) {
  addLogLine(msg, 'in-line');
}

function logSerialOut(msg) {
  addLogLine(msg, 'out-line');
}

function logSerialOutErr(msg) {
  addLogLine(`[Error Out] ${msg}`, 'err-line');
}

function addLogLine(text, className) {
  const line = document.createElement('div');
  line.className = `log-line ${className}`;
  line.textContent = text;
  terminalConsole.appendChild(line);
  
  // Scroll to bottom
  terminalConsole.scrollTop = terminalConsole.scrollHeight;
}

// Send Commands via WS
function sendServerMessage(data) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(data));
  } else {
    logSystem('Error: WebSocket is not open to send command.');
  }
}

// Send Upload streams settings
function sendUploadConfig() {
  const t = streamTotal.checked ? 1 : 0;
  const r = streamRealtime.checked ? 1 : 0;
  const s = streamSpeed.checked ? 1 : 0;
  sendServerMessage({ type: 'set_upload', upload: [t, r, s] });
}

// Handle Server Messages
function handleServerMessage(msg) {
  switch (msg.type) {
    case 'status':
      if (msg.key === 'serial') {
        if (msg.port) {
          comPortInput.value = msg.port;
        }
        if (msg.val === 'connected') {
          updateBadge(serialStatus, 'ok', `Serial: Connected (${msg.port})`);
          logSystem(`Serial port ${msg.port} connected.`);
        } else if (msg.val === 'connecting') {
          updateBadge(serialStatus, 'alert', `Serial: Connecting...`);
          logSystem(`Attempting serial connection to ${msg.port}...`);
        } else {
          updateBadge(serialStatus, 'alert', 'Serial: Disconnected');
          logSystem(`Serial port disconnected.${msg.error ? ' Error: ' + msg.error : ''}`);
        }
      }
      break;

    case 'battery': {
      if (msg.voltage <= 2.0) {
        batteryFill.style.width = '0%';
        batteryValue.textContent = 'Unknown';
        batteryFill.style.background = '#64748b'; // gray
      } else {
        // Map 6.0V - 8.4V battery pack (2S LiPo usually) to percentage
        const minV = 6.0;
        const maxV = 8.4;
        const pct = Math.min(100, Math.max(0, ((msg.voltage - minV) / (maxV - minV)) * 100));
        batteryFill.style.width = `${pct}%`;
        batteryValue.textContent = `${msg.voltage.toFixed(2)} V`;
        
        // Update battery colors based on voltage levels
        if (msg.voltage > 7.4) {
          batteryFill.style.background = 'linear-gradient(90deg, #39ff14, #00f2fe)';
        } else if (msg.voltage > 6.8) {
          batteryFill.style.background = 'linear-gradient(90deg, #ffb700, #ffea00)';
        } else {
          batteryFill.style.background = 'linear-gradient(90deg, #ff0055, #ff0000)';
        }
      }
      break;
    }

    case 'encoder_total': {
      totalValM1.textContent = msg.m1;
      totalValM2.textContent = msg.m2;
      totalValM3.textContent = msg.m3;
      totalValM4.textContent = msg.m4;

      // Update diagnostics page ticks if they exist
      const testTicksM1 = document.getElementById('test-ticks-m1');
      if (testTicksM1) {
        testTicksM1.textContent = msg.m1 - encoderOffsets[0];
        document.getElementById('test-ticks-m2').textContent = msg.m2 - encoderOffsets[1];
        document.getElementById('test-ticks-m3').textContent = msg.m3 - encoderOffsets[2];
        document.getElementById('test-ticks-m4').textContent = msg.m4 - encoderOffsets[3];
      }

      // Initialize straight drive test offsets if not yet set
      if (straightTestOffsets === null) {
        straightTestOffsets = [msg.m1, msg.m2, msg.m3, msg.m4];
      }

      // Calculate relative ticks for straight test area
      const relM1 = msg.m1 - straightTestOffsets[0];
      const relM2 = msg.m2 - straightTestOffsets[1];
      const relM3 = msg.m3 - straightTestOffsets[2];
      const relM4 = msg.m4 - straightTestOffsets[3];

      // Update straight drive test area ticks
      const straightTicksM1 = document.getElementById('straight-ticks-m1');
      if (straightTicksM1) {
        straightTicksM1.textContent = relM1;
        document.getElementById('straight-ticks-m2').textContent = relM2;
        document.getElementById('straight-ticks-m3').textContent = relM3;
        document.getElementById('straight-ticks-m4').textContent = relM4;
      }

      // Update symmetry metrics
      if (typeof updateStraightDriveMetrics === 'function') {
        updateStraightDriveMetrics(relM1, relM2, relM3, relM4);
      }
      // Calculate RPM and MPH
      const now = Date.now();
      if (prevEncoderTime !== null) {
        const dt = (now - prevEncoderTime) / 1000.0;
        if (dt > 0.1) {
          const deltaM1 = msg.m1 - prevM1;
          const deltaM2 = msg.m2 - prevM2;
          const deltaM3 = msg.m3 - prevM3;
          const deltaM4 = msg.m4 - prevM4;

          const tpr = 937.2;
          const rpm1 = (deltaM1 / dt) / tpr * 60.0;
          const rpm2 = (deltaM2 / dt) / tpr * 60.0;
          const rpm3 = (deltaM3 / dt) / tpr * 60.0;
          const rpm4 = (deltaM4 / dt) / tpr * 60.0;

          const rpmToMph = 2.559 * Math.PI * 5.0 / 5280.0; // ~0.007613
          const mph1 = rpm1 * rpmToMph;
          const mph2 = rpm2 * rpmToMph;
          const mph3 = rpm3 * rpmToMph;
          const mph4 = rpm4 * rpmToMph;

          speedValM1.innerHTML = `${Math.abs(mph1).toFixed(2)} <small>mph</small>`;
          speedValM2.innerHTML = `${Math.abs(mph2).toFixed(2)} <small>mph</small>`;
          speedValM3.innerHTML = `${Math.abs(mph3).toFixed(2)} <small>mph</small>`;
          speedValM4.innerHTML = `${Math.abs(mph4).toFixed(2)} <small>mph</small>`;

          realValM1.textContent = rpm1.toFixed(1);
          realValM2.textContent = rpm2.toFixed(1);
          realValM3.textContent = rpm3.toFixed(1);
          realValM4.textContent = rpm4.toFixed(1);

          // Update diagnostics page RPM values
          const testRpmM1 = document.getElementById('test-rpm-m1');
          if (testRpmM1) {
            testRpmM1.textContent = rpm1.toFixed(1);
            document.getElementById('test-rpm-m2').textContent = rpm2.toFixed(1);
            document.getElementById('test-rpm-m3').textContent = rpm3.toFixed(1);
            document.getElementById('test-rpm-m4').textContent = rpm4.toFixed(1);
          }
          // Update straight test area RPM
          const straightRpmM1 = document.getElementById('straight-rpm-m1');
          if (straightRpmM1) {
            straightRpmM1.textContent = rpm1.toFixed(1);
            document.getElementById('straight-rpm-m2').textContent = rpm2.toFixed(1);
            document.getElementById('straight-rpm-m3').textContent = rpm3.toFixed(1);
            document.getElementById('straight-rpm-m4').textContent = rpm4.toFixed(1);
          }

          if (autoTestActive && autotestInitialized) {
            const rpmLeft = (rpm1 + rpm3) / 2.0;
            const rpmRight = (rpm2 + rpm4) / 2.0;
            updateAutoTestVisualizer(msg.m1, msg.m2, msg.m3, msg.m4, rpmLeft, rpmRight);
          }
        }
      }
      prevM1 = msg.m1;
      prevM2 = msg.m2;
      prevM3 = msg.m3;
      prevM4 = msg.m4;
      prevEncoderTime = now;

      break;
    }

    case 'motor_speeds': {
      const testPwmM1 = document.getElementById('test-pwm-m1');
      if (testPwmM1 && Array.isArray(msg.speeds)) {
        testPwmM1.textContent = msg.speeds[0];
        document.getElementById('test-pwm-m2').textContent = msg.speeds[1];
        document.getElementById('test-pwm-m3').textContent = msg.speeds[2];
        document.getElementById('test-pwm-m4').textContent = msg.speeds[3];
      }
      break;
    }

    case 'encoder_realtime':
      realValM1.textContent = msg.m1;
      realValM2.textContent = msg.m2;
      realValM3.textContent = msg.m3;
      realValM4.textContent = msg.m4;
      break;

    case 'speed':
      m1Speed = msg.m1;
      m2Speed = msg.m2;
      m3Speed = msg.m3;
      m4Speed = msg.m4;

      speedValM1.innerHTML = `${msg.m1.toFixed(1)} <small>mm/s</small>`;
      speedValM2.innerHTML = `${msg.m2.toFixed(1)} <small>mm/s</small>`;
      speedValM3.innerHTML = `${msg.m3.toFixed(1)} <small>mm/s</small>`;
      speedValM4.innerHTML = `${msg.m4.toFixed(1)} <small>mm/s</small>`;
      
      // Update animations
      updateWheelAnimation(cardM1, msg.m1);
      updateWheelAnimation(cardM2, msg.m2);
      updateWheelAnimation(cardM3, msg.m3);
      updateWheelAnimation(cardM4, msg.m4);
      break;

    case 'encoder_activity':
      if (encoderActivity) {
        const c = msg.counts || [];
        const movementState = msg.hasNonZero ? `COUNTS: ${c.join(', ')}` : 'all zero';
        encoderActivity.textContent = `Encoder pkts: ${msg.packets} (${movementState})`;
      }
      break;

    case 'attitude':
      realIMUActive = true;
      imuRoll = msg.roll;
      imuPitch = msg.pitch;
      imuYaw = msg.yaw;
      document.getElementById('imu-roll').textContent = `${msg.roll.toFixed(1)}°`;
      document.getElementById('imu-pitch').textContent = `${msg.pitch.toFixed(1)}°`;
      document.getElementById('imu-yaw').textContent = `${msg.yaw.toFixed(1)}°`;
      odomTheta = msg.yaw * Math.PI / 180;
      update3DModelRotation(msg.pitch, msg.roll, msg.yaw);
      break;

    case 'motor_proof_status':
      logSystem(`[Motor Proof] ${msg.message}`);
      if (btnMotorProof) {
        if (msg.status === 'start') {
          btnMotorProof.disabled = true;
          btnMotorProof.textContent = 'Motor Proof Running...';
        }
        if (msg.status === 'done' || msg.status === 'error') {
          btnMotorProof.disabled = false;
          btnMotorProof.textContent = 'Run Motor Power Proof';
        }
      }
      break;

    case 'odom':
      realOdomActive = true;
      const odomXE = document.getElementById('odom-x-real');
      const odomYE = document.getElementById('odom-y-real');
      const odomYawE = document.getElementById('odom-yaw-real');
      const odomLeftE = document.getElementById('odom-left-dist');
      const odomRightE = document.getElementById('odom-right-dist');
      const odomVE = document.getElementById('odom-v-real');
      const odomWE = document.getElementById('odom-w-real');
      
      const odomEncM1 = document.getElementById('odom-enc-m1');
      const odomEncM2 = document.getElementById('odom-enc-m2');
      const odomEncM3 = document.getElementById('odom-enc-m3');
      const odomEncM4 = document.getElementById('odom-enc-m4');

      if (odomXE) odomXE.innerText = `${msg.x.toFixed(3)} m`;
      if (odomYE) odomYE.innerText = `${msg.y.toFixed(3)} m`;
      if (odomYawE) odomYawE.innerText = `${(msg.yaw * 180 / Math.PI).toFixed(1)}°`;

      // Update compass heading visualizer
      latestOdomYaw = msg.yaw;
      const arrowSvg = document.getElementById('cal-rot-arrow-group');
      const compassAngle = document.getElementById('cal-rot-compass-angle');
      const yawDeg = msg.yaw * 180 / Math.PI;
      if (arrowSvg) arrowSvg.style.transform = `rotate(${-yawDeg.toFixed(1)}deg)`;
      if (compassAngle) compassAngle.innerText = `${yawDeg.toFixed(1)}°`;
      if (odomLeftE) odomLeftE.innerText = `${msg.left_dist.toFixed(3)} m`;
      if (odomRightE) odomRightE.innerText = `${msg.right_dist.toFixed(3)} m`;
      if (odomVE) odomVE.innerText = `${msg.v.toFixed(3)} m/s`;
      if (odomWE) odomWE.innerText = `${msg.w.toFixed(3)} rad/s`;

      if (odomEncM1) odomEncM1.innerText = msg.encoders[0];
      if (odomEncM2) odomEncM2.innerText = msg.encoders[1];
      if (odomEncM3) odomEncM3.innerText = msg.encoders[2];
      if (odomEncM4) odomEncM4.innerText = msg.encoders[3];

      // Append coordinates in meters directly for canvas scaling
      pathHistory.push({ x: msg.x, y: msg.y });
      if (pathHistory.length > maxPathPoints) {
        pathHistory.shift();
      }
      drawPath();
      break;

    case 'backtrack_status':
      logSystem(`[Backtrack] Status: ${msg.status}${msg.reason ? ' | Reason: ' + msg.reason : ''}`);
      const btState = document.getElementById('backtrack-state-lbl');
      const btProgress = document.getElementById('backtrack-progress-bar');
      if (btState) {
        btState.innerText = msg.status.toUpperCase();
        btState.style.color = (msg.status === 'completed') ? '#10b981' : ((msg.status === 'aborted') ? '#ef4444' : '#f59e0b');
      }
      if (btProgress && msg.index !== undefined && msg.total !== undefined) {
        const percent = ((msg.total - msg.index) / msg.total * 100).toFixed(1);
        btProgress.style.width = `${percent}%`;
        btProgress.innerText = `${percent}%`;
      }
      if (msg.status === 'completed' || msg.status === 'aborted') {
        if (btProgress) {
          btProgress.style.width = '0%';
          btProgress.innerText = '';
        }
      }
      break;

    case 'path_status':
      const prState = document.getElementById('path-recording-lbl');
      const prBreadcrumbs = document.getElementById('path-breadcrumbs-lbl');
      if (prState) {
        prState.innerText = msg.recording ? 'RECORDING' : 'Idle';
        prState.style.color = msg.recording ? '#ef4444' : '#6b7280';
      }
      if (prBreadcrumbs) {
        prBreadcrumbs.innerText = msg.pathLength;
      }
      break;

    case 'limits_status':
      const flLabel = document.getElementById('limits-testing-lbl');
      if (flLabel) {
        flLabel.innerText = msg.floorTesting ? 'FLOOR TESTING (0.17 m/s)' : 'UNCLAMPED (0.80 m/s)';
        flLabel.style.color = msg.floorTesting ? '#f59e0b' : '#10b981';
      }
      const flChk = document.getElementById('limits-floor-testing');
      if (flChk) {
        flChk.checked = msg.floorTesting;
      }
      break;

    case 'imu':
      realIMUActive = true;
      imuYaw = msg.yaw;
      imuPitch = msg.pitch;
      imuRoll = msg.roll;
      
      // Update UI readouts
      document.getElementById('imu-roll').textContent = `${msg.roll.toFixed(1)}°`;
      document.getElementById('imu-pitch').textContent = `${msg.pitch.toFixed(1)}°`;
      document.getElementById('imu-yaw').textContent = `${msg.yaw.toFixed(1)}°`;
      
      document.getElementById('imu-ax').innerHTML = `${msg.ax.toFixed(2)} <small>g</small>`;
      document.getElementById('imu-ay').innerHTML = `${msg.ay.toFixed(2)} <small>g</small>`;
      document.getElementById('imu-az').innerHTML = `${msg.az.toFixed(2)} <small>g</small>`;
      
      document.getElementById('imu-gx').innerHTML = `${msg.gx.toFixed(1)} <small>°/s</small>`;
      document.getElementById('imu-gy').innerHTML = `${msg.gy.toFixed(1)} <small>°/s</small>`;
      document.getElementById('imu-gz').innerHTML = `${msg.gz.toFixed(1)} <small>°/s</small>`;
      
      // Override odometry heading with IMU yaw
      odomTheta = msg.yaw * Math.PI / 180;
      
      // Update 3D Model rotation
      update3DModelRotation(msg.pitch, msg.roll, msg.yaw);
      break;

    case 'raw_serial_in':
      logSerialIn(msg.data);
      break;

    case 'raw_serial_out':
      logSerialOut(msg.data);
      break;

    case 'raw_serial_out_err':
      logSerialOutErr(msg.error);
      break;

    case 'message':
      logSystem(`[Board Message] ${msg.data}`);
      break;

    case 'telemetry_other':
      logSystem(`[Other Telemetry] ${msg.cmd}: ${msg.values.join(',')}`);
      break;

    case 'cockpit_info': {
      const elCockpit = document.getElementById('ui-cockpit-deployed');
      if (elCockpit) elCockpit.innerText = msg.deployed;
      break;
    }

    case 'autotest_status': {
      const btnAutoTest = document.getElementById('btn-auto-test');
      const modal = document.getElementById('autotest-modal');
      const stepEl = document.getElementById('autotest-modal-step');
      const statusEl = document.getElementById('autotest-status-text');
      const copyBtn = document.getElementById('btn-autotest-modal-copy');
      const abortBtn = document.getElementById('btn-autotest-modal-abort');
      const closeBtn = document.getElementById('btn-autotest-modal-close');
      const panelCopyBtn = document.getElementById('btn-copy-test-data');

      currentVisualStep = msg.step;

      // Track final leg metrics when step changes
      if (msg.step !== lastProcessedStep) {
        if (lastProcessedStep > 0) {
          legResults[lastProcessedStep] = {
            maxDrift: currentLegMaxDrift,
            maxMismatch: currentLegMaxMismatch
          };
          renderStageResultsTable();
        }
        currentLegMaxDrift = 0;
        currentLegMaxMismatch = 0;
        lastProcessedStep = msg.step;
      }

      if (msg.step > 0) {
        autoTestActive = true;
        
        let stageName = "Slow";
        let repeatNum = 1;
        if (msg.step <= 6) {
          stageName = "Slow";
          repeatNum = Math.ceil(msg.step / 2);
        } else if (msg.step <= 12) {
          stageName = "Medium";
          repeatNum = Math.ceil((msg.step - 6) / 2);
        } else {
          stageName = "Fast";
          repeatNum = Math.ceil((msg.step - 12) / 2);
        }
        
        const legType = (msg.step % 2 === 1) ? 'FWD' : 'BWD';
        
        if (btnAutoTest) {
          btnAutoTest.textContent = `Abort [${stageName} ${legType} ${repeatNum}/3]`;
          btnAutoTest.style.background = 'linear-gradient(135deg, #ff0055, #ff0000)';
          btnAutoTest.style.boxShadow = '0 0 10px rgba(255, 0, 85, 0.4)';
        }

        if (!autotestInitialized) {
          const rawM1 = parseInt(document.getElementById('telemetry-total-m1').textContent || 0);
          const rawM2 = parseInt(document.getElementById('telemetry-total-m2').textContent || 0);
          const rawM3 = parseInt(document.getElementById('telemetry-total-m3').textContent || 0);
          const rawM4 = parseInt(document.getElementById('telemetry-total-m4').textContent || 0);
          autotestStartTicks = [rawM1, rawM2, rawM3, rawM4];
          autotestInitialized = true;
          
          // Reset integration trackers and logs
          visualX = 0.0;
          visualY = 0.0;
          visualYaw = 0.0;
          prevLeftTicks = 0.0;
          prevRightTicks = 0.0;
          clientTestLogs = [];
          
          // Reset tracking maxes and results table!
          legResults = {};
          currentLegMaxDrift = 0;
          currentLegMaxMismatch = 0;
          lastProcessedStep = msg.step;
          renderStageResultsTable();
        }

        if (copyBtn) copyBtn.style.display = 'none';
        if (closeBtn) closeBtn.style.display = 'none';
        if (abortBtn) abortBtn.style.display = 'inline-block';
        if (panelCopyBtn) panelCopyBtn.style.display = 'none';
        if (modal) modal.style.display = 'flex';
        if (stepEl) stepEl.textContent = `${stageName} [${legType} ${repeatNum}/3] (Leg ${msg.step}/18)`;
        if (statusEl) statusEl.textContent = `Status: ${msg.msg || 'Running test...'}`;
      } else {
        autoTestActive = false;
        autotestInitialized = false;

        if (btnAutoTest) {
          btnAutoTest.textContent = 'Auto Test (3ft)';
          btnAutoTest.style.background = 'linear-gradient(135deg, #00f0ff, #0072ff)';
          btnAutoTest.style.boxShadow = '0 0 10px rgba(0, 240, 255, 0.3)';
        }

        if (statusEl) statusEl.textContent = `Status: ${msg.msg || 'Test finished.'}`;
        if (copyBtn) copyBtn.style.display = 'inline-block';
        if (closeBtn) closeBtn.style.display = 'inline-block';
        if (abortBtn) abortBtn.style.display = 'none';
        if (panelCopyBtn && clientTestLogs.length > 0) {
          panelCopyBtn.style.display = 'inline-block';
        }

        if (modal && msg.msg && msg.msg.includes('ABORTED')) {
          modal.style.display = 'none';
        }
      }
      if (msg.msg) {
        logSystem(`[Auto Test] ${msg.msg}`);
      }
      break;
    }

    case 'firmware_info': {
      logSystem(`[Firmware] Name: ${msg.name} | Ver: ${msg.version} | Protocol: ${msg.protocol} | Source: ${msg.commit} | Build: ${msg.build} | Target: ${msg.target}`);
      const elFirmwareVer = document.getElementById('ui-firmware-version');
      const elFirmwareBuild = document.getElementById('ui-firmware-build');
      if (elFirmwareVer) elFirmwareVer.innerText = `${msg.version} (${msg.commit})`;
      if (elFirmwareBuild) elFirmwareBuild.innerText = msg.build;
      break;
    }

    case 'loop_timing': {
      // Rate-limit console logs for timing stats to every 5 seconds to avoid flooding the log viewer,
      // or print immediately if a new deadline is missed.
      if (!window._lastTimingLogTime) window._lastTimingLogTime = 0;
      if (!window._lastMissedDeadlines) window._lastMissedDeadlines = 0;
      const nowMs = Date.now();
      const missedDiff = msg.missedDeadlines - window._lastMissedDeadlines;
      if (nowMs - window._lastTimingLogTime > 5000 || missedDiff > 0) {
        window._lastTimingLogTime = nowMs;
        window._lastMissedDeadlines = msg.missedDeadlines;
        const msgStr = `[Loop Stats] Iterations: ${msg.totalIterations} | Missed: ${msg.missedDeadlines} | Last: ${msg.lastDurationUs}us | Min: ${msg.minDurationUs}us | Avg: ${msg.avgDurationUs}us | Max: ${msg.maxDurationUs}us`;
        if (missedDiff > 0) {
          logSerialOutErr(`⚠️ ${msgStr}`);
        } else {
          logSystem(msgStr);
        }
      }
      break;
    }

    case 'fault_report':
      if (!window._lastFaultFlags) window._lastFaultFlags = 0;
      if (msg.faultFlags !== window._lastFaultFlags) {
        window._lastFaultFlags = msg.faultFlags;
        if (msg.faultFlags !== 0) {
          logSerialOutErr(`🚨 [Safety Fault Triggered] Fault Code: 0x${msg.faultFlags.toString(16).toUpperCase()}`);
        } else {
          logSystem(`🔓 [Safety Faults Cleared] System returning to nominal.`);
        }
      }
      break;

    case 'calibration_status': {
      const calPanel = document.getElementById('pi-cal-panel');
      const calState = document.getElementById('pi-cal-state');
      const calPwmVal = document.getElementById('pi-cal-pwm-val');
      const calProgress = document.getElementById('pi-cal-progress');
      
      const protoDisplay = document.getElementById('pi-protocol-display');
      const sessDisplay = document.getElementById('pi-session-display');
      const lockBadge = document.getElementById('pi-lock-badge');
      const modeBadge = document.getElementById('pi-mode-badge');
      
      const motorLbl = document.getElementById('pi-cal-active-motor-lbl');
      const dirLbl = document.getElementById('pi-cal-direction-lbl');
      const pwmDetailLbl = document.getElementById('pi-cal-pwm-lbl');
      const deltaLbl = document.getElementById('pi-cal-delta-lbl');
      const movementLbl = document.getElementById('pi-cal-movement-lbl');

      const cState = msg.cal_state;
      const cMotor = msg.cal_motor;
      const cMotorNum = msg.cal_motor_num || (cMotor + 1);
      const cPwm = msg.cal_pwm;
      const direction = msg.direction;
      const encoderDelta = msg.encoderDelta;
      const movementDetected = msg.movementDetected;
      const motorLockStatus = msg.motorLockStatus;
      const isSimulation = msg.isSimulation;
      const failureReason = msg.failureReason;
      const fwd = msg.cal_fwd;
      const rev = msg.cal_rev;
      
      // Update protocol and session fields if present
      if (protoDisplay && msg.protoMajor !== undefined) {
        protoDisplay.innerText = `Protocol: v${msg.protoMajor}.${msg.protoMinor}`;
      }
      if (sessDisplay && msg.sessionId !== undefined) {
        sessDisplay.innerText = `Session: ${msg.sessionId}`;
      }
      
      // Update safety lock badge
      if (lockBadge) {
        if (motorLockStatus) {
          lockBadge.innerText = '🔒 Safety Lock: Active';
          lockBadge.style.background = 'rgba(16, 185, 129, 0.2)';
          lockBadge.style.color = '#10b981';
          lockBadge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
        } else {
          lockBadge.innerText = '⚠️ Safety Lock: Missing';
          lockBadge.style.background = 'rgba(239, 68, 68, 0.2)';
          lockBadge.style.color = '#ef4444';
          lockBadge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        }
      }
      
      // Update mode badge
      if (modeBadge && isSimulation !== undefined) {
        modeBadge.innerText = isSimulation ? 'Mode: Simulation Only' : 'Mode: Real-Drive';
      }
      
      // Update thresholds table
      const motors = ["m1", "m2", "m3", "m4"];
      for (let i = 0; i < 4; i++) {
        const fwdVal = fwd[i];
        const revVal = rev[i];
        document.getElementById(`pi-val-${motors[i]}-fwd`).innerText = fwdVal > 0 ? `${fwdVal} PWM` : "--";
        document.getElementById(`pi-val-${motors[i]}-rev`).innerText = revVal > 0 ? `${revVal} PWM` : "--";
      }
      
      if (cState > 0) {
        if (calPanel) calPanel.style.display = 'block';
        
        let stateText = "Running...";
        if (cState === 1) stateText = "PRECHECK: Checking safety locks...";
        else if (cState === 2) stateText = `MEASURING: Motor ${cMotorNum} Forward`;
        else if (cState === 3) stateText = `MEASURING: Motor ${cMotorNum} Reverse`;
        else if (cState === 4) stateText = `COOLDOWN: Cooldown pause (Motor ${cMotorNum})`;
        else if (cState === 5) stateText = `COMPLETE: Simulated calibration done!`;
        else if (cState === 6) stateText = `ABORTED: Simulation cancelled.`;
        else if (cState === 7) stateText = `FAILED: ${failureReason || 'Unknown error'}`;
        
        if (calState) calState.innerText = stateText;
        if (calPwmVal) calPwmVal.innerText = cPwm + " PWM";
        if (calProgress) calProgress.value = cPwm;
        
        // Update details
        if (motorLbl) motorLbl.innerText = `Motor ${cMotorNum}`;
        if (dirLbl) dirLbl.innerText = direction === 0 ? "FWD (Forward)" : "REV (Reverse)";
        if (pwmDetailLbl) pwmDetailLbl.innerText = `${cPwm} PWM`;
        if (deltaLbl) deltaLbl.innerText = `${encoderDelta || 0} ticks`;
        if (movementLbl) {
          movementLbl.innerText = movementDetected ? "YES (>= 8 ticks)" : "NO";
          movementLbl.style.color = movementDetected ? "#10b981" : "#f59e0b";
        }
      } else {
        if (calPanel) calPanel.style.display = 'none';
      }
      break;
    }

    case 'maintenance_status': {
      const active = msg.active;
      const activeMotor = msg.activeMotor;
      const activeMotorNum = msg.activeMotorNum;
      const direction = msg.direction;
      const testPwm = msg.testPwm;
      const actualPwm = msg.actualPwm;
      const deadmanActive = msg.deadmanActive;
      const remainingTimeout = msg.remainingTimeout;
      
      const badge = document.getElementById('maint-status-badge');
      if (badge) {
        if (active) {
          badge.innerText = `Active (Motor ${activeMotorNum})`;
          badge.style.background = 'rgba(59, 130, 246, 0.2)';
          badge.style.color = '#3b82f6';
          badge.style.border = '1px solid rgba(59, 130, 246, 0.4)';
        } else {
          badge.innerText = 'Locked';
          badge.style.background = 'rgba(239, 68, 68, 0.2)';
          badge.style.color = '#fca5a5';
          badge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        }
      }
      
      document.getElementById('tele-maint-session').innerText = msg.sessionId || '--';
      document.getElementById('tele-maint-active').innerText = active ? 'ACTIVE' : 'Inactive';
      document.getElementById('tele-maint-active').style.color = active ? '#10b981' : '#ef4444';
      document.getElementById('tele-maint-deadman').innerText = deadmanActive ? 'Active' : 'Offline';
      document.getElementById('tele-maint-deadman').style.color = deadmanActive ? '#10b981' : '#f59e0b';
      document.getElementById('tele-maint-timeout').innerText = active ? `${(remainingTimeout / 1000).toFixed(1)}s` : '--';
      document.getElementById('tele-maint-target').innerText = testPwm || '0';
      document.getElementById('tele-maint-actual').innerText = actualPwm || '0';
      
      let delta = 0;
      let total = 0;
      if (active && activeMotor >= 0 && activeMotor < 4) {
        const totalElems = [totalValM1, totalValM2, totalValM3, totalValM4];
        const speedElems = [realValM1, realValM2, realValM3, realValM4];
        total = totalElems[activeMotor] ? totalElems[activeMotor].textContent : '0';
        delta = speedElems[activeMotor] ? speedElems[activeMotor].textContent : '0';
      }
      document.getElementById('tele-maint-enc-delta').innerText = delta;
      document.getElementById('tele-maint-enc-total').innerText = total;
      
      const slider = document.getElementById('maint-pwm-slider');
      const stepFwd = document.getElementById('btn-maint-fwd-step');
      const stepRev = document.getElementById('btn-maint-rev-step');
      
      if (active) {
        slider.disabled = false;
        stepFwd.disabled = false;
        stepRev.disabled = false;
        stepFwd.style.background = 'rgba(59, 130, 246, 0.2)';
        stepFwd.style.color = '#93c5fd';
        stepFwd.style.cursor = 'pointer';
        stepRev.style.background = 'rgba(239, 68, 68, 0.2)';
        stepRev.style.color = '#fca5a5';
        stepRev.style.cursor = 'pointer';
      } else {
        slider.disabled = true;
        slider.value = 0;
        document.getElementById('maint-pwm-display').innerText = '0';
        stepFwd.disabled = true;
        stepRev.disabled = true;
        stepFwd.style.background = '#374151';
        stepFwd.style.color = '#9ca3af';
        stepFwd.style.cursor = 'not-allowed';
        stepRev.style.background = '#374151';
        stepRev.style.color = '#9ca3af';
        stepRev.style.cursor = 'not-allowed';
      }
      break;
    }

    case 'normal_drive_status': {
      const armed = msg.armed;
      const mode = msg.mode;
      const source = msg.source;
      const cmdAge = msg.cmdAge;
      const reqLinear = msg.reqLinear;
      const reqAngular = msg.reqAngular;
      const limLinear = msg.limLinear;
      const limAngular = msg.limAngular;
      const lockStatus = msg.lockStatus;
      
      driveArmed = armed;
      
      const badge = document.getElementById('normal-drive-badge');
      if (badge) {
        if (armed) {
          badge.innerText = 'Armed';
          badge.style.background = 'rgba(16, 185, 129, 0.2)';
          badge.style.color = '#10b981';
          badge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
        } else {
          badge.innerText = 'Disarmed';
          badge.style.background = 'rgba(239, 68, 68, 0.2)';
          badge.style.color = '#fca5a5';
          badge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        }
      }
      
      const elState = document.getElementById('tele-drive-state');
      if (elState) {
        elState.innerText = armed ? 'ARMED' : 'Disarmed';
        elState.style.color = armed ? '#10b981' : '#fca5a5';
      }
      
      const modes = ['LOCKED', 'MAINTENANCE', 'CALIBRATION', 'NORMAL_DRIVE', 'EMERGENCY_STOP', 'FAULTED'];
      const elMode = document.getElementById('tele-drive-mode');
      if (elMode) {
        elMode.innerText = modes[mode] || `UNKNOWN (${mode})`;
        elMode.style.color = (mode === 3) ? '#10b981' : ((mode === 4 || mode === 5) ? '#ef4444' : '#f59e0b');
      }
      
      const elPhys = document.getElementById('tele-drive-phys-lock');
      if (elPhys) {
        if (lockStatus) {
          elPhys.innerText = 'CLAMP ACTIVE';
          elPhys.style.color = '#ef4444';
        } else {
          elPhys.innerText = 'DISABLED (LIVE)';
          elPhys.style.color = '#10b981';
        }
      }
      
      const sources = ['NONE', 'WEB_JOYSTICK', 'USB_SERIAL', 'ROS', 'POSITION', 'CALIBRATION'];
      const elSource = document.getElementById('tele-drive-source');
      if (elSource) {
        elSource.innerText = sources[source] || `UNKNOWN (${source})`;
      }
      
      const elAge = document.getElementById('tele-drive-age');
      if (elAge) {
        elAge.innerText = (cmdAge === 999999) ? 'N/A' : `${cmdAge} ms`;
        if (cmdAge !== 999999 && cmdAge > 500) {
          elAge.style.color = '#ef4444';
        } else {
          elAge.style.color = '';
        }
      }
      
      const elReqLin = document.getElementById('tele-drive-req-lin');
      if (elReqLin) elReqLin.innerText = `${reqLinear.toFixed(2)} m/s`;
      
      const elReqAng = document.getElementById('tele-drive-req-ang');
      if (elReqAng) elReqAng.innerText = `${reqAngular.toFixed(2)} rad/s`;
      
      const elLimLin = document.getElementById('tele-drive-lim-lin');
      if (elLimLin) elLimLin.innerText = `${limLinear.toFixed(2)} m/s`;
      
      const elLimAng = document.getElementById('tele-drive-lim-ang');
      if (elLimAng) elLimAng.innerText = `${limAngular.toFixed(2)} rad/s`;

      // Update Gamepad Live Input HUD (Arm and ESTOP)
      const gpArm = document.getElementById('gp-live-arm');
      if (gpArm) {
        gpArm.innerText = armed ? 'ARMED' : 'DISARMED';
        gpArm.style.color = armed ? '#10b981' : '#6b7280';
      }
      const gpEstop = document.getElementById('gp-live-estop');
      if (gpEstop) {
        if (mode === 4) {
          gpEstop.innerText = 'ESTOP ACTIVE';
          gpEstop.style.color = '#ef4444';
        } else if (mode === 5) {
          gpEstop.innerText = 'FAULTED';
          gpEstop.style.color = '#ef4444';
        } else {
          gpEstop.innerText = 'NOMINAL';
          gpEstop.style.color = '#10b981';
        }
      }
      
      break;
    }

    case 'rover_params_sync':
      currentWheelDiameter = msg.diameter;
      currentTrackWidth = msg.separation;
      
      const elCurDia = document.getElementById('cal-dist-current-diameter');
      if (elCurDia) elCurDia.innerText = `${(msg.diameter * 1000).toFixed(1)} mm`;
      
      const elCurWidth = document.getElementById('cal-rot-current-width');
      if (elCurWidth) elCurWidth.innerText = `${(msg.separation * 1000).toFixed(1)} mm`;
      break;

    case 'rover_trims_sync': {
      const inputL = document.getElementById('input-left-trim');
      const inputR = document.getElementById('input-right-trim');
      const labelActive = document.getElementById('label-active-trims');
      
      if (inputL) inputL.value = msg.leftTrim.toFixed(3);
      if (inputR) inputR.value = msg.rightTrim.toFixed(3);
      if (labelActive) {
        labelActive.textContent = `L: ${msg.leftTrim.toFixed(3)} | R: ${msg.rightTrim.toFixed(3)}`;
        labelActive.style.color = '#10b981';
      }

      const activeFwdText = document.getElementById('active-fwd-trims');
      if (activeFwdText) {
        activeFwdText.textContent = `${msg.leftTrim.toFixed(4)} / ${msg.rightTrim.toFixed(4)}`;
      }
      break;
    }

    case 'calibration_db':
      updateCalibrationDbUI(msg.db);
      break;

    case 'rover_trims_rev_sync': {
      const activeRevText = document.getElementById('active-rev-trims');
      if (activeRevText) {
        activeRevText.textContent = `${msg.leftTrimRev.toFixed(4)} / ${msg.rightTrimRev.toFixed(4)}`;
      }
      logSystem(`[Config Sync] Synced active Reverse Trims: ${msg.leftTrimRev.toFixed(4)} / ${msg.rightTrimRev.toFixed(4)}`);
      break;
    }

    case 'lidar_test_status': {
      lidarTestState = msg.state;
      const badge = document.getElementById('lidar-test-state-badge');
      if (badge) {
        // Build progress label
        let progressLabel = msg.state;
        if (msg.speedTier && msg.totalPass) {
          progressLabel = `${msg.speedTier} Pass ${msg.totalPass}/${msg.totalPasses}`;
        }
        badge.textContent = progressLabel;
        badge.style.background = '';
        badge.style.color = '';
        badge.style.borderColor = '';
        
        if (msg.state === 'IDLE') {
          stopTestScanPolling();
          badge.style.background = 'rgba(107, 114, 128, 0.2)';
          badge.style.color = '#9ca3af';
          badge.style.border = '1px solid rgba(107, 114, 128, 0.4)';
          badge.textContent = 'IDLE';
          document.getElementById('btn-start-lidar-test').style.display = 'block';
          document.getElementById('btn-stop-lidar-test').style.display = 'none';
          
          if (lidarOdomPath.length > 0 || lidarPosePath.length > 0) {
            saveCurrentPathToHistory(lastTelemetrySpeedTier);
            lidarOdomPath = [];
            lidarPosePath = [];
            drawLidarTestCanvas();
          }
        } else if (msg.state === 'ZEROING' || msg.state === 'RETURNING_HOME_WAIT') {
          badge.style.background = 'rgba(245, 158, 11, 0.2)';
          badge.style.color = '#f59e0b';
          badge.style.border = '1px solid rgba(245, 158, 11, 0.4)';
          document.getElementById('btn-start-lidar-test').style.display = 'none';
          document.getElementById('btn-stop-lidar-test').style.display = 'block';
        } else if (msg.state === 'FORWARD_RUNNING' || msg.state === 'RETURNING_HOME') {
          badge.style.background = 'rgba(6, 182, 212, 0.2)';
          badge.style.color = '#06b6d4';
          badge.style.border = '1px solid rgba(6, 182, 212, 0.4)';
          document.getElementById('btn-start-lidar-test').style.display = 'none';
          document.getElementById('btn-stop-lidar-test').style.display = 'block';
        } else if (msg.state === 'COMPLETE') {
          stopTestScanPolling();
          badge.style.background = 'rgba(16, 185, 129, 0.2)';
          badge.style.color = '#10b981';
          badge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
          badge.textContent = 'COMPLETE ✓';
          document.getElementById('btn-start-lidar-test').style.display = 'block';
          document.getElementById('btn-stop-lidar-test').style.display = 'none';
          
          if (lidarOdomPath.length > 0 || lidarPosePath.length > 0) {
            saveCurrentPathToHistory(lastTelemetrySpeedTier);
            lidarOdomPath = [];
            lidarPosePath = [];
            drawLidarTestCanvas();
          }
        }
      }
      logSystem(`[Auto Calib] ${msg.msg || msg.state}`);
      break;
    }
 
    case 'lidar_test_telemetry': {
      lidarTestState = msg.state;
      lastTelemetrySpeedTier = msg.speedTier || 'SLOW';
      if (msg.state === 'ZEROING' || msg.state === 'FORWARD_READY') {
        if (lidarOdomPath.length > 0 || lidarPosePath.length > 0) {
          saveCurrentPathToHistory(lastTelemetrySpeedTier);
        }
        lidarOdomPath = [];
        lidarPosePath = [];
      }

      
      const xSpan = document.getElementById('stat-lidar-x');
      const ySpan = document.getElementById('stat-lidar-y');
      const yawSpan = document.getElementById('stat-lidar-yaw');
      const confSpan = document.getElementById('stat-lidar-conf');
      
      if (xSpan) xSpan.textContent = msg.lidarPose.x.toFixed(3) + 'm';
      if (ySpan) ySpan.textContent = msg.lidarPose.y.toFixed(3) + 'm';
      if (yawSpan) yawSpan.textContent = (msg.lidarPose.yaw * 180 / Math.PI).toFixed(2) + '°';
      if (confSpan) {
        confSpan.textContent = (msg.metrics.confidence * 100).toFixed(1) + '%';
        const confDiv = confSpan.parentNode;
        if (msg.metrics.rejectionReason) {
          confDiv.style.borderColor = '#ef4444';
          confDiv.title = msg.metrics.rejectionReason;
        } else {
          confDiv.style.borderColor = '';
          confDiv.title = '';
        }
      }
      
      if (msg.state === 'FORWARD_RUNNING' || msg.state === 'REVERSE_RUNNING' || msg.state === 'RETURNING_HOME') {
        lidarOdomPath.push({ x: msg.odomPose.x, y: msg.odomPose.y });
        lidarPosePath.push({ x: msg.lidarPose.x, y: msg.lidarPose.y, yaw: msg.lidarPose.yaw });
        drawLidarTestCanvas();
      }

      // Update Live Motor Power UI
      const lblLeft = document.getElementById('lbl-power-left');
      const lblRight = document.getElementById('lbl-power-right');
      const barLeft = document.getElementById('bar-power-left');
      const barRight = document.getElementById('bar-power-right');
      const powerTier = document.getElementById('power-active-tier');
      
      if (powerTier) powerTier.textContent = `Tier: ${msg.speedTier || 'SLOW'}`;
      
      const isMoving = msg.state === 'FORWARD_RUNNING' || msg.state === 'REVERSE_RUNNING' || msg.state === 'RETURNING_HOME';
      const dirText = !isMoving ? 'IDLE' : (msg.direction === 'FORWARD' ? 'FWD' : 'REV');
      
      if (lblLeft && barLeft) {
        const leftPower = isMoving ? (msg.leftPowerPct || 0) : 0;
        lblLeft.textContent = `${leftPower}% (${dirText})`;
        barLeft.style.width = `${leftPower}%`;
        lblLeft.style.color = leftPower > 70 ? '#ff0055' : (leftPower > 40 ? '#f59e0b' : '#10b981');
      }
      
      if (lblRight && barRight) {
        const rightPower = isMoving ? (msg.rightPowerPct || 0) : 0;
        lblRight.textContent = `${rightPower}% (${dirText})`;
        barRight.style.width = `${rightPower}%`;
        lblRight.style.color = rightPower > 70 ? '#ff0055' : (rightPower > 40 ? '#f59e0b' : '#10b981');
      }

      // Update Live Control Effort UI
      const lblEffort = document.getElementById('lbl-control-effort');
      const barEffort = document.getElementById('bar-control-effort');
      if (lblEffort && barEffort) {
        const effort = isMoving ? (msg.appliedCorrection || 0.0) : 0.0;
        const maxEffort = 0.35; // maxAngularCorr
        
        let effortPct = (effort / maxEffort) * 50; // map to -50% to 50%
        effortPct = Math.max(-50, Math.min(50, effortPct));
        
        if (effortPct >= 0) {
          barEffort.style.left = '50%';
          barEffort.style.width = `${effortPct}%`;
          barEffort.style.background = 'linear-gradient(90deg, #a855f7, #00f0ff)';
        } else {
          barEffort.style.left = `${50 + effortPct}%`;
          barEffort.style.width = `${Math.abs(effortPct)}%`;
          barEffort.style.background = 'linear-gradient(90deg, #ff0055, #a855f7)';
        }
        
        const effortDir = effort > 0.005 ? 'STEER LEFT' : (effort < -0.005 ? 'STEER RIGHT' : 'CENTER');
        lblEffort.textContent = isMoving ? `${effort.toFixed(3)} rad/s (${effortDir})` : '0.000 rad/s (CENTER)';
        lblEffort.style.color = isMoving ? (Math.abs(effort) > 0.2 ? '#ff0055' : (Math.abs(effort) > 0.08 ? '#f59e0b' : '#00f0ff')) : '#00f0ff';
      }
      break;
    }

    case 'lidar_test_results': {
      const passLabel = document.getElementById('pass-count-label');
      const proposedFwd = document.getElementById('proposed-fwd-trims');
      const proposedRev = document.getElementById('proposed-rev-trims');
      
      if (passLabel) passLabel.textContent = `Completed Passes: ${msg.acceptedPasses}/1`;
      if (proposedFwd) proposedFwd.textContent = `${msg.proposedFwdTrim.left.toFixed(4)} / ${msg.proposedFwdTrim.right.toFixed(4)}`;
      if (proposedRev) proposedRev.textContent = `${msg.proposedRevTrim.left.toFixed(4)} / ${msg.proposedRevTrim.right.toFixed(4)}`;
      
      const applyBtn = document.getElementById('btn-apply-proposed');
      if (applyBtn) {
        if (msg.acceptedPasses >= 1) {
          applyBtn.disabled = false;
          applyBtn.style.opacity = '1';
          applyBtn.style.cursor = 'pointer';
        } else {
          applyBtn.disabled = true;
          applyBtn.style.opacity = '0.6';
          applyBtn.style.cursor = 'not-allowed';
        }
      }
      break;
    }

    case 'test_abort':
      if (activeTest) {
        abortCalibrationTest(msg.reason);
      }
      break;
  }
}

// Wheel Animation Controller
function updateWheelAnimation(cardElement, speed) {
  const wheel = cardElement.querySelector('.wheel-visual');
  const badge = cardElement.querySelector('.dir-badge');
  
  if (Math.abs(speed) < 0.5) {
    // Stopped
    wheel.style.setProperty('--spin-duration', '0s');
    badge.textContent = 'STOP';
    badge.className = 'dir-badge status-indicator off';
  } else {
    // Map speed to spin duration (e.g. speed of 1000 = 0.1s spin, speed of 50 = 2s spin)
    const absSpeed = Math.abs(speed);
    const duration = Math.max(0.1, Math.min(5, 100 / absSpeed));
    
    wheel.style.setProperty('--spin-duration', `${duration}s`);
    wheel.style.setProperty('--spin-direction', speed > 0 ? 'normal' : 'reverse');
    
    if (speed > 0) {
      badge.textContent = 'FWD';
      badge.className = 'dir-badge forward';
    } else {
      badge.textContent = 'REV';
      badge.className = 'dir-badge reverse';
    }
  }
}

// Apply individual motor sliders and sync readouts
function updateIndividualSliderValues(m1, m2, m3, m4) {
  sliderM1.value = m1;
  sliderM2.value = m2;
  sliderM3.value = m3;
  sliderM4.value = m4;
  
  readoutM1.textContent = m1;
  readoutM2.textContent = m2;
  readoutM3.textContent = m3;
  readoutM4.textContent = m4;
}

// Send speed parameters
function sendMotorSpeeds(m1, m2, m3, m4) {
  sendServerMessage({
    type: 'set_speed',
    speeds: [parseInt(m1), parseInt(m2), parseInt(m3), parseInt(m4)]
  });
}

// Preset and Sync Sliders Input Event
syncSpeedSlider.addEventListener('input', (e) => {
  const val = e.target.value;
  syncSpeedReadout.textContent = val;
  updateIndividualSliderValues(val, val, val, val);
  sendMotorSpeeds(val, val, val, val);
});

// Preset Buttons Event Listeners
document.querySelectorAll('.preset-btn').forEach((btn) => {
  btn.addEventListener('click', (e) => {
    const val = parseInt(e.target.dataset.val);
    syncSpeedSlider.value = val;
    syncSpeedReadout.textContent = val;
    updateIndividualSliderValues(val, val, val, val);
    sendMotorSpeeds(val, val, val, val);
  });
});

// Individual slider changes
[sliderM1, sliderM2, sliderM3, sliderM4].forEach((slider, idx) => {
  slider.addEventListener('input', () => {
    // Reset sync slider to 0 to show it is un-synchronized
    syncSpeedSlider.value = 0;
    syncSpeedReadout.textContent = 'Mix';
    
    // Update individual readouts
    readoutM1.textContent = sliderM1.value;
    readoutM2.textContent = sliderM2.value;
    readoutM3.textContent = sliderM3.value;
    readoutM4.textContent = sliderM4.value;
    
    sendMotorSpeeds(sliderM1.value, sliderM2.value, sliderM3.value, sliderM4.value);
  });
});

// Emergency Stop Function
function triggerEstop() {
  syncSpeedSlider.value = 0;
  syncSpeedReadout.textContent = '0';
  updateIndividualSliderValues(0, 0, 0, 0);
  sendMotorSpeeds(0, 0, 0, 0);
  // Also send raw emergency pwm stop just in case
  sendServerMessage({ type: 'set_pwm', pwms: [0, 0, 0, 0] });
  logSystem('EMERGENCY STOP COMMAND SENT');
}

btnEstop.addEventListener('click', triggerEstop);
ctrlStopCenter.addEventListener('click', triggerEstop);

if (btnMotorProof) {
  btnMotorProof.addEventListener('click', () => {
    sendServerMessage({ type: 'run_motor_proof' });
    logSystem('Requested motor power proof sequence from server.');
  });
}

// Direction Pad Movements
let currentSpeedSetting = 500; // Default active speed to use when clicking DPad

function driveRover(direction) {
  if (!driveArmed) {
    logSystem("⚠️ Cannot drive: Coordinated Normal Drive is DISARMED. Press Arm first.");
    return;
  }

  // Intercept steering command if straight drive mode is locked
  if (straightDriveLocked && ['left', 'right', 'spin_left', 'spin_right'].includes(direction)) {
    logSystem("⚠️ Steering command ignored: Straight Drive Lock is active.");
    return;
  }

  let x = 0.0;
  let y = 0.0;

  switch (direction) {
    case 'forward':
      y = 1.0;
      break;
    case 'reverse':
      y = -1.0;
      break;
    case 'left':
      // Turn left: Left wheels backward, right wheels forward
      m1 = m3 = -activeSpeed;
      m2 = m4 = activeSpeed;
      break;
    case 'right':
      // Turn right: Left wheels forward, right wheels backward
      m1 = m3 = activeSpeed;
      m2 = m4 = -activeSpeed;
      break;
    case 'spin_left':
      // Spin left: Left wheels backward, right wheels forward
      m1 = m3 = -activeSpeed;
      m2 = m4 = activeSpeed;
      break;
    case 'spin_right':
      // Spin right: Left wheels forward, right wheels backward
      m1 = m3 = activeSpeed;
      m2 = m4 = -activeSpeed;
      break;
    case 'stop':
    default:
      x = 0.0;
      y = 0.0;
      break;
  }

  sendServerMessage({ type: 'joystick', x, y });
  logSystem(`Driving direction: ${direction.toUpperCase()} via Coordinated Joystick Path (x: ${x.toFixed(2)}, y: ${y.toFixed(2)})`);
}

// DPad Action Click Listeners
ctrlForward.addEventListener('click', () => driveRover('forward'));
ctrlReverse.addEventListener('click', () => driveRover('reverse'));
ctrlLeft.addEventListener('click', () => driveRover('left'));
ctrlRight.addEventListener('click', () => driveRover('right'));
ctrlSpinLeft.addEventListener('click', () => driveRover('spin_left'));
ctrlSpinRight.addEventListener('click', () => driveRover('spin_right'));

// Keyboard WASD / Arrows controls
document.addEventListener('keydown', (e) => {
  // Prevent default scroll behaviors for arrow keys/space inside dashboard
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key) && document.activeElement.tagName !== 'INPUT') {
    e.preventDefault();
  }

  if (document.activeElement.tagName === 'INPUT') return; // Skip if user typing

  // Ignore steering keys when straight drive is locked
  if (straightDriveLocked && ['a', 'arrowleft', 'd', 'arrowright', 'q', 'e'].includes(e.key.toLowerCase())) {
    return;
  }

  switch (e.key.toLowerCase()) {
    case 'w':
    case 'arrowup':
      ctrlForward.classList.add('active');
      driveRover('forward');
      break;
    case 's':
    case 'arrowdown':
      ctrlReverse.classList.add('active');
      driveRover('reverse');
      break;
    case 'a':
    case 'arrowleft':
      ctrlLeft.classList.add('active');
      driveRover('left');
      break;
    case 'd':
    case 'arrowright':
      ctrlRight.classList.add('active');
      driveRover('right');
      break;
    case 'q':
      ctrlSpinLeft.classList.add('active');
      driveRover('spin_left');
      break;
    case 'e':
      ctrlSpinRight.classList.add('active');
      driveRover('spin_right');
      break;
    case ' ':
    case 'escape':
      ctrlStopCenter.classList.add('active');
      triggerEstop();
      break;
  }
});

document.addEventListener('keyup', (e) => {
  const key = e.key.toLowerCase();
  
  if (['w', 'arrowup'].includes(key)) ctrlForward.classList.remove('active');
  if (['s', 'arrowdown'].includes(key)) ctrlReverse.classList.remove('active');
  if (['a', 'arrowleft'].includes(key)) ctrlLeft.classList.remove('active');
  if (['d', 'arrowright'].includes(key)) ctrlRight.classList.remove('active');
  if (key === 'q') ctrlSpinLeft.classList.remove('active');
  if (key === 'e') ctrlSpinRight.classList.remove('active');
  if ([' ', 'escape'].includes(key)) ctrlStopCenter.classList.remove('active');
});

// Telemetry toggles checkboxes
[streamTotal, streamRealtime, streamSpeed].forEach((checkbox) => {
  checkbox.addEventListener('change', sendUploadConfig);
});

// Config Form Submit Handler
configForm.addEventListener('submit', (e) => {
  e.preventDefault();
  
  const mType = motorType.value;
  const db = deadband.value;
  const pl = phaseLines.value;
  const rr = reductionRatio.value;
  const wd = wheelDiameter.value;
  
  const p = pidP.value;
  const i = pidI.value;
  const d = pidD.value;

  logSystem('Applying configurations to Maker ESP32 Pro board...');
  
  sendServerMessage({ type: 'config_motor_type', val: mType });
  setTimeout(() => sendServerMessage({ type: 'config_deadband', val: db }), 100);
  setTimeout(() => sendServerMessage({ type: 'config_phase_lines', val: pl }), 200);
  setTimeout(() => sendServerMessage({ type: 'config_reduction_ratio', val: rr }), 300);
  setTimeout(() => sendServerMessage({ type: 'config_wheel_diameter', val: wd }), 400);
  setTimeout(() => sendServerMessage({ type: 'config_pid', p, i, d }), 500);
  
  logSystem('Settings sent. Waiting for board confirmation...');
});

// Read and Reset Config Buttons
btnReadFlash.addEventListener('click', () => {
  logSystem('Querying board flash variables ($read_flash#)...');
  sendServerMessage({ type: 'read_flash' });
});

btnResetFlash.addEventListener('click', () => {
  if (confirm('Are you sure you want to restore default factory configurations? The board will restart.')) {
    logSystem('Restoring factory defaults ($flash_reset#)...');
    sendServerMessage({ type: 'flash_reset' });
  }
});

// Port change submission
btnChangePort.addEventListener('click', () => {
  const newPort = comPortInput.value.trim();
  if (newPort) {
    logSystem(`Requesting server reconnect serial port to: ${newPort}`);
    sendServerMessage({ type: 'change_port', port: newPort });
  }
});

// Phase 4 Arm and Disarm drive control actions
function armNormalDrive() {
  logSystem('Sending arm normal drive request...');
  fetch('/api/drive/arm', { method: 'POST' })
    .then(res => res.json())
    .then(data => {
      logSystem(data.message || 'Arm request processed.');
    })
    .catch(err => {
      logSystem(`⚠️ Error arming normal drive: ${err.message}`);
    });
}

function disarmNormalDrive() {
  logSystem('Sending disarm normal drive request...');
  fetch('/api/drive/disarm', { method: 'POST' })
    .then(res => res.json())
    .then(data => {
      logSystem(data.message || 'Disarm request processed.');
    })
    .catch(err => {
      logSystem(`⚠️ Error disarming normal drive: ${err.message}`);
    });
}

const btnArmDrive = document.getElementById('btn-arm-drive');
const btnDisarmDrive = document.getElementById('btn-disarm-drive');

if (btnArmDrive) {
  btnArmDrive.addEventListener('click', armNormalDrive);
}

if (btnDisarmDrive) {
  btnDisarmDrive.addEventListener('click', disarmNormalDrive);
}

// Terminal Submit
function sendRawCommandFromInput() {
  const rawCmd = terminalCommandInput.value.trim();
  if (rawCmd) {
    sendServerMessage({ type: 'raw_command', command: rawCmd });
    terminalCommandInput.value = '';
  }
}

btnSendRawCommand.addEventListener('click', sendRawCommandFromInput);
terminalCommandInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') {
    sendRawCommandFromInput();
  }
});

btnClearLogs.addEventListener('click', () => {
  terminalConsole.innerHTML = '';
  logSystem('Logs cleared.');
});

// Start Up Connect
connectWebSocket();

// --- Tab Switching Logic ---
const tabButtons = document.querySelectorAll('.tab-btn');
const tabContents = document.querySelectorAll('.tab-content');

tabButtons.forEach(btn => {
  btn.addEventListener('click', () => {
    const targetTab = btn.dataset.tab;
    
    // Toggle active tab buttons
    tabButtons.forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    
    // Toggle active tab contents
    tabContents.forEach(content => {
      if (content.id === targetTab) {
        content.classList.add('active');
      } else {
        content.classList.remove('active');
      }
    });
    
    // If canvas is shown, resize it
    if (targetTab === 'tab-imu') {
      resizeCanvas();
    }
  });
});

// --- Odometry / Dead Reckoning Kinematics Simulation ---
// Tracks X/Y positions and heading over time based on speed telemetry
const odomXDisplay = document.getElementById('odom-x');
const odomYDisplay = document.getElementById('odom-y');
const odomSpeedDisplay = document.getElementById('odom-speed');
const pathCanvas = document.getElementById('path-canvas');
const ctx = pathCanvas.getContext('2d');

function updateOdometry() {
  if (realOdomActive) return;
  const dt = 0.1; // 100ms loop
  
  // Calculate average left and right velocities
  const vLeft = (m1Speed + m3Speed) / 2;
  const vRight = (m2Speed + m4Speed) / 2;
  
  // Linear speed (mm/s) and angular speed (rad/s)
  const linearSpeed = (vLeft + vRight) / 2;
  const angularSpeed = (vRight - vLeft) / trackWidth;
  
  // Update position if rover is moving
  if (Math.abs(linearSpeed) > 0.1 || Math.abs(angularSpeed) > 0.001) {
    if (!realIMUActive) {
      odomTheta += angularSpeed * dt;
      // Normalize theta between -PI and PI
      odomTheta = Math.atan2(Math.sin(odomTheta), Math.cos(odomTheta));
      imuYaw = odomTheta * 180 / Math.PI;
    }
    
    odomX += linearSpeed * Math.cos(odomTheta) * dt;
    odomY += linearSpeed * Math.sin(odomTheta) * dt;
    
    // Append coordinates (convert mm to meters for mapping scale)
    pathHistory.push({ x: odomX / 1000, y: odomY / 1000 });
    if (pathHistory.length > maxPathPoints) {
      pathHistory.shift();
    }
    
    // If no real IMU, rotate the 3D model according to the simulated heading
    if (!realIMUActive) {
      update3DModelRotation(0, 0, imuYaw);
      document.getElementById('imu-yaw').textContent = `${imuYaw.toFixed(1)}°`;
      document.getElementById('imu-roll').textContent = `0.0°`;
      document.getElementById('imu-pitch').textContent = `0.0°`;
    }
  }
  
  // Update Stats UI
  odomXDisplay.innerHTML = `${(odomX / 1000).toFixed(2)} <small>m</small>`;
  odomYDisplay.innerHTML = `${(odomY / 1000).toFixed(2)} <small>m</small>`;
  odomSpeedDisplay.innerHTML = `${linearSpeed.toFixed(1)} <small>mm/s</small>`;
  
  // Draw path canvas
  drawPath();
}

// 3D Model Rotation Controller
const rover3DModel = document.getElementById('rover-3d-model');
function update3DModelRotation(pitch, roll, yaw) {
  if (rover3DModel) {
    // Pitch/Roll/Yaw mapped to CSS 3D Rotations
    // Added offsets so it displays in perspective nicely on load
    rover3DModel.style.transform = `rotateX(${-20 + pitch}deg) rotateY(${-30 + roll}deg) rotateZ(${-yaw}deg)`;
  }
}

// --- Canvas Trail Map Drawing ---
function resizeCanvas() {
  const rect = pathCanvas.parentElement.getBoundingClientRect();
  pathCanvas.width = rect.width;
  pathCanvas.height = rect.height;
  drawPath();
}

window.addEventListener('resize', () => {
  if (document.getElementById('tab-imu').classList.contains('active')) {
    resizeCanvas();
  }
});

function drawPath() {
  if (!ctx || pathCanvas.width === 0) return;
  
  // Clear canvas
  ctx.fillStyle = '#020308';
  ctx.fillRect(0, 0, pathCanvas.width, pathCanvas.height);
  
  const width = pathCanvas.width;
  const height = pathCanvas.height;
  const centerX = width / 2;
  const centerY = height / 2;
  
  // Dynamic scale (pixels per meter)
  // Auto-scales grid zoom level based on position distance to keep trace visible
  const maxDist = Math.max(0.5, ...pathHistory.map(p => Math.max(Math.abs(p.x), Math.abs(p.y))));
  const scale = (Math.min(width, height) * 0.4) / maxDist;
  
  // Draw grid lines (faint blue)
  ctx.strokeStyle = 'rgba(0, 242, 254, 0.05)';
  ctx.lineWidth = 1;
  const gridSize = 0.5 * scale; // every 0.5 meter
  
  ctx.beginPath();
  // Vertical lines
  for (let x = centerX % gridSize; x < width; x += gridSize) {
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
  }
  // Horizontal lines
  for (let y = centerY % gridSize; y < height; y += gridSize) {
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
  }
  ctx.stroke();
  
  // Draw Coordinate Axes
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
  ctx.beginPath();
  ctx.moveTo(centerX, 0);
  ctx.lineTo(centerX, height);
  ctx.moveTo(0, centerY);
  ctx.lineTo(width, centerY);
  ctx.stroke();
  
  // Draw Path Trail
  if (pathHistory.length > 1) {
    ctx.strokeStyle = 'var(--cyan-glow)';
    ctx.shadowBlur = 8;
    ctx.shadowColor = 'var(--cyan-glow)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX + pathHistory[0].x * scale, centerY - pathHistory[0].y * scale);
    for (let i = 1; i < pathHistory.length; i++) {
      ctx.lineTo(centerX + pathHistory[i].x * scale, centerY - pathHistory[i].y * scale);
    }
    ctx.stroke();
    ctx.shadowBlur = 0; // reset
  }
  
  // Draw Rover Indicator (Triangle pointing in current heading odomTheta)
  const rx = odomX / 1000;
  const ry = odomY / 1000;
  const screenX = centerX + rx * scale;
  const screenY = centerY - ry * scale;
  
  ctx.save();
  ctx.translate(screenX, screenY);
  ctx.rotate(-odomTheta); // Y-axis in canvas is inverted
  
  // Draw glowing rover triangle
  ctx.fillStyle = 'var(--red-glow)';
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(12, 0);   // Tip
  ctx.lineTo(-8, -8);  // Rear left
  ctx.lineTo(-4, 0);   // Rear center indent
  ctx.lineTo(-8, 8);   // Rear right
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

// --- Resets Event Listeners ---
document.getElementById('btn-reset-imu').addEventListener('click', () => {
  imuPitch = 0;
  imuRoll = 0;
  if (realIMUActive) {
    logSystem('Requested Pitch/Roll hardware offset reset.');
  } else {
    logSystem('Orientation model aligned to flat horizon.');
    update3DModelRotation(0, 0, imuYaw);
  }
});

document.getElementById('btn-reset-odometry').addEventListener('click', () => {
  odomX = 0;
  odomY = 0;
  odomTheta = 0;
  pathHistory = [];
  logSystem('Position odometer zeroed out.');
  drawPath();
});

// Run odometry kinematics loop at 10Hz (100ms)
setInterval(updateOdometry, 100);

// --- Encoder Diagnostics and Testing Tab Helpers ---
let encoderOffsets = [0, 0, 0, 0];

// FWD/REV holding speed test helper
let activeTestSpeeds = [0, 0, 0, 0];
function sendTestSpeed(motorIdx, speed) {
  activeTestSpeeds = [0, 0, 0, 0];
  if (motorIdx >= 0 && motorIdx < 4) {
    activeTestSpeeds[motorIdx] = speed;
  }
  // WebSocket uses range -1000..1000 for speeds
  sendServerMessage({ type: 'set_speed', speeds: activeTestSpeeds });
}

// Bind Hold-to-Run FWD/REV buttons for all 4 wheels
document.querySelectorAll('.btn-test').forEach(btn => {
  const motorIdx = parseInt(btn.dataset.motor) - 1;
  const isFwd = btn.classList.contains('btn-fwd');
  const testSpeed = isFwd ? 400 : -400; // 40% speed

  const startMotor = (e) => {
    e.preventDefault();
    btn.classList.add('active');
    sendTestSpeed(motorIdx, testSpeed);
  };

  const stopMotor = (e) => {
    e.preventDefault();
    btn.classList.remove('active');
    sendTestSpeed(motorIdx, 0);
  };

  // Mouse events
  btn.addEventListener('mousedown', startMotor);
  btn.addEventListener('mouseup', stopMotor);
  btn.addEventListener('mouseleave', stopMotor);

  // Touch events (for mobile screens)
  btn.addEventListener('touchstart', startMotor);
  btn.addEventListener('touchend', stopMotor);
  btn.addEventListener('touchcancel', stopMotor);
});

// Zero Encoders button (Frontend client-side offset calibration)
const btnResetEncodersUI = document.getElementById('btn-reset-encoders-ui');
if (btnResetEncodersUI) {
  btnResetEncodersUI.addEventListener('click', () => {
    const rawM1 = parseInt(document.getElementById('telemetry-total-m1').textContent || 0);
    const rawM2 = parseInt(document.getElementById('telemetry-total-m2').textContent || 0);
    const rawM3 = parseInt(document.getElementById('telemetry-total-m3').textContent || 0);
    const rawM4 = parseInt(document.getElementById('telemetry-total-m4').textContent || 0);

    encoderOffsets = [rawM1, rawM2, rawM3, rawM4];

    document.getElementById('test-ticks-m1').textContent = 0;
    document.getElementById('test-ticks-m2').textContent = 0;
    document.getElementById('test-ticks-m3').textContent = 0;
    document.getElementById('test-ticks-m4').textContent = 0;

    logSystem(`Zeroed out diagnostics encoder offsets: [${encoderOffsets.join(', ')}]`);
  });
}

// Straight Drive Test Event Listeners & Logic
const btnResetStraight = document.getElementById('btn-reset-straight-test');
if (btnResetStraight) {
  btnResetStraight.addEventListener('click', () => {
    const rawM1 = parseInt(document.getElementById('telemetry-total-m1').textContent || 0);
    const rawM2 = parseInt(document.getElementById('telemetry-total-m2').textContent || 0);
    const rawM3 = parseInt(document.getElementById('telemetry-total-m3').textContent || 0);
    const rawM4 = parseInt(document.getElementById('telemetry-total-m4').textContent || 0);

    straightTestOffsets = [rawM1, rawM2, rawM3, rawM4];

    document.getElementById('straight-ticks-m1').textContent = 0;
    document.getElementById('straight-ticks-m2').textContent = 0;
    document.getElementById('straight-ticks-m3').textContent = 0;
    document.getElementById('straight-ticks-m4').textContent = 0;

    updateStraightDriveMetrics(0, 0, 0, 0);
    logSystem("Zeroed out straight-drive test encoder reference offsets.");
  });
}

const btnAutoTest = document.getElementById('btn-auto-test');
if (btnAutoTest) {
  btnAutoTest.addEventListener('click', () => {
    const url = autoTestActive ? '/api/autotest/abort' : '/api/autotest/start';
    fetch(url)
      .then(res => res.json())
      .then(data => {
        if (data.ok) {
          logSystem(`Auto-test command successfully sent: ${url}`);
        } else {
          logSystem(`Error sending auto-test command: ${data.error || 'unknown error'}`);
        }
      })
      .catch(err => {
        logSystem(`Network error sending auto-test command: ${err.message}`);
      });
  });
}

const straightToggle = document.getElementById('straight-drive-lock-toggle');
const straightBadge = document.getElementById('straight-lock-badge');
if (straightToggle) {
  straightToggle.addEventListener('change', function() {
    straightDriveLocked = this.checked;
    if (straightBadge) {
      if (straightDriveLocked) {
        straightBadge.textContent = 'Steering Locked';
        straightBadge.style.background = 'rgba(239, 68, 68, 0.2)';
        straightBadge.style.color = '#fca5a5';
        straightBadge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        logSystem("🔒 Straight Drive Lock ENABLED. Steering controls are now disabled.");
      } else {
        straightBadge.textContent = 'Steering Unlocked';
        straightBadge.style.background = 'rgba(107, 114, 128, 0.2)';
        straightBadge.style.color = '#9ca3af';
        straightBadge.style.border = '1px solid rgba(107, 114, 128, 0.4)';
        logSystem("🔓 Straight Drive Lock DISABLED. Steering controls are active.");
      }
    }
  });
}

function updateStraightDriveMetrics(relM1, relM2, relM3, relM4) {
  const avgLeft = Math.round((relM1 + relM3) / 2);
  const avgRight = Math.round((relM2 + relM4) / 2);
  const delta = avgLeft - avgRight;

  const avgLeftEl = document.getElementById('straight-avg-left');
  const avgRightEl = document.getElementById('straight-avg-right');
  const deltaEl = document.getElementById('straight-mismatch-delta');
  const statusEl = document.getElementById('straight-symmetry-status');
  const barEl = document.getElementById('straight-balance-bar');
  const cursorEl = document.getElementById('straight-balance-cursor');

  if (avgLeftEl) avgLeftEl.textContent = avgLeft;
  if (avgRightEl) avgRightEl.textContent = avgRight;
  if (deltaEl) {
    deltaEl.textContent = (delta >= 0 ? '+' : '') + delta;
    deltaEl.style.color = Math.abs(delta) === 0 ? '#39ff14' : (Math.abs(delta) < 15 ? '#ffb700' : '#ff0055');
  }

  if (statusEl) {
    if (avgLeft === 0 && avgRight === 0) {
      statusEl.textContent = 'READY (0 ticks)';
      statusEl.style.color = 'var(--text-muted)';
    } else {
      const absDelta = Math.abs(delta);
      if (absDelta <= 4) {
        statusEl.textContent = 'PERFECTLY BALANCED';
        statusEl.style.color = '#39ff14';
      } else if (absDelta <= 15) {
        statusEl.textContent = 'OK (MINOR DRIFT)';
        statusEl.style.color = '#ffb700';
      } else {
        statusEl.textContent = 'MISMATCH DETECTED';
        statusEl.style.color = '#ff0055';
      }
    }
  }

  if (cursorEl && barEl) {
    const maxBarDelta = 100;
    const rawPct = (delta / maxBarDelta) * 50;
    const clampedPct = Math.max(-50, Math.min(50, rawPct));

    cursorEl.style.left = (50 + clampedPct) + '%';

    if (clampedPct >= 0) {
      barEl.style.left = '50%';
      barEl.style.width = clampedPct + '%';
      barEl.style.background = 'linear-gradient(90deg, var(--cyan-glow), var(--green-glow))';
    } else {
      barEl.style.left = (50 + clampedPct) + '%';
      barEl.style.width = Math.abs(clampedPct) + '%';
      barEl.style.background = 'linear-gradient(90deg, var(--red-glow), var(--cyan-glow))';
    }
  }
}
// Position turning triggers
document.querySelectorAll('.btn-turn').forEach(btn => {
  btn.addEventListener('click', () => {
    const wheel = btn.dataset.wheel; // "m1", "m2", "m3", "m4"
    const turns = parseFloat(document.getElementById('test-num-turns').value) || 1.0;
    logSystem(`Rotating wheel ${wheel.toUpperCase()} by ${turns} turns...`);
    fetch(`/api/turn?${wheel}=${turns}`)
      .then(res => res.json())
      .then(data => console.log('Turn started:', data))
      .catch(err => console.error('Turn API Error:', err));
  });
});

const btnTurnAll = document.getElementById('btn-turn-all');
if (btnTurnAll) {
  btnTurnAll.addEventListener('click', () => {
    const turns = parseFloat(document.getElementById('test-num-turns').value) || 1.0;
    logSystem(`Rotating ALL wheels by ${turns} turns...`);
    fetch(`/api/turn?m1=${turns}&m2=${turns}&m3=${turns}&m4=${turns}`)
      .then(res => res.json())
      .then(data => console.log('Rotate all started:', data))
      .catch(err => console.error('Turn API Error:', err));
  });
}

const btnEstopRotate = document.getElementById('btn-estop-rotate');
if (btnEstopRotate) {
  btnEstopRotate.addEventListener('click', () => {
    logSystem('⚠️ POSITION ESTOP SENT!');
    fetch('/api/turn?stop=1')
      .then(res => res.json())
      .then(data => {
        logSystem('Stopped all position modes.');
      })
      .catch(err => console.error('ESTOP API Error:', err));
  });
}

// ────────────────────────────────────────────────────────────
// Camera Control Operations
// ────────────────────────────────────────────────────────────
function updateCameraStatus(status, text, dotClass) {
  if (cameraStatusText) cameraStatusText.textContent = text;
  if (cameraStatusDot) {
    cameraStatusDot.className = 'status-indicator ' + dotClass;
  }
}

function startCameraStream() {
  if (!cameraStream) return;
  
  isCameraStreaming = true;
  updateCameraStatus('connecting', 'CONNECTING', 'alert');
  
  cameraStream.src = '/api/camera';
  
  cameraStream.onload = () => {
    updateCameraStatus('connected', 'ACTIVE', 'ok');
    logSystem("Camera stream connected successfully.");
    if (cameraStream) cameraStream.style.display = 'block';
    if (cameraPlaceholder) cameraPlaceholder.style.display = 'none';
    if (btnToggleCamera) {
      btnToggleCamera.textContent = 'Stop Feed';
      btnToggleCamera.className = 'btn btn-secondary btn-block';
    }
    if (btnFullscreenCamera) btnFullscreenCamera.disabled = false;
  };
  
  cameraStream.onerror = () => {
    if (isCameraStreaming) {
      updateCameraStatus('error', 'STREAM ERROR', 'alert');
      logSystem("Camera stream encountered an error.");
      if (cameraStream) cameraStream.style.display = 'none';
      if (cameraPlaceholder) cameraPlaceholder.style.display = 'flex';
      if (btnToggleCamera) {
        btnToggleCamera.textContent = 'Start Feed';
        btnToggleCamera.className = 'btn btn-primary btn-block';
      }
      if (btnFullscreenCamera) btnFullscreenCamera.disabled = true;
    }
  };
}

function stopCameraStream() {
  isCameraStreaming = false;
  if (cameraStream) {
    cameraStream.removeAttribute('src'); // Stop browser from fetching stream
    cameraStream.style.display = 'none';
  }
  if (cameraPlaceholder) {
    cameraPlaceholder.style.display = 'flex';
  }
  if (btnToggleCamera) {
    btnToggleCamera.textContent = 'Start Feed';
    btnToggleCamera.className = 'btn btn-primary btn-block';
  }
  if (btnFullscreenCamera) {
    btnFullscreenCamera.disabled = true;
  }
  updateCameraStatus('disconnected', 'STANDBY', 'off');
  logSystem("Camera stream stopped.");
}

if (btnToggleCamera) {
  btnToggleCamera.addEventListener('click', () => {
    if (isCameraStreaming) {
      stopCameraStream();
    } else {
      startCameraStream();
    }
  });
}

if (btnFullscreenCamera) {
  btnFullscreenCamera.addEventListener('click', () => {
    if (!cameraViewport) return;
    if (document.fullscreenElement) {
      document.exitFullscreen().catch(err => console.error(err));
    } else {
      cameraViewport.requestFullscreen().catch(err => console.error(err));
    }
  });
}

// ────────────────────────────────────────────────────────────
// Visual Compass Widget Rendering
// ────────────────────────────────────────────────────────────
const canvasCompass = document.getElementById('compass-gauge-canvas');
let compassCtx = null;
if (canvasCompass) {
  compassCtx = canvasCompass.getContext('2d');
}

function drawCompass(yawDegrees) {
  if (!compassCtx || !canvasCompass) return;
  
  const width = canvasCompass.width;
  const height = canvasCompass.height;
  const cx = width / 2;
  const cy = height / 2;
  const r = Math.min(cx, cy) - 10;
  
  // Clear
  compassCtx.clearRect(0, 0, width, height);
  
  // Draw outer ring
  compassCtx.strokeStyle = 'rgba(0, 242, 254, 0.3)';
  compassCtx.lineWidth = 2;
  compassCtx.beginPath();
  compassCtx.arc(cx, cy, r, 0, 2 * Math.PI);
  compassCtx.stroke();
  
  // Draw compass ticks and labels rotated relative to heading
  compassCtx.save();
  compassCtx.translate(cx, cy);
  compassCtx.rotate(-yawDegrees * Math.PI / 180.0);
  
  // Draw card directions (N, E, S, W)
  compassCtx.fillStyle = 'rgba(255,255,255,0.7)';
  compassCtx.font = 'bold 12px "JetBrains Mono", monospace';
  compassCtx.textAlign = 'center';
  compassCtx.textBaseline = 'middle';
  
  compassCtx.fillText('N', 0, -r + 20);
  compassCtx.fillText('S', 0, r - 20);
  compassCtx.fillText('E', r - 20, 0);
  compassCtx.fillText('W', -r + 20, 0);
  
  // Draw ticks every 30 degrees
  compassCtx.strokeStyle = 'rgba(0, 242, 254, 0.2)';
  compassCtx.lineWidth = 1;
  for (let i = 0; i < 360; i += 30) {
    if (i % 90 === 0) continue; // skip card points
    compassCtx.save();
    compassCtx.rotate(i * Math.PI / 180.0);
    compassCtx.beginPath();
    compassCtx.moveTo(0, -r);
    compassCtx.lineTo(0, -r + 10);
    compassCtx.stroke();
    compassCtx.restore();
  }
  
  compassCtx.restore();
  
  // Draw Heading indicator needle (static pointing UP)
  compassCtx.fillStyle = '#ff0055'; // neon red indicator
  compassCtx.beginPath();
  compassCtx.moveTo(cx, cy - r + 5);
  compassCtx.lineTo(cx - 6, cy - r + 15);
  compassCtx.lineTo(cx + 6, cy - r + 15);
  compassCtx.closePath();
  compassCtx.fill();
  
  // Draw center hub
  compassCtx.fillStyle = 'rgba(2, 3, 9, 0.9)';
  compassCtx.strokeStyle = 'var(--cyan-glow)';
  compassCtx.lineWidth = 1.5;
  compassCtx.beginPath();
  compassCtx.arc(cx, cy, 32, 0, 2 * Math.PI);
  compassCtx.fill();
  compassCtx.stroke();
  
  // Draw Heading degrees text in center
  compassCtx.fillStyle = '#fff';
  compassCtx.font = 'bold 11px "JetBrains Mono", monospace';
  compassCtx.textAlign = 'center';
  compassCtx.textBaseline = 'middle';
  
  // Normalize angle to [0, 360)
  let normHeading = Math.round(yawDegrees) % 360;
  if (normHeading < 0) normHeading += 360;
  
  compassCtx.fillText(`${normHeading}°`, cx, cy);
}

// Initial draw
drawCompass(0);

// ────────────────────────────────────────────────────────────
// LiDAR Monitor Controller
// ────────────────────────────────────────────────────────────
let lidarPollTimer = null;
let lastScanTime = 0;
let lidarActiveTab = 'tab-dashboard';
let latestLidarScan = null;
let hoverPoint = null;
let activeTouch = false;

function formatFeetInches(mm) {
  if (mm === undefined || mm === null || isNaN(mm) || mm === Infinity) return '--';
  const totalInches = mm / 25.4;
  const feet = Math.floor(totalInches / 12);
  const inches = (totalInches % 12).toFixed(1);
  if (feet > 0) {
    return `${feet}' ${inches}"`;
  }
  return `${inches}"`;
}

function startLidarPolling() {
  if (lidarPollTimer) return;
  pollLidar();
  lidarPollTimer = setInterval(pollLidar, 150); // poll at ~6.6Hz
  console.log('[LiDAR UI] Polling started.');
}

function stopLidarPolling() {
  if (lidarPollTimer) {
    clearInterval(lidarPollTimer);
    lidarPollTimer = null;
    console.log('[LiDAR UI] Polling stopped.');
  }
}

async function pollLidar() {
  try {
    const [statusRes, scanRes] = await Promise.all([
      fetch('/api/lidar/status').then(r => r.json()),
      fetch('/api/lidar/scan').then(r => {
        if (r.status === 200) return r.json();
        return null;
      }).catch(() => null)
    ]);
    
    updateLidarStatus(statusRes);
    if (scanRes) {
      updateLidarScan(scanRes);
    }
  } catch (err) {
    console.error('Error polling LiDAR:', err);
    const stateEl = document.getElementById('lidar-val-state');
    if (stateEl) {
      stateEl.textContent = 'ERROR';
      stateEl.style.color = '#ef4444';
    }
  }
}

function updateLidarStatus(status) {
  if (!status) return;
  
  const stateEl = document.getElementById('lidar-val-state');
  const deviceEl = document.getElementById('lidar-val-device');
  const modelEl = document.getElementById('lidar-val-model');
  const healthEl = document.getElementById('lidar-val-health');
  const fwEl = document.getElementById('lidar-val-firmware');
  const hwEl = document.getElementById('lidar-val-hardware');
  const hzEl = document.getElementById('lidar-val-scanHz');
  const ppsEl = document.getElementById('lidar-val-pps');
  const countEl = document.getElementById('lidar-val-pointCount');
  const uptimeEl = document.getElementById('lidar-val-uptime');
  const reconnectsEl = document.getElementById('lidar-val-reconnects');
  
  const errCard = document.getElementById('lidar-error-card');
  const errEl = document.getElementById('lidar-val-error');
  
  if (stateEl) {
    stateEl.textContent = status.state ? status.state.toUpperCase() : 'DISCONNECTED';
    if (status.state === 'scanning') {
      stateEl.style.color = '#10b981'; // Green
    } else if (status.state === 'connecting' || status.state === 'initializing') {
      stateEl.style.color = '#f59e0b'; // Amber
    } else {
      stateEl.style.color = '#ef4444'; // Red
    }
  }
  
  if (deviceEl) deviceEl.textContent = status.device || '--';
  if (modelEl) modelEl.textContent = status.model || '--';
  
  if (healthEl) {
    healthEl.textContent = status.health || '--';
    if (status.health === 'OK' || status.health === '0') {
      healthEl.style.color = '#10b981';
    } else if (status.health !== 'unknown') {
      healthEl.style.color = '#ef4444';
    }
  }
  
  if (fwEl) fwEl.textContent = status.firmwareVersion || '--';
  if (hwEl) hwEl.textContent = status.hardwareVersion || '--';
  if (hzEl) hzEl.textContent = status.scanHz !== undefined && status.scanHz !== null ? `${status.scanHz.toFixed(1)} Hz` : '-- Hz';
  if (ppsEl) ppsEl.textContent = status.pointsPerSecond !== undefined && status.pointsPerSecond !== null ? `${status.pointsPerSecond} pts/s` : '-- pts/s';
  if (countEl) countEl.textContent = status.latestScanPointCount !== undefined && status.latestScanPointCount !== null ? status.latestScanPointCount : '--';
  
  if (uptimeEl) {
    if (status.serviceUptimeSeconds !== undefined && status.serviceUptimeSeconds !== null) {
      const s = status.serviceUptimeSeconds;
      const hrs = Math.floor(s / 3600);
      const mins = Math.floor((s % 3600) / 60);
      const secs = s % 60;
      uptimeEl.textContent = `${hrs}h ${mins}m ${secs}s`;
    } else {
      uptimeEl.textContent = '--';
    }
  }
  
  if (reconnectsEl) reconnectsEl.textContent = status.reconnectCount !== undefined && status.reconnectCount !== null ? status.reconnectCount : '--';
  
  if (errCard && errEl) {
    if (status.lastError) {
      errCard.style.display = 'block';
      errEl.textContent = status.lastError;
    } else {
      errCard.style.display = 'none';
    }
  }
}

function updateLidarScan(scan) {
  if (!scan || !scan.timestamp) return;
  lastScanTime = Date.now();
  latestLidarScan = scan; // Keep track of the latest scan for hover/touch redraws
  
  const overlay = document.getElementById('lidar-stale-overlay');
  if (overlay) {
    overlay.style.display = 'none';
  }
  
  // Render Canvas Polar Display
  drawPolarScan(scan);
  
  // Render Table Samples
  renderSampleTable(scan);

  // Calculate and update track interference
  updateTrackInterference(scan);
}

function updateTrackInterference(scan) {
  if (!scan || !scan.points) return;
  
  const frontAngleOffset = parseFloat(document.getElementById('cfg-front-angle-offset')?.value || 0);
  const lx = parseFloat(document.getElementById('cfg-lidar-x')?.value || 0.0127);
  const ly = parseFloat(document.getElementById('cfg-lidar-y')?.value || 0.034925);
  const maxRangeCfg = parseFloat(document.getElementById('cfg-max-range')?.value || 4.0);
  
  const selectTrackWidth = document.getElementById('monitored-track-width');
  monitoredTrackWidth = selectTrackWidth ? parseFloat(selectTrackWidth.value) : 0.60;
  const W_track = monitoredTrackWidth;
  const L_track = 0.9144; // 3 feet target distance
  
  const pose = lidarPosePath.length > 0 ? lidarPosePath[lidarPosePath.length - 1] : { x: 0, y: 0, yaw: 0 };
  const x_est = pose.x;
  const y_est = pose.y;
  const yaw_est = pose.yaw;
  
  const rover_half_l = 0.2286 / 2.0 + 0.02; // chassis length/2 + margin
  const rover_half_w = 0.22225 / 2.0 + 0.02; // chassis width/2 + margin
  const rover_front_x = 0.2286 / 2.0 - lx;
  
  let minDFront = Infinity;
  let minDLeft = Infinity;
  let minDRight = Infinity;
  
  closestFrontObstacle = null;
  closestLeftObstacle = null;
  closestRightObstacle = null;
  
  const cos_yaw = Math.cos(yaw_est);
  const sin_yaw = Math.sin(yaw_est);
  
  scan.points.forEach(pt => {
    let angle = (pt.angleDeg - frontAngleOffset) % 360.0;
    if (angle < 0) angle += 360.0;
    
    const dist_m = pt.distanceMm / 1000.0;
    if (dist_m < 0.15 || dist_m > maxRangeCfg) return;
    
    const x_l = dist_m * Math.cos(angle * Math.PI / 180);
    const y_l = -dist_m * Math.sin(angle * Math.PI / 180);
    
    // Chassis self-mask
    const x_r = x_l + lx;
    const y_r = y_l + ly;
    if (x_r >= -rover_half_l && x_r <= rover_half_l && y_r >= -rover_half_w && y_r <= rover_half_w) {
      return;
    }
    
    // Transform to track frame
    const x_ref = x_l * cos_yaw - y_l * sin_yaw + x_est;
    const y_ref = x_l * sin_yaw + y_l * cos_yaw + y_est;
    
    // Categorize
    // A. Front path corridor
    if (y_ref >= -W_track / 2 && y_ref <= W_track / 2 && x_ref > x_est + rover_front_x && x_ref <= L_track + 1.0) {
      const d = x_ref - (x_est + rover_front_x);
      if (d < minDFront) {
        minDFront = d;
        closestFrontObstacle = { x: x_ref, y: y_ref, dist: d };
      }
    }
    
    // Side corridors along track length (rear of rover to target distance)
    if (x_ref >= x_est - rover_half_l && x_ref <= L_track) {
      if (y_ref > 0) {
        const d = y_ref - W_track / 2;
        if (d < minDLeft) {
          minDLeft = d;
          closestLeftObstacle = { x: x_ref, y: y_ref, dist: d };
        }
      } else {
        const d = -y_ref - W_track / 2;
        if (d < minDRight) {
          minDRight = d;
          closestRightObstacle = { x: x_ref, y: y_ref, dist: d };
        }
      }
    }
  });
  
  updateTrackInterferenceUI(minDFront, minDLeft, minDRight);
  
  // Redraw canvas if test is IDLE to show the track and obstacles in real-time
  if (lidarTestState === 'IDLE') {
    drawLidarTestCanvas();
  }
}

function updateTrackInterferenceUI(dFront, dLeft, dRight) {
  const elBadge = document.getElementById('interference-warning-badge');
  const elFront = document.getElementById('val-interfere-front');
  const elLeft = document.getElementById('val-interfere-left');
  const elRight = document.getElementById('val-interfere-right');
  
  const boxFront = document.getElementById('box-interfere-front');
  const boxLeft = document.getElementById('box-interfere-left');
  const boxRight = document.getElementById('box-interfere-right');
  
  function formatValAndStyle(el, box, d) {
    if (!el || !box) return;
    if (d === Infinity || d === -Infinity || d === undefined || d === null) {
      el.textContent = 'None';
      el.style.color = '#10b981';
      box.style.borderColor = 'rgba(255,255,255,0.05)';
      return;
    }
    
    const ftIn = formatFeetInches(Math.abs(d) * 1000);
    if (d < 0) {
      el.textContent = `Inside ${Math.abs(d).toFixed(2)}m (${ftIn})`;
      el.style.color = '#ff0055';
      box.style.borderColor = 'rgba(255, 0, 85, 0.4)';
    } else if (d < 0.15) {
      el.textContent = `Close: ${d.toFixed(2)}m (${ftIn})`;
      el.style.color = '#f59e0b';
      box.style.borderColor = 'rgba(245, 158, 11, 0.4)';
    } else {
      el.textContent = `${d.toFixed(2)}m (${ftIn})`;
      el.style.color = '#10b981';
      box.style.borderColor = 'rgba(16, 185, 129, 0.2)';
    }
  }
  
  formatValAndStyle(elFront, boxFront, dFront);
  formatValAndStyle(elLeft, boxLeft, dLeft);
  formatValAndStyle(elRight, boxRight, dRight);
  
  if (elBadge) {
    if (dFront < 0 || dLeft < 0 || dRight < 0) {
      elBadge.textContent = '⚠️ Interference';
      elBadge.style.background = 'rgba(255, 0, 85, 0.15)';
      elBadge.style.color = '#ff0055';
      elBadge.style.borderColor = 'rgba(255, 0, 85, 0.4)';
    } else if (dFront < 0.15 || dLeft < 0.15 || dRight < 0.15) {
      elBadge.textContent = '⚠️ Caution';
      elBadge.style.background = 'rgba(245, 158, 11, 0.15)';
      elBadge.style.color = '#f59e0b';
      elBadge.style.borderColor = 'rgba(245, 158, 11, 0.4)';
    } else {
      elBadge.textContent = '✓ Clear';
      elBadge.style.background = 'rgba(16, 185, 129, 0.15)';
      elBadge.style.color = '#10b981';
      elBadge.style.borderColor = 'rgba(16, 185, 129, 0.4)';
    }
  }
}


function drawPolarScan(scan) {
  const canvas = document.getElementById('lidar-polar-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  
  // Set dimensions correctly (support responsive canvas scaling)
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  
  const width = rect.width;
  const height = rect.height;
  const centerX = width / 2;
  const centerY = height / 2;
  const radius = Math.min(centerX, centerY) - 30; // padding for labels
  if (radius <= 0) return;

  
  // Get selected range (in mm)
  const rangeSelect = document.getElementById('lidar-range-select');
  const maxRangeMm = rangeSelect ? parseFloat(rangeSelect.value) : 3000.0;
  
  const scale = radius / maxRangeMm; // pixels per mm
  
  // Clear background
  ctx.fillStyle = '#0b0f19';
  ctx.fillRect(0, 0, width, height);
  
  // Draw concentric rings and grid inside the clip circle
  ctx.save();
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
  ctx.clip();
  
  // Draw concentric rings
  ctx.shadowBlur = 0;
  ctx.strokeStyle = 'rgba(0, 242, 254, 0.08)';
  ctx.lineWidth = 1;
  
  const ringStep = maxRangeMm / 3;
  for (let rMm = ringStep; rMm <= maxRangeMm; rMm += ringStep) {
    ctx.beginPath();
    ctx.arc(centerX, centerY, rMm * scale, 0, 2 * Math.PI);
    ctx.stroke();
    
    // Draw label
    ctx.fillStyle = 'rgba(0, 242, 254, 0.4)';
    ctx.font = '9px "JetBrains Mono", monospace';
    ctx.fillText(`${(rMm / 1000.0).toFixed(1)}m`, centerX + 5, centerY - rMm * scale - 2);
  }
  
  // Draw crosshair axes
  ctx.strokeStyle = 'rgba(0, 242, 254, 0.15)';
  ctx.beginPath();
  // Vertical line
  ctx.moveTo(centerX, centerY - radius);
  ctx.lineTo(centerX, centerY + radius);
  // Horizontal line
  ctx.moveTo(centerX - radius, centerY);
  ctx.lineTo(centerX + radius, centerY);
  ctx.stroke();
  
  ctx.restore();
  
  // Draw outer degree circle border
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
  ctx.stroke();
  
  // Draw orientation labels
  ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  
  ctx.fillText('0° FRONT', centerX, centerY - radius - 15);
  ctx.fillText('180° REAR', centerX, centerY + radius + 15);
  
  ctx.textAlign = 'left';
  ctx.fillText('90° RIGHT', centerX + radius + 8, centerY);
  
  ctx.textAlign = 'right';
  ctx.fillText('270° LEFT', centerX - radius - 8, centerY);
  
  // Draw the points
  if (scan.points && scan.points.length > 0) {
    let closestPt = null;
    let minDistance = Infinity;
    
    for (const pt of scan.points) {
      if (pt.distanceMm < minDistance) {
        minDistance = pt.distanceMm;
        closestPt = pt;
      }
      
      // Calculate coordinates (0° is vertical-up, clockwise angles)
      const angleRad = (pt.angleDeg - 90) * Math.PI / 180;
      const x = centerX + pt.distanceMm * scale * Math.cos(angleRad);
      const y = centerY + pt.distanceMm * scale * Math.sin(angleRad);
      
      // Only draw if point lies within the visual range circle
      const distFromCenter = pt.distanceMm * scale;
      if (distFromCenter <= radius) {
        ctx.beginPath();
        ctx.arc(x, y, 2.5, 0, 2 * Math.PI);
        ctx.shadowBlur = 4;
        
        if (pt.distanceMm < 500) {
          ctx.fillStyle = '#ff0055'; // Danger red
          ctx.shadowColor = 'rgba(255,0,85,0.6)';
        } else {
          ctx.fillStyle = '#00f2fe'; // Cyan glow
          ctx.shadowColor = 'rgba(0,242,254,0.6)';
        }
        ctx.fill();
      }
    }
    
    // Draw HUD text on canvas (top left and bottom left)
    ctx.shadowBlur = 0;
    ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`Scan Hz: ${scan.scanHz.toFixed(1)} Hz`, 15, 20);
    ctx.fillText(`Points: ${scan.pointCount}`, 15, 35);
    
    if (closestPt) {
      ctx.fillStyle = closestPt.distanceMm < 500 ? '#ff0055' : '#00f2fe';
      const closestFtIn = formatFeetInches(closestPt.distanceMm);
      ctx.fillText(`Closest: ${closestFtIn} (${closestPt.distanceMm}mm) @ ${closestPt.angleDeg.toFixed(1)}°`, 15, height - 20);
    }
    
    // Handle user touch/hover interaction on the LiDAR canvas
    if (hoverPoint) {
      // Calculate cursor vector from sensor (centerX, centerY)
      const dx = hoverPoint.x - centerX;
      const dy = hoverPoint.y - centerY;
      const distPx = Math.sqrt(dx * dx + dy * dy);
      
      // Calculate angle in degrees (0° front, clockwise)
      let angleRad = Math.atan2(dy, dx);
      let angleDeg = angleRad * 180 / Math.PI + 90;
      if (angleDeg < 0) angleDeg += 360;
      angleDeg = angleDeg % 360;
      
      // Snap to closest scan point within 15px radius in screen space
      let closestHoverPt = null;
      let minHoverDistPx = 15;
      
      for (const pt of scan.points) {
        const ptAngleRad = (pt.angleDeg - 90) * Math.PI / 180;
        const ptX = centerX + pt.distanceMm * scale * Math.cos(ptAngleRad);
        const ptY = centerY + pt.distanceMm * scale * Math.sin(ptAngleRad);
        
        const pdx = hoverPoint.x - ptX;
        const pdy = hoverPoint.y - ptY;
        const pDistPx = Math.sqrt(pdx * pdx + pdy * pdy);
        
        if (pDistPx < minHoverDistPx) {
          minHoverDistPx = pDistPx;
          closestHoverPt = pt;
        }
      }
      
      if (closestHoverPt) {
        // Highlight the snapped scan point
        const targetAngleRad = (closestHoverPt.angleDeg - 90) * Math.PI / 180;
        const targetX = centerX + closestHoverPt.distanceMm * scale * Math.cos(targetAngleRad);
        const targetY = centerY + closestHoverPt.distanceMm * scale * Math.sin(targetAngleRad);
        
        // Target pulse ring
        ctx.strokeStyle = '#ff0055';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(targetX, targetY, 8, 0, 2 * Math.PI);
        ctx.stroke();
        
        // Dotted line to target
        ctx.strokeStyle = 'rgba(255, 0, 85, 0.4)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(targetX, targetY);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // Tooltip label
        const ftInStr = formatFeetInches(closestHoverPt.distanceMm);
        const labelText = `${ftInStr} (${closestHoverPt.distanceMm}mm) @ ${closestHoverPt.angleDeg.toFixed(1)}°`;
        
        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        const textWidth = ctx.measureText(labelText).width;
        
        // Determine placement direction so it doesn't clip off-screen
        const tooltipX = targetX + 10 + textWidth + 15 > width ? targetX - textWidth - 25 : targetX + 10;
        const tooltipY = targetY - 13;
        
        ctx.fillStyle = 'rgba(11, 15, 25, 0.9)';
        ctx.strokeStyle = 'rgba(255, 0, 85, 0.8)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(tooltipX - 5, tooltipY - 9, textWidth + 10, 18, 4);
        ctx.fill();
        ctx.stroke();
        
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(labelText, tooltipX, tooltipY);
        
      } else if (distPx <= radius) {
        // Freeform hover within the radius
        ctx.strokeStyle = 'rgba(0, 242, 254, 0.3)';
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(hoverPoint.x, hoverPoint.y);
        ctx.stroke();
        ctx.setLineDash([]);
        
        // Small pointer dot
        ctx.fillStyle = '#00f2fe';
        ctx.beginPath();
        ctx.arc(hoverPoint.x, hoverPoint.y, 4, 0, 2 * Math.PI);
        ctx.fill();
        
        // Tooltip label at pointer
        const hoverMm = distPx / scale;
        const ftInStr = formatFeetInches(hoverMm);
        const labelText = `${ftInStr} (${Math.round(hoverMm)}mm) @ ${angleDeg.toFixed(1)}°`;
        
        ctx.font = 'bold 11px "JetBrains Mono", monospace';
        const textWidth = ctx.measureText(labelText).width;
        
        const tooltipX = hoverPoint.x + 10 + textWidth + 15 > width ? hoverPoint.x - textWidth - 25 : hoverPoint.x + 10;
        const tooltipY = hoverPoint.y - 13;
        
        ctx.fillStyle = 'rgba(11, 15, 25, 0.9)';
        ctx.strokeStyle = 'rgba(0, 242, 254, 0.8)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(tooltipX - 5, tooltipY - 9, textWidth + 10, 18, 4);
        ctx.fill();
        ctx.stroke();
        
        ctx.fillStyle = '#fff';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(labelText, tooltipX, tooltipY);
      }
    }
  } else {
    // No points
    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.font = '12px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('NO DATA', centerX, centerY + 30);
  }
  
  // Draw a top-down rover icon in the center
  ctx.save();
  ctx.translate(centerX, centerY);
  
  // Rover physical dimensions: 9" long (228.6mm) by 8.75" wide (222.25mm)
  // LiDAR is 4" (101.6mm) from front, 3" (76.2mm) from left
  // Scale factor (pixels/mm) is calculated above as `scale`.
  // To avoid the icon disappearing or getting too small at longer range views,
  // we clamp the drawing scale at a minimum of 0.08 (~2.75m range equivalent).
  const drawScale = Math.max(scale, 0.08);
  
  const roverW_mm = 8.75 * 25.4; // 222.25 mm
  const roverL_mm = 9.0 * 25.4;  // 228.6 mm
  const lidarOffsetFromFront_mm = 4.0 * 25.4; // 101.6 mm
  const lidarOffsetFromLeft_mm = 3.0 * 25.4;  // 76.2 mm
  
  // Calculate relative bounds where the LiDAR is at (0, 0)
  // Since 0 degrees is facing front (which is -Y in canvas space):
  // front boundary is along -Y
  const frontY = -lidarOffsetFromFront_mm * drawScale;
  const rearY = (roverL_mm - lidarOffsetFromFront_mm) * drawScale;
  // left boundary is along -X
  const leftX = -lidarOffsetFromLeft_mm * drawScale;
  const rightX = (roverW_mm - lidarOffsetFromLeft_mm) * drawScale;
  
  const bodyW = roverW_mm * drawScale;
  const bodyH = roverL_mm * drawScale;
  const bodyCenterX = leftX + bodyW / 2;
  
  // Draw tracks/wheels
  // Assume each track/wheel is 1.25" wide (31.75mm) and 2.5" long (63.5mm)
  ctx.fillStyle = '#1e293b';
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
  ctx.lineWidth = 1;
  const wheelW = 1.25 * 25.4 * drawScale;
  const wheelH = 2.5 * 25.4 * drawScale;
  
  // Left side wheels (LF and LR)
  // LF
  ctx.fillRect(leftX - wheelW, frontY + 0.1 * bodyH, wheelW, wheelH);
  ctx.strokeRect(leftX - wheelW, frontY + 0.1 * bodyH, wheelW, wheelH);
  // LR
  ctx.fillRect(leftX - wheelW, rearY - 0.1 * bodyH - wheelH, wheelW, wheelH);
  ctx.strokeRect(leftX - wheelW, rearY - 0.1 * bodyH - wheelH, wheelW, wheelH);
  
  // Right side wheels (RF and RR)
  // RF
  ctx.fillRect(rightX, frontY + 0.1 * bodyH, wheelW, wheelH);
  ctx.strokeRect(rightX, frontY + 0.1 * bodyH, wheelW, wheelH);
  // RR
  ctx.fillRect(rightX, rearY - 0.1 * bodyH - wheelH, wheelW, wheelH);
  ctx.strokeRect(rightX, rearY - 0.1 * bodyH - wheelH, wheelW, wheelH);
  
  // Draw body
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#00f2fe';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(leftX, frontY, bodyW, bodyH, 3 * drawScale);
  ctx.fill();
  ctx.stroke();
  
  // Draw front red nose indicator (centered on the body width)
  ctx.fillStyle = '#ff0055';
  ctx.beginPath();
  const noseW = 3.0 * 25.4 * drawScale; // 3 inches wide base
  const noseH = 4.0 * 25.4 * drawScale; // 4 inches long nose
  ctx.moveTo(bodyCenterX - noseW / 2, frontY);
  ctx.lineTo(bodyCenterX + noseW / 2, frontY);
  ctx.lineTo(bodyCenterX, frontY - noseH);
  ctx.closePath();
  ctx.fill();
  
  // Draw LiDAR physical mounting sensor outline at (0, 0)
  // RPLIDAR C1 has a physical diameter of roughly 55.6mm (2.2 inches)
  const lidarRadius = (2.2 / 2) * 25.4 * drawScale;
  ctx.fillStyle = '#1e293b';
  ctx.strokeStyle = '#ff3366'; // bright pink/red for physical lidar housing
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(0, 0, lidarRadius, 0, 2 * Math.PI);
  ctx.fill();
  ctx.stroke();
  
  // Small center dot for the LiDAR optical origin
  ctx.fillStyle = '#ff3366';
  ctx.beginPath();
  ctx.arc(0, 0, 1.5, 0, 2 * Math.PI);
  ctx.fill();
  
  ctx.restore();
}

function renderSampleTable(scan) {
  const tbody = document.getElementById('lidar-sample-table-body');
  if (!tbody) return;
  
  if (!scan.points || scan.points.length === 0) {
    tbody.innerHTML = `<tr><td colspan="5" style="text-align: center; padding: 20px; color: var(--text-muted);">No valid points available.</td></tr>`;
    return;
  }
  
  const numSamples = 15;
  const step = Math.max(1, Math.floor(scan.points.length / numSamples));
  let html = '';
  
  for (let i = 0; i < scan.points.length; i += step) {
    const pt = scan.points[i];
    const distFtIn = formatFeetInches(pt.distanceMm);
    
    // Warn if close
    const distColor = pt.distanceMm < 500 ? '#ff0055' : 'inherit';
    const distWeight = pt.distanceMm < 500 ? 'bold' : 'normal';
    
    html += `
      <tr style="color: ${distColor}; font-weight: ${distWeight};">
        <td style="padding: 8px 15px;">#${i + 1}</td>
        <td style="padding: 8px 15px;">${pt.angleDeg.toFixed(2)}°</td>
        <td style="padding: 8px 15px;">${pt.distanceMm} mm</td>
        <td style="padding: 8px 15px;">${distFtIn}</td>
        <td style="padding: 8px 15px;">${pt.quality}</td>
      </tr>
    `;
  }
  tbody.innerHTML = html;
}

// Watch tab switching for LiDAR polling triggers
document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    lidarActiveTab = btn.dataset.tab;
    if (lidarActiveTab === 'tab-lidar' || lidarActiveTab === 'tab-encoder') {
      startLidarPolling();
    } else {
      stopLidarPolling();
    }
  });
});

// Watch visibility changes
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    stopLidarPolling();
  } else if (lidarActiveTab === 'tab-lidar' || lidarActiveTab === 'tab-encoder') {
    startLidarPolling();
  }
});

// Periodic stale data check
setInterval(() => {
  const overlay = document.getElementById('lidar-stale-overlay');
  if (overlay && (lidarActiveTab === 'tab-lidar' || lidarActiveTab === 'tab-encoder') && lastScanTime > 0 && Date.now() - lastScanTime > 1000) {
    if (lidarActiveTab === 'tab-lidar') {
      overlay.style.display = 'flex';
    }
  }
}, 500);


// Setup mouse/touch event listeners on the LiDAR polar canvas for distance measurement
(function initLidarCanvasInteraction() {
  const canvas = document.getElementById('lidar-polar-canvas');
  if (!canvas) {
    // If not loaded yet, try again on DOMContentLoaded
    document.addEventListener('DOMContentLoaded', initLidarCanvasInteraction);
    return;
  }
  
  const updatePointer = (e) => {
    const rect = canvas.getBoundingClientRect();
    let clientX, clientY;
    if (e.touches && e.touches.length > 0) {
      clientX = e.touches[0].clientX;
      clientY = e.touches[0].clientY;
      activeTouch = true;
    } else {
      clientX = e.clientX;
      clientY = e.clientY;
      activeTouch = false;
    }
    
    // Scale coords to handle CSS scaling vs canvas internal dimensions
    const px = clientX - rect.left;
    const py = clientY - rect.top;
    
    hoverPoint = { x: px, y: py };
    
    // Redraw immediately when user moves pointer
    if (latestLidarScan) {
      drawPolarScan(latestLidarScan);
    }
  };
  
  const clearPointer = () => {
    hoverPoint = null;
    activeTouch = false;
    if (latestLidarScan) {
      drawPolarScan(latestLidarScan);
    }
  };
  
  canvas.addEventListener('mousemove', updatePointer);
  canvas.addEventListener('touchmove', updatePointer, { passive: true });
  canvas.addEventListener('touchstart', (e) => {
    // Prevent scrolling when tapping/dragging on the canvas
    e.preventDefault();
    updatePointer(e);
  }, { passive: false });
  
  canvas.addEventListener('mouseleave', clearPointer);
  canvas.addEventListener('mouseup', clearPointer);
  canvas.addEventListener('touchend', clearPointer);
  canvas.addEventListener('touchcancel', clearPointer);
})();

// ────────────────────────────────────────────────────────────
// Calibration Database UI & Management Handlers
// ────────────────────────────────────────────────────────────
function updateCalibrationDbUI(db) {
  if (!db) return;
  calibrationDatabase = db; // sync it locally
  
  // 1. Current Configuration
  const curDiameter = document.getElementById('db-cur-diameter');
  const curTrack = document.getElementById('db-cur-track');
  if (db.currentConfig) {
    if (db.currentConfig.wheelDiameter) {
      currentWheelDiameter = db.currentConfig.wheelDiameter;
      if (curDiameter) curDiameter.textContent = (db.currentConfig.wheelDiameter * 1000).toFixed(1);
    }
    if (db.currentConfig.effectiveTrackWidth) {
      currentTrackWidth = db.currentConfig.effectiveTrackWidth;
      if (curTrack) curTrack.textContent = (db.currentConfig.effectiveTrackWidth * 1000).toFixed(1);
    }
  }
  
  // Update labels in calibration cards
  const lblCurDia = document.getElementById('cal-dist-current-diameter');
  if (lblCurDia) lblCurDia.textContent = `${(currentWheelDiameter * 1000).toFixed(1)} mm`;
  
  const lblCurTrack = document.getElementById('cal-rot-current-width');
  if (lblCurTrack) lblCurTrack.textContent = `${(currentTrackWidth * 1000).toFixed(1)} mm`;
  
  // 2. Proposed Configuration
  const propDiameter = document.getElementById('db-prop-diameter');
  const propTrack = document.getElementById('db-prop-track');
  if (db.proposedConfig) {
    if (propDiameter) propDiameter.textContent = db.proposedConfig.wheelDiameter ? (db.proposedConfig.wheelDiameter * 1000).toFixed(1) : '--';
    if (propTrack) propTrack.textContent = db.proposedConfig.effectiveTrackWidth ? (db.proposedConfig.effectiveTrackWidth * 1000).toFixed(1) : '--';
  } else {
    if (propDiameter) propDiameter.textContent = '--';
    if (propTrack) propTrack.textContent = '--';
  }
  
  // 3. Previous Configuration
  const prevDiameter = document.getElementById('db-prev-diameter');
  const prevTrack = document.getElementById('db-prev-track');
  if (db.previousConfig) {
    if (prevDiameter) prevDiameter.textContent = db.previousConfig.wheelDiameter ? (db.previousConfig.wheelDiameter * 1000).toFixed(1) : '--';
    if (prevTrack) prevTrack.textContent = db.previousConfig.effectiveTrackWidth ? (db.previousConfig.effectiveTrackWidth * 1000).toFixed(1) : '--';
  } else {
    if (prevDiameter) prevDiameter.textContent = '--';
    if (prevTrack) prevTrack.textContent = '--';
  }
  
  // 4. History Logs Table
  const tbody = document.getElementById('cal-history-table-body');
  if (tbody) {
    const logs = db.testLogs || [];
    if (logs.length === 0) {
      tbody.innerHTML = `<tr><td colspan="5" style="text-align: center; padding: 20px; color: var(--text-muted);">No logs available in database.</td></tr>`;
    } else {
      const sortedLogs = [...logs].sort((a, b) => b.timestamp - a.timestamp);
      tbody.innerHTML = sortedLogs.map(log => {
        const dateStr = new Date(log.timestamp).toLocaleString();
        let summary = '';
        if (typeof log.results === 'string') {
          summary = log.results;
        } else if (log.results && typeof log.results === 'object') {
          summary = Object.entries(log.results)
            .map(([key, val]) => `${key}: ${typeof val === 'number' ? val.toFixed(4) : val}`)
            .join(', ');
        }
        return `
          <tr style="border-bottom: 1px solid var(--border);">
            <td style="padding: 8px 10px; font-family: monospace; white-space: nowrap;">${dateStr}</td>
            <td style="padding: 8px 10px; font-weight: bold; color: var(--cyan-glow);">${log.testType}</td>
            <td style="padding: 8px 10px;">${log.surfaceType}</td>
            <td style="padding: 8px 10px; font-family: monospace;">${log.firmwareVersion}</td>
            <td style="padding: 8px 10px; color: var(--text-muted); font-size: 11px;">${summary}</td>
          </tr>
        `;
      }).join('');
    }
  }
}

function applyRecommendedCalibration() {
  sendServerMessage({ type: 'apply_calibration' });
  logSystem('[Calibration] Requested applying proposed configuration.');
}

function restorePreviousCalibration() {
  sendServerMessage({ type: 'restore_previous' });
  logSystem('[Calibration] Requested restoring previous configuration.');
}

function saveSurfaceType() {
  const select = document.getElementById('cal-surface-type');
  if (select) {
    localStorage.setItem('cal_surface_type', select.value);
    logSystem(`[Calibration] Saved surface preference: ${select.value}`);
  }
}

// ── Distance & Wheel Calibration ──
function startDistanceTest() {
  const circum = Math.PI * currentWheelDiameter;
  const turns = 2.0 / circum;
  logSystem(`[Calibration Test] Starting 2m distance test (${turns.toFixed(3)} turns)...`);
  fetch(`/api/turn?m1=${turns}&m2=${turns}&m3=${turns}&m4=${turns}`)
    .then(res => res.json())
    .then(data => logSystem(`[Calibration Test] Distance test response: ${JSON.stringify(data)}`))
    .catch(err => console.error('Failed to run distance test:', err));
}

function onDistanceTrialChange() {
  const t1 = parseFloat(document.getElementById('cal-dist-trial1').value);
  const t2 = parseFloat(document.getElementById('cal-dist-trial2').value);
  const currentDiaMm = currentWheelDiameter * 1000;
  
  let prop1 = null;
  let prop2 = null;
  
  if (!isNaN(t1) && t1 > 0) {
    prop1 = currentDiaMm * (t1 / 2.0);
    document.getElementById('cal-dist-prop1').textContent = `${prop1.toFixed(1)} mm`;
  } else {
    document.getElementById('cal-dist-prop1').textContent = '-- mm';
  }
  
  if (!isNaN(t2) && t2 > 0) {
    prop2 = currentDiaMm * (t2 / 2.0);
    document.getElementById('cal-dist-prop2').textContent = `${prop2.toFixed(1)} mm`;
  } else {
    document.getElementById('cal-dist-prop2').textContent = '-- mm';
  }
  
  if (prop1 && prop2) {
    const avg = (prop1 + prop2) / 2;
    const diff = (Math.abs(prop1 - prop2) / avg) * 100;
    document.getElementById('cal-dist-avg').textContent = `${avg.toFixed(1)} mm`;
    document.getElementById('cal-dist-diff').textContent = `${diff.toFixed(2)} %`;
    
    if (diff > 3.0) {
      document.getElementById('cal-dist-warning').style.display = 'block';
      document.getElementById('btn-cal-dist-apply').disabled = true;
      document.getElementById('btn-cal-dist-apply').style.opacity = 0.6;
      document.getElementById('btn-cal-dist-apply').style.cursor = 'not-allowed';
    } else {
      document.getElementById('cal-dist-warning').style.display = 'none';
      document.getElementById('btn-cal-dist-apply').disabled = false;
      document.getElementById('btn-cal-dist-apply').style.opacity = 1.0;
      document.getElementById('btn-cal-dist-apply').style.cursor = 'pointer';
    }
  } else {
    document.getElementById('cal-dist-avg').textContent = '-- mm';
    document.getElementById('cal-dist-diff').textContent = '-- %';
    document.getElementById('cal-dist-warning').style.display = 'none';
    document.getElementById('btn-cal-dist-apply').disabled = true;
    document.getElementById('btn-cal-dist-apply').style.opacity = 0.6;
    document.getElementById('btn-cal-dist-apply').style.cursor = 'not-allowed';
  }
}

function applyWheelCalibration() {
  const avgText = document.getElementById('cal-dist-avg').textContent;
  const avgMm = parseFloat(avgText);
  if (!isNaN(avgMm)) {
    const diaM = avgMm / 1000.0;
    sendServerMessage({
      type: 'save_proposed_config',
      wheelDiameter: diaM,
      effectiveTrackWidth: currentTrackWidth
    });
    sendServerMessage({ type: 'apply_calibration' });
    logSystem(`[Calibration] Applied new wheel diameter: ${avgMm.toFixed(1)} mm`);
    
    const surfaceSelect = document.getElementById('cal-surface-type');
    const surface = surfaceSelect ? surfaceSelect.value : 'unknown';
    sendServerMessage({
      type: 'log_test_run',
      testType: 'Wheel Diameter',
      results: `Calibrated wheel diameter to ${avgMm.toFixed(1)} mm`,
      surfaceType: surface
    });
  }
}

function clearDistanceTrials() {
  document.getElementById('cal-dist-trial1').value = '';
  document.getElementById('cal-dist-trial2').value = '';
  onDistanceTrialChange();
}

// ── Rotation & Track Width Calibration ──
function startRotationTest(isCw) {
  const turns = currentTrackWidth / currentWheelDiameter;
  logSystem(`[Calibration Test] Starting 360° ${isCw ? 'CW' : 'CCW'} test (${turns.toFixed(3)} turns)...`);
  const m1 = isCw ? turns : -turns;
  const m2 = isCw ? -turns : turns;
  const m3 = isCw ? turns : -turns;
  const m4 = isCw ? -turns : turns;
  fetch(`/api/turn?m1=${m1}&m2=${m2}&m3=${m3}&m4=${m4}`)
    .then(res => res.json())
    .then(data => logSystem(`[Calibration Test] Rotation test response: ${JSON.stringify(data)}`))
    .catch(err => console.error('Failed to run rotation test:', err));
}

function startRotationVerification(isCw) {
  startRotationTest(isCw);
}

function onRotationTrialChange() {
  const cw = parseFloat(document.getElementById('cal-rot-cw-angle').value);
  const ccw = parseFloat(document.getElementById('cal-rot-ccw-angle').value);
  const currentWidthMm = currentTrackWidth * 1000;
  
  let prop1 = null;
  let prop2 = null;
  
  if (!isNaN(cw) && cw > 0) {
    prop1 = currentWidthMm * (360.0 / cw);
    document.getElementById('cal-rot-prop1').textContent = `${prop1.toFixed(1)} mm`;
  } else {
    document.getElementById('cal-rot-prop1').textContent = '-- mm';
  }
  
  if (!isNaN(ccw) && ccw > 0) {
    prop2 = currentWidthMm * (360.0 / ccw);
    document.getElementById('cal-rot-prop2').textContent = `${prop2.toFixed(1)} mm`;
  } else {
    document.getElementById('cal-rot-prop2').textContent = '-- mm';
  }
  
  if (prop1 && prop2) {
    const avg = (prop1 + prop2) / 2;
    const diff = (Math.abs(prop1 - prop2) / avg) * 100;
    document.getElementById('cal-rot-avg').textContent = `${avg.toFixed(1)} mm`;
    document.getElementById('cal-rot-diff').textContent = `${diff.toFixed(2)} %`;
    
    if (diff > 5.0) {
      document.getElementById('cal-rot-warning').style.display = 'block';
      document.getElementById('btn-cal-rot-apply').disabled = true;
      document.getElementById('btn-cal-rot-apply').style.opacity = 0.6;
      document.getElementById('btn-cal-rot-apply').style.cursor = 'not-allowed';
    } else {
      document.getElementById('cal-rot-warning').style.display = 'none';
      document.getElementById('btn-cal-rot-apply').disabled = false;
      document.getElementById('btn-cal-rot-apply').style.opacity = 1.0;
      document.getElementById('btn-cal-rot-apply').style.cursor = 'pointer';
    }
  } else {
    document.getElementById('cal-rot-avg').textContent = '-- mm';
    document.getElementById('cal-rot-diff').textContent = '-- %';
    document.getElementById('cal-rot-warning').style.display = 'none';
    document.getElementById('btn-cal-rot-apply').disabled = true;
    document.getElementById('btn-cal-rot-apply').style.opacity = 0.6;
    document.getElementById('btn-cal-rot-apply').style.cursor = 'not-allowed';
  }
}

function applyTrackWidthCalibration() {
  const avgText = document.getElementById('cal-rot-avg').textContent;
  const avgMm = parseFloat(avgText);
  if (!isNaN(avgMm)) {
    const widthM = avgMm / 1000.0;
    sendServerMessage({
      type: 'save_proposed_config',
      wheelDiameter: currentWheelDiameter,
      effectiveTrackWidth: widthM
    });
    sendServerMessage({ type: 'apply_calibration' });
    logSystem(`[Calibration] Applied new track width: ${avgMm.toFixed(1)} mm`);
    
    const surfaceSelect = document.getElementById('cal-surface-type');
    const surface = surfaceSelect ? surfaceSelect.value : 'unknown';
    sendServerMessage({
      type: 'log_test_run',
      testType: 'Track Width',
      results: `Calibrated effective track width to ${avgMm.toFixed(1)} mm`,
      surfaceType: surface
    });
  }
}

function clearRotationTrials() {
  document.getElementById('cal-rot-cw-angle').value = '';
  document.getElementById('cal-rot-ccw-angle').value = '';
  onRotationTrialChange();
}

// ── Out-and-Back Validation ──
function startOutAndBackTest() {
  logSystem('[Calibration Test] Starting Out-and-Back (autotest) sequence...');
  fetch('/api/autotest/start')
    .then(res => res.json())
    .then(data => logSystem(`[Calibration Test] Out-and-Back status: ${JSON.stringify(data)}`))
    .catch(err => console.error('Failed to start Out-and-Back test:', err));
}

function logOutAndBackTrial() {
  const surfaceSelect = document.getElementById('cal-surface-type');
  const surface = surfaceSelect ? surfaceSelect.value : 'unknown';
  sendServerMessage({
    type: 'log_test_run',
    testType: 'Out-and-Back Validation',
    results: 'Out-and-back validation test logged by user',
    surfaceType: surface
  });
}

// ── Backtracking Recording & Return ──
async function startPathRecording() {
  try {
    const res = await fetch('/api/path/record/start', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Path] Recording started: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to start path recording:', err);
  }
}

async function stopPathRecording() {
  try {
    const res = await fetch('/api/path/record/stop', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Path] Recording stopped: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to stop path recording:', err);
  }
}

async function startBacktrackHome() {
  try {
    const res = await fetch('/api/path/backtrack/start', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Path] Backtracking started: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to start backtrack:', err);
  }
}

async function abortBacktrackHome() {
  try {
    const res = await fetch('/api/path/backtrack/stop', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Path] Backtracking aborted: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to abort backtrack:', err);
  }
}

function logBacktrackTrial() {
  const surfaceSelect = document.getElementById('cal-surface-type');
  const surface = surfaceSelect ? surfaceSelect.value : 'unknown';
  sendServerMessage({
    type: 'log_test_run',
    testType: 'Backtrack Validation',
    results: 'Backtrack return validation test logged by user',
    surfaceType: surface
  });
}

// ── Breakaway Calibration & Safety Control ──
async function triggerCalibrateStart() {
  try {
    const res = await fetch('/api/calibration/simulate/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ safetyAck: true })
    });
    const data = await res.json();
    logSystem(`[Calibration Simulation] Started: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to start calibration simulation:', err);
  }
}

async function triggerCalibrateCancel() {
  try {
    const res = await fetch('/api/calibration/abort', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Calibration] Aborted: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to abort calibration:', err);
  }
}

async function triggerRealCalibrateStart() {
  try {
    const res = await fetch('/api/calibration/real/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ safetyAck: true })
    });
    const data = await res.json();
    logSystem(`[Real Calibration] Started: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to start real calibration:', err);
  }
}

// ── Maintenance Mode ──
async function enterMaintenanceMode() {
  try {
    const res = await fetch('/api/maintenance/enter', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ safetyAck: true })
    });
    const data = await res.json();
    logSystem(`[Maintenance] Entered: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to enter maintenance mode:', err);
  }
}

async function exitMaintenanceMode() {
  try {
    const res = await fetch('/api/maintenance/exit', { method: 'POST' });
    const data = await res.json();
    logSystem(`[Maintenance] Exited: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to exit maintenance mode:', err);
  }
}

async function stepMaintenanceOutput(value) {
  try {
    const res = await fetch('/api/maintenance/set_output', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ speeds: [value, value, value, value] })
    });
    const data = await res.json();
    logSystem(`[Maintenance] Step output set to ${value}: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to step maintenance output:', err);
  }
}

async function stopMaintenanceOutput() {
  try {
    const res = await fetch('/api/maintenance/set_output', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ speeds: [0, 0, 0, 0] })
    });
    const data = await res.json();
    logSystem(`[Maintenance] Stopped: ${JSON.stringify(data)}`);
  } catch (err) {
    console.error('Failed to stop maintenance output:', err);
  }
}

// ── Initialization of Listeners ──
function initCalibrationListeners() {
  const t1 = document.getElementById('cal-dist-trial1');
  const t2 = document.getElementById('cal-dist-trial2');
  if (t1) t1.addEventListener('input', onDistanceTrialChange);
  if (t2) t2.addEventListener('input', onDistanceTrialChange);

  const rCw = document.getElementById('cal-rot-cw-angle');
  const rCcw = document.getElementById('cal-rot-ccw-angle');
  if (rCw) rCw.addEventListener('input', onRotationTrialChange);
  if (rCcw) rCcw.addEventListener('input', onRotationTrialChange);
  
  const surfaceSelect = document.getElementById('cal-surface-type');
  if (surfaceSelect) {
    const saved = localStorage.getItem('cal_surface_type');
    if (saved) surfaceSelect.value = saved;
  }
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', () => {
    initCalibrationListeners();
    initLidarStraightLineTest();
  });
} else {
  initCalibrationListeners();
  initLidarStraightLineTest();
}

// ==============================================================================
// Real-time Clock Update
// ==============================================================================
function updateTimeBadge() {
  const timeStatus = document.getElementById('time-status');
  if (timeStatus) {
    const now = new Date();
    timeStatus.textContent = `Time: ${now.toLocaleTimeString()}`;
  }
}
setInterval(updateTimeBadge, 1000);
updateTimeBadge();

// ==============================================================================
// Gamepad Integration
// ==============================================================================
let gamepadIndex = null;
let gamepadActive = false;
let lastSentJoystick = { x: 0, y: 0, deadman: false };
let lastGamepadSendTime = 0;

window.addEventListener('gamepadconnected', (e) => {
  logSystem(`Gamepad connected: ${e.gamepad.id} at index ${e.gamepad.index}`);
  gamepadIndex = e.gamepad.index;
  gamepadActive = true;
  updateGamepadBadge(true, e.gamepad.id);
  startGamepadLoop();
});

window.addEventListener('gamepaddisconnected', (e) => {
  if (gamepadIndex === e.gamepad.index) {
    logSystem(`Gamepad disconnected: ${e.gamepad.id}`);
    gamepadIndex = null;
    gamepadActive = false;
    updateGamepadBadge(false);
    // Reset HUD
    updateGamepadHUD(0, 0, false, "None");
    // Send safety stop
    sendServerMessage({ type: 'joystick', x: 0, y: 0, deadman: false });
  }
});

function updateGamepadBadge(connected, name = '') {
  const gpStatus = document.getElementById('gamepad-status');
  if (gpStatus) {
    if (connected) {
      updateBadge(gpStatus, 'ok', `Gamepad: Connected (${name.substring(0, 12)}...)`);
    } else {
      updateBadge(gpStatus, 'off', 'Gamepad: Disconnected');
    }
  }
}

function updateGamepadHUD(x, y, deadman, pressedButtonsStr) {
  const elDeadman = document.getElementById('gp-live-deadman');
  if (elDeadman) {
    elDeadman.innerText = deadman ? 'ACTIVE' : 'RELEASED';
    elDeadman.style.color = deadman ? '#10b981' : '#ef4444';
  }
  const elLinear = document.getElementById('gp-live-linear');
  if (elLinear) {
    elLinear.innerText = y.toFixed(2);
  }
  const elAngular = document.getElementById('gp-live-angular');
  if (elAngular) {
    elAngular.innerText = x.toFixed(2);
  }
  const elButtons = document.getElementById('gp-live-buttons');
  if (elButtons) {
    elButtons.innerText = pressedButtonsStr || 'None';
  }
  const elStop = document.getElementById('gp-live-stop');
  if (elStop) {
    const isMoving = Math.abs(x) > 0.05 || Math.abs(y) > 0.05;
    elStop.innerText = isMoving ? 'MOVING' : 'STATIONARY';
    elStop.style.color = isMoving ? '#f59e0b' : '#10b981';
  }
}

function startGamepadLoop() {
  function poll() {
    if (!gamepadActive || gamepadIndex === null) return;
    const gp = navigator.getGamepads()[gamepadIndex];
    if (!gp) {
      requestAnimationFrame(poll);
      return;
    }

    // Read axes: Left stick vertical (1) and horizontal (0)
    // Gamepad axes: -1 is up/left, 1 is down/right.
    // Negate gp.axes[1] so positive is forward (up) and negative is reverse (down).
    let throttle = -gp.axes[1];
    let turn = gp.axes[0];

    // Apply deadzone
    if (Math.abs(throttle) < 0.1) throttle = 0;
    if (Math.abs(turn) < 0.1) turn = 0;

    // Detect buttons
    const pressedButtons = [];
    gp.buttons.forEach((btn, idx) => {
      if (btn.pressed) {
        pressedButtons.push(idx);
      }
    });

    // Translate buttons to name tags
    const buttonNames = [];
    pressedButtons.forEach(btnIdx => {
      if (btnIdx === 0) buttonNames.push("A (ESTOP)");
      else if (btnIdx === 1) buttonNames.push("B (ESTOP)");
      else if (btnIdx === 5) buttonNames.push("RB (Deadman)");
      else if (btnIdx === 7) buttonNames.push("RT (Deadman)");
      else if (btnIdx === 8) buttonNames.push("Select (Disarm)");
      else if (btnIdx === 9) buttonNames.push("Start (Arm)");
      else buttonNames.push(btnIdx);
    });
    const pressedButtonsStr = buttonNames.length > 0 ? buttonNames.join(", ") : "None";

    // Deadman switch: Hold RB (5) or RT (7)
    const deadmanPressed = gp.buttons[5].pressed || gp.buttons[7].pressed;

    // Safety buttons: A (0) or B (1) triggers ESTOP
    const estopPressed = gp.buttons[0].pressed || gp.buttons[1].pressed;

    // Arm/Disarm triggers
    const armPressed = gp.buttons[9].pressed;      // Start button
    const disarmPressed = gp.buttons[8].pressed;   // Select button

    if (estopPressed) {
      triggerEstop();
      updateGamepadHUD(0, 0, deadmanPressed, pressedButtonsStr);
      lastSentJoystick = { x: 0, y: 0, deadman: deadmanPressed };
      lastGamepadSendTime = Date.now();
    } else if (armPressed) {
      armNormalDrive();
      lastGamepadSendTime = Date.now() + 500; // Debounce arming
    } else if (disarmPressed) {
      disarmNormalDrive();
      lastGamepadSendTime = Date.now() + 500; // Debounce disarming
    }

    // Sync with HUD
    updateGamepadHUD(turn, throttle, deadmanPressed, pressedButtonsStr);

    // Send joystick commands to server
    if (!estopPressed && !armPressed && !disarmPressed) {
      const now = Date.now();
      const changed = turn !== lastSentJoystick.x || throttle !== lastSentJoystick.y || deadmanPressed !== lastSentJoystick.deadman;
      const timeElapsed = now - lastGamepadSendTime > 100;

      if (changed || (timeElapsed && (turn !== 0 || throttle !== 0 || deadmanPressed))) {
        sendServerMessage({
          type: 'joystick',
          x: turn,
          y: throttle,
          deadman: deadmanPressed
        });
        lastSentJoystick = { x: turn, y: throttle, deadman: deadmanPressed };
        lastGamepadSendTime = now;
      }
    }

    requestAnimationFrame(poll);
  }
  requestAnimationFrame(poll);
}

// ==============================================================================
// LiDAR Straight-Line Correction & Calibration Setup
// ==============================================================================
function initLidarStraightLineTest() {
  const chkRigid = document.getElementById('chk-rigid-mount');
  const chkLevel = document.getElementById('chk-level-mount');
  const btnStartWizard = document.getElementById('btn-start-wizard');
  const btnWizardYes = document.getElementById('btn-wizard-yes');
  const btnWizardCancel = document.getElementById('btn-wizard-cancel');
  
  const btnStartLidar = document.getElementById('btn-start-lidar-test');
  const btnStopLidar = document.getElementById('btn-stop-lidar-test');
  const btnApplyProposed = document.getElementById('btn-apply-proposed');
  const btnRollbackProposed = document.getElementById('btn-rollback-proposed');
  const btnResetTrims = document.getElementById('btn-reset-trims');

  const selectTrackWidth = document.getElementById('monitored-track-width');
  if (selectTrackWidth) {
    const savedWidth = localStorage.getItem('monitored_track_width');
    if (savedWidth) {
      monitoredTrackWidth = parseFloat(savedWidth);
      selectTrackWidth.value = savedWidth;
    } else {
      monitoredTrackWidth = parseFloat(selectTrackWidth.value);
    }
    selectTrackWidth.addEventListener('change', () => {
      monitoredTrackWidth = parseFloat(selectTrackWidth.value);
      localStorage.setItem('monitored_track_width', selectTrackWidth.value);
      drawLidarTestCanvas();
    });
  }


  function checkGates() {
    const rigid = chkRigid ? chkRigid.checked : false;
    const level = chkLevel ? chkLevel.checked : false;
    const btnStart = document.getElementById('btn-start-lidar-test');
    if (btnStart) {
      if (rigid && level && orientationVerified) {
        btnStart.disabled = false;
        btnStart.style.opacity = '1';
        btnStart.style.cursor = 'pointer';
      } else {
        btnStart.disabled = true;
        btnStart.style.opacity = '0.6';
        btnStart.style.cursor = 'not-allowed';
      }
    }
  }

  if (chkRigid) chkRigid.addEventListener('change', checkGates);
  if (chkLevel) chkLevel.addEventListener('change', checkGates);

  if (btnStartWizard) {
    btnStartWizard.addEventListener('click', () => {
      orientationStep = 1;
      document.getElementById('orientation-wizard-box').style.display = 'flex';
      runWizardStep();
    });
  }

  if (btnWizardYes) {
    btnWizardYes.addEventListener('click', () => {
      orientationStep++;
      if (orientationStep > 4) {
        orientationVerified = true;
        localStorage.setItem('lidar_orientation_verified', 'true');
        document.getElementById('orientation-wizard-box').style.display = 'none';
        stopWizardPolling();
        
        const badge = document.getElementById('orientation-verified-badge');
        if (badge) {
          badge.textContent = 'Verified';
          badge.style.background = 'rgba(16, 185, 129, 0.15)';
          badge.style.color = '#10b981';
          badge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
        }
        
        logSystem("✅ Coordinate orientation verified successfully via flat target checks.");
        checkGates();
      } else {
        runWizardStep();
      }
    });
  }

  if (btnWizardCancel) {
    btnWizardCancel.addEventListener('click', () => {
      document.getElementById('orientation-wizard-box').style.display = 'none';
      stopWizardPolling();
    });
  }

  if (btnStartLidar) {
    btnStartLidar.addEventListener('click', () => {
      // Clear path data for fresh test
      lidarOdomPath = [];
      lidarPosePath = [];
      calibPathHistory = []; // Clear historical paths for new run
      
      const frontAngleOffset = parseFloat(document.getElementById('cfg-front-angle-offset').value || 0);
      const lidarXOffset = parseFloat(document.getElementById('cfg-lidar-x').value || 0.0127);
      const lidarYOffset = parseFloat(document.getElementById('cfg-lidar-y').value || 0.034925);
      const maxRange = parseFloat(document.getElementById('cfg-max-range').value || 4.0);
      const minConfidence = parseFloat(document.getElementById('cfg-min-confidence').value || 0.65);
      const headingGain = parseFloat(document.getElementById('cfg-heading-gain').value || 0.80);
      const lateralGain = parseFloat(document.getElementById('cfg-lateral-gain').value || 1.20);
      const maxAngularCorr = parseFloat(document.getElementById('cfg-max-steering-corr').value || 0.35);
      const corrSlewRate = parseFloat(document.getElementById('cfg-slew-rate').value || 1.0);
      const angleSectorMasks = document.getElementById('cfg-sector-masks').value || '';

      sendServerMessage({
        type: 'start_lidar_test',
        frontAngleOffset,
        lidarXOffset,
        lidarYOffset,
        maxRange,
        minConfidence,
        headingGain,
        lateralGain,
        maxAngularCorr,
        corrSlewRate,
        angleSectorMasks
      });
      startTestScanPolling();
    });
  }

  if (btnStopLidar) {
    btnStopLidar.addEventListener('click', () => {
      sendServerMessage({ type: 'stop_lidar_test' });
    });
  }

  if (btnApplyProposed) {
    btnApplyProposed.addEventListener('click', () => {
      sendServerMessage({ type: 'apply_proposed_trims' });
    });
  }

  if (btnRollbackProposed) {
    btnRollbackProposed.addEventListener('click', () => {
      sendServerMessage({ type: 'rollback_trims' });
    });
  }

  if (btnResetTrims) {
    btnResetTrims.addEventListener('click', () => {
      sendServerMessage({ type: 'reset_trims' });
    });
  }
  
  if (orientationVerified) {
    const badge = document.getElementById('orientation-verified-badge');
    if (badge) {
      badge.textContent = 'Verified';
      badge.style.background = 'rgba(16, 185, 129, 0.15)';
      badge.style.color = '#10b981';
      badge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
    }
  }

  // Register speed tier toggle checkboxes to trigger redrawing the canvas
  const chkSlow = document.getElementById('chk-toggle-slow');
  const chkMed = document.getElementById('chk-toggle-med');
  const chkFast = document.getElementById('chk-toggle-fast');
  [chkSlow, chkMed, chkFast].forEach(chk => {
    if (chk) {
      chk.addEventListener('change', drawLidarTestCanvas);
    }
  });

  // Register floor testing limits toggle checkbox
  const limitsChk = document.getElementById('limits-floor-testing');
  if (limitsChk) {
    limitsChk.addEventListener('change', () => {
      fetch('/api/drive/limits', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ floorTesting: limitsChk.checked })
      })
      .catch(err => console.error('Failed to update floor testing limits:', err));
    });
  }

  checkGates();
  
  // Initial draw of empty canvas
  drawLidarTestCanvas();
}

function runWizardStep() {
  const textDiv = document.getElementById('wizard-step-text');
  if (!textDiv) return;
  
  if (orientationStep === 1) {
    textDiv.textContent = 'Step 1: Place a flat object exactly in front of the rover (0°).';
  } else if (orientationStep === 2) {
    textDiv.textContent = 'Step 2: Place a flat object exactly to the left of the rover (90°).';
  } else if (orientationStep === 3) {
    textDiv.textContent = 'Step 3: Place a flat object exactly behind the rover (180°).';
  } else if (orientationStep === 4) {
    textDiv.textContent = 'Step 4: Place a flat object exactly to the right of the rover (270°).';
  }
  
  startWizardPolling();
}

function startWizardPolling() {
  if (wizardPollInterval) clearInterval(wizardPollInterval);
  
  wizardPollInterval = setInterval(() => {
    let targetAngle = 0;
    if (orientationStep === 1) targetAngle = 0;
    else if (orientationStep === 2) targetAngle = 90;
    else if (orientationStep === 3) targetAngle = 180;
    else if (orientationStep === 4) targetAngle = 270;
    
    fetch('/api/lidar/scan')
      .then(res => res.json())
      .then(data => {
        if (data && data.points) {
          let minDist = 999.0;
          data.points.forEach(p => {
            let diff = Math.abs(p.angleDeg - targetAngle);
            if (diff > 180) diff = 360 - diff;
            if (diff <= 15) {
              const distM = p.distanceMm / 1000.0;
              if (distM < minDist) minDist = distM;
            }
          });
          
          const rangeSpan = document.getElementById('wizard-live-range');
          if (rangeSpan) {
            if (minDist < 10.0) {
              rangeSpan.textContent = `Live distance at ${targetAngle}°: ${minDist.toFixed(3)}m`;
            } else {
              rangeSpan.textContent = `Live distance at ${targetAngle}°: No point detected`;
            }
          }
        }
      })
      .catch(err => {
        console.error('Wizard scan poll failed:', err);
      });
  }, 300);
}

function stopWizardPolling() {
  if (wizardPollInterval) {
    clearInterval(wizardPollInterval);
    wizardPollInterval = null;
  }
}

const lidarCanvas = document.getElementById('lidar-path-canvas');
const lidarCtx = lidarCanvas ? lidarCanvas.getContext('2d') : null;

function drawLidarTestCanvas() {
  if (!lidarCanvas || !lidarCtx) return;
  
  lidarCtx.fillStyle = '#0b0f19';
  lidarCtx.fillRect(0, 0, lidarCanvas.width, lidarCanvas.height);
  
  // 1. Draw Monitored Track Width Corridor (grid/shaded corridor)
  const pyLeft = 75 - (monitoredTrackWidth / 2) * 366.6;
  const pyRight = 75 + (monitoredTrackWidth / 2) * 366.6;
  
  lidarCtx.fillStyle = 'rgba(0, 240, 255, 0.025)';
  lidarCtx.fillRect(40, pyLeft, 0.9144 * 400, monitoredTrackWidth * 366.6);
  
  lidarCtx.strokeStyle = 'rgba(0, 240, 255, 0.12)';
  lidarCtx.lineWidth = 1;
  lidarCtx.beginPath();
  lidarCtx.moveTo(40, pyLeft);
  lidarCtx.lineTo(40 + 0.9144 * 400, pyLeft);
  lidarCtx.moveTo(40, pyRight);
  lidarCtx.lineTo(40 + 0.9144 * 400, pyRight);
  lidarCtx.stroke();

  // Draw centerline and corridor bounds
  lidarCtx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
  lidarCtx.lineWidth = 1;
  lidarCtx.setLineDash([5, 5]);
  lidarCtx.beginPath();
  lidarCtx.moveTo(0, 75);
  lidarCtx.lineTo(lidarCanvas.width, 75);
  lidarCtx.stroke();
  lidarCtx.setLineDash([]);
  
  // Draw distance grid
  lidarCtx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  for (let x = 0.0; x <= 1.0; x += 0.25) {
    const px = 40 + x * 400;
    lidarCtx.beginPath();
    lidarCtx.moveTo(px, 0);
    lidarCtx.lineTo(px, lidarCanvas.height);
    lidarCtx.stroke();
    
    lidarCtx.fillStyle = 'rgba(255, 255, 255, 0.3)';
    lidarCtx.font = '8px monospace';
    lidarCtx.fillText((x * 39.37).toFixed(0) + '"', px + 2, 145);
  }
  
  // 5cm error corridor markers
  lidarCtx.strokeStyle = 'rgba(239, 68, 68, 0.08)';
  lidarCtx.beginPath();
  lidarCtx.moveTo(0, 75 - 0.05 * 366.6);
  lidarCtx.lineTo(lidarCanvas.width, 75 - 0.05 * 366.6);
  lidarCtx.moveTo(0, 75 + 0.05 * 366.6);
  lidarCtx.lineTo(lidarCanvas.width, 75 + 0.05 * 366.6);
  lidarCtx.stroke();
  
  // Start line (X = 0)
  lidarCtx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
  lidarCtx.lineWidth = 2;
  lidarCtx.beginPath();
  lidarCtx.moveTo(40, 15);
  lidarCtx.lineTo(40, 135);
  lidarCtx.stroke();
  
  // Target line (X = 3ft)
  const targetPx = 40 + 0.9144 * 400;
  lidarCtx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
  lidarCtx.beginPath();
  lidarCtx.moveTo(targetPx, 15);
  lidarCtx.lineTo(targetPx, 135);
  lidarCtx.stroke();
  
  // Color maps and checkbox states for speed tiers
  const tierColors = {
    SLOW: {
      lidar: '#10b981',  // Emerald Green
      odom: '#059669'   // Darker Green
    },
    MED: {
      lidar: '#00f0ff',   // Electric Cyan
      odom: '#0284c7'    // Sky Blue
    },
    FAST: {
      lidar: '#f59e0b',  // Neon Amber/Orange
      odom: '#d97706'   // Darker Orange
    }
  };

  const showSlow = document.getElementById('chk-toggle-slow') ? document.getElementById('chk-toggle-slow').checked : true;
  const showMed = document.getElementById('chk-toggle-med') ? document.getElementById('chk-toggle-med').checked : true;
  const showFast = document.getElementById('chk-toggle-fast') ? document.getElementById('chk-toggle-fast').checked : true;
  
  const showTier = {
    SLOW: showSlow,
    MED: showMed,
    FAST: showFast
  };

  // 2. Draw Historical Paths
  calibPathHistory.forEach(path => {
    const tier = path.tier || 'SLOW';
    if (!showTier[tier]) return;
    
    const colors = tierColors[tier];
    
    // Draw historical odom path (dashed)
    if (path.odom && path.odom.length > 0) {
      lidarCtx.save();
      lidarCtx.strokeStyle = colors.odom;
      lidarCtx.lineWidth = 1.2;
      lidarCtx.setLineDash([3, 3]);
      lidarCtx.beginPath();
      let first = true;
      path.odom.forEach(pt => {
        const px = 40 + pt.x * 400;
        const py = 75 - pt.y * 366.6;
        if (first) {
          lidarCtx.moveTo(px, py);
          first = false;
        } else {
          lidarCtx.lineTo(px, py);
        }
      });
      lidarCtx.stroke();
      lidarCtx.restore();
    }
    
    // Draw historical lidar path (solid)
    if (path.lidar && path.lidar.length > 0) {
      lidarCtx.save();
      lidarCtx.strokeStyle = colors.lidar;
      lidarCtx.lineWidth = 1.2;
      lidarCtx.beginPath();
      let first = true;
      path.lidar.forEach(pt => {
        const px = 40 + pt.x * 400;
        const py = 75 - pt.y * 366.6;
        if (first) {
          lidarCtx.moveTo(px, py);
          first = false;
        } else {
          lidarCtx.lineTo(px, py);
        }
      });
      lidarCtx.stroke();
      lidarCtx.restore();
    }
  });

  // 3. Draw Current Active Paths
  const activeTier = lastTelemetrySpeedTier || 'SLOW';
  if (showTier[activeTier]) {
    const colors = tierColors[activeTier];
    
    // Draw current active odom path (dashed)
    if (lidarOdomPath.length > 0) {
      lidarCtx.save();
      lidarCtx.strokeStyle = colors.odom;
      lidarCtx.lineWidth = 2.2;
      lidarCtx.setLineDash([4, 4]);
      lidarCtx.beginPath();
      let first = true;
      lidarOdomPath.forEach(pt => {
        const px = 40 + pt.x * 400;
        const py = 75 - pt.y * 366.6;
        if (first) {
          lidarCtx.moveTo(px, py);
          first = false;
        } else {
          lidarCtx.lineTo(px, py);
        }
      });
      lidarCtx.stroke();
      lidarCtx.restore();
    }
    
    // Draw current active lidar path (solid)
    if (lidarPosePath.length > 0) {
      lidarCtx.save();
      lidarCtx.strokeStyle = colors.lidar;
      lidarCtx.lineWidth = 2.2;
      lidarCtx.beginPath();
      let first = true;
      lidarPosePath.forEach(pt => {
        const px = 40 + pt.x * 400;
        const py = 75 - pt.y * 366.6;
        if (first) {
          lidarCtx.moveTo(px, py);
          first = false;
        } else {
          lidarCtx.lineTo(px, py);
        }
      });
      lidarCtx.stroke();
      lidarCtx.restore();
    }
  }
  
  // 4. Draw Raw LiDAR Scan Points projected in Track Frame
  if (lastLidarScanForTest && lastLidarScanForTest.points && lastLidarScanForTest.points.length > 0) {
    const latestPose = lidarPosePath.length > 0 ? lidarPosePath[lidarPosePath.length - 1] : { x: 0, y: 0, yaw: 0 };
    
    lidarCtx.fillStyle = 'rgba(6, 182, 212, 0.4)'; // glowing cyan with transparency
    lastLidarScanForTest.points.forEach(pt => {
      const angleRad = (pt.angleDeg - 90) * Math.PI / 180;
      const distM = pt.distanceMm / 1000.0;
      
      // Laser point in LiDAR local frame
      const x_lidar = distM * Math.cos(angleRad);
      const y_lidar = - distM * Math.sin(angleRad); // Negated for right-handed mapping
      
      // LiDAR to chassis frame translation
      const x_chassis = x_lidar + (parseFloat(document.getElementById('cfg-lidar-x')?.value) || 0.0127);
      const y_chassis = y_lidar + (parseFloat(document.getElementById('cfg-lidar-y')?.value) || 0.034925);
      
      // Chassis to track frame translation and rotation
      const x_track = latestPose.x + x_chassis * Math.cos(latestPose.yaw) - y_chassis * Math.sin(latestPose.yaw);
      const y_track = latestPose.y + x_chassis * Math.sin(latestPose.yaw) + y_chassis * Math.cos(latestPose.yaw);
      
      // Project to track canvas
      const px = 40 + x_track * 400;
      const py = 75 - y_track * 366.6;
      
      if (px >= 0 && px <= lidarCanvas.width && py >= 0 && py <= lidarCanvas.height) {
        lidarCtx.fillRect(px - 1, py - 1, 2, 2);
      }
    });
  }

  // Draw obstacles
  function drawObstaclePoint(pt, label, color) {
    if (!pt) return;
    const px = 40 + pt.x * 400;
    const py = 75 - pt.y * 366.6;
    
    if (px >= 0 && px <= lidarCanvas.width && py >= 0 && py <= lidarCanvas.height) {
      lidarCtx.save();
      lidarCtx.strokeStyle = color;
      lidarCtx.shadowBlur = 6;
      lidarCtx.shadowColor = color;
      lidarCtx.lineWidth = 1.5;
      
      lidarCtx.beginPath();
      lidarCtx.arc(px, py, 4, 0, 2 * Math.PI);
      lidarCtx.stroke();
      
      lidarCtx.fillStyle = color;
      lidarCtx.beginPath();
      lidarCtx.arc(px, py, 2, 0, 2 * Math.PI);
      lidarCtx.fill();
      
      lidarCtx.shadowBlur = 0;
      lidarCtx.fillStyle = '#ffffff';
      lidarCtx.font = 'bold 7px monospace';
      lidarCtx.fillText(label, px + 6, py - 3);
      lidarCtx.restore();
    }
  }
  
  drawObstaclePoint(closestFrontObstacle, 'FRONT', '#ff0055');
  drawObstaclePoint(closestLeftObstacle, 'LEFT', closestLeftObstacle && closestLeftObstacle.dist < 0 ? '#ff0055' : '#f59e0b');
  drawObstaclePoint(closestRightObstacle, 'RIGHT', closestRightObstacle && closestRightObstacle.dist < 0 ? '#ff0055' : '#f59e0b');

  // Draw top-down rover triangle sprite
  const latest = lidarPosePath.length > 0 ? lidarPosePath[lidarPosePath.length - 1] : { x: 0, y: 0, yaw: 0 };
  const px = 40 + latest.x * 400;
  const py = 75 - latest.y * 366.6;
  
  lidarCtx.save();
  lidarCtx.translate(px, py);
  lidarCtx.rotate(-latest.yaw);
  
  lidarCtx.fillStyle = 'rgba(0, 240, 255, 0.4)';
  lidarCtx.strokeStyle = '#00f0ff';
  lidarCtx.lineWidth = 1.5;
  lidarCtx.beginPath();
  lidarCtx.moveTo(12, 0);
  lidarCtx.lineTo(-8, -8);
  lidarCtx.lineTo(-8, 8);
  lidarCtx.closePath();
  lidarCtx.fill();
  lidarCtx.stroke();
  
  lidarCtx.restore();
}

function openLowEndCalibration() {
  const tabBtn = document.querySelector('.tab-btn[data-tab="tab-calibrate"]');
  if (tabBtn) {
    tabBtn.click();
    const section = document.getElementById('tab-calibrate');
    if (section) {
      section.scrollIntoView({ behavior: 'smooth' });
    }
  }
}

