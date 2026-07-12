let ws = null;
let reconnectTimer = null;
let reconnectInterval = 1000;
const maxReconnectInterval = 16000;
let driveArmed = false;
let realOdomActive = false;

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
        flLabel.innerText = msg.floorTesting ? 'FLOOR TESTING (0.08 m/s)' : 'UNCLAMPED (0.15 m/s)';
        flLabel.style.color = msg.floorTesting ? '#f59e0b' : '#10b981';
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

    case 'firmware_info':
      logSystem(`[Firmware] Name: ${msg.name} | Ver: ${msg.version} | Protocol: ${msg.protocol} | Source: ${msg.commit} | Build: ${msg.build} | Target: ${msg.target}`);
      break;

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
      y = 0.5;
      x = -0.5;
      break;
    case 'right':
      y = 0.5;
      x = 0.5;
      break;
    case 'spin_left':
      x = -1.0;
      break;
    case 'spin_right':
      x = 1.0;
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
  
  // Set image source to stream endpoint
  cameraStream.src = '/api/camera';
  cameraStream.style.display = 'block';
  if (cameraPlaceholder) cameraPlaceholder.style.display = 'none';
  if (btnToggleCamera) btnToggleCamera.textContent = 'Stop Feed';
  if (btnFullscreenCamera) btnFullscreenCamera.disabled = false;
}

function stopCameraStream() {
  if (!cameraStream) return;
  
  isCameraStreaming = false;
  updateCameraStatus('offline', 'OFFLINE', 'off');
  
  // Clear source to stop request
  cameraStream.src = '';
  cameraStream.style.display = 'none';
  if (cameraPlaceholder) {
    cameraPlaceholder.style.display = 'flex';
    const pText = cameraPlaceholder.querySelector('.placeholder-text');
    const pSub = cameraPlaceholder.querySelector('.placeholder-subtext');
    if (pText) pText.textContent = 'Camera Feed Offline';
    if (pSub) pSub.textContent = 'Click "Start Feed" to initialize the stream';
  }
  if (btnToggleCamera) btnToggleCamera.textContent = 'Start Feed';
  if (btnFullscreenCamera) btnFullscreenCamera.disabled = true;
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

if (cameraStream) {
  cameraStream.addEventListener('load', () => {
    if (isCameraStreaming) {
      updateCameraStatus('streaming', 'STREAMING', 'ok');
    }
  });

  cameraStream.addEventListener('error', () => {
    if (isCameraStreaming) {
      updateCameraStatus('error', 'ERROR', 'alert');
      // Show placeholder with error info
      cameraStream.style.display = 'none';
      if (cameraPlaceholder) {
        cameraPlaceholder.style.display = 'flex';
        const pText = cameraPlaceholder.querySelector('.placeholder-text');
        const pSub = cameraPlaceholder.querySelector('.placeholder-subtext');
        if (pText) pText.textContent = 'CAMERA ERROR';
        if (pSub) pSub.textContent = 'Failed to load video stream. Check connection and physical camera.';
      }
    }
  });
}

if (btnFullscreenCamera && cameraViewport) {
  btnFullscreenCamera.addEventListener('click', () => {
    if (!document.fullscreenElement) {
      cameraViewport.requestFullscreen().catch(err => {
        console.error(`Error attempting to enable full-screen mode: ${err.message}`);
      });
    } else {
      document.exitFullscreen();
    }
  });
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
let lastSentSpeeds = [0, 0, 0, 0];
let lastSpeedSendTime = 0;

window.addEventListener('gamepadconnected', (e) => {
  logSystem(`Gamepad connected: ${e.gamepad.id} at index ${e.gamepad.index}`);
  gamepadIndex = e.gamepad.index;
  gamepadActive = true;
  updateGamepadStatus(true, e.gamepad.id);
  startGamepadLoop();
});

window.addEventListener('gamepaddisconnected', (e) => {
  if (gamepadIndex === e.gamepad.index) {
    logSystem(`Gamepad disconnected: ${e.gamepad.id}`);
    gamepadIndex = null;
    gamepadActive = false;
    updateGamepadStatus(false);
    
    // Reset Live Input HUD
    const liveDeadman = document.getElementById('gp-live-deadman');
    if (liveDeadman) {
      liveDeadman.innerText = 'RELEASED';
      liveDeadman.style.color = '#ef4444';
    }
    const liveLinear = document.getElementById('gp-live-linear');
    if (liveLinear) liveLinear.innerText = '0.00';
    const liveAngular = document.getElementById('gp-live-angular');
    if (liveAngular) liveAngular.innerText = '0.00';
    const liveStop = document.getElementById('gp-live-stop');
    if (liveStop) {
      liveStop.innerText = 'STATIONARY';
      liveStop.style.color = '#10b981';
    }
    const liveBtns = document.getElementById('gp-live-buttons');
    if (liveBtns) {
      liveBtns.innerText = 'None';
    }
    
    // Enforce safety stop
    sendMotorSpeeds(0, 0, 0, 0);
  }
});

function updateGamepadStatus(connected, name = '') {
  const gpStatus = document.getElementById('gamepad-status');
  if (gpStatus) {
    if (connected) {
      updateBadge(gpStatus, 'ok', `Gamepad: Connected (${name.substring(0, 12)}...)`);
    } else {
      updateBadge(gpStatus, 'alert', 'Gamepad: Disconnected');
    }
  }
}

let lastStartPressed = false;
let lastSelectPressed = false;

function startGamepadLoop() {
  function poll() {
    if (!gamepadActive || gamepadIndex === null) return;
    const gp = navigator.getGamepads()[gamepadIndex];
    if (!gp) {
      requestAnimationFrame(poll);
      return;
    }

    // Safety: button A (0) or button B (1) triggers E-STOP
    if (gp.buttons[0].pressed || gp.buttons[1].pressed) {
      triggerEstop();
      requestAnimationFrame(poll);
      return;
    }

    // Start (9) arms, Select (8) disarms normal drive
    const startPressed = gp.buttons[9].pressed;
    const selectPressed = gp.buttons[8].pressed;

    if (startPressed && !lastStartPressed) {
      armNormalDrive();
    }
    if (selectPressed && !lastSelectPressed) {
      disarmNormalDrive();
    }

    lastStartPressed = startPressed;
    lastSelectPressed = selectPressed;

    let throttle = -gp.axes[1];
    let turn = gp.axes[0];

    // Apply deadzone of 0.10
    if (Math.abs(throttle) < 0.10) throttle = 0;
    if (Math.abs(turn) < 0.10) turn = 0;

    // Deadman button (Right Bumper = 5, Right Trigger = 7)
    const deadmanPressed = gp.buttons[5].pressed || gp.buttons[7].pressed;

    // Update Live Input HUD
    const liveDeadman = document.getElementById('gp-live-deadman');
    if (liveDeadman) {
      liveDeadman.innerText = deadmanPressed ? 'HELD' : 'RELEASED';
      liveDeadman.style.color = deadmanPressed ? '#10b981' : '#ef4444';
    }
    const liveLinear = document.getElementById('gp-live-linear');
    if (liveLinear) {
      liveLinear.innerText = throttle.toFixed(2);
    }
    const liveAngular = document.getElementById('gp-live-angular');
    if (liveAngular) {
      liveAngular.innerText = turn.toFixed(2);
    }
    const liveStop = document.getElementById('gp-live-stop');
    if (liveStop) {
      const isStopped = Math.abs(throttle) < 0.01 && Math.abs(turn) < 0.01;
      liveStop.innerText = isStopped ? 'STATIONARY' : 'MOVING';
      liveStop.style.color = isStopped ? '#10b981' : '#f59e0b';
    }
    const liveBtns = document.getElementById('gp-live-buttons');
    if (liveBtns) {
      const pressed = [];
      gp.buttons.forEach((btn, idx) => {
        if (btn.pressed || btn.value > 0.1) {
          pressed.push(idx);
        }
      });
      liveBtns.innerText = pressed.length > 0 ? pressed.join(', ') : 'None';
    }

    // Stream coordinates and deadman state
    sendServerMessage({ type: 'joystick', x: turn, y: throttle, deadman: deadmanPressed });

    requestAnimationFrame(poll);
  }
  requestAnimationFrame(poll);
}

// ==============================================================================
// Breakaway Calibration Actions
// ==============================================================================
async function triggerCalibrateStart() {
  logSystem("Requesting breakaway calibration simulation to start...");
  try {
    const res = await fetch('/api/calibration/simulate/start', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ safetyAck: true })
    });
    const data = await res.json();
    if (res.ok) {
      logSystem(`[Calibration] Simulation started. Session ID: ${data.sessionId}`);
    } else {
      logSystem(`[Calibration] Start failed: ${data.error}`);
    }
  } catch (err) {
    logSystem(`[Calibration] Start failed: ${err.message}`);
  }
}

async function triggerCalibrateCancel() {
  logSystem("Aborting breakaway calibration simulation...");
  try {
    const res = await fetch('/api/calibration/abort', {
      method: 'POST'
    });
    const data = await res.json();
    logSystem(`[Calibration] Abort response: ${data.message || 'success'}`);
  } catch (err) {
    logSystem(`[Calibration] Abort failed: ${err.message}`);
  }
}

async function triggerClearFaults() {
  logSystem("Clearing safety faults...");
  try {
    const res = await fetch('/api/faults/clear');
    const data = await res.json();
    logSystem(`[Safety] Clear faults response: ${data.message || 'success'}`);
  } catch (err) {
    logSystem(`[Safety] Clear faults failed: ${err.message}`);
  }
}

// --- Pre-calibration safety check-boxes initialization ---
function initSafetyChecks() {
  const chks = document.querySelectorAll('.safety-ack-chk');
  const startBtn = document.getElementById('btn-start-calibration');
  if (!startBtn) return;
  
  function updateStartButtonState() {
    const allChecked = Array.from(chks).every(chk => chk.checked);
    if (allChecked) {
      startBtn.disabled = false;
      startBtn.style.background = 'linear-gradient(135deg, #a855f7 0%, #06b6d4 100%)';
      startBtn.style.color = '#fff';
      startBtn.style.cursor = 'pointer';
    } else {
      startBtn.disabled = true;
      startBtn.style.background = '#4b5563';
      startBtn.style.color = '#9ca3af';
      startBtn.style.cursor = 'not-allowed';
    }
  }
  
  chks.forEach(chk => {
    chk.addEventListener('change', updateStartButtonState);
  });
}
initSafetyChecks();

// ==============================================================================
// Phase 3 — Raised-Wheel Maintenance Mode Actions
// ==============================================================================
let maintSessionActive = false;
let maintSessionId = 0;
let maintMotorIndex = 0;
let maintDeadmanTimer = null;

function initMaintenanceSafetyChecks() {
  const safetyChk = document.getElementById('maint-safety-chk');
  const btnEnterMaint = document.getElementById('btn-enter-maint');
  const slider = document.getElementById('maint-pwm-slider');
  const display = document.getElementById('maint-pwm-display');

  if (safetyChk && btnEnterMaint) {
    safetyChk.addEventListener('change', () => {
      if (safetyChk.checked) {
        btnEnterMaint.disabled = false;
        btnEnterMaint.style.background = 'var(--primary)';
        btnEnterMaint.style.color = '#fff';
        btnEnterMaint.style.cursor = 'pointer';
      } else {
        btnEnterMaint.disabled = true;
        btnEnterMaint.style.background = '#4b5563';
        btnEnterMaint.style.color = '#9ca3af';
        btnEnterMaint.style.cursor = 'not-allowed';
      }
    });
  }

  if (slider && display) {
    slider.addEventListener('input', () => {
      display.innerText = slider.value;
    });
  }
}
initMaintenanceSafetyChecks();

function enterMaintenanceMode() {
  const motorSelect = document.getElementById('maint-motor-select');
  const motorIdx = parseInt(motorSelect.value);
  const safetyChk = document.getElementById('maint-safety-chk');
  
  if (!safetyChk || !safetyChk.checked) {
    alert('Please verify and check the safety checkbox first.');
    return;
  }
  
  maintSessionId = Math.floor(Math.random() * 1000000) + 1;
  maintMotorIndex = motorIdx;
  
  fetch('/api/maintenance/enter', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      safetyAck: true,
      motorIndex: motorIdx,
      sessionId: maintSessionId
    })
  })
  .then(r => r.json())
  .then(res => {
    if (res.ok) {
      logSystem(`[Maintenance] Command sent to enter maintenance session ${maintSessionId}`);
      maintSessionActive = true;
      startMaintenanceDeadmanLoop();
    } else {
      logSerialOutErr(`Failed to enter maintenance: ${res.error}`);
    }
  })
  .catch(err => {
    logSerialOutErr(`HTTP error entering maintenance: ${err.message}`);
  });
}

function exitMaintenanceMode() {
  fetch('/api/maintenance/exit', { method: 'POST' })
  .then(r => r.json())
  .then(res => {
    if (res.ok) {
      logSystem('[Maintenance] Exit command sent.');
      stopMaintenanceDeadmanLoop();
      maintSessionActive = false;
    }
  })
  .catch(err => logSerialOutErr(`Error exiting maintenance: ${err.message}`));
}

function stepMaintenanceOutput(step) {
  const slider = document.getElementById('maint-pwm-slider');
  const display = document.getElementById('maint-pwm-display');
  if (slider && display) {
    let val = parseInt(slider.value) + step;
    val = Math.max(-80, Math.min(80, val));
    slider.value = val;
    display.innerText = val;
  }
}

function stopMaintenanceOutput() {
  const slider = document.getElementById('maint-pwm-slider');
  const display = document.getElementById('maint-pwm-display');
  if (slider && display) {
    slider.value = 0;
    display.innerText = 0;
    sendMaintenanceDeadmanPacket();
  }
}

function sendMaintenanceDeadmanPacket() {
  if (!maintSessionActive) return;
  const slider = document.getElementById('maint-pwm-slider');
  if (!slider) return;
  
  const val = parseInt(slider.value);
  const dir = (val >= 0) ? 0 : 1;
  const output = Math.abs(val);
  
  fetch('/api/maintenance/set_output', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      sessionId: maintSessionId,
      motorIndex: maintMotorIndex,
      direction: dir,
      output: output,
      enable: 1
    })
  }).catch(err => console.error('Deadman packet send error:', err));
}

function startMaintenanceDeadmanLoop() {
  stopMaintenanceDeadmanLoop();
  maintDeadmanTimer = setInterval(sendMaintenanceDeadmanPacket, 100);
}

function stopMaintenanceDeadmanLoop() {
  if (maintDeadmanTimer) {
    clearInterval(maintDeadmanTimer);
    maintDeadmanTimer = null;
  }
}

// --- Real calibration readiness safety check-boxes initialization ---
function initRealSafetyChecks() {
  const chks = document.querySelectorAll('.gate-ack-chk');
  const startBtn = document.getElementById('btn-start-real-calibration');
  if (!startBtn) return;
  
  function updateStartButtonState() {
    const motorDir = document.getElementById('gate-chk-motor-dir').checked;
    const encDir = document.getElementById('gate-chk-enc-dir').checked;
    const maintStop = document.getElementById('gate-chk-maint-stop').checked;
    const estop = document.getElementById('gate-chk-estop').checked;
    const deadman = document.getElementById('gate-chk-deadman').checked;
    
    const allChecked = motorDir && encDir && maintStop && estop && deadman;
    
    if (allChecked) {
      startBtn.disabled = false;
      startBtn.style.background = 'linear-gradient(135deg, #a855f7 0%, #3b82f6 100%)';
      startBtn.style.color = '#fff';
      startBtn.style.cursor = 'pointer';
    } else {
      startBtn.disabled = true;
      startBtn.style.background = '#4b5563';
      startBtn.style.color = '#9ca3af';
      startBtn.style.cursor = 'not-allowed';
    }
    
    // Sync to ESP32
    fetch('/api/calibration/verify_ready', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        motorDirectionsVerified: motorDir,
        encoderDirectionsVerified: encDir,
        maintenanceStopVerified: maintStop,
        emergencyStopVerified: estop,
        deadmanVerified: deadman
      })
    }).catch(err => console.error('Failed to sync readiness gates:', err));
  }
  
  chks.forEach(chk => {
    chk.addEventListener('change', updateStartButtonState);
  });
}
initRealSafetyChecks();

async function triggerRealCalibrateStart() {
  logSystem("Requesting real breakaway calibration to start...");
  try {
    const res = await fetch('/api/calibration/real/start', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ safetyAck: true })
    });
    const data = await res.json();
    if (res.ok) {
      logSystem(`[Calibration] Real calibration started. Session ID: ${data.sessionId}`);
    } else {
      logSystem(`[Calibration] Real start failed: ${data.error}`);
    }
  } catch (err) {
    logSystem(`[Calibration] Real start failed: ${err.message}`);
  }
}

// Phase 4 Path Recording & Backtracking UI Binding
const btnRecordStart = document.getElementById('btn-record-start');
const btnRecordStop = document.getElementById('btn-record-stop');
const btnRecordClear = document.getElementById('btn-record-clear');
const btnBacktrackStart = document.getElementById('btn-backtrack-start');
const btnBacktrackAbort = document.getElementById('btn-backtrack-abort');
const chkFloorTesting = document.getElementById('limits-floor-testing');

if (btnRecordStart) {
  btnRecordStart.addEventListener('click', () => {
    logSystem('Starting path recording...');
    fetch('/api/path/record/start', { method: 'POST' })
      .then(r => r.json())
      .catch(err => console.error(err));
  });
}

if (btnRecordStop) {
  btnRecordStop.addEventListener('click', () => {
    logSystem('Stopping path recording...');
    fetch('/api/path/record/stop', { method: 'POST' })
      .then(r => r.json())
      .catch(err => console.error(err));
  });
}

if (btnRecordClear) {
  btnRecordClear.addEventListener('click', () => {
    logSystem('Clearing path...');
    fetch('/api/path/record/clear', { method: 'POST' })
      .then(r => r.json())
      .then(() => {
        pathHistory = [];
        drawPath();
      })
      .catch(err => console.error(err));
  });
}

if (btnBacktrackStart) {
  btnBacktrackStart.addEventListener('click', () => {
    logSystem('Requesting path backtrack...');
    fetch('/api/path/backtrack/start', { method: 'POST' })
      .then(r => r.json())
      .then(res => {
        if (!res.ok) {
          logSystem(`⚠️ Backtrack start failed: ${res.error}`);
        }
      })
      .catch(err => console.error(err));
  });
}

if (btnBacktrackAbort) {
  btnBacktrackAbort.addEventListener('click', () => {
    logSystem('Aborting backtrack...');
    fetch('/api/path/backtrack/stop', { method: 'POST' })
      .then(r => r.json())
      .catch(err => console.error(err));
  });
}

if (chkFloorTesting) {
  chkFloorTesting.addEventListener('change', (e) => {
    const floorTesting = e.target.checked;
    logSystem(`Limits update: floorTesting=${floorTesting}`);
    fetch('/api/drive/limits', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ floorTesting })
    })
      .then(r => r.json())
      .catch(err => console.error(err));
  });
}


