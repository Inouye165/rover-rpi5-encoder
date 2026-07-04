let ws = null;
let reconnectTimer = null;
let reconnectInterval = 1000;
const maxReconnectInterval = 16000;

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

// Update UI Badge Status
function updateBadge(badgeElement, state, text) {
  const indicator = badgeElement.querySelector('.status-indicator');
  indicator.className = 'status-indicator ' + state;
  
  // Extract node text and replace
  const textNode = Array.from(badgeElement.childNodes).find(node => node.nodeType === Node.TEXT_NODE);
  if (textNode) {
    textNode.textContent = ' ' + text;
  } else {
    badgeElement.appendChild(document.createTextNode(' ' + text));
  }
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

    case 'battery':
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
      break;

    case 'encoder_total':
      totalValM1.textContent = msg.m1;
      totalValM2.textContent = msg.m2;
      totalValM3.textContent = msg.m3;
      totalValM4.textContent = msg.m4;
      break;

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
  let m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  
  // Dynamic active speed selection from sync slider if positive, otherwise default
  const activeSpeed = parseInt(syncSpeedSlider.value) > 0 ? parseInt(syncSpeedSlider.value) : currentSpeedSetting;

  switch (direction) {
    case 'forward':
      m1 = m2 = m3 = m4 = activeSpeed;
      break;
    case 'reverse':
      m1 = m2 = m3 = m4 = -activeSpeed;
      break;
    case 'left':
      // Turn left: Left wheels backward, right wheels forward
      m1 = m2 = -activeSpeed;
      m3 = m4 = activeSpeed;
      break;
    case 'right':
      // Turn right: Left wheels forward, right wheels backward
      m1 = m2 = activeSpeed;
      m3 = m4 = -activeSpeed;
      break;
    case 'spin_left':
      m1 = m2 = -activeSpeed;
      m3 = m4 = activeSpeed;
      break;
    case 'spin_right':
      m1 = m2 = activeSpeed;
      m3 = m4 = -activeSpeed;
      break;
    case 'stop':
    default:
      m1 = m2 = m3 = m4 = 0;
  }
  
  // Set UI input displays
  syncSpeedSlider.value = (direction === 'stop') ? 0 : activeSpeed;
  syncSpeedReadout.textContent = (direction === 'stop') ? '0' : activeSpeed;
  updateIndividualSliderValues(m1, m2, m3, m4);
  sendMotorSpeeds(m1, m2, m3, m4);
  
  logSystem(`Driving direction: ${direction.toUpperCase()} at speed ${activeSpeed}`);
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

  logSystem('Applying configurations to Yahboom board...');
  
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
  const dt = 0.1; // 100ms loop
  
  // Calculate average left and right velocities
  const vLeft = (m1Speed + m2Speed) / 2;
  const vRight = (m3Speed + m4Speed) / 2;
  
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

