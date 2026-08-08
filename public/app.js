const UI_BUILD_ID = '2026.07.31-operator-auth';

function isRememberBrowserEnabled() {
  const checkbox = document.getElementById('remember-token-checkbox');
  if (checkbox) return checkbox.checked;
  return !!(localStorage.getItem('rover_operator_token'));
}

function setStoredOperatorToken(val, remember) {
  if (!val) {
    sessionStorage.removeItem('rover_operator_token');
    localStorage.removeItem('rover_operator_token');
    return;
  }
  sessionStorage.setItem('rover_operator_token', val);
  if (remember) {
    localStorage.setItem('rover_operator_token', val);
  } else {
    localStorage.removeItem('rover_operator_token');
  }
}

function getOrSyncOperatorToken() {
  const inputToken = document.getElementById('operator-token-input');
  let val = '';
  if (inputToken && inputToken.value && inputToken.value.trim()) {
    val = inputToken.value.trim();
  }
  if (!val) {
    val = sessionStorage.getItem('rover_operator_token') || localStorage.getItem('rover_operator_token') || '';
  }
  if (val) {
    setStoredOperatorToken(val, isRememberBrowserEnabled());
  }
  return val;
}

function getOperatorAuthHeaders() {
  const token = getOrSyncOperatorToken();
  return token ? { 'X-Rover-Operator-Token': token } : {};
}

function showAuthErrorMessage(msg) {
  const displayMsg = msg || 'Operator token missing or invalid. Enter the token and try again.';
  if (typeof logSystem === 'function') {
    logSystem(`[AUTH ERROR] ${displayMsg}`);
  }
  const bannerArm = document.getElementById('arm-auth-error-banner');
  const banner1 = document.getElementById('v2-autonomy-error-banner');
  const banner2 = document.getElementById('v2-calib-error-banner');
  const banner3 = document.getElementById('maint-test-error-banner');
  const errBanner = bannerArm || banner1 || banner2 || banner3;
  if (errBanner) {
    errBanner.textContent = displayMsg;
    errBanner.style.display = 'block';
    setTimeout(() => {
      if (errBanner.textContent === displayMsg) {
        errBanner.style.display = 'none';
      }
    }, 7000);
  } else {
    alert(displayMsg);
  }
}

const activeProtectedRequests = new Map();

async function authenticatedFetch(url, options = {}) {
  const method = (options.method || 'GET').toUpperCase();
  const requestKey = `${method}:${url}`;

  if (activeProtectedRequests.get(requestKey)) {
    console.warn(`[Debounce] Protected request ${requestKey} is already in progress.`);
    return { ok: false, status: 429, json: { ok: false, error: 'Request in flight' } };
  }

  activeProtectedRequests.set(requestKey, true);

  try {
    const authHeaders = getOperatorAuthHeaders();
    const headers = {
      ...(options.headers || {}),
      ...authHeaders
    };

    const res = await fetch(url, { ...options, headers });

    if (res.status === 401 || res.status === 403) {
      let errText = (res.status === 401)
        ? 'Cannot arm rover: Operator token is missing. Enter the operator token and authenticate first.'
        : 'Cannot arm rover: Operator token is invalid. Re-enter the token and authenticate.';
      showAuthErrorMessage(errText);
      return { ok: false, status: res.status, res, json: { ok: false, error: errText } };
    }

    let json = null;
    try {
      json = await res.json();
    } catch (e) {}

    return { ok: res.ok, status: res.status, res, json };
  } catch (err) {
    console.error(`Fetch error for ${url}:`, err);
    return { ok: false, status: 0, error: err.message, json: { ok: false, error: err.message } };
  } finally {
    activeProtectedRequests.delete(requestKey);
  }
}

let ws = null;
let reconnectTimer = null;
let reconnectInterval = 1000;
const maxReconnectInterval = 16000;
let driveArmed = false;
let realOdomActive = false;
let gpDeadmanPressed = false;

// Calibration variables
let currentWheelDiameter = 0.065; // synchronized from ESP32 parameters
let currentTrackWidth = 0.3408575433;    // synchronized from ESP32 parameters / calibration_db (effective track width)
const PHYSICAL_TRACK_WIDTH_M = 0.197; // Physical wheel-center spacing: 7.75 inches = 0.19685 m
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
const btnClearLogs = document.getElementById('btn-clear-logs');

// Encoder calculations for RPM and MPH
let prevEncoderTime = null;
let prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;

// Connect WebSocket
let lastWsErrorLogTime = 0;

// Connect WebSocket
function connectWebSocket() {
  // Prevent duplicate concurrent WebSocket creation
  if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
    return;
  }

  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${wsProtocol}//${window.location.host}`;
  
  logSystem(`Connecting to server WebSocket at ${wsUrl}...`);
  updateBadge(wsStatus, 'alert', 'WS: Connecting...');

  try {
    ws = new WebSocket(wsUrl);
  } catch (e) {
    console.error('Failed to instantiate WebSocket:', e);
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    logSystem('WebSocket connected successfully.');
    updateBadge(wsStatus, 'ok', 'WS: Connected');
    reconnectInterval = 1000; // Reset reconnect timeout backoff
    if (reconnectTimer) clearTimeout(reconnectTimer);

    if (window.roverState && window.roverState.connection) {
      window.roverState.connection.ws = true;
    }

    const token = sessionStorage.getItem('rover_operator_token');
    if (token && ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: 'auth', token }));
    }
    updateEsp32Badge();
    
    // Automatically query firmware identity from ESP32
    fetch('/api/firmware').catch(err => console.error('Firmware query failed:', err));

    // Request current calibration database
    sendServerMessage({ type: 'get_calibration_db' });

    // Synchronize canonical drive status from backend on connection/reconnect
    fetchDriveStatus();

    // Check for connected gamepads on WebSocket connect
    if (typeof checkGamepadConnection === 'function') checkGamepadConnection();

    // Reset last sent joystick command state to require fresh input
    lastSentJoystick = { x: 0, y: 0, deadman: false };
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
    if (window.roverState && window.roverState.connection) {
      window.roverState.connection.ws = false;
    }
    if (typeof driveRover === 'function') {
      driveRover('stop');
    }
    updateEsp32Badge();
    scheduleReconnect();
  };

  ws.onerror = (err) => {
    console.error('WebSocket Error:', err);
    updateBadge(wsStatus, 'alert', 'WS: Connection Error');
    if (window.roverState && window.roverState.connection) {
      window.roverState.connection.ws = false;
    }
    updateEsp32Badge();
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

function updateEsp32Badge() {
  const espBadge = document.getElementById('esp32-version-badge');
  if (!espBadge) return;
  const isWsConnected = ws && ws.readyState === WebSocket.OPEN;
  const serialText = (serialStatus ? serialStatus.textContent : '').toLowerCase();
  const isSerialConnected = serialText.includes('connected');

  if (!isWsConnected || !isSerialConnected) {
    updateBadge(espBadge, 'alert', 'ESP32: Offline');
    return;
  }

  const fw = (window.roverState && window.roverState.firmware) ? window.roverState.firmware : null;
  if (fw && (fw.version || fw.name)) {
    const ver = fw.version || '1.3.0';
    const fwLabel = fw.commit ? `ESP32: v${ver} (${fw.commit})` : `ESP32: v${ver}`;
    updateBadge(espBadge, 'ok', fwLabel);
  } else {
    updateBadge(espBadge, 'off', 'ESP32: Unknown');
  }
}

const maxLogEntries = 500;
let lastLogText = '';
let lastLogTime = 0;
let showVerboseLogs = false;

function isVerboseLog(text) {
  if (!text) return false;
  const t = text.trim();
  return t.startsWith('[raw') ||
         t.startsWith('[Loop Stats]') ||
         t.startsWith('[Loop Timing]') ||
         t.startsWith('[Unknown 0x') ||
         t.startsWith('[Hex]') ||
         t.includes('Loop Stats') ||
         t.includes('raw ...');
}

function addLogLine(text, className, targetContainer) {
  const container = targetContainer || terminalConsole;
  if (!container) return;
  const now = Date.now();
  if (text === lastLogText && (now - lastLogTime < 1000) && className && className.includes('err')) {
    return;
  }
  lastLogText = text;
  lastLogTime = now;

  const line = document.createElement('div');
  line.className = `log-line ${className}`;
  line.textContent = text;
  container.appendChild(line);

  // Enforce max bounded log capacity to prevent unbounded DOM growth
  while (container.childNodes.length > maxLogEntries) {
    container.removeChild(container.firstChild);
  }

  // Scroll to bottom
  container.scrollTop = container.scrollHeight;
}

// Log Terminal Helpers
function logSystem(msg) {
  if (isVerboseLog(msg)) {
    logVerbose(msg, 'system-line');
  } else {
    addLogLine(msg, 'system-line');
  }
}

function logVerbose(msg, className = 'in-line') {
  const verboseConsole = document.getElementById('terminal-verbose-console');
  const verboseChk = document.getElementById('chk-show-verbose-logs');
  showVerboseLogs = verboseChk ? verboseChk.checked : false;

  if (verboseConsole && showVerboseLogs) {
    addLogLine(msg, className, verboseConsole);
  }
}

function logSerialIn(msg) {
  logVerbose(`[Serial In] ${msg}`, 'in-line');
}

function logSerialOut(msg) {
  logVerbose(`[Serial Out] ${msg}`, 'out-line');
}

function logSerialOutErr(msg) {
  logVerbose(`[Error Out] ${msg}`, 'err-line');
}

// Send Commands via WS with Rate-Limited Warning Logging
function sendServerMessage(data) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      ws.send(JSON.stringify(data));
    } catch (e) {
      console.error('Error sending WS message:', e);
    }
  } else if (ws && ws.readyState === WebSocket.CONNECTING) {
    // Drop movement commands silently while socket is connecting without queuing or error logging
    if (data && data.type === 'joystick' && (data.x !== 0 || data.y !== 0)) {
      // Intentionally drop non-zero movement commands while connecting
    }
  } else {
    // Socket is CLOSED or CLOSING
    if (data && data.type === 'joystick' && (data.x !== 0 || data.y !== 0)) {
      // Intentionally drop non-zero movement commands when offline
    }
    
    // Rate-limit terminal error logging to at most once per 5 seconds (5000ms)
    const now = Date.now();
    if (now - lastWsErrorLogTime > 5000) {
      lastWsErrorLogTime = now;
      logSystem('⚠️ Warning: WebSocket is not open to send command.');
    }
  }
}

// Send Upload streams settings
function sendUploadConfig() {
  const t = (streamTotal && streamTotal.checked) ? 1 : 0;
  const r = (streamRealtime && streamRealtime.checked) ? 1 : 0;
  const s = (streamSpeed && streamSpeed.checked) ? 1 : 0;
  // Legacy set_upload disabled for Maker ESP32 firmware
}

// Handle Server Messages

// Safe DOM helpers injected to prevent WebSocket crashes
const setDisabled = (id, value) => { const el = document.getElementById(id); if (el) el.disabled = value; };
const setText = (id, value) => { const el = document.getElementById(id); if (el) { el.innerText = value; el.textContent = value; } };
const setHTML = (id, value) => { const el = document.getElementById(id); if (el) el.innerHTML = value; };
const setStyle = (id, prop, value) => { const el = document.getElementById(id); if (el) el.style[prop] = value; };
const setValue = (id, value) => { const el = document.getElementById(id); if (el) el.value = value; };
const setClass = (id, value) => { const el = document.getElementById(id); if (el) el.className = value; };
const setChecked = (id, value) => { const el = document.getElementById(id); if (el) el.checked = value; };

function handleServerMessage(msg) {
  switch (msg.type) {
    case 'auto_calib_status':
      if (typeof updateAutoCalibUI === 'function') {
        updateAutoCalibUI(msg.status);
      }
      break;

    case 'status':
      if (msg.key === 'serial') {
        if (msg.port) {
          if (comPortInput) comPortInput.value = msg.port;
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
        updateEsp32Badge();
      }
      break;

    case 'battery': {
      if (msg.voltage <= 2.0) {
        if (batteryFill) batteryFill.style.width = '0%';
        if (batteryValue) batteryValue.textContent = 'Unknown';
        if (batteryFill) batteryFill.style.background = '#64748b'; // gray
      } else {
        // Map 6.0V - 8.4V battery pack (2S LiPo usually) to percentage
        const minV = 6.0;
        const maxV = 8.4;
        const pct = Math.min(100, Math.max(0, ((msg.voltage - minV) / (maxV - minV)) * 100));
        if (batteryFill) batteryFill.style.width = `${pct}%`;
        if (batteryValue) batteryValue.textContent = `${msg.voltage.toFixed(2)} V`;
        
        // Update battery colors based on voltage levels
        if (msg.voltage > 7.4) {
          if (batteryFill) batteryFill.style.background = 'linear-gradient(90deg, #39ff14, #00f2fe)';
        } else if (msg.voltage > 6.8) {
          if (batteryFill) batteryFill.style.background = 'linear-gradient(90deg, #ffb700, #ffea00)';
        } else {
          if (batteryFill) batteryFill.style.background = 'linear-gradient(90deg, #ff0055, #ff0000)';
        }
      }
      break;
    }

    case 'encoder_total': {
      if (totalValM1) totalValM1.textContent = msg.m1;
      if (totalValM2) totalValM2.textContent = msg.m2;
      if (totalValM3) totalValM3.textContent = msg.m3;
      if (totalValM4) totalValM4.textContent = msg.m4;

      // Update diagnostics page ticks if they exist
      const testTicksM1 = document.getElementById('test-ticks-m1');
      if (testTicksM1) {
        if (testTicksM1) testTicksM1.textContent = msg.m1 - encoderOffsets[0];
        setText('test-ticks-m2', msg.m2 - encoderOffsets[1]);
        setText('test-ticks-m3', msg.m3 - encoderOffsets[2]);
        setText('test-ticks-m4', msg.m4 - encoderOffsets[3]);
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
        if (straightTicksM1) straightTicksM1.textContent = relM1;
        setText('straight-ticks-m2', relM2);
        setText('straight-ticks-m3', relM3);
        setText('straight-ticks-m4', relM4);
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

          const tpr = 1974.1666666667;
          const rpm1 = (deltaM1 / dt) / tpr * 60.0;
          const rpm2 = (deltaM2 / dt) / tpr * 60.0;
          const rpm3 = (deltaM3 / dt) / tpr * 60.0;
          const rpm4 = (deltaM4 / dt) / tpr * 60.0;

          const rpmToMph = 2.559 * Math.PI * 5.0 / 5280.0; // ~0.007613
          const mph1 = rpm1 * rpmToMph;
          const mph2 = rpm2 * rpmToMph;
          const mph3 = rpm3 * rpmToMph;
          const mph4 = rpm4 * rpmToMph;

          if (speedValM1) speedValM1.innerHTML = `${Math.abs(mph1).toFixed(2)} <small>mph</small>`;
          if (speedValM2) speedValM2.innerHTML = `${Math.abs(mph2).toFixed(2)} <small>mph</small>`;
          if (speedValM3) speedValM3.innerHTML = `${Math.abs(mph3).toFixed(2)} <small>mph</small>`;
          if (speedValM4) speedValM4.innerHTML = `${Math.abs(mph4).toFixed(2)} <small>mph</small>`;

          if (realValM1) realValM1.textContent = rpm1.toFixed(1);
          if (realValM2) realValM2.textContent = rpm2.toFixed(1);
          if (realValM3) realValM3.textContent = rpm3.toFixed(1);
          if (realValM4) realValM4.textContent = rpm4.toFixed(1);

          // Update diagnostics page RPM values
          const testRpmM1 = document.getElementById('test-rpm-m1');
          if (testRpmM1) {
            if (testRpmM1) testRpmM1.textContent = rpm1.toFixed(1);
            setText('test-rpm-m2', rpm2.toFixed(1));
            setText('test-rpm-m3', rpm3.toFixed(1));
            setText('test-rpm-m4', rpm4.toFixed(1));
          }
          // Update straight test area RPM
          const straightRpmM1 = document.getElementById('straight-rpm-m1');
          if (straightRpmM1) {
            if (straightRpmM1) straightRpmM1.textContent = rpm1.toFixed(1);
            setText('straight-rpm-m2', rpm2.toFixed(1));
            setText('straight-rpm-m3', rpm3.toFixed(1));
            setText('straight-rpm-m4', rpm4.toFixed(1));
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
        if (testPwmM1) testPwmM1.textContent = msg.speeds[0];
        setText('test-pwm-m2', msg.speeds[1]);
        setText('test-pwm-m3', msg.speeds[2]);
        setText('test-pwm-m4', msg.speeds[3]);
      }
      break;
    }

    case 'encoder_realtime':
      if (realValM1) realValM1.textContent = msg.m1;
      if (realValM2) realValM2.textContent = msg.m2;
      if (realValM3) realValM3.textContent = msg.m3;
      if (realValM4) realValM4.textContent = msg.m4;
      break;

    case 'speed':
      m1Speed = msg.m1;
      m2Speed = msg.m2;
      m3Speed = msg.m3;
      m4Speed = msg.m4;

      if (speedValM1) speedValM1.innerHTML = `${msg.m1.toFixed(1)} <small>mm/s</small>`;
      if (speedValM2) speedValM2.innerHTML = `${msg.m2.toFixed(1)} <small>mm/s</small>`;
      if (speedValM3) speedValM3.innerHTML = `${msg.m3.toFixed(1)} <small>mm/s</small>`;
      if (speedValM4) speedValM4.innerHTML = `${msg.m4.toFixed(1)} <small>mm/s</small>`;
      
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
        if (encoderActivity) encoderActivity.textContent = `Encoder pkts: ${msg.packets} (${movementState})`;
      }
      break;

    case 'attitude':
      realIMUActive = true;
      imuRoll = msg.roll;
      imuPitch = msg.pitch;
      imuYaw = msg.yaw;
      setText('imu-roll', `${msg.roll.toFixed(1)}°`);
      setText('imu-pitch', `${msg.pitch.toFixed(1)}°`);
      setText('imu-yaw', `${msg.yaw.toFixed(1)}°`);
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
        if (btState) btState.innerText = msg.status.toUpperCase();
        if (btState) btState.style.color = (msg.status === 'completed') ? '#10b981' : ((msg.status === 'aborted') ? '#ef4444' : '#f59e0b');
      }
      if (btProgress && msg.index !== undefined && msg.total !== undefined) {
        const percent = ((msg.total - msg.index) / msg.total * 100).toFixed(1);
        if (btProgress) btProgress.style.width = `${percent}%`;
        if (btProgress) btProgress.innerText = `${percent}%`;
      }
      if (msg.status === 'completed' || msg.status === 'aborted') {
        if (btProgress) {
          if (btProgress) btProgress.style.width = '0%';
          if (btProgress) btProgress.innerText = '';
        }
      }
      break;

    case 'path_status':
      const prState = document.getElementById('path-recording-lbl');
      const prBreadcrumbs = document.getElementById('path-breadcrumbs-lbl');
      if (prState) {
        if (prState) prState.innerText = msg.recording ? 'RECORDING' : 'Idle';
        if (prState) prState.style.color = msg.recording ? '#ef4444' : '#6b7280';
      }
      if (prBreadcrumbs) {
        if (prBreadcrumbs) prBreadcrumbs.innerText = msg.pathLength;
      }
      break;

    case 'limits_status':
      const flLabel = document.getElementById('limits-testing-lbl');
      if (flLabel) {
        if (flLabel) flLabel.innerText = msg.floorTesting ? 'FLOOR TESTING (0.17 m/s)' : 'UNCLAMPED (0.80 m/s)';
        if (flLabel) flLabel.style.color = msg.floorTesting ? '#f59e0b' : '#10b981';
      }
      const flChk = document.getElementById('limits-floor-testing');
      if (flChk) {
        if (flChk) flChk.checked = msg.floorTesting;
      }
      break;

    case 'bno08x_imu': {
      realIMUActive = true;

      // Extract quaternion
      const qw = msg.orientation ? msg.orientation.w : 1.0;
      const qx = msg.orientation ? msg.orientation.x : 0.0;
      const qy = msg.orientation ? msg.orientation.y : 0.0;
      const qz = msg.orientation ? msg.orientation.z : 0.0;

      // Compute Euler pitch and roll from quaternion
      const sinr_cosp = 2 * (qw * qx + qy * qz);
      const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
      const rollRad = Math.atan2(sinr_cosp, cosr_cosp);

      const sinp = 2 * (qw * qy - qz * qx);
      const pitchRad = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

      const siny_cosp = 2 * (qw * qz + qx * qy);
      const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
      const yawRad = Math.atan2(siny_cosp, cosy_cosp);

      imuRoll = rollRad * 180 / Math.PI;
      imuPitch = pitchRad * 180 / Math.PI;
      imuYaw = yawRad * 180 / Math.PI;

      const gx = msg.gyro ? (msg.gyro.x * 180 / Math.PI) : 0.0;
      const gy = msg.gyro ? (msg.gyro.y * 180 / Math.PI) : 0.0;
      const gz = msg.gyro ? (msg.gyro.z * 180 / Math.PI) : 0.0;

      const ax = msg.accel ? (msg.accel.x / 9.80665) : 0.0; // g units for UI
      const ay = msg.accel ? (msg.accel.y / 9.80665) : 0.0;
      const az = msg.accel ? (msg.accel.z / 9.80665) : 0.0;

      // Update UI readouts
      setText('imu-roll', `${imuRoll.toFixed(1)}°`);
      setText('imu-pitch', `${imuPitch.toFixed(1)}°`);
      setText('imu-yaw', `${imuYaw.toFixed(1)}°`);

      setHTML('imu-ax', `${ax.toFixed(2)} <small>g</small>`);
      setHTML('imu-ay', `${ay.toFixed(2)} <small>g</small>`);
      setHTML('imu-az', `${az.toFixed(2)} <small>g</small>`);

      setHTML('imu-gx', `${gx.toFixed(1)} <small>°/s</small>`);
      setHTML('imu-gy', `${gy.toFixed(1)} <small>°/s</small>`);
      setHTML('imu-gz', `${gz.toFixed(1)} <small>°/s</small>`);

      // Update window.roverState for Diagnostics page
      if (window.roverState && window.roverState.imu) {
        window.roverState.imu.roll = imuRoll;
        window.roverState.imu.pitch = imuPitch;
        window.roverState.imu.yaw = imuYaw;
        window.roverState.imu.accel = [msg.accel.x, msg.accel.y, msg.accel.z];
        window.roverState.imu.gyro = [gx, gy, gz];
        window.roverState.imu.seq = msg.sequence;
        window.roverState.imu.gaps = msg.sequenceGaps;
        window.roverState.imu.resets = msg.resetCount;
        window.roverState.imu.calibStatus = msg.calibrationStatus;
        window.roverState.imu.inResetRecovery = msg.inResetRecovery;
      }

      // Update 3D Model rotation
      update3DModelRotation(imuPitch, imuRoll, imuYaw);
      break;
    }

    case 'imu':
      if (Date.now() - lastBno3ATimeMs < 2000) break; // Skip legacy if 0x3A is active
      realIMUActive = true;
      imuYaw = msg.yaw;
      imuPitch = msg.pitch;
      imuRoll = msg.roll;
      
      // Update UI readouts
      setText('imu-roll', `${msg.roll.toFixed(1)}°`);
      setText('imu-pitch', `${msg.pitch.toFixed(1)}°`);
      setText('imu-yaw', `${msg.yaw.toFixed(1)}°`);
      
      setHTML('imu-ax', `${msg.ax.toFixed(2)} <small>g</small>`);
      setHTML('imu-ay', `${msg.ay.toFixed(2)} <small>g</small>`);
      setHTML('imu-az', `${msg.az.toFixed(2)} <small>g</small>`);
      
      setHTML('imu-gx', `${msg.gx.toFixed(1)} <small>°/s</small>`);
      setHTML('imu-gy', `${msg.gy.toFixed(1)} <small>°/s</small>`);
      setHTML('imu-gz', `${msg.gz.toFixed(1)} <small>°/s</small>`);
      
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
      if (msg.data) {
        if (isVerboseLog(msg.data)) {
          logVerbose(`[Board Message] ${msg.data}`);
        } else {
          logSystem(`[Board Message] ${msg.data}`);
        }
      }
      break;

    case 'firmware_info':
      if (window.roverState && window.roverState.firmware) {
        window.roverState.firmware = {
          board: msg.target || msg.board || 'Maker-ESP32-Pro',
          name: msg.name || 'Maker-ESP32-Unified-Rover',
          version: msg.version || '1.3.0',
          proto: msg.protocol || msg.proto || 'v1.1',
          commit: msg.commit,
          build: msg.build
        };
        renderDiagnosticsV2();
      }
      updateEsp32Badge();
      logSystem(`[Firmware Info] Live board identity received: ${msg.name} v${msg.version}`);
      break;

    case 'telemetry_other':
      if (isVerboseLog(msg.cmd || '')) {
        logVerbose(`[Other Telemetry] ${msg.cmd}: ${msg.values ? msg.values.join(',') : ''}`);
      } else {
        logSystem(`[Other Telemetry] ${msg.cmd}: ${msg.values ? msg.values.join(',') : ''}`);
      }
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
          if (btnAutoTest) btnAutoTest.textContent = `Abort [${stageName} ${legType} ${repeatNum}/3]`;
          if (btnAutoTest) btnAutoTest.style.background = 'linear-gradient(135deg, #ff0055, #ff0000)';
          if (btnAutoTest) btnAutoTest.style.boxShadow = '0 0 10px rgba(255, 0, 85, 0.4)';
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
          if (btnAutoTest) btnAutoTest.textContent = 'Auto Test (3ft)';
          if (btnAutoTest) btnAutoTest.style.background = 'linear-gradient(135deg, #00f0ff, #0072ff)';
          if (btnAutoTest) btnAutoTest.style.boxShadow = '0 0 10px rgba(0, 240, 255, 0.3)';
        }

        if (statusEl) statusEl.textContent = `Status: ${msg.msg || 'Test finished.'}`;
        if (copyBtn) copyBtn.style.display = 'inline-block';
        if (closeBtn) closeBtn.style.display = 'inline-block';
        if (abortBtn) abortBtn.style.display = 'none';
        if (panelCopyBtn && clientTestLogs.length > 0) {
          if (panelCopyBtn) panelCopyBtn.style.display = 'inline-block';
        }

        if (modal && msg.msg && msg.msg.includes('ABORTED')) {
          if (modal) modal.style.display = 'none';
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
      updateEsp32Badge();
      break;
    }

    case 'loop_timing': {
      window._latestLoopTiming = {
        lastDurationUs: msg.lastDurationUs,
        minDurationUs: msg.minDurationUs,
        avgDurationUs: msg.avgDurationUs,
        maxDurationUs: msg.maxDurationUs,
        missedDeadlines: msg.missedDeadlines,
        totalIterations: msg.totalIterations,
        schedulingMetricsAvailable: (msg.schedulingMetricsAvailable === true),
        lastStartLatenessUs: msg.lastStartLatenessUs || 0,
        maxStartLatenessUs: msg.maxStartLatenessUs || 0,
        missedControlPeriods: msg.missedControlPeriods || 0,
        maxConsecutiveMissedPeriods: msg.maxConsecutiveMissedPeriods || 0,
        updatedAt: Date.now()
      };

      const elBadge = document.getElementById('v2-diag-badge-loop');
      const elStatus = document.getElementById('v2-diag-val-loop-status');
      const elMissed = document.getElementById('v2-diag-val-loop-missed');
      const elLast = document.getElementById('v2-diag-val-loop-last');
      const elAvg = document.getElementById('v2-diag-val-loop-avg');
      const elMinMax = document.getElementById('v2-diag-val-loop-minmax');
      const elTotal = document.getElementById('v2-diag-val-loop-total');
      const elAge = document.getElementById('v2-diag-val-loop-age');

      const elLatenessLast = document.getElementById('v2-diag-val-loop-lateness-last');
      const elLatenessMax = document.getElementById('v2-diag-val-loop-lateness-max');
      const elPeriodsMissed = document.getElementById('v2-diag-val-loop-periods-missed');
      const elPeriodsConsec = document.getElementById('v2-diag-val-loop-periods-consec');

      if (elMissed) elMissed.innerText = msg.missedDeadlines;
      if (elLast) elLast.innerText = `${msg.lastDurationUs} us (${(msg.lastDurationUs / 1000.0).toFixed(2)} ms)`;
      if (elAvg) elAvg.innerText = `${msg.avgDurationUs} us (${(msg.avgDurationUs / 1000.0).toFixed(2)} ms)`;
      if (elMinMax) elMinMax.innerText = `${msg.minDurationUs} us / ${msg.maxDurationUs} us`;
      if (elTotal) elTotal.innerText = msg.totalIterations;
      if (elAge) elAge.innerText = '0 ms';

      const isSchedAvailable = (msg.schedulingMetricsAvailable === true);

      if (!isSchedAvailable) {
        if (elLatenessLast) elLatenessLast.innerText = 'NOT AVAILABLE (24B Firmware)';
        if (elLatenessMax) elLatenessMax.innerText = 'NOT AVAILABLE (24B Firmware)';
        if (elPeriodsMissed) elPeriodsMissed.innerText = 'NOT AVAILABLE';
        if (elPeriodsConsec) elPeriodsConsec.innerText = 'NOT AVAILABLE';
      } else {
        const lastLateMs = ((msg.lastStartLatenessUs || 0) / 1000.0).toFixed(2);
        const maxLateMs = ((msg.maxStartLatenessUs || 0) / 1000.0).toFixed(2);

        if (elLatenessLast) elLatenessLast.innerText = `${msg.lastStartLatenessUs || 0} us (${lastLateMs} ms)`;
        if (elLatenessMax) elLatenessMax.innerText = `${msg.maxStartLatenessUs || 0} us (${maxLateMs} ms)`;
        if (elPeriodsMissed) elPeriodsMissed.innerText = msg.missedControlPeriods || 0;
        if (elPeriodsConsec) elPeriodsConsec.innerText = msg.maxConsecutiveMissedPeriods || 0;
      }

      const isDeadlineViolated = msg.maxDurationUs >= 10000;
      const hasMissedDeadlines = msg.missedDeadlines > 0;
      const missedPeriods = isSchedAvailable ? (msg.missedControlPeriods || 0) : 0;
      const maxLatenessUs = isSchedAvailable ? (msg.maxStartLatenessUs || 0) : 0;

      if (isSchedAvailable && (missedPeriods > 0 || maxLatenessUs >= 10000)) {
        if (elBadge) {
          elBadge.className = 'badge badge-danger';
          elBadge.innerText = 'SCHEDULING DELAY';
        }
        if (elStatus) {
          elStatus.className = 'status-val badge-danger';
          const maxLateMs = ((msg.maxStartLatenessUs || 0) / 1000.0).toFixed(2);
          elStatus.innerText = `SCHEDULING LATE: ${missedPeriods} missed periods (Peak Lateness: ${maxLateMs} ms)`;
        }
      } else if (hasMissedDeadlines || isDeadlineViolated) {
        if (elBadge) {
          elBadge.className = 'badge badge-danger';
          elBadge.innerText = 'EXECUTION OVERRUN';
        }
        if (elStatus) {
          elStatus.className = 'status-val badge-danger';
          elStatus.innerText = `EXECUTION OVERRUN: ${msg.missedDeadlines} overruns (Max Exec: ${msg.maxDurationUs} us)`;
        }
      } else {
        if (elBadge) {
          elBadge.className = 'badge badge-healthy';
          elBadge.innerText = 'HEALTHY (100 Hz)';
        }
        if (elStatus) {
          elStatus.className = 'status-val badge-healthy';
          if (isSchedAvailable) {
            const maxLateMs = ((msg.maxStartLatenessUs || 0) / 1000.0).toFixed(2);
            elStatus.innerText = `HEALTHY: On-time scheduling & execution (Avg Exec: ${msg.avgDurationUs} us, Peak Lateness: ${maxLateMs} ms)`;
          } else {
            elStatus.innerText = `EXECUTION HEALTHY: Avg Exec: ${msg.avgDurationUs} us (Scheduling telemetry requiring 40B firmware update)`;
          }
        }
      }

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
        if (protoDisplay) protoDisplay.innerText = `Protocol: v${msg.protoMajor}.${msg.protoMinor}`;
      }
      if (sessDisplay && msg.sessionId !== undefined) {
        if (sessDisplay) sessDisplay.innerText = `Session: ${msg.sessionId}`;
      }
      
      // Update safety lock badge
      if (lockBadge) {
        if (motorLockStatus) {
          if (lockBadge) lockBadge.innerText = '🔒 Safety Lock: Active';
          if (lockBadge) lockBadge.style.background = 'rgba(16, 185, 129, 0.2)';
          if (lockBadge) lockBadge.style.color = '#10b981';
          if (lockBadge) lockBadge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
        } else {
          if (lockBadge) lockBadge.innerText = '⚠️ Safety Lock: Missing';
          if (lockBadge) lockBadge.style.background = 'rgba(239, 68, 68, 0.2)';
          if (lockBadge) lockBadge.style.color = '#ef4444';
          if (lockBadge) lockBadge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        }
      }
      
      // Update mode badge
      if (modeBadge && isSimulation !== undefined) {
        if (modeBadge) modeBadge.innerText = isSimulation ? 'Mode: Simulation Only' : 'Mode: Real-Drive';
      }
      
      // Update thresholds table
      const motors = ["m1", "m2", "m3", "m4"];
      for (let i = 0; i < 4; i++) {
        const fwdVal = fwd[i];
        const revVal = rev[i];
        setText('pi-val-${motors[i]}-fwd', fwdVal > 0 ? `${fwdVal} PWM` : "--");
        setText('pi-val-${motors[i]}-rev', revVal > 0 ? `${revVal} PWM` : "--");
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
          if (movementLbl) movementLbl.innerText = movementDetected ? "YES (>= 8 ticks)" : "NO";
          if (movementLbl) movementLbl.style.color = movementDetected ? "#10b981" : "#f59e0b";
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
          if (badge) badge.innerText = `Active (Motor ${activeMotorNum})`;
          if (badge) badge.style.background = 'rgba(59, 130, 246, 0.2)';
          if (badge) badge.style.color = '#3b82f6';
          if (badge) badge.style.border = '1px solid rgba(59, 130, 246, 0.4)';
        } else {
          if (badge) badge.innerText = 'Locked';
          if (badge) badge.style.background = 'rgba(239, 68, 68, 0.2)';
          if (badge) badge.style.color = '#fca5a5';
          if (badge) badge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        }
      }
      
      setText('tele-maint-session', msg.sessionId || '--');
      setText('tele-maint-active', active ? 'ACTIVE' : 'Inactive');
      setStyle('tele-maint-active', 'color', active ? '#10b981' : '#ef4444');
      setText('tele-maint-deadman', deadmanActive ? 'Active' : 'Offline');
      setStyle('tele-maint-deadman', 'color', deadmanActive ? '#10b981' : '#f59e0b');
      setText('tele-maint-timeout', active ? `${(remainingTimeout / 1000).toFixed(1)}s` : '--');
      setText('tele-maint-target', testPwm || '0');
      setText('tele-maint-actual', actualPwm || '0');
      
      let delta = 0;
      let total = 0;
      if (active && activeMotor >= 0 && activeMotor < 4) {
        const totalElems = [totalValM1, totalValM2, totalValM3, totalValM4];
        const speedElems = [realValM1, realValM2, realValM3, realValM4];
        total = totalElems[activeMotor] ? totalElems[activeMotor].textContent : '0';
        delta = speedElems[activeMotor] ? speedElems[activeMotor].textContent : '0';
      }
      setText('tele-maint-enc-delta', delta);
      setText('tele-maint-enc-total', total);
      
      const slider = document.getElementById('maint-pwm-slider');
      const stepFwd = document.getElementById('btn-maint-fwd-step');
      const stepRev = document.getElementById('btn-maint-rev-step');
      
      if (active) {
        if (slider) slider.disabled = false;
        if (stepFwd) stepFwd.disabled = false;
        if (stepRev) stepRev.disabled = false;
        if (stepFwd) stepFwd.style.background = 'rgba(59, 130, 246, 0.2)';
        if (stepFwd) stepFwd.style.color = '#93c5fd';
        if (stepFwd) stepFwd.style.cursor = 'pointer';
        if (stepRev) stepRev.style.background = 'rgba(239, 68, 68, 0.2)';
        if (stepRev) stepRev.style.color = '#fca5a5';
        if (stepRev) stepRev.style.cursor = 'pointer';
      } else {
        if (slider) slider.disabled = true;
        if (slider) slider.value = 0;
        setText('maint-pwm-display', '0');
        if (stepFwd) stepFwd.disabled = true;
        if (stepRev) stepRev.disabled = true;
        if (stepFwd) stepFwd.style.background = '#374151';
        if (stepFwd) stepFwd.style.color = '#9ca3af';
        if (stepFwd) stepFwd.style.cursor = 'not-allowed';
        if (stepRev) stepRev.style.background = '#374151';
        if (stepRev) stepRev.style.color = '#9ca3af';
        if (stepRev) stepRev.style.cursor = 'not-allowed';
      }
      break;
    }

    case 'normal_drive_status': {
      updateCanonicalDriveState(msg);
      break;
    }

    case 'autonomy_status': {
      if (msg.status) updateAutonomyState(msg.status);
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
        if (labelActive) labelActive.textContent = `L: ${msg.leftTrim.toFixed(3)} | R: ${msg.rightTrim.toFixed(3)}`;
        if (labelActive) labelActive.style.color = '#10b981';
      }

      const activeFwdText = document.getElementById('active-fwd-trims');
      if (activeFwdText) {
        if (activeFwdText) activeFwdText.textContent = `${msg.leftTrim.toFixed(4)} / ${msg.rightTrim.toFixed(4)}`;
      }
      break;
    }

    case 'calibration_db':
      updateCalibrationDbUI(msg.db);
      break;

    case 'rover_trims_rev_sync': {
      const activeRevText = document.getElementById('active-rev-trims');
      if (activeRevText) {
        if (activeRevText) activeRevText.textContent = `${msg.leftTrimRev.toFixed(4)} / ${msg.rightTrimRev.toFixed(4)}`;
      }
      logSystem(`[Config Sync] Synced active Reverse Trims: ${msg.leftTrimRev.toFixed(4)} / ${msg.rightTrimRev.toFixed(4)}`);
      break;
    }

    case 'lidar_test_status': {
      lidarTestState = msg.state;
      const testBadge = document.getElementById('lidar-test-state-badge');
      if (testBadge) {
        // Build progress label
        let progressLabel = msg.state;
        if (msg.speedTier && msg.totalPass) {
          progressLabel = `${msg.speedTier} Pass ${msg.totalPass}/${msg.totalPasses}`;
        }
        if (testBadge) testBadge.textContent = progressLabel;
        if (testBadge) testBadge.style.background = '';
        if (testBadge) testBadge.style.color = '';
        if (testBadge) testBadge.style.borderColor = '';
        
        if (msg.state === 'IDLE') {
          stopTestScanPolling();
          if (testBadge) testBadge.style.background = 'rgba(107, 114, 128, 0.2)';
          if (testBadge) testBadge.style.color = '#9ca3af';
          if (testBadge) testBadge.style.border = '1px solid rgba(107, 114, 128, 0.4)';
          if (testBadge) testBadge.textContent = 'IDLE';
          setStyle('btn-start-lidar-test', 'display', 'block');
          setStyle('btn-stop-lidar-test', 'display', 'none');
          
          if (lidarOdomPath.length > 0 || lidarPosePath.length > 0) {
            saveCurrentPathToHistory(lastTelemetrySpeedTier);
            lidarOdomPath = [];
            lidarPosePath = [];
            drawLidarTestCanvas();
          }
        } else if (msg.state === 'ZEROING' || msg.state === 'RETURNING_HOME_WAIT') {
          if (testBadge) testBadge.style.background = 'rgba(245, 158, 11, 0.2)';
          if (testBadge) testBadge.style.color = '#f59e0b';
          if (testBadge) testBadge.style.border = '1px solid rgba(245, 158, 11, 0.4)';
          setStyle('btn-start-lidar-test', 'display', 'none');
          setStyle('btn-stop-lidar-test', 'display', 'block');
        } else if (msg.state === 'FORWARD_RUNNING' || msg.state === 'RETURNING_HOME') {
          if (testBadge) testBadge.style.background = 'rgba(6, 182, 212, 0.2)';
          if (testBadge) testBadge.style.color = '#06b6d4';
          if (testBadge) testBadge.style.border = '1px solid rgba(6, 182, 212, 0.4)';
          setStyle('btn-start-lidar-test', 'display', 'none');
          setStyle('btn-stop-lidar-test', 'display', 'block');
        } else if (msg.state === 'COMPLETE') {
          stopTestScanPolling();
          if (testBadge) testBadge.style.background = 'rgba(16, 185, 129, 0.2)';
          if (testBadge) testBadge.style.color = '#10b981';
          if (testBadge) testBadge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
          if (testBadge) testBadge.textContent = 'COMPLETE ✓';
          setStyle('btn-start-lidar-test', 'display', 'block');
          setStyle('btn-stop-lidar-test', 'display', 'none');
          
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
        if (confSpan) confSpan.textContent = (msg.metrics.confidence * 100).toFixed(1) + '%';
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
        if (lblLeft) lblLeft.textContent = `${leftPower}% (${dirText})`;
        if (barLeft) barLeft.style.width = `${leftPower}%`;
        if (lblLeft) lblLeft.style.color = leftPower > 70 ? '#ff0055' : (leftPower > 40 ? '#f59e0b' : '#10b981');
      }
      
      if (lblRight && barRight) {
        const rightPower = isMoving ? (msg.rightPowerPct || 0) : 0;
        if (lblRight) lblRight.textContent = `${rightPower}% (${dirText})`;
        if (barRight) barRight.style.width = `${rightPower}%`;
        if (lblRight) lblRight.style.color = rightPower > 70 ? '#ff0055' : (rightPower > 40 ? '#f59e0b' : '#10b981');
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
          if (barEffort) barEffort.style.left = '50%';
          if (barEffort) barEffort.style.width = `${effortPct}%`;
          if (barEffort) barEffort.style.background = 'linear-gradient(90deg, #a855f7, #00f0ff)';
        } else {
          if (barEffort) barEffort.style.left = `${50 + effortPct}%`;
          if (barEffort) barEffort.style.width = `${Math.abs(effortPct)}%`;
          if (barEffort) barEffort.style.background = 'linear-gradient(90deg, #ff0055, #a855f7)';
        }
        
        const effortDir = effort > 0.005 ? 'STEER LEFT' : (effort < -0.005 ? 'STEER RIGHT' : 'CENTER');
        if (lblEffort) lblEffort.textContent = isMoving ? `${effort.toFixed(3)} rad/s (${effortDir})` : '0.000 rad/s (CENTER)';
        if (lblEffort) lblEffort.style.color = isMoving ? (Math.abs(effort) > 0.2 ? '#ff0055' : (Math.abs(effort) > 0.08 ? '#f59e0b' : '#00f0ff')) : '#00f0ff';
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
          if (applyBtn) applyBtn.disabled = false;
          if (applyBtn) applyBtn.style.opacity = '1';
          if (applyBtn) applyBtn.style.cursor = 'pointer';
        } else {
          if (applyBtn) applyBtn.disabled = true;
          if (applyBtn) applyBtn.style.opacity = '0.6';
          if (applyBtn) applyBtn.style.cursor = 'not-allowed';
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
  const isArmed = (window.roverState && window.roverState.drive && window.roverState.drive.armed === true) || driveArmed;
  if (!isArmed && direction !== 'stop') {
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
      x = 0.0;
      break;
    case 'reverse':
      y = -1.0;
      x = 0.0;
      break;
    case 'left':
      x = -1.0;
      y = 0.0;
      break;
    case 'right':
      x = 1.0;
      y = 0.0;
      break;
    case 'spin_left':
      x = -1.0;
      y = 0.0;
      break;
    case 'spin_right':
      x = 1.0;
      y = 0.0;
      break;
    case 'stop':
    default:
      x = 0.0;
      y = 0.0;
      break;
  }

  sendServerMessage({ type: 'joystick', x, y });
  if (direction !== 'stop') {
    logSystem(`Driving direction: ${direction.toUpperCase()} via Coordinated Joystick Path (x: ${x.toFixed(2)}, y: ${y.toFixed(2)})`);
  }
}

// DPad Action Listeners with Deadman Release
function bindDpadButton(btnEl, direction) {
  if (!btnEl) return;
  
  const startDrive = (e) => {
    if (e) e.preventDefault();
    btnEl.classList.add('active');
    driveRover(direction);
  };

  const stopDrive = (e) => {
    if (e) e.preventDefault();
    btnEl.classList.remove('active');
    driveRover('stop');
  };

  btnEl.addEventListener('pointerdown', startDrive);
  btnEl.addEventListener('pointerup', stopDrive);
  btnEl.addEventListener('pointerleave', stopDrive);
  btnEl.addEventListener('pointercancel', stopDrive);
}

bindDpadButton(ctrlForward, 'forward');
bindDpadButton(ctrlReverse, 'reverse');
bindDpadButton(ctrlLeft, 'left');
bindDpadButton(ctrlRight, 'right');
bindDpadButton(ctrlSpinLeft, 'spin_left');
bindDpadButton(ctrlSpinRight, 'spin_right');

// Keyboard WASD / Arrows controls with Active Key Tracking & Safe Release
let activeKeyboardKeys = new Set();

document.addEventListener('keydown', (e) => {
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(e.key) && document.activeElement.tagName !== 'INPUT') {
    e.preventDefault();
  }

  if (document.activeElement.tagName === 'INPUT') return;
  if (e.repeat) return; // Ignore auto-repeat keydown events

  const key = e.key.toLowerCase();
  activeKeyboardKeys.add(key);

  if (straightDriveLocked && ['a', 'arrowleft', 'd', 'arrowright', 'q', 'e'].includes(key)) {
    return;
  }

  switch (key) {
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
  activeKeyboardKeys.delete(key);

  if (['w', 'arrowup'].includes(key)) ctrlForward.classList.remove('active');
  if (['s', 'arrowdown'].includes(key)) ctrlReverse.classList.remove('active');
  if (['a', 'arrowleft'].includes(key)) ctrlLeft.classList.remove('active');
  if (['d', 'arrowright'].includes(key)) ctrlRight.classList.remove('active');
  if (key === 'q') ctrlSpinLeft.classList.remove('active');
  if (key === 'e') ctrlSpinRight.classList.remove('active');
  if ([' ', 'escape'].includes(key)) ctrlStopCenter.classList.remove('active');

  const driveKeys = ['w', 's', 'a', 'd', 'q', 'e', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
  const anyDriveKeyActive = Array.from(activeKeyboardKeys).some(k => driveKeys.includes(k));
  if (!anyDriveKeyActive) {
    driveRover('stop');
  }
});

// Telemetry toggles checkboxes
[streamTotal, streamRealtime, streamSpeed].forEach((checkbox) => {
  if (checkbox) checkbox.addEventListener('change', sendUploadConfig);
});

// Config Form Submit Handler
if (configForm) {
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
}

// Read and Reset Config Buttons
if (btnReadFlash) {
  btnReadFlash.addEventListener('click', () => {
    logSystem('Querying board flash variables ($read_flash#)...');
    sendServerMessage({ type: 'read_flash' });
  });
}

if (btnResetFlash) {
  btnResetFlash.addEventListener('click', () => {
    if (confirm('Are you sure you want to restore default factory configurations? The board will restart.')) {
      logSystem('Restoring factory defaults ($flash_reset#)...');
      sendServerMessage({ type: 'flash_reset' });
    }
  });
}

// Port change submission
btnChangePort.addEventListener('click', () => {
  const newPort = comPortInput.value.trim();
  if (newPort) {
    logSystem(`Requesting server reconnect serial port to: ${newPort}`);
    sendServerMessage({ type: 'change_port', port: newPort });
  }
});

// Phase 4 Arm and Disarm drive control actions
function fetchDriveStatus() {
  return fetch('/api/drive/status')
    .then(res => {
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return res.json();
    })
    .then(data => {
      if (data && data.ok && data.status) {
        updateCanonicalDriveState(data.status);
      }
      return data;
    })
    .catch(err => {
      logSystem(`⚠️ Error fetching drive status: ${err.message}`);
    });
}

async function armNormalDrive() {
  logSystem('Sending arm normal drive request...');

  // Pre-check operator token presence before issuing network call
  const token = getOrSyncOperatorToken();
  if (!token) {
    const msg = 'Cannot arm rover: Operator token is missing. Enter the operator token and authenticate first.';
    logSystem(`⚠️ ${msg}`);
    showAuthErrorMessage(msg);
    await fetchDriveStatus();
    return;
  }

  try {
    const result = await authenticatedFetch('/api/drive/arm', { method: 'POST' });
    const data = result.json || {};
    if (result.ok && data.ok) {
      logSystem(data.message || 'Arm request processed.');
    } else {
      let errorMsg = data.error;
      if (result.status === 401) {
        errorMsg = 'Cannot arm rover: Operator token is missing. Enter the operator token and authenticate first.';
      } else if (result.status === 403) {
        errorMsg = 'Cannot arm rover: Operator token is invalid. Re-enter the token and authenticate.';
      } else if (!errorMsg) {
        errorMsg = 'Cannot arm rover: Re-authentication is required.';
      }
      logSystem(`⚠️ ${errorMsg}`);
      showAuthErrorMessage(errorMsg);
    }
  } catch (err) {
    const errorMsg = 'Cannot arm rover: Re-authentication is required.';
    logSystem(`⚠️ ${errorMsg}`);
    showAuthErrorMessage(errorMsg);
  }

  // Immediate status refresh & brief 150ms retry for ESP32 telemetry packet propagation
  await fetchDriveStatus();
  await new Promise(r => setTimeout(r, 150));
  await fetchDriveStatus();
}

async function disarmNormalDrive() {
  logSystem('Sending disarm normal drive request...');
  try {
    const result = await authenticatedFetch('/api/drive/disarm', { method: 'POST' });
    const data = result.json || {};
    if (result.ok && data.ok) {
      logSystem(data.message || 'Disarm request processed.');
    } else if (!result.ok && result.status !== 401 && result.status !== 403) {
      logSystem(`⚠️ Disarm request rejected: ${data.error || 'Unknown error'}`);
    }
  } catch (err) {
    logSystem(`⚠️ Error disarming normal drive: ${err.message}`);
  }

  // Immediate status refresh & brief 150ms retry for ESP32 telemetry packet propagation
  await fetchDriveStatus();
  await new Promise(r => setTimeout(r, 150));
  await fetchDriveStatus();
}

const btnArmDrive = document.getElementById('btn-arm-drive');
const btnDisarmDrive = document.getElementById('btn-disarm-drive');

if (btnArmDrive) {
  btnArmDrive.addEventListener('click', armNormalDrive);
}

if (btnDisarmDrive) {
  btnDisarmDrive.addEventListener('click', disarmNormalDrive);
}

btnClearLogs.addEventListener('click', () => {
  terminalConsole.innerHTML = '';
  logSystem('Logs cleared.');
});

// --- Stage 3 & 4 Canonical State & Read-Only Rendering ---
window.roverState = {
  connection: {
    ws: true,
    serial: true,
    gamepad: false,
    telemAgeMs: 12,
    odomAgeMs: 10
  },
  drive: {
    armed: null,
    known: false,
    mode: 0,
    source: 0,
    cmdAgeMs: 999999,
    reqLinear: 0.0,
    reqAngular: 0.0,
    limLinear: 0.0,
    limAngular: 0.0,
    lockStatus: null,
    estop: false,
    battery: '12.4'
  },
  ros: {
    system: 'Healthy',
    lidarBridge: 'Healthy',
    encoderOdom: 'Healthy',
    foxgloveBridge: 'Healthy (Port 8765)',
    diagHz: 1.0,
    odomHz: 10.0
  },
  odom: {
    x: 0.0,
    y: 0.0,
    yaw: 0.0,
    dist: 0.0
  },
  imu: {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0,
    accel: [0.0, 0.0, 9.8],
    gyro: [0.0, 0.0, 0.0]
  },
  lidar: {
    connected: true,
    state: 'scanning',
    health: 'Good',
    scanHz: 6.6,
    pointCount: 360,
    ageMs: 2
  },
  firmware: {
    board: 'Maker ESP32 Pro',
    name: 'esp-maker-usba-4motor',
    version: '1.0.0',
    proto: 'Binary Packet v1'
  }
};

function updateCanonicalDriveState(statusObj) {
  if (!statusObj || typeof statusObj !== 'object') return;
  const target = (statusObj.status && typeof statusObj.status === 'object') ? statusObj.status : statusObj;
  const st = window.roverState;

  if (typeof target.armed === 'boolean') {
    st.drive.armed = target.armed;
    driveArmed = target.armed;
  }
  if (typeof target.mode === 'number') {
    st.drive.mode = target.mode;
  }
  if (typeof target.source === 'number') {
    st.drive.source = target.source;
  }
  if (typeof target.cmdAge === 'number') {
    st.drive.cmdAgeMs = target.cmdAge;
  } else if (typeof target.cmdAgeMs === 'number') {
    st.drive.cmdAgeMs = target.cmdAgeMs;
  }
  if (typeof target.reqLinear === 'number') {
    st.drive.reqLinear = target.reqLinear;
  }
  if (typeof target.reqAngular === 'number') {
    st.drive.reqAngular = target.reqAngular;
  }
  if (typeof target.limLinear === 'number') {
    st.drive.limLinear = target.limLinear;
  }
  if (typeof target.limAngular === 'number') {
    st.drive.limAngular = target.limAngular;
  }
  if (typeof target.lockStatus === 'boolean') {
    st.drive.lockStatus = target.lockStatus;
  }
  if (typeof target.estop === 'boolean') {
    st.drive.estop = target.estop;
  }

  st.drive.known = true;
  renderDriveV2Status();
}

function renderStage3V2Panels() {
  renderDriveV2Status();
  renderAutonomyV2();
  renderSensorsV2Summary();
  renderDiagnosticsV2();
  if (typeof updateCalibrationReadiness === 'function') updateCalibrationReadiness();
}

function renderDriveV2Status() {
  const st = window.roverState;
  const drv = st.drive;

  const elWs = document.getElementById('v2-drive-val-ws');
  if (elWs) {
    elWs.textContent = st.connection.ws ? 'CONNECTED' : 'DISCONNECTED';
    elWs.style.color = st.connection.ws ? '#10b981' : '#ef4444';
  }

  const elSerial = document.getElementById('v2-drive-val-serial');
  if (elSerial) {
    elSerial.textContent = st.connection.serial ? 'CONNECTED' : 'DISCONNECTED';
    elSerial.style.color = st.connection.serial ? '#10b981' : '#ef4444';
  }

  const elGp = document.getElementById('v2-drive-val-gamepad');
  if (elGp) {
    elGp.textContent = st.connection.gamepad ? 'CONNECTED' : 'DISCONNECTED';
    elGp.style.color = st.connection.gamepad ? '#10b981' : '#9ca3af';
  }

  // 1. Top Status Strip Armed Badge (#v2-drive-val-armed)
  const elArmed = document.getElementById('v2-drive-val-armed');
  if (elArmed) {
    if (!drv.known || drv.armed === null) {
      elArmed.textContent = 'UNKNOWN';
      elArmed.style.color = '#9ca3af';
    } else if (drv.armed) {
      elArmed.textContent = 'ARMED';
      elArmed.style.color = '#10b981';
    } else {
      elArmed.textContent = 'DISARMED';
      elArmed.style.color = '#f59e0b';
    }
  }

  // 2. Operational Controls Card Header Badge (#normal-drive-badge)
  const badge = document.getElementById('normal-drive-badge');
  if (badge) {
    if (!drv.known || drv.armed === null) {
      badge.innerText = 'Unknown';
      badge.style.background = 'rgba(156, 163, 175, 0.2)';
      badge.style.color = '#9ca3af';
      badge.style.border = '1px solid rgba(156, 163, 175, 0.4)';
    } else if (drv.armed) {
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

  // 3. Telemetry HUD Drive State (#tele-drive-state)
  const elState = document.getElementById('tele-drive-state');
  if (elState) {
    if (!drv.known || drv.armed === null) {
      elState.innerText = 'Unknown';
      elState.style.color = '#9ca3af';
    } else if (drv.armed) {
      elState.innerText = 'ARMED';
      elState.style.color = '#10b981';
    } else {
      elState.innerText = 'Disarmed';
      elState.style.color = '#fca5a5';
    }
  }

  // 4. Telemetry HUD Driver Mode (#tele-drive-mode)
  const modes = ['LOCKED', 'MAINTENANCE', 'CALIBRATION', 'NORMAL_DRIVE', 'EMERGENCY_STOP', 'FAULTED'];
  const elMode = document.getElementById('tele-drive-mode');
  if (elMode) {
    if (!drv.known || typeof drv.mode !== 'number') {
      elMode.innerText = '--';
      elMode.style.color = '#9ca3af';
    } else {
      elMode.innerText = modes[drv.mode] || `UNKNOWN (${drv.mode})`;
      elMode.style.color = (drv.mode === 3) ? '#10b981' : ((drv.mode === 4 || drv.mode === 5) ? '#ef4444' : '#f59e0b');
    }
  }

  // 5. Telemetry HUD Physical Lock (#tele-drive-phys-lock)
  // Phase 6 Semantics:
  // lockStatus === true  => Physical Lock / Safety Clamp ACTIVE (Hardware locked)
  // lockStatus === false => Physical Lock CLEAR / DISABLED (Hardware live)
  const elPhys = document.getElementById('tele-drive-phys-lock');
  if (elPhys) {
    if (!drv.known || drv.lockStatus === null || drv.lockStatus === undefined) {
      elPhys.innerText = 'Unknown';
      elPhys.style.color = '#9ca3af';
    } else if (drv.lockStatus === true) {
      elPhys.innerText = 'CLAMP ACTIVE';
      elPhys.style.color = '#ef4444';
    } else {
      elPhys.innerText = 'CLEAR (DISABLED)';
      elPhys.style.color = '#10b981';
    }
  }

  // 6. Telemetry HUD Active Source (#tele-drive-source)
  const sources = ['NONE', 'WEB_JOYSTICK', 'USB_SERIAL', 'ROS', 'POSITION', 'CALIBRATION'];
  const elSource = document.getElementById('tele-drive-source');
  if (elSource) {
    if (!drv.known || typeof drv.source !== 'number') {
      elSource.innerText = '--';
    } else {
      elSource.innerText = sources[drv.source] || `UNKNOWN (${drv.source})`;
    }
  }

  // 7. Telemetry HUD Command Speeds & Age
  const elReqLin = document.getElementById('tele-drive-req-lin');
  if (elReqLin) elReqLin.innerText = `${(drv.reqLinear || 0).toFixed(2)} m/s`;

  const elReqAng = document.getElementById('tele-drive-req-ang');
  if (elReqAng) elReqAng.innerText = `${(drv.reqAngular || 0).toFixed(2)} rad/s`;

  const elLimLin = document.getElementById('tele-drive-lim-lin');
  if (elLimLin) elLimLin.innerText = `${(drv.limLinear || 0).toFixed(2)} m/s`;

  const elLimAng = document.getElementById('tele-drive-lim-ang');
  if (elLimAng) elLimAng.innerText = `${(drv.limAngular || 0).toFixed(2)} rad/s`;

  const elCmdAge = document.getElementById('tele-drive-age');
  if (elCmdAge) elCmdAge.innerText = (drv.cmdAgeMs !== undefined && drv.cmdAgeMs < 999999) ? `${drv.cmdAgeMs} ms` : '--';

  // 8. E-stop badge (#v2-drive-val-estop)
  const elEstop = document.getElementById('v2-drive-val-estop');
  if (elEstop) {
    elEstop.textContent = drv.estop ? 'ACTIVE (ESTOP)' : 'CLEAR';
    elEstop.style.color = drv.estop ? '#ef4444' : '#10b981';
  }

  // 9. Gamepad HUD Arm State (#gp-live-arm)
  const elGpArm = document.getElementById('gp-live-arm');
  if (elGpArm) {
    if (!drv.known || drv.armed === null) {
      elGpArm.innerText = 'UNKNOWN';
      elGpArm.style.color = '#9ca3af';
    } else if (drv.armed) {
      elGpArm.innerText = 'ARMED';
      elGpArm.style.color = '#10b981';
    } else {
      elGpArm.innerText = 'DISARMED';
      elGpArm.style.color = '#6b7280';
    }
  }

  // 10. Arm/Disarm Buttons Enabled/Disabled State
  if (btnArmDrive && btnDisarmDrive) {
    if (drv.known && drv.armed === true) {
      btnArmDrive.disabled = true;
      btnArmDrive.style.opacity = '0.5';
      btnArmDrive.style.cursor = 'not-allowed';
      btnDisarmDrive.disabled = false;
      btnDisarmDrive.style.opacity = '1.0';
      btnDisarmDrive.style.cursor = 'pointer';
    } else if (drv.known && drv.armed === false) {
      btnArmDrive.disabled = false;
      btnArmDrive.style.opacity = '1.0';
      btnArmDrive.style.cursor = 'pointer';
      btnDisarmDrive.disabled = true;
      btnDisarmDrive.style.opacity = '0.5';
      btnDisarmDrive.style.cursor = 'not-allowed';
    } else {
      btnArmDrive.disabled = false;
      btnArmDrive.style.opacity = '1.0';
      btnDisarmDrive.disabled = false;
      btnDisarmDrive.style.opacity = '1.0';
    }
  }

  // Battery and ages
  const elBat = document.getElementById('v2-drive-val-battery');
  if (elBat) elBat.textContent = `${drv.battery} V`;

  const elTelemAge = document.getElementById('v2-drive-val-telem-age');
  if (elTelemAge) elTelemAge.textContent = `${st.connection.telemAgeMs || 10} ms`;

  const elOdomAge = document.getElementById('v2-drive-val-odom-age');
  if (elOdomAge) elOdomAge.textContent = `${st.connection.odomAgeMs || 10} ms`;

  // Faults list
  const faultsContainer = document.getElementById('v2-drive-faults-list');
  if (faultsContainer) {
    const faults = [];
    if (!st.connection.ws) faults.push({ text: '⚠️ WebSocket Disconnected', level: 'warn' });
    if (!st.connection.serial) faults.push({ text: '⚠️ ESP32 Serial Link Offline', level: 'err' });
    if (drv.estop) faults.push({ text: '🛑 Emergency Stop Lock Active', level: 'err' });
    if (st.connection.telemAgeMs > 1000) faults.push({ text: '⚠️ Stale Telemetry Warning (>1s)', level: 'warn' });

    if (faults.length === 0) {
      faultsContainer.innerHTML = '<div class="v2-fault-item v2-fault-ok">✓ No Active Safety Faults (System Nominal)</div>';
    } else {
      faultsContainer.innerHTML = faults.map(f => `<div class="v2-fault-item v2-fault-${f.level}">${f.text}</div>`).join('');
    }
  }
}

function renderAutonomyV2() {
  const st = window.roverState;
  const elSys = document.getElementById('v2-ros-val-system');
  if (elSys) elSys.textContent = st.ros.system;
  const elLidar = document.getElementById('v2-ros-val-lidar');
  if (elLidar) elLidar.textContent = st.ros.lidarBridge;
  const elOdom = document.getElementById('v2-ros-val-odom');
  if (elOdom) elOdom.textContent = st.ros.encoderOdom;
  const elFox = document.getElementById('v2-ros-val-foxglove');
  if (elFox) elFox.textContent = st.ros.foxgloveBridge;
  const elDiagHz = document.getElementById('v2-ros-val-diag-hz');
  if (elDiagHz) elDiagHz.textContent = `${st.ros.diagHz.toFixed(1)} Hz`;
  const elOdomHz = document.getElementById('v2-ros-val-odom-hz');
  if (elOdomHz) elOdomHz.textContent = `${st.ros.odomHz.toFixed(1)} Hz`;

  // Localization Values
  const elLocX = document.getElementById('v2-loc-val-x');
  if (elLocX) elLocX.textContent = `${st.odom.x.toFixed(3)} m`;
  const elLocY = document.getElementById('v2-loc-val-y');
  if (elLocY) elLocY.textContent = `${st.odom.y.toFixed(3)} m`;
  const elLocYaw = document.getElementById('v2-loc-val-yaw');
  if (elLocYaw) elLocYaw.textContent = `${st.odom.yaw.toFixed(1)}°`;
}

function renderSensorsV2Summary() {
  const st = window.roverState;
  const elLidar = document.getElementById('v2-sensor-val-lidar');
  if (elLidar) elLidar.textContent = `${st.lidar.health} (${st.lidar.scanHz.toFixed(1)} Hz)`;
  const elImu = document.getElementById('v2-sensor-val-imu');
  if (elImu) elImu.textContent = `Healthy (10 Hz)`;
  const elOdom = document.getElementById('v2-sensor-val-odom');
  if (elOdom) elOdom.textContent = `Healthy (10 Hz)`;

  // LiDAR HUD items in sensors-lidar
  const elLidarHz = document.getElementById('v2-lidar-val-hz');
  if (elLidarHz) elLidarHz.textContent = `${st.lidar.scanHz.toFixed(1)} Hz`;
  const elLidarPts = document.getElementById('v2-lidar-val-points');
  if (elLidarPts) elLidarPts.textContent = `${st.lidar.pointCount}`;
  const elLidarAge = document.getElementById('v2-lidar-val-age');
  if (elLidarAge) elLidarAge.textContent = `${st.lidar.ageMs} ms`;

  // IMU HUD items in sensors-imu
  const elImuBadge = document.getElementById('v2-imu-badge');
  const elImuSummary = document.getElementById('v2-sensor-val-imu');
  const now = Date.now();
  const imuAgeMs = st.imu.lastTime ? (now - st.imu.lastTime) : 99999;
  const isImuFresh = realIMUActive && (imuAgeMs <= 500);

  if (!isImuFresh) {
    if (elImuBadge) {
      elImuBadge.className = 'badge badge-warning';
      elImuBadge.textContent = 'STALE / NOT AVAILABLE';
    }
    if (elImuSummary) elImuSummary.textContent = 'Stale / Offline';
  } else {
    if (elImuBadge) {
      elImuBadge.className = 'badge badge-success';
      elImuBadge.textContent = 'HEALTHY (50 Hz)';
    }
    if (elImuSummary) elImuSummary.textContent = 'Healthy (50 Hz)';
  }

  const elRoll = document.getElementById('v2-imu-val-roll');
  if (elRoll) elRoll.textContent = isImuFresh ? `${st.imu.roll.toFixed(1)}°` : '--';

  const elPitch = document.getElementById('v2-imu-val-pitch');
  if (elPitch) elPitch.textContent = isImuFresh ? `${st.imu.pitch.toFixed(1)}°` : '--';

  const elYaw = document.getElementById('v2-imu-val-yaw');
  if (elYaw) elYaw.textContent = isImuFresh ? `${st.imu.yaw.toFixed(1)}°` : '--';

  const elAccel = document.getElementById('v2-imu-val-accel');
  if (elAccel) elAccel.textContent = isImuFresh ? `${st.imu.accel.map(n => n.toFixed(1)).join(', ')} m/s²` : '-- m/s²';

  const elGyro = document.getElementById('v2-imu-val-gyro');
  if (elGyro) elGyro.textContent = isImuFresh ? `${st.imu.gyro.map(n => n.toFixed(1)).join(', ')} °/s` : '-- °/s';

  const elSeq = document.getElementById('v2-imu-val-seq');
  if (elSeq) elSeq.textContent = isImuFresh ? `#${st.imu.seq || 0} (${st.imu.gaps || 0} gaps)` : '--';

  const elResets = document.getElementById('v2-imu-val-resets');
  if (elResets) elResets.textContent = isImuFresh ? `${st.imu.resets || 0} (Rec: ${st.imu.inResetRecovery ? 'YES' : 'NO'})` : '--';

  const elCalib = document.getElementById('v2-imu-val-calib');
  if (elCalib) elCalib.textContent = isImuFresh ? `Lvl ${st.imu.calibStatus || 0} / ${imuAgeMs}ms` : '--';

  // Odometry HUD items in sensors-odometry
  const elOdomX = document.getElementById('v2-odom-val-x');
  if (elOdomX) elOdomX.textContent = `${st.odom.x.toFixed(3)} m`;
  const elOdomY = document.getElementById('v2-odom-val-y');
  if (elOdomY) elOdomY.textContent = `${st.odom.y.toFixed(3)} m`;
  const elOdomYaw = document.getElementById('v2-odom-val-yaw');
  if (elOdomYaw) elOdomYaw.textContent = `${st.odom.yaw.toFixed(1)}°`;
  const elOdomDist = document.getElementById('v2-odom-val-dist');
  if (elOdomDist) elOdomDist.textContent = `${st.odom.dist.toFixed(3)} m`;
}

function calculateOverallSystemHealth() {
  const st = window.roverState || {};
  const conn = st.connection || {};
  const drv = st.drive || {};
  const lidar = st.lidar || {};
  const ros = st.ros || {};

  let overall = 'HEALTHY';
  let driveState = 'READY';
  let calibState = 'IDLE';
  let wsState = conn.ws ? 'CONNECTED' : 'OFFLINE';
  let serialState = conn.serial ? 'CONNECTED' : 'OFFLINE';
  let lidarState = (lidar.connected !== false && lidar.state !== 'offline' && lidar.state !== 'error') ? 'HEALTHY' : 'OFFLINE';
  let odomState = (ros.encoderOdom === 'Healthy' || ros.encoderOdom === 'Running') ? 'HEALTHY' : 'OFFLINE';
  let rosState = (ros.system === 'Healthy' || ros.system === 'Running') ? 'HEALTHY' : 'OFFLINE';
  let cameraState = 'AVAILABLE';
  let reasons = [];

  if (drv.armed) driveState = 'ARMED';
  if (drv.estop) driveState = 'FAULT (E-STOP)';

  if (!conn.ws) reasons.push('WebSocket connection offline');
  if (!conn.serial) reasons.push('Serial UART link offline');
  if (drv.estop) reasons.push('Hardware E-Stop active');
  if (lidarState === 'OFFLINE') reasons.push('LiDAR sensor offline or stale');
  if (odomState === 'OFFLINE') reasons.push('ROS 2 Odometry node offline');
  if (rosState === 'OFFLINE') reasons.push('ROS 2 system container offline');

  if (!conn.ws || !conn.serial || drv.estop) {
    overall = 'FAULT';
  } else if (lidarState === 'OFFLINE' || odomState === 'OFFLINE' || rosState === 'OFFLINE') {
    overall = 'DEGRADED';
  }

  return {
    overall,
    drive: driveState,
    calib: calibState,
    ws: wsState,
    serial: serialState,
    lidar: lidarState,
    odom: odomState,
    ros: rosState,
    camera: cameraState,
    reason: reasons.length > 0 ? reasons.join('; ') : 'None'
  };
}

function renderDiagnosticsV2() {
  const st = window.roverState;
  if (!st) return;

  const h = calculateOverallSystemHealth();

  // Section A: Overall System Health
  setText('v2-health-val-overall', h.overall);
  setClass('v2-health-val-overall', `status-val ${h.overall === 'HEALTHY' ? 'badge-healthy' : (h.overall === 'DEGRADED' ? 'badge-warning' : 'badge-alert')}`);
  setText('v2-health-overall-badge', h.overall);
  setClass('v2-health-overall-badge', `badge ${h.overall === 'HEALTHY' ? 'badge-success' : (h.overall === 'DEGRADED' ? 'badge-warning' : 'badge-danger')}`);

  setText('v2-health-val-drive', h.drive);
  setText('v2-health-val-calib', h.calib);
  setText('v2-health-val-ws', h.ws);
  setText('v2-health-val-serial', h.serial);
  setText('v2-health-val-lidar', h.lidar);
  setText('v2-health-val-odom', h.odom);
  setText('v2-health-val-ros', h.ros);
  setText('v2-health-val-camera', h.camera);

  const reasonContainer = document.getElementById('v2-health-reason-container');
  if (reasonContainer) {
    if (h.overall !== 'HEALTHY') {
      reasonContainer.style.display = 'block';
      setText('v2-health-val-reason', h.reason);
    } else {
      reasonContainer.style.display = 'none';
    }
  }

  // Section B: Drive & Safety Read-Only
  const drv = st.drive || {};
  setText('v2-diag-val-armed', drv.armed ? 'ARMED' : (drv.armed === false ? 'DISARMED' : 'UNKNOWN'));
  setText('v2-diag-val-mode', `${drv.mode !== undefined ? drv.mode : 0} (${drv.mode === 0 ? 'COORDINATED' : 'INDIVIDUAL'})`);

  const srcMap = {
    0: 'None (Source 0)',
    1: 'Serial Terminal (Source 1)',
    2: 'Gamepad (Source 2)',
    3: 'Browser Joystick (Source 3)',
    4: 'Autonomous/Test (Source 4)'
  };
  const srcVal = drv.source !== undefined ? drv.source : 0;
  const srcLabel = srcMap[srcVal] || `Source ${srcVal}`;
  setText('v2-diag-val-source', srcLabel);

  const isZeroVel = ((drv.reqLinear || 0) === 0 && (drv.reqAngular || 0) === 0);
  setText('v2-diag-val-motion-status', isZeroVel ? 'Zero (No Movement Requested)' : `Moving (Linear: ${(drv.reqLinear || 0).toFixed(2)} m/s, Angular: ${(drv.reqAngular || 0).toFixed(2)} rad/s)`);

  setText('v2-diag-val-cmd-age', drv.cmdAgeMs !== undefined ? `${drv.cmdAgeMs} ms` : '-- ms');
  setText('v2-diag-val-req-lin', `${(drv.reqLinear || 0).toFixed(2)} m/s`);
  setText('v2-diag-val-req-ang', `${(drv.reqAngular || 0).toFixed(2)} rad/s`);
  setText('v2-diag-val-lim-lin', `${(drv.limLinear || 0).toFixed(2)} m/s`);
  setText('v2-diag-val-lim-ang', `${(drv.limAngular || 0).toFixed(2)} rad/s`);
  setText('v2-diag-val-motor-cmd', drv.motorCommand ? JSON.stringify(drv.motorCommand) : '[0, 0, 0, 0]');
  setText('v2-diag-val-lock', (typeof straightDriveLocked !== 'undefined' && straightDriveLocked) ? 'LOCKED' : 'UNLOCKED');
  setText('v2-diag-val-estop', drv.estop ? 'ACTIVE (STOPPED)' : 'INACTIVE');
  setText('v2-diag-val-floor', 'INACTIVE');
  setText('v2-diag-val-backtrack', 'IDLE');
  setText('v2-diag-val-recording', 'IDLE');
  setText('v2-diag-val-autocal', 'IDLE');
  setText('v2-diag-val-maint', 'INACTIVE');
  setText('v2-diag-val-fault', drv.estop ? 'E-Stop Triggered' : 'None');

  // Section C: Connections & Services
  setText('v2-diag-val-srv-node', 'RUNNING');
  setText('v2-diag-val-srv-ws', (st.connection && st.connection.ws) ? 'CONNECTED' : 'DISCONNECTED');
  setText('v2-diag-val-srv-lidar', (st.lidar && st.lidar.connected !== false) ? 'RUNNING' : 'OFFLINE');
  setText('v2-diag-val-srv-ros', (st.ros && st.ros.system) || 'RUNNING');
  setText('v2-diag-val-srv-foxglove', (st.ros && st.ros.foxgloveBridge) || 'RUNNING');
  setText('v2-diag-val-srv-camera', 'AVAILABLE');

  const fw = st.firmware || {};
  setText('v2-fw-val-board', fw.board || 'Awaiting firmware identity...');
  setText('v2-fw-val-name', fw.name || 'Awaiting firmware identity...');
  setText('v2-fw-val-ver', fw.version || 'Unknown');
  setText('v2-fw-val-proto', fw.proto || 'Unknown');

  setText('v2-serial-val-dev', '/dev/rover-esp32');
  setText('v2-serial-val-baud', '115200 Baud');
  setText('v2-serial-val-hz', '10 Hz');
  setText('v2-serial-val-errs', '0');

  // Section D: ROS 2 Diagnostics
  const ros = st.ros || {};
  setText('v2-rosdiag-val-health', ros.system || 'HEALTHY');
  setText('v2-rosdiag-val-diag', `${(ros.diagHz || 1.0).toFixed(1)} Hz`);
  setText('v2-rosdiag-val-scan', `${((st.lidar && st.lidar.scanHz) || 6.6).toFixed(1)} Hz`);
  setText('v2-rosdiag-val-odom', `${(ros.odomHz || 10.0).toFixed(1)} Hz`);
  setText('v2-rosdiag-val-tf', `${(ros.odomHz || 10.0).toFixed(1)} Hz`);
  setText('v2-rosdiag-val-tf-static', 'Active (Latched)');
  setText('v2-rosdiag-val-foxglove', 'Listening (Port 8765)');

  // Section E: Sensor Health
  const lid = st.lidar || {};
  setText('v2-diag-sensor-lidar-state', lid.state || 'Scanning');
  setText('v2-diag-sensor-lidar-hz', `${(lid.scanHz || 6.6).toFixed(1)} Hz`);
  setText('v2-diag-sensor-lidar-pts', `${lid.pointCount || 360} pts`);
  setText('v2-diag-sensor-lidar-age', `${lid.ageMs || 2} ms`);
  setText('v2-diag-sensor-lidar-err', 'None');

  const od = st.odom || {};
  setText('v2-diag-sensor-odom-health', ros.encoderOdom || 'Healthy');
  setText('v2-diag-sensor-odom-pos', `${(od.x || 0).toFixed(3)}m, ${(od.y || 0).toFixed(3)}m`);
  setText('v2-diag-sensor-odom-yaw', `${((od.yaw || 0) * 180 / Math.PI).toFixed(1)}°`);
  setText('v2-diag-sensor-odom-dist', `${(od.dist || 0).toFixed(3)}m`);
  setText('v2-diag-sensor-odom-stale', 'FRESH (<1s)');

  setText('v2-diag-sensor-camera-state', 'Available');
  setText('v2-diag-sensor-camera-url', '/api/camera');
  setText('v2-diag-sensor-camera-err', 'None');

  // Section G: Raw Telemetry JSON
  const rawJsonEl = document.getElementById('v2-diag-raw-json');
  if (rawJsonEl) {
    rawJsonEl.textContent = JSON.stringify(st, null, 2);
  }
}

function generateDiagnosticsBundle() {
  const st = window.roverState ? JSON.parse(JSON.stringify(window.roverState)) : {};
  const logLines = [];
  if (typeof terminalConsole !== 'undefined' && terminalConsole) {
    const lines = terminalConsole.querySelectorAll('.log-line');
    lines.forEach(l => logLines.push(l.textContent));
  }

  delete st.tokens;
  delete st.passwords;
  delete st.keys;

  return {
    timestamp: new Date().toISOString(),
    frontendVersion: 'v1.0.3 (f50adec)',
    userAgent: typeof navigator !== 'undefined' ? navigator.userAgent : 'NodeJS/Test',
    sharedState: st,
    boundedLogs: logLines.slice(-100)
  };
}

function initDiagnosticsEventHandlers() {
  const chkVerbose = document.getElementById('chk-show-verbose-logs');
  if (chkVerbose) {
    chkVerbose.addEventListener('change', () => {
      const details = document.getElementById('v2-diag-verbose-logs-details');
      if (details) {
        details.open = chkVerbose.checked;
      }
      if (typeof logSystem === 'function') {
        logSystem(chkVerbose.checked ? 'Verbose log stream enabled.' : 'Verbose log stream disabled.');
      }
    });
  }

  const btnCopyLogs = document.getElementById('btn-copy-logs');
  if (btnCopyLogs) {
    btnCopyLogs.addEventListener('click', () => {
      if (typeof terminalConsole === 'undefined' || !terminalConsole) return;
      const text = Array.from(terminalConsole.querySelectorAll('.log-line')).map(l => l.textContent).join('\n');
      if (typeof navigator !== 'undefined' && navigator.clipboard) {
        navigator.clipboard.writeText(text).then(() => { if (typeof logSystem === 'function') logSystem('Logs copied to clipboard.'); }).catch(() => {});
      }
    });
  }

  const btnCopyRaw = document.getElementById('btn-copy-raw-telemetry');
  if (btnCopyRaw) {
    btnCopyRaw.addEventListener('click', () => {
      const jsonStr = JSON.stringify(window.roverState || {}, null, 2);
      if (typeof navigator !== 'undefined' && navigator.clipboard) {
        navigator.clipboard.writeText(jsonStr).then(() => { if (typeof logSystem === 'function') logSystem('Raw telemetry JSON copied to clipboard.'); }).catch(() => {});
      }
    });
  }

  const btnDevRefresh = document.getElementById('btn-dev-refresh-status');
  if (btnDevRefresh) {
    btnDevRefresh.addEventListener('click', () => {
      if (typeof fetchDriveStatus === 'function') fetchDriveStatus();
      if (typeof fetch === 'function') {
        fetch('/api/status')
          .then(r => r.json())
          .then(data => {
            const out = document.getElementById('v2-dev-status-output');
            if (out) {
              out.style.display = 'block';
              out.textContent = `[API Status] ${JSON.stringify(data, null, 2)}`;
            }
            if (typeof logSystem === 'function') logSystem('Service status refreshed.');
          })
          .catch(err => { if (typeof logSystem === 'function') logSystem(`Status refresh failed: ${err.message}`); });
      }
    });
  }

  const btnDevReconnect = document.getElementById('btn-dev-reconnect-ws');
  if (btnDevReconnect) {
    btnDevReconnect.addEventListener('click', () => {
      if (typeof logSystem === 'function') logSystem('Manual WebSocket reconnect initiated.');
      if (typeof connectWebSocket === 'function') connectWebSocket();
    });
  }

  const btnDevFoxglove = document.getElementById('btn-dev-open-foxglove');
  if (btnDevFoxglove) {
    btnDevFoxglove.addEventListener('click', () => {
      if (typeof window !== 'undefined') window.open('http://10.0.0.246:8765', '_blank');
    });
  }

  const btnDevRawApi = document.getElementById('btn-dev-open-raw-api');
  if (btnDevRawApi) {
    btnDevRawApi.addEventListener('click', () => {
      if (typeof window !== 'undefined') window.open('/api/status', '_blank');
    });
  }

  const btnDevSyncCalib = document.getElementById('btn-dev-sync-calib-db');
  if (btnDevSyncCalib) {
    btnDevSyncCalib.addEventListener('click', () => {
      if (typeof sendServerMessage === 'function') {
        sendServerMessage({ type: 'get_calibration_db' });
        if (typeof logSystem === 'function') logSystem('Calibration database re-sync requested.');
      }
    });
  }

  const btnExportBundle = document.getElementById('btn-export-diag-bundle');
  if (btnExportBundle) {
    btnExportBundle.addEventListener('click', () => {
      const bundle = generateDiagnosticsBundle();
      if (typeof Blob !== 'undefined' && typeof URL !== 'undefined' && typeof document !== 'undefined') {
        const blob = new Blob([JSON.stringify(bundle, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `rover_diagnostics_${Date.now()}.json`;
        a.click();
        URL.revokeObjectURL(url);
        const statusEl = document.getElementById('v2-export-bundle-status');
        if (statusEl) statusEl.textContent = '✓ Diagnostics bundle downloaded.';
      }
    });
  }

  const btnCopyBundle = document.getElementById('btn-copy-diag-bundle');
  if (btnCopyBundle) {
    btnCopyBundle.addEventListener('click', () => {
      const bundle = generateDiagnosticsBundle();
      if (typeof navigator !== 'undefined' && navigator.clipboard) {
        navigator.clipboard.writeText(JSON.stringify(bundle, null, 2)).then(() => {
          const statusEl = document.getElementById('v2-export-bundle-status');
          if (statusEl) statusEl.textContent = '✓ Diagnostics bundle copied to clipboard.';
        }).catch(() => {});
      }
    });
  }
}

// Initial Stage 3 Rendering & WebSocket / Drive / Poller Startup
document.addEventListener('DOMContentLoaded', () => {
  renderStage3V2Panels();
  initDiagnosticsEventHandlers();
  if (typeof connectWebSocket === 'function') connectWebSocket();
  fetchDriveStatus();
  if (typeof updateLidarTabState === 'function') updateLidarTabState();
  if (typeof checkGamepadConnection === 'function') checkGamepadConnection();
});
setTimeout(() => {
  renderStage3V2Panels();
  initDiagnosticsEventHandlers();
  if (typeof connectWebSocket === 'function') connectWebSocket();
  fetchDriveStatus();
  if (typeof updateLidarTabState === 'function') updateLidarTabState();
  if (typeof checkGamepadConnection === 'function') checkGamepadConnection();
}, 100);

// --- Tab Switching Logic (Canonical 5-Tab Navigation) ---
const topTabButtons = document.querySelectorAll('.tab-navigation-bar .tab-btn');
const topTabContents = document.querySelectorAll('.tab-content');

let activeTopTabId = 'tab-drive-v2';

function updateLidarTabState() {
  const isLidarActive = (
    activeTopTabId === 'tab-drive-v2' ||
    activeTopTabId === 'tab-sensors-v2'
  );

  lidarActiveTab = activeTopTabId;
  if (isLidarActive) {
    if (typeof startLidarPolling === 'function') startLidarPolling();
    if (typeof pollLidar === 'function') pollLidar();
    requestAnimationFrame(() => {
      if (typeof latestLidarScan !== 'undefined' && latestLidarScan && typeof drawPolarScan === 'function') {
        drawPolarScan(latestLidarScan);
      }
    });
  } else {
    if (typeof stopLidarPolling === 'function') stopLidarPolling();
  }
}

function activateTopTab(targetTabId) {
  // Stage 4 & 5 Safety Requirement: Switching away from Drive or Calibration tab immediately stops motors / aborts tests
  if (activeTopTabId === 'tab-drive-v2' && targetTabId !== 'tab-drive-v2') {
    if (typeof driveRover === 'function') {
      driveRover('stop');
    }
  }
  if (activeTopTabId === 'tab-calibration-v2' && targetTabId !== 'tab-calibration-v2') {
    if (typeof abortAutoCalibrationTest === 'function') {
      abortAutoCalibrationTest();
    }
  }

  activeTopTabId = targetTabId;

  // Toggle active states and ARIA attributes on top-level buttons
  topTabButtons.forEach(btn => {
    const isTarget = (btn.dataset.tab === targetTabId);
    btn.classList.toggle('active', isTarget);
    btn.setAttribute('aria-selected', isTarget ? 'true' : 'false');
    btn.setAttribute('tabindex', isTarget ? '0' : '-1');
  });

  // Toggle visibility of top-level tab contents
  const topLevelTabIds = ['tab-drive-v2', 'tab-autonomy-v2', 'tab-sensors-v2', 'tab-calibration-v2', 'tab-diagnostics-v2'];
  topLevelTabIds.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.classList.toggle('active', id === targetTabId);
    }
  });

  updateLidarTabState();

  if (targetTabId === 'tab-sensors-v2' && typeof resizeCanvas === 'function') {
    resizeCanvas();
  }
}

// Bind Top-Level Navigation Buttons
topTabButtons.forEach(btn => {
  btn.addEventListener('click', () => {
    activateTopTab(btn.dataset.tab);
  });
});

// Keyboard Navigation for Navigation Bar (Arrow Left/Right, Home, End)
const navBar = document.querySelector('.tab-navigation-bar');
if (navBar) {
  navBar.addEventListener('keydown', (e) => {
    const buttons = Array.from(topTabButtons);
    const currentIndex = buttons.indexOf(document.activeElement);
    if (currentIndex === -1) return;

    let newIndex = currentIndex;
    if (e.key === 'ArrowRight' || e.key === 'ArrowDown') {
      newIndex = (currentIndex + 1) % buttons.length;
    } else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp') {
      newIndex = (currentIndex - 1 + buttons.length) % buttons.length;
    } else if (e.key === 'Home') {
      newIndex = 0;
    } else if (e.key === 'End') {
      newIndex = buttons.length - 1;
    } else {
      return;
    }

    e.preventDefault();
    buttons[newIndex].focus();
    activateTopTab(buttons[newIndex].dataset.tab);
  });
}


// --- Odometry / Dead Reckoning Kinematics Simulation ---
// Tracks X/Y positions and heading over time based on speed telemetry
const odomXDisplay = document.getElementById('odom-x');
const odomYDisplay = document.getElementById('odom-y');
const odomSpeedDisplay = document.getElementById('odom-speed');
const pathCanvas = document.getElementById('path-canvas') || document.getElementById('v2-odom-traj-canvas');
const ctx = pathCanvas ? pathCanvas.getContext('2d') : null;

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
      const yawEl = document.getElementById('imu-yaw');
      const rollEl = document.getElementById('imu-roll');
      const pitchEl = document.getElementById('imu-pitch');
      if (yawEl) yawEl.textContent = `${imuYaw.toFixed(1)}°`;
      if (rollEl) rollEl.textContent = `0.0°`;
      if (pitchEl) pitchEl.textContent = `0.0°`;
    }
  }
  
  // Update Stats UI
  if (odomXDisplay) odomXDisplay.innerHTML = `${(odomX / 1000).toFixed(2)} <small>m</small>`;
  if (odomYDisplay) odomYDisplay.innerHTML = `${(odomY / 1000).toFixed(2)} <small>m</small>`;
  if (odomSpeedDisplay) odomSpeedDisplay.innerHTML = `${linearSpeed.toFixed(1)} <small>mm/s</small>`;
  
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
  if (document.getElementById('tab-sensors-v2')?.classList.contains('active')) {
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
document.getElementById('btn-reset-imu')?.addEventListener('click', () => {
  imuPitch = 0;
  imuRoll = 0;
  if (realIMUActive) {
    logSystem('Requested Pitch/Roll hardware offset reset.');
  } else {
    logSystem('Orientation model aligned to flat horizon.');
    update3DModelRotation(0, 0, imuYaw);
  }
});

document.getElementById('btn-reset-odometry')?.addEventListener('click', () => {
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
    const rawM1 = parseInt(document.getElementById('telemetry-total-m1')?.textContent || 0);
    const rawM2 = parseInt(document.getElementById('telemetry-total-m2')?.textContent || 0);
    const rawM3 = parseInt(document.getElementById('telemetry-total-m3')?.textContent || 0);
    const rawM4 = parseInt(document.getElementById('telemetry-total-m4')?.textContent || 0);

    encoderOffsets = [rawM1, rawM2, rawM3, rawM4];

    setText('test-ticks-m1', 0);
    setText('test-ticks-m2', 0);
    setText('test-ticks-m3', 0);
    setText('test-ticks-m4', 0);

    logSystem(`Zeroed out diagnostics encoder offsets: [${encoderOffsets.join(', ')}]`);
  });
}

// Straight Drive Test Event Listeners & Logic
const btnResetStraight = document.getElementById('btn-reset-straight-test');
if (btnResetStraight) {
  btnResetStraight.addEventListener('click', () => {
    const rawM1 = parseInt(document.getElementById('telemetry-total-m1')?.textContent || 0);
    const rawM2 = parseInt(document.getElementById('telemetry-total-m2')?.textContent || 0);
    const rawM3 = parseInt(document.getElementById('telemetry-total-m3')?.textContent || 0);
    const rawM4 = parseInt(document.getElementById('telemetry-total-m4')?.textContent || 0);

    straightTestOffsets = [rawM1, rawM2, rawM3, rawM4];

    setText('straight-ticks-m1', 0);
    setText('straight-ticks-m2', 0);
    setText('straight-ticks-m3', 0);
    setText('straight-ticks-m4', 0);

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
        if (straightBadge) straightBadge.textContent = 'Steering Locked';
        if (straightBadge) straightBadge.style.background = 'rgba(239, 68, 68, 0.2)';
        if (straightBadge) straightBadge.style.color = '#fca5a5';
        if (straightBadge) straightBadge.style.border = '1px solid rgba(239, 68, 68, 0.4)';
        logSystem("🔒 Straight Drive Lock ENABLED. Steering controls are now disabled.");
      } else {
        if (straightBadge) straightBadge.textContent = 'Steering Unlocked';
        if (straightBadge) straightBadge.style.background = 'rgba(107, 114, 128, 0.2)';
        if (straightBadge) straightBadge.style.color = '#9ca3af';
        if (straightBadge) straightBadge.style.border = '1px solid rgba(107, 114, 128, 0.4)';
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
    if (deltaEl) deltaEl.textContent = (delta >= 0 ? '+' : '') + delta;
    if (deltaEl) deltaEl.style.color = Math.abs(delta) === 0 ? '#39ff14' : (Math.abs(delta) < 15 ? '#ffb700' : '#ff0055');
  }

  if (statusEl) {
    if (avgLeft === 0 && avgRight === 0) {
      if (statusEl) statusEl.textContent = 'READY (0 ticks)';
      if (statusEl) statusEl.style.color = 'var(--text-muted)';
    } else {
      const absDelta = Math.abs(delta);
      if (absDelta <= 4) {
        if (statusEl) statusEl.textContent = 'PERFECTLY BALANCED';
        if (statusEl) statusEl.style.color = '#39ff14';
      } else if (absDelta <= 15) {
        if (statusEl) statusEl.textContent = 'OK (MINOR DRIFT)';
        if (statusEl) statusEl.style.color = '#ffb700';
      } else {
        if (statusEl) statusEl.textContent = 'MISMATCH DETECTED';
        if (statusEl) statusEl.style.color = '#ff0055';
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
let lidarActiveTab = 'tab-drive-v2';
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

let isLidarPollPending = false;

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
    isLidarPollPending = false;
    console.log('[LiDAR UI] Polling stopped.');
  }
}

async function pollLidar() {
  if (isLidarPollPending) return;
  isLidarPollPending = true;

  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 1200);

    const [statusRes, scanRes] = await Promise.all([
      fetch('/api/lidar/status', { signal: controller.signal }).then(r => r.json()).catch(() => null),
      fetch('/api/lidar/scan', { signal: controller.signal }).then(r => {
        if (r && r.status === 200) return r.json();
        return null;
      }).catch(() => null)
    ]);
    clearTimeout(timeoutId);

    if (statusRes) updateLidarStatus(statusRes);
    if (scanRes) updateLidarScan(scanRes);
  } catch (err) {
    console.error('[LiDAR UI] Error polling LiDAR:', err);
    const stateEl = document.getElementById('lidar-val-state');
    if (stateEl) {
      if (stateEl) stateEl.textContent = 'ERROR';
      if (stateEl) stateEl.style.color = '#ef4444';
    }
  } finally {
    isLidarPollPending = false;
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
    if (stateEl) stateEl.textContent = status.state ? status.state.toUpperCase() : 'DISCONNECTED';
    if (status.state === 'scanning') {
      if (stateEl) stateEl.style.color = '#10b981';
    } else if (status.state === 'connecting' || status.state === 'initializing') {
      if (stateEl) stateEl.style.color = '#f59e0b';
    } else {
      if (stateEl) stateEl.style.color = '#ef4444';
    }
  }
  
  if (deviceEl) deviceEl.textContent = status.device || '--';
  if (modelEl) modelEl.textContent = status.model || '--';
  
  if (healthEl) {
    if (healthEl) healthEl.textContent = status.health || '--';
    if (status.health === 'OK' || status.health === '0') {
      if (healthEl) healthEl.style.color = '#10b981';
    } else if (status.health !== 'unknown') {
      if (healthEl) healthEl.style.color = '#ef4444';
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
      if (uptimeEl) uptimeEl.textContent = `${hrs}h ${mins}m ${secs}s`;
    } else {
      if (uptimeEl) uptimeEl.textContent = '--';
    }
  }
  
  if (reconnectsEl) reconnectsEl.textContent = status.reconnectCount !== undefined && status.reconnectCount !== null ? status.reconnectCount : '--';
  
  if (errCard) {
    if (status.lastError) {
      errCard.style.display = 'block';
      if (errEl) errEl.textContent = status.lastError;
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

  // Draw compact LiDAR local view on Drive tab
  try {
    if (typeof drawCompactLidarScan === 'function') {
      drawCompactLidarScan(scan);
    }
  } catch (err) {
    console.error('[LiDAR UI] Compact canvas draw error:', err);
  }

  // Draw heavy polar canvas & sample table only when Sensors tab is active
  if (activeTopTabId === 'tab-sensors-v2') {
    try {
      drawPolarScan(scan);
      renderSampleTable(scan);
      updateTrackInterference(scan);
    } catch (err) {
      console.error('[LiDAR UI] Polar canvas draw error:', err);
    }
  }
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
      if (elBadge) elBadge.textContent = '⚠️ Interference';
      if (elBadge) elBadge.style.background = 'rgba(255, 0, 85, 0.15)';
      if (elBadge) elBadge.style.color = '#ff0055';
      if (elBadge) elBadge.style.borderColor = 'rgba(255, 0, 85, 0.4)';
    } else if (dFront < 0.15 || dLeft < 0.15 || dRight < 0.15) {
      if (elBadge) elBadge.textContent = '⚠️ Caution';
      if (elBadge) elBadge.style.background = 'rgba(245, 158, 11, 0.15)';
      if (elBadge) elBadge.style.color = '#f59e0b';
      if (elBadge) elBadge.style.borderColor = 'rgba(245, 158, 11, 0.4)';
    } else {
      if (elBadge) elBadge.textContent = '✓ Clear';
      if (elBadge) elBadge.style.background = 'rgba(16, 185, 129, 0.15)';
      if (elBadge) elBadge.style.color = '#10b981';
      if (elBadge) elBadge.style.borderColor = 'rgba(16, 185, 129, 0.4)';
    }
  }
}


function drawPolarScan(scan) {
  if (typeof drawCompactLidarScan === 'function') {
    drawCompactLidarScan(scan);
  }

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
  const sampleTbody = document.getElementById('lidar-sample-table-body');
  if (!sampleTbody) return;
  
  if (!scan.points || scan.points.length === 0) {
    if (sampleTbody) sampleTbody.innerHTML = `<tr><td colspan="5" style="text-align: center; padding: 20px; color: var(--text-muted);">No valid points available.</td></tr>`;
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
  if (sampleTbody) sampleTbody.innerHTML = html;
}

// Watch visibility changes for LiDAR polling triggers
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    stopLidarPolling();
  } else {
    updateLidarTabState();
  }
});

// Periodic stale data check
setInterval(() => {
  const overlay = document.getElementById('lidar-stale-overlay');
  if (overlay && activeTopTabId === 'tab-sensors-v2' && lastScanTime > 0 && Date.now() - lastScanTime > 1000) {
    overlay.style.display = 'flex';
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
  
  // 4. Repeatability Session Statistics & Recommended Run HUD
  const logs = db.testLogs || [];
  const elPassRate = document.getElementById('rep-val-passrate');
  const elCount = document.getElementById('rep-val-count');
  const elDistMean = document.getElementById('rep-val-dist-mean');
  const elDistStd = document.getElementById('rep-val-dist-std');
  const elYawMean = document.getElementById('rep-val-yaw-mean');
  const elYawStd = document.getElementById('rep-val-yaw-std');
  const elRecFwd = document.getElementById('rep-rec-fwd');
  const elRecLeft = document.getElementById('rep-rec-left');
  const elRecRight = document.getElementById('rep-rec-right');
  const elRecTotal = document.getElementById('rep-rec-total');

  if (logs.length === 0) {
    if (elPassRate) elPassRate.textContent = 'Pass Rate: --%';
    if (elCount) elCount.textContent = '0';
    if (elDistMean) elDistMean.textContent = '--';
    if (elDistStd) elDistStd.textContent = '--';
    if (elYawMean) elYawMean.textContent = '--';
    if (elYawStd) elYawStd.textContent = '--';
    if (elRecFwd) elRecFwd.textContent = '0/5';
    if (elRecLeft) elRecLeft.textContent = '0/5';
    if (elRecRight) elRecRight.textContent = '0/5';
    if (elRecTotal) elRecTotal.textContent = '0/15';
  } else {
    const totalCount = logs.length;
    const passedCount = logs.filter(l => l.pass === true).length;
    const passRate = ((passedCount / totalCount) * 100).toFixed(1);

    const fwdLogs = logs.filter(l => (l.test || l.testType) === 'forward_1m');
    const fwdDistErrors = fwdLogs.map(l => l.distanceError !== undefined ? l.distanceError : (l.reportedDistance || 0));
    const fwdDistMean = fwdDistErrors.length > 0 ? (fwdDistErrors.reduce((a, b) => a + b, 0) / fwdDistErrors.length) : 0;
    const fwdDistStd = fwdDistErrors.length > 0 ? Math.sqrt(fwdDistErrors.reduce((a, b) => a + Math.pow(b - fwdDistMean, 2), 0) / fwdDistErrors.length) : 0;

    const turnLogs = logs.filter(l => (l.test || l.testType) === 'turn_left_90' || (l.test || l.testType) === 'turn_right_90');
    const turnYawErrors = turnLogs.map(l => l.yawErrorDegrees !== undefined ? l.yawErrorDegrees : (l.reportedYawDegrees || 0));
    const turnYawMean = turnYawErrors.length > 0 ? (turnYawErrors.reduce((a, b) => a + b, 0) / turnYawErrors.length) : 0;
    const turnYawStd = turnYawErrors.length > 0 ? Math.sqrt(turnYawErrors.reduce((a, b) => a + Math.pow(b - turnYawMean, 2), 0) / turnYawErrors.length) : 0;

    // Recommended counters ONLY count target_reached (pass=true) successful runs
    const fwdSuccessCount = fwdLogs.filter(l => l.stopReason === 'target_reached' || l.pass === true).length;
    const leftSuccessCount = logs.filter(l => (l.test || l.testType) === 'turn_left_90' && (l.stopReason === 'target_reached' || l.pass === true)).length;
    const rightSuccessCount = logs.filter(l => (l.test || l.testType) === 'turn_right_90' && (l.stopReason === 'target_reached' || l.pass === true)).length;
    const totalSuccessCount = fwdSuccessCount + leftSuccessCount + rightSuccessCount;

    if (elPassRate) elPassRate.textContent = `Pass Rate: ${passRate}%`;
    if (elCount) elCount.textContent = `${totalCount}`;
    if (elDistMean) elDistMean.textContent = fwdLogs.length > 0 ? `${fwdDistMean >= 0 ? '+' : ''}${fwdDistMean.toFixed(4)}m` : '--';
    if (elDistStd) elDistStd.textContent = fwdLogs.length > 0 ? `±${fwdDistStd.toFixed(4)}m` : '--';
    if (elYawMean) elYawMean.textContent = turnLogs.length > 0 ? `${turnYawMean >= 0 ? '+' : ''}${turnYawMean.toFixed(2)}°` : '--';
    if (elYawStd) elYawStd.textContent = turnLogs.length > 0 ? `±${turnYawStd.toFixed(2)}°` : '--';
    if (elRecFwd) elRecFwd.textContent = `${fwdSuccessCount}/5${fwdSuccessCount >= 5 ? ' ✓' : ''}`;
    if (elRecLeft) elRecLeft.textContent = `${leftSuccessCount}/5${leftSuccessCount >= 5 ? ' ✓' : ''}`;
    if (elRecRight) elRecRight.textContent = `${rightSuccessCount}/5${rightSuccessCount >= 5 ? ' ✓' : ''}`;
    if (elRecTotal) elRecTotal.textContent = `${totalSuccessCount}/15${totalSuccessCount >= 15 && fwdSuccessCount >= 5 && leftSuccessCount >= 5 && rightSuccessCount >= 5 ? ' ✓ COMPLETE' : ''}`;
  }

  // 5. History Logs Table
  const tbody = document.getElementById('cal-history-table-body');
  if (tbody) {
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
        } else {
          const passBadge = log.pass === true ? '<span style="color:#10b981; font-weight:bold;">[PASS]</span>' : (log.pass === false ? '<span style="color:#ef4444; font-weight:bold;">[FAULT]</span>' : '');
          const distStr = log.reportedDistance !== undefined ? `${log.reportedDistance.toFixed(3)}m` : '--';
          const yawStr = log.reportedYawDegrees !== undefined ? `${log.reportedYawDegrees.toFixed(1)}°` : '--';
          const reasonStr = log.stopReason ? log.stopReason : (log.fault || '--');
          summary = `${passBadge} Dist: ${distStr}, Yaw: ${yawStr}, Reason: ${reasonStr}`;
        }
        const testName = log.test || log.testType || 'unknown';
        return `
          <tr style="border-bottom: 1px solid var(--border);">
            <td style="padding: 8px 10px; font-family: monospace; white-space: nowrap;">${dateStr}</td>
            <td style="padding: 8px 10px; font-weight: bold; color: var(--cyan-glow);">${testName}</td>
            <td style="padding: 8px 10px;">${log.surfaceType || 'unknown'}</td>
            <td style="padding: 8px 10px; font-family: monospace;">${log.firmwareVersion || '1.3.0'}</td>
            <td style="padding: 8px 10px; color: var(--text-muted); font-size: 11px;">${summary}</td>
          </tr>
        `;
      }).join('');
    }
  }
}

function clearRepeatabilityHistory() {
  if (confirm('Are you sure you want to clear repeatability session history?')) {
    fetch('/api/calibration/repeatability/clear', { method: 'POST' })
      .then(res => res.json())
      .then(data => {
        if (data.ok) {
          logSystem('[Calibration] Repeatability history cleared.');
          if (calibrationDatabase) {
            calibrationDatabase.testLogs = [];
            updateCalibrationDbUI(calibrationDatabase);
          }
        }
      })
      .catch(err => console.error('Error clearing history:', err));
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
// ── Backtracking Recording & Return ──
async function startPathRecording() {
  try {
    const result = await authenticatedFetch('/api/path/record/start', { method: 'POST' });
    logSystem(`[Path] Recording started: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to start path recording:', err);
  }
}

async function stopPathRecording() {
  try {
    const result = await authenticatedFetch('/api/path/record/stop', { method: 'POST' });
    logSystem(`[Path] Recording stopped: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to stop path recording:', err);
  }
}

async function startBacktrackHome() {
  try {
    const result = await authenticatedFetch('/api/path/backtrack/start', { method: 'POST' });
    logSystem(`[Path] Backtracking started: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to start backtrack:', err);
  }
}

async function abortBacktrackHome() {
  try {
    const result = await authenticatedFetch('/api/path/backtrack/stop', { method: 'POST' });
    logSystem(`[Path] Backtracking aborted: ${JSON.stringify(result.json || {})}`);
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
    const result = await authenticatedFetch('/api/calibration/simulate/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ safetyAck: true })
    });
    logSystem(`[Calibration Simulation] Started: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to start calibration simulation:', err);
  }
}

async function triggerCalibrateCancel() {
  try {
    const result = await authenticatedFetch('/api/calibration/abort', { method: 'POST' });
    logSystem(`[Calibration] Aborted: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to abort calibration:', err);
  }
}

async function triggerRealCalibrateStart() {
  try {
    const result = await authenticatedFetch('/api/calibration/real/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ safetyAck: true })
    });
    logSystem(`[Real Calibration] Started: ${JSON.stringify(result.json || {})}`);
  } catch (err) {
    console.error('Failed to start real calibration:', err);
  }
}

// ── Maintenance Mode ──
async function runSingleMotorTest(motorIndex, direction) {
  const chk = document.getElementById('maint-safety-chk');
  if (chk && !chk.checked) {
    alert('Please confirm that the rover is physically raised and supported with all wheels off the ground.');
    showTestErrorBanner('Safety acknowledgement required: Confirm rover is physically raised off the floor.');
    return;
  }

  const buttons = document.querySelectorAll('#btn-m1-fwd, #btn-m1-rev, #btn-m2-fwd, #btn-m2-rev, #btn-m3-fwd, #btn-m3-rev, #btn-m4-fwd, #btn-m4-rev');
  buttons.forEach(b => { if (b) b.disabled = true; });

  try {
    logSystem(`[Maintenance Test] Starting single motor test: m${motorIndex + 1} ${direction}...`);
    const result = await authenticatedFetch('/api/maintenance/run_test', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        safetyAck: true,
        motorIndex: motorIndex,
        direction: direction,
        output: 50,
        durationSec: 2.0
      })
    });
    const data = result.json || {};
    if (result.ok && data.ok && data.test_result) {
      updateMaintenanceHUD(data.test_result);
      logSystem(`[Maintenance Test] Test complete for ${data.test_result.motor_label}: Delta=${data.test_result.encoder_delta} ticks, Steady=${data.test_result.encoder_steady}, Isolation=${data.test_result.isolation_verified}`);
    } else if (!result.ok && result.status !== 401 && result.status !== 403) {
      showTestErrorBanner(`Maintenance test failed: ${data.error || 'Unknown error'}`);
      logSystem(`[Maintenance Test ERROR] ${data.error || 'Unknown error'}`);
    }
  } catch (err) {
    console.error('Failed to run single motor maintenance test:', err);
    showTestErrorBanner(`API / Network error: ${err.message}`);
  } finally {
    buttons.forEach(b => { if (b) b.disabled = false; });
  }
}

async function stopAllMaintenance() {
  try {
    const result = await authenticatedFetch('/api/maintenance/exit', { method: 'POST' });
    logSystem(`[Maintenance Test] STOP ALL triggered: ${JSON.stringify(result.json || {})}`);
    showTestErrorBanner('🛑 EMERGENCY STOP ACTIVATED — All outputs locked.');
  } catch (err) {
    console.error('Failed to stop all maintenance:', err);
  }
}

function showTestErrorBanner(msg) {
  const banner = document.getElementById('maint-test-error-banner');
  const msgEl = document.getElementById('maint-test-error-msg');
  if (banner && msgEl) {
    msgEl.innerText = msg;
    banner.style.display = 'block';
  }
}

function clearTestResult() {
  const ids = [
    'maint-test-motor', 'maint-test-label', 'maint-test-cmd', 'maint-test-time',
    'maint-test-autostop', 'maint-test-armed', 'maint-test-delta-enc',
    'maint-test-steady', 'maint-test-isolation'
  ];
  ids.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.innerText = '--';
      el.style.color = '';
    }
  });

  const motorKeys = ['m1', 'm2', 'm3', 'm4'];
  motorKeys.forEach(m => {
    ['enc-start-', 'enc-end-', 'enc-delta-', 'enc-iso-'].forEach(prefix => {
      const el = document.getElementById(prefix + m);
      if (el) {
        el.innerText = '--';
        el.style.color = '';
      }
    });
  });

  const banner = document.getElementById('maint-test-error-banner');
  if (banner) banner.style.display = 'none';
}

function updateMaintenanceHUD(res) {
  if (!res) return;

  const mMotor = document.getElementById('maint-test-motor');
  const mLabel = document.getElementById('maint-test-label');
  const mCmd = document.getElementById('maint-test-cmd');
  const mDelta = document.getElementById('maint-test-delta-enc');
  const mTime = document.getElementById('maint-test-time');
  const mAutoStop = document.getElementById('maint-test-autostop');
  const mArmed = document.getElementById('maint-test-armed');
  const mSteady = document.getElementById('maint-test-steady');
  const mIso = document.getElementById('maint-test-isolation');
  const banner = document.getElementById('maint-test-error-banner');
  const errorMsg = document.getElementById('maint-test-error-msg');

  if (mMotor) mMotor.innerText = res.selected_motor.toUpperCase();
  if (mLabel) mLabel.innerText = res.motor_label;
  if (mCmd) mCmd.innerText = `${res.commanded_pwm} PWM (${res.direction})`;
  if (mTime) mTime.innerText = `${res.elapsed_test_time_sec}s`;
  if (mAutoStop) {
    mAutoStop.innerText = 'CONFIRMED (Motor PWM = 0)';
    mAutoStop.style.color = '#10b981';
  }
  if (mArmed) {
    mArmed.innerText = 'Disarmed';
    mArmed.style.color = '#6b7280';
  }

  if (mDelta) {
    const dVal = res.encoder_delta || 0;
    mDelta.innerText = `${dVal > 0 ? '+' : ''}${dVal} ticks`;
    mDelta.style.color = Math.abs(dVal) >= 10 ? '#10b981' : '#ef4444';
  }

  if (mSteady) {
    mSteady.innerText = res.encoder_steady ? 'PASS (Active >= 10 ticks)' : 'FAIL (Inactive < 10 ticks)';
    mSteady.style.color = res.encoder_steady ? '#10b981' : '#ef4444';
  }

  if (mIso) {
    mIso.innerText = res.isolation_verified ? 'PASS (Isolation Verified)' : 'FAIL (Cross-motor movement > 5 ticks)';
    mIso.style.color = res.isolation_verified ? '#10b981' : '#ef4444';
  }

  // Populate 4-wheel snapshot table
  const motorKeys = ['m1', 'm2', 'm3', 'm4'];
  const targetKey = res.selected_motor;
  const unselectedDeltas = res.unselected_motor_deltas || {};

  motorKeys.forEach((key, idx) => {
    const elStart = document.getElementById(`enc-start-${key}`);
    const elEnd = document.getElementById(`enc-end-${key}`);
    const elDelta = document.getElementById(`enc-delta-${key}`);
    const elIso = document.getElementById(`enc-iso-${key}`);

    const startVal = key === targetKey ? res.starting_encoder_count : (res.start_encoders ? res.start_encoders[key] : 0);
    const endVal = key === targetKey ? res.ending_encoder_count : (res.end_encoders ? res.end_encoders[key] : 0);
    const deltaVal = key === targetKey ? res.encoder_delta : (unselectedDeltas[key] !== undefined ? unselectedDeltas[key] : 0);

    if (elStart) elStart.innerText = startVal !== undefined ? startVal : '--';
    if (elEnd) elEnd.innerText = endVal !== undefined ? endVal : '--';
    if (elDelta) {
      elDelta.innerText = `${deltaVal > 0 ? '+' : ''}${deltaVal}`;
      if (key === targetKey) {
        elDelta.style.color = Math.abs(deltaVal) >= 10 ? '#10b981' : '#ef4444';
      } else {
        elDelta.style.color = Math.abs(deltaVal) <= 5 ? '#a7f3d0' : '#ef4444';
      }
    }

    if (elIso) {
      if (key === targetKey) {
        elIso.innerText = 'TARGET (Active)';
        elIso.style.color = '#38bdf8';
      } else {
        const isPass = Math.abs(deltaVal) <= 5;
        elIso.innerText = isPass ? 'PASS (<= 5 ticks)' : 'FAIL (> 5 ticks)';
        elIso.style.color = isPass ? '#10b981' : '#ef4444';
      }
    }
  });

  // Diagnostic Error Banner check
  const failures = [];
  if (!res.encoder_steady) failures.push(`Target motor ${targetKey.toUpperCase()} did not move significantly (delta ${res.encoder_delta} < 10 ticks).`);
  if (!res.isolation_verified) failures.push('Cross-motor movement detected: unselected motor moved beyond 5 ticks tolerance.');

  if (failures.length > 0) {
    if (banner && errorMsg) {
      errorMsg.innerText = failures.join(' ');
      banner.style.display = 'block';
    }
  } else {
    if (banner) banner.style.display = 'none';
  }
}

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
// Gamepad Controller Integration (Canonical Drive Ownership)
// ==============================================================================
let gamepadIndex = null;
let gamepadActive = false;
let gamepadLoopRunning = false;
let lastSentJoystick = { x: 0, y: 0, deadman: false };
let lastGamepadSendTime = 0;

function checkGamepadConnection() {
  const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < gamepads.length; i++) {
    if (gamepads[i]) {
      if (gamepadIndex !== i || !gamepadActive) {
        gamepadIndex = i;
        gamepadActive = true;
        updateGamepadBadge(true, gamepads[i].id);
        logSystem(`Gamepad detected: ${gamepads[i].id} at index ${i}`);
        startGamepadLoop();
      }
      return true;
    }
  }
  return false;
}

window.addEventListener('gamepadconnected', (e) => {
  logSystem(`Gamepad connected event: ${e.gamepad.id} at index ${e.gamepad.index}`);
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
    gamepadLoopRunning = false;
    updateGamepadBadge(false);
    updateGamepadHUD(0, 0, false, "None");
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

  if (window.roverState && window.roverState.connection) {
    window.roverState.connection.gamepad = connected;
  }

  const elGp = document.getElementById('v2-drive-val-gamepad');
  if (elGp) {
    elGp.textContent = connected ? 'CONNECTED' : 'DISCONNECTED';
    elGp.style.color = connected ? '#10b981' : '#9ca3af';
  }
}

function updateGamepadHUD(x, y, deadman, pressedButtonsStr, rawX = 0, rawY = 0, rtRaw = 0, txStatus = 'IDLE', modelName = '') {
  const elCtrl = document.getElementById('gp-live-controller');
  if (elCtrl) {
    elCtrl.innerText = gamepadActive ? (modelName || 'Connected') : 'Disconnected';
    elCtrl.style.color = gamepadActive ? '#00f2fe' : '#9ca3af';
  }
  const elArm = document.getElementById('gp-live-arm');
  if (elArm) {
    const isArmed = window.roverState && window.roverState.drive && window.roverState.drive.armed;
    elArm.innerText = isArmed ? 'ARMED' : 'DISARMED';
    elArm.style.color = isArmed ? '#10b981' : '#6b7280';
  }
  const elDeadman = document.getElementById('gp-live-deadman');
  if (elDeadman) {
    if (gamepadActive && !deadman) {
      elDeadman.innerText = `RELEASED (RT Raw: ${rtRaw.toFixed(2)})`;
      elDeadman.style.color = '#f59e0b'; // Amber
    } else if (deadman) {
      elDeadman.innerText = `ACTIVE (RT Raw: ${rtRaw.toFixed(2)})`;
      elDeadman.style.color = '#10b981'; // Green
    } else {
      elDeadman.innerText = 'RELEASED (RT Raw: 0.00)';
      elDeadman.style.color = '#ef4444'; // Red
    }
  }
  const elLinear = document.getElementById('gp-live-linear');
  if (elLinear) {
    elLinear.innerText = `${y.toFixed(2)} (Raw: ${rawY.toFixed(2)})`;
  }
  const elAngular = document.getElementById('gp-live-angular');
  if (elAngular) {
    elAngular.innerText = `${x.toFixed(2)} (Raw: ${rawX.toFixed(2)})`;
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
  const elEstop = document.getElementById('gp-live-estop');
  if (elEstop) {
    const isEstop = window.roverState && window.roverState.drive && window.roverState.drive.estop;
    elEstop.innerText = isEstop ? 'TRIGGERED' : 'NOMINAL';
    elEstop.style.color = isEstop ? '#ef4444' : '#10b981';
  }
  const elTx = document.getElementById('gp-live-tx-status');
  if (elTx) {
    elTx.innerText = txStatus;
    elTx.style.color = txStatus === 'TRANSMITTED' ? '#10b981' : (txStatus.startsWith('SUPPRESSED') ? '#f59e0b' : '#9ca3af');
  }
}

function startGamepadLoop() {
  if (gamepadLoopRunning) return;
  gamepadLoopRunning = true;

  function poll() {
    if (!gamepadActive || gamepadIndex === null) {
      if (!checkGamepadConnection()) {
        gamepadLoopRunning = false;
        return;
      }
    }

    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gamepads[gamepadIndex];
    if (!gp) {
      gamepadLoopRunning = false;
      if (checkGamepadConnection()) {
        requestAnimationFrame(poll);
      }
      return;
    }

    // Standard Xbox Controller mapping:
    // Steering: axes[0]
    // Throttle: -axes[1] (Forward stick is negative axes[1])
    const rawX = (gp.axes && gp.axes[0] !== undefined) ? gp.axes[0] : 0;
    const rawY = (gp.axes && gp.axes[1] !== undefined) ? gp.axes[1] : 0;

    let throttle = -rawY;
    let turn = rawX;

    // Apply deadzone (0.10) to block resting drift
    if (Math.abs(throttle) < 0.10) throttle = 0;
    if (Math.abs(turn) < 0.10) turn = 0;

    // Right Trigger (buttons[7]) Deadman evaluation (analog value > 0.5 or pressed)
    const rtVal = (gp.buttons && gp.buttons[7] && typeof gp.buttons[7].value === 'number') ? gp.buttons[7].value : 0;
    const rtPressed = Boolean(gp.buttons && gp.buttons[7] && (gp.buttons[7].pressed || rtVal > 0.5));
    const rbPressed = Boolean(gp.buttons && gp.buttons[5] && (gp.buttons[5].pressed || gp.buttons[5].value > 0.5));
    const deadmanPressed = Boolean(rtPressed || rbPressed);

    // Detect pressed buttons
    const pressedButtons = [];
    if (gp.buttons && gp.buttons.length) {
      gp.buttons.forEach((btn, idx) => {
        if (btn && (btn.pressed || btn.value > 0.5)) {
          pressedButtons.push(idx);
        }
      });
    }

    // Translate buttons to name tags for telemetry HUD
    const buttonNames = [];
    pressedButtons.forEach(btnIdx => {
      if (btnIdx === 0) buttonNames.push("A (ESTOP)");
      else if (btnIdx === 1) buttonNames.push("B (ESTOP)");
      else if (btnIdx === 5) buttonNames.push("RB (Deadman)");
      else if (btnIdx === 7) buttonNames.push(`RT (${rtVal.toFixed(2)})`);
      else if (btnIdx === 8) buttonNames.push("Select (Disarm)");
      else if (btnIdx === 9) buttonNames.push("Start (Arm)");
      else buttonNames.push(btnIdx);
    });
    const pressedButtonsStr = buttonNames.length > 0 ? buttonNames.join(", ") : "None";

    // Safety buttons: A (0) or B (1) triggers ESTOP
    const estopPressed = Boolean(
      (gp.buttons[0] && (gp.buttons[0].pressed || gp.buttons[0].value > 0.5)) ||
      (gp.buttons[1] && (gp.buttons[1].pressed || gp.buttons[1].value > 0.5))
    );

    // Arm/Disarm triggers: Start (9) arms, Select (8) disarms
    const armPressed = Boolean(gp.buttons[9] && (gp.buttons[9].pressed || gp.buttons[9].value > 0.5));
    const disarmPressed = Boolean(gp.buttons[8] && (gp.buttons[8].pressed || gp.buttons[8].value > 0.5));

    let txStatus = 'IDLE';
    if (estopPressed) {
      txStatus = 'SUPPRESSED: E-Stop Active';
      if (typeof triggerEstop === 'function') triggerEstop();
      updateGamepadHUD(0, 0, deadmanPressed, pressedButtonsStr, rawX, rawY, rtVal, txStatus, gp.id);
      lastSentJoystick = { x: 0, y: 0, deadman: deadmanPressed };
      lastGamepadSendTime = Date.now();
    } else if (armPressed) {
      txStatus = 'ARM TRIGGERED';
      if (typeof armNormalDrive === 'function') armNormalDrive();
      lastGamepadSendTime = Date.now() + 500;
    } else if (disarmPressed) {
      txStatus = 'DISARM TRIGGERED';
      if (typeof disarmNormalDrive === 'function') disarmNormalDrive();
      lastGamepadSendTime = Date.now() + 500;
    }

    // Send joystick commands to server
    if (!estopPressed && !armPressed && !disarmPressed) {
      const now = Date.now();
      const isArmed = window.roverState && window.roverState.drive && window.roverState.drive.armed;

      const changed = Math.abs(turn - lastSentJoystick.x) > 0.01 ||
                      Math.abs(throttle - lastSentJoystick.y) > 0.01 ||
                      deadmanPressed !== lastSentJoystick.deadman;
      const timeElapsed = now - lastGamepadSendTime > 50;

      if (!isArmed && (turn !== 0 || throttle !== 0 || deadmanPressed)) {
        txStatus = 'SUPPRESSED: Disarmed';
      } else if (!deadmanPressed && (turn !== 0 || throttle !== 0)) {
        txStatus = 'SUPPRESSED: Deadman Released';
      } else if (throttle === 0 && turn === 0 && !deadmanPressed && !changed) {
        txStatus = 'SUPPRESSED: Neutral';
      }

      if (changed || (timeElapsed && (turn !== 0 || throttle !== 0 || deadmanPressed))) {
        sendServerMessage({
          type: 'joystick',
          x: turn,
          y: throttle,
          deadman: deadmanPressed
        });
        lastSentJoystick = { x: turn, y: throttle, deadman: deadmanPressed };
        lastGamepadSendTime = now;
        if (deadmanPressed && (turn !== 0 || throttle !== 0)) {
          txStatus = 'TRANSMITTED';
        }
      }
    }

    // Sync with HUD
    updateGamepadHUD(turn, throttle, deadmanPressed, pressedButtonsStr, rawX, rawY, rtVal, txStatus, gp.id);

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
      if (selectTrackWidth) selectTrackWidth.value = savedWidth;
    } else {
      if (selectTrackWidth) monitoredTrackWidth = parseFloat(selectTrackWidth.value);
    }
    selectTrackWidth.addEventListener('change', () => {
      if (selectTrackWidth) monitoredTrackWidth = parseFloat(selectTrackWidth.value);
      if (selectTrackWidth) localStorage.setItem('monitored_track_width', selectTrackWidth.value);
      drawLidarTestCanvas();
    });
  }


  function checkGates() {
    const rigid = chkRigid ? chkRigid.checked : false;
    const level = chkLevel ? chkLevel.checked : false;
    const btnStart = document.getElementById('btn-start-lidar-test');
    if (btnStart) {
      if (rigid && level && orientationVerified) {
        if (btnStart) btnStart.disabled = false;
        if (btnStart) btnStart.style.opacity = '1';
        if (btnStart) btnStart.style.cursor = 'pointer';
      } else {
        if (btnStart) btnStart.disabled = true;
        if (btnStart) btnStart.style.opacity = '0.6';
        if (btnStart) btnStart.style.cursor = 'not-allowed';
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
        
        const orientBadge = document.getElementById('orientation-verified-badge');
        if (orientBadge) {
          if (orientBadge) orientBadge.textContent = 'Verified';
          if (orientBadge) orientBadge.style.background = 'rgba(16, 185, 129, 0.15)';
          if (orientBadge) orientBadge.style.color = '#10b981';
          if (orientBadge) orientBadge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
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
    const orientBadge = document.getElementById('orientation-verified-badge');
    if (orientBadge) {
      if (orientBadge) orientBadge.textContent = 'Verified';
      if (orientBadge) orientBadge.style.background = 'rgba(16, 185, 129, 0.15)';
      if (orientBadge) orientBadge.style.color = '#10b981';
      if (orientBadge) orientBadge.style.border = '1px solid rgba(16, 185, 129, 0.4)';
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
    if (textDiv) textDiv.textContent = 'Step 1: Place a flat object exactly in front of the rover (0°).';
  } else if (orientationStep === 2) {
    if (textDiv) textDiv.textContent = 'Step 2: Place a flat object exactly to the left of the rover (90°).';
  } else if (orientationStep === 3) {
    if (textDiv) textDiv.textContent = 'Step 3: Place a flat object exactly behind the rover (180°).';
  } else if (orientationStep === 4) {
    if (textDiv) textDiv.textContent = 'Step 4: Place a flat object exactly to the right of the rover (270°).';
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
              if (rangeSpan) rangeSpan.textContent = `Live distance at ${targetAngle}°: ${minDist.toFixed(3)}m`;
            } else {
              if (rangeSpan) rangeSpan.textContent = `Live distance at ${targetAngle}°: No point detected`;
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
  activateTopTab('tab-calibration-v2');
  const section = document.getElementById('tab-calibration-v2');
  if (section) {
    section.scrollIntoView({ behavior: 'smooth' });
  }
}

// ────────────────────────────────────────────────────────────
// Closed-Loop Automatic Calibration UI Logic
// ────────────────────────────────────────────────────────────
var pendingAutoCalibTest = null;
let autoCalibPollingInterval = null;

function promptAutoCalib(testType) {
  pendingAutoCalibTest = testType;
  const modal = document.getElementById('modal-auto-calib-confirm');
  if (modal) modal.style.display = 'flex';
}

function closeAutoCalibModal() {
  pendingAutoCalibTest = null;
  const modal = document.getElementById('modal-auto-calib-confirm');
  if (modal) modal.style.display = 'none';
}

async function confirmAndStartAutoCalib() {
  if (!pendingAutoCalibTest) return;
  const testType = pendingAutoCalibTest;
  closeAutoCalibModal();

  try {
    const res = await fetch('/api/calibration/auto/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ test: testType })
    });
    const data = await res.json();
    if (!data.ok) {
      alert(`Failed to start test: ${data.error}`);
    } else {
      updateAutoCalibUI(data.status);
      startAutoCalibPolling();
    }
  } catch (err) {
    alert(`Error starting automatic calibration test: ${err.message}`);
  }
}

async function abortAutoCalibrationTest() {
  try {
    const res = await fetch('/api/calibration/auto/abort', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    });
    const data = await res.json();
    if (data.status) {
      updateAutoCalibUI(data.status);
    }
  } catch (err) {
    console.error('Error aborting auto calibration test:', err.message);
  }
}

function startAutoCalibPolling() {
  if (autoCalibPollingInterval) clearInterval(autoCalibPollingInterval);
  autoCalibPollingInterval = setInterval(async () => {
    try {
      const res = await fetch('/api/calibration/auto/status');
      const data = await res.json();
      if (data.ok && data.status) {
        updateAutoCalibUI(data.status);
        if (!data.status.active) {
          clearInterval(autoCalibPollingInterval);
          autoCalibPollingInterval = null;
        }
      }
    } catch (e) {}
  }, 200);
}

function updateCalibrationReadiness() {
  const badge = document.getElementById('v2-calib-readiness-badge');
  const elArmed = document.getElementById('v2-calib-readiness-armed');
  const elActive = document.getElementById('v2-calib-readiness-active');
  const elSerial = document.getElementById('v2-calib-readiness-serial');
  const elTelem = document.getElementById('v2-calib-readiness-telemetry');
  const elOdom = document.getElementById('v2-calib-readiness-odom');
  const elEstop = document.getElementById('v2-calib-readiness-estop');
  const elDia = document.getElementById('v2-calib-readiness-diameter');
  const elTrack = document.getElementById('v2-calib-readiness-trackwidth');
  const elTicks = document.getElementById('v2-calib-readiness-ticks');
  const banner = document.getElementById('v2-calib-readiness-banner');

  const st = window.roverState || {};
  const drv = st.drive || {};
  const conn = st.connection || {};

  const isArmed = !!drv.armed;
  const isAutoActive = !!(window.lastAutoCalibStatus && window.lastAutoCalibStatus.active);
  const serialOk = !!conn.serial;
  const wsOk = !!conn.ws;
  const estopActive = !!drv.estop;
  const telemAge = conn.telemAgeMs !== undefined ? conn.telemAgeMs : 9999;
  const odomAge = conn.odomAgeMs !== undefined ? conn.odomAgeMs : 9999;
  const telemOk = telemAge < 2000;
  const odomOk = odomAge < 2000;

  if (elArmed) {
    elArmed.textContent = isArmed ? 'ARMED' : 'Disarmed';
    elArmed.style.color = isArmed ? '#f59e0b' : '#10b981';
  }
  if (elActive) {
    elActive.textContent = isAutoActive ? 'ACTIVE' : 'Idle';
    elActive.style.color = isAutoActive ? '#f59e0b' : '#38bdf8';
  }
  if (elSerial) {
    elSerial.textContent = (serialOk && wsOk) ? 'Connected' : 'Disconnected';
    elSerial.style.color = (serialOk && wsOk) ? '#10b981' : '#ef4444';
  }
  if (elTelem) {
    elTelem.textContent = telemOk ? 'Fresh' : 'Stale';
    elTelem.style.color = telemOk ? '#10b981' : '#ef4444';
  }
  if (elOdom) {
    elOdom.textContent = odomOk ? 'Fresh' : 'Stale';
    elOdom.style.color = odomOk ? '#10b981' : '#ef4444';
  }
  if (elEstop) {
    elEstop.textContent = estopActive ? 'ACTIVE' : 'Clear';
    elEstop.style.color = estopActive ? '#ef4444' : '#10b981';
  }

  // Active parameter readouts
  const diaVal = currentWheelDiameter || 0.065;
  const trackVal = currentTrackWidth || 0.3408575433;
  const wheelDiaMm = diaVal * 1000.0;
  const trackMm = trackVal * 1000.0;
  const ticksPerRev = 1974.1666666667;

  if (elDia) elDia.textContent = `${diaVal.toFixed(3)} m (${wheelDiaMm.toFixed(1)} mm)`;
  if (elTrack) elTrack.textContent = `${trackVal.toFixed(3)} m (${trackMm.toFixed(1)} mm)`;
  if (elTicks) elTicks.textContent = `${ticksPerRev.toFixed(1)}`;

  // Also update Constants Readout section (#v2-calib-val-dia-m, etc.)
  const cDiaM = document.getElementById('v2-calib-val-dia-m');
  const cDiaMm = document.getElementById('v2-calib-val-dia-mm');
  const cTrackM = document.getElementById('v2-calib-val-track-m');
  const cTrackMm = document.getElementById('v2-calib-val-track-mm');
  const cTicks = document.getElementById('v2-calib-val-ticks');

  if (cDiaM) cDiaM.textContent = `${diaVal.toFixed(3)} m`;
  if (cDiaMm) cDiaMm.textContent = `${wheelDiaMm.toFixed(1)} mm`;
  if (cTrackM) cTrackM.textContent = `${trackVal.toFixed(3)} m`;
  if (cTrackMm) cTrackMm.textContent = `${trackMm.toFixed(1)} mm`;
  if (cTicks) cTicks.textContent = `${ticksPerRev.toFixed(1)}`;

  // Check overall readiness
  let issues = [];
  if (!wsOk || !serialOk) issues.push('Serial connection disconnected');
  if (!telemOk) issues.push('Telemetry stale or missing');
  if (!odomOk) issues.push('Odometry stale or missing');
  if (estopActive) issues.push('E-stop active');

  const isReady = issues.length === 0;

  if (badge) {
    badge.textContent = isReady ? 'READY' : 'NOT READY';
    badge.style.background = isReady ? '#10b981' : '#ef4444';
    badge.style.color = '#fff';
  }

  if (banner) {
    if (!isReady) {
      banner.style.display = 'block';
      banner.textContent = `⚠️ Safety Gate Inhibited: ${issues.join(' | ')}`;
    } else {
      banner.style.display = 'none';
    }
  }
}

function updateAutoCalibUI(status) {
  if (!status) return;
  window.lastAutoCalibStatus = status;
  updateCalibrationReadiness();

  const setText = (idOrClass, val) => {
    document.querySelectorAll('#' + idOrClass + ', .' + idOrClass).forEach(el => {
      el.textContent = val;
    });
  };

  // Disable start buttons while test is active
  document.querySelectorAll('#btn-auto-fwd-1m, .btn-auto-fwd-1m, #btn-auto-turn-left, .btn-auto-turn-left, #btn-auto-turn-right, .btn-auto-turn-right').forEach(btn => {
    btn.disabled = !!status.active;
  });

  document.querySelectorAll('#auto-calib-active-banner, .auto-calib-active-banner').forEach(banner => {
    banner.style.display = status.active ? 'block' : 'none';
  });

  document.querySelectorAll('#auto-calib-hud-phase, .auto-calib-hud-phase').forEach(hudPhase => {
    hudPhase.textContent = status.phase || 'IDLE';
    hudPhase.style.background = status.active ? '#ef4444' : (status.lastResult && status.lastResult.pass ? '#10b981' : (status.lastResult && status.lastResult.pass === false ? '#f59e0b' : 'rgba(255,255,255,0.1)'));
  });

  setText('auto-calib-val-test', status.test || 'None');
  setText('auto-calib-val-elapsed', `${((status.elapsedMs || 0) / 1000.0).toFixed(1)} s`);
  setText('auto-calib-val-dist', `${(status.reportedDistance || 0).toFixed(3)} m`);
  setText('auto-calib-val-yaw', `${(status.reportedYawDegrees || 0).toFixed(1)}°`);
  setText('auto-calib-val-ages', `${status.telemetryAgeMs || 0}ms / ${status.odomAgeMs || 0}ms`);
  setText('auto-calib-val-cmd', status.motorCommand ? JSON.stringify(status.motorCommand) : '[0, 0, 0, 0]');
  setText('auto-calib-val-reason', status.stopReason || 'None');

  // Render Result Card if test finished (lastResult is present)
  const res = status.lastResult;
  const showCard = (res !== null);
  document.querySelectorAll('#auto-calib-result-card, .auto-calib-result-card').forEach(resultCard => {
    resultCard.style.display = showCard ? 'block' : 'none';
  });

  if (showCard) {
    document.querySelectorAll('#auto-calib-result-badge, .auto-calib-result-badge').forEach(resBadge => {
      if (res.pass === true) {
        resBadge.textContent = 'PASS';
        resBadge.style.background = '#10b981';
        resBadge.style.color = '#fff';
      } else {
        resBadge.textContent = 'FAIL';
        resBadge.style.background = '#ef4444';
        resBadge.style.color = '#fff';
      }
    });

    setText('res-test-type', (res && res.test) ? res.test : (status.test || '-'));
    const startP = (res && res.startPose) || status.startPose;
    if (startP) {
      setText('res-start-pose', `(${startP.x.toFixed(3)}, ${startP.y.toFixed(3)}, ${startP.yawDeg.toFixed(1)}°)`);
    }
    const finalP = (res && res.finalPose) || status.finalPose;
    if (finalP) {
      setText('res-ending-pose', `(${finalP.x.toFixed(3)}, ${finalP.y.toFixed(3)}, ${finalP.yawDeg.toFixed(1)}°)`);
    }
    const rDist = (res && res.reportedDistance !== undefined) ? res.reportedDistance : (status.reportedDistance || 0);
    setText('res-measured-dist', `${rDist.toFixed(3)} m`);
    const rYaw = (res && res.reportedYawDegrees !== undefined) ? res.reportedYawDegrees : (status.reportedYawDegrees || 0);
    setText('res-measured-yaw', `${rYaw.toFixed(1)}°`);
    const dErr = (res && res.distanceError !== undefined) ? res.distanceError : (status.distanceError || 0);
    const yErr = (res && res.yawErrorDegrees !== undefined) ? res.yawErrorDegrees : (status.yawErrorDegrees || 0);
    setText('res-target-errors', `Dist Err: ${dErr > 0 ? '+' : ''}${dErr}m | Yaw Err: ${yErr > 0 ? '+' : ''}${yErr}°`);
    const elMs = (res && res.elapsedMs !== undefined) ? res.elapsedMs : (status.elapsedMs || 0);
    setText('res-elapsed-time', `${(elMs / 1000.0).toFixed(1)} s`);
    const sReason = (res && res.stopReason) || status.stopReason || '-';
    const sFault = (res && res.fault !== undefined) ? res.fault : status.fault;
    setText('res-stop-reason', `${sReason} ${sFault ? '(' + sFault + ')' : ''}`);
  }
}

// Query initial automatic calibration status on page load
document.addEventListener('DOMContentLoaded', () => {
  const uiBadge = document.getElementById('ui-version-badge');
  if (uiBadge) {
    uiBadge.textContent = `UI: ${UI_BUILD_ID}`;
  }

  updateEsp32Badge();

  const btnSaveToken = document.getElementById('btn-save-token');
  const btnClearToken = document.getElementById('btn-clear-token');
  const inputToken = document.getElementById('operator-token-input');
  const rememberCheckbox = document.getElementById('remember-token-checkbox');

  // Token restoration on page load: prefer sessionStorage, otherwise restore localStorage
  const sessionTok = sessionStorage.getItem('rover_operator_token');
  const localTok = localStorage.getItem('rover_operator_token');
  let activeTok = '';

  if (sessionTok && sessionTok.trim()) {
    activeTok = sessionTok.trim();
    if (rememberCheckbox) {
      rememberCheckbox.checked = !!(localTok && localTok.trim() && localTok.trim() === activeTok);
    }
  } else if (localTok && localTok.trim()) {
    activeTok = localTok.trim();
    if (rememberCheckbox) {
      rememberCheckbox.checked = true;
    }
    sessionStorage.setItem('rover_operator_token', activeTok);
  }

  if (inputToken && activeTok) {
    inputToken.value = activeTok;
    inputToken.placeholder = 'Operator Token (Active)';
  }

  if (rememberCheckbox) {
    rememberCheckbox.addEventListener('change', () => {
      const curTok = getOrSyncOperatorToken();
      if (curTok) {
        setStoredOperatorToken(curTok, rememberCheckbox.checked);
        if (typeof logSystem === 'function') {
          logSystem(rememberCheckbox.checked ? 'Operator token remembered in localStorage.' : 'Operator token stored in sessionStorage only.');
        }
      }
    });
  }

  if (inputToken) {
    inputToken.addEventListener('input', () => {
      const val = inputToken.value.trim();
      if (val) {
        setStoredOperatorToken(val, isRememberBrowserEnabled());
        inputToken.placeholder = 'Operator Token (Active)';
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: 'auth', token: val }));
        }
      } else {
        sessionStorage.removeItem('rover_operator_token');
        localStorage.removeItem('rover_operator_token');
        inputToken.placeholder = 'Operator Token';
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: 'deauth' }));
        }
      }
    });
  }

  if (btnSaveToken) {
    btnSaveToken.addEventListener('click', () => {
      const val = getOrSyncOperatorToken();
      if (val) {
        if (inputToken) inputToken.placeholder = 'Operator Token (Active)';
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: 'auth', token: val }));
        }
        const storageType = isRememberBrowserEnabled() ? 'localStorage' : 'sessionStorage';
        if (typeof logSystem === 'function') {
          logSystem(`Operator token stored in ${storageType}.`);
        }
      } else {
        showAuthErrorMessage('Enter an operator token first.');
      }
    });
  }

  if (btnClearToken) {
    btnClearToken.addEventListener('click', () => {
      sessionStorage.removeItem('rover_operator_token');
      localStorage.removeItem('rover_operator_token');
      if (inputToken) {
        inputToken.value = '';
        inputToken.placeholder = 'Operator Token';
      }
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'deauth' }));
      }
      if (typeof logSystem === 'function') {
        logSystem('Operator authorization cleared.');
      }
    });
  }

  updateCalibrationReadiness();
  fetch('/api/calibration/auto/status')
    .then(res => res.json())
    .then(data => {
      if (data && data.ok && data.status) {
        updateAutoCalibUI(data.status);
      }
    })
    .catch(() => {});

  const btnClearResult = document.getElementById('btn-clear-calib-result');
  if (btnClearResult) {
    btnClearResult.addEventListener('click', () => {
      authenticatedFetch('/api/calibration/auto/clear_result', { method: 'POST' })
        .then(result => {
          if (result.ok && result.json && result.json.ok && result.json.status) {
            updateAutoCalibUI(result.json.status);
          }
        });
    });
  }

  const btnAutonomy = document.getElementById('btn-enable-autonomy');
  if (btnAutonomy) {
    btnAutonomy.addEventListener('click', () => {
      const isCurrentlyEnabled = btnAutonomy.textContent.includes('Disable');
      const endpoint = isCurrentlyEnabled ? '/api/autonomy/disable' : '/api/autonomy/enable';
      authenticatedFetch(endpoint, { method: 'POST' })
        .then(result => {
          if (result.ok && result.json && result.json.ok && result.json.status) {
            updateAutonomyState(result.json.status);
          }
        })
        .catch(err => console.error('Autonomy toggle failed:', err));
    });
  }

  fetch('/api/autonomy/status')
    .then(res => res.json())
    .then(data => {
      if (data.ok) updateAutonomyState(data);
    })
    .catch(() => {});
});

function updateAutonomyState(status) {
  if (!status) return;
  const elDriveAutonomy = document.getElementById('v2-drive-val-autonomy');
  const btnAutonomy = document.getElementById('btn-enable-autonomy');
  const isArmed = (typeof latestNormalDriveStatus !== 'undefined' && latestNormalDriveStatus && latestNormalDriveStatus.armed) || false;

  if (elDriveAutonomy) {
    const st = (status.state || (status.enabled ? (status.active ? 'ACTIVE' : 'READY') : 'DISABLED')).toUpperCase();
    if (st === 'ACTIVE') {
      elDriveAutonomy.textContent = 'ACTIVE (ROS 2)';
      elDriveAutonomy.className = 'status-val badge-armed';
    } else if (st === 'READY_ARMED' || st === 'READY') {
      elDriveAutonomy.textContent = 'READY (ARMED)';
      elDriveAutonomy.className = 'status-val badge-armed';
    } else if (st === 'READY_DISARMED') {
      elDriveAutonomy.textContent = 'READY (DISARMED)';
      elDriveAutonomy.className = 'status-val badge-disarmed';
    } else if (st === 'WAITING_FOR_ZERO') {
      elDriveAutonomy.textContent = `WAITING FOR ZERO (${status.zeroHandshakeCount || 0}/3)`;
      elDriveAutonomy.className = 'status-val badge-disarmed';
    } else if (st === 'STALE') {
      elDriveAutonomy.textContent = 'STALE (TIMEOUT)';
      elDriveAutonomy.className = 'status-val badge-disarmed';
    } else if (st === 'FAULT') {
      elDriveAutonomy.textContent = 'FAULT';
      elDriveAutonomy.className = 'status-val badge-disarmed';
    } else {
      elDriveAutonomy.textContent = 'DISABLED';
      elDriveAutonomy.className = 'status-val badge-disarmed';
    }
  }

  if (btnAutonomy) {
    if (status.enabled) {
      btnAutonomy.textContent = 'Disable Autonomy';
      btnAutonomy.style.background = 'rgba(239, 68, 68, 0.2)';
      btnAutonomy.style.borderColor = '#ef4444';
      btnAutonomy.style.color = '#fca5a5';
      btnAutonomy.disabled = false;
      btnAutonomy.title = 'Click to disable ROS 2 autonomy intake';
    } else {
      btnAutonomy.textContent = 'Enable Autonomy';
      btnAutonomy.style.background = 'rgba(0, 242, 254, 0.15)';
      btnAutonomy.style.borderColor = 'rgba(0, 242, 254, 0.4)';
      btnAutonomy.style.color = '#00f2fe';
      if (isArmed) {
        btnAutonomy.disabled = true;
        btnAutonomy.style.opacity = '0.5';
        btnAutonomy.title = 'Cannot enable autonomy while rover is armed. Disarm first.';
      } else {
        btnAutonomy.disabled = false;
        btnAutonomy.style.opacity = '1.0';
        btnAutonomy.title = 'Enables ROS 2 autonomy intake (requires zero-velocity handshake before arming)';
      }
    }
  }
}

// --- Stage 4 Compact LiDAR Local View Renderer & Safety Lifecycle Listeners ---
function drawCompactLidarScan(scan) {
  const canvas = document.getElementById('v2-compact-lidar-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) return;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);

  const width = rect.width;
  const height = rect.height;
  const centerX = width / 2;
  const centerY = height / 2;
  const radius = Math.min(centerX, centerY) - 15;
  if (radius <= 0) return;

  const maxRangeMm = 3000.0;
  const scale = radius / maxRangeMm;

  // Clear background
  ctx.fillStyle = '#0b0f19';
  ctx.fillRect(0, 0, width, height);

  // Concentric rings (1m, 2m, 3m)
  ctx.strokeStyle = 'rgba(0, 242, 254, 0.12)';
  ctx.lineWidth = 1;
  [1000, 2000, 3000].forEach(rMm => {
    ctx.beginPath();
    ctx.arc(centerX, centerY, rMm * scale, 0, 2 * Math.PI);
    ctx.stroke();
  });

  // Crosshairs
  ctx.strokeStyle = 'rgba(0, 242, 254, 0.2)';
  ctx.beginPath();
  ctx.moveTo(centerX, centerY - radius); ctx.lineTo(centerX, centerY + radius);
  ctx.moveTo(centerX - radius, centerY); ctx.lineTo(centerX + radius, centerY);
  ctx.stroke();

  // Forward indicator (0 deg front)
  ctx.fillStyle = 'rgba(0, 242, 254, 0.8)';
  ctx.font = 'bold 9px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.fillText('0° FRONT', centerX, centerY - radius + 12);

  let closestDistMm = 999999;

  if (scan && scan.points && scan.points.length > 0) {
    for (const pt of scan.points) {
      if (pt.distanceMm > 0) {
        if (pt.distanceMm < closestDistMm) closestDistMm = pt.distanceMm;

        const angleRad = (pt.angleDeg - 90) * Math.PI / 180;
        const x = centerX + pt.distanceMm * scale * Math.cos(angleRad);
        const y = centerY + pt.distanceMm * scale * Math.sin(angleRad);

        if (pt.distanceMm * scale <= radius) {
          ctx.beginPath();
          ctx.arc(x, y, 2.0, 0, 2 * Math.PI);
          ctx.fillStyle = (pt.distanceMm < 300) ? '#ff0055' : '#00f2fe';
          ctx.fill();
        }
      }
    }
  }

  // Draw Center Rover Icon
  ctx.fillStyle = '#10b981';
  ctx.fillRect(centerX - 6, centerY - 8, 12, 16);
  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(centerX, centerY - 12);
  ctx.lineTo(centerX - 4, centerY - 6);
  ctx.lineTo(centerX + 4, centerY - 6);
  ctx.closePath();
  ctx.fill();

  // Update Closest Obstacle Readout HUD
  const elDist = document.getElementById('v2-compact-lidar-dist');
  const elStatus = document.getElementById('v2-compact-lidar-status');
  if (closestDistMm < 999999) {
    const closestM = (closestDistMm / 1000.0).toFixed(2);
    if (elDist) elDist.textContent = `${closestM} m`;
    if (elStatus) {
      if (closestDistMm < 300) {
        elStatus.textContent = '🛑 Danger (<0.3m)';
        elStatus.style.color = '#ef4444';
      } else if (closestDistMm < 500) {
        elStatus.textContent = '⚠️ Caution (<0.5m)';
        elStatus.style.color = '#f59e0b';
      } else {
        elStatus.textContent = '✓ Clear';
        elStatus.style.color = '#10b981';
      }
    }
  } else {
    if (elDist) elDist.textContent = '-- m';
    if (elStatus) {
      elStatus.textContent = 'Offline';
      elStatus.style.color = '#9ca3af';
    }
  }
}

// Stage 4 Safety Event Handlers: Window blur, visibility change, and gamepad disconnect immediately trigger safe stop
window.addEventListener('blur', () => {
  if (typeof driveRover === 'function') driveRover('stop');
  sendServerMessage({ type: 'joystick', x: 0, y: 0, deadman: false });
  lastSentJoystick = { x: 0, y: 0, deadman: false };
});

document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    if (typeof driveRover === 'function') driveRover('stop');
    sendServerMessage({ type: 'joystick', x: 0, y: 0, deadman: false });
    lastSentJoystick = { x: 0, y: 0, deadman: false };
  }
});

window.addEventListener('gamepaddisconnected', () => {
  if (typeof driveRover === 'function') driveRover('stop');
  sendServerMessage({ type: 'joystick', x: 0, y: 0, deadman: false });
  lastSentJoystick = { x: 0, y: 0, deadman: false };
});
