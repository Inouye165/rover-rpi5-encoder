const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');


// Load environment variables manually from .env if present
if (fs.existsSync(path.join(__dirname, '.env'))) {
  const envContent = fs.readFileSync(path.join(__dirname, '.env'), 'utf8');
  envContent.split(/\r?\n/).forEach(line => {
    const trimmed = line.trim();
    if (trimmed && !trimmed.startsWith('#') && trimmed.includes('=')) {
      const parts = trimmed.split('=');
      const key = parts[0].trim();
      const value = parts.slice(1).join('=').trim();
      process.env[key] = value;
    }
  });
}

// Calculate last modified time of server code
let serverUpdatedTime = 'unknown';
try {
  const stats = fs.statSync(__filename);
  const d = new Date(stats.mtime);
  const pad = n => String(n).padStart(2, '0');
  serverUpdatedTime = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ${pad(d.getHours())}:${pad(d.getMinutes())} PDT`;
} catch (err) {
  console.error("Error reading server update time:", err);
}

const PORT = process.env.PORT || 3000;
let COM_PORT = process.env.ROVER_ESP32_DEVICE || process.env.SERIAL_PORT;
if (process.platform === 'win32' && COM_PORT && COM_PORT.startsWith('/dev/')) {
  COM_PORT = null;
}
if (!COM_PORT) {
  if (process.platform === 'linux') {
    if (fs.existsSync('/dev/rover-esp32')) {
      COM_PORT = '/dev/rover-esp32';
    } else {
      console.error("ERROR: /dev/rover-esp32 is absent and no serial port is configured. Remaining disconnected.");
      COM_PORT = null;
    }
  } else {
    COM_PORT = 'COM18';
  }
}
const BAUD_RATE = parseInt(process.env.BAUD_RATE) || 115200;

// ────────────────────────────────────────────────────────────
// Expansion Board Binary Protocol Constants
// Source: Compatible Serial Protocol SDK
// ────────────────────────────────────────────────────────────
const HEAD       = 0xFF;         // Frame start byte (both directions)
const DEVICE_ID  = 0xFC;         // Host → Board device ID
const BOARD_ID   = 0xFB;         // Board → Host device ID (0xFC - 1)
const COMPLEMENT = 257 - DEVICE_ID; // = 1; used in checksum for outgoing cmds

// Outgoing function codes (Host → Board)
const FUNC_MOTOR  = 0x10; // Set individual motor speeds (-100..100 each)
const FUNC_MOTION = 0x12; // Set velocity (vx, vy, vz as int16 * 1000)
const FUNC_BEEP   = 0x02; // Buzzer
const FUNC_CAR_TYPE = 0x44; // Car type/motion mode
const FUNC_START_CALIBRATE = 0x20; // Start calibration command
const FUNC_CANCEL_CALIBRATE = 0x21; // Cancel calibration command
const FUNC_CLEAR_FAULTS = 0x22; // Clear safety faults command
const FUNC_GET_FIRMWARE_INFO = 0x23; // Get firmware identity command
const FUNC_RESET_TIMING_STATS = 0x24; // Reset control timing statistics
const FUNC_ENTER_MAINTENANCE = 0x26;
const FUNC_MAINTENANCE_SET_OUTPUT = 0x27;
const FUNC_EXIT_MAINTENANCE = 0x28;
const FUNC_EMERGENCY_STOP = 0x29;
const FUNC_ARM_NORMAL_DRIVE = 0x2C;
const FUNC_DISARM_NORMAL_DRIVE = 0x2D;

// Incoming telemetry type codes (Board → Host)
const TYPE_BATTERY  = 0x0A; // Speed/battery packet (data[6] = voltage*10)
const TYPE_ATTITUDE = 0x0C; // IMU attitude
const TYPE_ENCODER  = 0x0D; // Encoder counts: 4x int32 LE (M1..M4)
const TYPE_IMU      = 0x0E; // 9-axis IMU raw packet (21 bytes total)
const TYPE_CALIBRATION = 0x30; // Calibration status telemetry
const TYPE_FIRMWARE_INFO = 0x32; // Firmware identification telemetry
const TYPE_LOOP_TIMING = 0x33; // Control loop timing statistics telemetry
const TYPE_FAULT_REPORT = 0x34; // Active safety fault flags telemetry
const TYPE_MAINTENANCE = 0x35; // Maintenance status telemetry
const TYPE_NORMAL_DRIVE_STATUS = 0x36;

// NOTE: No COMPLEMENT constant needed – checksum = sum(packet[2..n-2]) & 0xFF

// ────────────────────────────────────────────────────────────
// Express / WebSocket Setup
// ────────────────────────────────────────────────────────────
const app    = express();
const server = http.createServer(app);
const wss    = new WebSocket.Server({ server });

app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

let serialPort        = null;
let isSerialConnecting = false;
let reconnectTimeout  = null;
let targetMotorSpeeds = [0, 0, 0, 0];
let motorLoopStarted = false;
let motorStopNeedsFlush = false;
const MOTOR_COMMAND_INTERVAL_MS = 120;
let motorProofRunning = false;
let encoderPacketCount = 0;
let lastEncoderActivityBroadcast = 0;
let lastTelemetryReceivedTime = 0;

// Position Control State
let positionMode = [false, false, false, false];
let targetPosition = [0, 0, 0, 0];
let currentTicks = [0, 0, 0, 0];
let autoTestStep = 0; // 0=idle, 1..6 (3 cycles of fwd/bwd)
let autoTestStartTicks = [0, 0, 0, 0];
let currentAutoTestSpeedLimit = 25; // Dynamic cycle limits: 25, 45, 65
let autoTestRunLogs = []; // Array of telemetry logs for data exports
const TICKS_PER_REV = 937.2;
const KP_POSITION = 0.15;
const MIN_POSITION_SPEED = 20;
const MAX_POSITION_SPEED = 60;

// Phase 4 Coordinated Normal Drive State
let latestNormalDriveStatus = null;
let lastDeadmanPressedTime = Date.now();
let reverseWaitStartTime = 0;
let targetLinear = 0.0;
let targetAngular = 0.0;
let measuredLinearVel = 0.0;
let measuredAngularVel = 0.0;
let driveCommandNeedsFlush = false;
let driveLoopStarted = false;
let latestCalibrationStatus = null;
let latestMaintenanceStatus = null;

// Phase 4 Coordinated Drive Odometry State
let odomX = 0.0;
let odomY = 0.0;
let odomYaw = 0.0;
let accumLeftDist = 0.0;
let accumRightDist = 0.0;
let lastOdomTicks = [null, null, null, null];
let lastOdomTime = null;
let WHEEL_RADIUS = 0.0325; // mutable wheel radius (synchronized with ESP32 NVS)
let TRACK_WIDTH = 0.170;  // mutable track width (synchronized with ESP32 NVS)

// Limits configuration and active command source
let floorTesting = false; // default to safe floor testing limits on startup
let cmdSource = 'NONE';
let deadmanPressed = false;

// Path Recording State
let recording = false;
let recordedPath = [];

// Backtracking Controller State
let backtracking = false;
let backtrackIndex = -1;

// Calibration parameters database
const CALIBRATION_DB_FILE = path.join(__dirname, 'calibration_db.json');
let calibrationDb = {
  currentConfig: { wheelDiameter: 0.065, effectiveTrackWidth: 0.170 },
  proposedConfig: { wheelDiameter: null, effectiveTrackWidth: null },
  previousConfig: { wheelDiameter: null, effectiveTrackWidth: null },
  testLogs: []
};

// LiDAR straight-line test variables
let lidarTestState = 'IDLE'; // 'IDLE', 'ZEROING', 'FORWARD_RUNNING', 'COMPLETE', 'ABORTED'
let lidarTestMode = 'correct'; // always correct+learn now
let lidarRigidConfirmed = false;
let lidarLevelConfirmed = false;
let orientationVerified = false;

// Fully automated multi-pass calibration
let autoCalibActive = false;       // true = deadman bypassed, rover drives itself
const AUTO_CALIB_SPEEDS = [0.11]; // m/s: med only
const AUTO_CALIB_SPEED_LABELS = ['MED'];
const AUTO_CALIB_PASSES_PER_TIER = 1;
let autoCalibSpeedTier = 0;        // 0=med only
let autoCalibPassIndex = 0;        // 0
let autoCalibTotalPass = 0;        // 0-0 overall
let tierPassSummaries = { SLOW: [], MED: [], FAST: [] };

// Leg stall/movement monitoring variables
let calibLegStartTime = 0;
let calibLastMoveTime = 0;
let calibLastX = 0;
let calibLastTicks = [0, 0, 0, 0];
let calibSpeedBoost = 0.0;
let interPassPauseStartTime = 0;
let returnHomeWaitStartTime = 0;

// Calibration & offset settings
let fwdAngleOffset = 0.0;
let lidarXOffset = 0.0127;
let lidarYOffset = 0.034925;
let lidarYawOffset = 0.0;
let maxCalibRange = 4.0;
let chassisMargin = 0.02;
let angleSectorMasks = ''; // raw string, e.g. "45-60,180-200"

// Path controller gains & limits
let headingGain = 0.8;      // K_p^psi
let lateralGain = 2.5;      // K_p^y (Increased to make lateral correction stronger)
let lateralKi = 0.4;        // K_i^y (Added to eliminate steady-state drift)
let lateralErrorSum = 0.0;  // accumulated lateral error
let correctionDeadband = 0.005; // 5mm
let maxAngularCorr = 0.35;   // rad/s limit
let corrSlewRate = 1.0;     // rad/s^2 limit
let minConfidence = 0.65;
let maxScanAgeMs = 300;

// Test variables
let lidarPose = { x: 0, y: 0, yaw: 0 };
let lidarMetrics = { confidence: 0, inliers: 0, rmse: 0, scanAgeMs: 0, status: 'idle', rejectionReason: '' };
let lastLidarPoseTime = null;
let lastOdomPose = { x: 0, y: 0, yaw: 0 }; // relative encoders path tracking
let odomStartTicks = [0, 0, 0, 0];
let lastCorrectionApplied = 0.0;
let lastPathControllerTime = null;

// Bounded session logs for passes
let testSessionId = '';
let testPassNumber = 0;
let testDirection = 'FORWARD';
let sessionLogs = [];
let passSummaries = [];

// Proposed trims
let activeFwdTrim = { left: 1.0, right: 1.0 };
let activeRevTrim = { left: 1.0, right: 1.0 };
let proposedFwdTrim = { left: 1.0, right: 1.0 };
let proposedRevTrim = { left: 1.0, right: 1.0 };
let acceptedPasses = 0;
let calibrationConfidence = 0.0;
let backupTrims = { fwd: { left: 1.0, right: 1.0 }, rev: { left: 1.0, right: 1.0 } };


function loadCalibrationDb() {
  try {
    if (fs.existsSync(CALIBRATION_DB_FILE)) {
      const data = fs.readFileSync(CALIBRATION_DB_FILE, 'utf8');
      calibrationDb = JSON.parse(data);
      console.log('[Config DB] Calibration database loaded successfully.');
      if (calibrationDb.currentConfig) {
        if (calibrationDb.currentConfig.wheelDiameter) {
          WHEEL_RADIUS = calibrationDb.currentConfig.wheelDiameter / 2.0;
        }
        if (calibrationDb.currentConfig.effectiveTrackWidth) {
          TRACK_WIDTH = calibrationDb.currentConfig.effectiveTrackWidth;
        }
        console.log(`[Config DB] Set active dimensions: radius=${WHEEL_RADIUS} m, track=${TRACK_WIDTH} m`);
      }
      if (calibrationDb.floorTesting !== undefined) {
        floorTesting = calibrationDb.floorTesting;
        console.log(`[Config DB] Set floorTesting limits from database: ${floorTesting}`);
      }
      if (calibrationDb.fwdTrim) {
        activeFwdTrim = { ...calibrationDb.fwdTrim };
        proposedFwdTrim = { ...calibrationDb.fwdTrim };
        console.log(`[Config DB] Loaded FWD trims from DB: L=${activeFwdTrim.left.toFixed(4)} R=${activeFwdTrim.right.toFixed(4)}`);
      }
      if (calibrationDb.revTrim) {
        activeRevTrim = { ...calibrationDb.revTrim };
        proposedRevTrim = { ...calibrationDb.revTrim };
        console.log(`[Config DB] Loaded REV trims from DB: L=${activeRevTrim.left.toFixed(4)} R=${activeRevTrim.right.toFixed(4)}`);
      }
    } else {
      saveCalibrationDb();
    }
  } catch (err) {
    console.error('[Config DB] Failed to load calibration database, using defaults:', err.message);
  }
}

function saveCalibrationDb() {
  try {
    calibrationDb.floorTesting = floorTesting;
    calibrationDb.fwdTrim = { left: activeFwdTrim.left, right: activeFwdTrim.right };
    calibrationDb.revTrim = { left: activeRevTrim.left, right: activeRevTrim.right };
    fs.writeFileSync(CALIBRATION_DB_FILE, JSON.stringify(calibrationDb, null, 2), 'utf8');
    console.log('[Config DB] Calibration database saved successfully.');
  } catch (err) {
    console.error('[Config DB] Failed to save calibration database:', err.message);
  }
}

// Convert float to 4 little-endian bytes array
function floatToLEBytes(f) {
  const buf = Buffer.alloc(4);
  buf.writeFloatLE(f, 0);
  return Array.from(buf);
}

loadCalibrationDb();

// ────────────────────────────────────────────────────────────
// IMU State (for sensor fusion)
// ────────────────────────────────────────────────────────────
let imuYaw = 0;
let lastImuTimestamp = null;



// ────────────────────────────────────────────────────────────
// LiDAR Sidecar Configuration
// ────────────────────────────────────────────────────────────
const LIDAR_STATUS_URL = process.env.LIDAR_STATUS_URL || 'http://127.0.0.1:3002/status';
const LIDAR_SCAN_URL   = process.env.LIDAR_SCAN_URL   || 'http://127.0.0.1:3002/scan';
const LIDAR_TEST_START_URL = process.env.LIDAR_TEST_START_URL || 'http://127.0.0.1:3002/test/start';
const LIDAR_TEST_POSE_URL  = process.env.LIDAR_TEST_POSE_URL  || 'http://127.0.0.1:3002/test/pose';
const LIDAR_TEST_STOP_URL  = process.env.LIDAR_TEST_STOP_URL  || 'http://127.0.0.1:3002/test/stop';

// ────────────────────────────────────────────────────────────
// WebSocket Broadcast Helper
// ────────────────────────────────────────────────────────────
function broadcast(data) {
  const message = JSON.stringify(data);
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(message);
    }
  });
}

// ────────────────────────────────────────────────────────────
// Binary Packet Builder (Host → Board)
//
// Frame layout: [HEAD=0xFF, DEVICE_ID=0xFC, extLen, funcId, ...payload, checksum]
//   extLen   = bytes remaining after the length byte = funcId(1) + payload(N) + checksum(1)
//   checksum = sum of all bytes from extLen onward (NOT including HEAD or DEVICE_ID) & 0xFF
//
// Verified against real firmware capture:
//   Beep 100ms → ff fc 05 02 64 00 6b
//   checksum = 05+02+64+00 = 107 = 0x6b ✓
// ────────────────────────────────────────────────────────────
function buildPacket(funcId, payload) {
  // Build: [HEAD, DEVICE_ID, extLen_placeholder, funcId, ...payload]
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  // extLen counts bytes from the extLen position to end (excluding HEAD and DEVICE_ID)
  cmd[2] = cmd.length - 2; 
  // Checksum: sum bytes starting at index 2 (extLen byte), stop before checksum
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) {
    sum += cmd[i];
  }
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

// Legacy checksum variant:
function buildPacketLegacyChecksum(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 2;
  let sum = COMPLEMENT;
  for (let i = 0; i < cmd.length; i++) {
    sum += cmd[i];
  }
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

function writePacket(packet, funcId, label = '') {
  if (!serialPort || !serialPort.isOpen) {
    return;
  }
  serialPort.write(packet, (err) => {
    if (err) {
      console.error('Serial write error:', err.message);
      broadcast({ type: 'raw_serial_out_err', error: err.message });
    } else {
      const hex = Array.from(packet).map(b => b.toString(16).padStart(2,'0')).join(' ');
      const prefix = label ? `${label} ` : '';
      console.log(`[Binary Out] ${prefix}${hex}`);
      broadcast({ type: 'raw_serial_out', data: `${prefix}[0x${funcId.toString(16).padStart(2,'0')}] ${hex}` });
    }
  });
}

// Convenience: Build and send a packet over the serial port
function sendBinaryCommand(funcId, payload, options = {}) {
  const dualChecksum = options.dualChecksum === true;
  if (!serialPort || !serialPort.isOpen) {
    console.warn(`Cannot send binary command 0x${funcId.toString(16).padStart(2,'0')} – port not open`);
    broadcast({ type: 'raw_serial_out_err', error: 'Serial port is closed' });
    return;
  }

  const packetPrimary = buildPacket(funcId, payload);
  writePacket(packetPrimary, funcId, dualChecksum ? '[ext]' : '');

  if (dualChecksum) {
    const packetLegacy = buildPacketLegacyChecksum(funcId, payload);
    if (!packetLegacy.equals(packetPrimary)) {
      setTimeout(() => writePacket(packetLegacy, funcId, '[legacy]'), 6);
    }
  }
}

// ────────────────────────────────────────────────────────────
// Motor Speed Command
// Frontend sends speeds in range -1000..1000 (integers)
// Board expects signed bytes -100..100
// ────────────────────────────────────────────────────────────
function clampMotor(val) {
  // Accept either UI range (-1000..1000) or board-native range (-100..100).
  const n = Number(val) || 0;
  const normalized = Math.abs(n) <= 100 ? n : (n / 10);
  return Math.max(-100, Math.min(100, Math.round(normalized)));
}

function sendMotorPacket(m1, m2, m3, m4) {
  // Convert signed byte values (can be negative) to unsigned byte representation
  function toSignedByte(v) {
    const c = clampMotor(v);
    return c < 0 ? (256 + c) : c;
  }
  // No swapping needed – index matches ESP32 board terminal index layout (M1=LF, M2=RF, M3=LR, M4=RR)
  const payload = [toSignedByte(m1), toSignedByte(m2), toSignedByte(m3), toSignedByte(m4)];
  // Send both checksum styles to maximize compatibility with firmware variants.
  sendBinaryCommand(FUNC_MOTOR, payload, { dualChecksum: true });
}

function sendMotorSpeeds(m1, m2, m3, m4) {
  targetMotorSpeeds = [m1, m2, m3, m4];
  const allZero = targetMotorSpeeds.every(v => clampMotor(v) === 0);
  if (allZero) {
    motorStopNeedsFlush = true;
  }
  sendMotorPacket(targetMotorSpeeds[0], targetMotorSpeeds[1], targetMotorSpeeds[2], targetMotorSpeeds[3]);
  broadcast({ type: 'motor_speeds', speeds: targetMotorSpeeds });
}

function startMotorKeepaliveLoop() {
  if (motorLoopStarted) return;
  motorLoopStarted = true;

  setInterval(() => {
    if (!serialPort || !serialPort.isOpen) {
      return;
    }

    const allZero = targetMotorSpeeds.every(v => clampMotor(v) === 0);
    if (!allZero) {
      sendMotorPacket(targetMotorSpeeds[0], targetMotorSpeeds[1], targetMotorSpeeds[2], targetMotorSpeeds[3]);
      return;
    }

    // Send one stop frame after transitioning to zero to ensure hard stop.
    if (motorStopNeedsFlush) {
      sendMotorPacket(0, 0, 0, 0);
      motorStopNeedsFlush = false;
    }
  }, MOTOR_COMMAND_INTERVAL_MS);
}

function startDriveKeepaliveLoop() {
  if (driveLoopStarted) return;
  driveLoopStarted = true;

  setInterval(() => {
    if (!serialPort || !serialPort.isOpen) return;

    // Guard: Do not send conflicting background velocity commands during position/autotests
    const isPositionActive = positionMode.some(m => m === true) || (autoTestStep > 0);
    if (isPositionActive) return;

    const isArmed = latestNormalDriveStatus && latestNormalDriveStatus.armed;
    if (!isArmed) return;

    const vx = Math.round(targetLinear * 1000);
    const vy = 0;
    const vz = Math.round(targetAngular * 1000);
    
    sendBinaryCommand(FUNC_MOTION, [
      ...int16ToLE(vx),
      ...int16ToLE(vy),
      ...int16ToLE(vz)
    ], { dualChecksum: true });
  }, 50);
}

function int16ToLE(value) {
  const v = value < 0 ? (65536 + value) : value;
  return [v & 0xFF, (v >> 8) & 0xFF];
}

function delay(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function runMotorProofSequence() {
  if (motorProofRunning) {
    broadcast({ type: 'motor_proof_status', status: 'busy', message: 'Motor proof already running.' });
    return;
  }

  if (!serialPort || !serialPort.isOpen) {
    broadcast({ type: 'motor_proof_status', status: 'error', message: 'Serial port is not connected.' });
    return;
  }

  motorProofRunning = true;
  const holdSpeeds = [...targetMotorSpeeds];

  try {
    // Pause keepalive target while proof sequence runs.
    targetMotorSpeeds = [0, 0, 0, 0];
    motorStopNeedsFlush = true;
    sendMotorPacket(0, 0, 0, 0);
    broadcast({ type: 'motor_proof_status', status: 'start', message: 'Starting motor proof sequence (about 20s).' });

    sendBinaryCommand(FUNC_BEEP, [200, 0], { dualChecksum: true });
    await delay(900);

    for (const carType of [1, 2, 3, 4]) {
      sendBinaryCommand(FUNC_CAR_TYPE, [carType, 0], { dualChecksum: true });
      broadcast({ type: 'motor_proof_status', status: 'step', message: `Car type ${carType} set.` });
      await delay(240);

      sendMotorPacket(100, 100, 100, 100);
      broadcast({ type: 'motor_proof_status', status: 'step', message: `Car type ${carType}: all motors +100` });
      await delay(1500);

      sendMotorPacket(0, 0, 0, 0);
      await delay(400);

      sendBinaryCommand(FUNC_MOTION, [...int16ToLE(600), ...int16ToLE(0), ...int16ToLE(0)], { dualChecksum: true });
      broadcast({ type: 'motor_proof_status', status: 'step', message: `Car type ${carType}: motion vX=0.6m/s` });
      await delay(1500);

      sendBinaryCommand(FUNC_MOTION, [...int16ToLE(0), ...int16ToLE(0), ...int16ToLE(0)], { dualChecksum: true });
      await delay(500);
    }

    sendMotorPacket(0, 0, 0, 0);
    broadcast({ type: 'motor_proof_status', status: 'done', message: 'Motor proof sequence complete.' });
  } catch (err) {
    console.error('Motor proof error:', err.message);
    broadcast({ type: 'motor_proof_status', status: 'error', message: `Motor proof failed: ${err.message}` });
  } finally {
    targetMotorSpeeds = holdSpeeds;
    if (targetMotorSpeeds.every(v => clampMotor(v) === 0)) {
      motorStopNeedsFlush = true;
    }
    motorProofRunning = false;
  }
}

// ────────────────────────────────────────────────────────────
// Telemetry Packet Parser
// Board streams: [0xFF, 0xFB, extLen, extType, ...data..., checksum]
// checksum = (sum of all bytes from extLen through last data byte) & 0xFF
// data payload = extLen - 2 bytes (excl. extType and checksum)
// ────────────────────────────────────────────────────────────
let rxBuf = Buffer.alloc(0);

function processRxBuffer() {
  // Extract and print any ASCII debug lines from the serial stream
  const newlineIdx = rxBuf.indexOf(10); // LF character (0x0A)
  if (newlineIdx !== -1) {
    const lineBytes = rxBuf.subarray(0, newlineIdx);
    if (lineBytes.indexOf(0xFF) === -1) {
      const txt = lineBytes.toString('utf8').trim();
      if (txt) {
        console.log(`[ESP32 Serial Text] ${txt}`);
        broadcast({ type: 'message', data: `[ESP32] ${txt}` });
      }
      rxBuf = rxBuf.subarray(newlineIdx + 1);
      return processRxBuffer();
    }
  }

  while (rxBuf.length >= 4) {
    // Search for 0xFF 0xFB header
    const h1 = rxBuf.indexOf(0xFF);
    if (h1 === -1) {
      rxBuf = Buffer.alloc(0);
      break;
    }
    if (h1 > 0) {
      rxBuf = rxBuf.subarray(h1);
      continue;
    }
    if (rxBuf[h1 + 1] !== BOARD_ID) {
      // Not a valid second header byte – skip one byte and try again
      rxBuf = rxBuf.subarray(h1 + 1);
      continue;
    }

    // We have the start of a potential packet at h1
    if (rxBuf.length < h1 + 3) break; // Need at least header + extLen

    const extLen  = rxBuf[h1 + 2];
    const totalLen = h1 + 2 + extLen; // header(2) + length byte(1) + extLen more bytes

    if (rxBuf.length < totalLen) break; // Wait for more data

    // Extract packet bytes
    const extType = rxBuf[h1 + 3];
    // Data payload = bytes after extType, before checksum = extLen - 2 bytes
    const dataLen = extLen - 2;
    if (dataLen < 0) {
      // Malformed – discard and advance
      rxBuf = rxBuf.subarray(h1 + 1);
      continue;
    }

    const dataBytes = rxBuf.subarray(h1 + 4, h1 + 4 + dataLen);
    const rxChecksum = rxBuf[totalLen - 1];

    // Validate checksum: sum of bytes from extLen through last data byte
    let checksum = 0;
    for (let i = h1 + 2; i < totalLen - 1; i++) {
      checksum += rxBuf[i];
    }
    checksum = checksum & 0xFF;

    if (checksum === rxChecksum) {
      parseTelemetryPacket(extType, dataBytes);
    } else {
      console.warn(`[Binary In] Checksum error! extType=0x${extType.toString(16)} calc=0x${checksum.toString(16)} recv=0x${rxChecksum.toString(16)}`);
      broadcast({ type: 'message', data: `[Checksum Error] type=0x${extType.toString(16)}` });
    }

    // Consume this packet and continue
    rxBuf = rxBuf.subarray(totalLen);
  }
}

// ────────────────────────────────────────────────────────────
// Telemetry Packet Interpreter
// ────────────────────────────────────────────────────────────
function parseTelemetryPacket(extType, data) {
  lastTelemetryReceivedTime = Date.now();
  if (extType === TYPE_BATTERY) {
    // Observed packet: ff fb 0a 0a 00 00 00 00 00 00 VV CS
    // data[] = 7 bytes payload.  data[6] = voltage * 10
    if (data.length >= 7) {
      const voltage = data[6] / 10.0;
      console.log(`[Battery] ${voltage.toFixed(1)} V`);
      broadcast({ type: 'battery', voltage });
    }

  } else if (extType === TYPE_IMU) {
    // Observed packet: ff fb 15 0e GX GX GY GY GZ GZ AX AX AY AY AZ AZ MX MX MY MY MZ MZ CS
    // Total=21 bytes → extLen=0x15=21 → data = extLen-2 = 19 bytes
    // Layout (little-endian int16):
    //   data[0..1]  = Gx raw   data[2..3]  = Gy raw   data[4..5]  = Gz raw
    //   data[6..7]  = Ax raw   data[8..9]  = Ay raw   data[10..11] = Az raw
    //   data[12..13]= Mx raw   data[14..15]= My raw   data[16..17] = Mz raw
    //   data[18]    = encoder/extra byte
    if (data.length < 12) {
      console.warn(`[IMU] Packet too short: ${data.length} bytes`);
      return;
    }

    // Gyroscope scaling from Go SDK: 1/3754.9 → rad/s
    const gyroRatio = 1 / 3754.9;
    const gx =  readInt16LE(data, 0) * gyroRatio;
    const gy = -readInt16LE(data, 2) * gyroRatio;
    const gz = -readInt16LE(data, 4) * gyroRatio;

    // Accelerometer scaling from Go SDK: 1/1000 → g
    const accelRatio = 1 / 1000.0;
    const ax = readInt16LE(data, 6)  * accelRatio;
    const ay = readInt16LE(data, 8)  * accelRatio;
    const az = readInt16LE(data, 10) * accelRatio;

    // Magnetometer (raw LSB)
    let mx = 0, my = 0, mz = 0;
    if (data.length >= 18) {
      mx = readInt16LE(data, 12);
      my = readInt16LE(data, 14);
      mz = readInt16LE(data, 16);
    }

    // ── Sensor Fusion: tilt angles from accelerometer ──
    const pitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI;
    const roll  = Math.atan2(ay,  Math.sqrt(ax * ax + az * az)) * 180 / Math.PI;

    // Integrate gyro Z for yaw
    const now = Date.now();
    if (lastImuTimestamp !== null) {
      const dt = Math.min((now - lastImuTimestamp) / 1000, 0.1);
      imuYaw += (gz * 180 / Math.PI) * dt;
      imuYaw = ((imuYaw + 180) % 360 + 360) % 360 - 180;
    }
    lastImuTimestamp = now;

    // Gyro in deg/s for display
    const gxDeg = gx * 180 / Math.PI;
    const gyDeg = gy * 180 / Math.PI;
    const gzDeg = gz * 180 / Math.PI;

    broadcast({
      type:  'imu',
      yaw:   parseFloat(imuYaw.toFixed(2)),
      pitch: parseFloat(pitch.toFixed(2)),
      roll:  parseFloat(roll.toFixed(2)),
      ax, ay, az,
      gx: gxDeg, gy: gyDeg, gz: gzDeg,
      mx, my, mz,
    });

  } else if (extType === TYPE_ATTITUDE) {
    // ff fb 09 0c -- roll, pitch, yaw as int16 LE (radians * 10000)
    if (data.length >= 6) {
      const toDeg = 180 / Math.PI;
      const roll  = (readInt16LE(data, 0) / 10000) * toDeg;
      const pitch = (readInt16LE(data, 2) / 10000) * toDeg;
      const yaw   = (readInt16LE(data, 4) / 10000) * toDeg;
      broadcast({
        type: 'attitude',
        roll:  parseFloat(roll.toFixed(2)),
        pitch: parseFloat(pitch.toFixed(2)),
        yaw:   parseFloat(yaw.toFixed(2)),
      });
    }

  } else if (extType === TYPE_ENCODER) {
    // ff fb 13 0d -- four int32 LE wheel encoder counts (M1..M4)
    if (data.length >= 16) {
      const m1 = data.readInt32LE(0);  // LF
      const m2 = data.readInt32LE(4);  // RF
      const m3 = data.readInt32LE(8);  // LR
      const m4 = data.readInt32LE(12); // RR

      currentTicks[0] = m1;
      currentTicks[1] = m2;
      currentTicks[2] = m3;
      currentTicks[3] = m4;



      // ── Skid-Steer Encoder Odometry Integration ──
      const now = Date.now();
      let linearVel = 0.0;
      let angularVel = 0.0;
      if (lastOdomTicks[0] !== null && lastOdomTime !== null) {
        const dm1 = m1 - lastOdomTicks[0];
        const dm2 = m2 - lastOdomTicks[1];
        const dm3 = m3 - lastOdomTicks[2];
        const dm4 = m4 - lastOdomTicks[3];

        const dLeftTicks = (dm1 + dm3) / 2.0;
        const dRightTicks = (dm2 + dm4) / 2.0;

        const TICKS_PER_REV = 937.2;
        const M_PER_TICK = (2.0 * Math.PI * WHEEL_RADIUS) / TICKS_PER_REV;

        const dLeftDist = dLeftTicks * M_PER_TICK;
        const dRightDist = dRightTicks * M_PER_TICK;
        const dCenterDist = (dLeftDist + dRightDist) / 2.0;
        const dYaw = (dRightDist - dLeftDist) / TRACK_WIDTH;

        accumLeftDist += dLeftDist;
        accumRightDist += dRightDist;

        const dt = (now - lastOdomTime) / 1000.0;
        if (dt > 0) {
          linearVel = dCenterDist / dt;
          angularVel = dYaw / dt;
          measuredLinearVel = linearVel;
          measuredAngularVel = angularVel;
        }

        const yawAvg = odomYaw + dYaw / 2.0;
        odomX += dCenterDist * Math.cos(yawAvg);
        odomY += dCenterDist * Math.sin(yawAvg);
        odomYaw += dYaw;

        // Normalize yaw to [-PI, PI]
        odomYaw = Math.atan2(Math.sin(odomYaw), Math.cos(odomYaw));
      }

      lastOdomTicks[0] = m1;
      lastOdomTicks[1] = m2;
      lastOdomTicks[2] = m3;
      lastOdomTicks[3] = m4;
      lastOdomTime = now;

      // Broadcast odom telemetry
      broadcast({
        type: 'odom',
        timestamp: now,
        left_dist: parseFloat(accumLeftDist.toFixed(4)),
        right_dist: parseFloat(accumRightDist.toFixed(4)),
        x: parseFloat(odomX.toFixed(4)),
        y: parseFloat(odomY.toFixed(4)),
        yaw: parseFloat(odomYaw.toFixed(4)),
        v: parseFloat(linearVel.toFixed(4)),
        w: parseFloat(angularVel.toFixed(4)),
        encoders: [m1, m2, m3, m4]
      });

      // ── Path Recording ──
      if (recording) {
        if (recordedPath.length === 0) {
          recordedPath.push({
            timestamp: now,
            x: odomX,
            y: odomY,
            yaw: odomYaw,
            left_dist: accumLeftDist,
            right_dist: accumRightDist,
            v: linearVel,
            w: angularVel
          });
        } else {
          const lastW = recordedPath[recordedPath.length - 1];
          const dist = Math.hypot(odomX - lastW.x, odomY - lastW.y);
          const yawDiff = Math.abs(odomYaw - lastW.yaw);
          const timeDiff = now - lastW.timestamp;
          if (dist > 0.02 || yawDiff > 0.05 || (timeDiff > 200 && (Math.abs(linearVel) > 0.01 || Math.abs(angularVel) > 0.01))) {
            recordedPath.push({
              timestamp: now,
              x: odomX,
              y: odomY,
              yaw: odomYaw,
              left_dist: accumLeftDist,
              right_dist: accumRightDist,
              v: linearVel,
              w: angularVel
            });
          }
        }
      }

      // ── Backtracking Control Loop ──
      if (backtracking) {
        if (backtrackIndex < 0) {
          backtracking = false;
          targetLinear = 0.0;
          targetAngular = 0.0;
          cmdSource = 'NONE';
          console.log('[Backtrack] Completed successfully.');
          
          if (serialPort && serialPort.isOpen) {
            serialPort.write(buildPacket(FUNC_DISARM_NORMAL_DRIVE, [1]));
            broadcast({ type: 'raw_serial_out', data: `[Internal Backtrack Complete] disarm command sent` });
          }
          broadcast({ type: 'backtrack_status', status: 'completed' });
        } else {
          const target = recordedPath[backtrackIndex];
          const dx = target.x - odomX;
          const dy = target.y - odomY;
          const dist = Math.hypot(dx, dy);

          if (dist > 0.5) {
            abortBacktrack(`Excessive tracking error: ${dist.toFixed(2)}m`);
          } else if (dist < 0.06) {
            backtrackIndex--;
            broadcast({ type: 'backtrack_status', status: 'progress', index: backtrackIndex, total: recordedPath.length });
          } else {
            const targetYaw = Math.atan2(dy, dx);
            let yawError = targetYaw - odomYaw;
            yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

            if (Math.abs(yawError) > 0.4) {
              targetLinear = 0.0;
              targetAngular = Math.sign(yawError) * 0.25; // 0.25 rad/s slow turn
            } else {
              targetLinear = Math.min(0.06, dist * 0.5); // Max 0.06 m/s floor speed
              targetAngular = yawError * 1.5;
              targetAngular = Math.max(-0.3, Math.min(0.3, targetAngular)); // Max 0.3 rad/s steering
            }
            cmdSource = 'BACKTRACK';
            driveCommandNeedsFlush = true;
          }
        }
      }

      // Position Control Loop (legacy command support)
      let motorSpeeds = [0, 0, 0, 0];
      let anyPositionMode = false;
      
      if (autoTestStep > 0) {
        let active = positionMode.some(m => m === true);
        if (active) {
          anyPositionMode = true;
          let sumError = 0;
          for (let i = 0; i < 4; i++) {
            sumError += (targetPosition[i] - currentTicks[i]);
          }
          let avgError = sumError / 4.0;
          
          // Log telemetry data
          let relM1 = currentTicks[0] - autoTestStartTicks[0];
          let relM2 = currentTicks[1] - autoTestStartTicks[1];
          let relM3 = currentTicks[2] - autoTestStartTicks[2];
          let relM4 = currentTicks[3] - autoTestStartTicks[3];
          let leftTicks = (relM1 + relM3) / 2.0;
          let rightTicks = (relM2 + relM4) / 2.0;
          const WHEEL_RADIUS_LOC = 0.0325;
          const TRACK_WIDTH_LOC = 0.160;
          let leftDist = (leftTicks / TICKS_PER_REV) * (2.0 * Math.PI * WHEEL_RADIUS_LOC);
          let rightDist = (rightTicks / TICKS_PER_REV) * (2.0 * Math.PI * WHEEL_RADIUS_LOC);
          let centerDist = (leftDist + rightDist) / 2.0;
          let yaw = (rightDist - leftDist) / TRACK_WIDTH_LOC;
          let driftMm = Math.round(centerDist * Math.sin(yaw) * 1000.0);
          let mismatchPct = leftTicks !== 0 ? (((leftTicks - rightTicks) / leftTicks) * 100).toFixed(2) : '0.00';
          
          autoTestRunLogs.push({
            timestamp: Date.now(),
            step: autoTestStep,
            m1: currentTicks[0],
            m2: currentTicks[1],
            m3: currentTicks[2],
            m4: currentTicks[3],
            speedLimit: currentAutoTestSpeedLimit,
            driftMm,
            mismatchPct
          });

          // One-directional completion logic (no back-and-forth correction at targets)
          let isFwdLeg = (autoTestStep % 2 === 1);
          let completed = false;
          if (isFwdLeg) {
            if (avgError <= 45) completed = true;
          } else {
            if (avgError >= -45) completed = true;
          }
          
          if (completed) {
            positionMode = [false, false, false, false];
          } else {
            let speed = avgError * KP_POSITION;
            if (speed > 0) {
              if (speed > currentAutoTestSpeedLimit) speed = currentAutoTestSpeedLimit;
              if (speed < 5) speed = 5;
            } else {
              if (speed < -currentAutoTestSpeedLimit) speed = -currentAutoTestSpeedLimit;
              if (speed > -5) speed = -5;
            }
            let targetSpeed = Math.round(speed);
            motorSpeeds = [targetSpeed, targetSpeed, targetSpeed, targetSpeed];
          }
        }
      } else {
        // Legacy independent wheel position control (e.g. from /api/turn)
        for (let i = 0; i < 4; i++) {
          if (positionMode[i]) {
            anyPositionMode = true;
            const error = targetPosition[i] - currentTicks[i];
            if (Math.abs(error) <= 15) {
              positionMode[i] = false;
              console.log(`[Position Control] Motor ${i + 1} reached target.`);
            } else {
              let speed = error * KP_POSITION;
              if (speed > 0) {
                if (speed < MIN_POSITION_SPEED) speed = MIN_POSITION_SPEED;
                if (speed > MAX_POSITION_SPEED) speed = MAX_POSITION_SPEED;
              } else {
                if (speed > -MIN_POSITION_SPEED) speed = -MIN_POSITION_SPEED;
                if (speed < -MAX_POSITION_SPEED) speed = -MAX_POSITION_SPEED;
              }
              motorSpeeds[i] = Math.round(speed);
            }
          }
        }
      }
      
      let stillPositionMode = positionMode.some(m => m === true);
      if (anyPositionMode && !stillPositionMode) {
        console.log('[Position Control] All motors reached target.');
        sendMotorSpeeds(0, 0, 0, 0); // Stop them
        if (autoTestStep > 0) {
          console.log(`[Auto Test] Leg ${autoTestStep}/6 complete. Settle pause...`);
          broadcast({ type: 'autotest_status', step: autoTestStep, msg: `Leg ${autoTestStep}/6 complete. Pausing for 1.5s...` });
          setTimeout(() => {
            if (autoTestStep > 0) {
              autoTestStep += 1;
              handleAutoTestNextStep();
            }
          }, 1500);
        }
      } else if (anyPositionMode) {
        sendMotorSpeeds(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
      }

      encoderPacketCount += 1;
      if ((now - lastEncoderActivityBroadcast) > 250) {
        lastEncoderActivityBroadcast = now;
        broadcast({ type: 'encoder_total', m1, m2, m3, m4 });
        broadcast({
          type: 'encoder_activity',
          packets: encoderPacketCount,
          hasNonZero: (m1 !== 0 || m2 !== 0 || m3 !== 0 || m4 !== 0),
          counts: [m1, m2, m3, m4],
        });
      }
    }

  } else if (extType === TYPE_CALIBRATION) {
    if (data.length >= 55) {
      const protoMajor = data[0];
      const protoMinor = data[1];
      const sessionId = data.readUInt32LE(2);
      const isSimulation = data[6];
      const cal_state = data[7];
      const cal_motor = data[8];
      const cal_motor_num = data[9];
      const direction = data[10];
      const cal_pwm = data[11];
      const encoderDelta = data[12];
      const movementDetected = data[13];
      const motorLockStatus = data[14];
      const cal_fwd = [data[15], data[16], data[17], data[18]];
      const cal_rev = [data[19], data[20], data[21], data[22]];
      const failureReason = data.toString('utf8', 23, 55).replace(/\0/g, '').trim();
      
      latestCalibrationStatus = {
        protoMajor,
        protoMinor,
        sessionId,
        isSimulation: isSimulation === 1,
        cal_state,
        cal_motor,
        cal_motor_num,
        direction,
        cal_pwm,
        encoderDelta,
        movementDetected: movementDetected === 1,
        motorLockStatus: motorLockStatus === 1,
        cal_fwd,
        cal_rev,
        failureReason
      };
      
      broadcast({
        type: 'calibration_status',
        ...latestCalibrationStatus
      });
    } else if (data.length >= 11) {
      const cal_state = data[0];
      const cal_motor = data[1];
      const cal_pwm = data[2];
      const cal_fwd = [data[3], data[4], data[5], data[6]];
      const cal_rev = [data[7], data[8], data[9], data[10]];
      
      latestCalibrationStatus = {
        protoMajor: 1,
        protoMinor: 0,
        sessionId: 0,
        isSimulation: false,
        cal_state,
        cal_motor,
        cal_motor_num: cal_motor + 1,
        direction: 0,
        cal_pwm,
        encoderDelta: 0,
        movementDetected: false,
        motorLockStatus: false,
        cal_fwd,
        cal_rev,
        failureReason: ''
      };
      
      broadcast({
        type: 'calibration_status',
        ...latestCalibrationStatus
      });
    }

  } else if (extType === TYPE_MAINTENANCE) {
    if (data.length >= 15) {
      const protoMajor = data[0];
      const protoMinor = data[1];
      const sessionId = data.readUInt32LE(2);
      const active = data[6];
      const activeMotor = data[7];
      const activeMotorNum = data[8];
      const direction = data[9];
      const testPwm = data[10];
      const actualPwm = data[11];
      const deadmanActive = data[12];
      const remainingTimeout = data.readUInt16LE(13);
      
      latestMaintenanceStatus = {
        protoMajor,
        protoMinor,
        sessionId,
        active: active === 1,
        activeMotor,
        activeMotorNum,
        direction,
        testPwm,
        actualPwm,
        deadmanActive: deadmanActive === 1,
        remainingTimeout
      };
      
      broadcast({
        type: 'maintenance_status',
        ...latestMaintenanceStatus
      });
    }

  } else if (extType === TYPE_FIRMWARE_INFO) {
    if (data.length >= 112) {
      const name = data.toString('utf8', 0, 32).replace(/\0/g, '').trim();
      const version = data.toString('utf8', 32, 48).replace(/\0/g, '').trim();
      const protocol = data.toString('utf8', 48, 56).replace(/\0/g, '').trim();
      const commit = data.toString('utf8', 56, 72).replace(/\0/g, '').trim();
      const build = data.toString('utf8', 72, 96).replace(/\0/g, '').trim();
      const target = data.toString('utf8', 96, 112).replace(/\0/g, '').trim();
      
      console.log(`[Firmware Info] Name: ${name}, Ver: ${version}, Protocol: ${protocol}, Source: ${commit}, Build: ${build}, Target: ${target}`);
      broadcast({
        type: 'firmware_info',
        name,
        version,
        protocol,
        commit,
        build,
        target
      });
    }

  } else if (extType === TYPE_LOOP_TIMING) {
    if (data.length >= 24) {
      const lastDurationUs = data.readUInt32LE(0);
      const minDurationUs = data.readUInt32LE(4);
      const avgDurationUs = data.readUInt32LE(8);
      const maxDurationUs = data.readUInt32LE(12);
      const missedDeadlines = data.readUInt32LE(16);
      const totalIterations = data.readUInt32LE(20);
      
      broadcast({
        type: 'loop_timing',
        lastDurationUs,
        minDurationUs,
        avgDurationUs,
        maxDurationUs,
        missedDeadlines,
        totalIterations
      });
    }

  } else if (extType === TYPE_FAULT_REPORT) {
    if (data.length >= 4) {
      const faultFlags = data.readUInt32LE(0);
      broadcast({ type: 'fault_report', faultFlags });
    }

  } else if (extType === TYPE_NORMAL_DRIVE_STATUS) {
    if (data.length >= 24) {
      const armed = data[0] === 1;
      const mode = data[1];
      const source = data[2];
      const cmdAge = data.readUInt32LE(3);
      const reqLinear = data.readFloatLE(7);
      const reqAngular = data.readFloatLE(11);
      const limLinear = data.readFloatLE(15);
      const limAngular = data.readFloatLE(19);
      const lockStatus = data[23] === 1;
      
      latestNormalDriveStatus = {
        armed,
        mode,
        source,
        cmdAge,
        reqLinear,
        reqAngular,
        limLinear,
        limAngular,
        lockStatus
      };
      
      broadcast({
        type: 'normal_drive_status',
        ...latestNormalDriveStatus
      });
    }

  } else if (extType === 0x37) { // TYPE_ROVER_PARAMS
    if (data.length >= 8) {
      const diameter = data.readFloatLE(0);
      const separation = data.readFloatLE(4);
      WHEEL_RADIUS = diameter / 2.0;
      TRACK_WIDTH = separation;
      console.log(`[Config Sync] ESP32 reported wheel diameter = ${diameter.toFixed(4)} m, effective track width = ${separation.toFixed(4)} m`);
      broadcast({ type: 'rover_params_sync', diameter, separation });
    }

  } else if (extType === 0x38) { // TYPE_ROVER_TRIMS (Forward)
    if (data.length >= 8) {
      const leftTrim = data.readFloatLE(0);
      const rightTrim = data.readFloatLE(4);
      activeFwdTrim = { left: leftTrim, right: rightTrim };
      console.log(`[Config Sync] ESP32 reported Left FWD Trim = ${leftTrim.toFixed(4)}, Right FWD Trim = ${rightTrim.toFixed(4)}`);
      broadcast({ type: 'rover_trims_sync', leftTrim, rightTrim });
    }

  } else if (extType === 0x39) { // TYPE_ROVER_TRIMS_REV (Reverse)
    if (data.length >= 8) {
      const leftTrimRev = data.readFloatLE(0);
      const rightTrimRev = data.readFloatLE(4);
      activeRevTrim = { left: leftTrimRev, right: rightTrimRev };
      console.log(`[Config Sync] ESP32 reported Left REV Trim = ${leftTrimRev.toFixed(4)}, Right REV Trim = ${rightTrimRev.toFixed(4)}`);
      broadcast({ type: 'rover_trims_rev_sync', leftTrimRev, rightTrimRev });
    }

  } else {
    // Unknown telemetry packet – log once per type to avoid flooding
    const hex = Array.from(data).map(b => b.toString(16).padStart(2,'0')).join(' ');
    broadcast({ type: 'raw_serial_in', data: `[Unknown 0x${extType.toString(16).padStart(2,'0')}] ${hex}` });
  }
}

// Helper: read little-endian signed 16-bit integer from a Buffer
function readInt16LE(buf, offset) {
  const lo = buf[offset]     || 0;
  const hi = buf[offset + 1] || 0;
  let val = (hi << 8) | lo;
  if (val & 0x8000) val = val - 0x10000;
  return val;
}

// ────────────────────────────────────────────────────────────
// Serial Port Initialization
// ────────────────────────────────────────────────────────────
function initSerial(portName = COM_PORT) {
  if (serialPort && serialPort.isOpen) {
    console.log(`Closing existing serial connection to ${COM_PORT}...`);
    serialPort.close();
  }

  COM_PORT = portName;
  if (!COM_PORT) {
    console.error("ERROR: No valid serial port configured. Skipping serial initialization.");
    broadcast({ type: 'status', key: 'serial', val: 'disconnected', error: 'No valid serial port configured' });
    isSerialConnecting = false;
    scheduleReconnect();
    return;
  }
  rxBuf = Buffer.alloc(0);
  lastImuTimestamp = null;
  isSerialConnecting = true;

  console.log(`Connecting to Maker ESP32 Pro Board on ${COM_PORT} at ${BAUD_RATE} baud (binary protocol)...`);
  broadcast({ type: 'status', key: 'serial', val: 'connecting', port: COM_PORT });

  serialPort = new SerialPort({
    path: COM_PORT,
    baudRate: BAUD_RATE,
    autoOpen: false,
  });

  serialPort.open((err) => {
    isSerialConnecting = false;
    if (err) {
      console.error(`Error opening ${COM_PORT}:`, err.message);
      broadcast({ type: 'status', key: 'serial', val: 'disconnected', error: err.message });
      scheduleReconnect();
      return;
    }

    console.log(`Serial port ${COM_PORT} opened successfully (binary mode).`);
    broadcast({ type: 'status', key: 'serial', val: 'connected', port: COM_PORT });

    serialPort.set({ dtr: false, rts: false }, (setErr) => {
      if (setErr) {
        console.warn(`[Serial] Warning: Failed to set DTR/RTS: ${setErr.message}`);
      } else {
        console.log('[Serial] DTR/RTS set to false (releasing ESP32 reset).');
      }
    });

    // The board auto-streams telemetry. No explicit command needed to start it.
    // Set a default car type and send a short beep to confirm two-way communication.
    sendBinaryCommand(FUNC_CAR_TYPE, [1, 0], { dualChecksum: true });
    setTimeout(() => {
      if (calibrationDb && calibrationDb.currentConfig && calibrationDb.currentConfig.wheelDiameter) {
        const diaBytes = floatToLEBytes(calibrationDb.currentConfig.wheelDiameter);
        const sepBytes = floatToLEBytes(calibrationDb.currentConfig.effectiveTrackWidth);
        sendBinaryCommand(0x37, [...diaBytes, ...sepBytes], { dualChecksum: true });
        console.log(`[Config Sync] Pushed database parameters to ESP32: dia=${calibrationDb.currentConfig.wheelDiameter}, sep=${calibrationDb.currentConfig.effectiveTrackWidth}`);
      } else {
        sendBinaryCommand(0x37, [], { dualChecksum: true });
      }
    }, 600);
    setTimeout(() => {
      if (activeFwdTrim) {
        const leftBytes = floatToLEBytes(activeFwdTrim.left);
        const rightBytes = floatToLEBytes(activeFwdTrim.right);
        sendBinaryCommand(0x38, [...leftBytes, ...rightBytes], { dualChecksum: true });
        setTimeout(() => {
          sendBinaryCommand(0x38, [], { dualChecksum: true });
        }, 100);
        console.log(`[Config Sync] Pushed FWD trims to ESP32: L=${activeFwdTrim.left} R=${activeFwdTrim.right}`);
      } else {
        sendBinaryCommand(0x38, [], { dualChecksum: true });
      }
      setTimeout(() => {
        if (activeRevTrim) {
          const leftBytes = floatToLEBytes(activeRevTrim.left);
          const rightBytes = floatToLEBytes(activeRevTrim.right);
          sendBinaryCommand(0x39, [...leftBytes, ...rightBytes], { dualChecksum: true });
          setTimeout(() => {
            sendBinaryCommand(0x39, [], { dualChecksum: true });
          }, 100);
          console.log(`[Config Sync] Pushed REV trims to ESP32: L=${activeRevTrim.left} R=${activeRevTrim.right}`);
        } else {
          sendBinaryCommand(0x39, [], { dualChecksum: true });
        }
      }, 250);
    }, 1000);
    setTimeout(() => {
      const beepLo = 100 & 0xFF;
      const beepHi = (100 >> 8) & 0xFF;
      sendBinaryCommand(FUNC_BEEP, [beepLo, beepHi], { dualChecksum: true });
    }, 1500);
  });

  let rawLogCounter = 0;
  serialPort.on('data', (chunk) => {
    rxBuf = Buffer.concat([rxBuf, chunk]);
    // Only log raw bytes occasionally to avoid flooding the UI
    rawLogCounter++;
    if (rawLogCounter % 50 === 0) {
      const hex = Array.from(chunk).map(b => b.toString(16).padStart(2,'0')).join(' ');
      broadcast({ type: 'raw_serial_in', data: `[raw ${rawLogCounter}] ${hex}` });
    }
    processRxBuffer();
  });

  serialPort.on('close', () => {
    console.log(`Serial port ${COM_PORT} closed.`);
    broadcast({ type: 'status', key: 'serial', val: 'disconnected' });
    scheduleReconnect();
  });

  serialPort.on('error', (err) => {
    console.error(`Serial port error:`, err.message);
    broadcast({ type: 'status', key: 'serial', val: 'error', error: err.message });
  });
}

// ────────────────────────────────────────────────────────────
// Reconnect Logic
// ────────────────────────────────────────────────────────────
function scheduleReconnect() {
  if (reconnectTimeout || isSerialConnecting) return;
  console.log('Scheduling serial reconnection in 5 seconds...');
  reconnectTimeout = setTimeout(() => {
    reconnectTimeout = null;
    initSerial(COM_PORT);
  }, 5000);
}

// ────────────────────────────────────────────────────────────
// WebSocket Message Handlers
// ────────────────────────────────────────────────────────────
wss.on('connection', (ws) => {
  console.log('Web client connected.');

  // Send current serial connection state
  const serialState = (serialPort && serialPort.isOpen) ? 'connected' : 'disconnected';
  ws.send(JSON.stringify({ type: 'status', key: 'serial', val: serialState, port: COM_PORT }));
  ws.send(JSON.stringify({ type: 'cockpit_info', deployed: serverUpdatedTime }));

  // Push dynamic parameters from database immediately on client connection
  if (serialPort && serialPort.isOpen) {
    if (calibrationDb && calibrationDb.currentConfig && calibrationDb.currentConfig.wheelDiameter) {
      const diaBytes = floatToLEBytes(calibrationDb.currentConfig.wheelDiameter);
      const sepBytes = floatToLEBytes(calibrationDb.currentConfig.effectiveTrackWidth);
      sendBinaryCommand(0x37, [...diaBytes, ...sepBytes], { dualChecksum: true });
    } else {
      sendBinaryCommand(0x37, [], { dualChecksum: true });
    }
  }

  ws.on('message', (message) => {
    try {
      const msg = JSON.parse(message);

      switch (msg.type) {
        case 'set_speed':
          if (autoTestStep > 0) {
            autoTestStep = 0;
            broadcast({ type: 'autotest_status', step: 0, msg: 'Auto-test aborted by manual override.' });
          }
          // Frontend sends { type:'set_speed', speeds:[m1,m2,m3,m4] } in range -1000..1000
          if (Array.isArray(msg.speeds) && msg.speeds.length === 4) {
            positionMode = [false, false, false, false];
            sendMotorSpeeds(msg.speeds[0], msg.speeds[1], msg.speeds[2], msg.speeds[3]);
          }
          break;

        case 'set_pwm':
          if (autoTestStep > 0) {
            autoTestStep = 0;
            broadcast({ type: 'autotest_status', step: 0, msg: 'Auto-test aborted by manual override.' });
          }
          // Alias for set_speed (frontend uses both)
          if (Array.isArray(msg.pwms) && msg.pwms.length === 4) {
            positionMode = [false, false, false, false];
            sendMotorSpeeds(msg.pwms[0], msg.pwms[1], msg.pwms[2], msg.pwms[3]);
          }
          break;

        case 'joystick':
          if (msg.x !== undefined && msg.y !== undefined) {
            const x = Math.max(-1.0, Math.min(1.0, parseFloat(msg.x || 0)));
            const y = Math.max(-1.0, Math.min(1.0, parseFloat(msg.y || 0)));
            
            if (autoTestStep > 0 && (Math.abs(x) > 0.05 || Math.abs(y) > 0.05)) {
              autoTestStep = 0;
              broadcast({ type: 'autotest_status', step: 0, msg: 'Auto-test aborted by manual override.' });
            }
            
            const isGamepad = (msg.deadman !== undefined);
            if (isGamepad) {
              deadmanPressed = msg.deadman === true;
            } else {
              deadmanPressed = true; // Implicit deadman true for keyboard/browser
            }
            
            let throttle = Math.abs(y) < 0.10 ? 0 : Math.sign(y) * Math.pow(Math.abs(y), 2.0);
            
            let turnScaled = 0;
            if (Math.abs(x) >= 0.10) {
              const MIN_COEFF = 0.35; // Starts at 0.35 * MAX_ANGULAR to immediately break stiction
              const normalizedStick = (Math.abs(x) - 0.10) / 0.90;
              const scaledStick = Math.pow(normalizedStick, 1.5);
              turnScaled = Math.sign(x) * (MIN_COEFF + scaledStick * (1.0 - MIN_COEFF));
            }

            if (backtracking && (Math.abs(x) > 0.05 || Math.abs(y) > 0.05)) {
              abortBacktrack('Joystick override');
            }

            if (cmdSource === 'CALIBRATION_TEST' && (Math.abs(x) > 0.05 || Math.abs(y) > 0.05)) {
              broadcast({ type: 'test_abort', reason: 'Joystick override' });
              targetLinear = 0.0;
              targetAngular = 0.0;
              cmdSource = 'NONE';
            }

            // LiDAR Automated Calibration Test active handling
            if (lidarTestState !== 'IDLE') {
              if (autoCalibActive) {
                // Fully automated mode: any joystick movement = emergency abort
                if (Math.abs(x) > 0.15 || Math.abs(y) > 0.15) {
                  abortLidarTest('Joystick override — emergency abort.');
                  autoCalibActive = false;
                  console.log('[Auto Calib] Aborted by joystick override.');
                }
                // Ignore all joystick input otherwise — rover drives itself
                return;
              }
              
              // Legacy manual mode fallback (should not be reached in normal use)
              turnScaled = 0.0;
              if (lidarTestState === 'COMPLETE' || lidarTestState === 'ABORTED') {
                if (Math.abs(throttle) < 0.05) {
                  lidarTestState = 'IDLE';
                  broadcast({ type: 'lidar_test_status', state: lidarTestState, msg: 'Test cleared.' });
                }
              }
              return;
            }

            if (deadmanPressed) {
              if (!backtracking && cmdSource !== 'CALIBRATION_TEST') {
                const MAX_LINEAR = floorTesting ? 0.17 : 0.80;
                const MAX_ANGULAR = floorTesting ? 0.90 : 3.00;
                
                targetLinear = throttle * MAX_LINEAR;
                targetAngular = -turnScaled * MAX_ANGULAR;
                cmdSource = isGamepad ? 'GAMEPAD' : 'BROWSER';
                startDriveKeepaliveLoop();
              }
            } else {
              if (backtracking) {
                abortBacktrack('Deadman released');
              } else if (cmdSource === 'CALIBRATION_TEST') {
                broadcast({ type: 'test_abort', reason: 'Deadman released' });
                targetLinear = 0.0;
                targetAngular = 0.0;
                cmdSource = 'NONE';
              } else if (cmdSource === 'GAMEPAD' || cmdSource === 'BROWSER') {
                targetLinear = 0.0;
                targetAngular = 0.0;
                cmdSource = 'NONE';
              }
            }
          }
          break;

        case 'start_lidar_test':
          if (lidarTestState !== 'IDLE') {
            return ws.send(JSON.stringify({ type: 'error', message: 'LiDAR test is already running.' }));
          }
          lidarTestMode = 'correct'; // always correct+learn
          fwdAngleOffset = parseFloat(msg.frontAngleOffset || 0.0);
          lidarXOffset = parseFloat(msg.lidarXOffset || 0.0127);
          lidarYOffset = parseFloat(msg.lidarYOffset || 0.034925);
          lidarYawOffset = parseFloat(msg.lidarYawOffset || 0.0);
          maxCalibRange = parseFloat(msg.maxRange || 4.0);
          chassisMargin = parseFloat(msg.chassisMargin || 0.02);
          angleSectorMasks = msg.angleSectorMasks || '';
          
          headingGain = parseFloat(msg.headingGain || 0.8);
          lateralGain = parseFloat(msg.lateralGain || 1.2);
          maxAngularCorr = parseFloat(msg.maxAngularCorr || 0.35);
          corrSlewRate = parseFloat(msg.corrSlewRate || 1.0);
          minConfidence = parseFloat(msg.minConfidence || 0.65);
          
          // Reset multi-pass state
          sessionLogs = [];
          passSummaries = [];
          acceptedPasses = 0;
          calibrationConfidence = 0.0;
          testSessionId = 'session_' + Date.now();
          testPassNumber = 0;
          autoCalibSpeedTier = 0;
          autoCalibPassIndex = 0;
          autoCalibTotalPass = 0;
          autoCalibActive = true;
          tierPassSummaries = { SLOW: [], MED: [], FAST: [] };
          
          lidarTestState = 'ZEROING';
          console.log(`[Auto Calib] Starting 9-pass automated calibration test.`);
          
          let startUrl = `${LIDAR_TEST_START_URL}?front_angle_offset=${fwdAngleOffset}&lidar_x_offset=${lidarXOffset}&lidar_y_offset=${lidarYOffset}&lidar_yaw_offset=${lidarYawOffset}&min_range=0.15&max_range=${maxCalibRange}&chassis_margin=${chassisMargin}`;
          if (angleSectorMasks) {
            startUrl += `&angle_sector_masks=${encodeURIComponent(angleSectorMasks)}`;
          }
          
          http.get(startUrl, (sRes) => {
            let body = '';
            sRes.on('data', (c) => { body += c; });
            sRes.on('end', () => {
              console.log('[Auto Calib] Python sidecar start response:', body);
            });
          }).on('error', (err) => {
            console.error('[Auto Calib] Failed to notify python sidecar:', err.message);
          });
          
          if (serialPort && serialPort.isOpen) {
            const clearPkt = buildPacket(FUNC_CLEAR_FAULTS, [1]);
            serialPort.write(clearPkt);
            const armPkt = buildPacket(FUNC_ARM_NORMAL_DRIVE, [1]);
            serialPort.write(armPkt);
          }
          
          startTestPosePolling();
          broadcast({ 
            type: 'lidar_test_status', 
            state: lidarTestState, 
            msg: 'Initializing zeroing calibration. Keep rover stationary...',
            speedTier: AUTO_CALIB_SPEED_LABELS[autoCalibSpeedTier],
            passIndex: autoCalibPassIndex + 1,
            totalPass: autoCalibTotalPass + 1,
            totalPasses: AUTO_CALIB_SPEEDS.length * AUTO_CALIB_PASSES_PER_TIER
          });
          break;
          
        case 'stop_lidar_test':
          autoCalibActive = false;
          targetLinear = 0.0;
          targetAngular = 0.0;
          lastCorrectionApplied = 0.0;
          abortLidarTest('Manually stopped by operator.');
          lidarTestState = 'IDLE';
          console.log('[Auto Calib] Test stopped by operator. Manual control restored.');
          broadcast({ type: 'lidar_test_status', state: lidarTestState, msg: 'Test stopped. Manual control restored.' });
          break;
          
        case 'apply_proposed_trims':
          if (serialPort && serialPort.isOpen) {
            backupTrims.fwd = { left: activeFwdTrim.left, right: activeFwdTrim.right };
            backupTrims.rev = { left: activeRevTrim.left, right: activeRevTrim.right };
            
            const leftFwdBytes = floatToLEBytes(proposedFwdTrim.left);
            const rightFwdBytes = floatToLEBytes(proposedFwdTrim.right);
            sendBinaryCommand(0x38, [...leftFwdBytes, ...rightFwdBytes], { dualChecksum: true });
            
            setTimeout(() => {
              const leftRevBytes = floatToLEBytes(proposedRevTrim.left);
              const rightRevBytes = floatToLEBytes(proposedRevTrim.right);
              sendBinaryCommand(0x39, [...leftRevBytes, ...rightRevBytes], { dualChecksum: true });
            }, 100);
            
            setTimeout(() => {
              sendBinaryCommand(0x38, [], { dualChecksum: true });
              setTimeout(() => {
                sendBinaryCommand(0x39, [], { dualChecksum: true });
              }, 100);
            }, 500);
            
            broadcast({ type: 'message', data: `[LiDAR Calib] Saved trims. FWD: ${proposedFwdTrim.left.toFixed(4)} / ${proposedFwdTrim.right.toFixed(4)} | REV: ${proposedRevTrim.left.toFixed(4)} / ${proposedRevTrim.right.toFixed(4)}` });
          }
          break;
          
        case 'rollback_trims':
          if (serialPort && serialPort.isOpen) {
            const leftFwdBytes = floatToLEBytes(backupTrims.fwd.left);
            const rightFwdBytes = floatToLEBytes(backupTrims.fwd.right);
            sendBinaryCommand(0x38, [...leftFwdBytes, ...rightFwdBytes], { dualChecksum: true });
            
            setTimeout(() => {
              const leftRevBytes = floatToLEBytes(backupTrims.rev.left);
              const rightRevBytes = floatToLEBytes(backupTrims.rev.right);
              sendBinaryCommand(0x39, [...leftRevBytes, ...rightRevBytes], { dualChecksum: true });
            }, 100);
            
            setTimeout(() => {
              sendBinaryCommand(0x38, [], { dualChecksum: true });
              setTimeout(() => {
                sendBinaryCommand(0x39, [], { dualChecksum: true });
              }, 100);
            }, 500);
            
            broadcast({ type: 'message', data: `[LiDAR Calib] Rolled back trims to previous values.` });
          }
          break;
          
        case 'reset_trims':
          if (serialPort && serialPort.isOpen) {
            const leftBytes = floatToLEBytes(1.0);
            const rightBytes = floatToLEBytes(1.0);
            sendBinaryCommand(0x38, [...leftBytes, ...rightBytes], { dualChecksum: true });
            setTimeout(() => {
              sendBinaryCommand(0x39, [...leftBytes, ...rightBytes], { dualChecksum: true });
            }, 100);
            setTimeout(() => {
              sendBinaryCommand(0x38, [], { dualChecksum: true });
              setTimeout(() => {
                sendBinaryCommand(0x39, [], { dualChecksum: true });
              }, 100);
            }, 500);
            broadcast({ type: 'message', data: `[LiDAR Calib] Reset trims to 1.000.` });
          }
          break;

        case 'test_drive':
          if (msg.v !== undefined && msg.w !== undefined) {
            if (backtracking) break;
            if (deadmanPressed) {
              targetLinear = msg.v;
              targetAngular = msg.w;
              cmdSource = 'CALIBRATION_TEST';
              startDriveKeepaliveLoop();
            } else {
              targetLinear = 0.0;
              targetAngular = 0.0;
              cmdSource = 'NONE';
            }
          }
          break;

        case 'change_port':
          if (msg.port) {
            initSerial(msg.port);
          }
          break;

        case 'raw_command': {
          // Allow sending pre-built hex string as raw bytes for testing
          // Format: "ff fc 05 02 64 00 6b"
          if (msg.command) {
            const bytes = msg.command.trim().split(/\s+/).map(b => parseInt(b, 16)).filter(b => !isNaN(b));
            if (bytes.length > 0 && serialPort && serialPort.isOpen) {
              serialPort.write(Buffer.from(bytes));
              console.log(`[Raw Hex Out]: ${msg.command}`);
              broadcast({ type: 'raw_serial_out', data: msg.command });
            }
          }
          break;
        }

        case 'beep': {
          // { type: 'beep', duration: 500 }
          const onTime = msg.duration || 100;
          const lo = onTime & 0xFF;
          const hi = (onTime >> 8) & 0xFF;
          sendBinaryCommand(FUNC_BEEP, [lo, hi]);
          break;
        }

        case 'run_motor_proof':
          runMotorProofSequence();
          break;

        case 'get_calibration_db':
          ws.send(JSON.stringify({ type: 'calibration_db', db: calibrationDb }));
          break;

        case 'save_proposed_config':
          if (msg.wheelDiameter !== undefined && msg.effectiveTrackWidth !== undefined) {
            calibrationDb.proposedConfig.wheelDiameter = msg.wheelDiameter;
            calibrationDb.proposedConfig.effectiveTrackWidth = msg.effectiveTrackWidth;
            saveCalibrationDb();
            broadcast({ type: 'calibration_db', db: calibrationDb });
          }
          break;

        case 'apply_calibration':
          if (calibrationDb.proposedConfig.wheelDiameter && calibrationDb.proposedConfig.effectiveTrackWidth) {
            // Backup current to previous
            calibrationDb.previousConfig = { ...calibrationDb.currentConfig };
            // Set proposed to current
            calibrationDb.currentConfig.wheelDiameter = calibrationDb.proposedConfig.wheelDiameter;
            calibrationDb.currentConfig.effectiveTrackWidth = calibrationDb.proposedConfig.effectiveTrackWidth;
            saveCalibrationDb();
            
            // Program ESP32 NVS
            if (serialPort && serialPort.isOpen) {
              const diaBytes = floatToLEBytes(calibrationDb.currentConfig.wheelDiameter);
              const sepBytes = floatToLEBytes(calibrationDb.currentConfig.effectiveTrackWidth);
              sendBinaryCommand(0x37, [...diaBytes, ...sepBytes], { dualChecksum: true });
              broadcast({ type: 'message', data: `[Config] Applied calibration: diameter=${calibrationDb.currentConfig.wheelDiameter}m, track=${calibrationDb.currentConfig.effectiveTrackWidth}m` });
            }
            broadcast({ type: 'calibration_db', db: calibrationDb });
          }
          break;

        case 'restore_previous':
          if (calibrationDb.previousConfig.wheelDiameter && calibrationDb.previousConfig.effectiveTrackWidth) {
            const temp = { ...calibrationDb.currentConfig };
            calibrationDb.currentConfig = { ...calibrationDb.previousConfig };
            calibrationDb.previousConfig = temp;
            saveCalibrationDb();
            
            // Program ESP32 NVS
            if (serialPort && serialPort.isOpen) {
              const diaBytes = floatToLEBytes(calibrationDb.currentConfig.wheelDiameter);
              const sepBytes = floatToLEBytes(calibrationDb.currentConfig.effectiveTrackWidth);
              sendBinaryCommand(0x37, [...diaBytes, ...sepBytes], { dualChecksum: true });
              broadcast({ type: 'message', data: `[Config] Restored previous calibration: diameter=${calibrationDb.currentConfig.wheelDiameter}m, track=${calibrationDb.currentConfig.effectiveTrackWidth}m` });
            }
            broadcast({ type: 'calibration_db', db: calibrationDb });
          }
          break;

        case 'save_trims':
          if (msg.leftTrim !== undefined && msg.rightTrim !== undefined && serialPort && serialPort.isOpen) {
            const leftBytes = floatToLEBytes(msg.leftTrim);
            const rightBytes = floatToLEBytes(msg.rightTrim);
            sendBinaryCommand(0x38, [...leftBytes, ...rightBytes], { dualChecksum: true });
            console.log(`[Config Sync] Sent trims to ESP32: Left=${msg.leftTrim}, Right=${msg.rightTrim}`);
          }
          break;

        case 'get_trims':
          if (serialPort && serialPort.isOpen) {
            sendBinaryCommand(0x38, [], { dualChecksum: true });
          }
          break;

        case 'log_test_run':
          if (msg.testType && msg.results) {
            calibrationDb.testLogs.push({
              timestamp: Date.now(),
              testType: msg.testType,
              results: msg.results,
              surfaceType: msg.surfaceType || 'unknown',
              firmwareVersion: msg.firmwareVersion || '1.3.0-phase4'
            });
            saveCalibrationDb();
            broadcast({ type: 'calibration_db', db: calibrationDb });
          }
          break;

        // Legacy commands from old protocol – silently ignore to avoid errors
        case 'set_upload':
        case 'config_motor_type':
        case 'config_deadband':
        case 'config_phase_lines':
        case 'config_reduction_ratio':
        case 'config_wheel_diameter':
        case 'config_pid':
        case 'flash_reset':
        case 'read_flash':
          console.warn(`[Legacy command ignored] type: ${msg.type}`);
          broadcast({ type: 'message', data: `Note: Command "${msg.type}" is not supported on the ROS Expansion Board V3.0 binary protocol.` });
          break;

        default:
          console.warn('Unknown WebSocket message type:', msg.type);
      }
    } catch (e) {
      console.error('Failed to parse client message:', e.message);
    }
  });

  ws.on('close', () => {
    console.log('Web client disconnected.');
  });
});

// ────────────────────────────────────────────────────────────
// HTTP Test Endpoints (useful for manual debugging)
// GET /api/motor?m1=50&m2=50&m3=50&m4=50  (values -100..100)
// GET /api/stop
// GET /api/beep
// ────────────────────────────────────────────────────────────
app.get('/api/motor', (req, res) => {
  positionMode = [false, false, false, false];
  const m1 = parseInt(req.query.m1 || 0);
  const m2 = parseInt(req.query.m2 || 0);
  const m3 = parseInt(req.query.m3 || 0);
  const m4 = parseInt(req.query.m4 || 0);
  // Clamp to -100..100 directly (API uses board-native range)
  function clamp(v) { return Math.max(-100, Math.min(100, v)); }
  function toUnsigned(v) { const c = clamp(v); return c < 0 ? 256 + c : c; }
  const payload = [toUnsigned(m1), toUnsigned(m2), toUnsigned(m3), toUnsigned(m4)];
  const pkt = buildPacket(FUNC_MOTOR, payload);
  const hex = Array.from(pkt).map(b => b.toString(16).padStart(2, '0')).join(' ');
  if (serialPort && serialPort.isOpen) {
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/motor] ${hex}` });
    res.json({ ok: true, packet: hex, m1, m2, m3, m4 });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open', packet: hex });
  }
});

app.get('/api/stop', (req, res) => {
  positionMode = [false, false, false, false];

  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_EMERGENCY_STOP, [1]);
    serialPort.write(pkt);
    const hex = Array.from(pkt).map(b => b.toString(16).padStart(2, '0')).join(' ');
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/stop] EMERGENCY_STOP ${hex}` });
    res.json({ ok: true, packet: hex });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});


app.post('/api/calibration/simulate/start', (req, res) => {
  const { safetyAck } = req.body;
  if (!safetyAck) {
    return res.status(400).json({ ok: false, error: 'Safety acknowledgement is required to start calibration simulation.' });
  }
  
  if (serialPort && serialPort.isOpen) {
    const sessId = Math.floor(Math.random() * 1000000) + 1;
    const payload = [
      1, // safetyAck = true
      1, // simFlag = true
      sessId & 0xFF,
      (sessId >> 8) & 0xFF,
      (sessId >> 16) & 0xFF,
      (sessId >> 24) & 0xFF
    ];
    const pkt = buildPacket(FUNC_START_CALIBRATE, payload);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/calibration/simulate/start] session: ${sessId}` });
    res.json({ ok: true, message: 'Calibration simulation started.', sessionId: sessId });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/calibration/real/start', (req, res) => {
  const { safetyAck } = req.body;
  if (!safetyAck) {
    return res.status(400).json({ ok: false, error: 'Safety acknowledgement is required to start real calibration.' });
  }
  
  if (serialPort && serialPort.isOpen) {
    const sessId = Math.floor(Math.random() * 1000000) + 1;
    const payload = [
      1, // safetyAck = true
      0, // simFlag = false (Real!)
      sessId & 0xFF,
      (sessId >> 8) & 0xFF,
      (sessId >> 16) & 0xFF,
      (sessId >> 24) & 0xFF
    ];
    const pkt = buildPacket(FUNC_START_CALIBRATE, payload);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/calibration/real/start] session: ${sessId}` });
    res.json({ ok: true, message: 'Real calibration started.', sessionId: sessId });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/calibration/abort', (req, res) => {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_CANCEL_CALIBRATE, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/calibration/abort] cancel command sent` });
    res.json({ ok: true, message: 'Calibration aborted successfully.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.get('/api/calibration/status', (req, res) => {
  if (latestCalibrationStatus) {
    res.json({ ok: true, status: latestCalibrationStatus });
  } else {
    res.json({ ok: true, status: { cal_state: 0, message: 'No calibration run.' } });
  }
});

app.post('/api/maintenance/enter', (req, res) => {
  const { safetyAck, motorIndex, sessionId } = req.body;
  if (!safetyAck) {
    return res.status(400).json({ ok: false, error: 'Safety acknowledgement is required to enter maintenance mode.' });
  }
  if (motorIndex === undefined || motorIndex < 0 || motorIndex >= 4) {
    return res.status(400).json({ ok: false, error: 'Invalid motor index (must be 0-3).' });
  }
  const sessId = sessionId || (Math.floor(Math.random() * 1000000) + 1);
  
  if (serialPort && serialPort.isOpen) {
    const payload = [
      safetyAck ? 1 : 0,
      motorIndex,
      sessId & 0xFF,
      (sessId >> 8) & 0xFF,
      (sessId >> 16) & 0xFF,
      (sessId >> 24) & 0xFF
    ];
    const pkt = buildPacket(FUNC_ENTER_MAINTENANCE, payload);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/maintenance/enter] motor: ${motorIndex}, session: ${sessId}` });
    res.json({ ok: true, message: 'Maintenance mode enter command sent.', sessionId: sessId });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/maintenance/set_output', (req, res) => {
  const { sessionId, motorIndex, direction, output, enable } = req.body;
  if (sessionId === undefined || motorIndex === undefined || direction === undefined || output === undefined) {
    return res.status(400).json({ ok: false, error: 'Missing required parameters.' });
  }
  
  if (serialPort && serialPort.isOpen) {
    const payload = [
      1, // version major
      0, // sequence number placeholder
      sessionId & 0xFF,
      (sessionId >> 8) & 0xFF,
      (sessionId >> 16) & 0xFF,
      (sessionId >> 24) & 0xFF,
      motorIndex,
      direction,
      output & 0xFF,
      enable ? 1 : 0
    ];
    const pkt = buildPacket(FUNC_MAINTENANCE_SET_OUTPUT, payload);
    serialPort.write(pkt);
    res.json({ ok: true, message: 'Maintenance output updated.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/maintenance/exit', (req, res) => {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_EXIT_MAINTENANCE, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/maintenance/exit]` });
    res.json({ ok: true, message: 'Maintenance mode exit command sent.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.get('/api/maintenance/status', (req, res) => {
  if (latestMaintenanceStatus) {
    res.json({ ok: true, status: latestMaintenanceStatus });
  } else {
    res.json({ ok: true, status: { active: false } });
  }
});

app.post('/api/calibration/verify_ready', (req, res) => {
  const { motorDirectionsVerified, encoderDirectionsVerified, maintenanceStopVerified, emergencyStopVerified, deadmanVerified } = req.body;
  if (serialPort && serialPort.isOpen) {
    const payload = [
      motorDirectionsVerified ? 1 : 0,
      encoderDirectionsVerified ? 1 : 0,
      maintenanceStopVerified ? 1 : 0,
      emergencyStopVerified ? 1 : 0,
      deadmanVerified ? 1 : 0
    ];
    const pkt = buildPacket(0x2A, payload);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/calibration/verify_ready]` });
    res.json({ ok: true, message: 'Readiness verification command sent.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.get('/api/calibration/results', (req, res) => {
  if (latestCalibrationStatus) {
    res.json({
      ok: true,
      simulated: latestCalibrationStatus.isSimulation,
      saved_to_nvs: latestCalibrationStatus.isSimulation ? false : (latestCalibrationStatus.cal_state === 5),
      forward: latestCalibrationStatus.cal_fwd,
      reverse: latestCalibrationStatus.cal_rev,
      sessionId: latestCalibrationStatus.sessionId
    });
  } else {
    res.status(404).json({ ok: false, error: 'No calibration results available.' });
  }
});

app.get('/api/calibration/export', (req, res) => {
  res.setHeader('Content-disposition', 'attachment; filename=calibration_db.json');
  res.setHeader('Content-type', 'application/json');
  res.send(JSON.stringify(calibrationDb, null, 2));
});

app.get('/api/faults/clear', (req, res) => {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_CLEAR_FAULTS, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/faults/clear] clear command sent` });
    res.json({ ok: true, message: 'Faults clear command sent over Serial.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.get('/api/firmware', (req, res) => {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_GET_FIRMWARE_INFO, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/firmware] get info command sent` });
    res.json({ ok: true, message: 'Firmware info query requested over Serial.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/drive/arm', (req, res) => {
  targetLinear = 0.0;
  targetAngular = 0.0;
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_ARM_NORMAL_DRIVE, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/drive/arm] arm command sent` });
    res.json({ ok: true, message: 'Normal drive arm command sent.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.post('/api/drive/disarm', (req, res) => {
  targetLinear = 0.0;
  targetAngular = 0.0;
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_DISARM_NORMAL_DRIVE, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP POST /api/drive/disarm] disarm command sent` });
    res.json({ ok: true, message: 'Normal drive disarm command sent.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

app.get('/api/status', (req, res) => {
  res.json({
    ok: true,
    serialConnected: (serialPort && serialPort.isOpen) === true,
    port: COM_PORT,
    lastPacketAgeMs: lastTelemetryReceivedTime ? (Date.now() - lastTelemetryReceivedTime) : null,
    armed: latestNormalDriveStatus ? latestNormalDriveStatus.armed : false
  });
});

app.get('/api/drive/status', (req, res) => {
  res.json({
    ok: true,
    status: latestNormalDriveStatus || { armed: false },
    floorTesting,
    backtracking,
    recording,
    pathLength: recordedPath.length,
    odom: { x: odomX, y: odomY, yaw: odomYaw, left: accumLeftDist, right: accumRightDist }
  });
});

app.post('/api/drive/limits', (req, res) => {
  floorTesting = req.body.floorTesting !== false;
  console.log(`[Drive] Limits configured: floorTesting=${floorTesting}`);
  saveCalibrationDb();
  broadcast({ type: 'limits_status', floorTesting });
  res.json({ ok: true, floorTesting });
});

app.post('/api/path/record/start', (req, res) => {
  recording = true;
  console.log('[Path] Path recording started.');
  broadcast({ type: 'path_status', recording, pathLength: recordedPath.length });
  res.json({ ok: true, recording });
});

app.post('/api/path/record/stop', (req, res) => {
  recording = false;
  console.log('[Path] Path recording stopped.');
  broadcast({ type: 'path_status', recording, pathLength: recordedPath.length });
  res.json({ ok: true, recording });
});

app.post('/api/path/record/clear', (req, res) => {
  recording = false;
  recordedPath = [];
  console.log('[Path] Recorded path cleared.');
  broadcast({ type: 'path_status', recording, pathLength: 0 });
  res.json({ ok: true, recording, pathLength: 0 });
});

app.post('/api/path/backtrack/start', (req, res) => {
  if (backtracking) {
    return res.status(400).json({ ok: false, error: 'Backtracking already in progress' });
  }
  if (recordedPath.length === 0) {
    return res.status(400).json({ ok: false, error: 'No recorded path available' });
  }

  const limLinear = latestNormalDriveStatus ? latestNormalDriveStatus.limLinear : 0;
  const limAngular = latestNormalDriveStatus ? latestNormalDriveStatus.limAngular : 0;
  
  if (Math.abs(limLinear) > 0.02 || Math.abs(limAngular) > 0.02) {
    return res.status(400).json({ ok: false, error: 'Rover must be stationary to start backtracking' });
  }

  if (!deadmanPressed) {
    return res.status(400).json({ ok: false, error: 'Deadman must be held to start backtracking' });
  }

  backtracking = true;
  backtrackIndex = recordedPath.length - 1;
  cmdSource = 'BACKTRACK';
  startDriveKeepaliveLoop();
  
  console.log(`[Backtrack] Started backtracking with ${recordedPath.length} waypoints.`);
  broadcast({ type: 'backtrack_status', status: 'started', total: recordedPath.length });
  res.json({ ok: true, message: 'Backtracking started.' });
});

app.post('/api/path/backtrack/stop', (req, res) => {
  if (!backtracking) {
    return res.status(400).json({ ok: false, error: 'Backtracking not in progress' });
  }
  abortBacktrack('Operator stop request');
  res.json({ ok: true, message: 'Backtracking aborted.' });
});

function abortBacktrack(reason) {
  if (!backtracking) return;
  backtracking = false;
  backtrackIndex = -1;
  targetLinear = 0.0;
  targetAngular = 0.0;
  cmdSource = 'NONE';
  console.log(`[Backtrack] Aborted: ${reason}`);
  broadcast({ type: 'backtrack_status', status: 'aborted', reason });
}

app.get('/api/timing/reset', (req, res) => {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_RESET_TIMING_STATS, [1]);
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/timing/reset] stats reset command sent` });
    res.json({ ok: true, message: 'Loop timing stats reset request sent over Serial.' });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});

function handleAutoTestNextStep() {
  if (autoTestStep < 1 || autoTestStep > 18) {
    autoTestStep = 0;
    
    // Save completed run logs
    try {
      fs.writeFileSync('public/autotest_data.json', JSON.stringify(autoTestRunLogs, null, 2));
      let csv = 'Timestamp,Step,M1_Ticks,M2_Ticks,M3_Ticks,M4_Ticks,SpeedLimit,Drift_mm,Mismatch_pct\n';
      autoTestRunLogs.forEach(row => {
        csv += `${row.timestamp},${row.step},${row.m1},${row.m2},${row.m3},${row.m4},${row.speedLimit},${row.driftMm},${row.mismatchPct}\n`;
      });
      fs.writeFileSync('public/autotest_data.csv', csv);
      console.log('[Auto Test] Data logs saved successfully (JSON & CSV).');
    } catch (err) {
      console.error('[Auto Test] Failed to write logs:', err.message);
    }
    
    broadcast({ type: 'autotest_status', step: 0, msg: 'Auto-test sequence completed successfully.' });
    if (serialPort && serialPort.isOpen) {
      const pkt = buildPacket(FUNC_BEEP, [200 & 0xFF, (200 >> 8) & 0xFF]);
      serialPort.write(pkt);
    }
    return;
  }

  // Set speed limit based on cycle
  let speedLimit = 25;
  let stageName = "Slow";
  let cycle = Math.ceil(autoTestStep / 2);
  let repeatNum = 1;
  
  if (autoTestStep <= 6) {
    speedLimit = 25;
    stageName = "Slow";
    repeatNum = Math.ceil(autoTestStep / 2);
  } else if (autoTestStep <= 12) {
    speedLimit = 45;
    stageName = "Medium";
    repeatNum = Math.ceil((autoTestStep - 6) / 2);
  } else {
    speedLimit = 65;
    stageName = "Fast";
    repeatNum = Math.ceil((autoTestStep - 12) / 2);
  }
  currentAutoTestSpeedLimit = speedLimit;

  const targetDistanceMeters = 3.0 * 0.3048; // 0.9144 m
  const ticksPerMeter = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS);
  const targetTicksOffset = Math.round(targetDistanceMeters * ticksPerMeter);

  const isForward = (autoTestStep % 2 === 1);
  for (let i = 0; i < 4; i++) {
    if (isForward) {
      targetPosition[i] = autoTestStartTicks[i] + targetTicksOffset;
    } else {
      targetPosition[i] = autoTestStartTicks[i];
    }
    positionMode[i] = true;
  }

  const legType = isForward ? 'Forward' : 'Backward';
  const infoMsg = `Driving ${legType} (${stageName} Repeat ${repeatNum}/3, Leg ${autoTestStep}/18) - Speed Limit: ${currentAutoTestSpeedLimit}% - Target Ticks: ${isForward ? '+' : ''}${isForward ? targetTicksOffset : 0}`;
  console.log(`[Auto Test] ${infoMsg}`);
  broadcast({ type: 'autotest_status', step: autoTestStep, msg: infoMsg });
}

app.get('/api/autotest/start', (req, res) => {
  if (autoTestStep > 0) {
    return res.json({ ok: false, error: 'Auto-test is already running.' });
  }
  console.log('[Auto Test] Launching 3ft forward/backward auto-test sequence (3 loops)...');
  
  // Clear any existing logs for the new run
  autoTestRunLogs = [];
  
  // Automatically clear faults and arm normal drive to ensure the wheels can receive commands
  if (serialPort && serialPort.isOpen) {
    const clearPkt = buildPacket(FUNC_CLEAR_FAULTS, [1]);
    serialPort.write(clearPkt);
    const armPkt = buildPacket(FUNC_ARM_NORMAL_DRIVE, [1]);
    serialPort.write(armPkt);
    console.log('[Auto Test] Cleared safety faults and sent arming command to ESP32.');
  }

  autoTestStartTicks = [...currentTicks];
  autoTestStep = 1;
  handleAutoTestNextStep();
  res.json({ ok: true, message: 'Auto-test sequence started.' });
});

app.get('/api/autotest/abort', (req, res) => {
  autoTestStep = 0;
  positionMode = [false, false, false, false];
  sendMotorSpeeds(0, 0, 0, 0);
  
  // Save whatever logs we collected before the abort
  try {
    fs.writeFileSync('public/autotest_data.json', JSON.stringify(autoTestRunLogs, null, 2));
    let csv = 'Timestamp,Step,M1_Ticks,M2_Ticks,M3_Ticks,M4_Ticks,SpeedLimit,Drift_mm,Mismatch_pct\n';
    autoTestRunLogs.forEach(row => {
      csv += `${row.timestamp},${row.step},${row.m1},${row.m2},${row.m3},${row.m4},${row.speedLimit},${row.driftMm},${row.mismatchPct}\n`;
    });
    fs.writeFileSync('public/autotest_data.csv', csv);
    console.log('[Auto Test] Aborted run logs saved.');
  } catch (err) {
    console.error('[Auto Test] Failed to write logs on abort:', err.message);
  }
  
  console.log('[Auto Test] Sequence ABORTED - all wheels stopped.');
  broadcast({ type: 'autotest_status', step: 0, msg: 'Auto-test sequence ABORTED.' });
  res.json({ ok: true, message: 'Auto-test sequence aborted.' });
});


app.get('/api/turn', (req, res) => {
  if (req.query.stop || req.query.estop) {
    positionMode = [false, false, false, false];
    sendMotorSpeeds(0, 0, 0, 0);
    console.log('[Position Control] ESTOP triggered - all wheels stopped.');
    return res.json({ ok: true, stopped: true });
  }

  let active = false;
  ['m1', 'm2', 'm3', 'm4'].forEach((key, idx) => {
    if (req.query[key]) {
      const turns = parseFloat(req.query[key]);
      targetPosition[idx] = currentTicks[idx] + Math.round(turns * TICKS_PER_REV);
      positionMode[idx] = true;
      active = true;
      console.log(`[Position Control] Motor ${idx + 1} target set to ${targetPosition[idx]} (current: ${currentTicks[idx]})`);
    }
  });

  res.json({ ok: true, positionMode, targetPosition });
});

app.get('/api/beep', (req, res) => {
  const ms = Math.max(50, Math.min(5000, parseInt(req.query.ms || 200)));
  const pkt = buildPacket(FUNC_BEEP, [ms & 0xFF, (ms >> 8) & 0xFF]);
  const hex = Array.from(pkt).map(b => b.toString(16).padStart(2, '0')).join(' ');
  if (serialPort && serialPort.isOpen) {
    serialPort.write(pkt);
    res.json({ ok: true, packet: hex, ms });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open' });
  }
});



// Helper to send a beep command to the ESP32
function sendBeep(ms) {
  if (serialPort && serialPort.isOpen) {
    const pkt = buildPacket(FUNC_BEEP, [ms & 0xFF, (ms >> 8) & 0xFF]);
    serialPort.write(pkt);
  }
}

// Aborts the active LiDAR straight-line test safely
function abortLidarTest(reason) {
  console.warn(`[Auto Calib Abort] ${reason}`);
  autoCalibActive = false;
  lidarTestState = 'IDLE';  // go straight to IDLE so manual control works immediately
  stopTestPosePolling();
  
  // Safe stop: send zero velocities to ESP32
  targetLinear = 0.0;
  targetAngular = 0.0;
  lastCorrectionApplied = 0.0;
  cmdSource = 'NONE';
  
  if (serialPort && serialPort.isOpen) {
    const stopPkt = buildPacket(FUNC_MOTION, [0, 0, 0, 0, 0, 0]);
    serialPort.write(stopPkt);
  }
  
  sendBeep(400);
  
  broadcast({
    type: 'lidar_test_status',
    state: lidarTestState,
    msg: `Test ABORTED: ${reason}. Manual control restored.`,
    reason: reason
  });
  
  http.get(LIDAR_TEST_STOP_URL, () => {}).on('error', () => {});
}

// Computes the path controller step and runs the state machine
function updatePathController() {
  const now = Date.now();
  if (lastPathControllerTime === null) {
    lastPathControllerTime = now;
    return;
  }
  const dt = (now - lastPathControllerTime) / 1000.0;
  lastPathControllerTime = now;
  
  if (lidarTestState === 'IDLE' || lidarTestState === 'ABORTED' || lidarTestState === 'ZEROING' || lidarTestState === 'COMPLETE') {
    targetLinear = 0.0;
    targetAngular = 0.0;
    lastCorrectionApplied = 0.0;
    return;
  }
  
  if (lidarTestState === 'RETURNING_HOME_WAIT') {
    targetLinear = 0.0;
    targetAngular = 0.0;
    lastCorrectionApplied = 0.0;
    if (Date.now() - returnHomeWaitStartTime > 2000) {
      lidarTestState = 'RETURNING_HOME';
      odomStartTicks = [...currentTicks];
      lastOdomPose = { x: 0, y: 0, yaw: 0 };
      calibLegStartTime = Date.now();
      calibLastMoveTime = Date.now();
      calibLastX = lidarPose.x;
      calibLastTicks = [...currentTicks];
      calibSpeedBoost = 0.0;
      lateralErrorSum = 0.0;
      
      sendBeep(150);
      console.log('[Auto Calib] Returning home started (REVERSE).');
      broadcastAutoCalibStatus('Returning home started (REVERSE)...');
    }
    return;
  }
  
  // 1. Calculate encoder relative odometry
  const relTicks = currentTicks.map((t, idx) => t - odomStartTicks[idx]);
  const avgLeft = (relTicks[0] + relTicks[2]) / 2.0;
  const avgRight = (relTicks[1] + relTicks[3]) / 2.0;
  
  const ticksPerMeter = TICKS_PER_REV / (2.0 * Math.PI * WHEEL_RADIUS);
  const distL = avgLeft / ticksPerMeter;
  const distR = avgRight / ticksPerMeter;
  const distTotal = (distL + distR) / 2.0;
  const odomYaw = (distR - distL) / TRACK_WIDTH;
  
  lastOdomPose = {
    x: distTotal * Math.cos(odomYaw),
    y: distTotal * Math.sin(odomYaw),
    yaw: odomYaw
  };
  
  // 2. Get current speed for this tier
  const currentSpeed = AUTO_CALIB_SPEEDS[autoCalibSpeedTier] || 0.08;
  const tierLabel = AUTO_CALIB_SPEED_LABELS[autoCalibSpeedTier] || 'MED';
  
  // 3. Evaluate state target conditions & Deceleration profile
  const x_meters = lidarPose.x;

  // Monitor for correct movement (stall check and direction safety bounds)
  if (lidarTestState === 'FORWARD_RUNNING') {
    const isDecel = (x_meters >= 0.762);
    
    if (!isDecel) {
      const elapsedLeg = Date.now() - calibLegStartTime;
      if (elapsedLeg > 500) {
        const deltaX = lidarPose.x - calibLastX;
        const tickDiff = currentTicks.reduce((sum, t, idx) => sum + Math.abs(t - calibLastTicks[idx]), 0);
        
        const progressX = deltaX;
        
        // If we see significant progress in correct direction (1.0cm) or wheels spinning (sum of 15 ticks across all wheels)
        const actualSpeed = Math.abs(measuredLinearVel);
        const targetSpeed = Math.abs(targetLinear);
        if (progressX > 0.01 || tickDiff > 15 || actualSpeed >= 0.7 * targetSpeed) {
          calibLastMoveTime = Date.now();
          calibLastX = lidarPose.x;
          calibLastTicks = [...currentTicks];
          calibSpeedBoost = Math.max(0.0, calibSpeedBoost - 0.005 * dt);
        } else {
          calibSpeedBoost = Math.min(0.08, calibSpeedBoost + 0.01 * dt);
        }
        
        // Safety: check for moving in the wrong direction by more than 8cm
        if (progressX < -0.08) {
          abortLidarTest(`Incorrect movement direction detected. Expected FORWARD_RUNNING, but drifted in opposite direction.`);
          return;
        }
        
        // Check for stall (no progress or wheel spin for 4.0 seconds)
        const stallDuration = Date.now() - calibLastMoveTime;
        if (stallDuration > 4000) {
          abortLidarTest(`Stall detected: Rover failed to move for 4.0s during FORWARD_RUNNING.`);
          return;
        }
      }
    } else {
      calibLastMoveTime = Date.now();
      calibLastX = lidarPose.x;
      calibLastTicks = [...currentTicks];
    }
  }

  // Monitor progress for RETURNING_HOME
  if (lidarTestState === 'RETURNING_HOME') {
    const isDecel = (x_meters <= 0.20);
    
    if (!isDecel) {
      const elapsedLeg = Date.now() - calibLegStartTime;
      if (elapsedLeg > 500) {
        const deltaX = calibLastX - lidarPose.x; // positive progress when moving backward
        const tickDiff = currentTicks.reduce((sum, t, idx) => sum + Math.abs(t - calibLastTicks[idx]), 0);
        
        const progressX = deltaX;
        const actualSpeed = Math.abs(measuredLinearVel);
        const targetSpeed = Math.abs(targetLinear);
        if (progressX > 0.01 || tickDiff > 15 || actualSpeed >= 0.7 * targetSpeed) {
          calibLastMoveTime = Date.now();
          calibLastX = lidarPose.x;
          calibLastTicks = [...currentTicks];
          calibSpeedBoost = Math.max(0.0, calibSpeedBoost - 0.005 * dt);
        } else {
          calibSpeedBoost = Math.min(0.08, calibSpeedBoost + 0.01 * dt);
        }
        
        if (progressX < -0.08) {
          abortLidarTest(`Incorrect movement direction detected. Expected RETURNING_HOME, but drifted in opposite direction.`);
          return;
        }
        
        const stallDuration = Date.now() - calibLastMoveTime;
        if (stallDuration > 4000) {
          abortLidarTest(`Stall detected: Rover failed to move for 4.0s during RETURNING_HOME.`);
          return;
        }
      }
    } else {
      calibLastMoveTime = Date.now();
      calibLastX = lidarPose.x;
      calibLastTicks = [...currentTicks];
    }
  }
  
  if (lidarTestState === 'FORWARD_RUNNING') {
    testDirection = 'FORWARD';
    targetLinear = currentSpeed + calibSpeedBoost;
    cmdSource = 'LIDAR_TEST';
    
    if (x_meters >= 0.9144) {
      targetLinear = 0.0;
      targetAngular = 0.0;
      lastCorrectionApplied = 0.0;
      sendBeep(300);
      
      saveCompletedPassSummary();
      computeFinalTrims();
      
      lidarTestState = 'RETURNING_HOME_WAIT';
      returnHomeWaitStartTime = Date.now();
      
      console.log('[Auto Calib] Forward pass complete. Saving trims and starting return-to-home sequence...');
      broadcastAutoCalibStatus('Forward pass complete. Pausing before return...');
      return;
    } else if (x_meters >= 0.762) {
      const scale = (0.9144 - x_meters) / 0.1524;
      const clampedScale = Math.max(0.15, Math.min(1.0, scale));
      targetLinear = targetLinear * clampedScale;
    }
  }

  if (lidarTestState === 'RETURNING_HOME') {
    testDirection = 'REVERSE';
    targetLinear = -currentSpeed - calibSpeedBoost;
    cmdSource = 'LIDAR_TEST';
    
    const x_meters = lidarPose.x;
    if (x_meters <= 0.05) {
      targetLinear = 0.0;
      targetAngular = 0.0;
      lastCorrectionApplied = 0.0;
      sendBeep(500);
      
      lidarTestState = 'COMPLETE';
      autoCalibActive = false;
      
      console.log('[Auto Calib] Returned home successfully. Calibration finished.');
      broadcastAutoCalibStatus('Returned home successfully. Calibration finished.');
      stopTestPosePolling();
      http.get(LIDAR_TEST_STOP_URL, () => {}).on('error', () => {});
      
      setTimeout(() => {
        lidarTestState = 'IDLE';
        broadcast({ type: 'lidar_test_status', state: lidarTestState, msg: 'Calibration complete. Manual control restored.' });
      }, 5000);
      return;
    } else if (x_meters <= 0.20) {
      const scale = (x_meters - 0.05) / 0.15;
      const clampedScale = Math.max(0.15, Math.min(1.0, scale));
      targetLinear = targetLinear * clampedScale;
    }
  }
  
  // 4. Compute path correction (always active — LiDAR auto-corrects drift)
  let correction = 0.0;
  let sampleAccepted = false;
  
  if (lidarMetrics.scanAgeMs > maxScanAgeMs || lidarMetrics.confidence < minConfidence || lidarMetrics.rejectionReason) {
    correction = lastCorrectionApplied * 0.8;
    lastCorrectionApplied = correction;
  } else {
    const e_y = lidarPose.y;
    const e_yaw = lidarPose.yaw;
    const v_sign = (testDirection === 'FORWARD') ? 1.0 : -1.0;
    
    // Accumulate lateral error for integral action to eliminate steady-state drift
    lateralErrorSum += e_y * dt;
    lateralErrorSum = Math.max(-0.15, Math.min(0.15, lateralErrorSum));
    
    const omega_raw = - (v_sign * lateralGain * e_y + v_sign * lateralKi * lateralErrorSum + headingGain * e_yaw);
    
    const max_delta = corrSlewRate * dt;
    const d_omega = omega_raw - lastCorrectionApplied;
    const clamped_d = Math.max(-max_delta, Math.min(max_delta, d_omega));
    correction = lastCorrectionApplied + clamped_d;
    
    correction = Math.max(-maxAngularCorr, Math.min(maxAngularCorr, correction));
    
    const speedScale = Math.min(1.0, Math.abs(targetLinear) / 0.05);
    correction = correction * speedScale;
    
    lastCorrectionApplied = correction;
    sampleAccepted = true;
  }
  
  // Always apply correction (test is always in correct mode)
  targetAngular = correction;
  
  // 5. High-rate session logging
  const timestamp = Date.now();
  const logRow = {
    timestamp,
    sessionId: testSessionId,
    passNumber: autoCalibTotalPass,
    speedTier: tierLabel,
    direction: testDirection,
    requestedSpeed: targetLinear,
    m1_ticks: currentTicks[0],
    m2_ticks: currentTicks[1],
    m3_ticks: currentTicks[2],
    m4_ticks: currentTicks[3],
    odomX: lastOdomPose.x,
    odomY: lastOdomPose.y,
    odomYaw: lastOdomPose.yaw,
    lidarX: lidarPose.x,
    lidarY: lidarPose.y,
    lidarYaw: lidarPose.yaw,
    headingError: lidarPose.yaw,
    lateralError: lidarPose.y,
    appliedCorrection: targetAngular,
    leftTrimOffset: activeFwdTrim.left,
    rightTrimOffset: activeFwdTrim.right,
    confidence: lidarMetrics.confidence,
    rmse: lidarMetrics.rmse,
    sampleAccepted
  };
  
  if (lidarTestState === 'FORWARD_RUNNING') {
    sessionLogs.push(logRow);
  }
  
  // Estimate motor power output (PWM magnitude percentage)
  const L_width = 0.170; // track width (m)
  const r_wheel = 0.0325; // wheel radius (m)
  const kV_approx = 45.0; // kV parameter approx
  const fwd_breakaway_approx = 45.0; // breakaway PWM approx
  
  const leftMps = targetLinear - (targetAngular * L_width / 2.0);
  const rightMps = targetLinear + (targetAngular * L_width / 2.0);
  
  const leftRadps = leftMps / r_wheel;
  const rightRadps = rightMps / r_wheel;
  
  let leftPwm = 0;
  let rightPwm = 0;
  if (Math.abs(leftMps) > 0.005) {
    const sign = Math.sign(leftRadps);
    leftPwm = sign * fwd_breakaway_approx + kV_approx * leftRadps;
  }
  if (Math.abs(rightMps) > 0.005) {
    const sign = Math.sign(rightRadps);
    rightPwm = sign * fwd_breakaway_approx + kV_approx * rightRadps;
  }
  
  leftPwm = Math.max(-255, Math.min(255, leftPwm));
  rightPwm = Math.max(-255, Math.min(255, rightPwm));
  
  const leftPowerPct = Math.round((Math.abs(leftPwm) / 255.0) * 100);
  const rightPowerPct = Math.round((Math.abs(rightPwm) / 255.0) * 100);

  // Broadcast telemetry updates to WebSocket clients
  broadcast({
    type: 'lidar_test_telemetry',
    state: lidarTestState,
    direction: testDirection,
    requestedSpeed: targetLinear,
    leftPowerPct,
    rightPowerPct,
    odomPose: lastOdomPose,
    lidarPose: lidarPose,
    metrics: lidarMetrics,
    appliedCorrection: targetAngular,
    lastCorrection: lastCorrectionApplied,
    speedTier: tierLabel,
    passIndex: autoCalibPassIndex + 1,
    totalPass: autoCalibTotalPass + 1,
    totalPasses: AUTO_CALIB_SPEEDS.length * AUTO_CALIB_PASSES_PER_TIER
  });

  // 6. 1Hz rate console logging for calibration diagnostics
  const elapsed = Date.now() - calibLegStartTime;
  if (elapsed % 1000 < 100) { 
    console.log(`[Auto Calib Debug] State: ${lidarTestState} | X: ${lidarPose.x.toFixed(3)}m, Y: ${lidarPose.y.toFixed(4)}m, Yaw: ${lidarPose.yaw.toFixed(4)}rad | Cmd: V_lin=${targetLinear.toFixed(3)}m/s, W_ang=${targetAngular.toFixed(3)}rad/s | Boost: ${calibSpeedBoost.toFixed(3)}m/s | Integrator: ${lateralErrorSum.toFixed(4)}`);
  }
}

// Helper: broadcast auto-calib status with progress info
function broadcastAutoCalibStatus(msg) {
  broadcast({
    type: 'lidar_test_status',
    state: lidarTestState,
    msg,
    speedTier: AUTO_CALIB_SPEED_LABELS[autoCalibSpeedTier],
    passIndex: autoCalibPassIndex + 1,
    totalPass: autoCalibTotalPass + 1,
    totalPasses: AUTO_CALIB_SPEEDS.length * AUTO_CALIB_PASSES_PER_TIER
  });
}

// Compute final proposed trims from the completed passes
function computeFinalTrims() {
  proposedFwdTrim.left = activeFwdTrim.left;
  proposedFwdTrim.right = activeFwdTrim.right;
  proposedRevTrim.left = activeRevTrim.left;
  proposedRevTrim.right = activeRevTrim.right;
  
  acceptedPasses = passSummaries.length;
  calibrationConfidence = 1.0;
  
  console.log(`[Auto Calib] Final proposed trims — FWD L=${proposedFwdTrim.left.toFixed(4)} R=${proposedFwdTrim.right.toFixed(4)} | REV L=${proposedRevTrim.left.toFixed(4)} R=${proposedRevTrim.right.toFixed(4)} | Conf=${calibrationConfidence.toFixed(2)}`);
  
  // Save session data
  try {
    fs.writeFileSync('public/lidar_test_session.json', JSON.stringify(sessionLogs, null, 2));
    let csv = 'Timestamp,Pass,SpeedTier,Direction,RequestedSpeed,OdomX,OdomY,OdomYaw,LidarX,LidarY,LidarYaw,Correction,FwdTrimL,FwdTrimR,Confidence\n';
    sessionLogs.forEach(row => {
      csv += `${row.timestamp},${row.passNumber},${row.speedTier},${row.direction},${row.requestedSpeed},${row.odomX.toFixed(4)},${row.odomY.toFixed(4)},${row.odomYaw.toFixed(4)},${row.lidarX.toFixed(4)},${row.lidarY.toFixed(4)},${row.lidarYaw.toFixed(4)},${row.appliedCorrection.toFixed(4)},${row.leftTrimOffset.toFixed(4)},${row.rightTrimOffset.toFixed(4)},${row.confidence.toFixed(4)}\n`;
    });
    fs.writeFileSync('public/lidar_test_session.csv', csv);
    fs.writeFileSync('public/lidar_test_summaries.json', JSON.stringify({ passSummaries, proposedFwdTrim, proposedRevTrim, calibrationConfidence }, null, 2));
  } catch (err) {
    console.error('[Auto Calib] Failed to write session data:', err.message);
  }
  
  broadcast({
    type: 'lidar_test_results',
    acceptedPasses,
    passSummaries,
    proposedFwdTrim,
    proposedRevTrim,
    calibrationConfidence
  });
}

// Summarize and learn proposed trims when a pass is completed
function saveCompletedPassSummary() {
  if (sessionLogs.length === 0) return;
  
  const currentPassRows = sessionLogs.filter(r => r.sampleAccepted);
  const speedVal = AUTO_CALIB_SPEEDS[autoCalibSpeedTier] || 0.11;
  const tierLab = AUTO_CALIB_SPEED_LABELS[autoCalibSpeedTier] || 'MED';
  
  if (currentPassRows.length > 0) {
    const maxLatErr = Math.max(...currentPassRows.map(r => Math.abs(r.lateralError)));
    const finalLatErr = currentPassRows[currentPassRows.length - 1].lateralError;
    const maxHeadErr = Math.max(...currentPassRows.map(r => Math.abs(r.headingError)));
    const finalHeadErr = currentPassRows[currentPassRows.length - 1].headingError;
    const avgCorr = currentPassRows.reduce((sum, r) => sum + r.appliedCorrection, 0) / currentPassRows.length;
    const peakCorr = Math.max(...currentPassRows.map(r => Math.abs(r.appliedCorrection)));
    const disagreement = Math.max(...currentPassRows.map(r => Math.abs(r.odomX - r.lidarX)));
    
    const direction = 'FORWARD';
    const distanceVal = currentPassRows[currentPassRows.length - 1].lidarX;
    
    const passStats = {
      direction,
      speedTier: tierLab,
      speed: speedVal,
      distance: distanceVal,
      maxLateralError: maxLatErr,
      finalLateralError: finalLatErr,
      maxHeadingError: maxHeadErr,
      finalHeadingError: finalHeadErr,
      avgCorrection: avgCorr,
      peakCorrection: peakCorr,
      disagreement
    };
    passSummaries.push(passStats);
    
    // Calculate the hardware trim: exact inverse of the average control effort
    const trimOffset = (avgCorr * TRACK_WIDTH) / speedVal;
    
    activeFwdTrim.left = Math.max(0.80, Math.min(1.20, activeFwdTrim.left * (1 - trimOffset)));
    activeFwdTrim.right = Math.max(0.80, Math.min(1.20, activeFwdTrim.right * (1 + trimOffset)));
    
    proposedFwdTrim.left = activeFwdTrim.left;
    proposedFwdTrim.right = activeFwdTrim.right;
    
    // Calibrate encoder motors on ESP32: apply and save trims
    if (serialPort && serialPort.isOpen) {
      const leftBytes = floatToLEBytes(activeFwdTrim.left);
      const rightBytes = floatToLEBytes(activeFwdTrim.right);
      sendBinaryCommand(0x38, [...leftBytes, ...rightBytes], { dualChecksum: true });
      setTimeout(() => {
        sendBinaryCommand(0x38, [], { dualChecksum: true });
      }, 100);
      console.log(`[Auto Calib] Dynamically saved FWD trims on ESP32: L=${activeFwdTrim.left.toFixed(4)} R=${activeFwdTrim.right.toFixed(4)}`);
    }
    
    saveCalibrationDb();
    
    acceptedPasses = passSummaries.length;
    calibrationConfidence = 1.0;
  }
  
  try {
    fs.writeFileSync('public/lidar_test_session.json', JSON.stringify(sessionLogs, null, 2));
    let csv = 'Timestamp,Pass,Direction,RequestedSpeed,OdomX,OdomY,OdomYaw,LidarX,LidarY,LidarYaw,Correction,FwdTrimL,FwdTrimR,Confidence\n';
    sessionLogs.forEach(row => {
      csv += `${row.timestamp},${row.passNumber},${row.direction},${row.requestedSpeed},${row.odomX.toFixed(4)},${row.odomY.toFixed(4)},${row.odomYaw.toFixed(4)},${row.lidarX.toFixed(4)},${row.lidarY.toFixed(4)},${row.lidarYaw.toFixed(4)},${row.appliedCorrection.toFixed(4)},${row.leftTrimOffset.toFixed(4)},${row.rightTrimOffset.toFixed(4)},${row.confidence.toFixed(4)}\n`;
    });
    fs.writeFileSync('public/lidar_test_session.csv', csv);
    fs.writeFileSync('public/lidar_test_summaries.json', JSON.stringify(passSummaries, null, 2));
  } catch (err) {
    console.error('[LiDAR Test Logs] Failed to write logs:', err.message);
  }
  
  broadcast({
    type: 'lidar_test_results',
    acceptedPasses,
    passSummaries,
    proposedFwdTrim,
    proposedRevTrim,
    calibrationConfidence
  });
}

let testPoseInterval = null;

function startTestPosePolling() {
  if (testPoseInterval) return;
  lastPathControllerTime = Date.now();
  
  testPoseInterval = setInterval(() => {
    if (lidarTestState === 'IDLE') {
      stopTestPosePolling();
      return;
    }
    
    http.get(LIDAR_TEST_POSE_URL, (sRes) => {
      let body = '';
      sRes.on('data', (c) => { body += c; });
      sRes.on('end', () => {
        try {
          const resObj = JSON.parse(body);
          if (resObj && resObj.pose) {
            lidarPose = resObj.pose;
            lidarMetrics = resObj.metrics;
            
            if (lidarTestState === 'ZEROING' && resObj.state === 'READY') {
              lidarTestState = 'FORWARD_RUNNING';
              odomStartTicks = [...currentTicks];
              lastOdomPose = { x: 0, y: 0, yaw: 0 };
              testPassNumber = 1;
              lastPathControllerTime = Date.now();
              
              calibLegStartTime = Date.now();
              calibLastMoveTime = Date.now();
              calibLastX = resObj.pose.x;
              calibLastTicks = [...currentTicks];
              calibSpeedBoost = 0.0; // Reset speed boost!
              lateralErrorSum = 0.0; // Reset lateral error integrator!
              
              sendBeep(150);
              console.log('[Test State Machine] Zeroing complete. Automated forward run started.');
              broadcast({ type: 'lidar_test_status', state: lidarTestState, msg: 'Zeroing complete. Automated forward run started...' });
              startDriveKeepaliveLoop();
            }
            
            if (lidarTestState === 'ZEROING' && resObj.metrics.status === 'error') {
              abortLidarTest(resObj.metrics.rejectionReason || 'Zeroing failed.');
            }
            
            updatePathController();
          }
        } catch (e) {
          console.error('[Test Polling] Failed to parse pose response:', e.message);
        }
      });
    }).on('error', (err) => {
      console.error('[Test Polling] LiDAR sidecar not reachable:', err.message);
      if (lidarTestState !== 'ZEROING') {
        abortLidarTest('LiDAR sidecar disconnected or unreachable.');
      }
    });
  }, 100);
}

function stopTestPosePolling() {
  if (testPoseInterval) {
    clearInterval(testPoseInterval);
    testPoseInterval = null;
  }
}


// GET /api/lidar/test/start - start a test session with the python sidecar
app.get('/api/lidar/test/start', (req, res) => {
  const qs = req.url.includes('?') ? req.url.slice(req.url.indexOf('?')) : '';
  http.get(LIDAR_TEST_START_URL + qs, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ ok: false, error: 'Bad response from LiDAR sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ ok: false, error: `LiDAR sidecar not reachable: ${err.message}` });
  });
});

// GET /api/lidar/test/pose - get pose from python sidecar
app.get('/api/lidar/test/pose', (req, res) => {
  http.get(LIDAR_TEST_POSE_URL, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ error: 'Bad response from LiDAR sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ error: `LiDAR sidecar not reachable: ${err.message}` });
  });
});

// GET /api/lidar/test/stop - stop test session with python sidecar
app.get('/api/lidar/test/stop', (req, res) => {
  http.get(LIDAR_TEST_STOP_URL, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ ok: false, error: 'Bad response from LiDAR sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ ok: false, error: `LiDAR sidecar not reachable: ${err.message}` });
  });
});


// GET /api/lidar/status - proxy the latest LiDAR status from the python sidecar
app.get('/api/lidar/status', (req, res) => {
  http.get(LIDAR_STATUS_URL, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ connected: false, state: 'error', lastError: 'Bad response from LiDAR sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ connected: false, state: 'disconnected', lastError: `LiDAR sidecar not reachable: ${err.message}` });
  });
});

// GET /api/lidar/scan - proxy the latest complete LiDAR scan rotation from the python sidecar
app.get('/api/lidar/scan', (req, res) => {
  http.get(LIDAR_SCAN_URL, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ error: 'Bad response from LiDAR sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ error: `LiDAR sidecar not reachable: ${err.message}` });
  });
});

// ────────────────────────────────────────────────────────────
// Camera Streaming Engine (RPi5 CSI/USB camera support)
// ────────────────────────────────────────────────────────────
let cameraProcess = null;
let cameraClients = new Set();
let latestFrame = null;
let cameraTimeout = null;

function startCamera() {
  if (cameraProcess) return;

  console.log('[Camera] Starting camera streaming subprocess...');
  
  // Resolution 640x480, 15fps, MJPEG codec
  const width = 640;
  const height = 480;
  const fps = 15;

  const rpiArgs = [
    '-t', '0',                    // continuous streaming
    '-n',                         // do not display a preview window
    '--codec', 'mjpeg',
    '--inline',
    '--width', width.toString(),
    '--height', height.toString(),
    '--framerate', fps.toString(),
    '--hflip',                    // Rotate 180 (horizontal flip)
    '--vflip',                    // Rotate 180 (vertical flip)
    '-o', '-'                     // output to stdout
  ];

  // Try rpicam-vid first (modern RPi OS Bookworm)
  let cmd = 'rpicam-vid';
  let args = rpiArgs;

  cameraProcess = spawn(cmd, args);

  cameraProcess.on('error', (err) => {
    console.warn(`[Camera] Failed to start ${cmd}: ${err.message}. Trying libcamera-vid...`);
    cmd = 'libcamera-vid';
    cameraProcess = spawn(cmd, args);

    cameraProcess.on('error', (err2) => {
      console.warn(`[Camera] Failed to start ${cmd}: ${err2.message}. Trying ffmpeg with /dev/video0 (USB Webcam)...`);
      cmd = 'ffmpeg';
      // FFmpeg USB grab command: ffmpeg -f v4l2 -video_size 640x480 -framerate 15 -i /dev/video0 -vf vflip,hflip -c:v mjpeg -f mjpeg -
      args = [
        '-f', 'v4l2',
        '-video_size', `${width}x${height}`,
        '-framerate', fps.toString(),
        '-i', '/dev/video0',
        '-vf', 'vflip,hflip',
        '-c:v', 'mjpeg',
        '-f', 'mjpeg',
        '-'
      ];
      cameraProcess = spawn(cmd, args);

      cameraProcess.on('error', (err3) => {
        console.error(`[Camera] Failed to start USB camera stream via ffmpeg: ${err3.message}`);
        broadcast({ type: 'camera_status', status: 'error', error: 'No camera utilities succeeded.' });
        cameraProcess = null;
      });

      if (cameraProcess) handleCameraOutput(cameraProcess);
    });

    if (cameraProcess) handleCameraOutput(cameraProcess);
  });

  if (cameraProcess) handleCameraOutput(cameraProcess);
}

function handleCameraOutput(proc) {
  let buffer = Buffer.alloc(0);

  proc.stdout.on('data', (chunk) => {
    buffer = Buffer.concat([buffer, chunk]);

    while (true) {
      const startIndex = buffer.indexOf(Buffer.from([0xFF, 0xD8]));
      if (startIndex === -1) {
        // Discard buffer unless it ends with 0xFF (part of SOI)
        if (buffer.length > 0 && buffer[buffer.length - 1] === 0xFF) {
          buffer = buffer.subarray(buffer.length - 1);
        } else {
          buffer = Buffer.alloc(0);
        }
        break;
      }

      if (startIndex > 0) {
        buffer = buffer.subarray(startIndex);
      }

      const endIndex = buffer.indexOf(Buffer.from([0xFF, 0xD9]));
      if (endIndex === -1) {
        break; // Wait for the rest of the frame
      }

      const frame = buffer.subarray(0, endIndex + 2);
      buffer = buffer.subarray(endIndex + 2);

      latestFrame = frame;

      // Broadcast frame to all HTTP clients
      for (const client of cameraClients) {
        try {
          client.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`);
          client.write(frame);
          client.write('\r\n');
        } catch (err) {
          cameraClients.delete(client);
        }
      }
    }
  });

  proc.stderr.on('data', (data) => {
    const msg = data.toString().trim();
    if (msg.toLowerCase().includes('error')) {
      console.warn(`[Camera Process Error] ${msg}`);
    }
  });

  proc.on('close', (code) => {
    console.log(`[Camera] Process exited with code ${code}`);
    cameraProcess = null;
    latestFrame = null;
    // Close remaining client connections
    for (const client of cameraClients) {
      try { client.end(); } catch(e) {}
    }
    cameraClients.clear();
  });
}

function stopCamera() {
  if (!cameraProcess) return;
  console.log('[Camera] Stopping camera stream (no active clients)...');
  cameraProcess.kill('SIGINT');
  cameraProcess = null;
}

// GET /api/camera - MJPEG video stream route
app.get('/api/camera', (req, res) => {
  cameraClients.add(res);

  res.writeHead(200, {
    'Cache-Control': 'no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0',
    'Pragma': 'no-cache',
    'Connection': 'close',
    'Content-Type': 'multipart/x-mixed-replace; boundary=--frame'
  });

  if (cameraTimeout) {
    clearTimeout(cameraTimeout);
    cameraTimeout = null;
  }

  if (!cameraProcess) {
    startCamera();
  } else if (latestFrame) {
    // Write last frame immediately
    try {
      res.write(`--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${latestFrame.length}\r\n\r\n`);
      res.write(latestFrame);
      res.write('\r\n');
    } catch (e) {}
  }

  req.on('close', () => {
    cameraClients.delete(res);
    if (cameraClients.size === 0) {
      // Shutdown camera after 5 seconds of inactivity
      cameraTimeout = setTimeout(() => {
        if (cameraClients.size === 0) {
          stopCamera();
        }
      }, 5000);
    }
  });
});

// GET /api/camera/status - query current camera engine status
app.get('/api/camera/status', (req, res) => {
  res.json({
    active: cameraProcess !== null,
    clients: cameraClients.size
  });
});




// ────────────────────────────────────────────────────────────
// Startup
// ────────────────────────────────────────────────────────────
loadCalibrationDb();
startMotorKeepaliveLoop();
initSerial(COM_PORT);

server.listen(PORT, () => {
  console.log(`Maker ESP32 Pro Cockpit running at http://localhost:${PORT}`);
  console.log(`Binary protocol on ${COM_PORT} @ ${BAUD_RATE} baud`);
  console.log(`Motor test: http://localhost:${PORT}/api/motor?m1=50&m2=0&m3=0&m4=0`);
  console.log(`Stop:       http://localhost:${PORT}/api/stop`);
  console.log(`Beep:       http://localhost:${PORT}/api/beep`);

});
