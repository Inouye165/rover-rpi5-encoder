const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const path = require('path');
const httpGet = require('http');  // used for I2C sidecar polling
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

const PORT = process.env.PORT || 3000;
let COM_PORT = process.env.SERIAL_PORT;
if (process.platform === 'win32' && COM_PORT && COM_PORT.startsWith('/dev/')) {
  COM_PORT = null;
}
if (!COM_PORT) {
  if (process.platform === 'linux') {
    if (fs.existsSync('/dev/ttyUSB0')) {
      COM_PORT = '/dev/ttyUSB0';
    } else if (fs.existsSync('/dev/ttyACM0')) {
      COM_PORT = '/dev/ttyACM0';
    } else {
      COM_PORT = '/dev/ttyAMA0';
    }
  } else {
    COM_PORT = 'COM18';
  }
}
const BAUD_RATE = parseInt(process.env.BAUD_RATE) || 115200;

// ────────────────────────────────────────────────────────────
// Rosmaster Binary Protocol Constants
// Source: github.com/mattzi/rosmasterx3-golang / Yahboom SDK
// ────────────────────────────────────────────────────────────
const HEAD       = 0xFF;         // Frame start byte (both directions)
const DEVICE_ID  = 0xFC;         // Host → Board device ID
const BOARD_ID   = 0xFB;         // Board → Host device ID (0xFC - 1)
const COMPLEMENT = 257 - DEVICE_ID; // = 1; used in checksum for outgoing cmds

// Outgoing function codes (Host → Board)
const FUNC_MOTOR  = 0x10; // Set individual motor speeds (-100..100 each)
const FUNC_MOTION = 0x12; // Set velocity (vx, vy, vz as int16 * 1000)
const FUNC_BEEP   = 0x02; // Buzzer
const FUNC_CAR_TYPE = 0x44; // Car type/motion mode (1 = 4WD/Mecanum on most firmware)

// Incoming telemetry type codes (Board → Host)
// Mapping per official Rosmaster_Lib protocol:
const TYPE_BATTERY  = 0x0A; // Speed/battery packet (data[6] = voltage*10)
const TYPE_ATTITUDE = 0x0C; // IMU attitude: roll,pitch,yaw int16 LE (rad*10000)
const TYPE_ENCODER  = 0x0D; // Encoder counts: 4x int32 LE (M1..M4)
const TYPE_IMU      = 0x0E; // 9-axis IMU raw packet (21 bytes total)

// NOTE: No COMPLEMENT constant needed – checksum = sum(packet[2..n-2]) & 0xFF

// ────────────────────────────────────────────────────────────
// Express / WebSocket Setup
// ────────────────────────────────────────────────────────────
const app    = express();
const server = http.createServer(app);
const wss    = new WebSocket.Server({ server });

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

// Position Control State
let positionMode = [false, false, false, false];
let targetPosition = [0, 0, 0, 0];
let currentTicks = [0, 0, 0, 0];
const TICKS_PER_REV = 937.2;
const KP_POSITION = 0.15;
const MIN_POSITION_SPEED = 20;
const MAX_POSITION_SPEED = 60;

// ────────────────────────────────────────────────────────────
// IMU State (for sensor fusion)
// ────────────────────────────────────────────────────────────
let imuYaw = 0;
let lastImuTimestamp = null;

// ────────────────────────────────────────────────────────────
// I2C Sidecar State
// Polls yahboom_i2c.py running on localhost:3001
// ────────────────────────────────────────────────────────────
const I2C_SIDECAR_URL  = process.env.I2C_SIDECAR_URL || 'http://127.0.0.1:3001/data';
const I2C_POLL_MS      = 250;   // poll every 250 ms
let   i2cSidecarOnline = false;

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
  while (rxBuf.length >= 4) {
    // Search for 0xFF 0xFB header
    const h1 = rxBuf.indexOf(0xFF);
    if (h1 === -1 || h1 >= rxBuf.length - 1) break;
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

      // Position Control Loop (run every time we get an encoder telemetry update)
      let motorSpeeds = [0, 0, 0, 0];
      let anyPositionMode = false;
      
      for (let i = 0; i < 4; i++) {
        if (positionMode[i]) {
          anyPositionMode = true;
          const error = targetPosition[i] - currentTicks[i];
          if (Math.abs(error) <= 15) { // Widened from 3 to 15 ticks (~5.7 deg) to stop end-of-move oscillation
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
      
      if (anyPositionMode) {
        // Send motor speeds to serial
        sendMotorSpeeds(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2], motorSpeeds[3]);
      }

      encoderPacketCount += 1;
      const now = Date.now();
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

    // The board auto-streams telemetry. No explicit command needed to start it.
    // Set a default car type and send a short beep to confirm two-way communication.
    sendBinaryCommand(FUNC_CAR_TYPE, [1, 0], { dualChecksum: true });
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

  ws.on('message', (message) => {
    try {
      const msg = JSON.parse(message);

      switch (msg.type) {
        case 'set_speed':
          // Frontend sends { type:'set_speed', speeds:[m1,m2,m3,m4] } in range -1000..1000
          if (Array.isArray(msg.speeds) && msg.speeds.length === 4) {
            positionMode = [false, false, false, false];
            sendMotorSpeeds(msg.speeds[0], msg.speeds[1], msg.speeds[2], msg.speeds[3]);
          }
          break;

        case 'set_pwm':
          // Alias for set_speed (frontend uses both)
          if (Array.isArray(msg.pwms) && msg.pwms.length === 4) {
            positionMode = [false, false, false, false];
            sendMotorSpeeds(msg.pwms[0], msg.pwms[1], msg.pwms[2], msg.pwms[3]);
          }
          break;

        case 'change_port':
          if (msg.port) {
            initSerial(msg.port);
          }
          break;

        case 'raw_command':
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

        case 'beep':
          // { type: 'beep', duration: 500 }
          const onTime = msg.duration || 100;
          const lo = onTime & 0xFF;
          const hi = (onTime >> 8) & 0xFF;
          sendBinaryCommand(FUNC_BEEP, [lo, hi]);
          break;

        case 'run_motor_proof':
          runMotorProofSequence();
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
  const pkt = buildPacket(FUNC_MOTOR, [0, 0, 0, 0]);
  const hex = Array.from(pkt).map(b => b.toString(16).padStart(2, '0')).join(' ');
  if (serialPort && serialPort.isOpen) {
    serialPort.write(pkt);
    broadcast({ type: 'raw_serial_out', data: `[HTTP /api/stop] ${hex}` });
    res.json({ ok: true, packet: hex });
  } else {
    res.status(503).json({ ok: false, error: 'Serial port not open', packet: hex });
  }
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

// GET /api/i2c – proxy the latest I2C sidecar reading as JSON
app.get('/api/i2c', (req, res) => {
  httpGet.get(I2C_SIDECAR_URL, (sRes) => {
    let body = '';
    sRes.on('data', (c) => { body += c; });
    sRes.on('end', () => {
      try {
        res.json(JSON.parse(body));
      } catch (e) {
        res.status(502).json({ ok: false, error: 'Bad response from I2C sidecar' });
      }
    });
  }).on('error', (err) => {
    res.status(503).json({ ok: false, error: `I2C sidecar not reachable: ${err.message}` });
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
// I2C Sidecar Poller
// Fetches JSON from yahboom_i2c.py and re-broadcasts it to all
// WebSocket clients so the dashboard gets real I2C sensor data.
// ────────────────────────────────────────────────────────────
function startI2CSidecarPoller() {
  setInterval(() => {
    httpGet.get(I2C_SIDECAR_URL, (res) => {
      let body = '';
      res.on('data', (chunk) => { body += chunk; });
      res.on('end', () => {
        try {
          const d = JSON.parse(body);
          if (!i2cSidecarOnline) {
            i2cSidecarOnline = true;
            console.log('[I2C] Sidecar online at', I2C_SIDECAR_URL);
            broadcast({ type: 'status', key: 'i2c', val: 'connected' });
          }
          if (d.ok) {
            // Broadcast battery update from I2C
            if (d.battery !== null) {
              broadcast({ type: 'battery', voltage: d.battery, source: 'i2c' });
            }
            // Broadcast IMU update from I2C
            broadcast({
              type:  'imu',
              source: 'i2c',
              yaw:   d.yaw,
              pitch: d.pitch,
              roll:  d.roll,
              ax: d.ax, ay: d.ay, az: d.az,
              gx: d.gx, gy: d.gy, gz: d.gz,
              mx: d.mx, my: d.my, mz: d.mz,
            });
          } else {
            broadcast({ type: 'status', key: 'i2c', val: 'error', error: d.error || 'unknown' });
          }
        } catch (e) {
          // Malformed response – ignore
        }
      });
    }).on('error', (err) => {
      if (i2cSidecarOnline) {
        i2cSidecarOnline = false;
        console.warn('[I2C] Sidecar offline:', err.message);
        broadcast({ type: 'status', key: 'i2c', val: 'disconnected', error: err.message });
      }
    });
  }, I2C_POLL_MS);
}

// ────────────────────────────────────────────────────────────
// Startup
// ────────────────────────────────────────────────────────────
startMotorKeepaliveLoop();
startI2CSidecarPoller();
initSerial(COM_PORT);

server.listen(PORT, () => {
  console.log(`Maker ESP32 Pro Cockpit running at http://localhost:${PORT}`);
  console.log(`Binary protocol on ${COM_PORT} @ ${BAUD_RATE} baud`);
  console.log(`Motor test: http://localhost:${PORT}/api/motor?m1=50&m2=0&m3=0&m4=0`);
  console.log(`Stop:       http://localhost:${PORT}/api/stop`);
  console.log(`Beep:       http://localhost:${PORT}/api/beep`);

});
