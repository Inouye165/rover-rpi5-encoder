#!/usr/bin/env node
/**
 * scripts/record_turn_diagnostic.js
 * 
 * Synchronized Diagnostic Telemetry Recorder for Rover 1 Turning Audit.
 * Captures high-resolution timestamped events across all layers:
 * - Operator inputs (joystick commands, deadman)
 * - Host velocity targets and limits (reqLinear, reqAngular, limLinear, limAngular)
 * - Serial transmission packets (FUNC_MOTION, FUNC_DISARM)
 * - Firmware states (uptimeMs, bootCount, armed, mode, spinRecoveryState, noMotionBudgetMs, spinFailedExhausted)
 * - Motor PID metrics (m1..m4 targetRadps, measuredRadps, PWM, stictionState)
 * - IMU telemetry (hardwareInitialized, rotVecValid, gyroValid, gyroAgeMs, gz, orientation)
 * - Front Safety Zone proximity (state, minMm, action)
 */

const fs = require('fs');
const path = require('path');
const http = require('http');
const WebSocket = require('ws');

// Parse CLI arguments
const args = process.argv.slice(2);
let host = '127.0.0.1';
let port = 3000;
let durationSec = 10.0;
let mode = 'listen'; // 'listen' (passive) or 'command' (active bounded test)
let turnSpeed = 0.40; // rad/s equivalent or normalized stick
let outputFile = null;

for (let i = 0; i < args.length; i++) {
  if (args[i] === '--host' && args[i + 1]) host = args[++i];
  else if (args[i] === '--port' && args[i + 1]) port = parseInt(args[++i], 10);
  else if (args[i] === '--duration' && args[i + 1]) durationSec = parseFloat(args[++i]);
  else if (args[i] === '--command') mode = 'command';
  else if (args[i] === '--turn-speed' && args[i + 1]) turnSpeed = parseFloat(args[++i]);
  else if (args[i] === '--out' && args[i + 1]) outputFile = args[++i];
}

// Load operator token from environment or local .env
let operatorToken = process.env.ROVER_OPERATOR_TOKEN || '';
if (!operatorToken) {
  const envPath = path.join(__dirname, '..', '.env');
  if (fs.existsSync(envPath)) {
    const lines = fs.readFileSync(envPath, 'utf8').split('\n');
    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.startsWith('ROVER_OPERATOR_TOKEN=')) {
        operatorToken = trimmed.split('=')[1].trim().replace(/['"]/g, '');
        break;
      }
    }
  }
}

if (!outputFile) {
  const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
  const reportsDir = path.join(__dirname, '..', 'reports', 'turn_diagnostics');
  if (!fs.existsSync(reportsDir)) fs.mkdirSync(reportsDir, { recursive: true });
  outputFile = path.join(reportsDir, `turn_diagnostic_${timestamp}.json`);
}

console.log('==============================================================================');
console.log('ROVER 1 SYNCHRONIZED TURNING DIAGNOSTIC RECORDER');
console.log('==============================================================================');
console.log(`Target:       ws://${host}:${port}`);
console.log(`Mode:         ${mode.toUpperCase()}`);
console.log(`Duration:     ${durationSec}s`);
console.log(`Output:       ${outputFile}`);
console.log('==============================================================================\n');

const wsUrl = `ws://${host}:${port}`;
const ws = new WebSocket(wsUrl);

const sessionData = {
  metadata: {
    startTime: new Date().toISOString(),
    host,
    port,
    mode,
    durationSec,
    runningRevision: null,
    firmwareUptimeAtStartMs: null,
    imuInitializedAtStart: null
  },
  events: [],
  summary: {
    firstTriggeringCondition: null,
    firmwareOutputCutTimeMs: null,
    hostDisarmTimeMs: null,
    totalRecords: 0
  }
};

let startTimeMs = null;
let recordingActive = false;
let keepaliveInterval = null;
let stopTimer = null;

function logEvent(category, data) {
  const now = Date.now();
  const relTimeMs = startTimeMs ? (now - startTimeMs) : 0;
  const event = {
    t_rel_ms: relTimeMs,
    wall_iso: new Date(now).toISOString(),
    cat: category,
    data
  };
  sessionData.events.push(event);

  // Live console trace for key state transitions
  if (category === 'status_change') {
    console.log(`[+${(relTimeMs / 1000).toFixed(3)}s] STATUS: ${JSON.stringify(data)}`);
  } else if (category === 'drivetrain_fault') {
    console.log(`[+${(relTimeMs / 1000).toFixed(3)}s] ⚠️ FAULT: ${JSON.stringify(data)}`);
  } else if (category === 'serial_tx' && (data.includes('FUNC_MOTION') || data.includes('disarm'))) {
    // console.log(`[+${(relTimeMs / 1000).toFixed(3)}s] TX: ${data}`);
  }
}

ws.on('open', () => {
  console.log('Connected to Cockpit WebSocket. Authenticating...');
  ws.send(JSON.stringify({ type: 'auth', token: operatorToken }));
});

let prevSpinRecoveryState = null;
let prevSpinFailedExhausted = null;
let prevArmed = null;
let prevFrontSafetyState = null;

ws.on('message', (raw) => {
  try {
    const msg = JSON.parse(raw);
    const now = Date.now();

    if (msg.type === 'auth_result') {
      if (msg.ok) {
        console.log('Authentication successful. Ready.');
        startRecording();
      } else {
        console.error('Authentication failed:', msg.error);
        process.exit(1);
      }
      return;
    }

    if (!recordingActive) return;

    // Ingest all message types
    if (msg.type === 'normal_drive_status') {
      logEvent('drive_status', {
        armed: msg.armed,
        mode: msg.mode,
        reqLinear: msg.reqLinear,
        reqAngular: msg.reqAngular,
        limLinear: msg.limLinear,
        limAngular: msg.limAngular,
        uptimeMs: msg.uptimeMs,
        bootCount: msg.bootCount,
        resetReason: msg.resetReason,
        spinRecoveryCode: msg.spinRecoveryCode,
        spinRecoveryState: msg.spinRecoveryState,
        noMotionBudgetMs: msg.noMotionBudgetMs,
        spinFailedExhausted: msg.spinFailedExhausted,
        chassisRebreakoutConsumed: msg.chassisRebreakoutConsumed,
        cmdAge: msg.cmdAge
      });

      // Detect transitions
      if (msg.spinRecoveryState !== prevSpinRecoveryState || msg.spinFailedExhausted !== prevSpinFailedExhausted) {
        logEvent('status_change', {
          event: 'spin_recovery_transition',
          from: prevSpinRecoveryState,
          to: msg.spinRecoveryState,
          exhausted: msg.spinFailedExhausted,
          noMotionBudgetMs: msg.noMotionBudgetMs
        });
        if (msg.spinFailedExhausted && !sessionData.summary.firstTriggeringCondition) {
          sessionData.summary.firstTriggeringCondition = `spinFailedExhausted latched (${msg.spinRecoveryState}) at noMotionBudget=${msg.noMotionBudgetMs}ms`;
          sessionData.summary.firmwareOutputCutTimeMs = Date.now() - startTimeMs;
        }
        prevSpinRecoveryState = msg.spinRecoveryState;
        prevSpinFailedExhausted = msg.spinFailedExhausted;
      }

      if (msg.armed !== prevArmed) {
        logEvent('status_change', { event: 'armed_transition', from: prevArmed, to: msg.armed });
        if (prevArmed === true && msg.armed === false && !sessionData.summary.hostDisarmTimeMs) {
          sessionData.summary.hostDisarmTimeMs = Date.now() - startTimeMs;
        }
        prevArmed = msg.armed;
      }
    } else if (msg.type === 'pid_telemetry') {
      const telem = msg.telemetry || {};
      logEvent('pid_telemetry', {
        seq: msg.sequence,
        ageMs: msg.ageMs,
        m1: telem.m1 ? { target: telem.m1.targetRadps, meas: telem.m1.measuredRadps, pwm: telem.m1.finalPwm, stiction: telem.m1.stictionState } : null,
        m2: telem.m2 ? { target: telem.m2.targetRadps, meas: telem.m2.measuredRadps, pwm: telem.m2.finalPwm, stiction: telem.m2.stictionState } : null,
        m3: telem.m3 ? { target: telem.m3.targetRadps, meas: telem.m3.measuredRadps, pwm: telem.m3.finalPwm, stiction: telem.m3.stictionState } : null,
        m4: telem.m4 ? { target: telem.m4.targetRadps, meas: telem.m4.measuredRadps, pwm: telem.m4.finalPwm, stiction: telem.m4.stictionState } : null
      });
    } else if (msg.type === 'imu_telemetry' || msg.type === 'imu') {
      logEvent('imu_telemetry', {
        hwInit: msg.hardwareInitialized,
        valid: msg.gyroValid,
        gyroAgeMs: msg.gyroAgeMs,
        gz: msg.gyro ? msg.gyro.z : null,
        orientation: msg.orientation,
        inResetRecovery: msg.inResetRecovery
      });
    } else if (msg.type === 'front_safety_zone') {
      const zone = msg.zone || msg;
      logEvent('front_safety_zone', {
        state: zone.state,
        minMm: zone.minMm,
        action: zone.action,
        intervening: zone.intervening
      });
      if (zone.state !== prevFrontSafetyState) {
        logEvent('status_change', { event: 'front_zone_transition', from: prevFrontSafetyState, to: zone.state, minMm: zone.minMm });
        prevFrontSafetyState = zone.state;
      }
    } else if (msg.type === 'raw_serial_out') {
      logEvent('serial_tx', msg.data);
    } else if (msg.type === 'manual_drive_watchdog') {
      logEvent('drivetrain_fault', { type: 'watchdog_timeout', reason: msg.reason });
    } else if (msg.type === 'hint') {
      logEvent('hint', msg.text);
    }
  } catch (e) {
    console.error('Error handling WebSocket message:', e.message);
  }
});

function startRecording() {
  startTimeMs = Date.now();
  recordingActive = true;
  console.log(`>>> RECORDING STARTED (${durationSec} seconds) <<<\n`);

  if (mode === 'command') {
    console.log(`Arming rover via POST /api/drive/arm and executing bounded turn test (turnSpeed=${turnSpeed})...`);

    const req = http.request({
      hostname: host,
      port: port,
      path: '/api/drive/arm',
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-rover-operator-token': operatorToken
      }
    }, (res) => {
      let data = '';
      res.on('data', chunk => data += chunk);
      res.on('end', () => {
        console.log(`Arm response: ${res.statusCode} ${data}`);
        // Send joystick frames at 50 ms (20 Hz)
        const stickX = turnSpeed; // Positive X = turn command
        const stickY = 0.0;
        keepaliveInterval = setInterval(() => {
          ws.send(JSON.stringify({ type: 'joystick', x: stickX, y: stickY, deadman: true, keepalive: true }));
        }, 50);
      });
    });
    req.on('error', (err) => {
      console.error('Arm request error:', err.message);
    });
    req.end(JSON.stringify({}));
  } else {
    console.log('Listening passively. Operator may hold gamepad or web controls to drive.');
  }

  stopTimer = setTimeout(() => {
    finishRecording('Duration elapsed');
  }, durationSec * 1000);
}

function finishRecording(reason) {
  if (!recordingActive) return;
  recordingActive = false;
  clearInterval(keepaliveInterval);
  clearTimeout(stopTimer);

  console.log(`\n>>> RECORDING FINISHED (${reason}) <<<`);

  // Ensure rover is stopped and disarmed
  try {
    ws.send(JSON.stringify({ type: 'joystick', x: 0, y: 0, deadman: false }));
    ws.send(JSON.stringify({ type: 'disarm_normal_drive' }));
  } catch (e) {}

  try {
    const disarmReq = http.request({
      hostname: host,
      port: port,
      path: '/api/drive/disarm',
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-rover-operator-token': operatorToken
      }
    });
    disarmReq.on('error', () => {});
    disarmReq.end(JSON.stringify({}));
  } catch (e) {}

  sessionData.metadata.endTime = new Date().toISOString();
  sessionData.summary.totalRecords = sessionData.events.length;

  fs.writeFileSync(outputFile, JSON.stringify(sessionData, null, 2), 'utf8');
  console.log(`Saved ${sessionData.events.length} records to: ${outputFile}\n`);

  // Print Summary Analysis Table
  printSummaryAnalysis();

  setTimeout(() => {
    ws.close();
    process.exit(0);
  }, 500);
}

function printSummaryAnalysis() {
  console.log('==============================================================================');
  console.log('CAUSAL TIMELINE & SUMMARY ANALYSIS');
  console.log('==============================================================================');
  console.log(`First Triggering Condition: ${sessionData.summary.firstTriggeringCondition || 'None detected'}`);
  console.log(`Firmware Output Cut Time:   ${sessionData.summary.firmwareOutputCutTimeMs !== null ? `+${(sessionData.summary.firmwareOutputCutTimeMs / 1000).toFixed(3)}s` : 'N/A'}`);
  console.log(`Host Disarm Time:           ${sessionData.summary.hostDisarmTimeMs !== null ? `+${(sessionData.summary.hostDisarmTimeMs / 1000).toFixed(3)}s` : 'N/A'}`);
  console.log('------------------------------------------------------------------------------');
  console.log('Key State Events:');
  const keyEvents = sessionData.events.filter(e => e.cat === 'status_change' || e.cat === 'drivetrain_fault');
  if (keyEvents.length === 0) {
    console.log('  (No state transition events recorded)');
  } else {
    for (const ev of keyEvents) {
      console.log(`  +${(ev.t_rel_ms / 1000).toFixed(3)}s [${ev.cat}] ${JSON.stringify(ev.data)}`);
    }
  }
  console.log('==============================================================================\n');
}

// Trap SIGINT
process.on('SIGINT', () => {
  console.log('\nReceived SIGINT. Halting gracefully...');
  finishRecording('Operator abort (SIGINT)');
});

