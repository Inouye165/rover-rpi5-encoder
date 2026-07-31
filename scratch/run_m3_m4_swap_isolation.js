// ==============================================================================
// scratch/run_m3_m4_swap_isolation.js - M3/M4 Motor-Cable-Swap Isolation Test
// ==============================================================================
// Usage:
//   CONFIRM_RAISED="All wheels are off the ground and clear to rotate." \
//   node scratch/run_m3_m4_swap_isolation.js
// ==============================================================================

const http = require('http');

const REQUIRED_CONFIRMATION = "All wheels are off the ground and clear to rotate.";
const userConfirmation = process.env.CONFIRM_RAISED || "";

if (userConfirmation.trim() !== REQUIRED_CONFIRMATION) {
  console.error("\n[SAFETY ERROR] Required pre-test confirmation missing or incorrect!");
  console.error(`Expected exact string: "${REQUIRED_CONFIRMATION}"`);
  console.error("Set environment variable CONFIRM_RAISED to proceed.\n");
  process.exit(1);
}

const HOST = process.env.ROVER_HOST || '127.0.0.1';
const PORT = parseInt(process.env.PORT || '3000');
const OPERATOR_TOKEN = process.env.ROVER_OPERATOR_TOKEN || '';

function httpRequest(options, postData) {
  return new Promise((resolve, reject) => {
    const req = http.request(options, (res) => {
      let data = '';
      res.on('data', chunk => { data += chunk; });
      res.on('end', () => {
        let json = null;
        try { json = JSON.parse(data); } catch (e) {}
        resolve({ statusCode: res.statusCode, headers: res.headers, data, json });
      });
    });
    req.on('error', reject);
    if (postData !== undefined) {
      req.write(typeof postData === 'string' ? postData : JSON.stringify(postData));
    }
    req.end();
  });
}

function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function performCleanup() {
  console.log("\n[Safety Cleanup] Initiating post-test safety cleanup...");
  try {
    // 1. Force stop all motors
    await httpRequest({
      hostname: HOST, port: PORT, path: '/api/stop', method: 'GET'
    });
    // 2. Exit maintenance mode
    await httpRequest({
      hostname: HOST, port: PORT, path: '/api/maintenance/exit', method: 'POST'
    });
    // 3. Disable autonomy
    const headers = { 'Content-Type': 'application/json' };
    if (OPERATOR_TOKEN) headers['X-Rover-Operator-Token'] = OPERATOR_TOKEN;
    await httpRequest({
      hostname: HOST, port: PORT, path: '/api/autonomy/disable', method: 'POST', headers
    });
    console.log("[Safety Cleanup] Motors zeroed, maintenance exited, autonomy disabled.");
  } catch (err) {
    console.error("[Safety Cleanup Warning] Cleanup error:", err.message);
  }
}

// Intercept Ctrl+C (SIGINT) and SIGTERM for graceful emergency stop
process.on('SIGINT', async () => {
  console.error("\n[ABORT TRIGGERED] Interrupt signal received (SIGINT). Emergency stopping!");
  await performCleanup();
  process.exit(1);
});

process.on('SIGTERM', async () => {
  console.error("\n[ABORT TRIGGERED] Termination signal received (SIGTERM). Emergency stopping!");
  await performCleanup();
  process.exit(1);
});

async function runM3M4SwapIsolationTest() {
  console.log("==========================================================");
  console.log("M3/M4 CABLE-SWAP HARDWARE ISOLATION DIAGNOSTIC TEST");
  console.log("==========================================================");
  console.log(`Confirmation verified: "${userConfirmation}"`);
  console.log(`Target Cockpit API: http://${HOST}:${PORT}`);
  console.log("Physical Wiring Setup:");
  console.log("  - Driver Channel M4 (Index 3) -> Physical Left-Rear Motor (Evaluates Encoder M3)");
  console.log("  - Driver Channel M3 (Index 2) -> Physical Right-Rear Motor (Evaluates Encoder M4)");
  console.log("----------------------------------------------------------\n");

  // --------------------------------------------------------------------------
  // Pre-test Verification Interlocks
  // --------------------------------------------------------------------------
  console.log("[Pre-test Guard] Verifying initial rover state...");
  const driveStatusRes = await httpRequest({
    hostname: HOST, port: PORT, path: '/api/drive/status', method: 'GET'
  });
  if (driveStatusRes.statusCode !== 200 || !driveStatusRes.json || !driveStatusRes.json.ok) {
    throw new Error(`Failed to fetch drive status (HTTP ${driveStatusRes.statusCode})`);
  }
  const driveStatus = driveStatusRes.json.status || {};
  if (driveStatus.armed) {
    throw new Error("Pre-test check failed: Rover is currently ARMED. Disarm rover before running test.");
  }
  if (driveStatus.reqLinear !== 0 || driveStatus.reqAngular !== 0) {
    throw new Error("Pre-test check failed: Non-zero requested motion target active.");
  }

  const encodersRes = await httpRequest({
    hostname: HOST, port: PORT, path: '/api/encoders', method: 'GET'
  });
  if (encodersRes.statusCode !== 200 || !encodersRes.json || !encodersRes.json.ok) {
    throw new Error(`Failed to fetch encoder telemetry (HTTP ${encodersRes.statusCode})`);
  }
  if (!encodersRes.json.serialConnected) {
    throw new Error("Pre-test check failed: ESP32 serial port is not connected.");
  }
  if (encodersRes.json.lastPacketAgeMs === null || encodersRes.json.lastPacketAgeMs > 1000) {
    throw new Error(`Pre-test check failed: Telemetry stale (${encodersRes.json.lastPacketAgeMs}ms age).`);
  }

  console.log("✓ Pre-test Guard Passed: Disarmed, autonomy idle, serial connected, telemetry fresh.\n");

  const testConfig = [
    {
      testLabel: 'Test A',
      driverChannel: 'M4',
      motorIndex: 3,
      physicalMotor: 'Left Rear',
      encoderChannel: 'm3',
      requestedPwm: 60,
      direction: 'forward',
      durationSec: 0.5
    },
    {
      testLabel: 'Test B',
      driverChannel: 'M3',
      motorIndex: 2,
      physicalMotor: 'Right Rear',
      encoderChannel: 'm4',
      requestedPwm: 60,
      direction: 'forward',
      durationSec: 0.5
    }
  ];

  const testResults = [];

  try {
    for (let idx = 0; idx < testConfig.length; idx++) {
      const tc = testConfig[idx];
      if (idx > 0) {
        console.log("\nPausing 2.0 seconds between tests...");
        await delay(2000);
      }

      console.log(`Executing ${tc.testLabel}: Driver ${tc.driverChannel} (Index ${tc.motorIndex}) -> Physical ${tc.physicalMotor} | FWD 60 PWM ...`);

      const payload = {
        safetyAck: true,
        motorIndex: tc.motorIndex,
        direction: tc.direction,
        output: tc.requestedPwm,
        durationSec: tc.durationSec
      };

      const res = await httpRequest({
        hostname: HOST,
        port: PORT,
        path: '/api/maintenance/run_test',
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      }, payload);

      if (res.statusCode !== 200 || !res.json || !res.json.ok) {
        const errorMsg = res.json ? (res.json.error || 'Failed') : `HTTP ${res.statusCode}`;
        console.error(`  -> ${tc.testLabel} REJECTED/FAILED:`, errorMsg);
        testResults.push({
          ...tc,
          reportedPwm: 0,
          startCount: 0,
          endCount: 0,
          encoderDelta: 0,
          movementDetected: false,
          telemetryAge: 'N/A',
          stopReason: `FAILED: ${errorMsg}`
        });
        throw new Error(`${tc.testLabel} failed: ${errorMsg}`);
      }

      const tr = res.json.test_result || {};
      let evalStart = 0;
      let evalEnd = 0;
      let evalDelta = 0;

      if (tc.encoderChannel === 'm3') {
        if (tr.selected_motor === 'm3') {
          evalStart = tr.starting_encoder_count || 0;
          evalEnd = tr.ending_encoder_count || 0;
          evalDelta = tr.encoder_delta || 0;
        } else {
          evalDelta = tr.unselected_motor_deltas ? (tr.unselected_motor_deltas.m3 || 0) : 0;
        }
      } else if (tc.encoderChannel === 'm4') {
        if (tr.selected_motor === 'm4') {
          evalStart = tr.starting_encoder_count || 0;
          evalEnd = tr.ending_encoder_count || 0;
          evalDelta = tr.encoder_delta || 0;
        } else {
          evalDelta = tr.unselected_motor_deltas ? (tr.unselected_motor_deltas.m4 || 0) : 0;
        }
      }

      const movementDetected = Math.abs(evalDelta) >= 5;

      console.log(`  -> ${tc.testLabel} SUCCESS: evaluated encoder ${tc.encoderChannel.toUpperCase()} delta = ${evalDelta} (movementDetected = ${movementDetected})`);

      testResults.push({
        ...tc,
        reportedPwm: tr.commanded_pwm || tc.requestedPwm,
        startCount: evalStart,
        endCount: evalEnd,
        encoderDelta: evalDelta,
        movementDetected,
        telemetryAge: `${tr.elapsed_test_time_sec || 0.5}s`,
        stopReason: 'Completed (500ms pulse)'
      });
    }

    console.log("\n==========================================================");
    console.log("CABLE-SWAP ISOLATION TEST RESULTS SUMMARY");
    console.log("==========================================================\n");

    console.log("| Test | Driver Channel | Motor Index | Physical Motor | Evaluated Encoder | Req PWM | Actual PWM | Enc Delta | Movement Detected | Stop Reason |");
    console.log("|---|---|---|---|---|---|---|---|---|---|");
    for (const r of testResults) {
      console.log(`| ${r.testLabel} | ${r.driverChannel} | ${r.motorIndex} | ${r.physicalMotor} | ${r.encoderChannel.toUpperCase()} | ${r.requestedPwm} | ${r.reportedPwm} | ${r.encoderDelta} | ${r.movementDetected ? 'TRUE' : 'FALSE'} | ${r.stopReason} |`);
    }

    console.log("\nDiagnostic Interpretation:");
    const testA = testResults.find(r => r.testLabel === 'Test A');
    const testB = testResults.find(r => r.testLabel === 'Test B');

    if (testA && testB) {
      if (!testA.movementDetected && testB.movementDetected) {
        console.log("  -> RESULT: Physical Left Rear motor (driven by M4) STILL FAILED in Forward.");
        console.log("             The forward failure FOLLOWED DRIVER CHANNEL M4.");
        console.log("             SUSPECT: Driver Channel M4 hardware (failed high-side MOSFET on GPIO 14 / IN1 leg) or board trace defect.");
      } else if (testA.movementDetected && !testB.movementDetected) {
        console.log("  -> RESULT: Physical Right Rear motor (driven by M3) NOW FAILED in Forward.");
        console.log("             The forward failure FOLLOWED THE PHYSICAL MOTOR (Right Rear).");
        console.log("             SUSPECT: Physical motor, internal motor brushes, gearbox drag, or motor wiring harness.");
      } else {
        console.log(`  -> RESULT: Test A movement=${testA.movementDetected}, Test B movement=${testB.movementDetected}. See table details.`);
      }
    }

  } finally {
    await performCleanup();
  }
}

runM3M4SwapIsolationTest().catch(async (err) => {
  console.error("\n[ISOLATION TEST ERROR]", err.message);
  await performCleanup();
  process.exit(1);
});
