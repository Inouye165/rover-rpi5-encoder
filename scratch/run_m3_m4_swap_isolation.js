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

// Resilient cleanup function
// isEmergency: if true (triggered by error, SIGINT, SIGTERM, or timeout), calls /api/stop.
// If false (routine successful completion), DOES NOT call /api/stop to avoid latching E-Stop.
async function performCleanup(isEmergency = false) {
  console.log(`\n[Safety Cleanup] Initiating resilient safety cleanup (isEmergency=${isEmergency})...`);
  const headers = { 'Content-Type': 'application/json' };
  if (OPERATOR_TOKEN) headers['X-Rover-Operator-Token'] = OPERATOR_TOKEN;

  // 1. Force emergency stop ONLY on emergency/error triggers
  if (isEmergency) {
    try {
      await httpRequest({ hostname: HOST, port: PORT, path: '/api/stop', method: 'GET' });
      console.log("  ✓ Emergency stop requested (/api/stop)");
    } catch (err) {
      console.error("  x Emergency stop call failed:", err.message);
    }
  }

  // 2. Exit maintenance mode
  try {
    await httpRequest({ hostname: HOST, port: PORT, path: '/api/maintenance/exit', method: 'POST', headers });
    console.log("  ✓ Maintenance mode exited (/api/maintenance/exit)");
  } catch (err) {
    console.error("  x Exit maintenance failed:", err.message);
  }

  // 3. Disarm normal drive
  try {
    await httpRequest({ hostname: HOST, port: PORT, path: '/api/drive/disarm', method: 'POST', headers });
    console.log("  ✓ Normal drive disarmed (/api/drive/disarm)");
  } catch (err) {
    console.error("  x Disarm drive failed:", err.message);
  }

  // 4. Disable autonomy
  try {
    await httpRequest({ hostname: HOST, port: PORT, path: '/api/autonomy/disable', method: 'POST', headers });
    console.log("  ✓ Autonomy disabled (/api/autonomy/disable)");
  } catch (err) {
    console.error("  x Disable autonomy failed:", err.message);
  }

  // 5. Final State Verification
  try {
    console.log("[Safety Cleanup] Verifying final rover safety state...");
    const driveRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/drive/status', method: 'GET' });
    const driveStatus = (driveRes.json && driveRes.json.status) ? driveRes.json.status : {};

    const maintRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/maintenance/status', method: 'GET' });
    const maintStatus = (maintRes.json && maintRes.json.status) ? maintRes.json.status : {};

    const autoRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/autonomy/status', method: 'GET' });
    const autoStatus = (autoRes.json) ? autoRes.json : {};

    const armed = driveStatus.armed === true;
    const mode = driveStatus.mode !== undefined ? driveStatus.mode : 0;
    const autonomyState = autoStatus.state || 'DISABLED';
    const maintActive = maintStatus.active === true;
    const testPwm = maintStatus.testPwm || 0;
    const actualPwm = maintStatus.actualPwm || 0;
    const reqLin = driveStatus.reqLinear || 0;
    const reqAng = driveStatus.reqAngular || 0;
    const limLin = driveStatus.limLinear || 0;
    const limAng = driveStatus.limAngular || 0;
    const cmdSource = driveStatus.cmdSource || 'NONE';

    const modeOk = isEmergency ? true : (mode !== 3);
    const checksPassed = !armed && 
                         modeOk &&
                         (autonomyState === 'DISABLED' || autonomyState === 'OFF') && 
                         !maintActive && 
                         testPwm === 0 && 
                         actualPwm === 0 && 
                         reqLin === 0 && 
                         reqAng === 0 && 
                         limLin === 0 && 
                         limAng === 0 && 
                         cmdSource === 'NONE';

    if (checksPassed) {
      console.log("  ✓ Final Verification PASSED: Rover disarmed, autonomy disabled, maintenance inactive, motion zeroed.");
    } else {
      console.error("  x Final Verification WARNING: Unexpected safety state:", {
        armed, mode, autonomyState, maintActive, testPwm, actualPwm, reqLin, reqAng, limLin, limAng, cmdSource
      });
    }
  } catch (err) {
    console.error("  x Final verification check failed:", err.message);
  }
}

// Intercept Ctrl+C (SIGINT) and SIGTERM for emergency cleanup
process.on('SIGINT', async () => {
  console.error("\n[ABORT TRIGGERED] Interrupt signal received (SIGINT). Emergency stopping!");
  await performCleanup(true);
  process.exit(1);
});

process.on('SIGTERM', async () => {
  console.error("\n[ABORT TRIGGERED] Termination signal received (SIGTERM). Emergency stopping!");
  await performCleanup(true);
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
  // Pre-test Verification Interlocks & Emergency Stop Latch Handling
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

  const encodersResPre = await httpRequest({
    hostname: HOST, port: PORT, path: '/api/encoders', method: 'GET'
  });
  if (encodersResPre.statusCode !== 200 || !encodersResPre.json || !encodersResPre.json.ok) {
    throw new Error(`Failed to fetch encoder telemetry (HTTP ${encodersResPre.statusCode})`);
  }
  if (!encodersResPre.json.serialConnected) {
    throw new Error("Pre-test check failed: ESP32 serial port is not connected.");
  }
  if (encodersResPre.json.lastPacketAgeMs === null || encodersResPre.json.lastPacketAgeMs > 1000) {
    throw new Error(`Pre-test check failed: Telemetry stale (${encodersResPre.json.lastPacketAgeMs}ms age).`);
  }

  // Detect Latched Emergency Stop (mode === 3)
  if (driveStatus.mode === 3) {
    console.log("⚠️ Latched Emergency Stop detected on ESP32 (mode=3). Attempting safe reset...");
    const autoRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/autonomy/status', method: 'GET' });
    const maintRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/maintenance/status', method: 'GET' });
    const autoStatus = autoRes.json || {};
    const maintStatus = (maintRes.json && maintRes.json.status) ? maintRes.json.status : {};

    if (autoStatus.state !== 'DISABLED' && autoStatus.state !== 'OFF') {
      throw new Error("Cannot clear E-Stop: Autonomy is not disabled.");
    }
    if (maintStatus.active) {
      throw new Error("Cannot clear E-Stop: Maintenance is currently active.");
    }

    console.log("Sending GET /api/faults/clear to reset ESP32 emergency stop latch...");
    const clearRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/faults/clear', method: 'GET' });
    if (clearRes.statusCode !== 200 || !clearRes.json || !clearRes.json.ok) {
      throw new Error(`Failed to clear E-Stop via /api/faults/clear: ${clearRes.data}`);
    }

    await delay(300);

    const recheckRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/drive/status', method: 'GET' });
    const recheckStatus = (recheckRes.json && recheckRes.json.status) ? recheckRes.json.status : {};
    if (recheckStatus.mode === 3) {
      throw new Error("Pre-test check failed: Could not clear latched Emergency Stop (mode remains 3).");
    }
    console.log("✓ ESP32 Emergency Stop latch successfully cleared (mode is now 0/LOCKED).");
  }

  console.log("✓ Pre-test Guard Passed: Disarmed, autonomy idle, serial connected, telemetry fresh, E-Stop clear.\n");

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
  let testErrorEncountered = false;

  try {
    for (let idx = 0; idx < testConfig.length; idx++) {
      const tc = testConfig[idx];
      if (idx > 0) {
        console.log("\nPausing 2.0 seconds between tests...");
        await delay(2000);
      }

      console.log(`Executing ${tc.testLabel}: Driver ${tc.driverChannel} (Index ${tc.motorIndex}) -> Physical ${tc.physicalMotor} | FWD 60 PWM ...`);

      // 1. Fetch fresh pre-pulse encoder snapshot directly from /api/encoders
      const preEncRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/encoders', method: 'GET' });
      if (preEncRes.statusCode !== 200 || !preEncRes.json || !preEncRes.json.ok) {
        throw new Error(`${tc.testLabel} pre-pulse encoder snapshot failed (HTTP ${preEncRes.statusCode})`);
      }
      if (preEncRes.json.lastPacketAgeMs === null || preEncRes.json.lastPacketAgeMs > 1000) {
        throw new Error(`${tc.testLabel} pre-pulse telemetry stale (${preEncRes.json.lastPacketAgeMs}ms age)`);
      }
      const preEncoders = preEncRes.json.encoders || {};
      const preAge = preEncRes.json.lastPacketAgeMs;

      // 2. Execute maintenance test pulse (500 ms max)
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
        console.error(`  -> ${tc.testLabel} COMMAND FAILED:`, errorMsg);
        testResults.push({
          ...tc,
          commandResult: 'COMMAND FAILED',
          reportedPwm: 0,
          startCount: preEncoders[tc.encoderChannel] || 0,
          endCount: preEncoders[tc.encoderChannel] || 0,
          signedDelta: 0,
          absDelta: 0,
          movementStatus: 'NO MOVEMENT DETECTED',
          telemetryAge: `${preAge}ms`,
          stopReason: `FAILED: ${errorMsg}`
        });
        throw new Error(`${tc.testLabel} command execution failed: ${errorMsg}`);
      }

      // 3. Fetch fresh post-pulse encoder snapshot directly from /api/encoders
      await delay(100);
      const postEncRes = await httpRequest({ hostname: HOST, port: PORT, path: '/api/encoders', method: 'GET' });
      if (postEncRes.statusCode !== 200 || !postEncRes.json || !postEncRes.json.ok) {
        throw new Error(`${tc.testLabel} post-pulse encoder snapshot failed (HTTP ${postEncRes.statusCode})`);
      }
      if (postEncRes.json.lastPacketAgeMs === null || postEncRes.json.lastPacketAgeMs > 1000) {
        throw new Error(`${tc.testLabel} post-pulse telemetry stale (${postEncRes.json.lastPacketAgeMs}ms age)`);
      }
      const postEncoders = postEncRes.json.encoders || {};
      const postAge = postEncRes.json.lastPacketAgeMs;

      // 4. Calculate crossed encoder delta directly from before and after samples
      const startCount = preEncoders[tc.encoderChannel] !== undefined ? preEncoders[tc.encoderChannel] : 0;
      const endCount = postEncoders[tc.encoderChannel] !== undefined ? postEncoders[tc.encoderChannel] : 0;
      const signedDelta = endCount - startCount;
      const absDelta = Math.abs(signedDelta);
      const movementStatus = absDelta >= 5 ? 'MOVEMENT DETECTED' : 'NO MOVEMENT DETECTED';

      console.log(`  -> ${tc.testLabel} COMMAND COMPLETED: evaluated encoder ${tc.encoderChannel.toUpperCase()} (${startCount} -> ${endCount}, delta=${signedDelta}) | ${movementStatus}`);

      testResults.push({
        ...tc,
        commandResult: 'COMMAND COMPLETED',
        reportedPwm: tc.requestedPwm,
        startCount,
        endCount,
        signedDelta,
        absDelta,
        movementStatus,
        telemetryAge: `pre:${preAge}ms/post:${postAge}ms`,
        stopReason: 'Completed (500ms pulse)'
      });
    }

    console.log("\n==========================================================");
    console.log("CABLE-SWAP ISOLATION TEST RESULTS SUMMARY");
    console.log("==========================================================\n");

    console.log("| Test | Driver Channel | Motor Index | Physical Motor | Evaluated Encoder | Req PWM | Start Ticks | End Ticks | Signed Delta | Physical Result | Stop Reason |");
    console.log("|---|---|---|---|---|---|---|---|---|---|---|");
    for (const r of testResults) {
      console.log(`| ${r.testLabel} | ${r.driverChannel} | ${r.motorIndex} | ${r.physicalMotor} | ${r.encoderChannel.toUpperCase()} | ${r.requestedPwm} | ${r.startCount} | ${r.endCount} | ${r.signedDelta} | ${r.movementStatus} | ${r.stopReason} |`);
    }

    console.log("\nDiagnostic Interpretation:");
    const testA = testResults.find(r => r.testLabel === 'Test A');
    const testB = testResults.find(r => r.testLabel === 'Test B');

    if (testA && testB) {
      if (testA.movementStatus === 'NO MOVEMENT DETECTED' && testB.movementStatus === 'MOVEMENT DETECTED') {
        console.log("  -> RESULT: Physical Left Rear motor (driven by M4) STILL FAILED in Forward.");
        console.log("             The forward failure FOLLOWED DRIVER CHANNEL M4.");
        console.log("             SUSPECT: Driver Channel M4 hardware (failed high-side MOSFET on GPIO 14 / IN1 leg) or board trace defect.");
      } else if (testA.movementStatus === 'MOVEMENT DETECTED' && testB.movementStatus === 'NO MOVEMENT DETECTED') {
        console.log("  -> RESULT: Physical Right Rear motor (driven by M3) NOW FAILED in Forward.");
        console.log("             The forward failure FOLLOWED THE PHYSICAL MOTOR (Right Rear).");
        console.log("             SUSPECT: Physical motor, internal motor brushes, gearbox drag, or motor wiring harness.");
      } else {
        console.log(`  -> RESULT: Test A=${testA.movementStatus}, Test B=${testB.movementStatus}. See summary table.`);
      }
    }

  } catch (err) {
    testErrorEncountered = true;
    throw err;
  } finally {
    await performCleanup(testErrorEncountered);
  }
}

runM3M4SwapIsolationTest().catch(async (err) => {
  console.error("\n[ISOLATION TEST ERROR]", err.message);
  await performCleanup(true);
  process.exit(1);
});
