// ==============================================================================
// test_localization_safety.js — Focused Automated Tests for Localization Gating
// Covers: Startup uninitialized, pose estimate, lost localization disarm, recovery
// ==============================================================================

const assert = require('assert');
const http = require('http');

const VALID_TOKEN = 'a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890'; // gitleaks:allow

process.env.PORT = '3820';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3830';
process.env.ROVER_CMD_VEL_TOKEN = VALID_TOKEN;
process.env.ROVER_OPERATOR_TOKEN = 'test_operator_token_12345678901234567890123456789012';

const serverModule = require('./server.js');
const publicApp = serverModule.app;
const publicServer = serverModule.server;
const internalApp = serverModule.internalCmdApp;
const internalServer = serverModule.internalCmdServer;

const PUBLIC_PORT = 3820;
const INTERNAL_PORT = 3830;
const OPERATOR_TOKEN = 'test_operator_token_12345678901234567890123456789012';

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

async function runTests() {
  console.log('--- STARTING LOCALIZATION SAFETY AUTOMATED TEST SUITE ---');

  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));
  await new Promise(r => internalServer.listen(INTERNAL_PORT, '127.0.0.1', r));

  try {
    // --------------------------------------------------------------------------
    // TEST 1: Startup / Uninitialized State
    // --------------------------------------------------------------------------
    console.log('\n[Test 1] Verifying Startup Uninitialized Behavior...');
    {
      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.statusCode, 200);
      assert.strictEqual(statusRes.json.localization.localized, false, 'Localization must start as NOT localized');
      assert.strictEqual(statusRes.json.localization.state, 'NOT_LOCALIZED');
      console.log('  ✓ Initial state confirmed: NOT_LOCALIZED');

      // Attempt to enable autonomy while NOT localized -> MUST reject with HTTP 409
      const enableRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/enable',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, {});
      assert.strictEqual(enableRes.statusCode, 409, 'Enabling autonomy while NOT localized must return HTTP 409');
      assert(enableRes.json.error.includes('NOT LOCALIZED'), 'Error message must specify vehicle is NOT LOCALIZED');
      console.log('  ✓ Autonomy enable blocked while unlocalized (HTTP 409)');

      // Attempt to arm rover with autonomy flag while NOT localized -> MUST reject with HTTP 409
      const armRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, { autonomy: true });
      assert.strictEqual(armRes.statusCode, 409, 'Arming for autonomy while NOT localized must return HTTP 409');
      console.log('  ✓ Arming for autonomy blocked while unlocalized (HTTP 409)');

      // Attempt to send /cmd_vel while NOT localized -> MUST reject with HTTP 403
      const cmdRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': VALID_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
      assert.strictEqual(cmdRes.statusCode, 403, 'Forward commands must be rejected with 403');
      console.log('  ✓ Motor command rejected while unlocalized (HTTP 403)');
    }

    // --------------------------------------------------------------------------
    // TEST 2: Pose Initialization (Foxglove 2D Pose -> AMCL convergence)
    // --------------------------------------------------------------------------
    console.log('\n[Test 2] Simulating Operator 2D Pose Initialization in Foxglove...');
    {
      // Feed simulated converged AMCL pose
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      const locRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/localization/status',
        method: 'GET'
      });
      assert.strictEqual(locRes.statusCode, 200);
      assert.strictEqual(locRes.json.localized, true, 'Vehicle must report localized: true');
      assert.strictEqual(locRes.json.state, 'LOCALIZED');
      assert.strictEqual(locRes.json.x, 1.166);
      console.log('  ✓ Localization state recognized: LOCALIZED (pose tracking nominal)');

      // Now enable autonomy -> MUST succeed (HTTP 200)
      const enableRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/enable',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, {});
      assert.strictEqual(enableRes.statusCode, 200, 'Enabling autonomy must succeed when LOCALIZED');
      assert.strictEqual(serverModule.autonomyState.state, 'WAITING_FOR_ZERO');
      console.log('  ✓ Autonomy enabled (entered WAITING_FOR_ZERO)');

      // Complete zero handshake (3 zero packets)
      for (let i = 0; i < 3; i++) {
        await httpRequest({
          hostname: '127.0.0.1',
          port: INTERNAL_PORT,
          path: '/api/cmd_vel',
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'X-Rover-Bridge-Token': VALID_TOKEN
          }
        }, { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
      }
      assert.strictEqual(serverModule.autonomyState.state, 'READY_DISARMED');
      console.log('  ✓ Zero velocity handshake complete (READY_DISARMED)');
    }

    // --------------------------------------------------------------------------
    // TEST 3: Fail-Safe Stop & Disarm on Lost Localization During Navigation
    // --------------------------------------------------------------------------
    console.log('\n[Test 3] Simulating Lost Localization During Active Navigation...');
    {
      // Mock rover armed
      serverModule.injectNormalDriveStatus({ armed: true, mode: 3 });
      serverModule.autonomyState.state = 'ACTIVE';
      serverModule.autonomyState.active = true;
      serverModule.autonomyState.limitedLinear = 0.12;

      console.log('  Active autonomous motion running (speed=0.12 m/s, armed=true, state=ACTIVE)');

      // Now simulate localization loss (e.g. AMCL divergence or stale data)
      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        details: 'Pose stale (2500ms > 2000ms)'
      });

      // Verify fail-safe trigger
      assert.strictEqual(serverModule.autonomyState.state, 'FAULT', 'Autonomy state must transition to FAULT');
      assert.strictEqual(serverModule.autonomyState.enabled, false, 'Autonomy must be disabled');
      assert.strictEqual(serverModule.autonomyState.active, false, 'Autonomy must be inactive');
      assert(serverModule.autonomyState.lastRejectionReason.includes('Localization lost'), 'Rejection reason must mention localization lost');

      // Verify rover is disarmed
      const driveStatus = serverModule.getLatestNormalDriveStatus();
      assert.strictEqual(driveStatus.armed, false, 'Rover MUST be disarmed upon localization loss');
      console.log('  ✓ Fail-safe trigger verified: Motors zeroed, autonomy FAULT, rover disarmed');

      // Subsequent /cmd_vel commands must be rejected
      const cmdRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': VALID_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
      assert.strictEqual(cmdRes.statusCode, 403);
      console.log('  ✓ Motion command rejected following localization fault');
    }

    // --------------------------------------------------------------------------
    // TEST 4: Pose Recovery After Fault
    // --------------------------------------------------------------------------
    console.log('\n[Test 4] Verifying Post-Fault Pose Recovery & Safe Stationary Latch...');
    {
      // Re-supply valid localization
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        details: 'Tracking recovered',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      const locRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/localization/status',
        method: 'GET'
      });
      assert.strictEqual(locRes.json.localized, true);

      // Verify rover did NOT spontaneously re-arm or resume motion
      const driveStatus = serverModule.getLatestNormalDriveStatus();
      assert.strictEqual(driveStatus.armed, false, 'Rover must remain disarmed after recovery');
      assert.strictEqual(serverModule.autonomyState.enabled, false, 'Autonomy must remain disabled');
      assert.strictEqual(serverModule.autonomyState.state, 'FAULT', 'Autonomy must remain in FAULT until explicit re-enable');
      console.log('  ✓ Rover remained safely disarmed and stopped upon localization recovery');
    }

    console.log('\n=======================================================');
    console.log('✓ ALL 4 LOCALIZATION SAFETY TESTS PASSED NOMINALLY');
    console.log('=======================================================\n');
    process.exit(0);

  } catch (err) {
    console.error('\n❌ TEST FAILURE:', err);
    process.exit(1);
  } finally {
    try { publicServer.close(); } catch(e) {}
    try { internalServer.close(); } catch(e) {}
  }
}

runTests();
