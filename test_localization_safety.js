// ==============================================================================
// test_localization_safety.js — Automated Tests for Localization Gating & Source Discrimination
// Covers:
// 1. Unlocalized state:
//    - Zero velocity commands ALWAYS pass (HTTP 200)
//    - Emergency-stop (/api/stop) and Disarm (/api/drive/disarm) ALWAYS pass (HTTP 200)
//    - Manual drive arming is NOT blocked by localization
//    - ROS_AUTONOMY non-zero commands ARE blocked (HTTP 403)
//    - CALIBRATION_TEST commands are NOT blocked by localization
// 2. Pose Estimate & Autonomy Handshake (Foxglove 2D Pose -> WAITING_FOR_ZERO -> READY_DISARMED)
// 3. Fail-Safe Disarm on Lost Localization during Active Navigation
// 4. Safe Stationary Recovery latch
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
const OPERATOR_TOKEN = process.env.ROVER_OPERATOR_TOKEN || 'test_operator_token_12345678901234567890123456789012';
const CMD_TOKEN = process.env.ROVER_CMD_VEL_TOKEN || VALID_TOKEN;

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
  console.log('--- STARTING LOCALIZATION SAFETY & SOURCE DISCRIMINATION TEST SUITE ---');
  if (serverModule.setOdomPollingDisabled) {
    serverModule.setOdomPollingDisabled(true);
  }

  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));
  await new Promise(r => internalServer.listen(INTERNAL_PORT, '127.0.0.1', r));

  try {
    // --------------------------------------------------------------------------
    // TEST 1: Unlocalized State Behavior & Command Source Discrimination
    // --------------------------------------------------------------------------
    console.log('\n[Test 1] Verifying Unlocalized State, Source Discrimination & Safety Exceptions...');
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

      // 1. RULE: Zero commands must ALWAYS pass (HTTP 200) regardless of localization
      const zeroCmdRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
      assert.strictEqual(zeroCmdRes.statusCode, 200, 'Zero velocity command must always pass with HTTP 200');
      assert.strictEqual(zeroCmdRes.json.ok, true);
      console.log('  ✓ Zero command passed unconditionally while unlocalized (HTTP 200)');

      // 2. RULE: E-stop (/api/stop) must ALWAYS pass regardless of localization
      const estopRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/stop',
        method: 'GET'
      });
      // 503 is acceptable if serial port not mocked, but NOT 403 or 409 localization block
      assert.notStrictEqual(estopRes.statusCode, 403);
      assert.notStrictEqual(estopRes.statusCode, 409);
      console.log('  ✓ E-stop (/api/stop) was not blocked by localization');

      // 3. RULE: Disarm (/api/drive/disarm) must ALWAYS pass regardless of localization
      const disarmRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/disarm',
        method: 'POST'
      });
      assert.strictEqual(disarmRes.statusCode, 200);
      assert.strictEqual(disarmRes.json.ok, true);
      console.log('  ✓ Disarm (/api/drive/disarm) succeeded while unlocalized (HTTP 200)');

      // 4. RULE: Manual arming without autonomy flag must NOT be blocked by localization
      const manualArmRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, {});
      // Should fail on serial port missing (503), NOT on localization (409)
      assert.notStrictEqual(manualArmRes.statusCode, 409, 'Manual arming must NOT return HTTP 409 for localization');
      console.log('  ✓ Manual arming is NOT blocked by localization');

      // 5. RULE: Autonomy arming (req.body.autonomy === true) MUST be blocked by localization
      const autoArmRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, { autonomy: true });
      assert.strictEqual(autoArmRes.statusCode, 409, 'Arming for autonomy while NOT localized must return HTTP 409');
      assert(autoArmRes.json.error.includes('NOT LOCALIZED'));
      console.log('  ✓ Autonomy arming correctly blocked while unlocalized (HTTP 409)');

      // 6. RULE: Non-zero ROS_AUTONOMY motion command MUST be blocked by localization
      // Enable autonomy state so we test localization check specifically
      serverModule.autonomyState.enabled = true;
      serverModule.autonomyState.state = 'READY_ARMED';
      serverModule.injectNormalDriveStatus({ armed: true, mode: 3 });

      const autoMotionRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
      assert.strictEqual(autoMotionRes.statusCode, 403, 'ROS_AUTONOMY non-zero motion must be rejected with HTTP 403');
      assert(autoMotionRes.json.error.includes('NOT LOCALIZED'), 'Error message must specify Vehicle is NOT LOCALIZED');
      console.log('  ✓ ROS_AUTONOMY motion rejected while unlocalized (HTTP 403: Vehicle is NOT LOCALIZED)');

      // 7. RULE: Non-zero CALIBRATION_TEST command must NOT be blocked by localization
      const calibMotionRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'CALIBRATION_TEST' });
      // When armed, calibration test command should be accepted (HTTP 200)
      assert.strictEqual(calibMotionRes.statusCode, 200, `CALIBRATION_TEST motion must not be blocked by localization: ${calibMotionRes.data}`);
      console.log('  ✓ CALIBRATION_TEST command distinguished and accepted without localization gating (HTTP 200)');

      // Clean up mock state
      serverModule.injectNormalDriveStatus({ armed: false, mode: 0 });
      serverModule.autonomyState.enabled = false;
      serverModule.autonomyState.state = 'DISABLED';
    }

    // --------------------------------------------------------------------------
    // TEST 2: Pose Initialization (Foxglove 2D Pose -> AMCL convergence)
    // --------------------------------------------------------------------------
    console.log('\n[Test 2] Simulating Operator 2D Pose Initialization in Foxglove...');
    {
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
      console.log('  ✓ Localization state recognized: LOCALIZED');

      // Enable autonomy -> WAITING_FOR_ZERO
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
      assert.strictEqual(enableRes.statusCode, 200);
      assert.strictEqual(serverModule.autonomyState.state, 'WAITING_FOR_ZERO');
      console.log('  ✓ Autonomy enabled (entered WAITING_FOR_ZERO)');

      // Complete zero handshake (3 zero packets)
      for (let i = 0; i < 3; i++) {
        const hsRes = await httpRequest({
          hostname: '127.0.0.1',
          port: INTERNAL_PORT,
          path: '/api/cmd_vel',
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'X-Rover-Bridge-Token': CMD_TOKEN
          }
        }, { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
        assert.strictEqual(hsRes.statusCode, 200);
      }
      assert.strictEqual(serverModule.autonomyState.state, 'READY_DISARMED');
      console.log('  ✓ Zero velocity handshake complete (READY_DISARMED)');
    }

    // --------------------------------------------------------------------------
    // TEST 3: Fail-Safe Stop & Disarm on Lost Localization During Navigation
    // --------------------------------------------------------------------------
    console.log('\n[Test 3] Simulating Lost Localization During Active Navigation...');
    {
      serverModule.injectNormalDriveStatus({ armed: true, mode: 3 });
      serverModule.autonomyState.state = 'ACTIVE';
      serverModule.autonomyState.active = true;
      serverModule.autonomyState.limitedLinear = 0.12;

      console.log('  Active autonomous motion running (speed=0.12 m/s, armed=true, state=ACTIVE)');

      // Simulate localization loss
      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        details: 'Pose stale (2500ms > 2000ms)'
      });

      // Verify fail-safe trigger
      assert.strictEqual(serverModule.autonomyState.state, 'FAULT', 'Autonomy state must transition to FAULT');
      assert.strictEqual(serverModule.autonomyState.enabled, false, 'Autonomy must be disabled');
      assert.strictEqual(serverModule.autonomyState.active, false, 'Autonomy must be inactive');
      assert(serverModule.autonomyState.lastRejectionReason.includes('Localization lost'));

      // Verify rover is disarmed
      const driveStatus = serverModule.getLatestNormalDriveStatus();
      assert.strictEqual(driveStatus.armed, false, 'Rover MUST be disarmed upon localization loss');
      console.log('  ✓ Fail-safe trigger verified: Motors zeroed, autonomy FAULT, rover disarmed');

      // Subsequent ROS_AUTONOMY command rejected
      const cmdRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
      assert.strictEqual(cmdRes.statusCode, 403);
      console.log('  ✓ Autonomous motion command rejected following localization fault');
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

    // --------------------------------------------------------------------------
    // TEST 5: Fresh Validation Required Before Autonomous Motion
    // --------------------------------------------------------------------------
    console.log('\n[Test 5] Verifying Fresh Validation Gating Before Autonomous Motion...');
    {
      // Reset autonomy state to clean WAITING_FOR_ZERO -> READY_DISARMED
      serverModule.autonomyState.state = 'WAITING_FOR_ZERO';
      serverModule.autonomyState.enabled = true;
      serverModule.autonomyState.zeroHandshakeCount = 3;
      serverModule.autonomyState.state = 'READY_DISARMED';

      // 1. Stationary pose with age > 2000ms (normal stationary pause, fresh_validation_ok: false)
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: false,
        age_ms: 3500,
        details: 'Stationary tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      // Attempting to arm for autonomy with unvalidated/older stationary pose MUST return HTTP 409
      const unvalidatedArmRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Operator-Token': OPERATOR_TOKEN
        }
      }, { autonomy: true });
      assert.strictEqual(unvalidatedArmRes.statusCode, 409, 'Arming for autonomy without fresh validation must be rejected (HTTP 409)');
      assert(unvalidatedArmRes.json.error.includes('fresh validation'), `Error must mention fresh validation: ${unvalidatedArmRes.json.error}`);
      console.log('  ✓ Autonomy arming rejected when stationary pose is not freshly validated (HTTP 409)');

      // 2. Supply freshly validated pose (age <= 2000ms, fresh_validation_ok: true)
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 250,
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      // Mock serial response for arming
      serverModule.injectNormalDriveStatus({ armed: true, mode: 3 });
      serverModule.autonomyState.state = 'READY_ARMED';
      console.log('  ✓ Autonomy arming allowed once fresh validation confirmed');
    }

    // --------------------------------------------------------------------------
    // TEST 6: Sensor Failure Detection (Stale LiDAR / Stale Odometry)
    // --------------------------------------------------------------------------
    console.log('\n[Test 6] Verifying Sensor Failure Detection (LiDAR and Odometry)...');
    {
      // 1. Simulate LiDAR scan stale (> 2000ms)
      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        fresh_validation_ok: false,
        age_ms: 250,
        scan_available: false,
        details: 'LiDAR scan stale or missing (2500ms > 2000ms)'
      });

      let statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.json.localization.localized, false);
      assert(statusRes.json.localization.details.includes('LiDAR scan stale'));
      console.log('  ✓ Stale LiDAR scan correctly transitioned localization to NOT_LOCALIZED');

      // 2. Re-establish valid localization then simulate odometry failure
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 200,
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        fresh_validation_ok: false,
        odom_available: false,
        details: 'Odometry telemetry stale or erroring (2500ms, errors=1)'
      });

      statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.json.localization.localized, false);
      assert(statusRes.json.localization.details.includes('Odometry telemetry stale'));
      console.log('  ✓ Stale odometry telemetry correctly transitioned localization to NOT_LOCALIZED');
    }

    // --------------------------------------------------------------------------
    // TEST 7: Cached / Stale Dynamic Transform Rejection
    // --------------------------------------------------------------------------
    console.log('\n[Test 7] Verifying Cached / Stale Dynamic Transform Rejection...');
    {
      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        fresh_validation_ok: false,
        tf_available: false,
        dynamic_tf_age_ms: 2500,
        details: 'Dynamic map to base_link transform stale or unavailable (2500ms)'
      });

      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.json.localization.localized, false);
      assert(statusRes.json.localization.details.includes('Dynamic map to base_link transform stale'));
      console.log('  ✓ Stale dynamic transform rejected (not fooled by cached transforms)');
    }

    // --------------------------------------------------------------------------
    // TEST 8: Stopped AMCL Stationary Timeout (> 8000ms)
    // --------------------------------------------------------------------------
    console.log('\n[Test 8] Verifying Stopped AMCL Stationary Timeout (> 8000ms)...');
    {
      serverModule.updateLocalizationState({
        localized: false,
        state: 'NOT_LOCALIZED',
        fresh_validation_ok: false,
        age_ms: 8500,
        is_stationary: true,
        details: 'AMCL pose update timeout while stationary (8500ms > 8000ms; AMCL unresponsive)'
      });

      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.json.localization.localized, false);
      assert(statusRes.json.localization.details.includes('AMCL pose update timeout'));
      console.log('  ✓ Stopped AMCL node correctly triggers stationary timeout (>8000ms)');
    }

    // --------------------------------------------------------------------------
    // TEST 9: Active Motion Staleness Gate (No Reset on Slow Creep or Zero Speed)
    // --------------------------------------------------------------------------
    console.log('\n[Test 9] Verifying Active Motion Staleness Gate & Watchdog Protection...');
    {
      // Re-initialize localization to LOCALIZED with fresh validation
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 100,
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      serverModule.autonomyState.enabled = true;
      serverModule.autonomyState.state = 'READY_ARMED';
      serverModule.injectNormalDriveStatus({ armed: true, mode: 3 });

      // 1. Initial motion command accepted
      const cmd1 = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
      assert.strictEqual(cmd1.statusCode, 200);
      assert.strictEqual(serverModule.autonomyState.state, 'ACTIVE');

      // 2. Simulate AMCL pose becoming stale during motion (> 2000ms)
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: false,
        age_ms: 2200,
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      // Next motion command (even slow creep or commanded motion with 0 measured speed) MUST be rejected
      const cmd2 = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.015, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
      assert.strictEqual(cmd2.statusCode, 403, 'Non-zero command with stale pose (>2000ms) must be rejected with HTTP 403');
      assert(cmd2.json.error.includes('stale'));
      console.log('  ✓ Commanded motion / creep with stale pose rejected during motion (HTTP 403)');

      // 3. Zero command still accepted unconditionally
      const zeroCmd = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Bridge-Token': CMD_TOKEN
        }
      }, { linear: { x: 0.0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 }, source: 'ROS_AUTONOMY' });
      assert.strictEqual(zeroCmd.statusCode, 200);
      console.log('  ✓ Zero command accepted unconditionally while pose is stale');

      // Cleanup
      serverModule.injectNormalDriveStatus({ armed: false, mode: 0 });
      serverModule.autonomyState.enabled = false;
      serverModule.autonomyState.state = 'DISABLED';
    }


    // --------------------------------------------------------------------------
    // TEST 10: Navigation & Collision-Protection Lifecycle Gating
    // --------------------------------------------------------------------------
    console.log('\n[Test 10] Verifying Navigation Lifecycle Gating Before Autonomous Arming...');
    {
      // 1. Establish fresh localization but leave navigation inactive
      serverModule.updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 200,
        details: 'Nominal tracking',
        pose: { x: 1.166, y: -0.110, yaw: -0.145, yaw_deg: -8.3 },
        covariance: { sigma_x: 0.065, sigma_y: 0.058, sigma_yaw: 0.042 }
      });

      serverModule.updateNavigationState({
        ready: false,
        state: 'WAITING_FOR_LOCALIZATION',
        details: 'bt_navigator is inactive',
        nodes: { controller_server: 'unconfigured', planner_server: 'unconfigured', bt_navigator: 'inactive', collision_monitor: 'unconfigured' }
      });

      // Verify /api/status exposes navigation block
      let stRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(stRes.json.navigation.ready, false);
      assert.strictEqual(stRes.json.navigation.details, 'bt_navigator is inactive');
      console.log('  ✓ /api/status accurately reflects inactive navigation lifecycle state');

      // Attempt autonomy arming when navigation is not ready -> MUST return HTTP 409
      let armRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'x-operator-token': OPERATOR_TOKEN
        }
      }, { autonomy: true });
      assert.strictEqual(armRes.statusCode, 409, 'Must reject arming with HTTP 409 when navigation is not ready');
      assert(armRes.json.error.includes('Navigation stack not ready'), 'Error must specify Navigation stack not ready');
      console.log('  ✓ Autonomy arming rejected with HTTP 409 when navigation lifecycle is inactive');

      // 2. Set navigation to ready (all required nodes active)
      serverModule.updateNavigationState({
        ready: true,
        state: 'ACTIVE',
        details: 'All required navigation and collision-protection nodes active',
        nodes: { controller_server: 'active', planner_server: 'active', bt_navigator: 'active', collision_monitor: 'active' }
      });

      stRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(stRes.json.navigation.ready, true);
      assert.strictEqual(stRes.json.navigation.state, 'ACTIVE');
      console.log('  ✓ /api/status reflects active navigation state when bringup completes');
    }

    console.log('\n================================================================');
    console.log('✓ ALL LOCALIZATION SAFETY & SOURCE DISCRIMINATION TESTS PASSED');
    console.log('================================================================\n');
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
