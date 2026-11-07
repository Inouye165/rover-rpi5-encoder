// ==============================================================================
// test_dispatch_nav_readiness.js
// Regression tests for Nav2 readiness gate before arming/dispatching:
// 1. Stale cached ACTIVATION_FAILED plus live all-active/ready -> dispatch proceeds
// 2. Cached ready plus live inactive -> dispatch is rejected before arming
// 3. Refresh failure -> dispatch is rejected before arming
// 4. Response fields cannot report ready=false/inactive while simultaneously listing
//    all required nodes active and action-ready.
// ==============================================================================

const assert = require('assert');
const http = require('http');
const path = require('path');
const fs = require('fs');

process.env.PORT = '3862';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3863';
process.env.ROVER_OPERATOR_TOKEN = 'test_token_secret_123';
process.env.ROVER_NAV_BRIDGE_URL = 'http://127.0.0.1:3005';

const serverPath = fs.existsSync(path.join(__dirname, 'server.js'))
  ? './server.js'
  : (fs.existsSync(path.join(__dirname, '../server.js')) ? '../server.js' : './server.js');

const serverModule = require(serverPath);
const {
  app: publicApp,
  server: publicServer,
  internalCmdServer,
  autonomyState,
  navigationState,
  updateNavigationState,
  localizationState,
  updateLocalizationState,
  injectNormalDriveStatus,
  getLatestNormalDriveStatus,
  setSerialPort,
  setOdomPollingDisabled,
  refreshNavigationReadiness,
  REQUIRED_NAV_LIFECYCLE_NODES
} = serverModule;

const OPERATOR_TOKEN = process.env.ROVER_OPERATOR_TOKEN;
const PUBLIC_PORT = 3862;
const INTERNAL_PORT = 3863;
const NAV_BRIDGE_PORT = 3005;

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

async function runRegressionSuite() {
  console.log('==============================================================================');
  console.log('REGRESSION TEST: AUTHORITATIVE NAV2 READINESS GATE & DISPATCH INVARIANTS');
  console.log('==============================================================================\n');

  if (setOdomPollingDisabled) {
    setOdomPollingDisabled(true);
  }

  // Intercept serial packets
  const writtenPackets = [];
  const mockSerial = {
    isOpen: true,
    write: (pkt, cb) => {
      const buf = Buffer.from(pkt);
      writtenPackets.push(buf);
      // Auto-acknowledge arm packet FUNC_ARM_NORMAL_DRIVE (0x2C)
      if (buf.length >= 4 && buf[0] === 0xFF && buf[1] === 0xFC && buf[3] === 0x2C) {
        setImmediate(() => {
          injectNormalDriveStatus({
            armed: true,
            mode: 3,
            seq: (getLatestNormalDriveStatus() ? getLatestNormalDriveStatus().seq : 0) + 1,
            source: 0,
            reqLinear: 0.0,
            reqAngular: 0.0,
            limLinear: 0.0,
            limAngular: 0.0
          });
        });
      }
      if (typeof cb === 'function') cb(null);
    },
    close: (cb) => { if (cb) cb(); }
  };
  setSerialPort(mockSerial);

  function findPackets(funcId) {
    return writtenPackets.filter(pkt =>
      pkt.length >= 4 && pkt[0] === 0xFF && pkt[1] === 0xFC && pkt[3] === funcId
    );
  }

  // Configurable mock for rover_nav_bridge
  let bridgeStatusResponse = null;
  let bridgeStatusStatusCode = 200;
  let nav2GoalSent = false;
  let receivedNav2GoalPayload = null;

  const mockNav2Server = http.createServer((req, res) => {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      if (req.url === '/api/nav/status' && req.method === 'GET') {
        res.writeHead(bridgeStatusStatusCode, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(bridgeStatusResponse || {}));
      } else if (req.url === '/api/nav/dispatch' && req.method === 'POST') {
        nav2GoalSent = true;
        try { receivedNav2GoalPayload = JSON.parse(body); } catch (_) { receivedNav2GoalPayload = body; }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, message: 'Goal dispatched to Nav2' }));
      } else if (req.url === '/api/nav/nomotion_update' && req.method === 'POST') {
        // Mock AMCL no-motion update succeeding
        localizationState.seq = (localizationState.seq || 0) + 1;
        localizationState.sampleTimestamp = Date.now();
        localizationState.ageMs = 10;
        localizationState.freshValidationOk = true;
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, message: 'AMCL updated' }));
      } else {
        res.writeHead(404);
        res.end();
      }
    });
  });

  await new Promise(r => mockNav2Server.listen(NAV_BRIDGE_PORT, '127.0.0.1', r));
  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));
  await new Promise(r => internalCmdServer.listen(INTERNAL_PORT, '127.0.0.1', r));

  try {
    // --------------------------------------------------------------------------
    // TEST 1: Stale cached ACTIVATION_FAILED plus live all-active/ready -> dispatch proceeds
    // --------------------------------------------------------------------------
    console.log('[TEST 1] Stale cached ACTIVATION_FAILED + live all-active/ready -> dispatch proceeds...');
    {
      // 1. Preload stale cached state: ACTIVATION_FAILED, ready: false
      updateNavigationState({
        ready: false,
        state: 'ACTIVATION_FAILED',
        details: 'Navigation bringup failed after 3 attempts. Inactive: bt_navigator=inactive',
        nodes: { controller_server: 'active', planner_server: 'active', bt_navigator: 'inactive', collision_monitor: 'active' }
      });
      assert.strictEqual(navigationState.ready, false);
      assert.strictEqual(navigationState.state, 'ACTIVATION_FAILED');

      // 2. Configure mock live bridge status: everything is healthy and active!
      bridgeStatusStatusCode = 200;
      bridgeStatusResponse = {
        ok: true,
        action_server_ready: true,
        ready_to_accept_goals: true,
        navigation: {
          ready: true,
          state: 'ACTIVE',
          details: 'All required navigation and collision-protection nodes active',
          nodes: {
            controller_server: 'active',
            planner_server: 'active',
            bt_navigator: 'active',
            collision_monitor: 'active'
          }
        }
      };

      // 3. Ensure localization is healthy
      updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 50,
        is_stationary: true,
        pose: { x: 1.0, y: 1.0, yaw: 0.0, yaw_deg: 0.0 },
        seq: 10,
        sample_timestamp: Date.now()
      });

      // Initially disarmed
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        source: 0,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      writtenPackets.length = 0;
      nav2GoalSent = false;
      receivedNav2GoalPayload = null;

      // 4. Dispatch goal
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/dispatch',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Operator-Token': OPERATOR_TOKEN
        }
      }, { target_x: 2.0, target_y: 2.0 });

      assert.strictEqual(res.statusCode, 200, `Dispatch should succeed with 200, got ${res.statusCode}: ${res.data}`);
      assert.strictEqual(res.json.ok, true);
      assert.strictEqual(nav2GoalSent, true, 'Goal must be dispatched to Nav2 bridge');
      assert.strictEqual(navigationState.ready, true, 'Cached navigationState.ready must be atomically updated to true');
      assert.strictEqual(navigationState.state, 'ACTIVE', 'Cached navigationState.state must be updated to ACTIVE');
      
      const armPackets = findPackets(0x2C);
      assert(armPackets.length > 0, 'Drivetrain must be armed for valid dispatch');
      console.log('  PASS: Dispatch proceeded despite stale ACTIVATION_FAILED cache; cache updated atomically.');
    }

    // --------------------------------------------------------------------------
    // TEST 2: Cached ready plus live inactive -> dispatch rejected before arming
    // --------------------------------------------------------------------------
    console.log('\n[TEST 2] Cached ready + live inactive (bt_navigator inactive) -> rejected before arming...');
    {
      // 1. Preload cached state as READY
      updateNavigationState({
        ready: true,
        state: 'ACTIVE',
        details: 'All required navigation and collision-protection nodes active',
        nodes: { controller_server: 'active', planner_server: 'active', bt_navigator: 'active', collision_monitor: 'active' }
      });
      assert.strictEqual(navigationState.ready, true);

      // 2. Configure mock live bridge status: bt_navigator has gone inactive!
      bridgeStatusStatusCode = 200;
      bridgeStatusResponse = {
        ok: false,
        action_server_ready: true,
        ready_to_accept_goals: false,
        navigation: {
          ready: false,
          state: 'INACTIVE',
          details: 'Required navigation lifecycle nodes inactive: bt_navigator=inactive',
          nodes: {
            controller_server: 'active',
            planner_server: 'active',
            bt_navigator: 'inactive',
            collision_monitor: 'active'
          }
        }
      };

      // Disarmed
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        source: 0,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      writtenPackets.length = 0;
      nav2GoalSent = false;
      receivedNav2GoalPayload = null;

      // 3. Dispatch goal
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/dispatch',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Operator-Token': OPERATOR_TOKEN
        }
      }, { target_x: 2.0, target_y: 2.0 });

      assert.strictEqual(res.statusCode, 409, `Dispatch must be rejected with HTTP 409, got ${res.statusCode}: ${res.data}`);
      assert.strictEqual(res.json.ok, false);
      assert(res.json.error.includes('Navigation stack not ready') || res.json.error.includes('bt_navigator'), `Error must explain inactive node: ${res.json.error}`);
      assert.strictEqual(nav2GoalSent, false, 'No Nav2 goal must be sent');
      
      const armPackets = findPackets(0x2C);
      assert.strictEqual(armPackets.length, 0, 'CRITICAL: Drivetrain MUST NOT be armed when live readiness fails');
      assert.strictEqual(getLatestNormalDriveStatus().armed, false, 'Rover must remain disarmed');
      assert.strictEqual(navigationState.ready, false, 'Cached navigationState must reflect failure');
      console.log('  PASS: Rejected before arming when live node is inactive; zero arm packets sent.');
    }

    // --------------------------------------------------------------------------
    // TEST 3: Refresh failure (HTTP 500 / network error) -> rejected before arming
    // --------------------------------------------------------------------------
    console.log('\n[TEST 3] Refresh failure (bridge returns HTTP 500) -> rejected before arming...');
    {
      // 1. Cached state was ready
      updateNavigationState({
        ready: true,
        state: 'ACTIVE',
        details: 'All required navigation and collision-protection nodes active',
        nodes: { controller_server: 'active', planner_server: 'active', bt_navigator: 'active', collision_monitor: 'active' }
      });

      // 2. Mock bridge status fails with HTTP 500
      bridgeStatusStatusCode = 500;
      bridgeStatusResponse = { error: 'Internal Bridge Failure' };

      // Disarmed
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        source: 0,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      writtenPackets.length = 0;
      nav2GoalSent = false;
      receivedNav2GoalPayload = null;

      // 3. Dispatch goal
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/dispatch',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Operator-Token': OPERATOR_TOKEN
        }
      }, { target_x: 2.0, target_y: 2.0 });

      assert.strictEqual(res.statusCode, 409, `Dispatch must be rejected with HTTP 409, got ${res.statusCode}: ${res.data}`);
      assert.strictEqual(res.json.ok, false);
      assert(res.json.error.includes('Navigation stack not ready') || res.json.error.includes('rover_nav_bridge returned HTTP 500'), `Error details: ${res.json.error}`);
      assert.strictEqual(nav2GoalSent, false, 'No Nav2 goal must be sent');
      
      const armPackets = findPackets(0x2C);
      assert.strictEqual(armPackets.length, 0, 'CRITICAL: Drivetrain MUST NOT be armed when refresh fails');
      assert.strictEqual(getLatestNormalDriveStatus().armed, false, 'Rover must remain disarmed');
      console.log('  PASS: Rejected before arming on bridge refresh failure; zero arm packets sent.');
    }

    // --------------------------------------------------------------------------
    // TEST 4: Response fields cannot report ready=false/inactive while all nodes active & action-ready
    // --------------------------------------------------------------------------
    console.log('\n[TEST 4] Contradiction prevention: cannot report ready=false with all nodes active & action ready...');
    {
      // Attempt to set contradiction: ready=false, state=ACTIVATION_FAILED, but all required nodes active
      updateNavigationState({
        ready: false,
        state: 'ACTIVATION_FAILED',
        details: 'Old stale error from boot',
        nodes: {
          controller_server: 'active',
          planner_server: 'active',
          bt_navigator: 'active',
          collision_monitor: 'active'
        },
        action_server_ready: true
      });

      // Invariant: updateNavigationState enforces consistency
      assert.strictEqual(navigationState.ready, true, 'Must reconcile to ready=true when all nodes are active');
      assert.strictEqual(navigationState.state, 'ACTIVE', 'Must reconcile state to ACTIVE');

      // Check /api/status response consistency
      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.statusCode, 200);
      assert.strictEqual(statusRes.json.navigation.ready, true);
      assert.strictEqual(statusRes.json.navigation.state, 'ACTIVE');

      // Now verify that if any required node is inactive, ready CANNOT be true
      updateNavigationState({
        ready: true, // Caller tries to claim ready=true
        state: 'ACTIVE',
        nodes: {
          controller_server: 'active',
          planner_server: 'active',
          bt_navigator: 'inactive', // but bt_navigator is inactive!
          collision_monitor: 'active'
        },
        action_server_ready: true
      });
      assert.strictEqual(navigationState.ready, false, 'Must enforce ready=false if any required node is inactive');
      assert.notStrictEqual(navigationState.state, 'ACTIVE', 'State must not be ACTIVE when node is inactive');

      const statusRes2 = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes2.json.navigation.ready, false);
      assert.notStrictEqual(statusRes2.json.navigation.state, 'ACTIVE');
      console.log('  PASS: Reconciled status consistency verified; contradictory states prevented.');
    }

    console.log('\n==============================================================================');
    console.log('ALL NAV2 READINESS GATE REGRESSION TESTS PASSED SUCCESSFULLY!');
    console.log('==============================================================================\n');
    process.exit(0);
  } catch (err) {
    console.error('\nTEST FAILURE:', err);
    process.exit(1);
  } finally {
    try { mockNav2Server.close(); } catch (_) {}
    try { publicServer.close(); } catch (_) {}
    try { internalCmdServer.close(); } catch (_) {}
  }
}

runRegressionSuite();
