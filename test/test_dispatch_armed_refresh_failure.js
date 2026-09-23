// ==============================================================================
// test_dispatch_armed_refresh_failure.js
// Regression test: If pre-dispatch localization refresh or post-refresh validation
// fails while the rover was already armed, the dispatch path must:
// 1. Send NO Nav2 goal
// 2. Command zero velocity
// 3. Invoke disarm
// 4. Return HTTP 409
// ==============================================================================

const assert = require('assert');
const http = require('http');
const path = require('path');
const fs = require('fs');

process.env.PORT = '3850';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3851';

const serverPath = fs.existsSync(path.join(__dirname, 'server.js'))
  ? './server.js'
  : (fs.existsSync(path.join(__dirname, '../server.js')) ? '../server.js' : './server.js');

const serverModule = require(serverPath);
const {
  app: publicApp,
  server: publicServer,
  internalCmdServer,
  autonomyState,
  localizationState,
  updateLocalizationState,
  injectNormalDriveStatus,
  getLatestNormalDriveStatus,
  setSerialPort,
  setOdomPollingDisabled
} = serverModule;

const OPERATOR_TOKEN = process.env.ROVER_OPERATOR_TOKEN;
const PUBLIC_PORT = 3850;
const INTERNAL_PORT = 3851;
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
  console.log('REGRESSION TEST: ARMED DISPATCH REFRESH FAILURE INVARIANT');
  console.log('==============================================================================\n');

  // Disable background odometry polling to prevent external HTTP requests
  if (setOdomPollingDisabled) {
    setOdomPollingDisabled(true);
  }

  // Intercept all serial packets sent to hardware
  const writtenPackets = [];
  const mockSerial = {
    isOpen: true,
    write: (pkt, cb) => {
      writtenPackets.push(Buffer.from(pkt));
      if (typeof cb === 'function') cb(null);
    },
    close: (cb) => { if (cb) cb(); }
  };
  setSerialPort(mockSerial);

  // Helper to inspect binary packets
  // Frame layout: [HEAD=0xFF, DEVICE_ID=0xFC, extLen, funcId, ...payload, checksum]
  function findPackets(funcId) {
    return writtenPackets.filter(pkt =>
      pkt.length >= 4 && pkt[0] === 0xFF && pkt[1] === 0xFC && pkt[3] === funcId
    );
  }

  // Start a mock Nav2 bridge server on 127.0.0.1:3005 to track whether Nav2 goals are sent
  let nav2GoalSent = false;
  let receivedNav2GoalPayload = null;
  let simulateRefreshSuccessForTest2 = false;

  const mockNav2Server = http.createServer((req, res) => {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      if (req.url === '/api/nav/dispatch' && req.method === 'POST') {
        nav2GoalSent = true;
        try { receivedNav2GoalPayload = JSON.parse(body); } catch (_) { receivedNav2GoalPayload = body; }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, message: 'Goal dispatched to Nav2' }));
      } else if (req.url === '/api/nav/nomotion_update' && req.method === 'POST') {
        if (simulateRefreshSuccessForTest2) {
          // Advance seq and timestamp so refresh succeeds, but pose coordinates remain NaN
          localizationState.seq = (localizationState.seq || 0) + 1;
          localizationState.sampleTimestamp = Date.now();
          localizationState.ageMs = 10;
          res.writeHead(200, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify({ ok: true, message: 'AMCL updated' }));
        } else {
          // Simulate AMCL service failing or returning 503
          res.writeHead(503, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify({ ok: false, error: 'AMCL service unavailable' }));
        }
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
    // TEST 1: Pre-dispatch refresh failure while ALREADY ARMED
    // --------------------------------------------------------------------------
    console.log('[TEST 1] Pre-dispatch localization refresh failure while already armed...');
    {
      simulateRefreshSuccessForTest2 = false;

      // 1. Establish already-armed simulated state
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        reqLinear: 0.15,
        reqAngular: 0.05,
        limLinear: 0.15,
        limAngular: 0.05
      });
      assert.strictEqual(getLatestNormalDriveStatus().armed, true, 'Initial state must be armed');

      autonomyState.enabled = true;
      autonomyState.state = 'READY_ARMED';

      // 2. Set localization state so that refresh will fail (stale sample, seq/timestamp frozen)
      updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: false,
        age_ms: 3500,
        is_stationary: true,
        pose: { x: 1.0, y: 2.0, yaw: 0.0, yaw_deg: 0.0 },
        seq: 100,
        sample_timestamp: 100000
      });

      // Clear trackers
      writtenPackets.length = 0;
      nav2GoalSent = false;
      receivedNav2GoalPayload = null;

      // 3. Attempt dispatch while armed
      console.log('  Sending POST /api/navigation/dispatch while armed (refresh will fail)...');
      const dispatchRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/dispatch',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Operator-Token': OPERATOR_TOKEN
        }
      }, { target_x: 2.5, target_y: 3.5 });

      console.log(`  Dispatch response: HTTP ${dispatchRes.statusCode}`);
      console.log(`  Response body: ${dispatchRes.data}`);

      // Invariant 1: HTTP 409 is returned
      assert.strictEqual(dispatchRes.statusCode, 409, 'Must return HTTP 409 on refresh failure');
      assert.strictEqual(dispatchRes.json.ok, false, 'ok field must be false');
      assert(
        dispatchRes.json.error.includes('Pre-dispatch localization refresh failed'),
        `Error must mention refresh failure: ${dispatchRes.json.error}`
      );
      console.log('  PASS 1: HTTP 409 returned with refresh failure error.');

      // Invariant 2: No Nav2 goal is sent
      assert.strictEqual(nav2GoalSent, false, 'CRITICAL INVARIANT: No Nav2 goal must be sent to Nav2 bridge');
      assert.strictEqual(receivedNav2GoalPayload, null, 'Nav2 payload must be null');
      console.log('  PASS 2: No Nav2 goal was sent to Nav2 bridge.');

      // Invariant 3: Zero velocity is commanded
      const motionPackets = findPackets(0x12);
      assert(motionPackets.length > 0, 'At least one FUNC_MOTION (0x12) packet must be commanded');
      // Last motion packet payload: vx (2 bytes), vy (2 bytes), vz (2 bytes) = all 0
      const lastMotion = motionPackets[motionPackets.length - 1];
      const vx = lastMotion.readInt16LE(4);
      const vy = lastMotion.readInt16LE(6);
      const vz = lastMotion.readInt16LE(8);
      assert.strictEqual(vx, 0, 'Commanded linear velocity vx must be 0');
      assert.strictEqual(vy, 0, 'Commanded velocity vy must be 0');
      assert.strictEqual(vz, 0, 'Commanded angular velocity vz must be 0');
      console.log(`  PASS 3: Zero motion commanded via FUNC_MOTION [vx=${vx}, vy=${vy}, vz=${vz}].`);

      // Invariant 4: Disarm is invoked
      const disarmPackets = findPackets(0x2D);
      assert(disarmPackets.length > 0, 'FUNC_DISARM_NORMAL_DRIVE (0x2D) packet must be sent to serial');
      assert.strictEqual(disarmPackets[0][4], 1, 'Disarm payload byte must be 1');
      assert.strictEqual(getLatestNormalDriveStatus().armed, false, 'latestNormalDriveStatus.armed must be false');
      assert.strictEqual(autonomyState.state, 'FAULT', 'autonomyState.state must be FAULT');
      console.log('  PASS 4: Disarm invoked: FUNC_DISARM_NORMAL_DRIVE packet sent and armed=false confirmed.');
    }

    // --------------------------------------------------------------------------
    // TEST 2: Post-refresh validation failure while ALREADY ARMED
    // --------------------------------------------------------------------------
    console.log('\n[TEST 2] Post-refresh localization validation failure while already armed...');
    {
      simulateRefreshSuccessForTest2 = true;

      // 1. Establish already-armed simulated state
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        source: 0,
        reqLinear: 0.10,
        reqAngular: 0.0,
        limLinear: 0.10,
        limAngular: 0.0
      });
      assert.strictEqual(getLatestNormalDriveStatus().armed, true, 'Initial state must be armed');

      autonomyState.enabled = true;
      autonomyState.state = 'READY_ARMED';

      // 2. Set localization state: fresh_validation_ok true, but pose coordinates are NaN
      updateLocalizationState({
        localized: true,
        state: 'LOCALIZED',
        fresh_validation_ok: true,
        age_ms: 50,
        is_stationary: true,
        pose: { x: NaN, y: NaN, yaw: 0.0, yaw_deg: 0.0 },
        seq: 500,
        sample_timestamp: Date.now()
      });

      // Clear trackers
      writtenPackets.length = 0;
      nav2GoalSent = false;
      receivedNav2GoalPayload = null;

      // 3. Attempt dispatch while armed
      console.log('  Sending POST /api/navigation/dispatch with NaN pose while armed...');
      const dispatchRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/dispatch',
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Rover-Operator-Token': OPERATOR_TOKEN
        }
      }, { target_x: 1.0, target_y: 1.0 });

      console.log(`  Dispatch response: HTTP ${dispatchRes.statusCode}`);
      console.log(`  Response body: ${dispatchRes.data}`);

      // Invariant 1: HTTP 409 is returned
      assert.strictEqual(dispatchRes.statusCode, 409, 'Must return HTTP 409 on validation failure');
      assert.strictEqual(dispatchRes.json.ok, false);
      assert(
        dispatchRes.json.error.includes('Rover is not localized with valid AMCL pose after refresh') ||
        dispatchRes.json.error.includes('Pre-dispatch localization refresh failed'),
        `Error must explain localization validation failure: ${dispatchRes.json.error}`
      );
      console.log('  PASS 1: HTTP 409 returned on validation failure.');

      // Invariant 2: No Nav2 goal is sent
      assert.strictEqual(nav2GoalSent, false, 'CRITICAL INVARIANT: No Nav2 goal must be sent');
      console.log('  PASS 2: No Nav2 goal was sent.');

      // Invariant 3: Zero velocity commanded
      const motionPackets = findPackets(0x12);
      assert(motionPackets.length > 0, 'FUNC_MOTION zero packet must be sent');
      const lastMotion = motionPackets[motionPackets.length - 1];
      assert.strictEqual(lastMotion.readInt16LE(4), 0, 'vx must be 0');
      assert.strictEqual(lastMotion.readInt16LE(8), 0, 'vz must be 0');
      console.log('  PASS 3: Zero velocity commanded to serial.');

      // Invariant 4: Disarm is invoked
      const disarmPackets = findPackets(0x2D);
      assert(disarmPackets.length > 0, 'FUNC_DISARM_NORMAL_DRIVE packet must be sent');
      assert.strictEqual(getLatestNormalDriveStatus().armed, false, 'latestNormalDriveStatus.armed must be false');
      assert.strictEqual(autonomyState.state, 'FAULT');
      console.log('  PASS 4: Disarm invoked and confirmed.');
    }

    console.log('\n==============================================================================');
    console.log('ALL INVARIANTS VERIFIED SUCCESSFULLY:');
    console.log('  - No Nav2 goal sent on refresh/validation failure');
    console.log('  - Zero velocity commanded immediately');
    console.log('  - Drivetrain disarmed via FUNC_DISARM_NORMAL_DRIVE packet');
    console.log('  - HTTP 409 returned to caller');
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
