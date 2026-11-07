// ==============================================================================
// test_cockpit_recovery_fault.js
// Regression test: Verifies that Cockpit shared autonomy fault handler
// and navigation bridge proxy correctly function:
// 1. Exercises the real /api/navigation/status route against a mock nav bridge
// 2. Verifies drivetrain recovery exhaustion (spinFailedExhausted):
//    - Sends outbound POST /api/nav/cancel to mock bridge
//    - Disarms drivetrain via serial protocol (FUNC_DISARM_NORMAL_DRIVE)
//    - Locks autonomyState.state in 'FAULT' and disables command acceptance
//    - Rejects subsequent autonomy commands with 403 Forbidden
// 3. Tests nav bridge failure explicitly:
//    - Bridge returning HTTP 500 error
//    - Bridge unreachable / connection refused
//    - Ensures triggerNav2Cancel() and /api/navigation/status fail safe without crash
// ==============================================================================

const assert = require('assert');
const http = require('http');

process.env.PORT = '3870';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3871';
process.env.ROVER_NAV_BRIDGE_URL = 'http://127.0.0.1:3875';
process.env.ROVER_CMD_VEL_TOKEN = '0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef';

const serverModule = require('../server.js');
const {
  app: publicApp,
  server: publicServer,
  internalCmdServer,
  autonomyState,
  injectNormalDriveStatus,
  setSerialPort,
  setOdomPollingDisabled,
  updateLocalizationState
} = serverModule;

const PUBLIC_PORT = 3870;
const INTERNAL_PORT = 3871;
const NAV_BRIDGE_PORT = 3875;

const FUNC_DISARM_NORMAL_DRIVE = 0x2D;

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

function sendCmdVel(payload) {
  return httpRequest({
    hostname: '127.0.0.1',
    port: INTERNAL_PORT,
    path: '/api/cmd_vel',
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'x-rover-bridge-token': process.env.ROVER_CMD_VEL_TOKEN
    }
  }, payload);
}

// Mock SerialPort tracking written packets
class MockSerialPort {
  constructor() {
    this.isOpen = true;
    this.writtenPackets = [];
  }
  write(buf) {
    this.writtenPackets.push(Buffer.from(buf));
  }
}

async function runRecoveryFaultSuite() {
  console.log('==============================================================================');
  console.log('TEST SUITE: Cockpit Navigation Status & Autonomy Recovery Fault Integration');
  console.log('==============================================================================');

  setOdomPollingDisabled(true);

  // 1. Setup Mock Nav Bridge Server on port 3875
  let navCancelCount = 0;
  let navStatusQueryCount = 0;
  let lastNavCancelBody = null;
  let bridgeFailStatus = false;
  let currentGoalId = 'goal_1791153309774_p1bldt';
  let currentGoalStatus = 'EXECUTING';

  const mockNavBridge = http.createServer((req, res) => {
    if (req.method === 'POST' && req.url === '/api/nav/cancel') {
      navCancelCount++;
      let body = '';
      req.on('data', c => body += c);
      req.on('end', () => {
        lastNavCancelBody = body;
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: true, cancelled: true, goal_id: currentGoalId }));
      });
      return;
    }
    if (req.method === 'GET' && req.url === '/api/nav/status') {
      navStatusQueryCount++;
      if (bridgeFailStatus) {
        res.writeHead(500, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: false, error: 'Simulated bridge internal error' }));
        return;
      }
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({
        ok: true,
        status: currentGoalStatus,
        goal_id: currentGoalId,
        action_server_ready: true,
        navigation: {
          ready: true,
          nodes: {
            controller_server: 'active',
            planner_server: 'active',
            bt_navigator: 'active',
            collision_monitor: 'active'
          }
        }
      }));
      return;
    }
    res.writeHead(404);
    res.end();
  });

  await new Promise(r => mockNavBridge.listen(NAV_BRIDGE_PORT, '127.0.0.1', r));
  console.log('  [Setup] Mock Nav Bridge listening on port', NAV_BRIDGE_PORT);

  // 2. Start Cockpit servers
  if (!publicServer.listening) {
    await new Promise(r => publicServer.listen(PUBLIC_PORT, r));
  }
  if (!internalCmdServer.listening) {
    await new Promise(r => internalCmdServer.listen(INTERNAL_PORT, '127.0.0.1', r));
  }

  const mockSerial = new MockSerialPort();
  setSerialPort(mockSerial);

  try {
    // --------------------------------------------------------------------------
    // Test 1: Exercise Real /api/navigation/status Route (Healthy Bridge)
    // --------------------------------------------------------------------------
    console.log('\n  [Step 1] Exercising real /api/navigation/status against mock bridge...');
    const statusRes = await httpRequest({
      hostname: '127.0.0.1',
      port: PUBLIC_PORT,
      path: '/api/navigation/status',
      method: 'GET'
    });
    assert.strictEqual(statusRes.statusCode, 200, `Expected HTTP 200 from /api/navigation/status, got ${statusRes.statusCode}: ${statusRes.data}`);
    assert.strictEqual(statusRes.json.ok, true, 'Status response must have ok=true');
    assert.strictEqual(statusRes.json.status, 'EXECUTING', 'Status response must match mock bridge status');
    assert.strictEqual(statusRes.json.goal_id, currentGoalId, 'Status response must reflect goal_id');
    assert.strictEqual(navStatusQueryCount, 1, 'Mock bridge must have received status query');
    console.log('    Confirmed: /api/navigation/status routed successfully with HTTP 200.');

    // --------------------------------------------------------------------------
    // Test 2: Establish Active Autonomy Navigation State
    // --------------------------------------------------------------------------
    console.log('\n  [Step 2] Establishing active autonomy navigation state...');
    
    // Simulate initial disarmed status to enable autonomy
    injectNormalDriveStatus({
      armed: false,
      mode: 0,
      lockStatus: false,
      spinRecoveryState: 'IDLE',
      spinFailedExhausted: false
    });

    // Operator enables autonomy
    const enableRes = await httpRequest({
      hostname: '127.0.0.1',
      port: PUBLIC_PORT,
      path: '/api/autonomy/enable',
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    }, {});
    assert.strictEqual(enableRes.statusCode, 200);
    assert.strictEqual(autonomyState.state, 'WAITING_FOR_ZERO');

    // Send 3 zero commands to complete zero handshake
    for (let i = 0; i < 3; i++) {
      const zRes = await sendCmdVel({ linear: { x: 0.0 }, angular: { z: 0.0 } });
      assert.strictEqual(zRes.statusCode, 200);
    }
    assert.strictEqual(autonomyState.state, 'READY_DISARMED');

    // Simulate arming rover for autonomy
    injectNormalDriveStatus({
      armed: true,
      mode: 3,
      lockStatus: false,
      spinRecoveryState: 'IDLE',
      spinFailedExhausted: false
    });
    autonomyState.state = 'READY_ARMED';

    // Set localization state to valid
    updateLocalizationState({
      localized: true,
      state: 'LOCALIZED',
      age_ms: 100,
      x: 1.19,
      y: -0.04,
      yaw_deg: -5.0
    });

    // Send active non-zero motion command
    const moveRes = await sendCmdVel({ linear: { x: 0.15 }, angular: { z: 0.05 } });
    assert.strictEqual(moveRes.statusCode, 200);
    assert.strictEqual(autonomyState.state, 'ACTIVE');
    assert.strictEqual(autonomyState.enabled, true);
    console.log('    Confirmed: Autonomy is ACTIVE, rover is armed, goal in pursuit.');

    // --------------------------------------------------------------------------
    // Test 3: Ingest Recovery Exhaustion (spinFailedExhausted = true)
    // --------------------------------------------------------------------------
    console.log('\n  [Step 3] Ingesting recovery exhaustion (spinFailedExhausted = true)...');
    mockSerial.writtenPackets = [];
    const prevNavCancels = navCancelCount;

    injectNormalDriveStatus({
      armed: true,
      mode: 3,
      lockStatus: false,
      spinRecoveryState: 'SPIN_RECOVERY_FAILED_LATCHED',
      spinFailedExhausted: true
    });

    // Wait 50 ms for async triggerNav2Cancel HTTP request to complete
    await new Promise(r => setTimeout(r, 60));

    // Assertions for fault response:
    assert.strictEqual(navCancelCount, prevNavCancels + 1, 'Nav2 outbound cancel must be called exactly once');
    assert.strictEqual(autonomyState.state, 'FAULT', 'Autonomy state must transition to FAULT');
    assert.strictEqual(autonomyState.enabled, false, 'Autonomy must be disabled');
    assert.strictEqual(autonomyState.active, false, 'Active motion must be false');
    
    // Verify serial disarm packet was sent (FUNC_DISARM_NORMAL_DRIVE = 0x2D at index 3)
    const disarmPkts = mockSerial.writtenPackets.filter(p => p.length >= 4 && p[3] === FUNC_DISARM_NORMAL_DRIVE);
    assert.strictEqual(disarmPkts.length, 1, 'Serial FUNC_DISARM_NORMAL_DRIVE packet must be sent');
    console.log('    Confirmed: Outbound goal cancellation sent, serial disarm sent, autonomy locked in FAULT.');

    // --------------------------------------------------------------------------
    // Test 4: Repeated Fault Telemetry Packets (Idempotency)
    // --------------------------------------------------------------------------
    console.log('\n  [Step 4] Sending repeated fault telemetry packets...');
    const cancelsBeforeRepeats = navCancelCount;

    for (let i = 0; i < 5; i++) {
      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        lockStatus: false,
        spinRecoveryState: 'SPIN_RECOVERY_FAILED_LATCHED',
        spinFailedExhausted: true
      });
    }
    await new Promise(r => setTimeout(r, 50));

    const disarmPktsAfterRepeats = mockSerial.writtenPackets.filter(p => p.length >= 4 && p[3] === FUNC_DISARM_NORMAL_DRIVE);
    assert.strictEqual(navCancelCount, cancelsBeforeRepeats, 'Nav2 cancel must NOT be spammed on repeated fault packets');
    assert.strictEqual(disarmPktsAfterRepeats.length, 1, 'Serial disarm must NOT be spammed (remains exactly 1)');
    assert.strictEqual(autonomyState.state, 'FAULT');
    console.log('    Confirmed: Repeated fault packets handled idempotently without spam.');

    // --------------------------------------------------------------------------
    // Test 5: Reject Further Autonomy Commands While in FAULT
    // --------------------------------------------------------------------------
    console.log('\n  [Step 5] Rejecting further autonomy commands while locked in FAULT...');
    
    // In firmware, disarm clears spinFailedExhausted
    injectNormalDriveStatus({
      armed: false,
      mode: 0,
      lockStatus: true,
      spinRecoveryState: 'IDLE',
      spinFailedExhausted: false
    });

    // In Cockpit, autonomyState MUST remain locked in FAULT
    assert.strictEqual(autonomyState.state, 'FAULT', 'Autonomy state must remain locked in FAULT');

    // Nav2 attempts to send motion command from previous goal
    const rejectedCmd = await sendCmdVel({ linear: { x: 0.10 }, angular: { z: 0.00 } });
    assert.strictEqual(rejectedCmd.statusCode, 403, 'Subsequent commands must be rejected with 403 Forbidden');
    assert.strictEqual(rejectedCmd.json.ok, false);
    assert.strictEqual(autonomyState.state, 'FAULT', 'Autonomy state remains FAULT');
    console.log('    Confirmed: Subsequent commands strictly rejected with 403 Forbidden.');

    // --------------------------------------------------------------------------
    // Test 6: Explicit Nav Bridge Failure Testing
    // --------------------------------------------------------------------------
    console.log('\n  [Step 6] Testing explicit nav bridge failure modes...');
    
    // 6A. Bridge returns HTTP 500 error
    bridgeFailStatus = true;
    const bridgeErrRes = await httpRequest({
      hostname: '127.0.0.1',
      port: PUBLIC_PORT,
      path: '/api/navigation/status',
      method: 'GET'
    });
    // Server should forward bridge status code or return error json without unhandled crash
    assert.ok([500, 502, 503].includes(bridgeErrRes.statusCode), `Expected 5xx on bridge error, got ${bridgeErrRes.statusCode}`);
    console.log(`    6A: Handled bridge HTTP 500 cleanly (returned ${bridgeErrRes.statusCode}) without crashing.`);
    bridgeFailStatus = false;

    // 6B. Test triggerNav2Cancel when bridge is down or unreachable
    await new Promise(r => mockNavBridge.close(r));
    console.log('    Mock Nav Bridge closed to simulate bridge disconnection / down state.');

    // Re-enable and trigger fault with bridge completely down
    autonomyState.state = 'ACTIVE';
    autonomyState.enabled = true;
    mockSerial.writtenPackets = [];

    // Inject fault while bridge is unreachable
    injectNormalDriveStatus({
      armed: true,
      mode: 3,
      lockStatus: false,
      spinRecoveryState: 'SPIN_RECOVERY_FAILED_LATCHED',
      spinFailedExhausted: true
    });

    await new Promise(r => setTimeout(r, 60));
    assert.strictEqual(autonomyState.state, 'FAULT', 'Autonomy must still lock in FAULT even if bridge cancel fails');
    const disarmWithBridgeDown = mockSerial.writtenPackets.filter(p => p.length >= 4 && p[3] === FUNC_DISARM_NORMAL_DRIVE);
    assert.strictEqual(disarmWithBridgeDown.length, 1, 'Serial disarm must still be sent even if bridge cancel fails');
    console.log('    6B: Drivetrain disarmed and locked in FAULT even with bridge unreachable.');

    // 6C. Test /api/navigation/status when bridge connection refused
    const connRefusedRes = await httpRequest({
      hostname: '127.0.0.1',
      port: PUBLIC_PORT,
      path: '/api/navigation/status',
      method: 'GET'
    });
    assert.ok([500, 502, 503].includes(connRefusedRes.statusCode), `Expected error status when bridge down, got ${connRefusedRes.statusCode}`);
    console.log(`    6C: /api/navigation/status handled ECONNREFUSED cleanly (returned ${connRefusedRes.statusCode}) without crashing.`);

    console.log('\n==============================================================================');
    console.log('ALL COCKPIT RECOVERY FAULT & BRIDGE INTEGRATION TESTS PASSED');
    console.log('==============================================================================');
    process.exit(0);
  } finally {
    try { mockNavBridge.close(); } catch (_) {}
    publicServer.close();
    internalCmdServer.close();
  }
}

runRecoveryFaultSuite().catch(err => {
  console.error('\nTEST FAILURE:', err);
  process.exit(1);
});
