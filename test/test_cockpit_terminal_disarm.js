// ==============================================================================
// test_cockpit_terminal_disarm.js
// Regression test: Verifies that Cockpit terminal navigation handling
// (SUCCEEDED, CANCELLED, ABORTED, FAILED, STOPPED) sends zero and disarm packets
// without fabricating confirmation: /api/drive/status remains armed until
// real simulated ESP32 telemetry arrives.
// ==============================================================================

const assert = require('assert');
const http = require('http');
const path = require('path');
const fs = require('fs');

process.env.PORT = '3860';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3861';
process.env.ROVER_NAV_BRIDGE_URL = 'http://127.0.0.1:3865';

const serverPath = fs.existsSync(path.join(__dirname, 'server.js'))
  ? './server.js'
  : (fs.existsSync(path.join(__dirname, '../server.js')) ? '../server.js' : './server.js');

const serverModule = require(serverPath);
const {
  app: publicApp,
  server: publicServer,
  internalCmdServer,
  autonomyState,
  injectNormalDriveStatus,
  setSerialPort,
  setOdomPollingDisabled
} = serverModule;

const PUBLIC_PORT = 3860;
const NAV_BRIDGE_PORT = 3865;

const FUNC_MOTION = 0x12;
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

async function runTerminalDisarmSuite() {
  console.log('==============================================================================');
  console.log('TEST SUITE: COCKPIT TERMINAL NAVIGATION ZERO AND DISARM INVARIANTS');
  console.log('==============================================================================\n');

  if (setOdomPollingDisabled) {
    setOdomPollingDisabled(true);
  }

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

  function findPackets(funcId) {
    return writtenPackets.filter(pkt =>
      pkt.length >= 4 && pkt[0] === 0xFF && pkt[1] === 0xFC && pkt[3] === funcId
    );
  }

  let bridgeNavStatus = { ok: true, status: 'EXECUTING', goal_id: 'goal_test_001', distance_remaining_m: 0.5 };
  let navCancelCalls = [];

  const mockNav2Server = http.createServer((req, res) => {
    if (req.url === '/api/nav/status' && req.method === 'GET') {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(bridgeNavStatus));
    } else if (req.url === '/api/nav/cancel' && req.method === 'POST') {
      navCancelCalls.push(Date.now());
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: true, message: 'Nav2 cancel called' }));
    } else {
      res.writeHead(404);
      res.end();
    }
  });

  await new Promise(r => mockNav2Server.listen(NAV_BRIDGE_PORT, '127.0.0.1', r));
  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));

  try {
    // --------------------------------------------------------------------------
    // TEST 1: ACTIVE -> SUCCEEDED sends disarm request; telemetry remains armed
    // until simulated ESP32 packet confirms disarm.
    // --------------------------------------------------------------------------
    console.log('[TEST 1] ACTIVE -> SUCCEEDED sends zero/disarm packet without fabricating telemetry...');
    {
      writtenPackets.length = 0;
      autonomyState.enabled = true;
      autonomyState.state = 'ACTIVE';
      autonomyState.active = true;

      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        bootCount: 1,
        reqLinear: 0.20,
        reqAngular: 0.0,
        limLinear: 0.20,
        limAngular: 0.0
      });

      bridgeNavStatus = { ok: true, status: 'SUCCEEDED', goal_id: 'goal_test_001', distance_remaining_m: 0.0 };

      // Poll navigation status (triggers Cockpit terminal handling)
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/status',
        method: 'GET'
      });

      assert.strictEqual(res.statusCode, 200);
      assert.strictEqual(res.json.status, 'SUCCEEDED');
      assert.strictEqual(navCancelCalls.length, 0, 'triggerNav2Cancel MUST NOT be called when status is SUCCEEDED');

      // 1. Cockpit autonomy state transitioned to READY_DISARMED
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'Autonomy state must become READY_DISARMED');
      assert.strictEqual(autonomyState.active, false, 'Autonomy must not be active');
      assert.strictEqual(autonomyState.enabled, false, 'Autonomy must not be enabled');

      // 2. Hardware packets were transmitted to serial
      const disarmPackets = findPackets(FUNC_DISARM_NORMAL_DRIVE);
      const motionPackets = findPackets(FUNC_MOTION);
      assert(disarmPackets.length > 0, 'FUNC_DISARM_NORMAL_DRIVE packet must be sent to serial');
      assert(motionPackets.length > 0, 'FUNC_MOTION (zero) packet must be sent to serial');

      // 3. CRUCIAL: Immediately after sending request, drive status MUST STILL BE ARMED
      // (Cockpit must NOT fabricate armed=false; it must report actual unconfirmed ESP32 state)
      const driveImmediate = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveImmediate.json.status.armed, true, 'Status must remain armed before ESP32 confirmation');
      assert.strictEqual(driveImmediate.json.status.mode, 3, 'Status must remain mode 3 before ESP32 confirmation');
      assert.strictEqual(driveImmediate.json.cmdSource, 'NONE', 'cmdSource must be released to NONE');

      // 4. Now simulate arrival of genuine ESP32 confirmation telemetry packet
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        bootCount: 1,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      // 5. Subsequent drive status query confirms Mode 0 disarmed
      const driveConfirmed = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveConfirmed.json.status.armed, false, 'Status must report armed=false after ESP32 packet');
      assert.strictEqual(driveConfirmed.json.status.mode, 0, 'Status must report mode=0 after ESP32 packet');
      assert.strictEqual(driveConfirmed.json.cmdSource, 'NONE');
      assert.strictEqual(driveConfirmed.json.status.reqLinear, 0.0);
      assert.strictEqual(driveConfirmed.json.status.limLinear, 0.0);

      console.log('  ✓ Verified status remains armed immediately after request and only disarms after ESP32 confirmation');
    }

    // --------------------------------------------------------------------------
    // TEST 2: READY_ARMED -> SUCCEEDED transition
    // --------------------------------------------------------------------------
    console.log('[TEST 2] READY_ARMED -> SUCCEEDED transition requires ESP32 telemetry confirmation...');
    {
      writtenPackets.length = 0;
      autonomyState.enabled = true;
      autonomyState.state = 'READY_ARMED';
      autonomyState.active = false;

      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        bootCount: 1,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      bridgeNavStatus = { ok: true, status: 'SUCCEEDED', goal_id: 'goal_test_002', distance_remaining_m: 0.0 };

      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/status',
        method: 'GET'
      });

      assert.strictEqual(res.statusCode, 200);
      assert.strictEqual(autonomyState.state, 'READY_DISARMED');

      // Unconfirmed: still armed
      const driveImmediate = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveImmediate.json.status.armed, true);

      // Confirm via simulated ESP32 packet
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        bootCount: 1,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      const driveConfirmed = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveConfirmed.json.status.armed, false);
      assert.strictEqual(driveConfirmed.json.status.mode, 0);
      console.log('  ✓ Verified READY_ARMED requires ESP32 confirmation before reporting disarmed');
    }

    // --------------------------------------------------------------------------
    // TEST 3: Terminal states (CANCELLED, ABORTED, FAILED, STOPPED_OBSTACLE)
    // --------------------------------------------------------------------------
    console.log('[TEST 3] CANCELLED, ABORTED, and STOPPED terminal states disarm via ESP32 telemetry...');
    for (const termStatus of ['CANCELLED', 'ABORTED', 'FAILED', 'STOPPED_OBSTACLE']) {
      writtenPackets.length = 0;
      autonomyState.enabled = true;
      autonomyState.state = 'ACTIVE';
      autonomyState.active = true;

      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        bootCount: 1,
        reqLinear: 0.15,
        reqAngular: 0.0,
        limLinear: 0.15,
        limAngular: 0.0
      });

      bridgeNavStatus = { ok: true, status: termStatus, goal_id: 'goal_test_term', distance_remaining_m: 0.2 };

      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/status',
        method: 'GET'
      });

      assert.strictEqual(res.statusCode, 200);
      assert.strictEqual(autonomyState.state, 'READY_DISARMED');

      // Still armed before ESP32 confirms
      const driveImmediate = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveImmediate.json.status.armed, true);

      // Confirm via simulated ESP32 packet
      injectNormalDriveStatus({
        armed: false,
        mode: 0,
        bootCount: 1,
        reqLinear: 0.0,
        reqAngular: 0.0,
        limLinear: 0.0,
        limAngular: 0.0
      });

      const driveConfirmed = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveConfirmed.json.status.armed, false);
      assert.strictEqual(driveConfirmed.json.status.mode, 0);
      console.log(`  ✓ Verified terminal status ${termStatus} disarms upon confirmation`);
    }

    // --------------------------------------------------------------------------
    // TEST 4: Missing ESP32 confirmation leaves status armed
    // --------------------------------------------------------------------------
    console.log('[TEST 4] Missing ESP32 confirmation strictly leaves status armed...');
    {
      autonomyState.enabled = true;
      autonomyState.state = 'ACTIVE';
      autonomyState.active = true;

      injectNormalDriveStatus({
        armed: true,
        mode: 3,
        bootCount: 1,
        reqLinear: 0.20,
        reqAngular: 0.0,
        limLinear: 0.20,
        limAngular: 0.0
      });

      bridgeNavStatus = { ok: true, status: 'SUCCEEDED', goal_id: 'goal_unconfirmed', distance_remaining_m: 0.0 };

      await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/status',
        method: 'GET'
      });

      // No ESP32 confirmation injected
      const driveUnconfirmed = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveUnconfirmed.json.status.armed, true, 'Without ESP32 telemetry packet, status remains armed');
      assert.strictEqual(driveUnconfirmed.json.status.mode, 3, 'Without ESP32 telemetry packet, mode remains 3');
      console.log('  ✓ Verified drive status strictly retains actual armed telemetry when unconfirmed');
    }

    console.log('\n==============================================================================');
    console.log('ALL TERMINAL NAVIGATION DISARM INVARIANTS VERIFIED SUCCESSFULLY');
    console.log('==============================================================================');
  } finally {
    await new Promise(r => mockNav2Server.close(r));
    await new Promise(r => publicServer.close(r));
    process.exit(0);
  }
}

runTerminalDisarmSuite().catch(err => {
  console.error('Test Suite Failed:', err);
  process.exit(1);
});
