// ==============================================================================
// test_cockpit_terminal_disarm.js
// Regression test: Verifies that Cockpit terminal navigation handling
// (SUCCEEDED, CANCELLED, ABORTED, FAILED, STOPPED) zeroes commands and disarms
// from both READY_ARMED and ACTIVE states to Mode 0 disarmed.
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

  const mockNav2Server = http.createServer((req, res) => {
    if (req.url === '/api/nav/status' && req.method === 'GET') {
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify(bridgeNavStatus));
    } else if (req.url === '/api/nav/cancel' && req.method === 'POST') {
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
    // TEST 1: ACTIVE -> SUCCEEDED -> disarmed Mode 0
    // --------------------------------------------------------------------------
    console.log('[TEST 1] ACTIVE -> SUCCEEDED transition zeroes commands and disarms to Mode 0...');
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

      // Poll navigation status
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/navigation/status',
        method: 'GET'
      });

      assert.strictEqual(res.statusCode, 200);
      assert.strictEqual(res.json.status, 'SUCCEEDED');

      // Verify Cockpit autonomyState transitioned to READY_DISARMED
      assert.strictEqual(autonomyState.state, 'READY_DISARMED', 'State must become READY_DISARMED');
      assert.strictEqual(autonomyState.active, false, 'Autonomy must not be active');
      assert.strictEqual(autonomyState.enabled, false, 'Autonomy must not be enabled');

      // Verify drive status endpoint reports disarmed, mode 0, cmdSource NONE
      const driveRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveRes.statusCode, 200);
      assert.strictEqual(driveRes.json.status.armed, false, 'Drive status must report armed=false');
      assert.strictEqual(driveRes.json.status.mode, 0, 'Drive status must report mode=0');
      assert.strictEqual(driveRes.json.cmdSource, 'NONE', 'cmdSource must be NONE');
      assert.strictEqual(driveRes.json.status.reqLinear, 0.0, 'reqLinear must be 0.0');
      assert.strictEqual(driveRes.json.status.limLinear, 0.0, 'limLinear must be 0.0');

      // Verify hardware disarm packet and zero motion packet sent to serial
      const disarmPackets = findPackets(FUNC_DISARM_NORMAL_DRIVE);
      const motionPackets = findPackets(FUNC_MOTION);
      assert(disarmPackets.length > 0, 'FUNC_DISARM_NORMAL_DRIVE packet must be sent');
      assert(motionPackets.length > 0, 'FUNC_MOTION (zero) packet must be sent');

      console.log('  ✓ Verified ACTIVE -> SUCCEEDED disarms to Mode 0, zeroes velocity, sets cmdSource=NONE');
    }

    // --------------------------------------------------------------------------
    // TEST 2: READY_ARMED -> SUCCEEDED -> disarmed Mode 0
    // --------------------------------------------------------------------------
    console.log('[TEST 2] READY_ARMED -> SUCCEEDED transition zeroes commands and disarms...');
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
      assert.strictEqual(autonomyState.active, false);

      const driveRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveRes.json.status.armed, false);
      assert.strictEqual(driveRes.json.status.mode, 0);
      assert.strictEqual(driveRes.json.cmdSource, 'NONE');

      const disarmPackets = findPackets(FUNC_DISARM_NORMAL_DRIVE);
      assert(disarmPackets.length > 0, 'FUNC_DISARM_NORMAL_DRIVE packet must be sent');
      console.log('  ✓ Verified READY_ARMED -> SUCCEEDED disarms to Mode 0 and zeroes commands');
    }

    // --------------------------------------------------------------------------
    // TEST 3: ACTIVE -> CANCELLED / ABORTED / STOPPED terminal handling
    // --------------------------------------------------------------------------
    console.log('[TEST 3] ACTIVE -> CANCELLED, ABORTED, and STOPPED terminal states disarm...');
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
      assert.strictEqual(autonomyState.active, false);

      const driveRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/status',
        method: 'GET'
      });
      assert.strictEqual(driveRes.json.status.armed, false);
      assert.strictEqual(driveRes.json.status.mode, 0);
      assert.strictEqual(driveRes.json.cmdSource, 'NONE');

      const disarmPackets = findPackets(FUNC_DISARM_NORMAL_DRIVE);
      assert(disarmPackets.length > 0, `FUNC_DISARM_NORMAL_DRIVE packet must be sent for ${termStatus}`);
      console.log(`  ✓ Verified terminal status ${termStatus} disarms to Mode 0`);
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
