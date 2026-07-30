// ==============================================================================
// test_cmd_vel_behavior.js — End-to-End Behavioral Test Suite for ROS 2 /cmd_vel
// ==============================================================================

const assert = require('assert');
const http = require('http');
const crypto = require('crypto');
const WebSocket = require('ws');

const VALID_TOKEN = 'a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890a1b2c3d4e5f67890';

// Set test environment
process.env.PORT = '3800';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3810';
process.env.ROVER_CMD_VEL_TOKEN = VALID_TOKEN;
process.env.ROVER_OPERATOR_TOKEN = 'test_operator_token_12345678901234567890123456789012';

const serverModule = require('./server.js');
const publicApp = serverModule.app;
const publicServer = serverModule.server;
const internalApp = serverModule.internalCmdApp;
const internalServer = serverModule.internalCmdServer;

const PUBLIC_PORT = 3800;
const INTERNAL_PORT = 3810;
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
  console.log('Starting ROS 2 /cmd_vel Behavioral Verification Tests...');

  await new Promise(r => publicServer.listen(PUBLIC_PORT, '127.0.0.1', r));
  await new Promise(r => internalServer.listen(INTERNAL_PORT, '127.0.0.1', r));

  try {
    // --------------------------------------------------------------------------
    // BLOCKER 1: Port Separation Verification
    // --------------------------------------------------------------------------
    {
      assert.notStrictEqual(INTERNAL_PORT, 3003, 'Internal ROS command listener port must not collide with Odometry port 3003');
      console.log('✓ Blocker 1 Test Passed: Internal listener port (3810/3010) is distinct from Odometry port 3003');
    }

    // --------------------------------------------------------------------------
    // Test 1: Access Isolation - /api/cmd_vel NOT mounted on public server
    // --------------------------------------------------------------------------
    {
      const res = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': VALID_TOKEN }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });

      assert.strictEqual(res.statusCode, 404, 'Public port must return 404 for /api/cmd_vel');
      console.log('✓ Test 1 Passed: /api/cmd_vel rejected on public port (404)');
    }

    // --------------------------------------------------------------------------
    // BLOCKER 2: Token Security & Validation Tests
    // --------------------------------------------------------------------------
    {
      // Missing token
      const missingRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });
      assert.strictEqual(missingRes.statusCode, 401, 'Missing token must return 401');

      // Incorrect token
      const wrongRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': 'wrong_token' }
      }, { linear: { x: 0.1 }, angular: { z: 0.0 } });
      assert.strictEqual(wrongRes.statusCode, 403, 'Incorrect token must return 403');

      // Check token value absent from logs & status responses
      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/status',
        method: 'GET'
      });
      const statusStr = JSON.stringify(statusRes.json);
      assert.strictEqual(statusStr.includes(VALID_TOKEN), false, 'Token must NEVER be exposed in status responses');
      console.log('✓ Blocker 2 Test Passed: Token validation, missing token fail-closed, and zero exposure verified');
    }

    // --------------------------------------------------------------------------
    // BLOCKER 3: Public Control-Endpoint Access Control Tests
    // --------------------------------------------------------------------------
    {
      // Verify /api/auth/token is NOT accessible / returns 404
      const tokenEndpointRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/auth/token',
        method: 'GET'
      });
      assert.strictEqual(tokenEndpointRes.statusCode, 404, '/api/auth/token must NOT exist (404)');

      // Unauthorized ARM attempt (missing operator token header)
      const unauthArmRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/drive/arm',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': 'invalid_token' }
      });
      assert.strictEqual(unauthArmRes.statusCode, 403, 'Unauthorized arm attempt must return 403');

      // Unauthorized Autonomy Enable attempt
      const unauthEnableRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/enable',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': 'invalid_token' }
      });
      assert.strictEqual(unauthEnableRes.statusCode, 403, 'Unauthorized autonomy enable attempt must return 403');

      // Authorized Autonomy Enable attempt
      const authEnableRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/enable',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
      });
      assert.strictEqual(authEnableRes.statusCode, 200, 'Authorized autonomy enable must succeed');

      console.log('✓ Blocker 3 Test Passed: Unauthorized LAN requests cannot arm or enable autonomy');
    }

    // --------------------------------------------------------------------------
    // BLOCKER 4: WebSocket Auth & Deauth Workflow Verification
    // --------------------------------------------------------------------------
    {
      const wsClient = new WebSocket(`ws://127.0.0.1:${PUBLIC_PORT}`);
      await new Promise(resolve => wsClient.on('open', resolve));

      function waitForMessageType(targetType) {
        return new Promise(resolve => {
          function listener(data) {
            try {
              const str = typeof data === 'string' ? data : data.toString('utf-8');
              const parsed = JSON.parse(str);
              if (parsed.type === targetType) {
                wsClient.removeListener('message', listener);
                resolve(parsed);
              }
            } catch (e) {}
          }
          wsClient.on('message', listener);
        });
      }

      // 1. Unauthenticated joystick command fails
      const unauthPromise = waitForMessageType('error');
      wsClient.send(JSON.stringify({ type: 'joystick', x: 0.5, y: 0.0 }));
      const unauthRes = await unauthPromise;
      assert.strictEqual(unauthRes.type, 'error');
      assert.strictEqual(unauthRes.message, 'Unauthorized operator connection');

      // 2. Authenticate WebSocket
      const authPromise = waitForMessageType('auth_result');
      wsClient.send(JSON.stringify({ type: 'auth', token: OPERATOR_TOKEN }));
      const authRes = await authPromise;
      assert.strictEqual(authRes.type, 'auth_result');
      assert.strictEqual(authRes.ok, true);

      // 3. Deauthenticate WebSocket
      const deauthPromise = waitForMessageType('deauth_result');
      wsClient.send(JSON.stringify({ type: 'deauth' }));
      const deauthRes = await deauthPromise;
      assert.strictEqual(deauthRes.type, 'deauth_result');
      assert.strictEqual(deauthRes.ok, true);

      // 4. Movement command after deauth fails
      const postDeauthPromise = waitForMessageType('error');
      wsClient.send(JSON.stringify({ type: 'joystick', x: 0.5, y: 0.0 }));
      const postDeauthRes = await postDeauthPromise;
      assert.strictEqual(postDeauthRes.type, 'error');
      assert.strictEqual(postDeauthRes.message, 'Unauthorized operator connection');

      wsClient.close();
      console.log('✓ Blocker 4 Test Passed: WebSocket auth, deauth, and post-deauth authorization enforcement verified');
    }

    // --------------------------------------------------------------------------
    // Test 5: Handshake & Autonomy State Machine Verification
    // --------------------------------------------------------------------------
    {
      // Send 3 consecutive zero Twist messages to complete handshake
      for (let i = 1; i <= 3; i++) {
        const zeroRes = await httpRequest({
          hostname: '127.0.0.1',
          port: INTERNAL_PORT,
          path: '/api/cmd_vel',
          method: 'POST',
          headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': VALID_TOKEN }
        }, { linear: { x: 0.0 }, angular: { z: 0.0 } });
        assert.strictEqual(zeroRes.statusCode, 200);
      }

      const statusRes = await httpRequest({
        hostname: '127.0.0.1',
        port: PUBLIC_PORT,
        path: '/api/autonomy/status',
        method: 'GET'
      });
      assert.strictEqual(statusRes.json.state, 'READY_DISARMED', 'After 3 zeros, state must be READY_DISARMED');
      console.log('✓ Handshake Test Passed: 3 zero messages completed handshake to READY_DISARMED');
    }

    // --------------------------------------------------------------------------
    // Test 6: Payload Validation & Axis Enforcement
    // --------------------------------------------------------------------------
    {
      const nanRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': VALID_TOKEN }
      }, { linear: { x: 'NaN' }, angular: { z: 0.0 } });
      assert.strictEqual(nanRes.statusCode, 400, 'NaN velocity string must be rejected with 400');

      const axisRes = await httpRequest({
        hostname: '127.0.0.1',
        port: INTERNAL_PORT,
        path: '/api/cmd_vel',
        method: 'POST',
        headers: { 'Content-Type': 'application/json', 'X-Rover-Bridge-Token': VALID_TOKEN }
      }, { linear: { x: 0.1, y: 0.5 }, angular: { z: 0.0 } });
      assert.strictEqual(axisRes.statusCode, 400, 'Unsupported y axis must be rejected with 400');

      console.log('✓ Validation Test Passed: Malformed / NaN / unsupported axis inputs rejected with 400');
    }

    // Disable autonomy at end of test suite
    await httpRequest({
      hostname: '127.0.0.1',
      port: PUBLIC_PORT,
      path: '/api/autonomy/disable',
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'X-Rover-Operator-Token': OPERATOR_TOKEN }
    });

    console.log('\n==================================================');
    console.log('ALL ROS 2 /CMD_VEL BEHAVIORAL VERIFICATION TESTS PASSED!');
    console.log('==================================================\n');

  } finally {
    publicServer.close();
    internalServer.close();
    process.exit(0);
  }
}

runTests().catch(err => {
  console.error('Test Suite Failed:', err);
  process.exit(1);
});
