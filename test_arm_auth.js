const assert = require('assert');
const http = require('http');
const { app, server } = require('./server');

console.log('=== Running Arm Authorization & Feedback Tests ===');

const PORT = 3099;
let testServer;

function makePostRequest(path, headers = {}) {
  return new Promise((resolve, reject) => {
    const req = http.request({
      hostname: '127.0.0.1', // Use non-loopback IP simulation or pass custom header
      port: PORT,
      path: path,
      method: 'POST',
      headers: {
        'X-Forwarded-For': '192.168.1.100', // Simulate remote non-loopback client
        ...headers
      }
    }, (res) => {
      let body = '';
      res.on('data', chunk => body += chunk);
      res.on('end', () => {
        try {
          resolve({ status: res.statusCode, data: JSON.parse(body) });
        } catch (e) {
          resolve({ status: res.statusCode, body });
        }
      });
    });
    req.on('error', reject);
    req.end();
  });
}

async function runTests() {
  testServer = app.listen(PORT, async () => {
    try {
      // Test 1: Missing Token -> HTTP 401 & Actionable Message
      console.log('Test 1: Arm request with missing operator token...');
      const res401 = await makePostRequest('/api/drive/arm');
      assert.strictEqual(res401.status, 401);
      assert.strictEqual(res401.data.ok, false);
      assert.ok(res401.data.error.includes('Operator token is missing'), 'Error message must explain token is missing');
      console.log('  -> PASS: HTTP 401 returned with clear actionable missing token message');

      // Test 2: Invalid Token -> HTTP 403 & Actionable Message
      console.log('Test 2: Arm request with invalid operator token...');
      const res403 = await makePostRequest('/api/drive/arm', { 'X-Rover-Operator-Token': 'wrong_invalid_token_123' });
      assert.strictEqual(res403.status, 403);
      assert.strictEqual(res403.data.ok, false);
      assert.ok(res403.data.error.includes('Operator token is invalid'), 'Error message must explain token is invalid');
      assert.ok(!JSON.stringify(res403).includes('wrong_invalid_token_123'), 'Token must never be leaked in error responses');
      console.log('  -> PASS: HTTP 403 returned with clear actionable invalid token message');

      // Test 3: Valid Token -> HTTP 200
      console.log('Test 3: Arm request with valid operator token...');
      const validToken = process.env.ROVER_OPERATOR_TOKEN || 'c34a2663959c5d0ef40d463d11b22e11ec9a37e1b5900ec41c0ee1076b1f24d7';
      const res200 = await makePostRequest('/api/drive/arm', { 'X-Rover-Operator-Token': validToken });
      assert.strictEqual(res200.status, 200);
      assert.strictEqual(res200.data.ok, true);
      console.log('  -> PASS: HTTP 200 returned for valid token');

      console.log('All Arm Authorization & Feedback Tests PASSED successfully.');
      testServer.close();
      process.exit(0);
    } catch (err) {
      console.error('Test FAILED:', err);
      if (testServer) testServer.close();
      process.exit(1);
    }
  });
}

runTests();
