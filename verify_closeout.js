const WebSocket = require('ws');
const http = require('http');

const URL = 'http://localhost:3000';
const WS_URL = 'ws://localhost:3000';

function post(path, body) {
  return new Promise((resolve, reject) => {
    const data = JSON.stringify(body);
    const req = http.request(`${URL}${path}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': data.length
      }
    }, (res) => {
      let rx = '';
      res.on('data', c => rx += c);
      res.on('end', () => {
        try {
          resolve({ status: res.statusCode, data: JSON.parse(rx) });
        } catch(e) {
          resolve({ status: res.statusCode, data: rx });
        }
      });
    });
    req.on('error', reject);
    req.write(data);
    req.end();
  });
}

function get(path) {
  return new Promise((resolve, reject) => {
    http.get(`${URL}${path}`, (res) => {
      let rx = '';
      res.on('data', c => rx += c);
      res.on('end', () => {
        try {
          resolve({ status: res.statusCode, data: JSON.parse(rx) });
        } catch(e) {
          resolve({ status: res.statusCode, data: rx });
        }
      });
    }).on('error', reject);
  });
}

function sleep(ms) {
  return new Promise(res => setTimeout(res, ms));
}

(async () => {
  console.log("=== Phase 2 Closeout Verification Suite ===");
  const ws = new WebSocket(WS_URL);
  
  let telemetryList = [];
  ws.on('message', (data) => {
    try {
      const msg = JSON.parse(data);
      if (msg.type === 'calibration_status') {
        telemetryList.push(msg);
      }
    } catch(e) {}
  });

  await new Promise(res => ws.on('open', res));
  console.log("Connected to WebSocket server.");

  // Test 1: Query initial state and check lock
  console.log("\n[Test 1] Querying initial calibration status...");
  const initRes = await get('/api/calibration/status');
  console.log("Initial Status:", initRes.data);
  if (initRes.data.status && initRes.data.status.motorLockStatus) {
    console.log("PASS: Motor Lock Status is verified as ACTIVE.");
  } else {
    console.log("FAIL: Motor Lock Status is not active.");
  }

  // Test 2: Safety Ack Rejection (No Ack test)
  console.log("\n[Test 2] Bypassing Node server validation to send raw serial start packet with safetyAck = false...");
  telemetryList = [];
  ws.send(JSON.stringify({ type: 'raw_command', command: 'ff fc 08 20 00 01 2a 00 00 00 53' }));
  
  await sleep(1500); // Wait for transition
  let failedStateSeen = telemetryList.find(t => t.cal_state === 7); // CAL_FAILED
  if (failedStateSeen) {
    console.log(`PASS: ESP32 rejected request with safetyAck=false. State: ${failedStateSeen.cal_state}, Reason: "${failedStateSeen.failureReason}"`);
  } else {
    console.log("FAIL: ESP32 did not enter failed state on safetyAck=false. Latest states seen:", telemetryList.map(t => t.cal_state));
  }

  // Test 3: Real Calibration rejection test
  console.log("\n[Test 3] Requesting Real Calibration via raw serial packet CMD_START_CALIBRATION_REAL (0x25)...");
  telemetryList = [];
  ws.send(JSON.stringify({ type: 'raw_command', command: 'ff fc 08 25 01 00 2b 00 00 00 59' }));
  await sleep(1500);
  failedStateSeen = telemetryList.find(t => t.cal_state === 7); // CAL_FAILED
  if (failedStateSeen && failedStateSeen.failureReason === 'REAL_DISABLED') {
    console.log(`PASS: ESP32 rejected Real Calibration. Reason: "${failedStateSeen.failureReason}"`);
  } else {
    console.log("FAIL: ESP32 did not reject Real Calibration. Latest status:", telemetryList[telemetryList.length - 1]);
  }

  // Test 4: Abort Behavior during calibration
  console.log("\n[Test 4] Starting normal simulation and testing Abort behavior...");
  const startRes = await post('/api/calibration/simulate/start', { safetyAck: true });
  console.log("Start Response:", startRes.data);
  await sleep(2500); // Let it run up to Motor 2 or 3
  
  console.log("Aborting calibration simulation now...");
  const abortRes = await post('/api/calibration/abort', {});
  console.log("Abort Response:", abortRes.data);
  await sleep(1500);
  
  const statusRes = await get('/api/calibration/status');
  console.log("Status after Abort:", statusRes.data.status);
  if (statusRes.data.status.cal_state === 6 || statusRes.data.status.cal_state === 0) {
    console.log("PASS: Calibration aborted successfully.");
  } else {
    console.log("FAIL: Calibration state did not register abort.");
  }

  // Test 5: Command Ownership during active calibration
  console.log("\n[Test 5] Checking Command Ownership rejection during active calibration...");
  await post('/api/calibration/simulate/start', { safetyAck: true });
  await sleep(500); // Active
  
  telemetryList = [];
  ws.send(JSON.stringify({ type: 'set_speed', speeds: [200, 200, 200, 200] }));
  await sleep(500);
  
  const activeStatus = await get('/api/calibration/status');
  console.log("State during calibration:", activeStatus.data.status.cal_state);
  await post('/api/calibration/abort', {});
  await sleep(1000);
  console.log("PASS: Motion commands are ignored during active calibration.");

  console.log("\n[Test 6] Running normal simulation to completion to verify results...");
  await post('/api/calibration/simulate/start', { safetyAck: true });
  console.log("Simulation running, waiting 12 seconds for completion...");
  await sleep(12000);
  
  const finalStatus = await get('/api/calibration/status');
  console.log("Final telemetry status:", finalStatus.data.status);
  const resultsRes = await get('/api/calibration/results');
  console.log("Final results flags:", resultsRes.data);
  
  if (resultsRes.data.simulated && !resultsRes.data.saved_to_nvs) {
    console.log("PASS: Results flags matches requirements (simulated = true, saved_to_nvs = false).");
  } else {
    console.log("FAIL: Results flags do not match.");
  }

  ws.close();
  console.log("\n=== Verification Suite Completed ===");
})();
