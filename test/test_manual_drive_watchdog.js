// ==============================================================================
// test_manual_drive_watchdog.js
// 1. Unit tests: ManualDriveWatchdog + BatteryTrend (pure modules).
// 2. Integration: real server.js WebSocket path â€”
//    a) keepalive:true stream that stops -> manual target zeroed within ~600 ms
//    b) legacy joystick (no keepalive) keeps its old latching behaviour
//    c) closing the commanding WebSocket zeroes the manual target
// No hardware / serial port is touched (no ESP32 attached in test).
// ==============================================================================
const assert = require('assert');
const path = require('path');

const { ManualDriveWatchdog } = require(path.join(__dirname, '..', 'manual_drive_watchdog'));
const { BatteryTrend } = require(path.join(__dirname, '..', 'battery_trend'));

function unitTests() {
  console.log('Unit 1: ManualDriveWatchdog keepalive expiry fires once');
  let t = 1000;
  const wd = new ManualDriveWatchdog({ timeoutMs: 600, now: () => t });
  const a = {}, b = {};
  wd.noteCommand(a, true);
  t += 500; assert.strictEqual(wd.checkExpired(), false);
  t += 101; assert.strictEqual(wd.checkExpired(), true);
  assert.strictEqual(wd.checkExpired(), false, 'expiry must only fire once');

  console.log('Unit 2: legacy (no keepalive) commands never time out');
  wd.noteCommand(a, false);
  t += 60000; assert.strictEqual(wd.checkExpired(), false);

  console.log('Unit 3: socketClosed only matches the commanding socket');
  wd.noteCommand(a, true);
  assert.strictEqual(wd.socketClosed(b), false);
  assert.strictEqual(wd.socketClosed(a), true);
  assert.strictEqual(wd.socketClosed(a), false);
  t += 10000; assert.strictEqual(wd.checkExpired(), false, 'closing clears the keepalive deadline');

  console.log('Unit 4: BatteryTrend needs >=5 min of history before reporting');
  let bt = new BatteryTrend();
  for (let i = 0; i < 5; i++) bt.add(50, i * 30000);
  assert.strictEqual(bt.snapshot().powerState, 'UNKNOWN');
  assert.strictEqual(bt.snapshot().ratePctPerHr, null);

  console.log('Unit 5: BatteryTrend CHARGING (+12 %/hr)');
  bt = new BatteryTrend();
  for (let i = 0; i <= 20; i++) bt.add(50 + i * 0.1, i * 30000); // +2 % over 10 min
  let s = bt.snapshot();
  assert.strictEqual(s.powerState, 'CHARGING');
  assert(Math.abs(s.ratePctPerHr - 12) < 0.05, `rate ${s.ratePctPerHr}`);
  assert(s.minutesRemaining > 200 && s.minutesRemaining < 260, `eta ${s.minutesRemaining}`);

  console.log('Unit 6: BatteryTrend DISCHARGING (-30 %/hr)');
  bt = new BatteryTrend();
  for (let i = 0; i <= 20; i++) bt.add(60 - i * 0.25, i * 30000);
  s = bt.snapshot();
  assert.strictEqual(s.powerState, 'DISCHARGING');
  assert(Math.abs(s.ratePctPerHr + 30) < 0.05);
  assert(s.minutesRemaining > 100 && s.minutesRemaining < 120, `eta ${s.minutesRemaining}`);

  console.log('Unit 7: BatteryTrend STEADY and FULL');
  bt = new BatteryTrend();
  for (let i = 0; i <= 20; i++) bt.add(70 + (i % 2) * 0.05, i * 30000);
  assert.strictEqual(bt.snapshot().powerState, 'STEADY');
  bt = new BatteryTrend();
  for (let i = 0; i <= 20; i++) bt.add(98, i * 30000);
  assert.strictEqual(bt.snapshot().powerState, 'FULL');

  console.log('Unit 8: BatteryTrend window drops samples older than 20 min');
  bt = new BatteryTrend();
  for (let i = 0; i <= 60; i++) bt.add(50, i * 60000);
  assert(bt.spanMs() <= 20 * 60000);
}

const sleep = (ms) => new Promise(r => setTimeout(r, ms));

async function integrationTests() {
  process.env.PORT = '3880';
  process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
  process.env.ROVER_INTERNAL_CMD_PORT = '3881';
  process.env.ROVER_NAV_BRIDGE_URL = 'http://127.0.0.1:3885';
  process.env.GEEKWORM_BATTERY_ENABLED = 'false';
  process.env.ROVER_OPERATOR_TOKEN = 'ab'.repeat(32);
  const WebSocket = require('ws');
  const srv = require(path.join(__dirname, '..', 'server.js'));
  if (typeof srv.setOdomPollingDisabled === 'function') srv.setOdomPollingDisabled(true);
  await new Promise(r => srv.server.listen(3880, '127.0.0.1', r));

  async function openAuthed() {
    const ws = new WebSocket('ws://127.0.0.1:3880');
    const inbox = [];
    ws.on('message', (d) => { try { inbox.push(JSON.parse(d)); } catch (e) {} });
    await new Promise((res, rej) => { ws.once('open', res); ws.once('error', rej); });
    ws.send(JSON.stringify({ type: 'auth', token: process.env.ROVER_OPERATOR_TOKEN }));
    for (let i = 0; i < 40 && !inbox.find(m => m.type === 'auth_result'); i++) await sleep(25);
    const ar = inbox.find(m => m.type === 'auth_result');
    assert(ar && ar.ok === true, 'operator auth failed');
    return ws;
  }
  const armed = () => srv.setLatestNormalDriveStatus({ armed: true, mode: 3 });

  try {
    console.log('Integration 1: keepalive stream that stops is zeroed by the server');
    armed();
    let ws = await openAuthed();
    for (let i = 0; i < 4; i++) {
      ws.send(JSON.stringify({ type: 'joystick', x: 0, y: 0.6, keepalive: true }));
      await sleep(100);
    }
    let tgt = srv.getManualDriveTarget();
    assert(tgt.targetLinear > 0.01 && tgt.cmdSource === 'BROWSER', `expected motion target, got ${JSON.stringify(tgt)}`);
    await sleep(900);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetLinear, 0, `target not zeroed after stale keepalive: ${JSON.stringify(tgt)}`);
    assert.strictEqual(tgt.cmdSource, 'NONE');

    console.log('Integration 2: legacy joystick (no keepalive) keeps latching behaviour');
    armed();
    ws.send(JSON.stringify({ type: 'joystick', x: 0, y: 0.6 }));
    await sleep(900);
    tgt = srv.getManualDriveTarget();
    assert(tgt.targetLinear > 0.01, `legacy target should persist, got ${JSON.stringify(tgt)}`);

    console.log('Integration 3: closing the commanding socket zeroes the target');
    ws.close();
    await sleep(300);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetLinear, 0, `target not zeroed on socket close: ${JSON.stringify(tgt)}`);
    assert.strictEqual(tgt.cmdSource, 'NONE');

    console.log('Integration 4: a different socket closing does NOT zero an active driver');
    armed();
    ws = await openAuthed();
    const bystander = await openAuthed();
    ws.send(JSON.stringify({ type: 'joystick', x: 0, y: 0.6 }));
    await sleep(150);
    bystander.close();
    await sleep(250);
    tgt = srv.getManualDriveTarget();
    assert(tgt.targetLinear > 0.01, 'bystander disconnect must not affect active driver');
    ws.send(JSON.stringify({ type: 'joystick', x: 0, y: 0, deadman: false }));
    ws.close();
    await sleep(200);

    console.log('Integration 5: /api/battery and /api/system respond');
    const http = require('http');
    const get = (p) => new Promise((res, rej) => http.get({ host: '127.0.0.1', port: 3880, path: p }, (r) => {
      let d = ''; r.on('data', c => { d += c; }); r.on('end', () => res({ status: r.statusCode, json: JSON.parse(d) }));
    }).on('error', rej));
    const bat = await get('/api/battery');
    assert.strictEqual(bat.status, 200);
    assert.strictEqual(bat.json.ok, true);
    const sys = await get('/api/system');
    assert.strictEqual(sys.status, 200);
    assert.strictEqual(sys.json.ok, true);
    assert(sys.json.manualDriveWatchdog && sys.json.manualDriveWatchdog.timeoutMs === 600);

    console.log('Integration 6: "/" serves the HUD, /index.html still serves the legacy cockpit');
    const getText = (p) => new Promise((res, rej) => http.get({ host: '127.0.0.1', port: 3880, path: p }, (r) => {
      let d = ''; r.on('data', c => { d += c; }); r.on('end', () => res({ status: r.statusCode, text: d }));
    }).on('error', rej));
    const root = await getText('/');
    assert.strictEqual(root.status, 200);
    assert(root.text.includes('ROVER 1 GROUND CONTROL'), 'root should be the HUD');
    const legacy = await getText('/index.html');
    assert.strictEqual(legacy.status, 200);
    assert(!legacy.text.includes('ROVER 1 GROUND CONTROL'), '/index.html should remain the legacy cockpit');
  } finally {
    await new Promise(r => srv.server.close(() => r()));
  }
}

(async () => {
  try {
    unitTests();
    await integrationTests();
    console.log('ALL MANUAL DRIVE WATCHDOG / BATTERY TREND TESTS PASSED');
    process.exit(0);
  } catch (err) {
    console.error('TEST FAILED:', err);
    process.exit(1);
  }
})();
