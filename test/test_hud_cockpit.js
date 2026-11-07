// ==============================================================================
// test_hud_cockpit.js
// jsdom tests for the Rover 2-style HUD cockpit (public/hud*.{html,js}).
// Verifies safety wiring only — no network, no hardware:
//   - no hard-coded operator token, no direct ROS /cmd_vel publishing
//   - auth only when a token is stored; token entry sends `auth`
//   - E-STOP fans out to joystick-zero + disarm + emergency_stop + HTTP disarm/cancel
//   - drive frames carry keepalive:true, respect throttle, need ARM + auth, end with zeros
//   - ARM sends the X-Rover-Operator-Token header
//   - map transform round-trip, LiDAR frame transform, mission parser
//   - mobile page loads and its E-STOP works
// ==============================================================================
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const { JSDOM, VirtualConsole } = require('jsdom');
const {
  FRONT_STOP_THRESHOLD_MM,
  FRONT_CRAWL_THRESHOLD_MM,
  DEFAULT_CRAWL_LINEAR_MPS,
  evaluateFrontSafetyZone,
  applyFrontSafetyZoneToVelocity
} = require('../front_safety_zone');

const PUB = path.join(__dirname, '..', 'public');
const read = (f) => fs.readFileSync(path.join(PUB, f), 'utf8');
const TOKEN = 'test-operator-token-123';
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

function staticChecks() {
  console.log('Static 1: HUD files contain no hard-coded 64-hex operator token');
  for (const f of ['hud.html', 'hud-core.js', 'hud-desktop.js', 'hud-mobile.html', 'hud-mobile.js']) {
    const txt = read(f);
    assert(!/[0-9a-f]{64}/i.test(txt), `${f} contains a 64-hex token literal`);
  }
  console.log('Static 2: HUD never publishes ROS topics directly (no rosbridge advertise/publish)');
  for (const f of ['hud-core.js', 'hud-desktop.js', 'hud-mobile.js']) {
    const txt = read(f);
    assert(!/op:\s*['"](advertise|publish)['"]/.test(txt), `${f} publishes to rosbridge`);
    assert(!/:9090/.test(txt), `${f} talks to rosbridge port 9090`);
  }
  console.log('Static 3: hud.css carries the Rover 2 palette verbatim');
  const css = read('hud.css');
  ['#080c12', '#00ffcc', '.cockpit-grid', '.center-stage.mode-split', '.map-bottom-dock'].forEach(tok => assert(css.includes(tok), `hud.css missing ${tok}`));
}

function makeDom(file, { token, extraScripts }) {
  const html = read(file).replace(/<script src="[^"]+"><\/script>/g, '');
  const vc = new VirtualConsole();
  vc.on('error', (e) => { if (!/getContext|Not implemented/.test(String(e && e.message || e))) console.error('[dom error]', e); });
  vc.on('jsdomError', (e) => { if (!/getContext|Not implemented/.test(String(e && e.message || e))) console.error('[jsdomError]', e.message); });
  const dom = new JSDOM(html, {
    url: `http://10.0.0.246:3000/${file}?mode=desktop`,
    runScripts: 'outside-only',
    pretendToBeVisual: true,
    virtualConsole: vc
  });
  const w = dom.window;
  const sent = [];
  const fetches = [];
  class MockWS {
    constructor(url) {
      this.url = url;
      this.readyState = 0;
      MockWS.instances.push(this);
      setTimeout(() => { this.readyState = 1; if (this.onopen) this.onopen(); }, 0);
    }
    send(d) { sent.push(JSON.parse(d)); }
    close() { this.readyState = 3; if (this.onclose) this.onclose(); }
    serverSend(obj) { if (this.onmessage) this.onmessage({ data: JSON.stringify(obj) }); }
  }
  MockWS.instances = [];
  w.WebSocket = MockWS;
  w.fetch = async (url, opts = {}) => {
    fetches.push({ url, opts });
    const u = String(url);
    let body = { ok: true };
    if (u.startsWith('/api/navigation/home')) body = { ok: true, home: { x: 1.0, y: 2.0, yaw_rad: 0.5, yaw_deg: 28.6 } };
    else if (u.startsWith('/api/localization/status')) body = { ok: true, localized: true, state: 'LOCALIZED', x: 0.5, y: -0.25, yawDeg: 90 };
    else if (u.startsWith('/api/navigation/plan')) body = { ok: true, waypoints: [{ x: 0.5, y: -0.25 }, { x: 1, y: 1 }], cumulative_length_m: 1.31 };
    else if (u.startsWith('/api/lidar/scan')) body = { error: 'No scan data available' };
    return { ok: !u.startsWith('/api/lidar/scan'), status: u.startsWith('/api/lidar/scan') ? 404 : 200, json: async () => body };
  };
  w.confirm = () => true;
  w.alert = () => {};
  if (token) w.localStorage.setItem('rover_operator_token', token);
  w.eval(read('hud-core.js'));
  (extraScripts || []).forEach(s => w.eval(read(s)));
  return { dom, w, sent, fetches, MockWS, H: w.RoverHUD };
}

async function desktopTests() {
  console.log('Desktop 1: no auth message without a stored token');
  let env = makeDom('hud.html', { extraScripts: ['hud-desktop.js'] });
  await sleep(30);
  assert.strictEqual(env.sent.filter(m => m.type === 'auth').length, 0, 'must not auth without token');

  console.log('Desktop 2: saving a token sends auth over the WebSocket');
  env.w.document.getElementById('token-input').value = TOKEN;
  env.w.document.getElementById('btn-token-save').click();
  const auth = env.sent.find(m => m.type === 'auth');
  assert(auth && auth.token === TOKEN, 'auth message with token expected');
  assert.strictEqual(env.w.localStorage.getItem('rover_operator_token'), TOKEN);
  env.dom.window.close();

  env = makeDom('hud.html', { token: TOKEN, extraScripts: ['hud-desktop.js'] });
  const { w, sent, fetches, H } = env;
  await sleep(30);
  const ws = env.MockWS.instances[0];
  assert(sent.find(m => m.type === 'auth' && m.token === TOKEN), 'stored token should auth on connect');
  ws.serverSend({ type: 'auth_result', ok: true });
  assert.strictEqual(H.state.authOk, true);

  console.log('Desktop 3: driving while DISARMED sends nothing');
  sent.length = 0;
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'w', bubbles: true }));
  H._driveTick();
  assert.strictEqual(sent.filter(m => m.type === 'joystick').length, 0, 'no joystick frames while disarmed');
  w.dispatchEvent(new w.KeyboardEvent('keyup', { key: 'w', bubbles: true }));

  console.log('Desktop 4: armed keyboard drive streams keepalive frames scaled by throttle, then zeros');
  ws.serverSend({ type: 'normal_drive_status', armed: true });
  sent.length = 0;
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'w', bubbles: true }));
  H._driveTick(); H._driveTick();
  const frames = sent.filter(m => m.type === 'joystick');
  assert(frames.length >= 2, 'expected streamed joystick frames');
  frames.forEach(f => {
    assert.strictEqual(f.keepalive, true, 'joystick frames must opt into the server watchdog');
    assert.strictEqual(f.deadman, undefined, 'keyboard frames use implicit deadman');
    assert(Math.abs(f.y - 0.5) < 1e-9 && f.x === 0, `expected y=0.5 (50% throttle), got ${JSON.stringify(f)}`);
  });
  w.dispatchEvent(new w.KeyboardEvent('keyup', { key: 'w', bubbles: true }));
  sent.length = 0;
  for (let i = 0; i < 6; i++) H._driveTick();
  const zeros = sent.filter(m => m.type === 'joystick');
  assert.strictEqual(zeros.length, 3, `expected exactly 3 zero frames after release, got ${zeros.length}`);
  zeros.forEach(f => assert(f.x === 0 && f.y === 0));

  console.log('Desktop 5: E-STOP fans out to every stop path');
  sent.length = 0; fetches.length = 0;
  w.document.getElementById('btn-top-estop').click();
  assert(sent.find(m => m.type === 'joystick' && m.x === 0 && m.y === 0 && m.deadman === false), 'zero joystick w/ deadman:false');
  assert(sent.find(m => m.type === 'disarm_normal_drive'), 'disarm_normal_drive');
  assert(sent.find(m => m.type === 'emergency_stop'), 'emergency_stop');
  assert(fetches.find(f => f.url === '/api/drive/disarm' && f.opts.method === 'POST'), 'HTTP disarm');
  assert(fetches.find(f => f.url === '/api/navigation/cancel' && f.opts.method === 'POST'), 'HTTP nav cancel');
  assert.strictEqual(H.state.drive.armed, false);

  console.log('Desktop 6: Spacebar triggers E-STOP; held keys are released');
  ws.serverSend({ type: 'normal_drive_status', armed: true });
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'd', bubbles: true }));
  sent.length = 0;
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: ' ', bubbles: true }));
  assert(sent.find(m => m.type === 'emergency_stop'), 'space must E-STOP');
  sent.length = 0;
  ws.serverSend({ type: 'normal_drive_status', armed: true });
  H._driveTick();
  assert.strictEqual(sent.filter(m => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length, 0, 'E-STOP must clear held drive input');
  w.dispatchEvent(new w.KeyboardEvent('keyup', { key: 'd', bubbles: true }));

  console.log('Desktop 7: ARM posts /api/drive/arm with operator token header');
  ws.serverSend({ type: 'normal_drive_status', armed: false });
  fetches.length = 0;
  w.document.getElementById('btn-arm-toggle').click();
  await sleep(10);
  const armReq = fetches.find(f => f.url === '/api/drive/arm');
  assert(armReq, 'arm request expected');
  assert.strictEqual(armReq.opts.method, 'POST');
  assert.strictEqual(armReq.opts.headers['X-Rover-Operator-Token'], TOKEN);

  console.log('Desktop 8: preset preview plans from the localized pose and never dispatches');
  await sleep(50); // let localization + home pollers land
  fetches.length = 0;
  w.document.querySelector('.preset-chip[data-dist="0.70"]').click();
  await sleep(20);
  const planReq = fetches.find(f => f.url === '/api/navigation/plan');
  assert(planReq, 'plan request expected');
  const body = JSON.parse(planReq.opts.body);
  assert(Math.abs(body.start_x - 0.5) < 1e-9 && Math.abs(body.start_y + 0.25) < 1e-9, 'start = localized pose');
  assert(Math.abs(body.target_x - 0.5) < 1e-6 && Math.abs(body.target_y - 0.45) < 1e-6, `target 0.7 m along +90°, got ${body.target_x},${body.target_y}`);
  assert(!fetches.find(f => f.url === '/api/navigation/dispatch'), 'preview must not dispatch');

  console.log('Desktop 9: GO dispatches with token header only after confirm()');
  let confirmed = false;
  w.confirm = () => { confirmed = true; return false; };
  fetches.length = 0;
  w.document.querySelector('.map-bottom-dock .js-go').click();
  await sleep(10);
  assert(confirmed, 'confirm dialog expected');
  assert(!fetches.find(f => f.url === '/api/navigation/dispatch'), 'declined confirm must not dispatch');
  w.confirm = () => true;
  w.document.querySelector('.map-bottom-dock .js-go').click();
  await sleep(10);
  const disp = fetches.find(f => f.url === '/api/navigation/dispatch');
  assert(disp && disp.opts.headers['X-Rover-Operator-Token'] === TOKEN, 'dispatch with token header');

  console.log('Desktop 10: WebSocket drop releases drive inputs');
  ws.serverSend({ type: 'normal_drive_status', armed: true });
  w.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'w', bubbles: true }));
  ws.close();
  assert.strictEqual(H.state.wsConnected, false);
  sent.length = 0;
  H._driveTick();
  assert.strictEqual(sent.length, 0);

  console.log('Core 1: map world<->canvas round trip');
  const view = H.createMapView({ scale: 95 });
  view.follow = false; view.pan = { x: 1.5, y: -2 };
  const c = H.worldToCanvas(view, 2.25, -1.1, 900, 650, 95);
  const back = H.canvasToWorld(view, c.u, c.v, 900, 650, 95);
  assert(Math.abs(back.x - 2.25) < 1e-9 && Math.abs(back.y + 1.1) < 1e-9);
  const center = H.worldToCanvas(view, 1.5, -2, 900, 650, 95);
  assert(Math.abs(center.u - 450) < 1e-9 && Math.abs(center.v - 325) < 1e-9);

  console.log('Core 2: LiDAR clockwise angle -> robot frame, near points masked');
  const pts = H.lidarPointsToRobotFrame([
    { angleDeg: 0, distanceMm: 1000 }, { angleDeg: 90, distanceMm: 2000 }, { angleDeg: 45, distanceMm: 100 }
  ]);
  assert.strictEqual(pts.length, 2, 'points under 0.15 m must be masked');
  assert(Math.abs(pts[0].x - 1) < 1e-9 && Math.abs(pts[0].y) < 1e-9);
  assert(Math.abs(pts[1].x) < 1e-9 && Math.abs(pts[1].y + 2) < 1e-9, '90° clockwise = robot right (-y)');

  console.log('Core 3: mission parser');
  const legs = H.parseMissionScript('forward 1 m, turn left 90 degrees, forward 100 cm, return home', { x: 0, y: 0, yawRad: 0 }, { x: 5, y: 6, yawRad: 1 });
  assert.strictEqual(legs.length, 4);
  assert(Math.abs(legs[2].target.x - 1) < 1e-9 && Math.abs(legs[2].target.y - 1) < 1e-9);
  assert.strictEqual(legs[3].type, 'RETURN_HOME');
  assert.throws(() => H.parseMissionScript('dance wildly', { x: 0, y: 0, yawRad: 0 }, null));
  assert.throws(() => H.parseMissionScript('return home', { x: 0, y: 0, yawRad: 0 }, null), /HOME/);

  console.log('Core 4: battery view mapping + 0.0V dummy guard');
  assert.strictEqual(H.batteryView({ voltage: 3.97, percentage: 49.2, powerState: 'CHARGING' }).plug, 'AC: CHARGING');
  assert.strictEqual(H.batteryView({ voltage: 3.4, percentage: 8, powerState: 'DISCHARGING' }).level, 'critical');
  assert.strictEqual(H.batteryView({ voltage: 0.0 }).pct, null, '0.0V dummy packet must not report valid battery');
  assert.strictEqual(H.batteryView({ voltage: 0.0 }).level, 'unknown');
  assert.strictEqual(H.batteryView(null).pct, null);

  console.log('Core 5: ToF 8x8 -> robot frame + radar live/nearest with micro zoom');
  const grid64 = new Array(64).fill(-1);
  grid64[27] = 100; // 0.10 m ahead of front-right ToF sensor
  const tofPayload = {
    stale: false,
    mapping: {
      front_right_l7cx: 0,
      front_right_l7cx_rot: 'none'
    },
    data: {
      channels: [
        { ch: 0, type: 'VL53L7CX', present: 1, init: 1, age_ms: 40, grid_mm: grid64 },
        { ch: 2, type: 'VL53L4CD', present: 1, init: 1, age_ms: 40, dist_mm: 150 }
      ]
    }
  };
  const tofPts = H.tofToRobotFrame(tofPayload);
  assert.strictEqual(tofPts.length, 1, 'single populated column should yield 1 point; single-zone CH 2 ignored');
  assert(tofPts[0].x > 0.14 && tofPts[0].y < 0, 'CH 0 front-right reading should project forward (+x) and right (-y)');
  H.state.scan = { points: [], t: 0 };
  H.state.tof = { points: tofPts, t: Date.now() };
  const radarCtx = {
    fillRect() {}, strokeRect() {}, beginPath() {}, arc() {},
    stroke() {}, fill() {}, moveTo() {}, lineTo() {}, fillText() {}
  };
  const radarRes = H.drawRadar(radarCtx, 260, { maxRange: 0.25 });
  assert.strictEqual(radarRes.live, true, 'radar must report live when ToF is active even if LiDAR has 0 points');
  assert.strictEqual(radarRes.tofLive, true);
  assert.strictEqual(radarRes.lidarLive, false);
  assert(radarRes.nearest && radarRes.nearest.d > 0.15 && radarRes.nearest.d < 0.30, 'nearest distance must reflect close-in ToF obstacle');

  console.log('Core 6: Front Safety Zone (<=75mm crawl, <=30mm stop) for autonomous and manual driving');
  assert.strictEqual(FRONT_STOP_THRESHOLD_MM, 30);
  assert.strictEqual(FRONT_CRAWL_THRESHOLD_MM, 75);
  const nowMs = Date.now();
  const makeTofState = (leftMm, rightMm) => ({
    host_timestamp_ms: nowMs,
    channels: [
      { ch: 0, type: 'VL53L7CX', present: 1, init: 1, age_ms: 30, grid_mm: [rightMm, ...new Array(63).fill(-1)], min_mm: rightMm },
      { ch: 1, type: 'VL53L7CX', present: 1, init: 1, age_ms: 30, grid_mm: [leftMm, ...new Array(63).fill(-1)], min_mm: leftMm }
    ]
  });
  const mapping = { front_left_l7cx: 1, front_right_l7cx: 0 };

  // Explicit boundary cases: 0, 29, 30, 31, 74, 75, 76 mm
  // Boundary 1: 0 mm -> STOP (0..30 mm)
  const zBound0 = evaluateFrontSafetyZone(makeTofState(0, 120), mapping, { nowMs });
  assert.strictEqual(zBound0.state, 'STOP', '0 mm must be STOP');
  assert.strictEqual(zBound0.minMm, 0);

  // Boundary 2: 29 mm -> STOP (0..30 mm)
  const zBound29 = evaluateFrontSafetyZone(makeTofState(120, 29), mapping, { nowMs });
  assert.strictEqual(zBound29.state, 'STOP', '29 mm must be STOP');
  assert.strictEqual(zBound29.minMm, 29);

  // Boundary 3: 30 mm -> STOP (0..30 mm)
  const zBound30 = evaluateFrontSafetyZone(makeTofState(30, 120), mapping, { nowMs });
  assert.strictEqual(zBound30.state, 'STOP', '30 mm must be STOP');
  assert.strictEqual(zBound30.minMm, 30);

  // Boundary 4: 31 mm -> CRAWL (31..75 mm)
  const zBound31 = evaluateFrontSafetyZone(makeTofState(120, 31), mapping, { nowMs });
  assert.strictEqual(zBound31.state, 'CRAWL', '31 mm must be CRAWL');
  assert.strictEqual(zBound31.minMm, 31);

  // Boundary 5: 74 mm -> CRAWL (31..75 mm)
  const zBound74 = evaluateFrontSafetyZone(makeTofState(74, 120), mapping, { nowMs });
  assert.strictEqual(zBound74.state, 'CRAWL', '74 mm must be CRAWL');
  assert.strictEqual(zBound74.minMm, 74);

  // Boundary 6: 75 mm -> CRAWL (31..75 mm)
  const zBound75 = evaluateFrontSafetyZone(makeTofState(120, 75), mapping, { nowMs });
  assert.strictEqual(zBound75.state, 'CRAWL', '75 mm must be CRAWL');
  assert.strictEqual(zBound75.minMm, 75);

  // Boundary 7: 76 mm -> CLEAR (> 75 mm)
  const zBound76 = evaluateFrontSafetyZone(makeTofState(76, 120), mapping, { nowMs });
  assert.strictEqual(zBound76.state, 'CLEAR', '76 mm must be CLEAR');
  assert.strictEqual(zBound76.minMm, 76);

  // > 75 mm -> CLEAR velocity evaluation (normal uninhibited operation)
  const vClear = applyFrontSafetyZoneToVelocity(0.20, 0.50, zBound76);
  assert.strictEqual(vClear.linear, 0.20);
  assert.strictEqual(vClear.angular, 0.50);
  assert.strictEqual(vClear.clamped, false);

  // 31..75 mm -> CRAWL velocity evaluation (slows forward driving to crawlSpeedMps = 0.03 m/s)
  const vCrawl = applyFrontSafetyZoneToVelocity(0.20, 0.40, zBound75);
  assert.strictEqual(vCrawl.linear, DEFAULT_CRAWL_LINEAR_MPS, 'forward speed in <=75mm zone must clamp to crawl speed');
  assert(vCrawl.angular > 0 && vCrawl.angular < 0.40, 'turn rate scales proportionally with crawl speed');
  assert.strictEqual(vCrawl.clamped, true);
  assert.strictEqual(vCrawl.action, 'CRAWL');

  // 0..30 mm -> STOP velocity evaluation (fully stops forward motion, allows reverse)
  const vStopFwd = applyFrontSafetyZoneToVelocity(0.20, 0.15, zBound30);
  assert.strictEqual(vStopFwd.linear, 0.0, 'forward motion must fully stop within 30mm');
  assert.strictEqual(vStopFwd.angular, 0.0, 'angular motion must fully stop when forward blocked within 30mm');
  assert.strictEqual(vStopFwd.clamped, true);
  assert.strictEqual(vStopFwd.action, 'STOP');

  // Reverse motion must still be allowed in STOP zone so rover can back away (reverse escape)
  const vStopRev = applyFrontSafetyZoneToVelocity(-0.10, 0.0, zBound30);
  assert.strictEqual(vStopRev.linear, -0.10, 'reverse motion must be allowed when stopped at front obstacle');
  assert.strictEqual(vStopRev.action, 'REVERSE_ALLOWED');

  // Client-side browser fallback evaluateFrontZone boundary tests
  const makeResp = (mm) => ({
    stale: false,
    mapping: { front_left_l7cx: 1 },
    data: { channels: [{ ch: 1, type: 'VL53L7CX', present: 1, init: 1, age_ms: 20, min_mm: mm }] }
  });
  assert.strictEqual(H.evaluateFrontZone(makeResp(0)).state, 'STOP');
  assert.strictEqual(H.evaluateFrontZone(makeResp(29)).state, 'STOP');
  assert.strictEqual(H.evaluateFrontZone(makeResp(30)).state, 'STOP');
  assert.strictEqual(H.evaluateFrontZone(makeResp(31)).state, 'CRAWL');
  assert.strictEqual(H.evaluateFrontZone(makeResp(74)).state, 'CRAWL');
  assert.strictEqual(H.evaluateFrontZone(makeResp(75)).state, 'CRAWL');
  assert.strictEqual(H.evaluateFrontZone(makeResp(76)).state, 'CLEAR');

  // HUD client-side evaluateFrontZone + 8 mm close-in ToF radar point retention
  const closeGrid = new Array(64).fill(-1);
  closeGrid[27] = 8; // 8 mm (inside <=30mm stop zone)
  const closeResp = {
    stale: false,
    mapping: { front_right_l7cx: 0, front_right_l7cx_rot: 'none' },
    data: { channels: [{ ch: 0, type: 'VL53L7CX', present: 1, init: 1, age_ms: 25, grid_mm: closeGrid, min_mm: 8 }] }
  };
  const closePts = H.tofToRobotFrame(closeResp);
  assert.strictEqual(closePts.length, 1, '8 mm (< 20 mm) ToF reading must not be dropped by radar projection');
  const hudZone = H.evaluateFrontZone(closeResp);
  assert.strictEqual(hudZone.state, 'STOP');
  assert.strictEqual(hudZone.minMm, 8);

  console.log('Core 7: Radar box proximity indicator (below radar circles, authoritative states, stale expiration, separation)');
  // 1. Verify radar circles stay strictly outside the bottom indicator region (y >= 235 on 260px box)
  const drawnArcs = [];
  const drawnRects = [];
  const trackCtx = {
    fillRect(x, y, w, h) { drawnRects.push({ x, y, w, h }); },
    strokeRect() {},
    beginPath() {},
    arc(cx, cy, r) { drawnArcs.push({ cx, cy, r }); },
    stroke() {},
    fill() {},
    moveTo() {},
    lineTo() {},
    fillText() {}
  };
  H.state.tof = { points: [], t: 0 };
  H.drawRadar(trackCtx, 260, { maxRange: 0.25 });
  const maxCircleBottomY = Math.max(...drawnArcs.map(a => a.cy + a.r));
  assert(maxCircleBottomY < 210, `radar outer circle bottom (${maxCircleBottomY}) must stay well above the bottom indicator strip (y=239..257)`);
  // Only canvas background (260x260) and rover footprint box should be filled on desktop 260px radar (no zone boxes inside circles)
  assert.strictEqual(drawnRects.length, 2, 'no crawl/stop zone rectangles may be drawn inside the radar circles');

  // 2. Missing / unavailable / stale data -> Gray: PROXIMITY — OFFLINE
  const zOfflineNull = evaluateFrontSafetyZone(null, mapping, { nowMs });
  assert.strictEqual(zOfflineNull.state, 'OFFLINE');
  assert.strictEqual(zOfflineNull.indicatorCode, 'OFFLINE');
  assert.strictEqual(zOfflineNull.indicatorText, 'PROXIMITY — OFFLINE');
  const zOfflineStale = evaluateFrontSafetyZone({ ...makeTofState(30, 30), host_timestamp_ms: nowMs - 5000 }, mapping, { nowMs });
  assert.strictEqual(zOfflineStale.state, 'OFFLINE');
  assert.strictEqual(zOfflineStale.indicatorText, 'PROXIMITY — OFFLINE');

  H._handleMessage({ type: 'front_safety_zone', zone: zOfflineNull });
  let ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'OFFLINE');
  assert.strictEqual(ind.level, 'offline');
  assert.strictEqual(ind.text, 'PROXIMITY — OFFLINE');
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent.trim(), 'PROXIMITY — OFFLINE');
  assert(env.w.document.getElementById('hud-radar-drive-ind').classList.contains('ind-offline'));

  // 3. Stationary crawl zone (31..75 mm, no motion reduced) -> Yellow: PROXIMITY — CRAWL ZONE
  const zCrawlStationary = evaluateFrontSafetyZone(makeTofState(45, 90), mapping, { nowMs, requestedLinearMps: 0, requestedAngularRadps: 0 });
  assert.strictEqual(zCrawlStationary.state, 'CRAWL');
  assert.strictEqual(zCrawlStationary.intervening, false);
  assert.strictEqual(zCrawlStationary.indicatorCode, 'CRAWL_ZONE');
  assert.strictEqual(zCrawlStationary.indicatorText, 'PROXIMITY — CRAWL ZONE');
  H._handleMessage({ type: 'front_safety_zone', zone: zCrawlStationary });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'CRAWL_ZONE');
  assert.strictEqual(ind.level, 'warn');
  assert.strictEqual(ind.text, 'PROXIMITY — CRAWL ZONE');
  assert.strictEqual(ind.slowDownOverride, false, 'stationary crawl zone must not claim active slowdown intervention');
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent, 'PROXIMITY — CRAWL ZONE');
  assert(env.w.document.getElementById('hud-radar-drive-ind').classList.contains('ind-warn'));

  // 4. Active slowdown intervention (31..75 mm while forward motion > 0.03 m/s is reduced) -> Yellow: PROXIMITY — SLOWING
  const zSlowing = evaluateFrontSafetyZone(makeTofState(45, 90), mapping, { nowMs, requestedLinearMps: 0.20, requestedAngularRadps: 0 });
  assert.strictEqual(zSlowing.state, 'CRAWL');
  assert.strictEqual(zSlowing.intervening, true);
  assert.strictEqual(zSlowing.indicatorCode, 'SLOWING');
  assert.strictEqual(zSlowing.indicatorText, 'PROXIMITY — SLOWING');
  H._handleMessage({ type: 'front_safety_zone', zone: zSlowing });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'SLOWING');
  assert.strictEqual(ind.level, 'warn');
  assert.strictEqual(ind.text, 'PROXIMITY — SLOWING');
  assert.strictEqual(ind.slowDownOverride, true);
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent, 'PROXIMITY — SLOWING');

  // 5. Stationary stop zone (0..30 mm, no motion commanded) -> Red: PROXIMITY — STOP ZONE
  const zStopStationary = evaluateFrontSafetyZone(makeTofState(65, 20), mapping, { nowMs, requestedLinearMps: 0, requestedAngularRadps: 0 });
  assert.strictEqual(zStopStationary.state, 'STOP');
  assert.strictEqual(zStopStationary.intervening, false);
  assert.strictEqual(zStopStationary.indicatorCode, 'STOP_ZONE');
  assert.strictEqual(zStopStationary.indicatorText, 'PROXIMITY — STOP ZONE');
  H._handleMessage({ type: 'front_safety_zone', zone: zStopStationary });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'STOP_ZONE');
  assert.strictEqual(ind.level, 'bad');
  assert.strictEqual(ind.text, 'PROXIMITY — STOP ZONE');
  assert.strictEqual(ind.brakeApplied, false, 'stationary stop zone must not claim active stop intervention');
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent, 'PROXIMITY — STOP ZONE');
  assert(env.w.document.getElementById('hud-radar-drive-ind').classList.contains('ind-bad'));

  // 6. Active stop intervention (0..30 mm while forward/in-place motion is blocked) -> Red: PROXIMITY — STOP
  const zStopping = evaluateFrontSafetyZone(makeTofState(65, 20), mapping, { nowMs, requestedLinearMps: 0.18, requestedAngularRadps: 0 });
  assert.strictEqual(zStopping.state, 'STOP');
  assert.strictEqual(zStopping.intervening, true);
  assert.strictEqual(zStopping.indicatorCode, 'STOP');
  assert.strictEqual(zStopping.indicatorText, 'PROXIMITY — STOP');
  H._handleMessage({ type: 'front_safety_zone', zone: zStopping });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'STOP');
  assert.strictEqual(ind.level, 'bad');
  assert.strictEqual(ind.text, 'PROXIMITY — STOP');
  assert.strictEqual(ind.brakeApplied, true);
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent, 'PROXIMITY — STOP');

  // 7. Reverse escape in stop zone (vx < 0 allowed without restriction) -> Red: PROXIMITY — STOP ZONE (not claiming block)
  const zReverseEscape = evaluateFrontSafetyZone(makeTofState(65, 20), mapping, { nowMs, requestedLinearMps: -0.12, requestedAngularRadps: 0 });
  assert.strictEqual(zReverseEscape.state, 'STOP');
  assert.strictEqual(zReverseEscape.intervening, false);
  assert.strictEqual(zReverseEscape.effectiveLinearMps, -0.12);
  assert.strictEqual(zReverseEscape.indicatorCode, 'STOP_ZONE');
  assert.strictEqual(zReverseEscape.indicatorText, 'PROXIMITY — STOP ZONE');
  H._handleMessage({ type: 'front_safety_zone', zone: zReverseEscape });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'STOP_ZONE');
  assert.strictEqual(ind.text, 'PROXIMITY — STOP ZONE');

  // 8. Recovery to clear (> 75 mm) -> Neutral: PROXIMITY — CLEAR, and unrelated motor-power / E-stop / dynamic-brake do not pollute it
  const zRecovered = evaluateFrontSafetyZone(makeTofState(180, 220), mapping, { nowMs, requestedLinearMps: 0.20, requestedAngularRadps: 0 });
  H.state.drive = { armed: false, mode: 4 }; // Disarmed + E-stop mode
  H._handleMessage({
    type: 'pid_diagnostic',
    actuationState: 'BRAKE',
    m1: { targetRadps: 0, measuredRadps: 0.4, finalPwm: 0, stictionState: 'BLOCKED', stictionCode: 3 }
  });
  H._handleMessage({ type: 'front_safety_zone', zone: zRecovered });
  ind = H.evaluateRadarDriveIndicator(nowMs);
  assert.strictEqual(ind.code, 'CLEAR');
  assert.strictEqual(ind.level, 'clear');
  assert.strictEqual(ind.text, 'PROXIMITY — CLEAR', 'unrelated motor power, E-stop, or dynamic brake must not overwrite proximity indicator');
  assert.strictEqual(env.w.document.getElementById('hud-radar-drive-ind').textContent, 'PROXIMITY — CLEAR');
  assert(env.w.document.getElementById('hud-radar-drive-ind').classList.contains('ind-clear'));

  // 9. Stale telemetry expiration + WebSocket disconnect/reconnect handling
  H._connect(); // Reconnect active WS after Desktop 10 closed the first socket
  await sleep(20);
  const activeWs = env.MockWS.instances[env.MockWS.instances.length - 1];
  activeWs.serverSend({ type: 'front_safety_zone', zone: zStopping });
  assert.strictEqual(H.evaluateRadarDriveIndicator().code, 'STOP');
  // Simulate WS disconnect while in STOP intervention -> intervention drops immediately to STOP_ZONE
  activeWs.close();
  assert.strictEqual(H.evaluateRadarDriveIndicator().code, 'STOP_ZONE', 'WS disconnect must immediately clear active intervention claim');
  // Advance clock past FRONT_ZONE_STALE_MS (2500 ms) -> expires old STOP_ZONE state to OFFLINE
  const expiredInd = H.evaluateRadarDriveIndicator(Date.now() + H.FRONT_ZONE_STALE_MS + 200);
  assert.strictEqual(expiredInd.code, 'OFFLINE', 'stale telemetry must expire to OFFLINE so old STOP states cannot remain live');
  assert.strictEqual(expiredInd.level, 'offline');
  assert.strictEqual(expiredInd.text, 'PROXIMITY — OFFLINE');
  // Fresh update recovers from OFFLINE
  H._handleMessage({ type: 'front_safety_zone', zone: zRecovered });
  assert.strictEqual(H.evaluateRadarDriveIndicator().code, 'CLEAR');

  console.log('Desktop 11: top-bar prioritizes battery/plug/conn/auth before secondary buttons and right pane puts Drivetrain/Drive Controls above Autonomy');
  const htmlTxt = read('hud.html');
  assert(htmlTxt.includes('id="hud-radar-drive-ind"'), 'radar box must contain #hud-radar-drive-ind indicator');
  const idxPlug = htmlTxt.indexOf('id="ups-plug-badge"');
  const idxBat = htmlTxt.indexOf('id="battery-badge"');
  const idxConn = htmlTxt.indexOf('id="conn-badge"');
  const idxAuth = htmlTxt.indexOf('id="auth-badge"');
  const idxHome = htmlTxt.indexOf('id="btn-top-home"');
  const idxInitHome = htmlTxt.indexOf('id="btn-top-init-home"');
  const idxMotor = htmlTxt.indexOf('id="motor-battery-badge"');
  const idxGamepad = htmlTxt.indexOf('id="gamepad-badge"');
  assert(idxPlug > 0 && idxBat > 0 && idxConn > 0 && idxAuth > 0, 'critical top-bar badges missing');
  assert(idxPlug < idxHome && idxBat < idxHome && idxConn < idxHome && idxAuth < idxHome, 'critical badges must precede Return Home');
  assert(idxBat < idxInitHome && idxBat < idxMotor && idxBat < idxGamepad, 'battery badge must precede secondary badges');
  const idxDriveCard = htmlTxt.indexOf('id="drivetrain-card"');
  const idxCtrlCard = htmlTxt.indexOf('id="drive-controls-card"');
  const idxAutonomyCard = htmlTxt.indexOf('Autonomy &amp; Mission Control');
  assert(idxDriveCard > 0 && idxCtrlCard > 0 && idxDriveCard < idxAutonomyCard && idxCtrlCard < idxAutonomyCard, 'Drivetrain & Drive Controls must precede Autonomy card in right sidebar');
  const cssTxt = read('hud.css');
  assert(cssTxt.includes('@media (max-width: 1620px)') && cssTxt.includes('@media (max-width: 1340px)') && cssTxt.includes('@media (max-width: 1080px)'), 'responsive @media rules missing in hud.css');

  console.log('Desktop 12: pagehide while driving immediately flushes zero joystick frame');
  const env2 = makeDom('hud.html', { token: TOKEN, extraScripts: ['hud-desktop.js'] });
  await sleep(30);
  const ws2 = env2.MockWS.instances[0];
  ws2.serverSend({ type: 'auth_result', ok: true });
  ws2.serverSend({ type: 'normal_drive_status', armed: true });
  env2.w.dispatchEvent(new env2.w.KeyboardEvent('keydown', { key: 'w', bubbles: true }));
  env2.H._driveTick();
  env2.sent.length = 0;
  env2.w.dispatchEvent(new env2.w.Event('pagehide'));
  const zeroFlush = env2.sent.find(m => m.type === 'joystick' && m.x === 0 && m.y === 0 && m.deadman === false);
  assert(zeroFlush, 'pagehide while driving must immediately send zero joystick frame with deadman:false');
  env2.dom.window.close();

  env.dom.window.close();
}

async function mobileTests() {
  console.log('Mobile 1: page loads and E-STOP fans out');
  const env = makeDom('hud-mobile.html', { token: TOKEN, extraScripts: ['hud-mobile.js'] });
  await sleep(30);
  env.MockWS.instances[0].serverSend({ type: 'auth_result', ok: true });
  env.sent.length = 0;
  env.fetches.length = 0;
  env.w.document.getElementById('ctlEstop').dispatchEvent(new env.w.Event('pointerdown', { bubbles: true, cancelable: true }));
  assert(env.sent.find(m => m.type === 'emergency_stop'), 'mobile emergency_stop');
  assert(env.sent.find(m => m.type === 'disarm_normal_drive'), 'mobile disarm');
  assert(env.fetches.find(f => f.url === '/api/drive/disarm'), 'mobile HTTP disarm');

  console.log('Mobile 2: Rover 2 mobile CSS + non-overlapping landscape 3-column grid and top-bar battery priority');
  const mobHtml = read('hud-mobile.html');
  assert(mobHtml.indexOf('id="cBat"') < mobHtml.indexOf('id="btnTopHome"'), 'mobile #cBat must precede #btnTopHome');
  assert(mobHtml.indexOf('id="cAuth"') < mobHtml.indexOf('id="btnTopHome"'), 'mobile #cAuth must precede #btnTopHome');
  const styleTxt = env.w.document.querySelector('style').textContent;
  assert(styleTxt.includes('#stage.mode-split') && styleTxt.includes('.stick.v'), 'Rover 2 mobile CSS missing');
  assert(styleTxt.includes('#stkLWrap') && styleTxt.includes('#stkRWrap') && styleTxt.includes('display:contents'), 'landscape 3-column non-overlapping grid CSS missing');
  assert(!styleTxt.includes('ROVER2_MOBILE_CSS'), 'template placeholder left in page');

  console.log('Mobile 3: canvas proximity indicator renders at all responsive sizes (<200 and >=200) without clipping or overlapping circles');
  env.H._handleMessage({
    type: 'front_safety_zone',
    zone: {
      state: 'CRAWL',
      active: true,
      sensorLive: true,
      intervening: true,
      indicatorCode: 'SLOWING',
      indicatorLevel: 'warn',
      indicatorText: 'PROXIMITY — SLOWING',
      minMm: 28
    }
  });
  for (const testSize of [88, 140, 200, 240]) {
    const arcs = [];
    const rects = [];
    const texts = [];
    const mCtx = {
      fillRect(x, y, w, h) { rects.push({ x, y, w, h }); },
      strokeRect() {},
      beginPath() {},
      arc(cx, cy, r) { arcs.push({ cx, cy, r }); },
      stroke() {},
      fill() {},
      moveTo() {},
      lineTo() {},
      fillText(text, x, y, maxW) { texts.push({ text, x, y, maxW }); }
    };
    const r = env.H.drawRadar(mCtx, testSize, { maxRange: 0.50, color: 'range', pointSize: 2 });
    assert.strictEqual(r.driveIndicator.text, 'PROXIMITY — SLOWING');
    const indText = texts.find(t => t.text === 'PROXIMITY — SLOWING');
    assert(indText, `mobile canvas indicator text must render at size=${testSize}px (must not disappear at >=200px)`);
    assert(typeof indText.maxW === 'number' && indText.maxW > 0 && indText.maxW < testSize, `mobile canvas indicator must constrain maxWidth to prevent clipping at size=${testSize}px`);
    const indBar = rects[rects.length - 1];
    const outerCircleBottom = Math.max(...arcs.map(a => a.cy + a.r));
    assert(indBar.y > outerCircleBottom, `indicator bar top (${indBar.y}) must be below outer radar circle (${outerCircleBottom}) at size=${testSize}px`);
    assert(indBar.x >= 0 && (indBar.x + indBar.w) <= testSize && (indBar.y + indBar.h) <= testSize, `indicator bar must fit inside canvas bounds at size=${testSize}px`);
  }
  env.dom.window.close();
}

(async () => {
  try {
    staticChecks();
    await desktopTests();
    await mobileTests();
    console.log('ALL HUD COCKPIT TESTS PASSED');
    process.exit(0);
  } catch (err) {
    console.error('TEST FAILED:', err);
    process.exit(1);
  }
})();

