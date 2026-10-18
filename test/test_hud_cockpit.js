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

  console.log('Core 4: battery view mapping');
  assert.strictEqual(H.batteryView({ voltage: 3.97, percentage: 49.2, powerState: 'CHARGING' }).plug, 'AC: CHARGING');
  assert.strictEqual(H.batteryView({ voltage: 3.4, percentage: 8, powerState: 'DISCHARGING' }).level, 'critical');
  assert.strictEqual(H.batteryView(null).pct, null);

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

  console.log('Mobile 2: Rover 2 mobile CSS was injected');
  const styleTxt = env.w.document.querySelector('style').textContent;
  assert(styleTxt.includes('#stage.mode-split') && styleTxt.includes('.stick.v'), 'Rover 2 mobile CSS missing');
  assert(!styleTxt.includes('ROVER2_MOBILE_CSS'), 'template placeholder left in page');
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
