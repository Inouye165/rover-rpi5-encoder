// ==============================================================================
// test_hud_gamepad.js
// Behavioral regression tests for Xbox controller compatibility in Rover 1 HUD.
// Uses real HUD HTML and scripts across BOTH layouts:
//   - Desktop HUD:  hud.html + hud-core.js + hud-gamepad.js + hud-desktop.js
//   - Mobile HUD:   hud-mobile.html + hud-core.js + hud-gamepad.js + hud-mobile.js
// Tested with mocked navigator.getGamepads, WebSocket, and HTTP APIs.
// ==============================================================================
const assert = require('assert');
const fs = require('fs');
const path = require('path');
const { JSDOM, VirtualConsole } = require('jsdom');

const PUB = path.join(__dirname, '..', 'public');
const read = (f) => fs.readFileSync(path.join(PUB, f), 'utf8');
const TOKEN = 'test-operator-token-xbox-123';
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function createMockGamepad({
  connected = true,
  id = 'Xbox 360 Controller (XInput STANDARD GAMEPAD)',
  buttons = {},
  axes = [0, 0, 0, 0]
} = {}) {
  const btnList = [];
  for (let i = 0; i < 16; i++) {
    const val = buttons[i] !== undefined ? buttons[i] : 0;
    btnList.push({
      pressed: val > 0.5,
      value: Number(val)
    });
  }
  return {
    id,
    connected,
    mapping: 'standard',
    buttons: btnList,
    axes: [...axes]
  };
}

function makeDom({ layout = 'desktop', token = TOKEN, mockPads = [] } = {}) {
  const isDesktop = layout === 'desktop';
  const htmlFile = isDesktop ? 'hud.html' : 'hud-mobile.html';
  const scripts = isDesktop
    ? ['hud-core.js', 'hud-gamepad.js', 'hud-desktop.js']
    : ['hud-core.js', 'hud-gamepad.js', 'hud-mobile.js'];

  const html = read(htmlFile).replace(/<script src="[^"]+"><\/script>/g, '');
  const vc = new VirtualConsole();
  vc.on('error', (...args) => {
    const str = args.map(String).join(' ');
    if (!/getContext|Not implemented/.test(str)) {
      console.error(`[dom error ${layout}]`, ...args);
    }
  });

  const dom = new JSDOM(html, {
    url: `http://10.0.0.246:3000/${htmlFile}?mode=${layout}`,
    runScripts: 'outside-only',
    pretendToBeVisual: true,
    virtualConsole: vc
  });
  const w = dom.window;

  // Mock gamepad state container
  let currentPads = mockPads;
  w.navigator.getGamepads = () => currentPads;

  const sent = [];
  const fetches = [];
  class MockWS {
    constructor(url) {
      this.url = url;
      this.readyState = 0;
      MockWS.instances.push(this);
      setTimeout(() => {
        this.readyState = 1;
        if (this.onopen) this.onopen();
      }, 0);
    }
    send(d) {
      sent.push(JSON.parse(d));
    }
    close() {
      this.readyState = 3;
      if (this.onclose) this.onclose();
    }
    serverSend(obj) {
      if (this.onmessage) this.onmessage({ data: JSON.stringify(obj) });
    }
  }
  MockWS.instances = [];
  w.WebSocket = MockWS;

  w.fetch = async (url, opts = {}) => {
    fetches.push({ url, opts });
    return {
      ok: true,
      status: 200,
      json: async () => ({ ok: true })
    };
  };
  w.confirm = () => true;
  w.alert = () => {};

  if (token) w.localStorage.setItem('rover_operator_token', token);

  scripts.forEach((s) => w.eval(read(s)));

  const stepGamepad = async () => {
    if (typeof w._pollGamepad === 'function') {
      w._pollGamepad();
    } else {
      await sleep(60);
    }
  };

  return {
    dom,
    w,
    sent,
    fetches,
    MockWS,
    H: w.RoverHUD,
    layout,
    setPads: (pads) => {
      currentPads = pads;
    },
    stepGamepad
  };
}

async function runLayoutSuite(layout) {
  const L = layout.toUpperCase();
  console.log(`\n--- Running Gamepad Suite on ${L} HUD (${layout === 'desktop' ? 'hud.html' : 'hud-mobile.html'}) ---`);

  // 1. Authenticated, armed RT + forward stick emits nonzero keepalive joystick frames.
  console.log(`  [${L}] 1: Authenticated, armed RT (button 7) + forward stick emits nonzero keepalive frames`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();

    env.sent.length = 0;
    env.H._driveTick();
    const frames = env.sent.filter((m) => m.type === 'joystick');
    assert(frames.length > 0, `FAIL [${L}]: Expected nonzero joystick frame with RT held + forward stick`);
    assert(frames[0].y > 0, `FAIL [${L}]: Expected positive forward y, got ${frames[0].y}`);
    assert.strictEqual(frames[0].keepalive, true, `FAIL [${L}]: Joystick frame must have keepalive: true`);
    env.dom.window.close();
  }

  // 2. RB independently works as deadman without speed boost.
  console.log(`  [${L}] 2: RB (button 5) independently works as deadman without speed boost`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // RT measurement
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    const rtFrames = env.sent.filter((m) => m.type === 'joystick');
    assert(rtFrames.length > 0, `FAIL [${L}]: RT driving should produce frame`);
    const rtY = rtFrames[0].y;

    // RB measurement
    env.setPads([createMockGamepad({ buttons: { 5: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    const rbFrames = env.sent.filter((m) => m.type === 'joystick');
    assert(rbFrames.length > 0, `FAIL [${L}]: RB alone must engage deadman and produce frame`);
    const rbY = rbFrames[0].y;

    assert(
      Math.abs(rbY - rtY) < 1e-4,
      `FAIL [${L}]: RB must not increase speed merely because it is deadman (rbY=${rbY}, rtY=${rtY})`
    );
    env.dom.window.close();
  }

  // 3. Left-stick steering works; neutral/deadzone produces no motion.
  console.log(`  [${L}] 3: Left-stick steering (axes[0]) works; deadband suppresses resting drift`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Right steer
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0.75, 0, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    const steerFrames = env.sent.filter((m) => m.type === 'joystick');
    assert(steerFrames.length > 0 && steerFrames[0].x > 0, `FAIL [${L}]: Steer right must produce positive x`);

    // Deadzone
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0.05, 0.05, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    const deadzoneFrames = env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0));
    assert.strictEqual(deadzoneFrames.length, 0, `FAIL [${L}]: Deadband must produce no motion`);
    env.dom.window.close();
  }

  // 4. Without deadman, auth, connection, or arming, no nonzero motion is transmitted.
  console.log(`  [${L}] 4: Safety gates block motion when disarmed, unauthed, or deadman released`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];

    // Disarmed / unauthed
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    assert.strictEqual(env.sent.filter((m) => m.type === 'joystick').length, 0, `FAIL [${L}]: Disarmed must send no joystick`);

    // Authed and armed, but NO deadman
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });
    env.setPads([createMockGamepad({ buttons: {}, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    const noDeadmanFrames = env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0));
    assert.strictEqual(noDeadmanFrames.length, 0, `FAIL [${L}]: No deadman must send no nonzero frames`);
    env.dom.window.close();
  }

  // 5. Deadman release and disconnect stop commanded motion.
  console.log(`  [${L}] 5: Deadman release and disconnect stop motion immediately`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Active
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.H._driveTick();

    // Release
    env.setPads([createMockGamepad({ buttons: { 7: 0.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Release must stop motion`
    );

    // Disconnect
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.H._driveTick();

    env.setPads([]);
    await env.stepGamepad();
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Disconnect must stop motion`
    );
    env.dom.window.close();
  }

  // 6. Start sends exactly one authenticated arm request per press; holding does not repeat.
  console.log(`  [${L}] 6: Start button (9) edge-triggered arm; holding does not repeat`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: false });

    env.fetches.length = 0;
    env.setPads([createMockGamepad({ buttons: { 9: 1.0 } })]);
    await env.stepGamepad();
    await sleep(10);

    const armReqs = env.fetches.filter((f) => f.url === '/api/drive/arm');
    assert.strictEqual(armReqs.length, 1, `FAIL [${L}]: Expected exactly 1 arm request on Start`);
    assert.strictEqual(armReqs[0].opts.headers['X-Rover-Operator-Token'], TOKEN);

    // Holding across polls
    await env.stepGamepad();
    await env.stepGamepad();
    await sleep(10);
    assert.strictEqual(
      env.fetches.filter((f) => f.url === '/api/drive/arm').length,
      1,
      `FAIL [${L}]: Holding Start must not send duplicate requests`
    );
    env.dom.window.close();
  }

  // 7. Back disarms; A and B trigger established E-stop paths.
  console.log(`  [${L}] 7: Back (8) disarms; A (0) and B (1) trigger established E-stop paths`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Back disarms
    env.sent.length = 0;
    env.fetches.length = 0;
    env.setPads([createMockGamepad({ buttons: { 8: 1.0 } })]);
    await env.stepGamepad();
    await sleep(10);

    assert(env.sent.some((m) => m.type === 'disarm_normal_drive'), `FAIL [${L}]: Back must send disarm_normal_drive`);
    assert(env.fetches.some((f) => f.url === '/api/drive/disarm'), `FAIL [${L}]: Back must post /api/drive/disarm`);

    // A triggers E-stop
    ws.serverSend({ type: 'normal_drive_status', armed: true });
    env.sent.length = 0;
    env.fetches.length = 0;
    env.setPads([createMockGamepad({ buttons: { 0: 1.0 } })]);
    await env.stepGamepad();
    await sleep(10);

    assert(env.sent.some((m) => m.type === 'emergency_stop'), `FAIL [${L}]: Button A must trigger emergency_stop`);

    // B triggers E-stop
    env.setPads([createMockGamepad({ buttons: { 0: 0.0 } })]);
    await env.stepGamepad();
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    env.sent.length = 0;
    env.setPads([createMockGamepad({ buttons: { 1: 1.0 } })]);
    await env.stepGamepad();
    await sleep(10);

    assert(env.sent.some((m) => m.type === 'emergency_stop'), `FAIL [${L}]: Button B must trigger emergency_stop`);
    env.dom.window.close();
  }

  // 8. Stop/disarm wins over simultaneous arm or drive inputs.
  console.log(`  [${L}] 8: Stop/disarm strictly takes precedence over simultaneous arm/motion`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Simultaneous: A (estop) + 9 (arm) + 7 (deadman) + stick forward
    env.sent.length = 0;
    env.fetches.length = 0;
    env.setPads([
      createMockGamepad({
        buttons: { 0: 1.0, 7: 1.0, 9: 1.0 },
        axes: [0, -0.9, 0, 0]
      })
    ]);
    await env.stepGamepad();
    await sleep(10);

    assert(env.sent.some((m) => m.type === 'emergency_stop'), `FAIL [${L}]: E-stop must trigger`);
    assert(!env.fetches.some((f) => f.url === '/api/drive/arm'), `FAIL [${L}]: Arm must NOT fire during E-stop`);

    env.sent.length = 0;
    env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Motion must be blocked during E-stop`
    );
    env.dom.window.close();
  }

  // 9. Interruption / focus loss / pagehide stops motion and requires fresh deadman re-engagement.
  console.log(`  [${L}] 9: Interruption stops motion; fresh deadman re-engagement strictly required`);
  {
    const env = makeDom({ layout });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Drive actively
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    assert(env.sent.some((m) => m.type === 'joystick' && m.y > 0), `FAIL [${L}]: Initial drive must be active`);

    // Window blur
    env.w.dispatchEvent(new env.w.Event('blur'));
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Focus loss must stop motion`
    );

    // Repeated polling while RT is STILL held cannot restart motion!
    await env.stepGamepad();
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Must not resume without deliberate release`
    );

    // Release RT
    env.setPads([createMockGamepad({ buttons: { 7: 0.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();

    // Re-squeeze RT (fresh engagement)
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    env.sent.length = 0;
    env.H._driveTick();
    assert(env.sent.some((m) => m.type === 'joystick' && m.y > 0), `FAIL [${L}]: Fresh deadman must resume motion`);

    // Hidden page test
    Object.defineProperty(env.w.document, 'hidden', { value: true, configurable: true, writable: true });
    env.w.document.dispatchEvent(new env.w.Event('visibilitychange'));
    await env.stepGamepad();
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      `FAIL [${L}]: Hidden page must stop motion`
    );
    env.dom.window.close();
  }
}

async function runDesktopSpecificTests() {
  console.log('\n--- Running Desktop-Specific Tests ---');
  console.log('  [DESKTOP] Keyboard driving remains functional alongside gamepad support');
  {
    const env = makeDom({ layout: 'desktop' });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Gamepad idle
    env.setPads([createMockGamepad({ buttons: {}, axes: [0, 0, 0, 0] })]);
    await env.stepGamepad();

    // Press 'w' on keyboard
    env.w.dispatchEvent(new env.w.KeyboardEvent('keydown', { key: 'w', bubbles: true }));
    env.sent.length = 0;
    env.H._driveTick();
    const kbFrames = env.sent.filter((m) => m.type === 'joystick' && m.y > 0);
    assert(kbFrames.length > 0, 'FAIL: Keyboard driving must emit forward motion');

    env.w.dispatchEvent(new env.w.KeyboardEvent('keyup', { key: 'w', bubbles: true }));
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      'FAIL: Keyboard release must stop motion'
    );
    env.dom.window.close();
  }

  console.log('  [DESKTOP] Status badge messaging and error handling');
  {
    const env = makeDom({ layout: 'desktop' });
    await sleep(25);

    // Exception handling
    env.w.navigator.getGamepads = () => {
      throw new Error('SecurityError: Gamepad blocked');
    };
    await env.stepGamepad();
    assert.strictEqual(env.w.document.getElementById('gamepad-status-text').textContent, 'HTTP BLOCKED');

    // Missing API
    delete env.w.navigator.getGamepads;
    await env.stepGamepad();
    assert.strictEqual(env.w.document.getElementById('gamepad-status-text').textContent, 'API UNAVAILABLE');
    env.dom.window.close();
  }
}

async function runMobileSpecificTests() {
  console.log('\n--- Running Mobile-Specific Tests ---');
  console.log('  [MOBILE] Touch sticks driving remains fully functional');
  {
    const env = makeDom({ layout: 'mobile' });
    await sleep(25);
    const ws = env.MockWS.instances[0];
    ws.serverSend({ type: 'auth_result', ok: true });
    ws.serverSend({ type: 'normal_drive_status', armed: true });

    // Gamepad idle
    env.setPads([createMockGamepad({ buttons: {}, axes: [0, 0, 0, 0] })]);
    await env.stepGamepad();

    // Touch stick input on mobile
    env.H.setDriveInput('touch', 0.0, 0.75); // Forward touch drive
    env.sent.length = 0;
    env.H._driveTick();

    const touchFrames = env.sent.filter((m) => m.type === 'joystick' && m.y > 0);
    assert(touchFrames.length > 0, 'FAIL [MOBILE]: Touch drive must emit forward motion when gamepad is idle');

    // Touch release
    env.H.clearDriveInput('touch');
    env.sent.length = 0;
    for (let i = 0; i < 5; i++) env.H._driveTick();
    assert.strictEqual(
      env.sent.filter((m) => m.type === 'joystick' && (m.x !== 0 || m.y !== 0)).length,
      0,
      'FAIL [MOBILE]: Touch release must stop motion'
    );
    env.dom.window.close();
  }

  console.log('  [MOBILE] Top-bar controller status chip (#cGp) updates properly');
  {
    const env = makeDom({ layout: 'mobile' });
    await sleep(25);
    const cGp = env.w.document.getElementById('cGp');
    assert(cGp, 'FAIL [MOBILE]: #cGp chip must exist in mobile top-bar');

    // Case 1: Insecure HTTP IP origin without pad -> reports HTTP BLOCKED
    env.setPads([]);
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 BLOCKED');

    // Case 2: Secure context / localhost without pad -> reports OFF
    env.w.isSecureContext = true;
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 OFF');

    // Case 3: Gamepad connected, deadman released
    env.setPads([createMockGamepad({ buttons: {}, axes: [0, 0, 0, 0] })]);
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 OK');

    // Case 4: Deadman held & active
    env.setPads([createMockGamepad({ buttons: { 7: 1.0 }, axes: [0, -0.8, 0, 0] })]);
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 HELD');

    // Case 5: Interlocked (blur while held)
    env.w.dispatchEvent(new env.w.Event('blur'));
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 RE-ENG');

    // Case 6: HTTP Blocked via exception
    env.w.navigator.getGamepads = () => {
      throw new Error('SecurityError: Gamepad blocked on HTTP');
    };
    await env.stepGamepad();
    assert.strictEqual(cGp.textContent, '🎮 BLOCKED');
    env.dom.window.close();
  }

  console.log('  [GENERAL] Exactly one controller polling loop started per page');
  {
    const env = makeDom({ layout: 'mobile' });
    await sleep(20);
    assert.strictEqual(env.w._roverGamepadLoopStarted, true);
    assert(typeof env.w._roverGamepadInterval !== 'undefined');
    env.dom.window.close();
  }
}

async function runTests() {
  console.log('=== Running Rover 1 HUD Xbox Controller Regression Suite (Both Layouts) ===');
  await runLayoutSuite('desktop');
  await runLayoutSuite('mobile');
  await runDesktopSpecificTests();
  await runMobileSpecificTests();
  console.log('\nALL GAMEPAD TESTS PASSED ACROSS BOTH DESKTOP AND MOBILE HUDs!');
}

if (require.main === module) {
  runTests().catch((err) => {
    console.error('\n' + err.message);
    process.exit(1);
  });
}

module.exports = { runTests };
