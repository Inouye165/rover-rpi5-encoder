// ==============================================================================
// test_manual_turn_smoothing.js
// Regression test suite for smooth, progressive manual turning for Rover 1:
// 1. Zero/dead-zone behavior (|x| < 0.10, |y| < 0.10 -> 0)
// 2. Continuity just beyond dead zone (no 35% minimum speed jump)
// 3. Small stick movements request slow turns (fine control near center)
// 4. Monotonic response and left/right symmetry
// 5. Existing maximum limits remain enforced (0.80 m/s lin, 3.00 rad/s ang)
// 6. Angular acceleration bounded using controlled elapsed time (dt)
// 7. Smooth reversal through zero without sudden jump
// 8. Safety stops immediately bypass smoothing (E-stop, disarm, deadman, watchdog)
// 9. Ramp state resets after interruption (no stale velocity resumption)
// 10. Forward/reverse behavior preserved (pure straight and steering-while-moving)
// 11. Consistency between server.js and public/hud-core.js estimateCommandedVelocity
// ==============================================================================

const assert = require('assert');
const path = require('path');
const fs = require('fs');

process.env.PORT = '3882';
process.env.ROVER_INTERNAL_CMD_HOST = '127.0.0.1';
process.env.ROVER_INTERNAL_CMD_PORT = '3883';
process.env.ROVER_NAV_BRIDGE_URL = 'http://127.0.0.1:3887';
process.env.GEEKWORM_BATTERY_ENABLED = 'false';
process.env.ROVER_OPERATOR_TOKEN = 'cd'.repeat(32);

const srv = require(path.join(__dirname, '..', 'server.js'));

// Load public/hud-core.js in an isolated sandbox context
const hudCoreCode = fs.readFileSync(path.join(__dirname, '..', 'public', 'hud-core.js'), 'utf8');
const hudSandbox = {};
const hudFn = new Function('global', hudCoreCode);
hudFn(hudSandbox);
const RoverHUD = hudSandbox.RoverHUD || globalThis.RoverHUD;

function runUnitTests() {
  console.log('--- Unit Tests: Input Shaping (computeManualJoystickVelocity) ---');

  // 1. Zero/dead-zone behavior
  console.log('Unit 1: Zero / dead-zone behavior');
  const z0 = srv.computeManualJoystickVelocity(0, 0);
  assert.strictEqual(z0.rawTargetLin, 0.0);
  assert.strictEqual(z0.rawTargetAng, 0.0);
  assert.strictEqual(z0.turnScaled, 0.0);
  assert.strictEqual(z0.throttle, 0.0);

  const zDeadX = srv.computeManualJoystickVelocity(0.09, 0);
  assert.strictEqual(zDeadX.rawTargetAng, 0.0, 'stick X within deadzone must yield 0 turn');
  const zDeadNegX = srv.computeManualJoystickVelocity(-0.09, 0);
  assert.strictEqual(zDeadNegX.rawTargetAng, 0.0, '-stick X within deadzone must yield 0 turn');

  const zDeadY = srv.computeManualJoystickVelocity(0, 0.09);
  assert.strictEqual(zDeadY.rawTargetLin, 0.0, 'stick Y within deadzone must yield 0 linear');
  const zDeadNegY = srv.computeManualJoystickVelocity(0, -0.09);
  assert.strictEqual(zDeadNegY.rawTargetLin, 0.0, '-stick Y within deadzone must yield 0 linear');

  // 2. Continuity just beyond dead zone (no minimum-speed jump)
  console.log('Unit 2: Continuity just beyond dead zone (no 35% minimum speed jump)');
  const justPastDead = srv.computeManualJoystickVelocity(0.1001, 0);
  assert(Math.abs(justPastDead.rawTargetAng) < 1e-4,
    `turn velocity just past dead zone must be continuous near 0, got ${justPastDead.rawTargetAng}`);
  assert(Math.abs(justPastDead.turnScaled) < 1e-4,
    `turnScaled just past dead zone must be near 0, got ${justPastDead.turnScaled}`);
  assert(Math.abs(justPastDead.rawTargetAng) < 0.35 * 3.00,
    'must not exhibit the legacy MIN_COEFF = 0.35 jump');

  // 3. Small stick movements produce small turn requests (fine control near center)
  console.log('Unit 3: Small inputs produce small turn requests');
  const smallInput15 = srv.computeManualJoystickVelocity(0.15, 0); // 5% into active 0.90 travel
  assert(Math.abs(smallInput15.rawTargetAng) < 0.02,
    `x=0.15 should request < 0.02 rad/s turn, got ${smallInput15.rawTargetAng}`);

  const smallInput20 = srv.computeManualJoystickVelocity(0.20, 0); // 11% into travel
  assert(Math.abs(smallInput20.rawTargetAng) < 0.05,
    `x=0.20 should request < 0.05 rad/s turn, got ${smallInput20.rawTargetAng}`);

  const moderateInput30 = srv.computeManualJoystickVelocity(0.30, 0); // 22% into travel
  assert(Math.abs(moderateInput30.rawTargetAng) < 0.20,
    `x=0.30 should request < 0.20 rad/s turn, got ${moderateInput30.rawTargetAng}`);

  // 4. Monotonic response and left/right symmetry
  console.log('Unit 4: Monotonic response and left/right symmetry');
  let prevMag = 0.0;
  for (let xInt = 10; xInt <= 100; xInt += 2) {
    const xVal = xInt / 100.0;
    const vPos = srv.computeManualJoystickVelocity(xVal, 0);
    const vNeg = srv.computeManualJoystickVelocity(-xVal, 0);

    const magPos = Math.abs(vPos.rawTargetAng);
    const magNeg = Math.abs(vNeg.rawTargetAng);

    assert(magPos >= prevMag - 1e-9, `response must be monotonic at x=${xVal}: ${magPos} vs prev ${prevMag}`);
    assert(Math.abs(magPos - magNeg) < 1e-9, `response must be symmetric at x=${xVal}: pos=${vPos.rawTargetAng}, neg=${vNeg.rawTargetAng}`);
    assert(Math.abs(vPos.rawTargetAng + vNeg.rawTargetAng) < 1e-9, `signs must be opposing for opposite stick: pos=${vPos.rawTargetAng}, neg=${vNeg.rawTargetAng}`);
    prevMag = magPos;
  }

  // 5. Existing maximum limits remain enforced
  console.log('Unit 5: Maximum limits remain enforced');
  const fullRight = srv.computeManualJoystickVelocity(1.0, 0, false);
  assert.strictEqual(fullRight.rawTargetAng, -3.00, 'full right must reach -3.00 rad/s');
  const fullLeft = srv.computeManualJoystickVelocity(-1.0, 0, false);
  assert.strictEqual(fullLeft.rawTargetAng, 3.00, 'full left must reach +3.00 rad/s');

  const fullFwd = srv.computeManualJoystickVelocity(0, 1.0, false);
  assert.strictEqual(fullFwd.rawTargetLin, 0.80, 'full forward must reach 0.80 m/s');
  const fullRev = srv.computeManualJoystickVelocity(0, -1.0, false);
  assert.strictEqual(fullRev.rawTargetLin, -0.80, 'full reverse must reach -0.80 m/s');

  // Floor testing limits
  const floorRight = srv.computeManualJoystickVelocity(1.0, 0, true);
  assert.strictEqual(floorRight.rawTargetAng, -0.90, 'floor full right must reach -0.90 rad/s');
  const floorFwd = srv.computeManualJoystickVelocity(0, 1.0, true);
  assert.strictEqual(floorFwd.rawTargetLin, 0.17, 'floor full forward must reach 0.17 m/s');

  // 6. Forward/reverse throttle behavior preserved
  console.log('Unit 6: Forward/reverse response preserved');
  const halfFwd = srv.computeManualJoystickVelocity(0, 0.50, false);
  assert(Math.abs(halfFwd.rawTargetLin - 0.20) < 1e-4, `y=0.50 throttle should be (0.50^2)*0.80 = 0.20, got ${halfFwd.rawTargetLin}`);

  // 7. HUD estimation matches server shaping
  console.log('Unit 7: RoverHUD.estimateCommandedVelocity mirrors server shaping');
  assert(typeof RoverHUD.estimateCommandedVelocity === 'function', 'RoverHUD must export estimateCommandedVelocity');
  for (const x of [0, 0.05, 0.1001, 0.15, 0.30, 0.70, 1.0, -0.15, -0.50, -1.0]) {
    for (const y of [0, 0.05, 0.25, 0.80, -0.60]) {
      const srvCmd = srv.computeManualJoystickVelocity(x, y, false);
      const hudEst = RoverHUD.estimateCommandedVelocity(x, y);
      assert(Math.abs(srvCmd.rawTargetLin - hudEst.linear) < 1e-4,
        `HUD linear mismatch at x=${x}, y=${y}: srv=${srvCmd.rawTargetLin}, hud=${hudEst.linear}`);
      assert(Math.abs(srvCmd.rawTargetAng - hudEst.angular) < 1e-4,
        `HUD angular mismatch at x=${x}, y=${y}: srv=${srvCmd.rawTargetAng}, hud=${hudEst.angular}`);
    }
  }

  console.log('--- Unit Tests: Angular Slew Rate Limiter (stepManualAngularVelocity) ---');

  // 8. Angular acceleration is bounded using controlled elapsed time
  console.log('Unit 8: Angular acceleration bounded using controlled elapsed time');
  let currentW = 0.0;
  const targetW = 3.00;
  const dt = 0.05; // 50 ms tick
  const maxAccel = 2.00; // 2.00 rad/s^2

  const step1 = srv.stepManualAngularVelocity(currentW, targetW, dt, { maxAngAccel: maxAccel, maxAngDecel: 3.00 });
  const observedAccel = (step1 - currentW) / dt;
  assert(observedAccel <= maxAccel + 1e-6, `acceleration must be <= ${maxAccel}, got ${observedAccel}`);
  assert(Math.abs(step1 - 0.10) < 1e-6, `step1 should be 0.10 rad/s, got ${step1}`);

  // Run full acceleration profile to target
  let elapsed = 0;
  currentW = 0.0;
  while (currentW < targetW) {
    const nextW = srv.stepManualAngularVelocity(currentW, targetW, dt, { maxAngAccel: maxAccel, maxAngDecel: 3.00 });
    const acc = (nextW - currentW) / dt;
    assert(acc <= maxAccel + 1e-6, `acceleration exceeded maxAccel at t=${elapsed}: ${acc}`);
    currentW = nextW;
    elapsed += dt;
    if (elapsed > 5.0) break;
  }
  assert.strictEqual(currentW, targetW, 'should reach target velocity');
  assert(Math.abs(elapsed - 1.50) < 0.06, `0 to 3.0 rad/s at 2.0 rad/s^2 should take ~1.50 s, took ${elapsed} s`);

  // 9. Smooth reversal through zero
  console.log('Unit 9: Smooth reversal through zero');
  currentW = 1.50; // Turning left (+1.5 rad/s)
  const reversalTarget = -1.50; // Operator reverses to right (-1.5 rad/s)
  let zeroCrossed = false;
  let prevW = currentW;
  elapsed = 0;

  while (currentW > reversalTarget) {
    const nextW = srv.stepManualAngularVelocity(currentW, reversalTarget, dt, { maxAngAccel: 2.00, maxAngDecel: 3.00 });
    const rate = Math.abs(nextW - currentW) / dt;

    // While currentW > 0 (decelerating toward zero): rate must be <= maxDecel (3.00)
    if (currentW > 0) {
      assert(rate <= 3.00 + 1e-6, `decel rate exceeded: ${rate}`);
    } else {
      // Once currentW <= 0 (accelerating away from zero toward -1.5): rate must be <= maxAccel (2.00)
      assert(rate <= 2.00 + 1e-6, `accel rate exceeded: ${rate}`);
    }

    if (prevW > 0 && nextW <= 0) {
      zeroCrossed = true;
    }

    prevW = currentW;
    currentW = nextW;
    elapsed += dt;
    if (elapsed > 5.0) break;
  }
  assert(zeroCrossed, 'reversal must cross zero smoothly');
  assert.strictEqual(currentW, reversalTarget, 'reversal must reach target velocity');
  // Decel from 1.5 to 0 at 3.0 rad/s^2 = 0.5s. Accel from 0 to -1.5 at 2.0 rad/s^2 = 0.75s. Total ~1.25s.
  assert(Math.abs(elapsed - 1.25) < 0.06, `reversal should take ~1.25 s, took ${elapsed} s`);

  // 10. Controlled return to zero with clean threshold snap
  console.log('Unit 10: Return to zero with clean threshold snap');
  currentW = 0.80;
  elapsed = 0;
  while (currentW > 0.0) {
    currentW = srv.stepManualAngularVelocity(currentW, 0.0, dt, { maxAngAccel: 2.00, maxAngDecel: 3.00 });
    elapsed += dt;
    if (elapsed > 5.0) break;
  }
  assert.strictEqual(currentW, 0.0, 'must snap cleanly to 0.0 rad/s');
  assert(elapsed <= 0.35, `stopping from 0.8 rad/s at 3.0 rad/s^2 should take <= 0.35 s, took ${elapsed} s`);
}

const sleep = (ms) => new Promise(r => setTimeout(r, ms));

async function runIntegrationTests() {
  console.log('--- Integration Tests: Production server.js WebSocket & Safety Paths ---');
  const WebSocket = require('ws');
  if (typeof srv.setOdomPollingDisabled === 'function') srv.setOdomPollingDisabled(true);
  await new Promise(r => srv.server.listen(3882, '127.0.0.1', r));

  async function openAuthed() {
    const ws = new WebSocket('ws://127.0.0.1:3882');
    const inbox = [];
    ws.on('message', (d) => { try { inbox.push(JSON.parse(d)); } catch (e) {} });
    await new Promise((res, rej) => { ws.once('open', res); ws.once('error', rej); });
    ws.send(JSON.stringify({ type: 'auth', token: process.env.ROVER_OPERATOR_TOKEN }));
    for (let i = 0; i < 40 && !inbox.find(m => m.type === 'auth_result'); i++) await sleep(25);
    const ar = inbox.find(m => m.type === 'auth_result');
    assert(ar && ar.ok === true, 'operator auth failed');
    return ws;
  }

  const armRover = () => srv.setLatestNormalDriveStatus({ armed: true, mode: 3 });

  try {
    // Integration 1: Live turning input produces progressive requested angular target without 35% jump
    console.log('Integration 1: Progressive requested angular target over WebSocket');
    armRover();
    const ws = await openAuthed();

    // Small input: x = 0.15 (5% into active travel)
    ws.send(JSON.stringify({ type: 'joystick', x: 0.15, y: 0.0, keepalive: true }));
    await sleep(60);
    let tgt = srv.getManualDriveTarget();
    assert(Math.abs(tgt.targetAngular) > 0 && Math.abs(tgt.targetAngular) < 0.03,
      `targetAngular for x=0.15 should be small (<0.03), got ${tgt.targetAngular}`);
    assert(Math.abs(tgt.targetAngular) < 0.35 * 3.00,
      'must not exhibit legacy 35% jump');

    // Integration 2: Slew-limiter progressively ramps limitedAngular toward requested targetAngular
    console.log('Integration 2: Bounded angular slew in keepalive loop');
    // Sudden full right stick: x = 1.0 (requests -3.00 rad/s)
    ws.send(JSON.stringify({ type: 'joystick', x: 1.0, y: 0.0, keepalive: true }));
    await sleep(60); // 1 tick
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, -3.00, 'requested targetAngular should immediately be -3.00');
    // On the first 50-60 ms tick, limitedAngular should have ramped smoothly (not jumped to -3.00)
    assert(tgt.limitedAngular < 0.0, 'limitedAngular should begin ramping negative');
    assert(tgt.limitedAngular > -1.00, `limitedAngular after 1 tick should be <= 1.0 rad/s magnitude, got ${tgt.limitedAngular}`);

    // Integration 3: Steering while moving forward preserves direct linear control
    console.log('Integration 3: Steering while moving forward preserves direct linear control');
    ws.send(JSON.stringify({ type: 'joystick', x: 0.50, y: 0.50, keepalive: true }));
    await sleep(60);
    tgt = srv.getManualDriveTarget();
    assert(Math.abs(tgt.targetLinear - 0.20) < 1e-3, `targetLinear should be 0.20, got ${tgt.targetLinear}`);
    assert(Math.abs(tgt.limitedLinear - 0.20) < 1e-3, `limitedLinear should be direct 0.20, got ${tgt.limitedLinear}`);
    // Angular should be smoothly ramping toward targetAngular
    assert(tgt.targetAngular < -0.40, `targetAngular for x=0.50 should be ~ -0.59, got ${tgt.targetAngular}`);

    // Integration 4: Deadman release immediately zeroes both target and limited velocity without waiting for comfort ramp
    console.log('Integration 4: Deadman release bypasses ramp immediately');
    // Set active high turn
    ws.send(JSON.stringify({ type: 'joystick', x: 1.0, y: 0.5, deadman: true, keepalive: true }));
    await sleep(200); // let limitedAngular ramp up
    tgt = srv.getManualDriveTarget();
    assert(Math.abs(tgt.limitedAngular) > 0.1, `should have nonzero limitedAngular before release, got ${tgt.limitedAngular}`);

    // Release deadman
    ws.send(JSON.stringify({ type: 'joystick', x: 1.0, y: 0.5, deadman: false }));
    await sleep(60);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, 0.0, 'deadman release must immediately zero targetAngular');
    assert.strictEqual(tgt.limitedAngular, 0.0, 'deadman release must immediately zero limitedAngular (no comfort ramp)');
    assert.strictEqual(tgt.targetLinear, 0.0, 'deadman release must immediately zero targetLinear');
    assert.strictEqual(tgt.limitedLinear, 0.0, 'deadman release must immediately zero limitedLinear');
    assert.strictEqual(tgt.cmdSource, 'NONE');

    // Integration 5: Ramp state resets after interruption (no stale velocity resumption)
    console.log('Integration 5: Ramp state resets after interruption (fresh ramp from 0.0)');
    // Re-engage deadman with opposite turn
    ws.send(JSON.stringify({ type: 'joystick', x: -1.0, y: 0.0, deadman: true, keepalive: true }));
    await sleep(60); // 1 tick
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, 3.00, 'new request should be +3.00');
    // Must start ramping up from 0.0 (limitedAngular should be around +0.10, definitely < +0.50)
    assert(tgt.limitedAngular > 0.0, 'should start ramping positive');
    assert(tgt.limitedAngular < 0.50, `must ramp from 0.0, not resume stale velocity, got ${tgt.limitedAngular}`);

    // Integration 6: E-Stop immediately zeroes target and limited velocity
    console.log('Integration 6: Emergency Stop bypasses ramp immediately');
    ws.send(JSON.stringify({ type: 'emergency_stop' }));
    await sleep(60);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, 0.0, 'estop must immediately zero targetAngular');
    assert.strictEqual(tgt.limitedAngular, 0.0, 'estop must immediately zero limitedAngular');
    assert.strictEqual(tgt.cmdSource, 'NONE');

    // Integration 7: Disarm immediately zeroes target and limited velocity
    console.log('Integration 7: Disarm bypasses ramp immediately');
    armRover();
    ws.send(JSON.stringify({ type: 'joystick', x: 0.8, y: 0.0, deadman: true, keepalive: true }));
    await sleep(150);
    ws.send(JSON.stringify({ type: 'disarm_normal_drive' }));
    await sleep(60);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, 0.0, 'disarm must immediately zero targetAngular');
    assert.strictEqual(tgt.limitedAngular, 0.0, 'disarm must immediately zero limitedAngular');

    // Integration 8: Watchdog timeout immediately zeroes target and limited velocity
    console.log('Integration 8: Watchdog timeout bypasses ramp immediately');
    armRover();
    for (let i = 0; i < 3; i++) {
      ws.send(JSON.stringify({ type: 'joystick', x: 0.8, y: 0.0, keepalive: true }));
      await sleep(100);
    }
    tgt = srv.getManualDriveTarget();
    assert(Math.abs(tgt.limitedAngular) > 0.05, `should have nonzero limitedAngular, got ${tgt.limitedAngular}`);
    // Wait for keepalive expiry (> 600 ms)
    await sleep(800);
    tgt = srv.getManualDriveTarget();
    assert.strictEqual(tgt.targetAngular, 0.0, 'watchdog must zero targetAngular');
    assert.strictEqual(tgt.limitedAngular, 0.0, 'watchdog must zero limitedAngular');
    assert.strictEqual(tgt.cmdSource, 'NONE');

    // Cleanup
    ws.close();
    await sleep(100);
    await new Promise(r => srv.server.close(r));
    console.log('--- All Unit & Integration Tests PASSED Successfully ---');
  } catch (err) {
    try { await new Promise(r => srv.server.close(r)); } catch (e) {}
    throw err;
  }
}

async function main() {
  runUnitTests();
  await runIntegrationTests();
  console.log('ALL MANUAL TURN SMOOTHING TESTS PASSED');
  process.exit(0);
}

main().catch((err) => {
  console.error('\nFAILED TEST:', err);
  process.exit(1);
});
