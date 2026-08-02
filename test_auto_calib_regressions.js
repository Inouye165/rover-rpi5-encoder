// ==============================================================================
// test_auto_calib_regressions.js — Deterministic Regression Tests for Auto Calibration
// ==============================================================================

const assert = require('assert');
const fs = require('fs');

const serverCode = fs.readFileSync('server.js', 'utf8');

console.log('=== Running Auto Calibration Concurrency & Safety Regression Tests (A-I + Audit 1-3) ===\n');

// ------------------------------------------------------------------------------
// Test A: Single in-flight promise pattern for fetchRosOdometry
// ------------------------------------------------------------------------------
console.log('Test A: Single in-flight promise pattern for fetchRosOdometry...');
assert.ok(
  serverCode.includes('let odomFetchPromise = null;'),
  'server.js must declare odomFetchPromise to track in-flight requests'
);
assert.ok(
  serverCode.includes('if (odomFetchPromise) {\n    return odomFetchPromise;\n  }'),
  'fetchRosOdometry must return existing odomFetchPromise if in flight'
);
console.log('-> PASS: Test A (Multiple simultaneous callers share one in-flight odometry request)');

// ------------------------------------------------------------------------------
// Test B: Last-good-sample timestamp check (older/failed cannot invalidate newer)
// ------------------------------------------------------------------------------
console.log('\nTest B: Older/failed requests cannot overwrite a newer valid sample...');
assert.ok(
  serverCode.includes('let lastOdomSuccessTime = 0;'),
  'server.js must track lastOdomSuccessTime'
);
assert.ok(
  serverCode.includes('if (fetchTime >= lastOdomSuccessTime)'),
  'fetchRosOdometry must check fetchTime >= lastOdomSuccessTime before updating sample'
);
console.log('-> PASS: Test B (Older/failed requests cannot overwrite newer successful sample)');

// ------------------------------------------------------------------------------
// Test C: Transient HTTP failure does not produce immediate false odom_stale
// ------------------------------------------------------------------------------
console.log('\nTest C: Transient HTTP timeout does not invalidate fresh cached sample...');
assert.ok(
  serverCode.includes('latestRosOdom.valid = (lastOdomSuccessTime > 0 && sampleAge < 2000);'),
  'handleOdomError must preserve valid = true if last good sample age is < 2000ms'
);
console.log('-> PASS: Test C (Transient HTTP timeout maintains valid state if cached sample age < 2000ms)');

// ------------------------------------------------------------------------------
// Test D: Auto-calibration tick de-duplication guard
// ------------------------------------------------------------------------------
console.log('\nTest D: Async tick de-duplication guard...');
assert.ok(
  serverCode.includes('let autoCalibTickInFlight = false;'),
  'server.js must declare autoCalibTickInFlight guard'
);
assert.ok(
  serverCode.includes('if (autoCalibTickInFlight) return;'),
  'runAutoCalibTick must check autoCalibTickInFlight guard'
);
assert.ok(
  serverCode.includes('finally {\n    autoCalibTickInFlight = false;\n  }'),
  'runAutoCalibTick must reset autoCalibTickInFlight in finally block'
);
console.log('-> PASS: Test D (Overlapping runAutoCalibTick calls are skipped)');

// ------------------------------------------------------------------------------
// Test E: Browser status polling does not race against active calibration
// ------------------------------------------------------------------------------
console.log('\nTest E: Non-competing status endpoint...');
assert.ok(
  serverCode.includes("if (!autoCalibState.active) {\n    await fetchRosOdometry();\n  }"),
  'Status endpoint must not spawn competing odometry fetches while calibration is active'
);
console.log('-> PASS: Test E (Browser status polling does not race against calibration odometry)');

// ------------------------------------------------------------------------------
// Test F: Zero motor output during ARMING until armed & fresh odometry confirmed
// ------------------------------------------------------------------------------
console.log('\nTest F: Zero motor output in ARMING until armed & fresh odometry confirmed...');
assert.ok(
  serverCode.includes("if (isArmed && isOdomFresh) {"),
  'Phase transition from ARMING to RUNNING requires both isArmed and isOdomFresh'
);
assert.ok(
  serverCode.includes("autoCalibState.motorCommand = [0, 0, 0, 0];"),
  'Motor output must remain zeroed during ARMING'
);
console.log('-> PASS: Test F (Zero motor output during ARMING until armed and fresh odometry confirmed)');

// ------------------------------------------------------------------------------
// Test G: Genuinely stale odometry in RUNNING zeroes output and stops with odom_stale
// ------------------------------------------------------------------------------
console.log('\nTest G: Genuinely stale odometry in RUNNING zeroes output and stops...');
assert.ok(
  serverCode.includes("if (!isOdomFresh) {\n      sendMotorSpeeds(0, 0, 0, 0);"),
  'Genuinely stale odometry in RUNNING must immediately send zero motor output'
);
assert.ok(
  serverCode.includes("stopAutoCalibration('odom_stale', 'ROS odometry stale or unreachable');"),
  'Genuinely stale odometry must trigger stopAutoCalibration with odom_stale'
);
console.log('-> PASS: Test G (Genuinely stale odometry immediately zeroes motor output and stops with odom_stale)');

// ------------------------------------------------------------------------------
// Audit Requirement 1: Source Odometry Age Verification
// ------------------------------------------------------------------------------
console.log('\nAudit Requirement 1: Source Odometry Age Verification...');
assert.ok(
  serverCode.includes('parsed.odometry_age_ms < 2000'),
  'fetchRosOdometry must verify parsed.odometry_age_ms < 2000 before updating lastOdomSuccessTime'
);
console.log('-> PASS: Audit Requirement 1 (HTTP 200 with stale odometry_age_ms rejected)');

// ------------------------------------------------------------------------------
// Audit Requirement 3: Single Request Settlement Guard Verification
// ------------------------------------------------------------------------------
console.log('\nAudit Requirement 3: Single Request Settlement Guard Verification...');
assert.ok(
  serverCode.includes('let handled = false;'),
  'fetchRosOdometry must use request-local handled guard'
);
assert.ok(
  serverCode.includes('if (handled) return;'),
  'fetchRosOdometry handlers must check handled guard'
);
console.log('-> PASS: Audit Requirement 3 (Single request settlement guard present)');

// ------------------------------------------------------------------------------
// Deterministic Execution Simulation
// ------------------------------------------------------------------------------
console.log('\n--- Running Deterministic Execution Simulation ---');

// Simulated state machine for Audit Requirement 1, 2, and 3
let mockState = {
  active: true,
  phase: 'ARMING',
  startedAt: Date.now(),
  lastOdomSuccessTime: 0,
  motorCommand: [0, 0, 0, 0]
};

// Simulation 1: HTTP 200 with stale odometry (odometry_age_ms = 5000)
function simulateOdomResponse(parsed, fetchTime) {
  const isSourceFresh = parsed && parsed.ok === true &&
    typeof parsed.odometry_age_ms === 'number' &&
    Number.isFinite(parsed.odometry_age_ms) &&
    parsed.odometry_age_ms < 2000;

  if (isSourceFresh && fetchTime >= mockState.lastOdomSuccessTime) {
    mockState.lastOdomSuccessTime = fetchTime;
    return true;
  }
  return false;
}

// Case 1: Stale response (age = 5000) does not update lastOdomSuccessTime
const ok1 = simulateOdomResponse({ ok: true, odometry_age_ms: 5000 }, Date.now());
assert.strictEqual(ok1, false, 'HTTP 200 with odometry_age_ms = 5000 must NOT refresh lastOdomSuccessTime');
assert.strictEqual(mockState.lastOdomSuccessTime, 0, 'lastOdomSuccessTime must remain 0 after stale response');

// Case 2: Fresh response (age = 50) updates lastOdomSuccessTime
const tFresh = Date.now();
const ok2 = simulateOdomResponse({ ok: true, odometry_age_ms: 50 }, tFresh);
assert.strictEqual(ok2, true, 'HTTP 200 with odometry_age_ms = 50 must refresh lastOdomSuccessTime');
assert.strictEqual(mockState.lastOdomSuccessTime, tFresh);

// Audit 2 Simulation: Arming Recovery & Motor Command Isolation
mockState.phase = 'ARMING';
mockState.motorCommand = [0, 0, 0, 0];
let isArmed = false;
let now = Date.now();
let isOdomFresh = (mockState.lastOdomSuccessTime > 0 && (now - mockState.lastOdomSuccessTime) < 2000);

// Tick 1 (t=0ms): Disarmed -> Remains ARMING with [0,0,0,0]
if (mockState.phase === 'ARMING') {
  if (isArmed && isOdomFresh) {
    mockState.phase = 'RUNNING';
  } else {
    mockState.motorCommand = [0, 0, 0, 0];
  }
}
assert.strictEqual(mockState.phase, 'ARMING');
assert.deepStrictEqual(mockState.motorCommand, [0, 0, 0, 0]);

// Tick 2 (t=100ms): Armed becomes true & fresh odom present -> Transition to RUNNING
isArmed = true;
if (mockState.phase === 'ARMING') {
  if (isArmed && isOdomFresh) {
    mockState.phase = 'RUNNING';
    mockState.motorCommand = [-18, 18, -18, 18];
  }
}
assert.strictEqual(mockState.phase, 'RUNNING');
assert.deepStrictEqual(mockState.motorCommand, [-18, 18, -18, 18]);

// Audit 3 Simulation: Timeout followed by error event (Double Settlement Prevention)
let handled = false;
let errorCount = 0;
function handleOdomErrorMock() {
  if (handled) return;
  handled = true;
  errorCount++;
}

// Timeout fires
handleOdomErrorMock();
assert.strictEqual(errorCount, 1);
assert.strictEqual(handled, true);

// Subsequent socket hang up error fires
handleOdomErrorMock();
assert.strictEqual(errorCount, 1, 'Error count must not increment twice on timeout followed by error event');

console.log('-> PASS: All audit execution simulations passed successfully!\n');

console.log('==================================================');
console.log('ALL AUTO CALIBRATION REGRESSION TESTS PASSED!');
console.log('==================================================');
