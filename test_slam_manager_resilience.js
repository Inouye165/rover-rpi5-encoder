const assert = require('assert');
const slamManager = require('./slam_manager');

async function runTests() {
  console.log('[Test] Starting SlamManager unit & resilience tests...');

  // Mock runCommand to count process invocations and simulate responses
  let execCount = 0;
  let simulatedProcesses = new Set();
  let simulatedLifecycle = 'unconfigured';

  slamManager.runCommand = async (cmd, timeoutMs) => {
    execCount++;
    if (cmd.includes('pgrep')) {
      if (simulatedProcesses.has('async_slam_toolbox_node')) {
        return { ok: true, stdout: '12345\n' };
      }
      return { ok: false, error: 'No matching processes', stdout: '' };
    }
    if (cmd.includes('ros2 lifecycle get')) {
      if (simulatedProcesses.has('async_slam_toolbox_node')) {
        return { ok: true, stdout: `State: ${simulatedLifecycle}\n` };
      }
      return { ok: false, error: 'Node not found', stdout: '' };
    }
    if (cmd.includes('ros2 launch')) {
      simulatedProcesses.add('async_slam_toolbox_node');
      setTimeout(() => {
        simulatedLifecycle = 'active';
      }, 500);
      return { ok: true, stdout: 'Launch started' };
    }
    if (cmd.includes('pkill')) {
      simulatedProcesses.clear();
      simulatedLifecycle = 'unconfigured';
      return { ok: true, stdout: '' };
    }
    return { ok: true, stdout: '' };
  };

  // 1. Initial State Check (STOPPED)
  execCount = 0;
  const initialStatus = await slamManager.getStatus();
  assert.strictEqual(initialStatus.state, 'STOPPED');
  console.log('✔ Test 1 passed: Initial state is STOPPED.');

  // 2. Concurrent Status Polling (Deduplication Check)
  execCount = 0;
  slamManager.lastCheckTime = 0; // invalidate cache
  const results = await Promise.all([
    slamManager.getStatus(),
    slamManager.getStatus(),
    slamManager.getStatus(),
    slamManager.getStatus()
  ]);
  assert.strictEqual(execCount, 1, `Expected 1 exec call for 4 concurrent polls, got ${execCount}`);
  assert.strictEqual(results[0].state, 'STOPPED');
  console.log(`✔ Test 2 passed: 4 concurrent status polls executed only ${execCount} process call.`);

  // 3. Start SLAM & Concurrent Start Attempts
  execCount = 0;
  const startPromise = slamManager.startSlam();
  
  // Try calling startSlam while already starting
  const duplicateStart = await slamManager.startSlam();
  assert.strictEqual(duplicateStart.ok, false);
  assert.strictEqual(duplicateStart.state, 'STARTING');
  console.log('✔ Test 3 passed: Duplicate start attempt rejected with STARTING error.');

  const startResult = await startPromise;
  assert.strictEqual(startResult.ok, true);
  assert.strictEqual(startResult.state, 'RUNNING');
  console.log('✔ Test 4 passed: SLAM started successfully and reached RUNNING state.');

  // 4. Start while already RUNNING
  const runningStart = await slamManager.startSlam();
  assert.strictEqual(runningStart.ok, false);
  assert.strictEqual(runningStart.state, 'RUNNING');
  console.log('✔ Test 5 passed: Start attempt while RUNNING rejected.');

  // 5. Stop while RUNNING
  const stopResult = await slamManager.stopSlam();
  assert.strictEqual(stopResult.ok, true);
  assert.strictEqual(stopResult.state, 'STOPPED');
  console.log('✔ Test 6 passed: Stop SLAM returned to STOPPED.');

  console.log('\nALL SLAM MANAGER RESILIENCE TESTS PASSED CLEANLY! 🎉');
}

runTests().catch(err => {
  console.error('❌ Test failed:', err);
  process.exit(1);
});
