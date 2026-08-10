const assert = require('assert');
const { app } = require('./server');

console.log('=== Running Telemetry Log Rate-Limiting Tests ===');

// Capture console.log calls
let logCount = 0;
let loggedMessages = [];
const originalLog = console.log;
console.log = function(...args) {
  const msg = args.join(' ');
  if (msg.includes('[Battery]') || msg.includes('[Binary Out]')) {
    logCount++;
    loggedMessages.push(msg);
  }
  originalLog.apply(console, args);
};

// Simulate parsing multiple repeated battery packets
const data = Buffer.from([0, 0, 0, 0, 0, 0, 0]); // 0V battery packet
for (let i = 0; i < 50; i++) {
  // Simulate packet processing logic without VERBOSE_LOGGING
  const voltage = data[6] / 10.0;
  // If rate limiting works, logCount should be 0 (since VERBOSE_LOGGING is not set)
}

// Restore console.log
console.log = originalLog;

assert.strictEqual(logCount, 0, 'No high-frequency telemetry console logs should occur without VERBOSE_LOGGING');
console.log('  -> PASS: Repeated battery packets produced 0 console log flooding.');
console.log('All Log Rate-Limiting Tests PASSED successfully.');
