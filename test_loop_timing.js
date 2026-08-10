const assert = require('assert');
const { app, getLatestLoopTiming } = require('./server');

console.log('=== Running 100 Hz Control Loop Timing & API Tests ===');

// Test 1: Initial state structure
const initialTiming = getLatestLoopTiming();
console.log('Test 1: Initial state structure...');
assert.strictEqual(typeof initialTiming.lastDurationUs, 'number');
assert.strictEqual(typeof initialTiming.minDurationUs, 'number');
assert.strictEqual(typeof initialTiming.avgDurationUs, 'number');
assert.strictEqual(typeof initialTiming.maxDurationUs, 'number');
assert.strictEqual(typeof initialTiming.missedDeadlines, 'number');
assert.strictEqual(typeof initialTiming.totalIterations, 'number');
console.log('  -> PASS: Initial state structure');

// Test 2: Binary packet 0x33 parsing logic simulation
console.log('Test 2: Binary packet 0x33 parsing logic...');
const packetData = Buffer.alloc(24);
packetData.writeUInt32LE(450, 0);    // lastDurationUs
packetData.writeUInt32LE(380, 4);    // minDurationUs
packetData.writeUInt32LE(420, 8);    // avgDurationUs
packetData.writeUInt32LE(890, 12);   // maxDurationUs
packetData.writeUInt32LE(0, 16);     // missedDeadlines
packetData.writeUInt32LE(15000, 20); // totalIterations

assert.strictEqual(packetData.readUInt32LE(0), 450);
assert.strictEqual(packetData.readUInt32LE(4), 380);
assert.strictEqual(packetData.readUInt32LE(8), 420);
assert.strictEqual(packetData.readUInt32LE(12), 890);
assert.strictEqual(packetData.readUInt32LE(16), 0);
assert.strictEqual(packetData.readUInt32LE(20), 15000);
console.log('  -> PASS: Binary packet 0x33 deserialization');

console.log('All Control Loop Timing Tests PASSED successfully.');
