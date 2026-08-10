// test_scheduling_telemetry_protocol.js
// Unit tests asserting backward-compatible 24-byte & extended 40-byte 0x33 telemetry wire framing and payload parsing.

const assert = require('assert');

function parseLoopTimingPayload(buf) {
  if (buf.length < 24) return null;

  const lastDurationUs = buf.readUInt32LE(0);
  const minDurationUs = buf.readUInt32LE(4);
  const avgDurationUs = buf.readUInt32LE(8);
  const maxDurationUs = buf.readUInt32LE(12);
  const missedDeadlines = buf.readUInt32LE(16);
  const totalIterations = buf.readUInt32LE(20);

  const schedulingMetricsAvailable = (buf.length >= 40);

  let lastStartLatenessUs = 0;
  let maxStartLatenessUs = 0;
  let missedControlPeriods = 0;
  let maxConsecutiveMissedPeriods = 0;

  if (schedulingMetricsAvailable) {
    lastStartLatenessUs = buf.readUInt32LE(24);
    maxStartLatenessUs = buf.readUInt32LE(28);
    missedControlPeriods = buf.readUInt32LE(32);
    maxConsecutiveMissedPeriods = buf.readUInt32LE(36);
  }

  return {
    lastDurationUs,
    minDurationUs,
    avgDurationUs,
    maxDurationUs,
    missedDeadlines,
    totalIterations,
    schedulingMetricsAvailable,
    lastStartLatenessUs,
    maxStartLatenessUs,
    missedControlPeriods,
    maxConsecutiveMissedPeriods
  };
}

// Simulates full server.js framing parser for wire buffers
function parseWireFrame(frameBuf) {
  assert.strictEqual(frameBuf[0], 0xFF, 'Header byte 0 must be 0xFF');
  assert.strictEqual(frameBuf[1], 0xFB, 'Header byte 1 must be 0xFB');

  const extLen = frameBuf[2];
  const extType = frameBuf[3];
  const dataLen = extLen - 3; // extLen includes extLen(1) + type(1) + dataLen + checksum(1) -> dataLen = extLen - 3
  const totalLen = frameBuf.length;

  assert.strictEqual(totalLen, dataLen + 5, 'Total frame wire length must equal dataLen + 5');

  // Verify checksum: sum of bytes from extLen through last data byte
  let checksum = 0;
  for (let i = 2; i < totalLen - 1; i++) {
    checksum += frameBuf[i];
  }
  checksum = checksum & 0xFF;
  assert.strictEqual(checksum, frameBuf[totalLen - 1], 'Checksum must match');

  const payloadBuf = frameBuf.subarray(4, 4 + dataLen);
  return {
    extLen,
    extType,
    dataLen,
    totalLen,
    payload: parseLoopTimingPayload(payloadBuf)
  };
}

console.log('=== Running Whole-Loop 100 Hz Scheduling Telemetry Protocol & Wire Framing Tests ===');

// 1. Test legacy 24-byte payload backward compatibility & availability flag
const legacyPayload = Buffer.alloc(24);
legacyPayload.writeUInt32LE(67, 0);     // lastDurationUs
legacyPayload.writeUInt32LE(45, 4);     // minDurationUs
legacyPayload.writeUInt32LE(67, 8);     // avgDurationUs
legacyPayload.writeUInt32LE(480, 12);   // maxDurationUs
legacyPayload.writeUInt32LE(0, 16);     // missedDeadlines
legacyPayload.writeUInt32LE(50000, 20); // totalIterations

const parsedLegacy = parseLoopTimingPayload(legacyPayload);
assert.strictEqual(parsedLegacy.lastDurationUs, 67);
assert.strictEqual(parsedLegacy.maxDurationUs, 480);
assert.strictEqual(parsedLegacy.schedulingMetricsAvailable, false, 'Legacy payload sets schedulingMetricsAvailable = false');
assert.strictEqual(parsedLegacy.lastStartLatenessUs, 0, 'Legacy payload defaults lateness to 0');
assert.strictEqual(parsedLegacy.missedControlPeriods, 0, 'Legacy payload defaults missed periods to 0');
console.log(' [PASS] 1. Legacy 24-byte payload parsed cleanly with schedulingMetricsAvailable = false');

// 2. Test extended 40-byte payload parsing & availability flag
const extendedPayload = Buffer.alloc(40);
extendedPayload.writeUInt32LE(67, 0);     // lastDurationUs
extendedPayload.writeUInt32LE(45, 4);     // minDurationUs
extendedPayload.writeUInt32LE(67, 8);     // avgDurationUs
extendedPayload.writeUInt32LE(480, 12);   // maxDurationUs
extendedPayload.writeUInt32LE(0, 16);     // missedDeadlines
extendedPayload.writeUInt32LE(50000, 20); // totalIterations
extendedPayload.writeUInt32LE(15200, 24); // lastStartLatenessUs
extendedPayload.writeUInt32LE(19850, 28); // maxStartLatenessUs
extendedPayload.writeUInt32LE(12, 32);    // missedControlPeriods
extendedPayload.writeUInt32LE(2, 36);     // maxConsecutiveMissedPeriods

const parsedExtended = parseLoopTimingPayload(extendedPayload);
assert.strictEqual(parsedExtended.lastDurationUs, 67);
assert.strictEqual(parsedExtended.schedulingMetricsAvailable, true, 'Extended payload sets schedulingMetricsAvailable = true');
assert.strictEqual(parsedExtended.lastStartLatenessUs, 15200);
assert.strictEqual(parsedExtended.maxStartLatenessUs, 19850);
assert.strictEqual(parsedExtended.missedControlPeriods, 12);
assert.strictEqual(parsedExtended.maxConsecutiveMissedPeriods, 2);
console.log(' [PASS] 2. Extended 40-byte payload parsed cleanly with schedulingMetricsAvailable = true');

// 3. Golden Wire Frame Test for Legacy 24-Byte Payload (extLen = 0x1B = 27, frame = 29B)
const legacyWireFrame = Buffer.alloc(29);
legacyWireFrame[0] = 0xFF;
legacyWireFrame[1] = 0xFB;
legacyWireFrame[2] = 27;   // extLen = dataLen(24) + 3 = 27 (0x1B)
legacyWireFrame[3] = 0x33; // type
legacyPayload.copy(legacyWireFrame, 4);
let legSum = 0;
for (let i = 2; i < 28; i++) legSum += legacyWireFrame[i];
legacyWireFrame[28] = legSum & 0xFF;

const wireLegacy = parseWireFrame(legacyWireFrame);
assert.strictEqual(wireLegacy.extLen, 27);
assert.strictEqual(wireLegacy.extType, 0x33);
assert.strictEqual(wireLegacy.dataLen, 24);
assert.strictEqual(wireLegacy.totalLen, 29);
assert.strictEqual(wireLegacy.payload.schedulingMetricsAvailable, false);
console.log(' [PASS] 3. Golden 29-byte legacy wire frame (extLen=27, payload=24B) parsed cleanly');

// 4. Golden Wire Frame Test for Extended 40-Byte Payload (extLen = 0x2B = 43, frame = 45B)
const extendedWireFrame = Buffer.alloc(45);
extendedWireFrame[0] = 0xFF;
extendedWireFrame[1] = 0xFB;
extendedWireFrame[2] = 43;   // extLen = dataLen(40) + 3 = 43 (0x2B)
extendedWireFrame[3] = 0x33; // type
extendedPayload.copy(extendedWireFrame, 4);
let extSum = 0;
for (let i = 2; i < 44; i++) extSum += extendedWireFrame[i];
extendedWireFrame[44] = extSum & 0xFF;

const wireExtended = parseWireFrame(extendedWireFrame);
assert.strictEqual(wireExtended.extLen, 43);
assert.strictEqual(wireExtended.extType, 0x33);
assert.strictEqual(wireExtended.dataLen, 40);
assert.strictEqual(wireExtended.totalLen, 45);
assert.strictEqual(wireExtended.payload.schedulingMetricsAvailable, true);
assert.strictEqual(wireExtended.payload.lastStartLatenessUs, 15200);
console.log(' [PASS] 4. Golden 45-byte extended wire frame (extLen=43, payload=40B) parsed cleanly');

console.log('\nALL SCHEDULING TELEMETRY PROTOCOL & WIRE FRAMING TESTS PASSED 100%!');
