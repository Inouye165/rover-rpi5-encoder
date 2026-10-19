const assert = require('assert');

console.log('=== Running Geekworm X1206 Battery Subsystem Tests ===');

// 1. Test MAX17043 Math Calculations
console.log('Test 1: MAX17043 VCELL & SOC formula validation...');
function calcVoltage(rawWordLE) {
  const swapped = ((rawWordLE & 0xFF) << 8) | (rawWordLE >> 8);
  return swapped * 78.125 / 1000000.0;
}

function calcSOC(rawWordLE) {
  const swapped = ((rawWordLE & 0xFF) << 8) | (rawWordLE >> 8);
  return swapped / 256.0;
}

const rawV = 0x40CE;
const v = calcVoltage(rawV);
assert(Math.abs(v - 4.125) < 0.001, 'Voltage calculation failed');
console.log('  -> Voltage calculation verified: raw=0x40CE -> ' + v.toFixed(3) + ' V');

const rawSOC = 0x8055;
const soc = calcSOC(rawSOC);
assert(Math.abs(soc - 85.5) < 0.01, 'SOC calculation failed');
console.log('  -> SOC calculation verified: raw=0x8055 -> ' + soc.toFixed(1) + '%');

// 2. Test UI Case Logic Simulation
console.log('Test 2: UI Battery Handler Percentage & Formatting...');
function simulateUiBatteryHandler(msg) {
  let pct = null;
  if (typeof msg.percentage === 'number' && !isNaN(msg.percentage)) {
    pct = Math.min(100, Math.max(0, msg.percentage));
  } else if (typeof msg.voltage === 'number' && msg.voltage > 2.0) {
    if (msg.voltage <= 4.35) {
      pct = Math.min(100, Math.max(0, ((msg.voltage - 3.2) / (4.2 - 3.2)) * 100));
    } else if (msg.voltage <= 8.5) {
      pct = Math.min(100, Math.max(0, ((msg.voltage - 6.0) / (8.4 - 6.0)) * 100));
    } else {
      pct = Math.min(100, Math.max(0, ((msg.voltage - 9.0) / (12.6 - 9.0)) * 100));
    }
  }

  if (pct === null || (typeof msg.voltage === 'number' && msg.voltage <= 2.0)) {
    return { pct: 0, text: 'Unknown', bg: '#64748b' };
  } else {
    const vText = (typeof msg.voltage === 'number') ? msg.voltage.toFixed(2) + 'V' : '';
    const pctText = pct.toFixed(0) + '%';
    const displayText = vText ? (pctText + ' (' + vText + ')') : pctText;
    let bg = '';
    if (pct >= 50) bg = 'green';
    else if (pct >= 20) bg = 'yellow';
    else bg = 'red';
    return { pct, text: displayText, bg };
  }
}

// Case A: Explicit Geekworm telemetry (1S 4.08V, 87.2%)
const resA = simulateUiBatteryHandler({ voltage: 4.08, percentage: 87.2, source: 'geekworm_x1206' });
assert.strictEqual(resA.text, '87% (4.08V)');
assert.strictEqual(resA.bg, 'green');
console.log('  -> Case A (Geekworm explicit): ' + resA.text + ' [' + resA.bg + ']');

// Case B: Low battery Geekworm (1S 3.35V, 15%)
const resB = simulateUiBatteryHandler({ voltage: 3.35, percentage: 15.0, source: 'geekworm_x1206' });
assert.strictEqual(resB.text, '15% (3.35V)');
assert.strictEqual(resB.bg, 'red');
console.log('  -> Case B (Low battery alert): ' + resB.text + ' [' + resB.bg + ']');

// Case C: Legacy ESP32 dummy packet (0.0V)
const resC = simulateUiBatteryHandler({ voltage: 0.0 });
assert.strictEqual(resC.text, 'Unknown');
assert.strictEqual(resC.bg, '#64748b');
console.log('  -> Case C (Dummy 0.0V): ' + resC.text + ' [' + resC.bg + ']');

// Case D: Fallback 1S voltage without explicit percentage
const resD = simulateUiBatteryHandler({ voltage: 3.7 });
assert(resD.pct > 45 && resD.pct < 55);
assert.strictEqual(resD.text, '50% (3.70V)');
console.log('  -> Case D (1S voltage fallback): ' + resD.text + ' [' + resD.bg + ']');

// 3. Test Dummy Packet Shield
console.log('Test 3: Telemetry Dummy Packet Shielding...');
let latestBatteryVoltage = 4.12;
let latestBatteryPercentage = 89.0;
let lastGeekwormBatteryTime = Date.now();

function shouldAcceptEsp32Packet(rawVoltage, now) {
  const isGeekwormActive = (now - lastGeekwormBatteryTime < 15000) && (latestBatteryPercentage !== null);
  if (rawVoltage <= 0.1 || isGeekwormActive) {
    return false; // rejected
  }
  return true;
}

assert.strictEqual(shouldAcceptEsp32Packet(0.0, Date.now()), false, 'Should reject 0.0V dummy packet');
assert.strictEqual(shouldAcceptEsp32Packet(7.4, Date.now()), false, 'Should reject ESP32 packet when Geekworm is active');
assert.strictEqual(shouldAcceptEsp32Packet(7.4, Date.now() + 20000), true, 'Should accept ESP32 packet if Geekworm went offline >15s');
console.log('  -> Dummy packet shield verified.');

console.log('\nAll Geekworm X1206 Battery Tests PASSED successfully.');
