/**
 * motor_test.js
 * Tests multiple motor command approaches and logs exact bytes sent.
 * Run with: node motor_test.js
 *
 * Tries:
 *  1. FUNC_MOTOR (0x10) - direct motor speed bytes (-100..100)
 *  2. FUNC_MOTION (0x12) - chassis velocity (vX, vY, vZ as int16 * 1000)
 *  3. Different speed values and motor orderings
 */
const { SerialPort } = require('serialport');

const HEAD       = 0xFF;
const DEVICE_ID  = 0xFC;
const BOARD_ID   = 0xFB;
const FUNC_MOTOR  = 0x10;
const FUNC_MOTION = 0x12;
const FUNC_BEEP   = 0x02;

function buildPacket(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

function toUnsigned8(v) {
  // Convert signed -128..127 to unsigned byte 0..255
  v = Math.max(-100, Math.min(100, v));
  return v < 0 ? 256 + v : v;
}

function int16ToBytes(v) {
  // Little-endian signed int16
  const u = v < 0 ? (65536 + v) : v;
  return [u & 0xFF, (u >> 8) & 0xFF];
}

const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

let rxBuf = Buffer.alloc(0);
port.on('data', (chunk) => {
  rxBuf = Buffer.concat([rxBuf, chunk]);
  // Check for any unexpected response packets
  while (rxBuf.length >= 4) {
    const h = rxBuf.indexOf(0xFF);
    if (h === -1 || h >= rxBuf.length - 1) break;
    if (rxBuf[h + 1] !== BOARD_ID) { rxBuf = rxBuf.subarray(h + 1); continue; }
    if (rxBuf.length < h + 3) break;
    const extLen = rxBuf[h + 2];
    const totalLen = h + 2 + extLen;
    if (rxBuf.length < totalLen) break;
    const extType = rxBuf[h + 3];
    // Only log non-routine packets (not battery/encoder/misc)
    if (extType !== 0x0A && extType !== 0x0C && extType !== 0x0D) {
      const hex = Array.from(rxBuf.subarray(h, totalLen)).map(b => b.toString(16).padStart(2,'0')).join(' ');
      console.log(`  [RX type=0x${extType.toString(16)}] ${hex}`);
    }
    rxBuf = rxBuf.subarray(totalLen);
  }
});

async function send(label, funcId, payload) {
  const pkt = buildPacket(funcId, payload);
  const hex = Array.from(pkt).map(b => b.toString(16).padStart(2,'0')).join(' ');
  console.log(`\n[SEND] ${label}`);
  console.log(`  Bytes: ${hex}`);
  await new Promise((res, rej) => port.write(pkt, err => err ? rej(err) : res()));
  await new Promise(r => setTimeout(r, 1500));
}

port.on('open', async () => {
  console.log('Port open. Starting motor command tests...\n');
  await new Promise(r => setTimeout(r, 500));

  // --- Test 1: Beep to confirm two-way comms ---
  await send('BEEP 200ms (comms check)', FUNC_BEEP, [200, 0]);

  // --- Test 2: FUNC_MOTOR 0x10 - M1 at 50% forward ---
  await send('FUNC_MOTOR 0x10: M1=50, M2=0, M3=0, M4=0', FUNC_MOTOR,
    [toUnsigned8(50), toUnsigned8(0), toUnsigned8(0), toUnsigned8(0)]);

  // --- Test 3: All motors forward ---
  await send('FUNC_MOTOR 0x10: ALL=50', FUNC_MOTOR,
    [toUnsigned8(50), toUnsigned8(50), toUnsigned8(50), toUnsigned8(50)]);

  // --- Test 4: FUNC_MOTION 0x12 - vX=0.3 m/s forward ---
  // vX, vY, vZ as int16 little-endian (value * 1000)
  const vX = 300; // 0.3 m/s forward
  const vY = 0;
  const vZ = 0;
  await send('FUNC_MOTION 0x12: vX=0.3m/s, vY=0, vZ=0', FUNC_MOTION, [
    ...int16ToBytes(vX), ...int16ToBytes(vY), ...int16ToBytes(vZ)
  ]);

  // --- Test 5: Stop all ---
  await send('FUNC_MOTOR 0x10: STOP (all zeros)', FUNC_MOTOR, [0, 0, 0, 0]);
  await send('FUNC_MOTION 0x12: STOP', FUNC_MOTION, [0, 0, 0, 0, 0, 0]);

  // --- Test 6: Try opcode scan for motor-like responses ---
  console.log('\n--- Trying alternate motor opcodes ---');
  for (const op of [0x11, 0x13, 0x14, 0x20, 0x30]) {
    await send(`Opcode 0x${op.toString(16)} with M1=50`, op,
      [toUnsigned8(50), toUnsigned8(0), toUnsigned8(0), toUnsigned8(0)]);
    // Stop after each attempt
    await send(`Opcode 0x${op.toString(16)} STOP`, op, [0, 0, 0, 0]);
  }

  console.log('\nAll tests complete.');
  port.close();
  process.exit(0);
});

port.on('error', err => { console.error('Port error:', err.message); process.exit(1); });
