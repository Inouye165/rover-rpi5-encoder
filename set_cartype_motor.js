/**
 * set_cartype_motor.js
 * The V3.0 compatible expansion board firmware requires carType to be set/confirmed before
 * motor commands work. This script:
 *   1. Sets carType = 1 (Mecanum/4-wheel standard)
 *   2. Sends motor commands using FUNC_MOTOR (0x10)
 *   3. Sends motion commands using FUNC_MOTION (0x12)
 *   4. Also tries setting carType = 2 (differential) and repeating
 *
 * Run: node set_cartype_motor.js
 */
const { SerialPort } = require('serialport');

const HEAD      = 0xFF;
const DEVICE_ID = 0xFC;
const BOARD_ID  = 0xFB;

// Known function codes
const FUNC_BEEP      = 0x02;
const FUNC_MOTOR     = 0x10; // direct per-motor speed bytes
const FUNC_MOTION    = 0x12; // chassis velocity vX,vY,vZ int16*1000
const FUNC_CAR_TYPE  = 0x44; // Set car type (from Python SDK analysis)

// Also worth trying – these are from the expansion board Python set_motor docs
const FUNC_PWM_SERVO = 0x20;  // PWM servo

function buildPacket(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

function toU8(v) { const c = Math.max(-100, Math.min(100, v)); return c < 0 ? 256 + c : c; }
function i16le(v) { const u = v < 0 ? (65536 + v) : v; return [u & 0xFF, (u >> 8) & 0xFF]; }

const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

let rxBuf = Buffer.alloc(0);
port.on('data', (chunk) => {
  rxBuf = Buffer.concat([rxBuf, chunk]);
  while (rxBuf.length >= 4) {
    const h = rxBuf.indexOf(0xFF);
    if (h === -1 || h >= rxBuf.length - 1) break;
    if (rxBuf[h + 1] !== BOARD_ID) { rxBuf = rxBuf.subarray(h + 1); continue; }
    if (rxBuf.length < h + 3) break;
    const extLen = rxBuf[h + 2];
    const totalLen = h + 2 + extLen;
    if (rxBuf.length < totalLen) break;
    const extType = rxBuf[h + 3];
    if (extType !== 0x0A && extType !== 0x0C && extType !== 0x0D && extType !== 0x0E) {
      const hex = Array.from(rxBuf.subarray(h, totalLen)).map(b => b.toString(16).padStart(2,'0')).join(' ');
      console.log(`  [RX type=0x${extType.toString(16)}] ${hex}`);
    }
    rxBuf = rxBuf.subarray(totalLen);
  }
});

async function send(label, funcId, payload) {
  const pkt = buildPacket(funcId, payload);
  const hex = Array.from(pkt).map(b => b.toString(16).padStart(2,'0')).join(' ');
  console.log(`\n[TX] ${label}`);
  console.log(`  ${hex}`);
  await new Promise((res, rej) => port.write(pkt, err => err ? rej(err) : res()));
}

async function wait(ms) { return new Promise(r => setTimeout(r, ms)); }

port.on('open', async () => {
  console.log('=== V3.0 Car Type + Motor Test ===\n');
  await wait(500);

  // ── Step 1: Confirm comms with beep ──
  await send('BEEP (comms check)', FUNC_BEEP, [200, 0]);
  await wait(1000);

  // ── Step 2: Try setting car type to different values ──
  // Compatible Python SDK uses FUNC_CAR_TYPE to set the car mode
  // carType 1 = Mecanum 4WD, 2 = Ackermann, 3 = Tank diff
  for (const carType of [1, 2, 3, 4]) {
    console.log(`\n--- Trying carType=${carType} ---`);
    await send(`Set carType=${carType}`, FUNC_CAR_TYPE, [carType, 0]);
    await wait(200);

    // Motor forward (M1 = 50%)
    await send(`Motor M1=50 after carType=${carType}`, FUNC_MOTOR,
      [toU8(50), toU8(0), toU8(0), toU8(0)]);
    await wait(1500);

    // Stop
    await send('Motor STOP', FUNC_MOTOR, [0, 0, 0, 0]);
    await wait(500);

    // Motion forward
    await send(`Motion vX=300 after carType=${carType}`, FUNC_MOTION,
      [...i16le(300), ...i16le(0), ...i16le(0)]);
    await wait(1500);

    // Stop motion
    await send('Motion STOP', FUNC_MOTION, [...i16le(0), ...i16le(0), ...i16le(0)]);
    await wait(500);
  }

  // ── Step 3: Try with much larger speed values ──
  console.log('\n--- Trying with larger speed values (no carType set) ---');
  for (const spd of [100, -100]) {
    await send(`FUNC_MOTOR M1=${spd}`, FUNC_MOTOR,
      [toU8(spd), toU8(0), toU8(0), toU8(0)]);
    await wait(1500);
    await send('Stop', FUNC_MOTOR, [0, 0, 0, 0]);
    await wait(500);
  }

  // ── Step 4: Motion with larger vX ──
  for (const v of [500, 1000]) {
    await send(`FUNC_MOTION vX=${v}`, FUNC_MOTION,
      [...i16le(v), ...i16le(0), ...i16le(0)]);
    await wait(1500);
    await send('Stop', FUNC_MOTION, [...i16le(0), ...i16le(0), ...i16le(0)]);
    await wait(500);
  }

  console.log('\n=== Done ===');
  await wait(500);
  port.close();
  process.exit(0);
});

port.on('error', err => { console.error('Port error:', err.message); process.exit(1); });
