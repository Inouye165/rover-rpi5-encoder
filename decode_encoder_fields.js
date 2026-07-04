/**
 * decode_encoder_fields.js  (bounded, auto-exits ~9s)
 * Decodes the 0x0C telemetry packet numerically as int16 LE fields and
 * prints per-field statistics for idle vs drive vs stop phases, so we can
 * see exactly which field responds to motor commands.
 */
const { SerialPort } = require('serialport');

const HEAD = 0xFF;
const DEVICE_ID = 0xFC;
const BOARD_ID = 0xFB;
const FUNC_MOTOR = 0x10;
const TYPE_ENCODER = 0x0C;

function buildPacket(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xff);
  return Buffer.from(cmd);
}

function toU8(v) {
  const c = Math.max(-100, Math.min(100, v));
  return c < 0 ? 256 + c : c;
}

function i16(buf, off) {
  let v = (buf[off + 1] << 8) | buf[off];
  if (v & 0x8000) v -= 0x10000;
  return v;
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

(async () => {
  const port = new SerialPort({ path: 'COM18', baudRate: 115200 });
  let rx = Buffer.alloc(0);
  let phase = 'idle';

  // samples[phase] = array of decoded field arrays
  const samples = { idle: [], drive: [], stop: [] };

  function parse() {
    while (rx.length >= 4) {
      const h = rx.indexOf(0xff);
      if (h < 0 || h + 1 >= rx.length) break;
      if (rx[h + 1] !== BOARD_ID) { rx = rx.subarray(h + 1); continue; }
      if (rx.length < h + 3) break;
      const extLen = rx[h + 2];
      const total = h + 2 + extLen;
      if (rx.length < total) break;
      const extType = rx[h + 3];
      const dataLen = extLen - 2;
      if (dataLen >= 0 && extType === TYPE_ENCODER) {
        const data = rx.subarray(h + 4, h + 4 + dataLen);
        // Decode as int16 LE pairs plus leftover byte
        const fields = [];
        let off = 0;
        while (off + 1 < data.length) { fields.push(i16(data, off)); off += 2; }
        if (off < data.length) fields.push(data[off]); // trailing byte
        samples[phase].push(fields);
      }
      rx = rx.subarray(total);
    }
  }

  port.on('data', (c) => { rx = Buffer.concat([rx, c]); parse(); });
  port.on('error', (e) => { console.error('Port error:', e.message); process.exit(1); });
  await new Promise((res) => port.on('open', res));
  console.log('COM18 open. Decoding 0x0C fields: idle(2.5s) -> drive(3.5s) -> stop(2.5s)');

  phase = 'idle';
  await sleep(2500);

  phase = 'drive';
  port.write(buildPacket(FUNC_MOTOR, [toU8(100), toU8(100), toU8(100), toU8(100)]));
  await sleep(3500);

  phase = 'stop';
  port.write(buildPacket(FUNC_MOTOR, [0, 0, 0, 0]));
  await sleep(2500);

  for (const ph of ['idle', 'drive', 'stop']) {
    const rows = samples[ph];
    if (rows.length === 0) { console.log(`\n[${ph.toUpperCase()}] no packets`); continue; }
    const nFields = Math.max(...rows.map(r => r.length));
    console.log(`\n[${ph.toUpperCase()}] packets=${rows.length}`);
    for (let f = 0; f < nFields; f++) {
      const vals = rows.map(r => r[f]).filter(v => v !== undefined);
      const min = Math.min(...vals);
      const max = Math.max(...vals);
      const avg = vals.reduce((a, b) => a + b, 0) / vals.length;
      console.log(`  field[${f}]: min=${min}  max=${max}  avg=${avg.toFixed(1)}  span=${max - min}`);
    }
  }

  console.log('\nInterpretation: a real wheel encoder count would show a large,');
  console.log('monotonic span increase in DRIVE vs IDLE. Small jitter = noise/ADC.');
  port.close(() => process.exit(0));
})();
