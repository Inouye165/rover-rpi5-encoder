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

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

(async () => {
  const port = new SerialPort({ path: 'COM18', baudRate: 115200 });
  let rx = Buffer.alloc(0);

  const buckets = {
    idle: new Map(),
    drive: new Map(),
    stop: new Map(),
  };

  let phase = 'idle';

  function countPacket(data) {
    const hex = Array.from(data).map((b) => b.toString(16).padStart(2, '0')).join(' ');
    const m = buckets[phase];
    m.set(hex, (m.get(hex) || 0) + 1);
  }

  function parse() {
    while (rx.length >= 4) {
      const h = rx.indexOf(0xff);
      if (h < 0 || h + 1 >= rx.length) break;
      if (rx[h + 1] !== BOARD_ID) {
        rx = rx.subarray(h + 1);
        continue;
      }
      if (rx.length < h + 3) break;
      const extLen = rx[h + 2];
      const total = h + 2 + extLen;
      if (rx.length < total) break;

      const extType = rx[h + 3];
      const dataLen = extLen - 2;
      if (dataLen >= 0 && extType === TYPE_ENCODER) {
        const data = rx.subarray(h + 4, h + 4 + dataLen);
        countPacket(data);
      }
      rx = rx.subarray(total);
    }
  }

  port.on('data', (chunk) => {
    rx = Buffer.concat([rx, chunk]);
    parse();
  });

  port.on('error', (e) => {
    console.error('Port error:', e.message);
    process.exit(1);
  });

  await new Promise((res) => port.on('open', res));
  console.log('COM18 open. Collecting encoder packets across phases.');

  phase = 'idle';
  await sleep(2500);

  phase = 'drive';
  const fwd = buildPacket(FUNC_MOTOR, [toU8(100), toU8(100), toU8(100), toU8(100)]);
  port.write(fwd);
  await sleep(3500);

  phase = 'stop';
  const stop = buildPacket(FUNC_MOTOR, [0, 0, 0, 0]);
  port.write(stop);
  await sleep(2500);

  function summarize(name) {
    const m = buckets[name];
    const total = Array.from(m.values()).reduce((a, b) => a + b, 0);
    const unique = m.size;
    const top = Array.from(m.entries()).sort((a, b) => b[1] - a[1]).slice(0, 5);
    console.log(`\n[${name.toUpperCase()}] packets=${total}, uniquePayloads=${unique}`);
    top.forEach(([hex, count], i) => {
      console.log(`  ${i + 1}. ${count}x  ${hex}`);
    });
  }

  summarize('idle');
  summarize('drive');
  summarize('stop');

  const sameProfile = JSON.stringify(Array.from(buckets.idle.entries()).sort()) === JSON.stringify(Array.from(buckets.drive.entries()).sort());
  if (sameProfile) {
    console.log('\nResult: Encoder payload profile did not change between idle and drive command windows.');
  } else {
    console.log('\nResult: Encoder payload profile changed during drive command window.');
  }

  port.close(() => process.exit(0));
})();
