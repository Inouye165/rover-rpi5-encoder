const { SerialPort } = require('serialport');

const HEAD = 0xFF;
const DEVICE_ID = 0xFC;
const FUNC_BEEP = 0x02;
const FUNC_MOTOR = 0x10;
const FUNC_MOTION = 0x12;
const FUNC_CAR_TYPE = 0x44;

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

function i16le(v) {
  const u = v < 0 ? 65536 + v : v;
  return [u & 0xff, (u >> 8) & 0xff];
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

async function send(port, label, funcId, payload) {
  const pkt = buildPacket(funcId, payload);
  const hex = Array.from(pkt).map((b) => b.toString(16).padStart(2, '0')).join(' ');
  console.log(`\n[TX] ${label}`);
  console.log(`  ${hex}`);
  await new Promise((res, rej) => port.write(pkt, (e) => (e ? rej(e) : res())));
}

(async () => {
  const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

  port.on('error', (e) => {
    console.error('Port error:', e.message);
    process.exit(1);
  });

  await new Promise((res) => port.on('open', res));
  console.log('COM18 open. Running one-shot motor power proof (about 20s).');

  await send(port, 'Beep 200ms', FUNC_BEEP, [200, 0]);
  await sleep(1000);

  for (const ct of [1, 2, 3, 4]) {
    await send(port, `Set car type ${ct}`, FUNC_CAR_TYPE, [ct, 0]);
    await sleep(250);

    await send(port, `FUNC_MOTOR all +100 (car type ${ct})`, FUNC_MOTOR, [toU8(100), toU8(100), toU8(100), toU8(100)]);
    await sleep(1800);

    await send(port, `FUNC_MOTOR stop (car type ${ct})`, FUNC_MOTOR, [0, 0, 0, 0]);
    await sleep(600);

    await send(port, `FUNC_MOTION vX=600 (car type ${ct})`, FUNC_MOTION, [...i16le(600), ...i16le(0), ...i16le(0)]);
    await sleep(1800);

    await send(port, `FUNC_MOTION stop (car type ${ct})`, FUNC_MOTION, [...i16le(0), ...i16le(0), ...i16le(0)]);
    await sleep(700);
  }

  await send(port, 'Final stop', FUNC_MOTOR, [0, 0, 0, 0]);
  await sleep(200);

  console.log('\nDone. If no motor movement occurred during this script, software command path is verified and issue is motor driver power/enable stage on hardware/firmware side.');
  port.close(() => process.exit(0));
})();
