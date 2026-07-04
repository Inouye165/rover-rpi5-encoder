const { SerialPort } = require('serialport');

const HEAD = 0xFF;
const DEVICE_ID = 0xFC;
const COMPLEMENT = 257 - DEVICE_ID;
const FUNC_MOTOR = 0x10;

function checksumExt(packetNoChecksum) {
  let sum = 0;
  for (let i = 2; i < packetNoChecksum.length; i++) sum += packetNoChecksum[i];
  return sum & 0xFF;
}

function checksumLegacy(packetNoChecksum) {
  let sum = COMPLEMENT;
  for (let i = 0; i < packetNoChecksum.length; i++) sum += packetNoChecksum[i];
  return sum & 0xFF;
}

function buildPacket(funcId, payload, legacy) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  const cs = legacy ? checksumLegacy(cmd) : checksumExt(cmd);
  cmd.push(cs);
  return Buffer.from(cmd);
}

function toU8(v) {
  const c = Math.max(-100, Math.min(100, v));
  return c < 0 ? 256 + c : c;
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

async function writeDual(port, payload, label) {
  const pExt = buildPacket(FUNC_MOTOR, payload, false);
  const pLegacy = buildPacket(FUNC_MOTOR, payload, true);
  const extHex = Array.from(pExt).map((b) => b.toString(16).padStart(2, '0')).join(' ');
  const legacyHex = Array.from(pLegacy).map((b) => b.toString(16).padStart(2, '0')).join(' ');
  console.log(`\n[${label}]`);
  console.log(`  ext:    ${extHex}`);
  console.log(`  legacy: ${legacyHex}`);

  await new Promise((res, rej) => port.write(pExt, (e) => (e ? rej(e) : res())));
  await sleep(8);
  if (!pLegacy.equals(pExt)) {
    await new Promise((res, rej) => port.write(pLegacy, (e) => (e ? rej(e) : res())));
  }
}

(async () => {
  const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

  port.on('error', (e) => {
    console.error('Port error:', e.message);
    process.exit(1);
  });

  await new Promise((res) => port.on('open', res));
  console.log('COM18 open. Channel mapping starts now.');
  console.log('Watch the motor and note which step moves your connected motor.');

  const speed = 80;
  const steps = [
    { name: 'M1 only', payload: [toU8(speed), 0, 0, 0] },
    { name: 'M2 only', payload: [0, toU8(speed), 0, 0] },
    { name: 'M3 only', payload: [0, 0, toU8(speed), 0] },
    { name: 'M4 only', payload: [0, 0, 0, toU8(speed)] },
  ];

  for (const step of steps) {
    await writeDual(port, step.payload, `${step.name} forward ${speed}`);
    await sleep(1800);

    await writeDual(port, [0, 0, 0, 0], `${step.name} stop`);
    await sleep(900);

    await writeDual(port, step.payload.map((v) => (v ? toU8(-speed) : 0)), `${step.name} reverse ${speed}`);
    await sleep(1800);

    await writeDual(port, [0, 0, 0, 0], `${step.name} stop`);
    await sleep(1200);
  }

  console.log('\nMapping run complete.');
  port.close(() => process.exit(0));
})();
