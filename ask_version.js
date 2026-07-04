/**
 * ask_version.js (bounded, auto-exits ~12s)
 * Sends "get firmware version" (0x51) requests using both checksum variants
 * and several device-ID candidates, and listens for a 0x51 reply.
 * A reply proves the board ACCEPTS host commands. Silence proves the
 * host->board command path is not being processed at all.
 */
const { SerialPort } = require('serialport');

const HEAD = 0xFF;
const BOARD_ID = 0xFB;
const FUNC_VERSION = 0x51;

function buildExt(deviceId, funcId, payload) {
  const cmd = [HEAD, deviceId, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xff);
  return Buffer.from(cmd);
}

function buildLegacy(deviceId, funcId, payload) {
  const cmd = [HEAD, deviceId, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 257 - deviceId;
  for (const b of cmd) sum += b;
  cmd.push(sum & 0xff);
  return Buffer.from(cmd);
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

(async () => {
  const port = new SerialPort({ path: 'COM18', baudRate: 115200 });
  let rx = Buffer.alloc(0);
  let gotReply = null;

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
      if (extType !== 0x0A && extType !== 0x0C && extType !== 0x0D && extType !== 0x0E) {
        const hex = Array.from(rx.subarray(h, total)).map(b => b.toString(16).padStart(2, '0')).join(' ');
        console.log(`  << NON-ROUTINE REPLY type=0x${extType.toString(16)}: ${hex}`);
        if (extType === FUNC_VERSION) gotReply = hex;
      }
      rx = rx.subarray(total);
    }
  }

  port.on('data', (c) => { rx = Buffer.concat([rx, c]); parse(); });
  port.on('error', (e) => { console.error('Port error:', e.message); process.exit(1); });
  await new Promise((res) => port.on('open', res));
  console.log('COM18 open. Probing version request 0x51 (both checksums, device IDs FC/FA/01)...');
  await sleep(400);

  const deviceIds = [0xFC, 0xFA, 0x01];
  for (const id of deviceIds) {
    for (const [name, builder] of [['ext', buildExt], ['legacy', buildLegacy]]) {
      const pkt = builder(id, FUNC_VERSION, [0x00]);
      const hex = Array.from(pkt).map(b => b.toString(16).padStart(2, '0')).join(' ');
      console.log(`>> devId=0x${id.toString(16)} ${name}: ${hex}`);
      await new Promise((res, rej) => port.write(pkt, e => e ? rej(e) : res()));
      await sleep(800);
      if (gotReply) break;
    }
    if (gotReply) break;
  }

  await sleep(1500);
  if (gotReply) {
    console.log(`\nRESULT: Board REPLIED to version request (${gotReply}). Commands ARE accepted.`);
    console.log('=> Motor failure is in power/driver/config, not comms.');
  } else {
    console.log('\nRESULT: No version reply. The board is NOT processing host->board commands.');
    console.log('=> Telemetry (board->PC) works, but the PC->board direction is dead or ignored.');
  }
  port.close(() => process.exit(0));
})();
