/**
 * beep_dtr_test.js (bounded, auto-exits ~16s)
 * For each DTR/RTS combination: send BEEP + VERSION request, listen 1.5s
 * for a 0x51 reply. Objectively detects whether any control-line state
 * makes the board accept commands.
 */
const { SerialPort } = require('serialport');

const HEAD = 0xFF, DEVICE_ID = 0xFC, BOARD_ID = 0xFB;
const FUNC_BEEP = 0x02, FUNC_VERSION = 0x23;
const TYPE_FIRMWARE_INFO = 0x32;

function build(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xff);
  return Buffer.from(cmd);
}

function sleep(ms) { return new Promise((r) => setTimeout(r, ms)); }

const states = [
  { dtr: false, rts: false },
  { dtr: true,  rts: false },
  { dtr: false, rts: true  },
  { dtr: true,  rts: true  },
];

(async () => {
  let winner = null;

  for (const st of states) {
    const port = new SerialPort({ path: process.env.SERIAL_PORT || process.env.ROVER_ESP32_DEVICE || '/dev/rover-esp32', baudRate: 115200 });
    let rx = Buffer.alloc(0);
    let versionReply = null;

    port.on('data', (c) => {
      rx = Buffer.concat([rx, c]);
      while (rx.length >= 4) {
        const h = rx.indexOf(0xff);
        if (h < 0 || h + 1 >= rx.length) break;
        if (rx[h + 1] !== BOARD_ID) { rx = rx.subarray(h + 1); continue; }
        if (rx.length < h + 3) break;
        const extLen = rx[h + 2];
        const total = h + 2 + extLen;
        if (rx.length < total) break;
        const extType = rx[h + 3];
        if (extType === TYPE_FIRMWARE_INFO) {
          versionReply = Array.from(rx.subarray(h, total)).map(b => b.toString(16).padStart(2, '0')).join(' ');
        }
        rx = rx.subarray(total);
      }
    });
    port.on('error', (e) => { console.error('Port error:', e.message); process.exit(1); });

    await new Promise((res) => port.on('open', res));
    await new Promise((res) => port.set(st, () => res()));
    await sleep(300);

    port.write(build(FUNC_BEEP, [200, 0]));
    await sleep(100);
    port.write(build(FUNC_VERSION, [0x00]));
    await sleep(1500);

    console.log(`DTR=${st.dtr} RTS=${st.rts}  ->  version reply: ${versionReply || 'NONE'}`);
    if (versionReply && !winner) winner = { ...st, reply: versionReply };

    await new Promise((res) => port.close(() => res()));
    await sleep(400);
  }

  if (winner) {
    console.log(`\nSUCCESS: board accepts commands with DTR=${winner.dtr} RTS=${winner.rts}`);
    console.log('=> Update server.js to set these control lines after opening the port.');
  } else {
    console.log('\nRESULT: No DTR/RTS state made the board reply.');
    console.log('=> Host->board command path is dead regardless of control lines.');
    console.log('=> This is a hardware/firmware-side issue, not a software issue.');
  }
  process.exit(0);
})();
