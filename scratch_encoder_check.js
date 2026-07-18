const { SerialPort } = require('serialport');

const HEAD = 0xFF;
const DEVICE_ID = 0xFC;
const BOARD_ID = 0xFB;
const FUNC_MOTOR = 0x10;
const TYPE_ENCODER = 0x0D;

function buildPacket(funcId, payload) {
  const cmd = [HEAD, DEVICE_ID, 0x00, funcId, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = 0;
  for (let i = 2; i < cmd.length; i++) sum += cmd[i];
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

function toU8(v) {
  const c = Math.max(-100, Math.min(100, v));
  return c < 0 ? 256 + c : c;
}

const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

let rxBuf = Buffer.alloc(0);
let initialTicks = null;
let currentTicks = null;

port.on('open', async () => {
  console.log('COM18 opened.');
  // Set DTR/RTS to false just in case
  port.set({ dtr: false, rts: false }, async (err) => {
    if (err) console.error('Error setting control lines:', err);
    console.log('Control lines set. Waiting 1s for telemetry stream to stabilize...');
    await new Promise(r => setTimeout(r, 1000));

    console.log('Sending MOTOR FWD (all = 50)...');
    const fwdPkt = buildPacket(FUNC_MOTOR, [toU8(50), toU8(50), toU8(50), toU8(50)]);
    port.write(fwdPkt);

    // Keep sending command to prevent safety timeout
    const interval = setInterval(() => {
      port.write(fwdPkt);
    }, 400);

    // Run for 2 seconds
    setTimeout(async () => {
      clearInterval(interval);
      console.log('Stopping motors...');
      port.write(buildPacket(FUNC_MOTOR, [0, 0, 0, 0]));
      await new Promise(r => setTimeout(r, 1000));
      
      console.log('\n--- Ticks Summary ---');
      console.log('Initial Ticks:', initialTicks);
      console.log('Final Ticks:  ', currentTicks);
      if (initialTicks && currentTicks) {
        console.log('Delta Ticks:  ', {
          M1: currentTicks[0] - initialTicks[0],
          M2: currentTicks[1] - initialTicks[1],
          M3: currentTicks[2] - initialTicks[2],
          M4: currentTicks[3] - initialTicks[3]
        });
      }
      port.close();
      process.exit(0);
    }, 2000);
  });
});

port.on('data', (chunk) => {
  rxBuf = Buffer.concat([rxBuf, chunk]);
  while (rxBuf.length >= 4) {
    const h = rxBuf.indexOf(0xFF);
    if (h === -1 || h >= rxBuf.length - 1) break;
    if (rxBuf[h + 1] !== BOARD_ID) {
      rxBuf = rxBuf.subarray(h + 1);
      continue;
    }
    if (rxBuf.length < h + 3) break;
    const extLen = rxBuf[h + 2];
    const totalLen = h + 2 + extLen;
    if (rxBuf.length < totalLen) break;
    const extType = rxBuf[h + 3];
    if (extType === TYPE_ENCODER) {
      const data = rxBuf.subarray(h + 4, h + 4 + extLen - 2);
      if (data.length >= 16) {
        const t1 = data.readInt32LE(0);
        const t2 = data.readInt32LE(4);
        const t3 = data.readInt32LE(8);
        const t4 = data.readInt32LE(12);
        currentTicks = [t1, t2, t3, t4];
        if (!initialTicks) {
          initialTicks = [...currentTicks];
        }
      }
    }
    rxBuf = rxBuf.subarray(totalLen);
  }
});
