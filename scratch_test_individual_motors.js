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
let currentTicks = [0, 0, 0, 0];

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
        currentTicks[0] = data.readInt32LE(0);
        currentTicks[1] = data.readInt32LE(4);
        currentTicks[2] = data.readInt32LE(8);
        currentTicks[3] = data.readInt32LE(12);
      }
    }
    rxBuf = rxBuf.subarray(totalLen);
  }
});

async function runMotor(index, speed, durationMs) {
  console.log(`\nTesting Motor M${index+1} at speed ${speed}...`);
  const startTicks = [...currentTicks];
  const payload = [0, 0, 0, 0];
  payload[index] = toU8(speed);
  
  const pkt = buildPacket(FUNC_MOTOR, payload);
  port.write(pkt);

  // Keep alive loop
  const interval = setInterval(() => {
    port.write(pkt);
  }, 300);

  await new Promise(r => setTimeout(r, durationMs));
  clearInterval(interval);
  
  // Stop motor
  port.write(buildPacket(FUNC_MOTOR, [0, 0, 0, 0]));
  await new Promise(r => setTimeout(r, 800)); // wait for settling

  const endTicks = [...currentTicks];
  const delta = endTicks[index] - startTicks[index];
  console.log(`Motor M${index+1} Delta Ticks: ${delta} (Start: ${startTicks[index]}, End: ${endTicks[index]})`);
  return delta;
}

port.on('open', async () => {
  console.log('COM18 opened. Waiting 1s for telemetry...');
  await new Promise(r => setTimeout(r, 1000));
  
  // Run all 4 motors one by one
  await runMotor(0, 50, 1500); // M1
  await runMotor(1, 50, 1500); // M2
  await runMotor(2, 50, 1500); // M3
  await runMotor(3, 50, 1500); // M4

  port.close();
  process.exit(0);
});
