/**
 * probe_opcodes.js
 * Sends probe packets for every opcode 0x01..0x20 to COM18
 * and logs any bytes that come back.
 * 
 * Rosmaster binary protocol:
 *   [0xFF, 0xFC, length, opcode, ...payload, checksum]
 *   length  = len(packet) - 1  (bytes after length byte)
 *   checksum = (sum_of_all_bytes + complement) & 0xFF
 *   complement = 257 - 0xFC = 1
 */
const { SerialPort } = require('serialport');

const HEAD        = 0xFF;
const DEVICE_ID   = 0xFC;
const COMPLEMENT  = 257 - DEVICE_ID; // = 1

function buildPacket(opcode, payload = []) {
  const cmd = [HEAD, DEVICE_ID, 0x00, opcode, ...payload];
  cmd[2] = cmd.length - 1;
  let sum = COMPLEMENT;
  for (const b of cmd) sum += b;
  cmd.push(sum & 0xFF);
  return Buffer.from(cmd);
}

const port = new SerialPort({ path: 'COM18', baudRate: 115200 });

let receivedBytes = 0;
port.on('data', (chunk) => {
  receivedBytes += chunk.length;
  const hex = Array.from(chunk).map(b => '0x' + b.toString(16).padStart(2,'0')).join(' ');
  console.log(`[IN] ${hex}`);
});

port.on('open', async () => {
  console.log('Port opened. Starting opcode probe...\n');
  await new Promise(r => setTimeout(r, 500));

  // First try the known "enable auto report" pattern:
  // Opcode 0x0C with payload [1, 0] = enable, not-forever
  const enablePackets = [
    { name: 'enable_report 0x0C [1,0]', pkt: buildPacket(0x0C, [1, 0]) },
    { name: 'enable_report 0x0C [1,1]', pkt: buildPacket(0x0C, [1, 1]) },
    { name: 'enable_report 0x0D [1,0]', pkt: buildPacket(0x0D, [1, 0]) },
    { name: 'enable_report 0x0E [1,0]', pkt: buildPacket(0x0E, [1, 0]) },
    { name: 'enable_report 0x0F [1,0]', pkt: buildPacket(0x0F, [1, 0]) },
    { name: 'enable_report 0x00 [1,0]', pkt: buildPacket(0x00, [1, 0]) },
    { name: 'enable_report 0x01 [1,0]', pkt: buildPacket(0x01, [1, 0]) },
  ];

  for (const { name, pkt } of enablePackets) {
    const hex = Array.from(pkt).map(b => '0x' + b.toString(16).padStart(2,'0')).join(' ');
    console.log(`[SEND] ${name}: ${hex}`);
    await new Promise((resolve, reject) => port.write(pkt, (err) => err ? reject(err) : resolve()));
    await new Promise(r => setTimeout(r, 1000));
    if (receivedBytes > 0) {
      console.log(`\n✅ Got response after: ${name}`);
      break;
    }
  }

  if (receivedBytes === 0) {
    console.log('\nNo response yet. Trying broad opcode sweep 0x00..0x20 with payload [1,0]...');
    for (let op = 0x00; op <= 0x20; op++) {
      const pkt = buildPacket(op, [1, 0]);
      const hex = Array.from(pkt).map(b => b.toString(16).padStart(2,'0')).join(' ');
      console.log(`[SEND opcode 0x${op.toString(16).padStart(2,'0')}] ${hex}`);
      await new Promise((resolve, reject) => port.write(pkt, (err) => err ? reject(err) : resolve()));
      await new Promise(r => setTimeout(r, 500));
      if (receivedBytes > 10) {
        console.log(`\n✅ STREAMING STARTED after opcode 0x${op.toString(16).padStart(2,'0')}!`);
        break;
      }
    }
  }

  // Listen for 5 more seconds to capture any streaming data
  console.log('\nListening for 5 seconds for streaming data...');
  await new Promise(r => setTimeout(r, 5000));

  console.log(`\nTotal bytes received: ${receivedBytes}`);
  port.close();
  process.exit(0);
});

port.on('error', (err) => {
  console.error('Error:', err.message);
  process.exit(1);
});
