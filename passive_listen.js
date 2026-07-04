/**
 * passive_listen.js - Just opens COM18 and listens passively for 10 seconds.
 * Does NOT send anything. Decodes and prints all incoming bytes as ASCII.
 */
const { SerialPort } = require('serialport');

const port = new SerialPort({ path: 'COM18', baudRate: 115200 });
let totalBytes = 0;
let asciiBuf = '';

port.on('open', () => {
  console.log('Port opened. Listening passively for 10 seconds (no commands sent)...\n');
});

port.on('data', (chunk) => {
  totalBytes += chunk.length;
  // Print as hex
  const hex = Array.from(chunk).map(b => b.toString(16).padStart(2,'0')).join(' ');
  console.log(`[HEX] ${hex}`);
  // Also try ASCII decode
  const ascii = chunk.toString('latin1').replace(/[^\x20-\x7E\r\n]/g, '.');
  console.log(`[TXT] ${ascii}`);
});

setTimeout(() => {
  console.log(`\nDone. Total bytes received passively: ${totalBytes}`);
  port.close();
  process.exit(0);
}, 10000);

port.on('error', (err) => {
  console.error('Port error:', err.message);
  process.exit(1);
});
