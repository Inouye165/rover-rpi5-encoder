const { SerialPort } = require('serialport');

const port = new SerialPort({
  path: 'COM18',
  baudRate: 115200,
});

console.log('Opening COM18...');
let bytesCount = 0;

port.on('open', () => {
  console.log('Serial port opened. Listening for bytes...');
});

port.on('data', (data) => {
  bytesCount += data.length;
  console.log(`Received chunk of ${data.length} bytes (Total: ${bytesCount}):`);
  console.log(data.toString('hex').match(/.{1,2}/g).join(' '));
  
  if (bytesCount > 500) {
    console.log('Received enough bytes, closing port...');
    port.close();
    process.exit(0);
  }
});

port.on('error', (err) => {
  console.error('Error:', err.message);
  process.exit(1);
});

// Set a timeout in case no data is received
setTimeout(() => {
  console.log('Timeout. Closing port...');
  port.close();
  process.exit(0);
}, 10000);
