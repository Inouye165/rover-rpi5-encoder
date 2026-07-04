const { SerialPort } = require('serialport');

const states = [
  { dtr: false, rts: false },
  { dtr: true, rts: false },
  { dtr: false, rts: true },
  { dtr: true, rts: true }
];

let stateIndex = 0;
let port = null;

function tryNextState() {
  if (stateIndex >= states.length) {
    console.log('Tested all DTR/RTS states. Exiting.');
    process.exit(0);
  }

  const state = states[stateIndex];
  console.log(`Testing state ${stateIndex + 1}/${states.length}: DTR = ${state.dtr}, RTS = ${state.rts}`);

  port = new SerialPort({
    path: 'COM18',
    baudRate: 115200,
  });

  let bytesReceived = 0;

  port.on('open', () => {
    port.set(state, (err) => {
      if (err) {
        console.error('Failed to set port state:', err.message);
      } else {
        console.log('Port state set. Listening for 3 seconds...');
      }
    });
  });

  port.on('data', (data) => {
    bytesReceived += data.length;
    console.log(`[STATE ${stateIndex}] Received ${data.length} bytes:`, data.toString('hex').match(/.{1,2}/g).join(' '));
  });

  setTimeout(() => {
    console.log(`[STATE ${stateIndex}] Total bytes received: ${bytesReceived}`);
    if (bytesReceived > 0) {
      console.log(`SUCCESS! State DTR=${state.dtr}, RTS=${state.rts} works!`);
      port.close();
      process.exit(0);
    }
    
    port.close((err) => {
      if (err) console.error('Error closing port:', err.message);
      stateIndex++;
      // Wait a short duration to ensure port is released
      setTimeout(tryNextState, 500);
    });
  }, 3000);
}

tryNextState();
