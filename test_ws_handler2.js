const fs = require('fs');

global.document = { readyState: 'loading',
    _elements: {},
    getElementById: function(id) { return this._elements[id] || null; },
    querySelectorAll: function(sel) { return []; },
    querySelector: function(sel) { return null; },
    addEventListener: function() {},
    createElement: function(tag) { return { style: {}, classList: { add: () => {} } }; }
};
global.window = { addEventListener: () => {},  location: { protocol: 'http:', host: 'localhost' } };

const html = fs.readFileSync('public/index.html', 'utf8');
const ids = html.match(/id=["']([^"']+)["']/g);
if (ids) {
    ids.forEach(idStr => {
        const id = idStr.substring(4, idStr.length - 1);
        global.document._elements[id] = {
            id: id,
            innerText: '',
            textContent: '',
            innerHTML: '',
            style: {},
            disabled: false,
            value: '',
            className: '',
            checked: false,
            classList: { add: () => {}, remove: () => {} }, addEventListener: () => {}, appendChild: () => {}, querySelectorAll: () => [], getContext: () => ({fillRect:()=>{}, clearRect:()=>{}, moveTo:()=>{}, lineTo:()=>{}, stroke:()=>{}, fill:()=>{}, setLineDash:()=>{}}), width: 100, height: 100
        };
    });
}

// Stub DOM globals and methods
global.logSystem = () => {};
global.logSerialIn = () => {};
global.logSerialOut = () => {};
global.logSerialOutErr = () => {};
global.updateBadge = () => {};
global.updateStraightDriveMetrics = () => {};
global.updateAutoTestVisualizer = () => {};
global.updateWheelAnimation = () => {};
global.update3DModelRotation = () => {};
global.drawPath = () => {};
global.drawLidarTestCanvas = () => {};
global.renderStageResultsTable = () => {};
global.lidarCanvas = {width:100,height:100};
global.fetch = () => Promise.resolve({json: () => Promise.resolve({})});
global.localStorage = { getItem: () => null, setItem: () => {} };

// Run app.js in this Node context
let appJsCode = fs.readFileSync('public/app.js', 'utf8');
// To make it load cleanly in Node (which has strict mode differences), we wrap it in a function
const runCode = new Function(appJsCode + "; return handleServerMessage;");

let handleServerMessage;
try {
    handleServerMessage = runCode();
} catch (e) {
    console.log("Setup failed: \n" + e.stack);
    process.exit(1);
}

const packet = {
    type: 'maintenance_status',
    active: true,
    activeMotor: 0,
    activeMotorNum: 1,
    direction: 0,
    testPwm: 120,
    actualPwm: 110,
    deadmanActive: true,
    remainingTimeout: 5000,
    sessionId: "test-session"
};

console.log("Testing handleServerMessage with maintenance_status packet...");
try {
    handleServerMessage(packet);
    console.log("SUCCESS: handleServerMessage processed packet without throwing!");
} catch (e) {
    console.error("FAILED: handleServerMessage threw an exception: " + e.message);
    console.error(e.stack);
    process.exit(1);
}

const normalDrivePacket = {
    type: 'normal_drive_status',
    armed: false,
    mode: 0,
    source: 0,
    cmdAge: 0,
    reqLinear: 0,
    reqAngular: 0,
    limLinear: 0,
    limAngular: 0,
    lockStatus: false
};

console.log("Testing handleServerMessage with normal_drive_status packet...");
try {
    handleServerMessage(normalDrivePacket);
    console.log("SUCCESS: normal_drive_status processed packet without throwing!");
} catch (e) {
    console.error("FAILED: normal_drive_status threw an exception: " + e.message);
    console.error(e.stack);
    process.exit(1);
}

// Test COMPLETELY empty DOM
global.document._elements = {};
console.log("Testing handleServerMessage with COMPLETELY EMPTY DOM...");
try {
    handleServerMessage(packet);
    console.log("SUCCESS: Completely empty DOM test passed (safe helpers work)!");
} catch (e) {
    console.error("FAILED on empty DOM test: " + e.message);
    console.error(e.stack);
    process.exit(1);
}

process.exit(0);
