// ==============================================================================
// test_operator_token_persistence.js — Unit Tests for Operator Token Persistence
// ==============================================================================

const assert = require('assert');
const fs = require('fs');
const path = require('path');

class MockStorage {
  constructor() {
    this.store = {};
  }
  getItem(key) {
    return this.store[key] !== undefined ? this.store[key] : null;
  }
  setItem(key, value) {
    this.store[key] = String(value);
  }
  removeItem(key) {
    delete this.store[key];
  }
  clear() {
    this.store = {};
  }
}

// Initialize mock environment
global.sessionStorage = new MockStorage();
global.localStorage = new MockStorage();
global.window = global;
global.window.addEventListener = function() {};
global.requestAnimationFrame = (cb) => setTimeout(cb, 0);
global.cancelAnimationFrame = () => {};
global.systemLogs = [];
global.logSystem = function(msg) {
  global.systemLogs.push(msg);
};
global.alertLogs = [];
global.alert = function(msg) {
  global.alertLogs.push(msg);
};

const mockElements = {};
const dummyElement = { addEventListener() {}, appendChild() {}, removeChild() {}, querySelector() { return null; }, querySelectorAll() { return []; }, childNodes: [], style: {}, innerText: '', textContent: '', value: '', checked: false, getContext: () => new Proxy({}, { get: () => () => {} }) };
const domListeners = [];
global.document = {
  readyState: 'loading',
  getElementById(id) {
    return mockElements[id] || dummyElement;
  },
  querySelector() { return null; },
  querySelectorAll() { return []; },
  createElement() { return { addEventListener() {}, setAttribute() {}, appendChild() {}, style: {} }; },
  createTextNode(str) { return str; },
  body: { appendChild() {} },
  addEventListener(event, fn) {
    if (event === 'DOMContentLoaded') {
      domListeners.push(fn);
    }
  }
};
global.location = { href: 'http://localhost:3000', reload() {} };

function createMockElement(id, props = {}) {
  const el = {
    id,
    value: props.value || '',
    placeholder: props.placeholder || '',
    checked: props.checked || false,
    style: {},
    listeners: {},
    addEventListener(event, fn) {
      this.listeners[event] = fn;
    },
    click() {
      if (this.listeners['click']) this.listeners['click']();
    },
    input(val) {
      this.value = val;
      if (this.listeners['input']) this.listeners['input']();
    },
    change(checked) {
      this.checked = checked;
      if (this.listeners['change']) this.listeners['change']();
    }
  };
  mockElements[id] = el;
  return el;
}

// Instantiate mock elements
const elInput = createMockElement('operator-token-input');
const elCheckbox = createMockElement('remember-token-checkbox');
const elBtnSave = createMockElement('btn-save-token');
const elBtnClear = createMockElement('btn-clear-token');

// Load app.js into context
const appJsPath = path.join(__dirname, 'public', 'app.js');
const appJsCode = fs.readFileSync(appJsPath, 'utf-8');
eval(appJsCode);

// Trigger DOMContentLoaded callbacks
domListeners.forEach(fn => fn());

console.log("Running Operator Token Persistence & Security Unit Tests...");

// ------------------------------------------------------------------------------
// TEST 1: Token saved to sessionStorage when Remember is OFF
// ------------------------------------------------------------------------------
global.sessionStorage.clear();
global.localStorage.clear();
elCheckbox.checked = false;
elInput.input('secret_token_123');
elBtnSave.click();

assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), 'secret_token_123', "Token must be saved in sessionStorage");
assert.strictEqual(global.localStorage.getItem('rover_operator_token'), null, "Token must NOT be in localStorage when Remember is OFF");
console.log("✓ TEST 1 PASSED: Token saved to sessionStorage when Remember is OFF.");

// ------------------------------------------------------------------------------
// TEST 2: Token saved to localStorage when Remember is ON
// ------------------------------------------------------------------------------
global.sessionStorage.clear();
global.localStorage.clear();
elCheckbox.checked = true;
elInput.input('secret_token_456');
elBtnSave.click();

assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), 'secret_token_456', "Token must be in sessionStorage");
assert.strictEqual(global.localStorage.getItem('rover_operator_token'), 'secret_token_456', "Token must be in localStorage when Remember is ON");
console.log("✓ TEST 2 PASSED: Token saved to localStorage when Remember is ON.");

// ------------------------------------------------------------------------------
// TEST 3: Migration of existing session token when Remember is enabled
// ------------------------------------------------------------------------------
global.sessionStorage.clear();
global.localStorage.clear();
elCheckbox.checked = false;
elInput.input('migration_token_789');

assert.strictEqual(global.localStorage.getItem('rover_operator_token'), null);

// Toggle Remember checkbox ON
elCheckbox.change(true);
assert.strictEqual(global.localStorage.getItem('rover_operator_token'), 'migration_token_789', "Token must migrate to localStorage upon checking Remember");
assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), 'migration_token_789');

// Toggle Remember checkbox OFF
elCheckbox.change(false);
assert.strictEqual(global.localStorage.getItem('rover_operator_token'), null, "Token must be removed from localStorage upon unchecking Remember");
assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), 'migration_token_789', "Token must remain in sessionStorage when Remember unchecked");
console.log("✓ TEST 3 PASSED: Existing session token migration on Remember toggle verified.");

// ------------------------------------------------------------------------------
// TEST 4: Clear authorization removes token from both stores
// ------------------------------------------------------------------------------
global.sessionStorage.setItem('rover_operator_token', 'clear_me');
global.localStorage.setItem('rover_operator_token', 'clear_me');
elInput.value = 'clear_me';

elBtnClear.click();

assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), null, "sessionStorage token must be cleared");
assert.strictEqual(global.localStorage.getItem('rover_operator_token'), null, "localStorage token must be cleared");
assert.strictEqual(elInput.value, '', "Input field must be cleared");
console.log("✓ TEST 4 PASSED: Clear authorization removes token from both stores.");

// ------------------------------------------------------------------------------
// TEST 5: authenticatedFetch sends X-Rover-Operator-Token header & omits when missing
// ------------------------------------------------------------------------------
(async () => {
  let capturedHeaders = null;
  global.fetch = async function(url, opts) {
    capturedHeaders = opts.headers;
    return { ok: true, status: 200, json: async () => ({ ok: true }) };
  };

  // With token
  global.sessionStorage.setItem('rover_operator_token', 'active_header_token');
  elInput.value = 'active_header_token';
  await authenticatedFetch('/api/drive/arm', { method: 'POST' });
  assert.strictEqual(capturedHeaders['X-Rover-Operator-Token'], 'active_header_token', "authenticatedFetch must include X-Rover-Operator-Token header");

  // Without token
  global.sessionStorage.clear();
  global.localStorage.clear();
  elInput.value = '';
  await authenticatedFetch('/api/drive/arm', { method: 'POST' });
  assert.strictEqual(capturedHeaders['X-Rover-Operator-Token'], undefined, "authenticatedFetch must omit header when token is missing");

  console.log("✓ TEST 5 PASSED: authenticatedFetch header handling verified.");

  // ------------------------------------------------------------------------------
  // TEST 6: Distinct 401 ("Operator token missing.") and 403 ("Operator token invalid.") error messages
  // ------------------------------------------------------------------------------
  const elBanner = createMockElement('v2-autonomy-error-banner');

  // 401 response
  global.fetch = async function() {
    return { ok: false, status: 401, json: async () => ({ ok: false, error: 'Unauthorized' }) };
  };
  await authenticatedFetch('/api/drive/arm', { method: 'POST' });
  assert.strictEqual(elBanner.textContent, 'Operator token missing.', "401 status must display 'Operator token missing.'");

  // 403 response
  global.fetch = async function() {
    return { ok: false, status: 403, json: async () => ({ ok: false, error: 'Forbidden' }) };
  };
  await authenticatedFetch('/api/drive/arm', { method: 'POST' });
  assert.strictEqual(elBanner.textContent, 'Operator token invalid.', "403 status must display 'Operator token invalid.'");

  console.log("✓ TEST 6 PASSED: Distinct 401 and 403 auth error messages verified.");

  // ------------------------------------------------------------------------------
  // TEST 7: Network error does NOT delete persistent token
  // ------------------------------------------------------------------------------
  global.localStorage.setItem('rover_operator_token', 'persistent_token');
  global.sessionStorage.setItem('rover_operator_token', 'persistent_token');
  elCheckbox.checked = true;

  global.fetch = async function() {
    throw new Error("Network connection failed");
  };

  const res = await authenticatedFetch('/api/drive/status');
  assert.strictEqual(res.ok, false);
  assert.strictEqual(global.localStorage.getItem('rover_operator_token'), 'persistent_token', "Network error must NOT delete localStorage token");
  assert.strictEqual(global.sessionStorage.getItem('rover_operator_token'), 'persistent_token', "Network error must NOT delete sessionStorage token");

  console.log("✓ TEST 7 PASSED: Persistent token retained across temporary network errors.");

  // ------------------------------------------------------------------------------
  // TEST 8: Token security (never logged or exposed in raw text)
  // ------------------------------------------------------------------------------
  const sampleSecret = 'super_secret_token_XYZ99';
  global.systemLogs = [];
  global.sessionStorage.setItem('rover_operator_token', sampleSecret);
  elCheckbox.checked = true;
  elInput.input(sampleSecret);
  elBtnSave.click();

  const containsSecretInLogs = global.systemLogs.some(log => String(log).includes(sampleSecret));
  assert.strictEqual(containsSecretInLogs, false, "Raw token string must never appear in system logs");

  console.log("✓ TEST 8 PASSED: Operator token security verified (no logging or text exposure).");

  console.log("==================================================");
  console.log("ALL OPERATOR TOKEN PERSISTENCE & SECURITY TESTS PASSED!");
  console.log("==================================================");
  process.exit(0);
})();
