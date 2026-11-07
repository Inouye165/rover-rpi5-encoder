/*
 * Rover 1 HUD Cockpit — shared core (desktop hud.html + mobile hud-mobile.html)
 *
 * Visual layout is ported from the Rover 2 "ROVER GROUND CONTROL" cockpit, but every
 * command goes through Rover 1's server.js safety path:
 *   - operator-token auth (WebSocket `auth` + `X-Rover-Operator-Token` header)
 *   - arming via POST /api/drive/arm (firmware arm), never a direct /cmd_vel publish
 *   - E-STOP fans out to every stop path the server exposes
 *
 * The operator token is NEVER hard-coded here. It is read from / written to the same
 * localStorage key the legacy engineering cockpit uses ('rover_operator_token').
 */
(function (global) {
  'use strict';

  const TOKEN_KEY = 'rover_operator_token';
  const JOY_SEND_HZ = 10;
  const JOY_ZERO_FRAMES = 3;
  const LIDAR_MIN_RANGE_M = 0.15;
  const MANUAL_MAX_LINEAR_MPS = 0.80;   // server.js manual limit (non floor-testing)
  const MANUAL_MAX_ANGULAR_RADPS = 3.00;
  const FRONT_STOP_MM = 30;             // <= 30 mm in front of rover -> full forward stop
  const FRONT_CRAWL_MM = 75;            // <= 75 mm in front of rover -> slow to crawl
  const FRONT_CRAWL_LINEAR_MPS = 0.03;  // 30 mm/s crawl cap inside 75 mm zone
  const FRONT_CRAWL_ANGULAR_RADPS = 0.35;
  const FRONT_ZONE_STALE_MS = 2500;     // Expire stale proximity telemetry to OFFLINE

  // ────────────────────────────────────────────────────────────
  // Event bus + state store
  // ────────────────────────────────────────────────────────────
  const bus = {};
  function on(evt, fn) {
    (bus[evt] = bus[evt] || []).push(fn);
    return () => { bus[evt] = (bus[evt] || []).filter(f => f !== fn); };
  }
  function emit(evt, payload) {
    (bus[evt] || []).slice().forEach(fn => {
      try { fn(payload); } catch (e) { console.error('[HUD] listener error for', evt, e); }
    });
  }

  const state = {
    wsConnected: false,
    authOk: false,
    authError: null,
    serial: 'unknown',
    serialPort: null,
    firmware: null,
    deployed: null,
    battery: null,
    system: null,
    odom: null,
    imu: null,
    imuEuler: null,
    drive: { armed: false },
    pid: null,
    autonomy: null,
    localization: null,
    nav: null,
    home: null,
    slam: null,
    lidarStatus: null,
    scan: { points: [], scanHz: null, t: 0, frameTimes: [] },
    tof: { points: [], t: 0 },
    frontZone: {
      state: 'OFFLINE',
      active: false,
      sensorLive: false,
      intervening: false,
      intervention: 'NONE',
      indicatorCode: 'OFFLINE',
      indicatorLevel: 'offline',
      indicatorText: 'PROXIMITY — OFFLINE',
      minMm: null,
      leftMinMm: null,
      rightMinMm: null,
      stopThresholdMm: FRONT_STOP_MM,
      crawlThresholdMm: FRONT_CRAWL_MM,
      crawlSpeedMps: FRONT_CRAWL_LINEAR_MPS,
      receivedAt: 0
    },
    map: null,
    mapVersion: 0,
    lastWsMsg: 0,
    lastFault: null
  };

  // ────────────────────────────────────────────────────────────
  // Small helpers
  // ────────────────────────────────────────────────────────────
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const isNum = v => typeof v === 'number' && isFinite(v);
  function normAngle(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }
  function quatToEuler(q) {
    if (!q || !isNum(q.w)) return null;
    const { w, x, y, z } = q;
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const sinp = clamp(2 * (w * y - z * x), -1, 1);
    const pitch = Math.asin(sinp);
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    return { yawDeg: yaw * 180 / Math.PI, pitchDeg: pitch * 180 / Math.PI, rollDeg: roll * 180 / Math.PI };
  }
  function toPoint(p) {
    if (!p) return null;
    if (Array.isArray(p)) return { x: Number(p[0]), y: Number(p[1]) };
    if (isNum(p.x) && isNum(p.y)) return { x: p.x, y: p.y };
    if (p.pose && p.pose.position) return { x: p.pose.position.x, y: p.pose.position.y };
    return null;
  }
  function toPath(arr) {
    if (!Array.isArray(arr)) return [];
    return arr.map(toPoint).filter(Boolean);
  }

  // ────────────────────────────────────────────────────────────
  // Operator token (no hard-coded fallback — ever)
  // ────────────────────────────────────────────────────────────
  function getToken() {
    try {
      return (global.localStorage && localStorage.getItem(TOKEN_KEY)) ||
             (global.sessionStorage && sessionStorage.getItem(TOKEN_KEY)) || '';
    } catch (e) {
      return '';
    }
  }
  function setToken(tok) {
    tok = String(tok || '').trim();
    try {
      if (tok) {
        localStorage.setItem(TOKEN_KEY, tok);
        sessionStorage.setItem(TOKEN_KEY, tok);
      } else {
        localStorage.removeItem(TOKEN_KEY);
        sessionStorage.removeItem(TOKEN_KEY);
      }
    } catch (e) { /* storage disabled */ }
    state.authOk = false;
    state.authError = null;
    if (tok) sendAuth(); else wsSend({ type: 'deauth' });
    emit('auth', state);
  }

  // ────────────────────────────────────────────────────────────
  // WebSocket client (same origin as server.js)
  // ────────────────────────────────────────────────────────────
  let ws = null;
  let wsRetry = null;
  function wsUrl() {
    const proto = global.location && global.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${proto}//${global.location.host}`;
  }
  function connect() {
    clearTimeout(wsRetry);
    let sock;
    try {
      sock = new global.WebSocket(wsUrl());
    } catch (e) {
      scheduleReconnect();
      return;
    }
    ws = sock;
    sock.onopen = () => {
      if (sock !== ws) return;
      state.wsConnected = true;
      emit('connection', state);
      sendAuth();
    };
    sock.onmessage = (ev) => {
      if (sock !== ws) return;
      let m;
      try { m = JSON.parse(ev.data); } catch (e) { return; }
      state.lastWsMsg = Date.now();
      handleMessage(m);
    };
    const drop = () => {
      if (sock !== ws) return;
      sock.onclose = sock.onerror = null;
      ws = null;
      state.wsConnected = false;
      state.authOk = false;
      releaseAllDriveInputs();
      if (state.frontZone && state.frontZone.intervening) {
        const zState = state.frontZone.state;
        state.frontZone = Object.assign({}, state.frontZone, {
          intervening: false,
          intervention: 'NONE',
          action: zState === 'STOP' ? 'STOP_ZONE' : (zState === 'CRAWL' ? 'CRAWL_MONITOR' : 'NONE'),
          indicatorCode: zState === 'STOP' ? 'STOP_ZONE' : (zState === 'CRAWL' ? 'CRAWL_ZONE' : state.frontZone.indicatorCode),
          indicatorText: zState === 'STOP' ? 'PROXIMITY — STOP ZONE' : (zState === 'CRAWL' ? 'PROXIMITY — CRAWL ZONE' : state.frontZone.indicatorText)
        });
        emit('front_zone', state.frontZone);
      }
      emit('connection', state);
      emit('auth', state);
      scheduleReconnect();
    };
    sock.onclose = drop;
    sock.onerror = drop;
  }
  function scheduleReconnect() {
    clearTimeout(wsRetry);
    wsRetry = setTimeout(connect, 2000);
  }
  function wsSend(obj) {
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }
  function sendAuth() {
    const t = getToken();
    if (t) wsSend({ type: 'auth', token: t });
  }

  function handleMessage(m) {
    switch (m.type) {
      case 'status':
        if (m.key === 'serial') {
          state.serial = m.val;
          state.serialPort = m.port || state.serialPort;
          emit('serial', state);
        }
        break;
      case 'cockpit_info':
        state.deployed = m.deployed || null;
        emit('cockpit_info', state);
        break;
      case 'auth_result':
        state.authOk = m.ok === true;
        state.authError = m.ok ? null : (m.error || 'Invalid operator token');
        emit('auth', state);
        break;
      case 'deauth_result':
        state.authOk = false;
        emit('auth', state);
        break;
      case 'battery':
        state.battery = Object.assign({}, state.battery || {}, m, { receivedAt: Date.now() });
        emit('battery', state.battery);
        break;
      case 'firmware_info':
        state.firmware = m;
        emit('firmware', m);
        break;
      case 'odom':
        state.odom = m;
        emit('odom', m);
        break;
      case 'normal_drive_status':
        state.drive = Object.assign({}, m, { receivedAt: Date.now() });
        emit('drive', state.drive);
        break;
      case 'pid_diagnostic': {
        const now = Date.now();
        const wheels = [m.m1, m.m2, m.m3, m.m4].filter(Boolean);
        const maxAbsPwm = wheels.reduce((mx, w) => Math.max(mx, Math.abs(w.finalPwm || 0)), 0);
        const maxAbsTarget = wheels.reduce((mx, w) => Math.max(mx, Math.abs(w.targetRadps || 0)), 0);
        const maxAbsMeas = wheels.reduce((mx, w) => Math.max(mx, Math.abs(w.measuredRadps || 0)), 0);
        const anyBlocked = wheels.some(w => w.stictionState === 'BLOCKED' || w.stictionCode === 3);
        const prevPid = state.pid || {};
        let lastBrakeAt = prevPid.lastBrakeAt || 0;
        if (m.actuationState === 'BRAKE') {
          lastBrakeAt = now;
        }
        let zeroMotionPwmSince = prevPid.zeroMotionPwmSince || 0;
        if (state.drive && state.drive.armed && maxAbsPwm >= 35 && maxAbsTarget >= 0.1 && maxAbsMeas < 0.05) {
          if (!zeroMotionPwmSince) zeroMotionPwmSince = now;
        } else {
          zeroMotionPwmSince = 0;
        }
        const noMotorPowerDetected = Boolean(
          state.drive && state.drive.armed && (
            (anyBlocked && maxAbsMeas < 0.05) ||
            (zeroMotionPwmSince > 0 && (now - zeroMotionPwmSince) >= 400)
          )
        );
        state.pid = Object.assign({}, m, {
          receivedAt: now,
          maxAbsPwm,
          maxAbsTarget,
          maxAbsMeas,
          anyBlocked,
          lastBrakeAt,
          zeroMotionPwmSince,
          noMotorPowerDetected
        });
        emit('pid', state.pid);
        break;
      }
      case 'bno08x_imu':
        state.imu = Object.assign({}, m, { receivedAt: Date.now() });
        state.imuEuler = quatToEuler(m.orientation);
        emit('imu', state.imu);
        break;
      case 'autonomy_status':
        state.autonomy = m.status || m;
        if (state.autonomy && state.autonomy.frontSafetyZone) {
          state.frontZone = evaluateFrontZone({ frontSafetyZone: state.autonomy.frontSafetyZone });
          emit('front_zone', state.frontZone);
        }
        emit('autonomy', state.autonomy);
        break;
      case 'front_safety_zone':
        if (m.zone) {
          state.frontZone = evaluateFrontZone({ frontSafetyZone: m.zone });
          emit('front_zone', state.frontZone);
        }
        break;
      case 'localization_status':
        if (m.localization) {
          state.localization = m.localization;
          emit('localization', state.localization);
        }
        break;
      case 'fault_report':
        state.lastFault = { flags: m.faultFlags, t: Date.now() };
        emit('fault', state.lastFault);
        break;
      case 'error':
        emit('toast', { level: 'error', text: m.message || 'Server error' });
        if (/unauthori[sz]ed/i.test(m.message || '')) {
          state.authOk = false;
          emit('auth', state);
        }
        break;
      default:
        break;
    }
  }

  // ────────────────────────────────────────────────────────────
  // REST helper with timeout
  // ────────────────────────────────────────────────────────────
  async function api(path, opts = {}) {
    const o = { method: opts.method || 'GET', headers: Object.assign({}, opts.headers || {}) };
    if (opts.body !== undefined) {
      o.headers['Content-Type'] = 'application/json';
      o.body = JSON.stringify(opts.body);
    }
    if (opts.auth) {
      const t = getToken();
      if (t) o.headers['X-Rover-Operator-Token'] = t;
    }
    const ctrl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
    let timer = null;
    if (ctrl) {
      o.signal = ctrl.signal;
      timer = setTimeout(() => ctrl.abort(), opts.timeoutMs || 8000);
    }
    try {
      const res = await global.fetch(path, o);
      let data = null;
      try { data = await res.json(); } catch (e) { data = null; }
      data = data || {};
      return { ok: res.ok && data.ok !== false, status: res.status, data };
    } catch (e) {
      return { ok: false, status: 0, data: { error: e && e.name === 'AbortError' ? 'Request timed out' : (e && e.message) || 'Network error' } };
    } finally {
      if (timer) clearTimeout(timer);
    }
  }

  // ────────────────────────────────────────────────────────────
  // Pollers (exponential back-off on failure, pause while tab hidden)
  // ────────────────────────────────────────────────────────────
  const pollers = [];
  function startPoller(name, intervalMs, task) {
    const p = { name, failures: 0, timer: null, stopped: false };
    async function tick() {
      if (p.stopped) return;
      let ok = true;
      if (!(global.document && global.document.hidden)) {
        try { ok = (await task()) !== false; } catch (e) { ok = false; }
      }
      p.failures = ok ? 0 : Math.min(p.failures + 1, 4);
      const base = typeof intervalMs === 'function' ? intervalMs() : intervalMs;
      p.timer = setTimeout(tick, base * Math.pow(2, p.failures));
    }
    p.timer = setTimeout(tick, 0);
    pollers.push(p);
    return p;
  }

  function lidarPointsToRobotFrame(points) {
    const out = [];
    if (!Array.isArray(points)) return out;
    for (let i = 0; i < points.length; i++) {
      const p = points[i];
      const d = (p.distanceMm || 0) / 1000.0;
      if (!(d >= LIDAR_MIN_RANGE_M)) continue;
      const a = (p.angleDeg || 0) * Math.PI / 180.0;
      // RPLiDAR angle is clockwise; robot frame is x forward, y left.
      out.push({ x: d * Math.cos(a), y: -d * Math.sin(a), d, q: p.quality });
    }
    return out;
  }

  // Front-corner VL53L7CX 8x8 ToF sensors: 125 mm measured lateral separation, centered symmetrically (y = ±0.0625 m),
  // angled ±8° outward. Note: forward position x (+0.14 m) and height z (+0.08 m) remain unverified measurements.
  const TOF_MOUNTS = {
    front_left_l7cx:  { x: 0.14, y:  0.0625, yawDeg:  8 },
    front_right_l7cx: { x: 0.14, y: -0.0625, yawDeg: -8 },
  };
  function tofDisplayToSource(z, rot) {
    const r = Math.floor(z / 8), c = z % 8;
    if (rot === '90_ccw') return (7 - c) * 8 + (7 - r);
    if (rot === '90_ccw_noflip') return c * 8 + (7 - r);
    if (rot === '90_cw') return (7 - c) * 8 + r;
    if (rot === '180') return 63 - z;
    return z;
  }
  // One radar point per display column: the nearest valid zone across all 8 rows.
  function tofToRobotFrame(resp) {
    const out = [];
    const data = resp && resp.data;
    if (!resp || resp.stale || !data || !Array.isArray(data.channels)) return out;
    const mapping = resp.mapping || {};
    for (const slot of Object.keys(TOF_MOUNTS)) {
      const chIdx = mapping[slot];
      if (typeof chIdx !== 'number') continue;
      const ch = data.channels.find(c => c.ch === chIdx);
      if (!ch || !ch.present || !ch.init || !Array.isArray(ch.grid_mm) || ch.grid_mm.length < 64) continue;
      if (!(ch.age_ms <= 1000)) continue;
      const rot = String(mapping[slot + '_rot'] || '90_ccw');
      const m = TOF_MOUNTS[slot];
      const my = m.yawDeg * Math.PI / 180;
      for (let c = 0; c < 8; c++) {
        let best = -1;
        for (let r = 0; r < 8; r++) {
          const d = ch.grid_mm[tofDisplayToSource(r * 8 + c, rot)];
          if (typeof d === 'number' && d >= 0 && d <= 4000 && (best < 0 || d < best)) best = d;
        }
        if (best < 0) continue;
        const dm = best / 1000;
        const a = (3.5 - c) * (45 / 8) * Math.PI / 180; // left positive
        const sx = dm * Math.cos(a), sy = dm * Math.sin(a);
        const x = m.x + sx * Math.cos(my) - sy * Math.sin(my);
        const y = m.y + sx * Math.sin(my) + sy * Math.cos(my);
        out.push({ x, y, d: Math.hypot(x, y), rawMm: best, slot });
      }
    }
    return out;
  }

  function evaluateFrontZone(resp, nowMs) {
    const now = typeof nowMs === 'number' ? nowMs : Date.now();
    const offlineBase = {
      state: 'OFFLINE',
      active: false,
      sensorLive: false,
      intervening: false,
      intervention: 'NONE',
      indicatorCode: 'OFFLINE',
      indicatorLevel: 'offline',
      indicatorText: 'PROXIMITY — OFFLINE',
      minMm: null,
      leftMinMm: null,
      rightMinMm: null,
      stopThresholdMm: FRONT_STOP_MM,
      crawlThresholdMm: FRONT_CRAWL_MM,
      crawlSpeedMps: FRONT_CRAWL_LINEAR_MPS,
      receivedAt: now
    };
    if (resp && resp.frontSafetyZone && typeof resp.frontSafetyZone === 'object') {
      const fz = resp.frontSafetyZone;
      const isOff = fz.state === 'OFFLINE' || fz.sensorLive === false || (resp.stale === true);
      if (isOff) {
        return Object.assign({}, offlineBase, fz, {
          state: 'OFFLINE',
          active: false,
          sensorLive: false,
          intervening: false,
          intervention: 'NONE',
          indicatorCode: 'OFFLINE',
          indicatorLevel: 'offline',
          indicatorText: 'PROXIMITY — OFFLINE',
          receivedAt: now
        });
      }
      return Object.assign({
        stopThresholdMm: FRONT_STOP_MM,
        crawlThresholdMm: FRONT_CRAWL_MM,
        crawlSpeedMps: FRONT_CRAWL_LINEAR_MPS,
        sensorLive: true
      }, fz, { receivedAt: now });
    }
    const data = resp && resp.data;
    if (!resp || resp.stale || !data || !Array.isArray(data.channels)) return offlineBase;
    const mapping = resp.mapping || {};
    const slotMins = {};
    let anySensorLive = false;
    for (const slot of Object.keys(TOF_MOUNTS)) {
      const chIdx = mapping[slot];
      if (typeof chIdx !== 'number') continue;
      const ch = data.channels.find(c => c && c.ch === chIdx);
      if (!ch || !ch.present || !ch.init || !(ch.age_ms <= 1000)) continue;
      anySensorLive = true;
      let best = null;
      if (Array.isArray(ch.grid_mm)) {
        for (let i = 0; i < ch.grid_mm.length; i++) {
          const d = ch.grid_mm[i];
          if (typeof d === 'number' && isFinite(d) && d >= 0 && d <= 4000) {
            if (best === null || d < best) best = d;
          }
        }
      }
      if (best === null && typeof ch.min_mm === 'number' && ch.min_mm >= 0 && ch.min_mm <= 4000) {
        best = ch.min_mm;
      }
      slotMins[slot] = best;
    }
    if (!anySensorLive) return offlineBase;
    const leftMinMm = slotMins.front_left_l7cx !== undefined ? slotMins.front_left_l7cx : null;
    const rightMinMm = slotMins.front_right_l7cx !== undefined ? slotMins.front_right_l7cx : null;
    let minMm = null;
    if (leftMinMm !== null && rightMinMm !== null) minMm = Math.min(leftMinMm, rightMinMm);
    else if (leftMinMm !== null) minMm = leftMinMm;
    else if (rightMinMm !== null) minMm = rightMinMm;

    let zState = 'CLEAR';
    let indCode = 'CLEAR';
    let indLevel = 'clear';
    let indText = 'PROXIMITY — CLEAR';
    if (minMm !== null && minMm <= FRONT_STOP_MM) {
      zState = 'STOP';
      indCode = 'STOP_ZONE';
      indLevel = 'bad';
      indText = 'PROXIMITY — STOP ZONE';
    } else if (minMm !== null && minMm <= FRONT_CRAWL_MM) {
      zState = 'CRAWL';
      indCode = 'CRAWL_ZONE';
      indLevel = 'warn';
      indText = 'PROXIMITY — CRAWL ZONE';
    }

    return {
      state: zState,
      active: zState !== 'CLEAR',
      sensorLive: true,
      intervening: false,
      intervention: 'NONE',
      indicatorCode: indCode,
      indicatorLevel: indLevel,
      indicatorText: indText,
      minMm,
      leftMinMm,
      rightMinMm,
      stopThresholdMm: FRONT_STOP_MM,
      crawlThresholdMm: FRONT_CRAWL_MM,
      crawlSpeedMps: FRONT_CRAWL_LINEAR_MPS,
      receivedAt: now
    };
  }

  async function pollTofSensors() {
    const r = await api('/api/tof-sensors', { timeoutMs: 1500 });
    if (!r.ok || !r.data) {
      if (state.frontZone && state.frontZone.receivedAt && (Date.now() - state.frontZone.receivedAt) > FRONT_ZONE_STALE_MS) {
        emit('front_zone', state.frontZone);
      }
      return false;
    }
    state.tof = { points: tofToRobotFrame(r.data), t: Date.now() };
    state.frontZone = evaluateFrontZone(r.data);
    emit('tof', state.tof);
    emit('front_zone', state.frontZone);
    return true;
  }

  async function pollLidarScan() {
    const r = await api('/api/lidar/scan', { timeoutMs: 1500 });
    if (!r.ok || !Array.isArray(r.data.points)) return false;
    const now = Date.now();
    const pts = lidarPointsToRobotFrame(r.data.points);
    const ft = state.scan.frameTimes;
    ft.push(now);
    while (ft.length > 10) ft.shift();
    state.scan = { points: pts, scanHz: isNum(r.data.scanHz) ? r.data.scanHz : null, t: now, frameTimes: ft, pointCount: r.data.pointCount || pts.length };
    emit('scan', state.scan);
    return true;
  }

  function buildMapImage(map) {
    if (!map || !Array.isArray(map.pixels) || !map.width || !map.height) return null;
    if (typeof document === 'undefined') return null;
    const c = document.createElement('canvas');
    c.width = map.width;
    c.height = map.height;
    const octx = c.getContext && c.getContext('2d');
    if (!octx || !octx.createImageData) return null;
    const img = octx.createImageData(map.width, map.height);
    const px = img.data;
    const n = map.width * map.height;
    for (let i = 0; i < n; i++) {
      const v = map.pixels[i];
      const k = i * 4;
      if (v >= 250) {            // free floor (254/255)
        px[k] = 15; px[k + 1] = 27; px[k + 2] = 42;
      } else if (v <= 50) {      // occupied wall
        px[k] = 0; px[k + 1] = 235; px[k + 2] = 210;
      } else if (v === 205) {    // unknown
        px[k] = 6; px[k + 1] = 10; px[k + 2] = 17;
      } else {                   // intermediate
        px[k] = 22; px[k + 1] = 55; px[k + 2] = 82;
      }
      px[k + 3] = 255;
    }
    octx.putImageData(img, 0, 0);
    return c;
  }

  function startPolling() {
    startPoller('tof-sensors', 250, pollTofSensors);
    startPoller('lidar-scan', () => (state.lidarStatus && state.lidarStatus.connected === false) ? 2000 : 200, pollLidarScan);
    startPoller('lidar-status', 2000, async () => {
      const r = await api('/api/lidar/status', { timeoutMs: 2000 });
      if (r.status === 0) return false;
      state.lidarStatus = r.data;
      emit('lidar_status', r.data);
      return true;
    });
    startPoller('system', 3000, async () => {
      const r = await api('/api/system', { timeoutMs: 3000 });
      if (!r.ok) return false;
      state.system = r.data;
      emit('system', r.data);
      return true;
    });
    startPoller('localization', 1000, async () => {
      const r = await api('/api/localization/status', { timeoutMs: 2000 });
      if (!r.ok) return false;
      state.localization = r.data;
      emit('localization', r.data);
      return true;
    });
    startPoller('nav-status', () => (state.nav && /EXEC|ACTIVE|NAVIGAT/i.test(state.nav.status || '')) ? 500 : 1000, async () => {
      const r = await api('/api/navigation/status', { timeoutMs: 2500 });
      if (!r.ok) return false;
      state.nav = r.data;
      emit('nav', r.data);
      return true;
    });
    startPoller('home', 30000, async () => {
      const r = await api('/api/navigation/home', { timeoutMs: 3000 });
      if (!r.ok || !r.data.home) return false;
      state.home = r.data.home;
      emit('home', state.home);
      return true;
    });
    startPoller('autonomy', 2000, async () => {
      const r = await api('/api/autonomy/status', { timeoutMs: 2500 });
      if (!r.ok) return false;
      state.autonomy = r.data;
      emit('autonomy', r.data);
      return true;
    });
    startPoller('slam', 5000, async () => {
      const r = await api('/api/slam/status', { timeoutMs: 3000 });
      if (!r.ok) return false;
      state.slam = r.data;
      emit('slam', r.data);
      return true;
    });
    startPoller('map', 15000, async () => {
      const r = await api('/api/navigation/map', { timeoutMs: 10000 });
      if (!r.ok || !Array.isArray(r.data.pixels)) return false;
      const m = r.data;
      const prev = state.map;
      const changed = !prev || prev.width !== m.width || prev.height !== m.height || prev.map_name !== m.map_name ||
        (prev.pixels && prev.pixels.length === m.pixels.length && prev.pixels.some((v, i) => v !== m.pixels[i]));
      if (changed) {
        state.map = {
          width: m.width, height: m.height, resolution: m.resolution,
          originX: (m.origin || [0, 0])[0], originY: (m.origin || [0, 0])[1],
          map_name: m.map_name, pixels: m.pixels, image: buildMapImage(m)
        };
        state.mapVersion++;
        emit('map', state.map);
      }
      return true;
    });
    startPoller('battery-once', 60000, async () => {
      if (state.battery && Date.now() - (state.battery.receivedAt || 0) < 20000) return true;
      const r = await api('/api/battery', { timeoutMs: 3000 });
      if (!r.ok) return false;
      state.battery = Object.assign({}, state.battery || {}, r.data, { receivedAt: Date.now() });
      emit('battery', state.battery);
      return true;
    });
  }

  // ────────────────────────────────────────────────────────────
  // Pose (map frame when AMCL localized, otherwise wheel odometry)
  // ────────────────────────────────────────────────────────────
  function getPose() {
    const loc = state.localization;
    if (loc && loc.localized && isNum(loc.x) && isNum(loc.y)) {
      const yawDeg = isNum(loc.yawDeg) ? loc.yawDeg : (isNum(loc.yaw) ? loc.yaw * 180 / Math.PI : 0);
      return { x: loc.x, y: loc.y, yawRad: yawDeg * Math.PI / 180, yawDeg, frame: 'map', localized: true };
    }
    const od = state.odom;
    if (od && isNum(od.x) && isNum(od.y)) {
      const yaw = isNum(od.yaw) ? od.yaw : 0;
      return { x: od.x, y: od.y, yawRad: yaw, yawDeg: yaw * 180 / Math.PI, frame: 'odom', localized: false };
    }
    return { x: 0, y: 0, yawRad: 0, yawDeg: 0, frame: 'none', localized: false };
  }

  // ────────────────────────────────────────────────────────────
  // Manual drive streaming (10 Hz, keepalive:true so server watchdog applies)
  // ────────────────────────────────────────────────────────────
  const drive = { throttle: 0.5, sources: {}, zeroFramesLeft: 0, lastCmd: { x: 0, y: 0 }, lastHintAt: 0 };
  const SOURCE_PRIORITY = ['gamepad', 'touch', 'keyboard'];

  function setThrottle(frac) { drive.throttle = clamp(Number(frac) || 0, 0.1, 1.0); emit('throttle', drive.throttle); }
  function getThrottle() { return drive.throttle; }
  function setDriveInput(src, x, y, boost) {
    drive.sources[src] = {
      x: clamp(Number(x) || 0, -1, 1),
      y: clamp(Number(y) || 0, -1, 1),
      boost: clamp(Number(boost) || 1, 1, 2),
      t: Date.now()
    };
  }
  function clearDriveInput(src) { delete drive.sources[src]; }
  function releaseAllDriveInputs() { drive.sources = {}; }

  function currentDriveVector() {
    for (const src of SOURCE_PRIORITY) {
      const s = drive.sources[src];
      if (s && (Math.abs(s.x) > 0.001 || Math.abs(s.y) > 0.001)) return { x: s.x, y: s.y, boost: s.boost || 1, src };
    }
    return { x: 0, y: 0, boost: 1, src: null };
  }

  function hint(text) {
    const now = Date.now();
    if (now - drive.lastHintAt < 1200) return;
    drive.lastHintAt = now;
    emit('hint', text);
  }

  function driveTick() {
    const v = currentDriveVector();
    const thr = Math.min(1, drive.throttle * v.boost);
    let x = clamp(v.x * thr, -1, 1);
    let y = clamp(v.y * thr, -1, 1);
    const nonZero = Math.abs(x) > 0.001 || Math.abs(y) > 0.001;
    let sent = { x: 0, y: 0 };
    if (nonZero) {
      if (!state.wsConnected) {
        hint('Not connected to rover');
      } else if (!state.authOk) {
        hint('Enter operator token to drive');
      } else if (!state.drive || !state.drive.armed) {
        hint('Drivetrain DISARMED — click ARM first');
      } else {
        const fz = state.frontZone;
        if (fz && fz.state === 'STOP' && y >= -0.001) {
          hint(`🛑 FRONT STOP (${fz.minMm} mm ≤ ${FRONT_STOP_MM} mm) — forward stopped (reverse allowed)`);
        } else if (fz && fz.state === 'CRAWL' && y > 0.001) {
          hint(`⚠️ FRONT CRAWL (${fz.minMm} mm ≤ ${FRONT_CRAWL_MM} mm) — slowed to crawl`);
        }
        wsSend({ type: 'joystick', x, y, keepalive: true });
        drive.zeroFramesLeft = JOY_ZERO_FRAMES;
        sent = { x, y };
      }
    } else if (drive.zeroFramesLeft > 0) {
      wsSend({ type: 'joystick', x: 0, y: 0, keepalive: true });
      drive.zeroFramesLeft--;
    }
    drive.lastCmd = sent;
    emit('drive_cmd', { x: sent.x, y: sent.y, rawX: v.x, rawY: v.y, src: v.src });
  }

  // Estimated commanded velocity (mirrors server.js joystick shaping + front safety zone) for display only.
  function estimateCommandedVelocity(x, y) {
    const throttle = Math.abs(y) < 0.10 ? 0 : Math.sign(y) * Math.pow(Math.abs(y), 2.0);
    let turn = 0;
    if (Math.abs(x) >= 0.10) {
      const n = (Math.abs(x) - 0.10) / 0.90;
      turn = Math.sign(x) * (0.35 + Math.pow(n, 1.5) * 0.65);
    }
    let linear = throttle * MANUAL_MAX_LINEAR_MPS;
    let angular = -turn * MANUAL_MAX_ANGULAR_RADPS;
    const fz = state.frontZone;
    if (fz && fz.state === 'STOP') {
      if (linear >= -1e-4) {
        linear = 0;
        angular = 0;
      } else {
        angular = clamp(angular, -FRONT_CRAWL_ANGULAR_RADPS, FRONT_CRAWL_ANGULAR_RADPS);
      }
    } else if (fz && fz.state === 'CRAWL') {
      if (linear > FRONT_CRAWL_LINEAR_MPS) {
        const scale = FRONT_CRAWL_LINEAR_MPS / linear;
        linear = FRONT_CRAWL_LINEAR_MPS;
        angular = clamp(angular * scale, -FRONT_CRAWL_ANGULAR_RADPS, FRONT_CRAWL_ANGULAR_RADPS);
      } else if (Math.abs(linear) <= 1e-4 && Math.abs(angular) > FRONT_CRAWL_ANGULAR_RADPS) {
        angular = clamp(angular, -FRONT_CRAWL_ANGULAR_RADPS, FRONT_CRAWL_ANGULAR_RADPS);
      }
    }
    return { linear, angular };
  }

  // ────────────────────────────────────────────────────────────
  // Safety: E-STOP / disarm / arm
  // ────────────────────────────────────────────────────────────
  let lastEstopAt = 0;
  function emergencyStop(sourceLabel) {
    releaseAllDriveInputs();
    drive.zeroFramesLeft = 0;
    // 1. Zero manual target with explicit deadman release.
    wsSend({ type: 'joystick', x: 0, y: 0, deadman: false });
    // 2. Firmware disarm + emergency stop (neither requires auth on server.js).
    wsSend({ type: 'disarm_normal_drive' });
    wsSend({ type: 'emergency_stop' });
    // 3. HTTP paths in case the WebSocket is down.
    const now = Date.now();
    if (now - lastEstopAt > 250) {
      lastEstopAt = now;
      api('/api/drive/disarm', { method: 'POST', timeoutMs: 3000 });
      api('/api/navigation/cancel', { method: 'POST', timeoutMs: 3000 });
    }
    state.drive = Object.assign({}, state.drive, { armed: false });
    emit('drive', state.drive);
    emit('estop', { source: sourceLabel || 'E-STOP', t: now });
  }

  async function disarm() {
    releaseAllDriveInputs();
    wsSend({ type: 'joystick', x: 0, y: 0, deadman: false });
    wsSend({ type: 'disarm_normal_drive' });
    const r = await api('/api/drive/disarm', { method: 'POST', timeoutMs: 3000 });
    return r;
  }

  async function arm() {
    if (!getToken()) {
      return { ok: false, data: { error: 'Operator token required to arm' } };
    }
    releaseAllDriveInputs();
    const r = await api('/api/drive/arm', { method: 'POST', auth: true, body: {}, timeoutMs: 5000 });
    return r;
  }

  // ────────────────────────────────────────────────────────────
  // Navigation helpers (Nav2 via server.js)
  // ────────────────────────────────────────────────────────────
  function goalBody(goal) {
    const p = getPose();
    return {
      start_x: p.x, start_y: p.y, start_yaw: p.yawRad,
      target_x: goal.x, target_y: goal.y, target_yaw: goal.yawRad,
      x: goal.x, y: goal.y, yaw: goal.yawRad
    };
  }
  async function planPath(goal) {
    const r = await api('/api/navigation/plan', { method: 'POST', body: goalBody(goal), timeoutMs: 15000 });
    if (r.ok) {
      r.path = toPath(r.data.waypoints || r.data.path || []);
    }
    return r;
  }
  async function dispatchGoal(goal) {
    if (!getToken()) return { ok: false, data: { error: 'Operator token required to dispatch' } };
    return api('/api/navigation/dispatch', { method: 'POST', auth: true, body: goalBody(goal), timeoutMs: 20000 });
  }
  async function cancelNav() {
    return api('/api/navigation/cancel', { method: 'POST', timeoutMs: 4000 });
  }
  async function initHome() {
    return api('/api/navigation/init_home', { method: 'POST', body: {}, timeoutMs: 10000 });
  }
  function homeGoal() {
    const h = state.home;
    if (!h || !isNum(h.x) || !isNum(h.y)) return null;
    const yawRad = isNum(h.yaw_rad) ? h.yaw_rad : (isNum(h.yaw_deg) ? h.yaw_deg * Math.PI / 180 : 0);
    return { x: h.x, y: h.y, yawRad, label: 'HOME' };
  }
  function relativeGoal(distM, angleDeg) {
    const p = getPose();
    const yaw = normAngle(p.yawRad + (angleDeg || 0) * Math.PI / 180);
    // Turn first (in place) then translate along the new heading — matches Rover 2 preset semantics.
    return {
      x: p.x + distM * Math.cos(yaw),
      y: p.y + distM * Math.sin(yaw),
      yawRad: yaw,
      label: `${distM ? (distM > 0 ? '+' : '') + distM.toFixed(2) + 'm' : ''}${angleDeg ? ' ' + (angleDeg > 0 ? '+' : '') + angleDeg + '°' : ''}`.trim()
    };
  }

  // Mission script parser (ported from legacy app.js parseClientMissionScript).
  function parseMissionScript(scriptText, startPose, homePose) {
    const commands = String(scriptText || '').split(/[,;\n]+/).map(c => c.trim()).filter(c => c.length > 0);
    const legs = [];
    let currX = startPose.x;
    let currY = startPose.y;
    let currYaw = startPose.yawRad || 0;
    const unitToM = (val, unit) => {
      unit = (unit || 'm').toLowerCase();
      if (unit === 'ft' || unit === 'feet' || unit === 'foot') return val * 0.3048;
      if (unit === 'in' || unit === 'inch' || unit === 'inches') return val * 0.0254;
      if (unit === 'cm') return val * 0.01;
      return val;
    };
    for (let idx = 0; idx < commands.length; idx++) {
      const cmd = commands[idx];
      const c = cmd.toLowerCase();
      const leg = { index: idx + 1, command: cmd, start: { x: currX, y: currY, yaw: currYaw } };
      if (c.includes('home')) {
        if (!homePose) throw new Error('HOME pose is not available from the nav bridge');
        leg.type = 'RETURN_HOME';
        leg.target = { x: homePose.x, y: homePose.y, yaw: homePose.yawRad };
        legs.push(leg);
        currX = homePose.x; currY = homePose.y; currYaw = homePose.yawRad;
        continue;
      }
      const mFwd = c.match(/(?:forward|fwd|move|advance)\s+([0-9.]+)\s*(feet|foot|ft|meters|meter|m|inches|inch|in|cm)?/);
      if (mFwd) {
        const dist = unitToM(parseFloat(mFwd[1]), mFwd[2]);
        currX += dist * Math.cos(currYaw);
        currY += dist * Math.sin(currYaw);
        leg.type = 'FORWARD';
        leg.distance_m = Math.round(dist * 1000) / 1000;
        leg.target = { x: currX, y: currY, yaw: currYaw };
        legs.push(leg);
        continue;
      }
      const mBack = c.match(/(?:backward|back|reverse)\s+([0-9.]+)\s*(feet|foot|ft|meters|meter|m|inches|inch|in|cm)?/);
      if (mBack) {
        const dist = unitToM(parseFloat(mBack[1]), mBack[2]);
        currX -= dist * Math.cos(currYaw);
        currY -= dist * Math.sin(currYaw);
        leg.type = 'BACKWARD';
        leg.distance_m = Math.round(dist * 1000) / 1000;
        leg.target = { x: currX, y: currY, yaw: currYaw };
        legs.push(leg);
        continue;
      }
      const mTurn = c.match(/(?:turn\s+)?(left|right|cw|ccw)\s+([0-9.]+)\s*(?:deg|degrees)?/);
      if (mTurn) {
        const deg = parseFloat(mTurn[2]);
        let rad = deg * Math.PI / 180.0;
        if (mTurn[1] === 'right' || mTurn[1] === 'cw') rad = -rad;
        currYaw = normAngle(currYaw + rad);
        leg.type = 'TURN';
        leg.angle_deg = (mTurn[1] === 'left' || mTurn[1] === 'ccw') ? deg : -deg;
        leg.target = { x: currX, y: currY, yaw: currYaw };
        legs.push(leg);
        continue;
      }
      const mGoto = c.match(/goto\s+([0-9.-]+)\s+([0-9.-]+)(?:\s+([0-9.-]+))?/);
      if (mGoto) {
        currX = parseFloat(mGoto[1]);
        currY = parseFloat(mGoto[2]);
        currYaw = normAngle((mGoto[3] ? parseFloat(mGoto[3]) : 0) * Math.PI / 180.0);
        leg.type = 'GOTO';
        leg.target = { x: currX, y: currY, yaw: currYaw };
        legs.push(leg);
        continue;
      }
      throw new Error(`Unrecognized command: '${cmd}'`);
    }
    return legs;
  }

  // Preview-only: plans each leg with Nav2 and returns the concatenated path. Never dispatches.
  async function previewMission(scriptText) {
    const start = getPose();
    const legs = parseMissionScript(scriptText, start, homeGoal());
    const fullPath = [];
    let total = 0;
    let legStart = { x: start.x, y: start.y, yawRad: start.yawRad };
    for (const leg of legs) {
      const body = {
        start_x: legStart.x, start_y: legStart.y, start_yaw: legStart.yawRad,
        target_x: leg.target.x, target_y: leg.target.y, target_yaw: leg.target.yaw,
        x: leg.target.x, y: leg.target.y, yaw: leg.target.yaw
      };
      if (leg.type === 'TURN') {
        fullPath.push({ x: leg.target.x, y: leg.target.y });
      } else {
        const r = await api('/api/navigation/plan', { method: 'POST', body, timeoutMs: 15000 });
        if (!r.ok) {
          return { ok: false, legs, path: fullPath, error: `Leg ${leg.index} (${leg.command}): ${r.data.error || 'planning failed'}` };
        }
        toPath(r.data.waypoints || []).forEach(p => fullPath.push(p));
        total += Number(r.data.cumulative_length_m || 0);
      }
      legStart = { x: leg.target.x, y: leg.target.y, yawRad: leg.target.yaw };
    }
    return { ok: true, legs, path: fullPath, lengthM: total };
  }

  // ────────────────────────────────────────────────────────────
  // Map view model + renderer (style ported from Rover 2 drawTacticalNavMap)
  // ────────────────────────────────────────────────────────────
  function createMapView(opts) {
    return {
      scale: opts && opts.scale || 95,
      miniScale: opts && opts.miniScale || 48,
      follow: true,
      pan: { x: 0, y: 0 },
      goal: null,
      plannedPath: [],
      trail: [],
      lastTrailAt: 0
    };
  }
  function mapCenter(view) {
    const p = getPose();
    return view.follow ? { x: p.x + view.pan.x, y: p.y + view.pan.y } : { x: view.pan.x, y: view.pan.y };
  }
  function worldToCanvas(view, wx, wy, cw, ch, scale) {
    const c = mapCenter(view);
    return { u: cw * 0.5 + (wx - c.x) * scale, v: ch * 0.5 - (wy - c.y) * scale };
  }
  function canvasToWorld(view, u, v, cw, ch, scale) {
    const c = mapCenter(view);
    return { x: c.x + (u - cw * 0.5) / scale, y: c.y - (v - ch * 0.5) / scale };
  }

  function drawMap(ctx, view, cw, ch, isMini) {
    if (!ctx) return;
    const scale = isMini ? view.miniScale : view.scale;
    const w2c = (x, y) => worldToCanvas(view, x, y, cw, ch, scale);
    const pose = getPose();

    ctx.clearRect(0, 0, cw, ch);
    ctx.fillStyle = '#060a11';
    ctx.fillRect(0, 0, cw, ch);

    // 1. Occupancy grid (only meaningful in map frame)
    const map = state.map;
    if (map && map.image) {
      const tl = w2c(map.originX, map.originY + map.height * map.resolution);
      ctx.save();
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(map.image, tl.u, tl.v, map.width * map.resolution * scale, map.height * map.resolution * scale);
      ctx.restore();
    }

    // 2. 0.5 m grid
    const c = mapCenter(view);
    const hx = (cw * 0.5) / scale;
    const hy = (ch * 0.5) / scale;
    ctx.lineWidth = 1;
    for (let gx = Math.floor((c.x - hx) * 2) / 2; gx <= Math.ceil((c.x + hx) * 2) / 2; gx += 0.5) {
      const pt = w2c(gx, 0);
      const isOrigin = Math.abs(gx) < 0.001;
      const isMajor = Math.abs(gx % 1.0) < 0.001;
      ctx.strokeStyle = isOrigin ? 'rgba(0, 255, 204, 0.28)' : (isMajor ? 'rgba(95, 122, 145, 0.18)' : 'rgba(95, 122, 145, 0.08)');
      ctx.beginPath(); ctx.moveTo(pt.u, 0); ctx.lineTo(pt.u, ch); ctx.stroke();
      if (!isMini && isMajor && pt.u > 35 && pt.u < cw - 35) {
        ctx.fillStyle = 'rgba(139, 162, 181, 0.45)'; ctx.font = '9px monospace';
        ctx.fillText(`${gx.toFixed(0)}m`, pt.u + 3, ch - 6);
      }
    }
    for (let gy = Math.floor((c.y - hy) * 2) / 2; gy <= Math.ceil((c.y + hy) * 2) / 2; gy += 0.5) {
      const pt = w2c(0, gy);
      const isOrigin = Math.abs(gy) < 0.001;
      const isMajor = Math.abs(gy % 1.0) < 0.001;
      ctx.strokeStyle = isOrigin ? 'rgba(0, 255, 204, 0.28)' : (isMajor ? 'rgba(95, 122, 145, 0.18)' : 'rgba(95, 122, 145, 0.08)');
      ctx.beginPath(); ctx.moveTo(0, pt.v); ctx.lineTo(cw, pt.v); ctx.stroke();
      if (!isMini && isMajor && pt.v > 45 && pt.v < ch - 45) {
        ctx.fillStyle = 'rgba(139, 162, 181, 0.45)'; ctx.font = '9px monospace';
        ctx.fillText(`${gy.toFixed(0)}m`, 6, pt.v - 3);
      }
    }

    // 3. Live LiDAR projected into the current frame
    const cosR = Math.cos(pose.yawRad), sinR = Math.sin(pose.yawRad);
    if (state.scan.points.length && Date.now() - state.scan.t < 3000) {
      ctx.fillStyle = '#00ff66';
      const sz = isMini ? 1.8 : 2.8;
      for (let i = 0; i < state.scan.points.length; i += (isMini ? 2 : 1)) {
        const lp = state.scan.points[i];
        const cp = w2c(pose.x + cosR * lp.x - sinR * lp.y, pose.y + sinR * lp.x + cosR * lp.y);
        if (cp.u >= 0 && cp.u <= cw && cp.v >= 0 && cp.v <= ch) ctx.fillRect(cp.u - sz / 2, cp.v - sz / 2, sz, sz);
      }
    }

    // 4. Trail
    if (view.trail.length >= 2) {
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.42)';
      ctx.lineWidth = isMini ? 1.5 : 2;
      ctx.beginPath();
      view.trail.forEach((p, i) => { const cp = w2c(p.x, p.y); if (i === 0) ctx.moveTo(cp.u, cp.v); else ctx.lineTo(cp.u, cp.v); });
      ctx.stroke();
    }

    // 5. HOME pad
    const home = homeGoal();
    if (home && pose.frame !== 'odom') {
      const hp = w2c(home.x, home.y);
      const hr = Math.max(8, 0.14 * scale);
      ctx.save();
      ctx.fillStyle = 'rgba(0, 255, 136, 0.16)'; ctx.strokeStyle = '#00ff88'; ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.arc(hp.u, hp.v, hr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(hp.u, hp.v);
      ctx.lineTo(hp.u + Math.cos(home.yawRad) * hr * 1.35, hp.v - Math.sin(home.yawRad) * hr * 1.35); ctx.stroke();
      if (!isMini) { ctx.fillStyle = '#00ff88'; ctx.font = 'bold 10px sans-serif'; ctx.fillText('🏠 HOME', hp.u + hr + 4, hp.v + 3); }
      ctx.restore();
    }

    // 6. Paths: active Nav2 global path (green) or previewed plan (cyan dashed)
    const nav = state.nav;
    const active = nav && Array.isArray(nav.global_path) && nav.global_path.length >= 2 ? toPath(nav.global_path) : null;
    const path = active || view.plannedPath;
    if (path && path.length >= 2) {
      const executing = !!active;
      ctx.save();
      ctx.strokeStyle = executing ? '#00ff88' : '#00e5ff';
      ctx.lineWidth = isMini ? 2.2 : 3.2;
      ctx.shadowColor = executing ? 'rgba(0,255,136,0.6)' : 'rgba(0,229,255,0.6)';
      ctx.shadowBlur = 8;
      if (!executing) ctx.setLineDash([7, 5]);
      ctx.beginPath();
      path.forEach((p, i) => { const cp = w2c(p.x, p.y); if (i === 0) ctx.moveTo(cp.u, cp.v); else ctx.lineTo(cp.u, cp.v); });
      ctx.stroke();
      ctx.restore();
    }

    // 7. Goal reticle
    if (view.goal) {
      const gp = w2c(view.goal.x, view.goal.y);
      const gr = isMini ? 6 : 11;
      ctx.save();
      ctx.strokeStyle = '#ffaa00'; ctx.fillStyle = 'rgba(255, 170, 0, 0.22)'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(gp.u, gp.v, gr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      const al = isMini ? 13 : 24;
      ctx.beginPath(); ctx.moveTo(gp.u, gp.v);
      ctx.lineTo(gp.u + Math.cos(view.goal.yawRad) * al, gp.v - Math.sin(view.goal.yawRad) * al); ctx.stroke();
      if (!isMini) { ctx.fillStyle = '#ffaa00'; ctx.font = 'bold 10px sans-serif'; ctx.fillText(`🎯 ${view.goal.label || 'GOAL'}`, gp.u + gr + 5, gp.v - 4); }
      ctx.restore();
    }

    // 8. Rover 1 footprint (approx. 0.30 m L x 0.34 m W track) + nose arrow
    const rp = w2c(pose.x, pose.y);
    ctx.save();
    ctx.translate(rp.u, rp.v);
    ctx.rotate(-pose.yawRad);
    const halfL = Math.max(2, 0.15 * scale), halfW = Math.max(2, 0.17 * scale);
    ctx.fillStyle = 'rgba(0, 255, 204, 0.22)'; ctx.strokeStyle = '#00ffcc'; ctx.lineWidth = 2;
    ctx.fillRect(-halfL, -halfW, halfL * 2, halfW * 2); ctx.strokeRect(-halfL, -halfW, halfL * 2, halfW * 2);
    ctx.fillStyle = '#ff2a4b';
    ctx.beginPath(); ctx.moveTo(halfL + 6, 0); ctx.lineTo(halfL - 5, -5); ctx.lineTo(halfL - 5, 5); ctx.closePath(); ctx.fill();
    ctx.restore();

    // 9. Frame warning + legend (full map only)
    if (!isMini) {
      if (pose.frame !== 'map') {
        const txt = pose.frame === 'odom' ? '⚠️ NOT LOCALIZED — showing wheel-odometry pose (map overlay not aligned)' : '⚠️ NO POSE — waiting for localization / odometry';
        ctx.save();
        ctx.font = 'bold 11px monospace';
        const tw = Math.min(cw - 24, ctx.measureText(txt).width + 22);
        ctx.fillStyle = 'rgba(55, 32, 4, 0.92)'; ctx.strokeStyle = '#ffaa00'; ctx.lineWidth = 1.5;
        ctx.fillRect((cw - tw) / 2, 52, tw, 24); ctx.strokeRect((cw - tw) / 2, 52, tw, 24);
        ctx.fillStyle = '#ffdd55'; ctx.textAlign = 'center'; ctx.fillText(txt, cw / 2, 68, tw - 12);
        ctx.restore();
      }
      ctx.save();
      const lx = 10, ly = ch - 128;
      ctx.fillStyle = 'rgba(6, 12, 20, 0.82)'; ctx.strokeStyle = 'rgba(95, 122, 145, 0.35)'; ctx.lineWidth = 1;
      ctx.fillRect(lx, ly, 200, 58); ctx.strokeRect(lx, ly, 200, 58);
      ctx.font = '10px monospace';
      ctx.fillStyle = '#00ff88'; ctx.fillText('━━ Active Nav2 Path', lx + 8, ly + 14);
      ctx.fillStyle = '#00e5ff'; ctx.fillText('┈┈ Previewed Plan', lx + 8, ly + 27);
      ctx.fillStyle = 'rgba(56, 189, 248, 0.85)'; ctx.fillText('─── Driven Trail', lx + 8, ly + 40);
      ctx.fillStyle = '#00ff66'; ctx.fillText('■ Live RPLiDAR C1', lx + 8, ly + 53);
      ctx.restore();
    }
  }

  function updateTrail(view) {
    const p = getPose();
    if (p.frame === 'none') return;
    const now = Date.now();
    const last = view.trail[view.trail.length - 1];
    if (!last || (now - view.lastTrailAt > 400 && Math.hypot(p.x - last.x, p.y - last.y) > 0.03)) {
      view.trail.push({ x: p.x, y: p.y });
      view.lastTrailAt = now;
      if (view.trail.length > 600) view.trail.shift();
    }
  }

  // ────────────────────────────────────────────────────────────
  // Radar Proximity Indicator (authoritative server frontSafetyZone)
  // ────────────────────────────────────────────────────────────
  function evaluateRadarDriveIndicator(nowMs) {
    const now = typeof nowMs === 'number' ? nowMs : Date.now();
    const fz = state.frontZone || null;

    const isStaleByAge = Boolean(
      fz && (
        (typeof fz.receivedAt === 'number' && fz.receivedAt > 0 && (now - fz.receivedAt) > FRONT_ZONE_STALE_MS) ||
        (typeof fz.hostAgeMs === 'number' && fz.hostAgeMs > FRONT_ZONE_STALE_MS)
      )
    );
    const isOffline = Boolean(
      !fz ||
      fz.stale === true ||
      fz.sensorLive === false ||
      fz.state === 'OFFLINE' ||
      fz.indicatorCode === 'OFFLINE' ||
      isStaleByAge
    );

    if (isOffline) {
      return {
        code: 'OFFLINE',
        level: 'offline',
        text: 'PROXIMITY — OFFLINE',
        shortText: 'PROXIMITY — OFFLINE',
        state: 'OFFLINE',
        sensorLive: false,
        intervening: false,
        slowDownOverride: false,
        brakeApplied: false,
        zoneActive: false,
        minMm: null
      };
    }

    const zState = fz.state || 'CLEAR';
    const intervening = Boolean(
      fz.intervening === true ||
      fz.indicatorCode === 'SLOWING' ||
      fz.indicatorCode === 'STOP'
    );

    let code = 'CLEAR';
    let level = 'clear';
    let text = 'PROXIMITY — CLEAR';

    if (zState === 'STOP' || fz.indicatorCode === 'STOP' || fz.indicatorCode === 'STOP_ZONE') {
      if (intervening) {
        code = 'STOP';
        level = 'bad';
        text = 'PROXIMITY — STOP';
      } else {
        code = 'STOP_ZONE';
        level = 'bad';
        text = 'PROXIMITY — STOP ZONE';
      }
    } else if (zState === 'CRAWL' || fz.indicatorCode === 'SLOWING' || fz.indicatorCode === 'CRAWL_ZONE') {
      if (intervening) {
        code = 'SLOWING';
        level = 'warn';
        text = 'PROXIMITY — SLOWING';
      } else {
        code = 'CRAWL_ZONE';
        level = 'warn';
        text = 'PROXIMITY — CRAWL ZONE';
      }
    }

    return {
      code,
      level,
      text,
      shortText: text,
      state: zState,
      sensorLive: true,
      intervening,
      slowDownOverride: code === 'SLOWING',
      brakeApplied: code === 'STOP',
      zoneActive: zState === 'CRAWL' || zState === 'STOP',
      minMm: isNum(fz.minMm) ? fz.minMm : null
    };
  }

  // ────────────────────────────────────────────────────────────
  // Radar renderer (Rover 2 obstacle radar look, LiDAR + Front ToF)
  // ────────────────────────────────────────────────────────────
  function radarRings(maxR) {
    if (maxR <= 0.3) return [0.1, 0.2, 0.25];
    if (maxR <= 0.6) return [0.1, 0.25, 0.5];
    if (maxR <= 1.2) return [0.25, 0.5, 1.0];
    if (maxR <= 2.1) return [0.5, 1.0, 1.5, 2.0];
    if (maxR <= 3.1) return [1.0, 2.0, 3.0];
    if (maxR <= 5.1) return [1.0, 2.5, 5.0];
    return [2.0, 4.0, 6.0, 8.0];
  }
  function drawRadar(ctx, size, opts) {
    if (!ctx) return null;
    const maxR = (opts && opts.maxRange) || 3.0;
    const colorMode = (opts && opts.color) || 'range';
    const ptSize = (opts && opts.pointSize) || 2.4;
    const cx = size / 2;
    // Shift radar circles slightly above vertical center so the bottom margin of the radar box
    // (outside the outer radar circle) has dedicated clearance for the override/brake/power indicator.
    const cy = Math.round(size * 0.445);
    const outer = size * 0.335;
    const scale = outer / maxR;
    ctx.fillStyle = '#030508';
    ctx.fillRect(0, 0, size, size);
    ctx.lineWidth = 1;
    radarRings(maxR).forEach(r => {
      ctx.strokeStyle = r >= maxR - 0.05 ? 'rgba(0, 255, 204, 0.30)' : 'rgba(0, 255, 204, 0.13)';
      ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI); ctx.stroke();
      ctx.fillStyle = 'rgba(95, 122, 145, 0.75)'; ctx.font = '8px monospace'; ctx.textAlign = 'left';
      const label = maxR < 1.0 ? `${Math.round(r * 100)}cm` : `${r}m`;
      ctx.fillText(label, cx + 3, cy - r * scale + 8);
    });
    ctx.strokeStyle = 'rgba(0, 255, 204, 0.15)';
    ctx.beginPath();
    ctx.moveTo(cx, cy - outer - 3); ctx.lineTo(cx, cy + outer + 3);
    ctx.moveTo(cx - outer - 3, cy); ctx.lineTo(cx + outer + 3, cy);
    ctx.stroke();
    if (size >= 200) {
      ctx.fillStyle = 'rgba(95, 122, 145, 0.75)'; ctx.font = '8px Segoe UI, sans-serif';
      ctx.textAlign = 'left'; ctx.fillText('LEFT', 6, cy - 3);
      ctx.textAlign = 'right'; ctx.fillText('RIGHT', size - 6, cy - 3);
      ctx.textAlign = 'center';
      ctx.fillText('FWD', cx, Math.max(12, Math.round(cy - outer - 5)));
      ctx.fillText('REAR', cx, Math.round(cy + outer + 11));
      ctx.textAlign = 'left';
    }
    const rw = Math.max(6, 0.34 * scale), rl = Math.max(8, 0.30 * scale);
    const fz = state.frontZone || { state: 'CLEAR', minMm: null };
    const driveIndicator = evaluateRadarDriveIndicator();

    ctx.strokeStyle = 'rgba(0, 255, 204, 0.45)'; ctx.fillStyle = 'rgba(0, 255, 204, 0.08)';
    ctx.fillRect(cx - rw / 2, cy - rl / 2, rw, rl); ctx.strokeRect(cx - rw / 2, cy - rl / 2, rw, rl);
    ctx.fillStyle = '#00ffcc'; ctx.beginPath(); ctx.arc(cx, cy, 2.5, 0, 2 * Math.PI); ctx.fill();

    let nearest = null;
    const lidarLive = Boolean(state.scan.points.length && Date.now() - state.scan.t < 2500);
    if (lidarLive) {
      for (const p of state.scan.points) {
        if (!nearest || p.d < nearest.d) nearest = p;
        if (p.d > maxR) continue;
        // Robot frame x forward → canvas up; y left → canvas left.
        const u = cx - p.y * scale;
        const v = cy - p.x * scale;
        if (colorMode === 'range') {
          const t = clamp(p.d / maxR, 0, 1);
          ctx.fillStyle = t < 0.2 ? '#ff4455' : (t < 0.4 ? '#ffaa00' : (t < 0.7 ? '#00ff66' : '#00d4ff'));
        } else {
          ctx.fillStyle = '#00ff66';
        }
        ctx.fillRect(u - ptSize / 2, v - ptSize / 2, ptSize, ptSize);
      }
    }
    // Front ToF (VL53L7CX 8x8) overlay: magenta squares.
    const tofLive = Boolean(state.tof.points.length && Date.now() - state.tof.t < 2500);
    if (tofLive) {
      const tsz = ptSize + 1.6;
      ctx.fillStyle = '#ff44ee';
      for (const p of state.tof.points) {
        if (!nearest || p.d < nearest.d) nearest = p;
        if (p.d > maxR) continue;
        ctx.fillRect(cx - p.y * scale - tsz / 2, cy - p.x * scale - tsz / 2, tsz, tsz);
      }
    }
    // On mobile/canvas radar views without the DOM #hud-radar-drive-ind bar (or when explicitly
    // requested via opts.drawIndicatorOnCanvas), render the proximity indicator strip in the
    // dedicated bottom margin below the radar circles at all responsive sizes (<200 and >=200).
    const hasDomInd = Boolean(global.document && global.document.getElementById && global.document.getElementById('hud-radar-drive-ind'));
    const drawOnCanvas = (opts && typeof opts.drawIndicatorOnCanvas === 'boolean') ? opts.drawIndicatorOnCanvas : !hasDomInd;
    if (drawOnCanvas) {
      const barH = Math.max(14, Math.min(20, Math.round(size * 0.08)));
      const padX = Math.max(3, Math.round(size * 0.02));
      const padBottom = Math.max(2, Math.round(size * 0.012));
      const barY = size - barH - padBottom;
      const barX = padX;
      const barW = Math.max(40, size - padX * 2);
      if (driveIndicator.level === 'bad') {
        ctx.fillStyle = 'rgba(255, 42, 75, 0.30)';
        ctx.strokeStyle = '#ff2a4b';
      } else if (driveIndicator.level === 'warn') {
        ctx.fillStyle = 'rgba(255, 170, 0, 0.25)';
        ctx.strokeStyle = '#ffaa00';
      } else if (driveIndicator.level === 'clear') {
        ctx.fillStyle = 'rgba(16, 28, 42, 0.85)';
        ctx.strokeStyle = 'rgba(0, 255, 204, 0.35)';
      } else {
        ctx.fillStyle = 'rgba(20, 27, 36, 0.78)';
        ctx.strokeStyle = 'rgba(115, 134, 153, 0.45)';
      }
      ctx.fillRect(barX, barY, barW, barH);
      ctx.strokeRect(barX, barY, barW, barH);
      ctx.fillStyle = driveIndicator.level === 'bad' ? '#ffccd5' : (
        driveIndicator.level === 'warn' ? '#ffdd55' : (
          driveIndicator.level === 'clear' ? '#b8d4e8' : '#7f93a6'
        )
      );
      const fontPx = Math.max(8, Math.min(10, Math.round(size * 0.048)));
      ctx.font = `bold ${fontPx}px monospace`;
      ctx.textAlign = 'center';
      ctx.fillText(driveIndicator.text, cx, Math.round(barY + barH * 0.70), barW - 6);
      ctx.textAlign = 'left';
    }
    return {
      live: Boolean(lidarLive || tofLive),
      lidarLive,
      tofLive,
      nearest,
      frontZone: fz,
      driveIndicator
    };
  }

  // ────────────────────────────────────────────────────────────
  // Formatting helpers shared by both layouts
  // ────────────────────────────────────────────────────────────
  function tempColor(t) {
    if (!isNum(t)) return '#888888';
    if (t >= 80) return '#ff3344';
    if (t >= 70) return '#ffaa00';
    if (t >= 60) return '#ffe066';
    return '#00ff88';
  }
  function formatMinutes(mins) {
    if (!isNum(mins) || mins <= 0) return '--';
    if (mins < 60) return `${Math.round(mins)} min`;
    return `${Math.floor(mins / 60)}h ${String(Math.round(mins % 60)).padStart(2, '0')}m`;
  }
  function batteryView(b) {
    if (!b || !isNum(b.voltage) || b.voltage <= 2.0) {
      return { pct: null, text: '--%', voltText: '-- V', level: 'unknown', plug: 'CHECKING', plugClass: '' };
    }
    const pct = isNum(b.percentage) ? clamp(b.percentage, 0, 100) : null;
    let level = 'ok';
    if ((pct !== null && pct <= 10) || (b.voltage <= 4.35 && b.voltage < 3.35)) level = 'critical';
    else if ((pct !== null && pct <= 20) || (b.voltage <= 4.35 && b.voltage < 3.50)) level = 'warn';
    const ps = b.powerState || 'UNKNOWN';
    let plug = 'TREND…', plugClass = '';
    if (ps === 'CHARGING') { plug = 'AC: CHARGING'; plugClass = 'plug-state-ac'; }
    else if (ps === 'FULL') { plug = 'AC: FULL'; plugClass = 'plug-state-ac'; }
    else if (ps === 'STEADY') { plug = 'AC: LIKELY'; plugClass = 'plug-state-ac'; }
    else if (ps === 'DISCHARGING') { plug = 'ON BATTERY'; plugClass = 'plug-state-bat'; }
    return {
      pct,
      text: pct !== null ? `${pct.toFixed(0)}%` : '--%',
      voltText: `${b.voltage.toFixed(2)} V`,
      level, plug, plugClass, powerState: ps
    };
  }

  function flushZeroOnBackground() {
    const active = currentDriveVector();
    const hadMotion = active.src !== null || drive.zeroFramesLeft > 0 || Math.abs(drive.lastCmd.x) > 0.001 || Math.abs(drive.lastCmd.y) > 0.001;
    releaseAllDriveInputs();
    drive.zeroFramesLeft = 0;
    drive.lastCmd = { x: 0, y: 0 };
    if (hadMotion) {
      wsSend({ type: 'joystick', x: 0, y: 0, keepalive: true, deadman: false });
    }
  }

  // ────────────────────────────────────────────────────────────
  // Boot
  // ────────────────────────────────────────────────────────────
  let started = false;
  function start() {
    if (started) return;
    started = true;
    connect();
    startPolling();
    setInterval(driveTick, 1000 / JOY_SEND_HZ);
    if (global.document) {
      global.document.addEventListener('visibilitychange', () => {
        if (global.document.hidden) flushZeroOnBackground();
      });
    }
    global.addEventListener && global.addEventListener('pagehide', () => flushZeroOnBackground());
    global.addEventListener && global.addEventListener('pageshow', (ev) => {
      releaseAllDriveInputs();
      if (ev && ev.persisted && (!ws || ws.readyState > 1)) connect();
    });
    global.addEventListener && global.addEventListener('blur', () => clearDriveInput('keyboard'));
  }

  global.RoverHUD = {
    TOKEN_KEY, JOY_SEND_HZ, LIDAR_MIN_RANGE_M,
    FRONT_STOP_MM, FRONT_CRAWL_MM, FRONT_CRAWL_LINEAR_MPS, FRONT_CRAWL_ANGULAR_RADPS, FRONT_ZONE_STALE_MS,
    TOF_MOUNTS,
    state, on, emit, start, api,
    getToken, setToken, wsSend,
    getPose, quatToEuler, normAngle, toPath, lidarPointsToRobotFrame, tofToRobotFrame, tofDisplayToSource, evaluateFrontZone, evaluateRadarDriveIndicator,
    setThrottle, getThrottle, setDriveInput, clearDriveInput, releaseAllDriveInputs, estimateCommandedVelocity,
    emergencyStop, arm, disarm,
    planPath, dispatchGoal, cancelNav, initHome, homeGoal, relativeGoal, parseMissionScript, previewMission,
    createMapView, worldToCanvas, canvasToWorld, drawMap, updateTrail, drawRadar,
    tempColor, formatMinutes, batteryView,
    _driveTick: driveTick, _handleMessage: handleMessage, _connect: connect
  };
})(typeof window !== 'undefined' ? window : globalThis);
