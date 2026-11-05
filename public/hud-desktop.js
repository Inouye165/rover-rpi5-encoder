/*
 * Rover 1 HUD Cockpit — desktop layout controller (hud.html)
 * Binds the Rover 2-style layout to RoverHUD core. No motion is ever started without an
 * explicit operator click (and a confirm() dialog for autonomous goals).
 */
(function () {
  'use strict';
  const H = window.RoverHUD;
  const $ = (id) => document.getElementById(id);
  const $$ = (sel) => Array.from(document.querySelectorAll(sel));
  const isNum = (v) => typeof v === 'number' && isFinite(v);
  const setText = (id, txt) => { const e = $(id); if (e && e.textContent !== txt) e.textContent = txt; };
  function setStat(id, txt, cls) {
    const e = $(id);
    if (!e) return;
    e.textContent = txt;
    e.classList.remove('ok', 'warn', 'bad', 'dim');
    if (cls) e.classList.add(cls);
  }

  // ── Toasts ──────────────────────────────────────────────────
  function toast(text, level) {
    const stack = $('toast-stack');
    if (!stack) return;
    const el = document.createElement('div');
    el.className = `hud-toast ${level || ''}`;
    el.textContent = text;
    stack.appendChild(el);
    while (stack.children.length > 4) stack.removeChild(stack.firstChild);
    setTimeout(() => el.remove(), level === 'error' ? 6000 : 3500);
  }
  H.on('toast', (t) => toast(t.text, t.level));

  // ── View modes ─────────────────────────────────────────────
  let viewMode = 'drive';
  function setViewMode(mode) {
    viewMode = mode;
    const stage = $('center-stage');
    stage.classList.remove('mode-drive', 'mode-map', 'mode-split');
    stage.classList.add(`mode-${mode}`);
    ['drive', 'map', 'split'].forEach(m => $(`tab-view-${m}`).classList.toggle('active', m === mode));
    [30, 200, 420].forEach(ms => setTimeout(resizeMapCanvas, ms));
  }
  $$('[data-view]').forEach(b => b.addEventListener('click', () => setViewMode(b.dataset.view)));

  // ── E-STOP everywhere ──────────────────────────────────────
  function estop(src) {
    keys.clear();
    H.emergencyStop(src || 'E-STOP');
    toast('🛑 E-STOP sent — drivetrain disarmed, Nav2 cancelled', 'error');
  }
  $('btn-top-estop').addEventListener('click', () => estop('TOP'));
  $$('.js-estop').forEach(b => b.addEventListener('click', () => estop('BUTTON')));

  // ── Keyboard ───────────────────────────────────────────────
  const keys = new Set();
  function isTyping(e) {
    const t = e.target;
    return t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable);
  }
  function updateKeyboardDrive() {
    const f = keys.has('w') || keys.has('arrowup');
    const b = keys.has('s') || keys.has('arrowdown');
    const l = keys.has('a') || keys.has('arrowleft');
    const r = keys.has('d') || keys.has('arrowright');
    const y = (f ? 1 : 0) - (b ? 1 : 0);
    const x = (r ? 1 : 0) - (l ? 1 : 0);
    if (x === 0 && y === 0) H.clearDriveInput('keyboard'); else H.setDriveInput('keyboard', x, y);
  }
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { e.preventDefault(); estop('ESC'); return; }
    if (isTyping(e)) return;
    if (e.key === ' ') { e.preventDefault(); estop('SPACE'); return; }
    const k = e.key.toLowerCase();
    if (['1', '2', '3'].includes(k)) { setViewMode({ 1: 'drive', 2: 'map', 3: 'split' }[k]); return; }
    if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) {
      e.preventDefault();
      keys.add(k);
      updateKeyboardDrive();
    }
  });
  window.addEventListener('keyup', (e) => {
    keys.delete(e.key.toLowerCase());
    updateKeyboardDrive();
  });
  window.addEventListener('blur', () => { keys.clear(); updateKeyboardDrive(); });

  // ── Throttle ───────────────────────────────────────────────
  const slider = $('throttle-slider');
  function applyThrottle() {
    H.setThrottle(slider.value / 100);
    setText('disp-throttle', `${slider.value}%`);
  }
  slider.addEventListener('input', applyThrottle);
  applyThrottle();

  // ── Operator token + arm ───────────────────────────────────
  function refreshAuthUI() {
    const s = H.state;
    const has = !!H.getToken();
    const badge = $('auth-badge');
    badge.classList.remove('auth-ok', 'auth-bad');
    if (s.authOk) { badge.textContent = '🔑 OPERATOR'; badge.classList.add('auth-ok'); }
    else if (has) { badge.textContent = s.authError ? '🔑 BAD TOKEN' : '🔑 AUTH…'; badge.classList.add('auth-bad'); }
    else { badge.textContent = '🔑 NO TOKEN'; badge.classList.add('auth-bad'); }
    setText('token-status', s.authOk ? 'Token: accepted by rover (drive enabled)' :
      (has ? (s.authError ? `Token rejected: ${s.authError}` : 'Token stored — waiting for rover…') : 'Token: not set (view-only)'));
  }
  H.on('auth', refreshAuthUI);
  $('auth-badge').addEventListener('click', () => { const i = $('token-input'); i.scrollIntoView({ block: 'center' }); i.focus(); });
  $('btn-token-save').addEventListener('click', () => {
    const v = $('token-input').value.trim();
    if (!v) return;
    H.setToken(v);
    $('token-input').value = '';
    toast('Operator token saved on this browser', 'ok');
  });
  $('token-input').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('btn-token-save').click(); });
  $('btn-token-clear').addEventListener('click', () => { H.setToken(''); toast('Operator token cleared', 'warn'); });

  const armBtn = $('btn-arm-toggle');
  let armBusy = false;
  function refreshArmUI() {
    const armed = !!(H.state.drive && H.state.drive.armed);
    armBtn.classList.toggle('armed', armed);
    armBtn.textContent = armed ? '🔓 ARMED' : '🔒 ARM';
    armBtn.disabled = armBusy;
  }
  armBtn.addEventListener('click', async () => {
    if (armBusy) return;
    armBusy = true; refreshArmUI();
    try {
      if (H.state.drive && H.state.drive.armed) {
        await H.disarm();
        toast('Drivetrain disarmed', 'warn');
      } else {
        const r = await H.arm();
        if (r.ok) toast('Drivetrain ARMED — WASD / gamepad (hold LB) to drive', 'ok');
        else toast(`Arm rejected: ${r.data.error || r.status}`, 'error');
      }
    } finally {
      armBusy = false; refreshArmUI();
    }
  });
  H.on('drive', refreshArmUI);

  let hintTimer = null;
  H.on('hint', (text) => {
    const e = $('drive-hint');
    e.textContent = `⚠️ ${text}`;
    e.style.display = 'block';
    clearTimeout(hintTimer);
    hintTimer = setTimeout(() => { e.style.display = 'none'; }, 1800);
  });

  // ── Gamepad ────────────────────────────────────────────────
  const GP_DEADBAND = 0.12;
  const ESTOP_BUTTONS = [1, 2, 3, 8, 10, 11];
  let gpPrevEstop = false;
  function shape(v) {
    const a = Math.abs(v);
    if (a < GP_DEADBAND) return 0;
    return Math.sign(v) * (a - GP_DEADBAND) / (1 - GP_DEADBAND);
  }
  function getPad() {
    if (!navigator.getGamepads) return null;
    const pads = navigator.getGamepads();
    for (let i = 0; i < pads.length; i++) if (pads[i] && pads[i].connected) return pads[i];
    return null;
  }
  function pollGamepad() {
    const gp = getPad();
    const badge = $('gamepad-badge');
    if (!gp) {
      setText('gamepad-status-text', 'NO GAMEPAD');
      badge.style.color = '#5f7a91';
      $('gp-conn-dot').style.background = '#5f7a91';
      setText('gp-name', 'No gamepad detected');
      $('gp-wake-hint').style.display = 'block';
      H.clearDriveInput('gamepad');
      return;
    }
    setText('gamepad-status-text', 'GAMEPAD OK');
    badge.style.color = '#00ffcc';
    $('gp-conn-dot').style.background = '#00ffcc';
    setText('gp-name', gp.id);
    $('gp-wake-hint').style.display = 'none';

    const pressed = (i) => !!(gp.buttons[i] && gp.buttons[i].pressed);
    const estopNow = ESTOP_BUTTONS.some(pressed);
    if (estopNow && !gpPrevEstop) estop('GAMEPAD');
    gpPrevEstop = estopNow;

    const deadman = pressed(4) || (gp.buttons[6] && gp.buttons[6].value > 0.5);
    const turbo = pressed(5);
    const y = -shape(gp.axes[1] || 0);
    const rx = shape(gp.axes[2] || 0);
    const x = Math.abs(rx) > 0 ? rx : shape(gp.axes[0] || 0);

    const dm = $('gp-deadman-indicator');
    dm.textContent = deadman ? 'LB/LT: HELD' : 'LB/LT: RELEASED';
    dm.style.borderColor = deadman ? '#00ff88' : '#ff2a4b';
    dm.style.color = deadman ? '#00ff88' : '#ff4765';
    const tb = $('gp-turbo-indicator');
    tb.textContent = turbo ? 'RB: TURBO ON' : 'RB: TURBO OFF';
    tb.style.borderColor = turbo ? '#ffaa00' : '#203146';
    tb.style.color = turbo ? '#ffaa00' : '#5f7a91';

    const moving = Math.abs(x) > 0 || Math.abs(y) > 0;
    $('gp-deadman-alert').style.display = (moving && !deadman && !estopNow) ? 'block' : 'none';
    if (deadman && !estopNow && moving) {
      H.setDriveInput('gamepad', x, y, turbo ? 1.5 : 1);
    } else {
      H.clearDriveInput('gamepad');
    }
  }
  setInterval(pollGamepad, 50);
  window.addEventListener('gamepadconnected', (e) => toast(`🎮 Controller connected: ${e.gamepad.id}`, 'ok'));
  window.addEventListener('gamepaddisconnected', () => { H.clearDriveInput('gamepad'); toast('🎮 Controller disconnected', 'warn'); });
  if (!window.isSecureContext && location.hostname !== 'localhost' && location.hostname !== '127.0.0.1') {
    $('gp-security-alert').style.display = 'block';
    setText('gp-origin', location.origin);
  }

  H.on('drive_cmd', (c) => {
    $('gp-lin-bar').style.width = `${Math.round(Math.abs(c.y) * 100)}%`;
    $('gp-ang-bar').style.width = `${Math.round(Math.abs(c.x) * 100)}%`;
  });

  // ── Camera ─────────────────────────────────────────────────
  const cam = $('camera-feed');
  let camOn = true;
  let camFlip = localStorage.getItem('hud_cam_flip') === '1';
  let camLoaded = false;
  function applyFlip() { cam.style.transform = camFlip ? 'rotate(180deg)' : 'none'; }
  function startCamera() {
    camLoaded = false;
    cam.onload = () => { camLoaded = true; cam.style.opacity = '1'; $('feed-standby').style.display = 'none'; refreshChecklist(); };
    cam.onerror = () => { camLoaded = false; cam.style.opacity = '0'; $('feed-standby').style.display = 'flex'; setText('feed-subtext', 'Camera stream error — retrying…'); refreshChecklist(); setTimeout(() => { if (camOn) startCamera(); }, 5000); };
    cam.src = `/api/camera?t=${Date.now()}`;
    setText('feed-subtext', 'USB CAMERA · /api/camera (MJPEG)');
    $('btn-camera-toggle').textContent = '⏸ CAMERA';
  }
  function stopCamera() {
    cam.onload = cam.onerror = null;
    cam.src = '';
    cam.removeAttribute('src');
    camLoaded = false;
    cam.style.opacity = '0';
    $('feed-standby').style.display = 'flex';
    setText('feed-subtext', 'Camera paused (click ▶ CAMERA)');
    $('btn-camera-toggle').textContent = '▶ CAMERA';
    refreshChecklist();
  }
  $('btn-camera-toggle').addEventListener('click', () => { camOn = !camOn; if (camOn) startCamera(); else stopCamera(); });
  $('btn-flip').addEventListener('click', () => { camFlip = !camFlip; localStorage.setItem('hud_cam_flip', camFlip ? '1' : '0'); applyFlip(); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { if (camOn) stopCamera(); }
    else if (camOn) startCamera();
  });
  window.addEventListener('pageshow', (ev) => {
    if (ev && ev.persisted && camOn) startCamera();
  });
  let hadWsConn = false;
  H.on('connection', (s) => {
    if (s.wsConnected) {
      if (hadWsConn && camOn && !camLoaded) startCamera();
      hadWsConn = true;
    }
  });
  applyFlip();
  startCamera();

  // ── Radar ──────────────────────────────────────────────────
  const radarCanvas = $('lidar-canvas');
  const radarCtx = radarCanvas.getContext('2d');
  function redrawRadar() {
    const rBox = radarCanvas.getBoundingClientRect && radarCanvas.getBoundingClientRect();
    const rSize = (rBox && rBox.width > 80) ? Math.round(rBox.width) : 260;
    if (radarCanvas.width !== rSize || radarCanvas.height !== rSize) {
      radarCanvas.width = rSize;
      radarCanvas.height = rSize;
    }
    const res = H.drawRadar(radarCtx, rSize, {
      maxRange: parseFloat($('sel-radar-range').value),
      color: $('sel-radar-color').value,
      pointSize: parseFloat($('sel-radar-ptsize').value)
    });
    const sc = H.state.scan;
    const ls = H.state.lidarStatus;
    const badge = $('stat-radar-badge');
    if (res && res.live) {
      badge.textContent = '360° LIDAR LIVE'; badge.style.color = '#00ff88';
      setText('hud-lidar-stat', '360° LIDAR: LIVE');
      setText('hud-nearest-stat', res.nearest ? `NEAREST: ${res.nearest.d.toFixed(2)} m` : 'NEAREST: --');
      setText('hud-lidar-min', res.nearest ? `MIN: ${(res.nearest.d * 100).toFixed(0)} cm` : 'MIN: --');
    } else {
      const disconnected = ls && ls.connected === false;
      badge.textContent = disconnected ? 'DISCONNECTED' : 'STALE';
      badge.style.color = disconnected ? '#ff4765' : '#ffaa00';
      setText('hud-lidar-stat', disconnected ? '360° LIDAR: OFFLINE' : '360° LIDAR: WAITING');
      setText('hud-nearest-stat', 'NEAREST: --');
      setText('hud-lidar-min', 'MIN: --');
    }
    setText('hud-scan-age', sc.t ? `AGE: ${((Date.now() - sc.t) / 1000).toFixed(1)}s` : 'AGE: --');
    setText('disp-lidar-pts', `${sc.points.length} pts`);
    setText('disp-lidar-hz', isNum(sc.scanHz) && res && res.live ? `${sc.scanHz.toFixed(1)} Hz` : '-- Hz');
  }
  ['sel-radar-range', 'sel-radar-color', 'sel-radar-ptsize'].forEach(id => $(id).addEventListener('change', redrawRadar));
  setInterval(redrawRadar, 150);
  H.on('lidar_status', (ls) => {
    setText('disp-lidar-dev', ls.model || 'RPLiDAR C1');
    setText('disp-lidar-reconn', isNum(ls.reconnectCount) ? String(ls.reconnectCount) : '--');
    const err = $('disp-lidar-error');
    if (ls.connected === false && ls.lastError) {
      err.style.display = 'block';
      err.textContent = `⚠️ ${String(ls.lastError).slice(0, 160)}`;
    } else {
      err.style.display = 'none';
    }
    refreshChecklist();
  });

  // ── Battery / UPS ──────────────────────────────────────────
  function refreshBattery() {
    const b = H.state.battery;
    const v = H.batteryView(b);
    const badge = $('battery-badge');
    badge.textContent = `🔋 RPi UPS: ${v.text} (${v.voltText})`;
    badge.classList.remove('batt-warn-yellow', 'batt-crit-flash');
    if (v.level === 'warn') badge.classList.add('batt-warn-yellow');
    if (v.level === 'critical') badge.classList.add('batt-crit-flash');

    const plug = $('ups-plug-badge');
    plug.classList.remove('plug-state-ac', 'plug-state-bat');
    plug.textContent = `🔌 ${v.plug}`;
    if (v.plugClass) plug.classList.add(v.plugClass);

    const banner = $('ups-plug-banner');
    banner.classList.remove('plug-state-ac', 'plug-state-bat');
    banner.classList.add(v.plugClass || 'plug-state-bat');
    const bannerText = {
      CHARGING: '🔌 AC POWER — CHARGING',
      FULL: '🔌 AC POWER — FULL',
      STEADY: '🔌 SOC STEADY — AC LIKELY CONNECTED',
      DISCHARGING: '🔋 ON BATTERY — DISCHARGING'
    }[v.powerState] || '🔌 LEARNING CHARGE TREND...';
    banner.textContent = bannerText;

    setText('ups-soc', v.pct !== null ? `${v.pct.toFixed(1)} %` : '-- %');
    setText('ups-voltage', v.voltText);
    setText('ups-state', b && b.source === 'geekworm_x1206' ? (v.powerState || 'UNKNOWN') : (b ? `${b.source || 'unknown'} (no SOC)` : 'Standby'));
    setText('ups-rate', b && isNum(b.ratePctPerHr) ? `${b.ratePctPerHr >= 0 ? '+' : ''}${b.ratePctPerHr.toFixed(1)} %/hr` : '-- %/hr');
    let eta = 'Calibrating';
    if (b && isNum(b.minutesRemaining)) {
      eta = v.powerState === 'CHARGING' ? `${H.formatMinutes(b.minutesRemaining)} to full` : `${H.formatMinutes(b.minutesRemaining)} left`;
    } else if (v.powerState === 'FULL') eta = 'Full';
    setText('ups-time-left', eta);
    setText('ups-window', b && isNum(b.trendWindowMin) ? `${b.trendWindowMin.toFixed(1)} min / ${b.trendSamples || 0} pts` : '--');

    const bar = $('ups-bar');
    bar.style.width = `${v.pct !== null ? Math.max(0, Math.min(100, v.pct)) : 0}%`;
    bar.style.background = v.level === 'critical' ? '#ff2a4b' : (v.level === 'warn' ? '#ffcc00' : '#00ffcc');

    const hb = $('ups-health-badge');
    const diag = $('ups-health-diag');
    if (!b || !isNum(b.voltage)) {
      hb.textContent = 'NO DATA'; hb.style.color = '#ffaa00';
      diag.textContent = 'Waiting for X1206 fuel-gauge telemetry...';
    } else if (v.level === 'critical') {
      hb.textContent = 'CRITICAL'; hb.style.color = '#ff4765';
      diag.textContent = `Cell at ${v.voltText}. Connect AC power now — Pi will brown out soon.`;
      diag.style.color = '#ff99aa';
    } else if (v.level === 'warn') {
      hb.textContent = 'LOW'; hb.style.color = '#ffcc00';
      diag.textContent = `Cell at ${v.voltText}. Plan to charge soon.`;
      diag.style.color = '#ffe066';
    } else {
      hb.textContent = 'HEALTHY'; hb.style.color = '#00ff88';
      diag.textContent = v.powerState && v.powerState !== 'UNKNOWN'
        ? `Cell ${v.voltText}. AC state inferred from SOC trend (X1206 has no AC-detect line wired).`
        : `Cell ${v.voltText}. Collecting ~5 min of SOC history to infer AC / battery state.`;
      diag.style.color = '#00ffcc';
    }
    refreshChecklist();
  }
  H.on('battery', refreshBattery);

  // ── System (RPi temp / power) ──────────────────────────────
  H.on('system', (s) => {
    const t = s && s.cpuTempC;
    const el = $('rpi-temp');
    el.textContent = isNum(t) ? `🌡️ ${t.toFixed(1)}°C` : '🌡️ --°C';
    el.style.color = H.tempColor(t);
    refreshChecklist();
  });

  // ── Connection badge ───────────────────────────────────────
  function refreshConn() {
    const s = H.state;
    const dot = $('conn-dot');
    dot.classList.toggle('active', s.wsConnected);
    if (!s.wsConnected) setText('conn-text', 'OFFLINE');
    else if (s.serial !== 'connected') setText('conn-text', 'NO ESP32');
    else setText('conn-text', 'ONLINE');
    refreshChecklist();
  }
  H.on('connection', refreshConn);
  H.on('serial', refreshConn);

  // ── IMU ────────────────────────────────────────────────────
  H.on('imu', (imu) => {
    const e = H.state.imuEuler;
    const valid = imu.hardwareInitialized && imu.rotVecValid && e;
    setText('imu-yaw', valid ? `${e.yawDeg.toFixed(1)}°` : '--°');
    setText('imu-pitch', valid ? `${e.pitchDeg.toFixed(1)}°` : '--°');
    setText('imu-roll', valid ? `${e.rollDeg.toFixed(1)}°` : '--°');
    const b = $('imu-badge');
    if (!imu.hardwareInitialized) { b.textContent = imu.inResetRecovery ? 'RESET RECOVERY' : 'NOT INITIALIZED'; b.style.color = '#ff4765'; }
    else if (!imu.rotVecValid) { b.textContent = 'NO ROT VEC'; b.style.color = '#ffaa00'; }
    else { b.textContent = `LIVE · CAL ${imu.calibrationStatus}`; b.style.color = '#00ff88'; }
    refreshChecklist();
  });

  // ── Drivetrain ─────────────────────────────────────────────
  H.on('odom', (o) => {
    setText('disp-lin', `${(o.v || 0).toFixed(2)} m/s`);
    setText('disp-ang', `${(o.w || 0).toFixed(2)} rad/s`);
  });
  H.on('drive', (d) => {
    setText('disp-cmd', `${(d.limLinear || 0).toFixed(2)} m/s / ${(d.limAngular || 0).toFixed(2)} r/s`);
    const b = $('drivetrain-badge');
    b.textContent = d.armed ? 'ARMED' : 'DISARMED';
    b.style.color = d.armed ? '#00ff88' : '#ff4765';
    refreshNavCard();
  });
  H.on('firmware', (f) => { setText('disp-fw', f.version || '--'); refreshChecklist(); });

  // ── Map ────────────────────────────────────────────────────
  const view = H.createMapView({ scale: 95, miniScale: 48 });
  const mapCanvas = $('nav-map-canvas');
  const mapCtx = mapCanvas.getContext('2d');
  let routeMetrics = null;
  function resizeMapCanvas() {
    const rect = $('autonomy-map-panel').getBoundingClientRect();
    if (rect.width > 20 && rect.height > 20) {
      mapCanvas.width = Math.round(rect.width);
      mapCanvas.height = Math.round(rect.height);
    }
  }
  window.addEventListener('resize', resizeMapCanvas);
  if (window.ResizeObserver) new ResizeObserver(resizeMapCanvas).observe($('autonomy-map-panel'));
  setTimeout(resizeMapCanvas, 100);

  function setFollow(on) {
    view.follow = on;
    if (on) view.pan = { x: 0, y: 0 };
    $('btn-map-follow').classList.toggle('active', on);
  }
  $('btn-map-follow').addEventListener('click', () => setFollow(!view.follow));
  $('btn-map-zoom-in').addEventListener('click', () => { view.scale = Math.min(280, view.scale * 1.25); });
  $('btn-map-zoom-out').addEventListener('click', () => { view.scale = Math.max(20, view.scale * 0.8); });
  $('btn-map-center').addEventListener('click', () => { view.scale = 95; setFollow(true); });
  $('btn-map-clear').addEventListener('click', () => { view.goal = null; view.plannedPath = []; routeMetrics = null; setNavStatus('Goal pin cleared.'); refreshNavCard(); });

  let dragGoal = false, panning = false, dragStart = { u: 0, v: 0 }, panStart = { x: 0, y: 0 };
  mapCanvas.addEventListener('contextmenu', (e) => e.preventDefault());
  mapCanvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    view.scale = Math.max(20, Math.min(280, view.scale * (e.deltaY < 0 ? 1.15 : 0.87)));
  }, { passive: false });
  mapCanvas.addEventListener('pointerdown', (e) => {
    if (viewMode === 'drive') { setViewMode('map'); return; }
    const r = mapCanvas.getBoundingClientRect();
    const u = e.clientX - r.left, v = e.clientY - r.top;
    dragStart = { u, v };
    if (e.button === 2 || e.button === 1 || e.shiftKey) {
      panning = true;
      const p = H.getPose();
      const c = view.follow ? { x: p.x + view.pan.x, y: p.y + view.pan.y } : view.pan;
      setFollow(false);
      view.pan = { x: c.x, y: c.y };
      panStart = { x: c.x, y: c.y };
      mapCanvas.setPointerCapture(e.pointerId);
      return;
    }
    const w = H.canvasToWorld(view, u, v, mapCanvas.width, mapCanvas.height, view.scale);
    const p = H.getPose();
    const yaw = Math.atan2(w.y - p.y, w.x - p.x);
    view.goal = { x: w.x, y: w.y, yawRad: yaw, label: `Map Pin (${w.x.toFixed(2)}m, ${w.y.toFixed(2)}m)` };
    view.plannedPath = [];
    routeMetrics = null;
    dragGoal = true;
    mapCanvas.setPointerCapture(e.pointerId);
    setNavStatus(`🎯 Goal pin (${w.x.toFixed(2)}m, ${w.y.toFixed(2)}m) — drag to aim arrival heading`);
    refreshNavCard();
  });
  mapCanvas.addEventListener('pointermove', (e) => {
    const r = mapCanvas.getBoundingClientRect();
    const u = e.clientX - r.left, v = e.clientY - r.top;
    if (panning) {
      view.pan.x = panStart.x - (u - dragStart.u) / view.scale;
      view.pan.y = panStart.y + (v - dragStart.v) / view.scale;
      return;
    }
    if (dragGoal && view.goal && Math.hypot(u - dragStart.u, v - dragStart.v) > 14) {
      const w = H.canvasToWorld(view, u, v, mapCanvas.width, mapCanvas.height, view.scale);
      view.goal.yawRad = Math.atan2(w.y - view.goal.y, w.x - view.goal.x);
      setNavStatus(`🎯 Arrival heading ${(view.goal.yawRad * 180 / Math.PI).toFixed(0)}° at (${view.goal.x.toFixed(2)}m, ${view.goal.y.toFixed(2)}m)`);
    }
  });
  const endPointer = () => {
    if (panning) { panning = false; return; }
    if (dragGoal) { dragGoal = false; if (view.goal) previewGoal(); }
  };
  mapCanvas.addEventListener('pointerup', endPointer);
  mapCanvas.addEventListener('pointercancel', endPointer);

  function drawLoop() {
    H.updateTrail(view);
    H.drawMap(mapCtx, view, mapCanvas.width, mapCanvas.height, viewMode === 'drive');
  }
  setInterval(drawLoop, 50);

  // ── Navigation actions ─────────────────────────────────────
  function setNavStatus(text) { setText('nav-card-status', text); setText('nav-hud-banner', text.length > 90 ? text.slice(0, 88) + '…' : text); }
  let planning = false;
  async function previewGoal(goal) {
    goal = goal || view.goal;
    if (!goal) { toast('Click the map (or a preset) to choose a goal first', 'warn'); return; }
    if (planning) return;
    view.goal = goal;
    planning = true;
    setNavStatus('⏳ Planning with Nav2 (rover stays disarmed)…');
    try {
      const r = await H.planPath(goal);
      if (r.ok) {
        view.plannedPath = r.path;
        routeMetrics = { len: Number(r.data.cumulative_length_m || 0), straight: Number(r.data.straight_distance_m || 0) };
        setNavStatus(`✅ Path preview: ${routeMetrics.len.toFixed(2)} m along ${r.path.length} waypoints. Rover has NOT moved.`);
      } else {
        view.plannedPath = [];
        routeMetrics = null;
        setNavStatus(`❌ Planner: ${r.data.error || 'planning failed'}`);
      }
    } finally {
      planning = false;
      refreshNavCard();
    }
  }
  async function dispatchGoal(goal) {
    goal = goal || view.goal;
    if (!goal) { toast('Choose a goal on the map first', 'warn'); return; }
    if (!H.getToken()) { toast('Operator token required to dispatch', 'error'); return; }
    const ok = window.confirm(`Nav2 will ARM Rover 1 and drive autonomously to\n${goal.label || ''}\n(x=${goal.x.toFixed(2)} m, y=${goal.y.toFixed(2)} m, yaw=${(goal.yawRad * 180 / Math.PI).toFixed(0)}°).\n\nArea clear and operator ready?`);
    if (!ok) return;
    view.goal = goal;
    setNavStatus('🚀 Dispatching goal to Nav2…');
    const r = await H.dispatchGoal(goal);
    if (r.ok) { setNavStatus(`🚀 Goal accepted — executing. E-STOP (Space) at any time.`); toast('Nav2 goal dispatched', 'ok'); }
    else { setNavStatus(`❌ Dispatch rejected: ${r.data.error || r.status}`); toast(`Dispatch rejected: ${r.data.error || r.status}`, 'error'); }
  }
  function returnHome() {
    const g = H.homeGoal();
    if (!g) { toast('HOME pose unavailable from nav bridge', 'error'); return; }
    dispatchGoal(g);
  }
  async function initHome() {
    if (!window.confirm('Set AMCL pose to the saved HOME pad?\nOnly do this when Rover 1 is physically sitting on HOME. (No motion.)')) return;
    const r = await H.initHome();
    if (r.ok) toast('📍 AMCL initialised at HOME', 'ok'); else toast(`Localize failed: ${r.data.error || r.status}`, 'error');
  }
  $$('.js-preview').forEach(b => b.addEventListener('click', () => previewGoal()));
  $$('.js-go').forEach(b => b.addEventListener('click', () => dispatchGoal()));
  $$('.js-return-home').forEach(b => b.addEventListener('click', returnHome));
  $('btn-top-home').addEventListener('click', returnHome);
  $$('.js-init-home').forEach(b => b.addEventListener('click', initHome));
  $('btn-top-init-home').addEventListener('click', initHome);
  $$('.preset-chip').forEach(b => b.addEventListener('click', () => {
    const g = H.relativeGoal(parseFloat(b.dataset.dist), parseFloat(b.dataset.ang));
    previewGoal(g);
  }));
  $('btn-mission-preview').addEventListener('click', async () => {
    setNavStatus('⏳ Planning mission legs…');
    try {
      const r = await H.previewMission($('mission-script-input').value);
      view.plannedPath = r.path;
      const last = r.legs[r.legs.length - 1];
      view.goal = last ? { x: last.target.x, y: last.target.y, yawRad: last.target.yaw, label: `Mission end (${r.legs.length} legs)` } : null;
      routeMetrics = r.ok ? { len: r.lengthM, straight: 0 } : null;
      setNavStatus(r.ok ? `✅ Mission preview: ${r.legs.length} legs, ${r.lengthM.toFixed(2)} m. Preview only — nothing dispatched.` : `❌ ${r.error}`);
    } catch (err) {
      setNavStatus(`❌ ${err.message}`);
    }
    refreshNavCard();
  });

  function refreshNavCard() {
    const p = H.getPose();
    setText('nav-pose-xy', p.frame === 'none' ? '--' : `${p.x.toFixed(2)}, ${p.y.toFixed(2)} m${p.frame === 'odom' ? ' (odom)' : ''}`);
    setText('nav-pose-yaw', p.frame === 'none' ? '--' : `${p.yawDeg.toFixed(1)}°`);
    const h = H.homeGoal();
    setText('nav-home-xy', h ? `${h.x.toFixed(2)}, ${h.y.toFixed(2)} m` : '--');
    const nav = H.state.nav;
    const navStatus = (nav && nav.status) || 'UNKNOWN';
    const executing = /EXEC|ACTIVE|NAVIGAT/i.test(navStatus);
    if (executing && isNum(nav.distance_remaining_m)) setText('nav-rem-dist', `${nav.distance_remaining_m.toFixed(2)} m`);
    else if (view.goal) setText('nav-rem-dist', `${Math.hypot(view.goal.x - p.x, view.goal.y - p.y).toFixed(2)} m`);
    else setText('nav-rem-dist', '-- m');
    const armed = !!(H.state.drive && H.state.drive.armed);
    const badge = $('nav-card-badge');
    badge.textContent = `${navStatus} (${armed ? 'ARMED' : 'DISARMED'})`;
    badge.style.color = executing ? '#00ff88' : (/FAIL|ABORT|REJECT/i.test(navStatus) ? '#ff4765' : '#00ffcc');
    $('nav-state-dot').classList.toggle('active', executing || (H.state.localization && H.state.localization.localized));
    setText('map-dock-target', view.goal ? view.goal.label || 'Goal' : 'Click map to set goal');
    setText('map-dock-metrics', routeMetrics ? `${routeMetrics.len.toFixed(2)} m` : '-- m');
  }
  ['nav', 'localization', 'odom', 'home'].forEach(evt => H.on(evt, refreshNavCard));

  // ── Hardware checklist ─────────────────────────────────────
  function refreshChecklist() {
    const s = H.state;
    const b = s.battery;
    if (b && b.source === 'geekworm_x1206' && Date.now() - (b.receivedAt || 0) < 20000) setStat('stat-ups', 'ONLINE', 'ok');
    else if (b) setStat('stat-ups', 'STALE', 'warn');
    else setStat('stat-ups', 'NO DATA', 'warn');

    if (!s.wsConnected) setStat('stat-serial', 'SERVER OFFLINE', 'bad');
    else if (s.serial === 'connected') setStat('stat-serial', s.firmware && s.firmware.version ? `OK · ${s.firmware.version}` : 'CONNECTED', 'ok');
    else setStat('stat-serial', 'DISCONNECTED', 'bad');

    const ls = s.lidarStatus;
    if (ls && ls.connected) setStat('stat-rplidar', `ONLINE · ${isNum(ls.scanHz) ? ls.scanHz.toFixed(1) : '--'} Hz`, 'ok');
    else if (ls) setStat('stat-rplidar', 'DISCONNECTED', 'bad');

    const imu = s.imu;
    if (imu) {
      if (imu.hardwareInitialized && imu.rotVecValid) setStat('stat-imu', 'ONLINE', 'ok');
      else setStat('stat-imu', imu.inResetRecovery ? 'RESET RECOVERY' : 'NOT INITIALIZED', 'bad');
    } else setStat('stat-imu', 'NO DATA', 'warn');

    if (!camOn) setStat('stat-camera', 'PAUSED', 'dim');
    else setStat('stat-camera', camLoaded ? 'STREAMING' : 'NO FRAMES', camLoaded ? 'ok' : 'warn');

    const nav = s.nav;
    if (nav) setStat('stat-nav', nav.action_server_ready ? (nav.ready_to_accept_goals ? 'READY' : 'UP · NOT READY') : 'ACTION SERVER DOWN', nav.ready_to_accept_goals ? 'ok' : 'warn');

    const loc = s.localization;
    if (loc) setStat('stat-loc', loc.localized ? 'LOCALIZED' : (loc.state || 'NOT LOCALIZED'), loc.localized ? 'ok' : 'warn');

    const slam = s.slam;
    if (slam) setStat('stat-slam', slam.state || '--', slam.state === 'RUNNING' || slam.state === 'ACTIVE' ? 'ok' : 'dim');

    const sys = s.system;
    if (sys) {
      const t = sys.cpuTempC;
      const throttled = sys.throttled && sys.throttled !== '0x0';
      const txt = `${isNum(t) ? t.toFixed(0) + '°C' : '--'}${isNum(sys.ext5vV) ? ' · ' + sys.ext5vV.toFixed(2) + 'V' : ''}${throttled ? ' · THROTTLED ' + sys.throttled : ''}`;
      setStat('stat-rpi', txt, throttled || (isNum(t) && t >= 80) ? 'bad' : (isNum(t) && t >= 70 ? 'warn' : 'ok'));
      if (sys.hailo && sys.hailo.present) {
        const isIdle = sys.hailo.runtimeStatus === 'suspended' || sys.hailo.bridgePowerState === 'D3hot';
        setStat('stat-hailo', isIdle ? `IDLE · ${sys.hailo.bridgePowerState === 'D3hot' ? 'D3hot' : 'SUSPENDED'}` : `ACTIVE (${sys.hailo.powerControl})`, isIdle ? 'ok' : 'warn');
      } else {
        setStat('stat-hailo', 'STANDBY', 'dim');
      }
    }
  }

  // ── Boot ───────────────────────────────────────────────────
  refreshAuthUI();
  refreshArmUI();
  refreshBattery();
  refreshNavCard();
  H.start();
  window.addEventListener('beforeunload', () => H.releaseAllDriveInputs());
  window.__hudDesktop = { setViewMode, view, previewGoal, refreshChecklist };
})();
