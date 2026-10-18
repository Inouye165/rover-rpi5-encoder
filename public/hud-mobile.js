/*
 * Rover 1 HUD Cockpit — mobile touch layout controller (hud-mobile.html)
 * Ported from the Rover 2 mobile.html stick layout. Drive goes through server.js
 * (`joystick` WebSocket messages with keepalive), never a raw /cmd_vel publish.
 */
(function () {
  'use strict';
  const H = window.RoverHUD;
  const $ = (id) => document.getElementById(id);
  const isNum = (v) => typeof v === 'number' && isFinite(v);
  const setText = (id, t) => { const e = $(id); if (e && e.textContent !== t) e.textContent = t; };

  // ── Toast ──────────────────────────────────────────────────
  let toastT = null;
  function toast(text, level) {
    const e = $('mobToast');
    e.textContent = text;
    e.className = level || '';
    e.style.display = 'block';
    clearTimeout(toastT);
    toastT = setTimeout(() => { e.style.display = 'none'; }, level === 'error' ? 4500 : 2500);
  }
  H.on('toast', (t) => toast(t.text, t.level));

  // ── View modes ─────────────────────────────────────────────
  let mobMode = 'drive';
  function setMobMode(mode) {
    mobMode = mode;
    const stg = $('stage');
    stg.classList.remove('mode-drive', 'mode-map', 'mode-split');
    stg.classList.add('mode-' + mode);
    $('tabDrive').classList.toggle('active', mode === 'drive');
    $('tabMap').classList.toggle('active', mode === 'map');
    $('tabSplit').classList.toggle('active', mode === 'split');
    [30, 180, 380].forEach(ms => setTimeout(() => { sizeRadar(); sizeMap(); }, ms));
  }
  document.querySelectorAll('[data-mode]').forEach(b => b.addEventListener('click', () => setMobMode(b.dataset.mode)));

  // ── E-STOP ─────────────────────────────────────────────────
  function estop() {
    H.emergencyStop('MOBILE');
    st.lin = st.ang = 0;
    resetKnobs();
    toast('🛑 E-STOP — disarmed, Nav2 cancelled', 'error');
  }
  ['estop', 'ctlEstop', 'btnMobTopEstop', 'btnMobCancel'].forEach(id => {
    $(id).addEventListener('pointerdown', (e) => { e.preventDefault(); estop(); });
  });
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' || (e.key === ' ' && (!e.target || e.target.tagName !== 'INPUT'))) { e.preventDefault(); estop(); }
  });

  // ── Arm / speed ────────────────────────────────────────────
  let armBusy = false;
  function refreshArm() {
    const armed = !!(H.state.drive && H.state.drive.armed);
    $('arm').classList.toggle('on', armed);
    $('arm').textContent = armBusy ? '…' : (armed ? 'ARMED' : 'ARM');
    const hs = $('hState');
    hs.textContent = armed ? 'ARMED' : 'DISARMED';
    hs.style.color = armed ? 'var(--ok)' : 'var(--bad)';
  }
  H.on('drive', refreshArm);
  $('arm').addEventListener('click', async () => {
    if (armBusy) return;
    if (!H.getToken()) { openToken(); return; }
    armBusy = true; refreshArm();
    try {
      if (H.state.drive && H.state.drive.armed) {
        await H.disarm();
      } else {
        const r = await H.arm();
        if (!r.ok) toast(`Arm rejected: ${r.data.error || r.status}`, 'error');
      }
    } finally { armBusy = false; refreshArm(); }
  });
  $('spd').addEventListener('input', (e) => { H.setThrottle(e.target.value / 100); setText('spdV', e.target.value); });
  H.setThrottle($('spd').value / 100);

  let hintT = null;
  H.on('hint', (msg) => {
    const e = $('hState');
    e.textContent = msg.length > 26 ? msg.slice(0, 25) + '…' : msg;
    e.style.color = 'var(--warn)';
    clearTimeout(hintT);
    hintT = setTimeout(refreshArm, 1500);
  });

  // ── Token ──────────────────────────────────────────────────
  function openToken() { $('tokenBar').classList.add('open'); if (mobMode === 'drive') setMobMode('map'); }
  function refreshAuth() {
    const c = $('cAuth');
    c.classList.remove('auth-ok', 'auth-bad');
    if (H.state.authOk) { c.textContent = '🔑 OK'; c.classList.add('auth-ok'); }
    else { c.textContent = H.getToken() ? '🔑 …' : '🔑 SET'; c.classList.add('auth-bad'); }
  }
  H.on('auth', refreshAuth);
  $('cAuth').addEventListener('click', openToken);
  $('btnMobTokenSave').addEventListener('click', () => {
    const v = $('mobTokenInput').value.trim();
    if (!v) return;
    H.setToken(v);
    $('mobTokenInput').value = '';
    $('tokenBar').classList.remove('open');
    toast('Token saved', 'ok');
  });
  $('btnMobTokenClear').addEventListener('click', () => { H.setToken(''); toast('Token forgotten', 'warn'); });
  $('btnMobTokenClose').addEventListener('click', () => $('tokenBar').classList.remove('open'));

  // ── Touch sticks (Rover 2 geometry) ────────────────────────
  const st = { lin: 0, ang: 0 };
  const knobs = [];
  function pushTouch() {
    if (Math.abs(st.lin) < 0.001 && Math.abs(st.ang) < 0.001) H.clearDriveInput('touch');
    else H.setDriveInput('touch', st.ang, st.lin); // x = +right turn, y = +forward
  }
  function makeStick(el, axis) {
    const knob = el.querySelector('.knob');
    knobs.push({ knob, axis });
    let pid = null;
    function upd(e) {
      const r = el.getBoundingClientRect();
      const cx = r.left + r.width / 2, cy = r.top + r.height / 2;
      let dx = (e.clientX - cx) / (r.width / 2), dy = (e.clientY - cy) / (r.height / 2);
      dx = Math.max(-1, Math.min(1, dx)); dy = Math.max(-1, Math.min(1, dy));
      if (axis === 'y') { st.lin = Math.abs(dy) < 0.12 ? 0 : -dy; knob.style.transform = `translateY(calc(-50% + ${dy * (r.height / 2 - r.width * 0.4)}px))`; }
      else { st.ang = Math.abs(dx) < 0.12 ? 0 : dx; knob.style.transform = `translate(${dx * r.width * 0.28}px, 0)`; }
      pushTouch();
    }
    el.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      if (pid !== null) return;
      pid = e.pointerId;
      el.setPointerCapture(pid);
      upd(e);
    });
    el.addEventListener('pointermove', (e) => { if (e.pointerId === pid) { e.preventDefault(); upd(e); } });
    const end = (e) => {
      if (e.pointerId !== pid) return;
      pid = null;
      if (axis === 'y') st.lin = 0; else st.ang = 0;
      resetKnob(knob, axis);
      pushTouch();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
    el.addEventListener('lostpointercapture', end);
  }
  function resetKnob(knob, axis) { knob.style.transform = axis === 'y' ? 'translateY(-50%)' : 'translate(0,0)'; }
  function resetKnobs() { knobs.forEach(k => resetKnob(k.knob, k.axis)); H.clearDriveInput('touch'); }
  makeStick($('stkL'), 'y');
  makeStick($('stkR'), 'x');
  document.addEventListener('visibilitychange', () => { if (document.hidden) { st.lin = st.ang = 0; resetKnobs(); } });

  H.on('drive_cmd', (c) => {
    const est = H.estimateCommandedVelocity(c.x, c.y);
    setText('hSpd', `${est.linear.toFixed(2)} m/s · ${est.angular.toFixed(2)} r/s`);
    setText('readout', `${Math.round(st.lin * 100)}% / ${Math.round(st.ang * 100)}%`);
  });

  // ── Camera ─────────────────────────────────────────────────
  const cam = $('cam');
  let camOn = true;
  let flip = localStorage.getItem('hud_cam_flip') === '1';
  function applyFlip() { cam.style.transform = flip ? 'rotate(180deg)' : 'none'; }
  function startCam() {
    cam.onload = () => { $('camMsg').style.display = 'none'; };
    cam.onerror = () => { $('camMsg').style.display = 'flex'; $('camMsg').textContent = 'Camera error — retrying…'; setTimeout(() => { if (camOn) startCam(); }, 5000); };
    cam.src = `/api/camera?t=${Date.now()}`;
    $('bCam').textContent = '⏸ Cam';
  }
  function stopCam() {
    cam.onload = cam.onerror = null;
    cam.removeAttribute('src');
    $('camMsg').style.display = 'flex';
    $('camMsg').textContent = 'Camera paused';
    $('bCam').textContent = '▶ Cam';
  }
  $('bCam').addEventListener('click', () => { camOn = !camOn; if (camOn) startCam(); else stopCam(); });
  $('bFlip').addEventListener('click', () => { flip = !flip; localStorage.setItem('hud_cam_flip', flip ? '1' : '0'); applyFlip(); });
  $('bFs').addEventListener('click', () => {
    const d = document.documentElement;
    if (document.fullscreenElement) document.exitFullscreen(); else if (d.requestFullscreen) d.requestFullscreen().catch(() => {});
  });
  document.addEventListener('visibilitychange', () => { if (document.hidden) { if (camOn) stopCam(); } else if (camOn) startCam(); });
  applyFlip();
  startCam();

  // ── Radar overlay ──────────────────────────────────────────
  const radar = $('radar');
  const rctx = radar.getContext('2d');
  let radarSize = 140;
  function sizeRadar() {
    const r = radar.getBoundingClientRect();
    radarSize = Math.max(80, Math.round(Math.min(r.width, r.height)));
    radar.width = radarSize; radar.height = radarSize;
  }
  window.addEventListener('resize', sizeRadar);
  setTimeout(sizeRadar, 60);
  setInterval(() => {
    const res = H.drawRadar(rctx, radarSize, { maxRange: 3.0, color: 'range', pointSize: 2 });
    if (res && res.live && res.nearest) setText('hMin', `LIDAR ${res.nearest.d.toFixed(2)} m`);
    else setText('hMin', H.state.lidarStatus && H.state.lidarStatus.connected === false ? 'LIDAR OFFLINE' : 'LIDAR --');
  }, 200);

  // ── Map ────────────────────────────────────────────────────
  const view = H.createMapView({ scale: 80, miniScale: 40 });
  const mc = $('mobMapCanvas');
  const mctx = mc.getContext('2d');
  function sizeMap() {
    const r = $('mapPane').getBoundingClientRect();
    if (r.width > 20 && r.height > 20) { mc.width = Math.round(r.width); mc.height = Math.round(r.height); }
  }
  window.addEventListener('resize', sizeMap);
  if (window.ResizeObserver) new ResizeObserver(sizeMap).observe($('mapPane'));
  setTimeout(sizeMap, 100);
  setInterval(() => { H.updateTrail(view); H.drawMap(mctx, view, mc.width, mc.height, mobMode === 'drive'); }, 66);

  function banner(t) { setText('mobNavBanner', t); }
  let dragging = false, dragStart = null;
  mc.addEventListener('pointerdown', (e) => {
    if (mobMode === 'drive') { setMobMode('map'); return; }
    const r = mc.getBoundingClientRect();
    const u = e.clientX - r.left, v = e.clientY - r.top;
    const w = H.canvasToWorld(view, u, v, mc.width, mc.height, view.scale);
    const p = H.getPose();
    view.goal = { x: w.x, y: w.y, yawRad: Math.atan2(w.y - p.y, w.x - p.x), label: `Pin (${w.x.toFixed(2)}, ${w.y.toFixed(2)})` };
    view.plannedPath = [];
    dragging = true; dragStart = { u, v };
    mc.setPointerCapture(e.pointerId);
    banner(`🎯 Pin ${w.x.toFixed(2)}, ${w.y.toFixed(2)} — drag to aim`);
  });
  mc.addEventListener('pointermove', (e) => {
    if (!dragging || !view.goal) return;
    const r = mc.getBoundingClientRect();
    const u = e.clientX - r.left, v = e.clientY - r.top;
    if (Math.hypot(u - dragStart.u, v - dragStart.v) > 16) {
      const w = H.canvasToWorld(view, u, v, mc.width, mc.height, view.scale);
      view.goal.yawRad = Math.atan2(w.y - view.goal.y, w.x - view.goal.x);
      banner(`🎯 Heading ${(view.goal.yawRad * 180 / Math.PI).toFixed(0)}°`);
    }
  });
  const endDrag = () => { if (dragging) { dragging = false; plan(); } };
  mc.addEventListener('pointerup', endDrag);
  mc.addEventListener('pointercancel', endDrag);

  $('btnMobFollow').addEventListener('click', () => { view.follow = !view.follow; if (view.follow) view.pan = { x: 0, y: 0 }; $('btnMobFollow').classList.toggle('active', view.follow); });
  $('btnMobZoomIn').addEventListener('click', () => { view.scale = Math.min(260, view.scale * 1.25); });
  $('btnMobZoomOut').addEventListener('click', () => { view.scale = Math.max(20, view.scale * 0.8); });
  $('btnMobDrawer').addEventListener('click', () => $('missionDrawer').classList.toggle('open'));
  $('btnCloseDrawer').addEventListener('click', () => $('missionDrawer').classList.remove('open'));

  let planning = false;
  async function plan(goal) {
    goal = goal || view.goal;
    if (!goal || planning) return;
    view.goal = goal;
    planning = true;
    banner('⏳ Planning…');
    try {
      const r = await H.planPath(goal);
      if (r.ok) { view.plannedPath = r.path; banner(`✅ ${Number(r.data.cumulative_length_m || 0).toFixed(2)} m path (not moving)`); }
      else { view.plannedPath = []; banner(`❌ ${r.data.error || 'plan failed'}`); }
    } finally { planning = false; }
  }
  async function go(goal) {
    goal = goal || view.goal;
    if (!goal) { toast('Tap the map to set a goal', 'warn'); return; }
    if (!H.getToken()) { openToken(); return; }
    if (!window.confirm(`Nav2 will ARM Rover 1 and drive to ${goal.label || 'goal'}.\nArea clear?`)) return;
    view.goal = goal;
    banner('🚀 Dispatching…');
    const r = await H.dispatchGoal(goal);
    if (r.ok) banner('🚀 Executing — E-STOP any time');
    else { banner(`❌ ${r.data.error || 'rejected'}`); toast(`Dispatch rejected: ${r.data.error || r.status}`, 'error'); }
  }
  function home() {
    const g = H.homeGoal();
    if (!g) { toast('HOME pose unavailable', 'error'); return; }
    go(g);
  }
  $('btnMobGo').addEventListener('click', () => go());
  $('btnMobPreview').addEventListener('click', () => plan());
  $('btnMobHome').addEventListener('click', home);
  $('btnTopHome').addEventListener('click', home);
  $('btnMobSetHome').addEventListener('click', async () => {
    if (!window.confirm('Set AMCL pose to the HOME pad? (Rover must be sitting on HOME. No motion.)')) return;
    const r = await H.initHome();
    toast(r.ok ? '📍 Localized at HOME' : `Localize failed: ${r.data.error || r.status}`, r.ok ? 'ok' : 'error');
  });
  document.querySelectorAll('.pbtn').forEach(b => b.addEventListener('click', () => {
    $('missionDrawer').classList.remove('open');
    plan(H.relativeGoal(parseFloat(b.dataset.dist), parseFloat(b.dataset.ang)));
  }));
  $('btnMobMissionPlan').addEventListener('click', async () => {
    banner('⏳ Planning mission…');
    try {
      const r = await H.previewMission($('mobMissionInput').value);
      view.plannedPath = r.path;
      const last = r.legs[r.legs.length - 1];
      view.goal = last ? { x: last.target.x, y: last.target.y, yawRad: last.target.yaw, label: 'Mission end' } : null;
      banner(r.ok ? `✅ ${r.legs.length} legs, ${r.lengthM.toFixed(2)} m (preview only)` : `❌ ${r.error}`);
    } catch (err) { banner(`❌ ${err.message}`); }
    $('missionDrawer').classList.remove('open');
  });

  // ── Status chips ───────────────────────────────────────────
  H.on('connection', (s) => { $('dot').classList.toggle('on', s.wsConnected); if (!s.wsConnected) { st.lin = st.ang = 0; resetKnobs(); } });
  H.on('battery', (b) => {
    const v = H.batteryView(b);
    const c = $('cBat');
    c.textContent = `🔋 ${v.text} ${v.voltText}`;
    c.classList.remove('plug-ac', 'warn-yellow', 'crit-flash');
    if (v.level === 'critical') c.classList.add('crit-flash');
    else if (v.level === 'warn') c.classList.add('warn-yellow');
    else if (v.plugClass === 'plug-state-ac') c.classList.add('plug-ac');
  });
  H.on('system', (s) => {
    const t = s && s.cpuTempC;
    setText('cTmp', isNum(t) ? `${t.toFixed(0)} °C` : '-- °C');
    $('cTmp').style.color = H.tempColor(t);
  });
  setInterval(() => {
    const age = H.state.lastWsMsg ? Date.now() - H.state.lastWsMsg : null;
    setText('cLat', age === null ? '-- ms' : (age > 5000 ? 'STALE' : `${age} ms`));
  }, 500);
  const refreshYaw = () => {
    const p = H.getPose();
    setText('hYaw', p.frame === 'none' ? 'YAW --°' : `YAW ${p.yawDeg.toFixed(0)}°${p.frame === 'odom' ? ' (odom)' : ''}`);
  };
  H.on('odom', refreshYaw);
  H.on('localization', refreshYaw);

  // ── Boot ───────────────────────────────────────────────────
  refreshArm();
  refreshAuth();
  H.start();
  if (!H.getToken()) setTimeout(() => toast('Tap 🔑 to enter the operator token (needed to ARM)', 'warn'), 1200);
  window.__hudMobile = { setMobMode, view };
})();
