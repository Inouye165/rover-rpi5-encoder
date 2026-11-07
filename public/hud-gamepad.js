/*
 * Rover 1 HUD Cockpit — Shared Gamepad Controller Module (public/hud-gamepad.js)
 * Shared between Desktop HUD (hud.html) and Mobile Touch Cockpit (hud-mobile.html).
 * Enforces Rover 1 Exact Motion Safety Contract & Standard Xbox Mapping:
 *   - RT (button 7) or RB (button 5) Deadman (LB (4) / LT (6) preserved as aliases)
 *   - Left Stick: axes[0] steering (+right), -axes[1] linear throttle (+forward)
 *   - Start (button 9): edge-triggered arm (guarded against concurrent/holding requests)
 *   - Back / Select (button 8): disarm
 *   - A (0), B (1), X (2), Y (3), L3 (10), R3 (11): E-STOP
 *   - Strict Safety Precedence: E-STOP > Disarm > Arm > Motion
 *   - RB must never increase speed merely because it is held as deadman (boost = 1)
 *   - Focus loss / pagehide / hidden page / disconnect safety interlock:
 *     Requires a deliberate fresh deadman release & re-engagement before motion resumes
 *   - Tolerates absent DOM elements on either layout
 */
(function (global) {
  'use strict';

  const H = global.RoverHUD;
  if (!H) {
    console.warn('[Gamepad] RoverHUD not found; deferred until HUD core is loaded.');
    return;
  }

  const $ = (id) => (global.document ? global.document.getElementById(id) : null);
  const setText = (id, text) => {
    const el = $(id);
    if (el && el.textContent !== text) el.textContent = text;
  };

  function sendToast(msg, level = 'ok') {
    if (H && typeof H.emit === 'function') {
      H.emit('toast', { text: msg, level });
    }
    if (typeof global.toast === 'function') {
      global.toast(msg, level);
    }
  }

  const GP_DEADBAND = 0.12;
  // A (0), B (1), X (2), Y (3), Left Stick Click (10), Right Stick Click (11) trigger ESTOP.
  // Back (8) disarms; Start (9) arms.
  const ESTOP_BUTTONS = [0, 1, 2, 3, 10, 11];

  let gpPrevEstop = false;
  let gpPrevDisarm = false;
  let gpPrevArm = false;
  let gpArmBusy = false;
  let gpDeadmanInterlocked = false;
  let gpBlockedError = false;

  function shape(v) {
    const a = Math.abs(v);
    if (a < GP_DEADBAND) return 0;
    return Math.sign(v) * (a - GP_DEADBAND) / (1 - GP_DEADBAND);
  }

  function isBtnPressed(gp, idx) {
    if (!gp || !gp.buttons) return false;
    const b = gp.buttons[idx];
    if (!b) return false;
    return Boolean(b.pressed || (typeof b.value === 'number' && b.value > 0.5));
  }

  function getPad() {
    if (!global.navigator || !global.navigator.getGamepads) return null;
    try {
      const pads = global.navigator.getGamepads();
      if (!pads) return null;
      for (let i = 0; i < pads.length; i++) {
        if (pads[i] && pads[i].connected) return pads[i];
      }
      return null;
    } catch (err) {
      gpBlockedError = true;
      return null;
    }
  }

  function resetGamepadDrive() {
    gpDeadmanInterlocked = true;
    H.clearDriveInput('gamepad');
  }

  // Interruption listeners
  if (global.addEventListener) {
    global.addEventListener('blur', resetGamepadDrive);
    global.addEventListener('pagehide', resetGamepadDrive);
    if (global.document && global.document.addEventListener) {
      global.document.addEventListener('visibilitychange', () => {
        if (global.document.hidden) resetGamepadDrive();
      });
    }
    global.addEventListener('gamepaddisconnected', () => {
      resetGamepadDrive();
      sendToast('🎮 Controller disconnected', 'warn');
    });
    global.addEventListener('gamepadconnected', (e) => {
      gpDeadmanInterlocked = true;
      sendToast(`🎮 Controller connected: ${e.gamepad.id}`, 'ok');
    });
  }

  function updateDesktopUI(gp, isHttpBlocked, deadmanHeld, deadmanActive, moving) {
    const badge = $('gamepad-badge');
    const connDot = $('gp-conn-dot');
    const secAlert = $('gp-security-alert');
    const wakeHint = $('gp-wake-hint');
    const dm = $('gp-deadman-indicator');
    const tb = $('gp-turbo-indicator');
    const alertEl = $('gp-deadman-alert');

    if (secAlert) {
      secAlert.style.display = isHttpBlocked ? 'block' : 'none';
      if (global.location && global.location.origin) {
        setText('gp-origin', global.location.origin);
      }
    }

    if (!global.navigator || !global.navigator.getGamepads) {
      setText('gamepad-status-text', 'API UNAVAILABLE');
      if (badge) badge.style.color = '#5f7a91';
      if (connDot) connDot.style.background = '#5f7a91';
      setText('gp-name', 'Gamepad API unavailable');
      if (wakeHint) wakeHint.style.display = 'none';
      if (secAlert) secAlert.style.display = 'none';
      return;
    }

    if (!gp) {
      if (isHttpBlocked) {
        setText('gamepad-status-text', 'HTTP BLOCKED');
        if (badge) badge.style.color = '#ffaa00';
        if (connDot) connDot.style.background = '#ffaa00';
        setText('gp-name', 'Gamepad blocked on insecure origin');
      } else {
        setText('gamepad-status-text', 'NO GAMEPAD');
        if (badge) badge.style.color = '#5f7a91';
        if (connDot) connDot.style.background = '#5f7a91';
        setText('gp-name', 'No gamepad detected');
      }
      if (wakeHint) wakeHint.style.display = isHttpBlocked ? 'none' : 'block';
      if (dm) {
        dm.textContent = 'DEADMAN: RELEASED';
        dm.style.borderColor = '#ff2a4b';
        dm.style.color = '#ff4765';
      }
      if (tb) {
        tb.textContent = 'START: ARM | BACK: DISARM';
        tb.style.borderColor = '#203146';
        tb.style.color = '#8ba2b5';
      }
      if (alertEl) alertEl.style.display = 'none';
      return;
    }

    setText('gamepad-status-text', 'GAMEPAD OK');
    if (badge) badge.style.color = '#00ffcc';
    if (connDot) connDot.style.background = '#00ffcc';
    setText('gp-name', gp.id);
    if (wakeHint) wakeHint.style.display = 'none';

    if (dm) {
      if (deadmanHeld && gpDeadmanInterlocked) {
        dm.textContent = 'DEADMAN: RE-ENGAGE';
        dm.style.borderColor = '#ffaa00';
        dm.style.color = '#ffaa00';
      } else if (deadmanActive) {
        dm.textContent = 'DEADMAN: HELD';
        dm.style.borderColor = '#00ff88';
        dm.style.color = '#00ff88';
      } else {
        dm.textContent = 'DEADMAN: RELEASED';
        dm.style.borderColor = '#ff2a4b';
        dm.style.color = '#ff4765';
      }
    }
    if (tb) {
      tb.textContent = 'START: ARM | BACK: DISARM';
      tb.style.borderColor = '#203146';
      tb.style.color = '#8ba2b5';
    }

    if (alertEl) {
      if (moving && deadmanHeld && gpDeadmanInterlocked) {
        alertEl.textContent = '⚠️ RELEASE & RE-SQUEEZE DEADMAN (Safety Re-engagement Required)';
        alertEl.style.display = 'block';
      } else if (moving && !deadmanActive) {
        alertEl.textContent = '⚠️ SQUEEZE RT OR RB TO DRIVE (Deadman Safety Active)';
        alertEl.style.display = 'block';
      } else {
        alertEl.style.display = 'none';
      }
    }
  }

  function updateMobileUI(gp, isHttpBlocked, deadmanHeld, deadmanActive, moving) {
    const cGp = $('cGp');
    if (!cGp) return;

    if (!global.navigator || !global.navigator.getGamepads) {
      cGp.textContent = '🎮 N/A';
      cGp.style.color = 'var(--dim)';
      cGp.style.borderColor = 'var(--line)';
      cGp.title = 'Gamepad API not available in this browser';
      return;
    }

    if (!gp) {
      if (isHttpBlocked) {
        cGp.textContent = '🎮 BLOCKED';
        cGp.style.color = 'var(--warn)';
        cGp.style.borderColor = 'var(--warn)';
        cGp.title = 'Gamepad blocked on insecure HTTP origin. Use ssh -L 3000:localhost:3000 or edge://flags';
      } else {
        cGp.textContent = '🎮 OFF';
        cGp.style.color = 'var(--dim)';
        cGp.style.borderColor = 'var(--line)';
        cGp.title = 'No gamepad detected. Press any button on controller to connect.';
      }
      return;
    }

    // Gamepad connected
    if (deadmanHeld && gpDeadmanInterlocked) {
      cGp.textContent = '🎮 RE-ENG';
      cGp.style.color = 'var(--warn)';
      cGp.style.borderColor = 'var(--warn)';
      cGp.title = 'Safety Interlock: Release and re-squeeze RT/RB to drive';
    } else if (deadmanActive) {
      cGp.textContent = '🎮 HELD';
      cGp.style.color = 'var(--green)';
      cGp.style.borderColor = 'var(--green)';
      cGp.title = `Driving: ${gp.id} (Deadman Active)`;
    } else {
      cGp.textContent = '🎮 OK';
      cGp.style.color = 'var(--ok)';
      cGp.style.borderColor = 'rgba(0,255,204,0.45)';
      cGp.title = `Connected: ${gp.id} (Hold RT/RB to drive)`;
    }
  }

  // Click handler on mobile cGp to show diagnostics toast
  if (global.addEventListener) {
    global.addEventListener('DOMContentLoaded', () => {
      const cGp = $('cGp');
      if (cGp) {
        cGp.style.cursor = 'pointer';
        cGp.addEventListener('click', () => {
          const gp = getPad();
          const isHttpBlocked =
            (!global.isSecureContext &&
              global.location &&
              global.location.hostname !== 'localhost' &&
              global.location.hostname !== '127.0.0.1') ||
            gpBlockedError;
          if (!global.navigator || !global.navigator.getGamepads) {
            sendToast('Gamepad API unavailable in this browser', 'warn');
          } else if (isHttpBlocked) {
            sendToast('⚠️ HTTP Block: Tunnel via ssh -L 3000:localhost:3000 or add origin to edge://flags', 'warn');
          } else if (!gp) {
            sendToast('🎮 No gamepad detected — press any button on controller', 'ok');
          } else {
            sendToast(`🎮 Controller active: ${gp.id} — hold RT/RB to drive`, 'ok');
          }
        });
      }
    });
  }

  function checkHttpBlocked() {
    if (gpBlockedError) return true;
    if (global.isSecureContext === true) return false;
    if (
      global.location &&
      (global.location.hostname === 'localhost' || global.location.hostname === '127.0.0.1')
    ) {
      return false;
    }
    return true;
  }

  function pollGamepad() {
    // 1. Check Gamepad API availability
    if (!global.navigator || !global.navigator.getGamepads) {
      const isBlocked = checkHttpBlocked();
      updateDesktopUI(null, isBlocked, false, false, false);
      updateMobileUI(null, isBlocked, false, false, false);
      H.clearDriveInput('gamepad');
      return;
    }

    const gp = getPad();
    const isHttpBlocked = checkHttpBlocked();

    if (!gp) {
      updateDesktopUI(null, isHttpBlocked, false, false, false);
      updateMobileUI(null, isHttpBlocked, false, false, false);
      resetGamepadDrive();
      return;
    }

    // 2. Hidden page guard
    if (global.document && global.document.hidden) {
      resetGamepadDrive();
      updateDesktopUI(gp, isHttpBlocked, false, false, false);
      updateMobileUI(gp, isHttpBlocked, false, false, false);
      return;
    }

    // 3. Safety Priority 1: E-STOP (buttons A, B, X, Y, L3, R3)
    const estopNow = ESTOP_BUTTONS.some((i) => isBtnPressed(gp, i));
    const disarmNow = isBtnPressed(gp, 8);
    const armNow = isBtnPressed(gp, 9);

    if (estopNow) {
      if (!gpPrevEstop) {
        H.emergencyStop('GAMEPAD');
        sendToast('🛑 E-STOP via controller', 'error');
      }
      gpPrevEstop = true;
      gpPrevDisarm = disarmNow;
      gpPrevArm = armNow;
      H.clearDriveInput('gamepad');
      updateDesktopUI(gp, isHttpBlocked, false, false, false);
      updateMobileUI(gp, isHttpBlocked, false, false, false);
      return;
    }
    gpPrevEstop = false;

    // 4. Safety Priority 2: DISARM (Back / Select - button 8)
    if (disarmNow) {
      if (!gpPrevDisarm) {
        H.disarm();
        sendToast('🔒 Drivetrain disarmed via controller (Back)', 'warn');
      }
      gpPrevDisarm = true;
      gpPrevArm = armNow;
      H.clearDriveInput('gamepad');
      updateDesktopUI(gp, isHttpBlocked, false, false, false);
      updateMobileUI(gp, isHttpBlocked, false, false, false);
      return;
    }
    gpPrevDisarm = false;

    // 5. ARM: Start (button 9) - strictly edge-triggered and guarded against duplicates
    if (armNow) {
      if (!gpPrevArm && !gpArmBusy) {
        gpPrevArm = true;
        if (!H.getToken()) {
          sendToast('Operator token required to arm', 'warn');
          const tokInput = $('token-input') || $('mobTokenInput');
          if (tokInput) tokInput.focus();
        } else if (!H.state.drive || !H.state.drive.armed) {
          gpArmBusy = true;
          H.arm()
            .then((res) => {
              if (res && !res.ok) {
                sendToast((res.data && res.data.error) || 'Arm failed', 'error');
              } else {
                sendToast('🔓 Drivetrain armed via controller (Start)', 'ok');
              }
            })
            .catch((err) => {
              sendToast((err && err.message) || 'Arm error', 'error');
            })
            .finally(() => {
              gpArmBusy = false;
            });
        }
      }
      H.clearDriveInput('gamepad');
      updateDesktopUI(gp, isHttpBlocked, false, false, false);
      updateMobileUI(gp, isHttpBlocked, false, false, false);
      return;
    }
    gpPrevArm = false;

    // 6. Deadman evaluation: RT (7) or RB (5), with aliases LB (4) or LT (6)
    const deadmanHeld =
      isBtnPressed(gp, 7) || isBtnPressed(gp, 5) || isBtnPressed(gp, 4) || isBtnPressed(gp, 6);

    if (!deadmanHeld) {
      // Deliberate release clears safety interlock after interruption
      gpDeadmanInterlocked = false;
    }

    const deadmanActive = deadmanHeld && !gpDeadmanInterlocked;

    // Left-stick steering (axes[0]) and linear throttle (-axes[1])
    const y = -shape(gp.axes[1] || 0);
    const x = shape(gp.axes[0] || 0);
    const moving = Math.abs(x) > 0 || Math.abs(y) > 0;

    updateDesktopUI(gp, isHttpBlocked, deadmanHeld, deadmanActive, moving);
    updateMobileUI(gp, isHttpBlocked, deadmanHeld, deadmanActive, moving);

    if (deadmanActive && moving) {
      // RB must never increase speed merely because it is held as deadman (boost=1)
      H.setDriveInput('gamepad', x, y, 1);
    } else {
      H.clearDriveInput('gamepad');
    }
  }

  // Exactly one controller polling loop
  if (!global._roverGamepadLoopStarted) {
    global._roverGamepadLoopStarted = true;
    global._roverGamepadInterval = setInterval(pollGamepad, 50);
  }
  global._pollGamepad = pollGamepad;
  global._getPad = getPad;
  global._resetGamepadDrive = resetGamepadDrive;

  global.RoverGamepad = {
    poll: pollGamepad,
    resetDrive: resetGamepadDrive,
    getPad
  };
})(typeof window !== 'undefined' ? window : globalThis);
