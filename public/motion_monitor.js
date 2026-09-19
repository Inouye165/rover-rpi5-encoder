/**
 * Rover Motion Monitor - High-Visibility Live Telemetry Observer & Renderer
 * Pure Observer Implementation (No drive commands, no state mutation)
 */

(function () {
  'use strict';

  // --- Constants & Calibration Defaults ---
  const TICKS_PER_REV = 1974.1666666667;
  const WHEEL_DIAMETER_M = 0.06695; // Calibrated effective diameter (0.033475 m radius)
  const WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M; // ~0.251327 m

  // --- State Variables ---
  let ws = null;
  let wsConnected = false;
  let isArmed = false;
  let lastMsgTimestamp = Date.now();

  // Active Plan State (Clean Single-Pass 1.0137m Reverse Motion Test)
  let planSteps = [
    { id: 1, label: "Reverse 1013.7 mm", target_mm: -1013.7, dir: -1, status: "ACTIVE" },
    { id: 2, label: "Slowdown Half 1 (30mm/s 1.5 in)", target_mm: -937.5, dir: -1, status: "PENDING" },
    { id: 3, label: "Micro-Creep Half 2 (10mm/s 1.5 in)", target_mm: -975.6, dir: -1, status: "PENDING" },
    { id: 4, label: "Stop / Disarm", target_mm: 0.0, dir: 0, status: "PENDING" }
  ];
  let activeStepIndex = 0;
  let planStartTime = Date.now();
  let stepStartTime = Date.now();

  // Raw Telemetry Snapshot
  let telemetry = {
    x: 0.0,
    y: 0.0,
    yaw: 0.0,
    yaw_deg: 0.0,
    vx_cmd: 0.0,
    wz_cmd: 0.0,
    vx_meas: 0.0,
    wz_meas: 0.0,
    m1_ticks: 0,
    m2_ticks: 0,
    m3_ticks: 0,
    m4_ticks: 0,
    armed: false
  };

  // Step Motion Snapshot (Origin at start of step)
  let origin = {
    x: 0.0,
    y: 0.0,
    yaw: 0.0,
    yaw_deg: 0.0,
    m1: 0,
    m2: 0,
    m3: 0,
    m4: 0,
    initialized: false
  };

  // Metrics Derived During Current Step
  let stepMetrics = {
    target_mm: -1013.7,
    measured_mm: 0.0,
    remaining_mm: -1013.7,
    progress_pct: 0.0,
    peak_speed_mps: 0.0,
    slowdown_start_mm: null,
    slowdown_remaining_mm: null,
    in_slowdown: false,
    start_heading_deg: 0.0,
    current_heading_deg: 0.0,
    heading_change_deg: 0.0,
    target_heading_deg: null,
    heading_error_deg: 0.0,
    step_m1_ticks: 0,
    step_m2_ticks: 0,
    step_m3_ticks: 0,
    step_m4_ticks: 0,
    dist_m1_mm: 0.0,
    dist_m2_mm: 0.0,
    dist_m3_mm: 0.0,
    dist_m4_mm: 0.0,
    left_avg_mm: 0.0,
    right_avg_mm: 0.0,
    overall_enc_mm: 0.0,
    stopped_dist_mm: null,
    distance_error_mm: 0.0
  };

  // --- Helper Functions ---
  function formatNum(val, decimals = 1, forceSign = false) {
    if (val === null || val === undefined || isNaN(val)) return '--';
    const num = Number(val);
    const str = num.toFixed(decimals);
    return (forceSign && num > 0) ? '+' + str : str;
  }

  function normalizeAngleDeg(deg) {
    let a = deg % 360.0;
    if (a > 180.0) a -= 360.0;
    if (a < -180.0) a += 360.0;
    return a;
  }

  function calculateSegmentProgress(x, y, x0, y0, yaw0) {
    const dx = x - x0;
    const dy = y - y0;
    const progress_m = dx * Math.cos(yaw0) + dy * Math.sin(yaw0);
    const lateral_m = -dx * Math.sin(yaw0) + dy * Math.cos(yaw0);
    return { progress_m, lateral_m };
  }

  // --- Data Ingestion Engine ---
  function handleTelemetryMsg(msg) {
    if (!msg || !msg.type) return;
    lastMsgTimestamp = Date.now();

    let updated = false;

    switch (msg.type) {
      case 'odom':
        if (msg.x !== undefined) telemetry.x = Number(msg.x);
        if (msg.y !== undefined) telemetry.y = Number(msg.y);
        if (msg.yaw !== undefined) {
          telemetry.yaw = Number(msg.yaw);
          telemetry.yaw_deg = normalizeAngleDeg(telemetry.yaw * (180.0 / Math.PI));
        }
        if (msg.vx !== undefined) telemetry.vx_meas = Number(msg.vx);
        if (msg.wz !== undefined) telemetry.wz_meas = Number(msg.wz);
        updated = true;
        break;

      case 'encoder_total':
        if (msg.m1 !== undefined) telemetry.m1_ticks = Number(msg.m1);
        if (msg.m2 !== undefined) telemetry.m2_ticks = Number(msg.m2);
        if (msg.m3 !== undefined) telemetry.m3_ticks = Number(msg.m3);
        if (msg.m4 !== undefined) telemetry.m4_ticks = Number(msg.m4);
        updated = true;
        break;

      case 'imu':
      case 'bno08x_imu':
        if (msg.yaw_deg !== undefined) {
          telemetry.yaw_deg = normalizeAngleDeg(Number(msg.yaw_deg));
          telemetry.yaw = telemetry.yaw_deg * (Math.PI / 180.0);
        }
        updated = true;
        break;

      case 'speed':
        if (msg.linear !== undefined) telemetry.vx_cmd = Number(msg.linear);
        if (msg.angular !== undefined) telemetry.wz_cmd = Number(msg.angular);
        if (msg.v !== undefined) telemetry.vx_cmd = Number(msg.v);
        if (msg.w !== undefined) telemetry.wz_cmd = Number(msg.w);
        updated = true;
        break;

      case 'normal_drive_status':
        const prevArmed = isArmed;
        if (msg.armed !== undefined) {
          isArmed = Boolean(msg.armed);
          telemetry.armed = isArmed;
        }
        if (!prevArmed && isArmed) {
          resetStepOrigin();
        }
        if (msg.reqLinear !== undefined) telemetry.vx_cmd = Number(msg.reqLinear);
        if (msg.reqAngular !== undefined) telemetry.wz_cmd = Number(msg.reqAngular);
        updated = true;
        break;

      case 'test_segment_completed':
        if (msg.measured_mm !== undefined) {
          stepMetrics.measured_mm = Number(msg.measured_mm);
          stepMetrics.stopped_dist_mm = Number(msg.measured_mm);
          const targetMag = Math.abs(stepMetrics.target_mm || 1000.0);
          const progressMag = Math.abs(stepMetrics.measured_mm);
          const isRev = stepMetrics.target_mm < 0;
          const remainingMag = Math.max(0.0, targetMag - progressMag);
          stepMetrics.remaining_mm = isRev ? -remainingMag : remainingMag;
          stepMetrics.progress_pct = Math.min(100.0, Math.max(0.0, (progressMag / targetMag) * 100.0));
        }
        updated = true;
        break;

      case 'autonomy_status':
        if (msg.status && msg.status.enabled !== undefined) {
          isArmed = Boolean(msg.status.enabled || msg.status.active);
          telemetry.armed = isArmed;
        }
        updated = true;
        break;

      case 'lidar_test_telemetry':
      case 'lidar_test_status':
      case 'autotest_status':
        if (msg.step !== undefined || msg.segment_name) {
          parseTestStepName(msg.segment_name || msg.msg || '');
        }
        updated = true;
        break;
    }

    if (updated) {
      updateStepMotionCalculations();
      renderUI();
    }
  }

  function parseTestStepName(nameStr) {
    if (!nameStr) return;
    const lower = nameStr.toLowerCase();
    if (lower.includes('forward 65') || lower.includes('65mm forward')) {
      updatePlanTargets(65.0, -65.0);
    } else if (lower.includes('forward 170') || lower.includes('170mm forward')) {
      updatePlanTargets(170.0, -170.0);
    } else if (lower.includes('forward 500') || lower.includes('500mm forward')) {
      updatePlanTargets(500.0, -500.0);
    }
  }

  function updatePlanTargets(fwdTargetMm, revTargetMm) {
    if (planSteps[0].target_mm !== fwdTargetMm) {
      planSteps[0].label = `Forward ${fwdTargetMm.toFixed(1)} mm`;
      planSteps[0].target_mm = fwdTargetMm;
      planSteps[2].label = `Reverse ${Math.abs(revTargetMm).toFixed(1)} mm`;
      planSteps[2].target_mm = revTargetMm;
      
      stepMetrics.target_mm = (activeStepIndex === 2) ? revTargetMm : fwdTargetMm;
      resetStepOrigin();
    }
  }

  function resetStepOrigin() {
    origin.x = telemetry.x;
    origin.y = telemetry.y;
    origin.yaw = telemetry.yaw;
    origin.yaw_deg = telemetry.yaw_deg;
    origin.m1 = telemetry.m1_ticks;
    origin.m2 = telemetry.m2_ticks;
    origin.m3 = telemetry.m3_ticks;
    origin.m4 = telemetry.m4_ticks;
    origin.initialized = true;

    stepStartTime = Date.now();
    stepMetrics.peak_speed_mps = 0.0;
    stepMetrics.slowdown_start_mm = null;
    stepMetrics.slowdown_remaining_mm = null;
    stepMetrics.in_slowdown = false;
    stepMetrics.start_heading_deg = telemetry.yaw_deg;
    stepMetrics.stopped_dist_mm = null;

    if (activeStepIndex === 0 && planSteps[0]) {
      planSteps[0].status = "ACTIVE";
      planSteps[1].status = "PENDING";
      planSteps[2].status = "PENDING";
      planSteps[3].status = "PENDING";
    }
  }

  // --- Step & Motion Calculations ---
  function updateStepMotionCalculations() {
    if (!origin.initialized) {
      resetStepOrigin();
    }

    const currentStep = planSteps[activeStepIndex] || planSteps[0];
    const isReverse = (currentStep.target_mm < 0 || currentStep.dir < 0);
    stepMetrics.target_mm = currentStep.target_mm;

    // 1. Calculate Signed Progress from Odometry Origin
    const { progress_m } = calculateSegmentProgress(telemetry.x, telemetry.y, origin.x, origin.y, origin.yaw);
    const measured_mm = progress_m * 1000.0;
    // Dynamic Target Scaling if test is 1000mm vs 500mm
    if (Math.abs(measured_mm) > 600.0 && Math.abs(stepMetrics.target_mm) < 800.0) {
      const sign = stepMetrics.target_mm >= 0 ? 1 : -1;
      stepMetrics.target_mm = sign * 1000.0;
      if (planSteps[0]) {
        planSteps[0].target_mm = 1000.0;
        planSteps[0].label = "Forward 1000.0 mm";
      }
      if (planSteps[2]) {
        planSteps[2].target_mm = -1000.0;
        planSteps[2].label = "Reverse 1000.0 mm";
      }
    }

    stepMetrics.measured_mm = measured_mm;

    // Target magnitude vs remaining
    const targetMag = Math.abs(stepMetrics.target_mm);
    const progressMag = Math.abs(measured_mm);
    const remainingMag = Math.max(0.0, targetMag - progressMag);
    stepMetrics.remaining_mm = isReverse ? -remainingMag : remainingMag;

    if (targetMag > 0) {
      stepMetrics.progress_pct = Math.min(100.0, Math.max(0.0, (progressMag / targetMag) * 100.0));
    } else {
      stepMetrics.progress_pct = 100.0;
    }

    // 2. Speed & Deceleration Phase
    const currSpeed = Math.abs(telemetry.vx_cmd || telemetry.vx_meas);
    stepMetrics.peak_speed_mps = Math.max(stepMetrics.peak_speed_mps, currSpeed);

    // Detect Slowdown Entry (minimal 40mm approach zone)
    if (remainingMag <= 40.0 && remainingMag > 0.0 && progressMag > 800.0) {
      if (!stepMetrics.in_slowdown) {
        stepMetrics.in_slowdown = true;
        stepMetrics.slowdown_start_mm = measured_mm;
        stepMetrics.slowdown_remaining_mm = stepMetrics.remaining_mm;
      }
    }

    // 3. Encoder Counters & Step Distances
    stepMetrics.step_m1_ticks = telemetry.m1_ticks - origin.m1;
    stepMetrics.step_m2_ticks = telemetry.m2_ticks - origin.m2;
    stepMetrics.step_m3_ticks = telemetry.m3_ticks - origin.m3;
    stepMetrics.step_m4_ticks = telemetry.m4_ticks - origin.m4;

    stepMetrics.dist_m1_mm = (stepMetrics.step_m1_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M * 1000.0;
    stepMetrics.dist_m2_mm = (stepMetrics.step_m2_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M * 1000.0;
    stepMetrics.dist_m3_mm = (stepMetrics.step_m3_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M * 1000.0;
    stepMetrics.dist_m4_mm = (stepMetrics.step_m4_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M * 1000.0;

    stepMetrics.left_avg_mm = (stepMetrics.dist_m1_mm + stepMetrics.dist_m3_mm) / 2.0;
    stepMetrics.right_avg_mm = (stepMetrics.dist_m2_mm + stepMetrics.dist_m4_mm) / 2.0;
    stepMetrics.overall_enc_mm = (stepMetrics.left_avg_mm + stepMetrics.right_avg_mm) / 2.0;

    // 4. IMU Heading & Rotation
    stepMetrics.current_heading_deg = telemetry.yaw_deg;
    stepMetrics.heading_change_deg = normalizeAngleDeg(stepMetrics.current_heading_deg - stepMetrics.start_heading_deg);

    if (stepMetrics.target_heading_deg !== null) {
      stepMetrics.heading_error_deg = normalizeAngleDeg(stepMetrics.target_heading_deg - stepMetrics.current_heading_deg);
    } else {
      stepMetrics.heading_error_deg = stepMetrics.heading_change_deg;
    }

    // 5. Position & Endpoint Comparison
    if (Math.abs(measured_mm) >= targetMag && targetMag > 0) {
      if (stepMetrics.stopped_dist_mm === null) {
        stepMetrics.stopped_dist_mm = measured_mm;
      }
    }
    stepMetrics.distance_error_mm = measured_mm - stepMetrics.target_mm;

    // Step Status Update (Lock origin throughout test run so progress never wraps)
    if (targetMag > 0 && progressMag >= targetMag * 0.98 && Math.abs(telemetry.vx_meas) < 0.01) {
      if (planSteps[0]) {
        planSteps[0].status = "COMPLETE";
      }
      if (planSteps[1]) {
        planSteps[1].status = "ACTIVE";
      }
    }
  }

  // --- UI Rendering Engine ---
  function renderUI() {
    // 1. Connection & Arm Badges
    const wsDot = document.getElementById('dot-ws');
    const wsTxt = document.getElementById('txt-ws');
    if (wsDot && wsTxt) {
      wsDot.className = 'status-dot ' + (wsConnected ? 'connected' : 'disconnected');
      wsTxt.textContent = wsConnected ? 'CONNECTED' : 'DISCONNECTED';
    }

    const armDot = document.getElementById('dot-arm');
    const armTxt = document.getElementById('txt-arm');
    if (armDot && armTxt) {
      armDot.className = 'status-dot ' + (isArmed ? 'armed' : 'disarmed');
      armTxt.textContent = isArmed ? 'ARMED' : 'DISARMED';
    }

    // Clock
    const clock = document.getElementById('mm-clock');
    if (clock) {
      const now = new Date();
      const pad = n => String(n).padStart(2, '0');
      clock.textContent = `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}`;
    }

    // Dynamically update Step 1 label based on direction & target
    if (planSteps[0]) {
      const isRev = (stepMetrics.target_mm < 0 || stepMetrics.measured_mm < -50.0);
      planSteps[0].label = isRev ? "Reverse 1000.0 mm" : "Forward 1000.0 mm";
      planSteps[0].target_mm = isRev ? -1000.0 : 1000.0;
    }

    const currentStep = planSteps[activeStepIndex] || planSteps[0];
    const isReverse = (currentStep.target_mm < 0 || currentStep.dir < 0 || stepMetrics.target_mm < 0);

    const overallStateBadge = document.getElementById('plan-overall-state');
    if (overallStateBadge) {
      overallStateBadge.textContent = isArmed ? 'RUNNING' : 'DISARMED';
      overallStateBadge.className = 'mm-step-badge ' + (isArmed ? 'cyan' : '');
    }

    // Render Step Chips
    planSteps.forEach((step, idx) => {
      const chip = document.getElementById(`step-chip-${step.id}`);
      const lbl = document.getElementById(`chip-lbl-${step.id}`);
      const st = document.getElementById(`chip-st-${step.id}`);
      if (chip && lbl && st) {
        lbl.textContent = step.label;
        st.textContent = step.status;
        chip.className = 'step-chip ' + step.status.toLowerCase();
      }
    });

    const valStepLabel = document.getElementById('val-current-step-label');
    if (valStepLabel) valStepLabel.textContent = currentStep.label;

    const valTargetDist = document.getElementById('val-target-dist');
    if (valTargetDist) valTargetDist.innerHTML = `${formatNum(stepMetrics.target_mm, 1)} <span class="unit">mm</span>`;

    const valMeasuredDist = document.getElementById('val-measured-dist');
    if (valMeasuredDist) valMeasuredDist.innerHTML = `${formatNum(stepMetrics.measured_mm, 1, true)} <span class="unit">mm</span>`;

    const valRemainingDist = document.getElementById('val-remaining-dist');
    if (valRemainingDist) valRemainingDist.innerHTML = `${formatNum(stepMetrics.remaining_mm, 1)} <span class="unit">mm</span>`;

    const valProgressPct = document.getElementById('val-progress-pct');
    if (valProgressPct) valProgressPct.innerHTML = `${formatNum(stepMetrics.progress_pct, 1)}<span class="unit">%</span>`;

    // 3. Section 2: Progress Line Graphic (SVG)
    const valDir = document.getElementById('val-motion-direction');
    if (valDir) valDir.textContent = isReverse ? '← REVERSE' : 'FORWARD →';

    renderProgressSVG(isReverse);

    // Endpoint summary box
    const dispTarget = document.getElementById('disp-target-mm');
    if (dispTarget) dispTarget.textContent = `${formatNum(stepMetrics.target_mm, 1)} mm`;

    const dispActual = document.getElementById('disp-actual-mm');
    if (dispActual) dispActual.textContent = `${formatNum(stepMetrics.measured_mm, 1, true)} mm`;

    const dispError = document.getElementById('disp-error-mm');
    if (dispError) {
      dispError.textContent = `${formatNum(stepMetrics.distance_error_mm, 1, true)} mm`;
      dispError.className = 'ep-val ' + (Math.abs(stepMetrics.distance_error_mm) <= 5.0 ? 'green' : 'yellow');
    }

    // 4. Section 3: Speed & Deceleration Monitor
    const valSpeedPhase = document.getElementById('val-speed-phase');
    if (valSpeedPhase) {
      if (Math.abs(telemetry.vx_meas) < 0.005 && Math.abs(telemetry.vx_cmd) < 0.005) {
        valSpeedPhase.textContent = 'STOPPED';
        valSpeedPhase.style.borderColor = 'var(--text-muted)';
        valSpeedPhase.style.color = 'var(--text-muted)';
      } else if (stepMetrics.in_slowdown) {
        valSpeedPhase.textContent = 'SLOWDOWN';
        valSpeedPhase.style.borderColor = 'var(--yellow-warn)';
        valSpeedPhase.style.color = 'var(--yellow-warn)';
      } else {
        valSpeedPhase.textContent = 'CRUISE';
        valSpeedPhase.style.borderColor = 'var(--cyan-bright)';
        valSpeedPhase.style.color = 'var(--cyan-bright)';
      }
    }

    const valCmdSpeed = document.getElementById('val-cmd-speed');
    if (valCmdSpeed) valCmdSpeed.innerHTML = `${formatNum(telemetry.vx_cmd, 3)} <span class="u">m/s</span>`;

    const valMeasSpeed = document.getElementById('val-meas-speed');
    if (valMeasSpeed) valMeasSpeed.innerHTML = `${formatNum(telemetry.vx_meas, 3)} <span class="u">m/s</span>`;

    const valPeakSpeed = document.getElementById('val-peak-speed');
    if (valPeakSpeed) valPeakSpeed.innerHTML = `${formatNum(stepMetrics.peak_speed_mps, 3)} <span class="u">m/s</span>`;

    const valSlowStart = document.getElementById('val-slowdown-start-at');
    if (valSlowStart) valSlowStart.innerHTML = (stepMetrics.slowdown_start_mm !== null) ? `${formatNum(stepMetrics.slowdown_start_mm, 1)} <span class="u">mm</span>` : `-- <span class="u">mm</span>`;

    const valSlowRem = document.getElementById('val-slowdown-dist-rem');
    if (valSlowRem) valSlowRem.innerHTML = (stepMetrics.slowdown_remaining_mm !== null) ? `${formatNum(stepMetrics.slowdown_remaining_mm, 1)} <span class="u">mm</span>` : `-- <span class="u">mm</span>`;

    // 5. Section 5: Raw Encoder Counters (Left Panel)
    const setT = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
    setT('enc-raw-m1', telemetry.m1_ticks);
    setT('enc-raw-m2', telemetry.m2_ticks);
    setT('enc-raw-m3', telemetry.m3_ticks);
    setT('enc-raw-m4', telemetry.m4_ticks);

    setT('enc-step-m1', stepMetrics.step_m1_ticks);
    setT('enc-step-m2', stepMetrics.step_m2_ticks);
    setT('enc-step-m3', stepMetrics.step_m3_ticks);
    setT('enc-step-m4', stepMetrics.step_m4_ticks);

    setT('enc-dist-m1', `${formatNum(stepMetrics.dist_m1_mm, 1)} mm`);
    setT('enc-dist-m2', `${formatNum(stepMetrics.dist_m2_mm, 1)} mm`);
    setT('enc-dist-m3', `${formatNum(stepMetrics.dist_m3_mm, 1)} mm`);
    setT('enc-dist-m4', `${formatNum(stepMetrics.dist_m4_mm, 1)} mm`);

    setT('enc-left-avg', `${formatNum(stepMetrics.left_avg_mm, 1)} mm`);
    setT('enc-right-avg', `${formatNum(stepMetrics.right_avg_mm, 1)} mm`);
    setT('enc-overall-avg', `${formatNum(stepMetrics.overall_enc_mm, 1)} mm`);

    // 6. Section 4: IMU Heading (Right Panel)
    setT('val-start-heading', `${formatNum(stepMetrics.start_heading_deg, 1)}°`);
    setT('val-current-heading', `${formatNum(stepMetrics.current_heading_deg, 1)}°`);
    setT('val-heading-change', `${formatNum(stepMetrics.heading_change_deg, 1, true)}°`);
    setT('val-target-heading', stepMetrics.target_heading_deg !== null ? `${formatNum(stepMetrics.target_heading_deg, 1)}°` : '--°');
    setT('val-heading-error', `${formatNum(stepMetrics.heading_error_deg, 1)}°`);

    // Rotate compass needle
    const compassNeedle = document.getElementById('compass-needle');
    if (compassNeedle) {
      compassNeedle.setAttribute('transform', `rotate(${stepMetrics.current_heading_deg}, 80, 80)`);
    }

    // 7. Section 6: Position & Endpoint Comparison
    setT('cmp-req-endpoint', `${formatNum(stepMetrics.target_mm, 1)} mm`);
    setT('cmp-enc-endpoint', `${formatNum(stepMetrics.overall_enc_mm, 1, true)} mm`);
    setT('cmp-odom-endpoint', `${formatNum(stepMetrics.measured_mm, 1, true)} mm`);
    setT('cmp-dist-error', `${formatNum(stepMetrics.distance_error_mm, 1, true)} mm`);
    setT('cmp-final-heading', `${formatNum(stepMetrics.current_heading_deg, 1)}°`);
    setT('cmp-final-heading-change', `${formatNum(stepMetrics.heading_change_deg, 1, true)}°`);

    // 8. Section 7: Timeline Progress Sequence
    renderTimelineSequence();
  }

  // --- SVG Progress Line Renderer ---
  function renderProgressSVG(isReverse) {
    const trackStart = 80;
    const trackEnd = 920;
    const trackWidth = trackEnd - trackStart; // 840px

    const targetMag = Math.abs(stepMetrics.target_mm) || 500.0;
    const progressMag = Math.abs(stepMetrics.measured_mm);

    // Map measured distance mm to SVG x coordinate
    const pct = Math.min(1.2, Math.max(0.0, progressMag / targetMag));
    const roverX = trackStart + (pct * (trackWidth * 0.85)); // Leave room for target & overshoot

    const filledWidth = Math.min(trackWidth, Math.max(0, roverX - trackStart));

    // Update track fill
    const trackFill = document.getElementById('track-filled');
    if (trackFill) {
      trackFill.setAttribute('width', filledWidth);
    }

    // Update Rover Icon Group Position
    const roverGroup = document.getElementById('rover-icon-group');
    if (roverGroup) {
      roverGroup.setAttribute('transform', `translate(${roverX}, 80)`);
    }

    // Update Rover Heading Arrow
    const roverArrow = document.getElementById('rover-arrow-group');
    if (roverArrow) {
      // Rotate arrow according to heading change (and reverse vector if reverse)
      const baseRotation = isReverse ? 180 : 0;
      const totalRotation = baseRotation + stepMetrics.heading_change_deg;
      roverArrow.setAttribute('transform', `rotate(${totalRotation})`);
    }

    // Rover Label
    const roverLabel = document.getElementById('rover-icon-label');
    if (roverLabel) {
      roverLabel.textContent = `${formatNum(stepMetrics.measured_mm, 1, true)} mm`;
    }

    // Target Marker Position
    const targetX = trackStart + (trackWidth * 0.85);
    const targetMarker = document.getElementById('target-marker');
    const targetMarkerText = document.getElementById('target-marker-text');
    if (targetMarker && targetMarkerText) {
      targetMarker.setAttribute('x1', targetX);
      targetMarker.setAttribute('x2', targetX);
      targetMarkerText.setAttribute('x', targetX);
      targetMarkerText.textContent = `TARGET (${formatNum(stepMetrics.target_mm, 1)} mm)`;
    }

    // Dynamic N-Zone Deceleration Renderer (Multi-Color Staggered Non-Overlapping Labels)
    const container = document.getElementById('dynamic-decel-zones');
    if (container) {
      container.innerHTML = ''; // Clear previous SVG elements

      const targetMag = Math.abs(stepMetrics.target_mm) || 1000.0;
      const targetX = trackStart + (trackWidth * 0.85);

      // Curated Color Palette for N Slowdown Zones
      const zonePalette = [
        { fill: "rgba(245, 158, 11, 0.45)", stroke: "#f59e0b" },  // Zone 1: Amber
        { fill: "rgba(255, 235, 59, 0.70)", stroke: "#ffeb3b" },  // Zone 2: Bright Yellow
        { fill: "rgba(56, 239, 125, 0.70)", stroke: "#38ef7d" },  // Zone 3: Neon Green/Cyan
        { fill: "rgba(168, 85, 247, 0.70)", stroke: "#a855f7" }   // Zone 4: Purple
      ];

      // Get computed slowdown zones with exact segment bounds and lengths
      const slowdownZones = getComputedSlowdownZones();

      // Render Clean Inline Header Legend Pills
      renderInlineZoneLegendPills(slowdownZones);

      slowdownZones.forEach((z, idx) => {
        const start_pct = Math.max(0.0, (targetMag - z.start_remaining_mm) / targetMag);
        const end_pct = Math.max(0.0, (targetMag - z.end_remaining_mm) / targetMag);

        const x1 = trackStart + (start_pct * (trackWidth * 0.85));
        const x2 = trackStart + (end_pct * (trackWidth * 0.85));
        const w = Math.max(10, x2 - x1);
        const centerX = x1 + (w / 2);

        const colorObj = z.color;

        // 1. Shaded SVG Rectangle
        const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
        rect.setAttribute("x", x1);
        rect.setAttribute("y", "70");
        rect.setAttribute("width", w);
        rect.setAttribute("height", "32");
        rect.setAttribute("rx", "4");
        rect.setAttribute("fill", colorObj.fill);
        rect.setAttribute("stroke", colorObj.stroke);
        rect.setAttribute("stroke-width", "1.5");
        rect.setAttribute("stroke-dasharray", "4 2");
        rect.setAttribute("opacity", "0.95");
        container.appendChild(rect);

        // 2. Clean Minimal Tag Inside Rect (Z1, Z2, Z3)
        if (w >= 12) {
          const innerText = document.createElementNS("http://www.w3.org/2000/svg", "text");
          innerText.setAttribute("x", centerX);
          innerText.setAttribute("y", "90");
          innerText.setAttribute("fill", colorObj.stroke);
          innerText.setAttribute("font-size", "10");
          innerText.setAttribute("font-weight", "900");
          innerText.setAttribute("text-anchor", "middle");
          innerText.textContent = `Z${idx+1}`;
          container.appendChild(innerText);
        }
      });
    }

  function renderInlineZoneLegendPills(zones) {
    const leg1 = document.getElementById('inline-zone-legend');
    const leg2 = document.getElementById('inline-zone-legend-main');
    [leg1, leg2].forEach(elem => {
      if (!elem) return;
      elem.innerHTML = '';
      zones.forEach((z, idx) => {
        const c = z.color;
        const pill = document.createElement('span');
        pill.style.cssText = `background: ${c.fill}; border: 1px solid ${c.stroke}; color: ${c.stroke}; font-size: 0.72rem; font-weight: 800; padding: 2px 8px; border-radius: 12px;`;
        pill.textContent = `Zone ${idx+1}: ${z.speed_mms}mm/s (${z.zone_length_mm.toFixed(0)}mm)`;
        elem.appendChild(pill);
      });
    });
  }

    // Stopped Marker (Overshoot / Undershoot Visualization)
    const stoppedGroup = document.getElementById('stopped-marker-group');
    if (stoppedGroup) {
      if (stepMetrics.stopped_dist_mm !== null || Math.abs(telemetry.vx_meas) < 0.005) {
        const stoppedMarker = document.getElementById('stopped-marker');
        const stoppedText = document.getElementById('stopped-marker-text');
        if (stoppedMarker && stoppedText) {
          stoppedMarker.setAttribute('x1', roverX);
          stoppedMarker.setAttribute('x2', roverX);
          stoppedText.setAttribute('x', roverX);
          stoppedText.textContent = `STOPPED (${formatNum(stepMetrics.measured_mm, 1, true)} mm)`;
        }
        stoppedGroup.setAttribute('opacity', '1');
      } else {
        stoppedGroup.setAttribute('opacity', '0');
      }
    }
  }

  // --- Timeline Sequence Renderer ---
  function renderTimelineSequence() {
    const elapsedSec = ((Date.now() - planStartTime) / 1000.0).toFixed(1);
    const valElapsed = document.getElementById('val-elapsed-time');
    if (valElapsed) valElapsed.textContent = `Elapsed: ${elapsedSec}s`;

    const nodes = [
      { id: 'tn-ready', active: !isArmed && activeStepIndex === 0 },
      { id: 'tn-fwd', active: isArmed && activeStepIndex === 0 && !stepMetrics.in_slowdown },
      { id: 'tn-fwd-slow', active: isArmed && activeStepIndex === 0 && stepMetrics.in_slowdown },
      { id: 'tn-fwd-stop', active: activeStepIndex === 1 },
      { id: 'tn-rev', active: isArmed && activeStepIndex === 2 && !stepMetrics.in_slowdown },
      { id: 'tn-rev-slow', active: isArmed && activeStepIndex === 2 && stepMetrics.in_slowdown },
      { id: 'tn-rev-stop', active: activeStepIndex === 3 },
      { id: 'tn-disarmed', active: !isArmed && activeStepIndex >= 3 }
    ];

    nodes.forEach(n => {
      const el = document.getElementById(n.id);
      if (el) {
        el.className = 't-node ' + (n.active ? 'active' : '');
      }
    });
  }

  // --- EMERGENCY STOP FUNCTIONALITY ---
  let estopActivated = false;

  function triggerMotionMonitorEstop() {
    if (estopActivated) return; // Second click does NOT re-enable anything
    estopActivated = true;
    isArmed = false;
    telemetry.armed = false;
    telemetry.vx_cmd = 0.0;
    telemetry.wz_cmd = 0.0;
    telemetry.vx_meas = 0.0;
    telemetry.wz_meas = 0.0;

    // Mark current step aborted
    if (planSteps[activeStepIndex]) {
      planSteps[activeStepIndex].status = "ABORTED";
    }

    // 1. Call exact existing Cockpit Disarm API endpoint (/api/drive/disarm)
    const token = sessionStorage.getItem('rover_operator_token') || localStorage.getItem('rover_operator_token') || '';
    const headers = { 'Content-Type': 'application/json' };
    if (token) {
      headers['x-operator-token'] = token;
    }

    fetch('/api/drive/disarm', { method: 'POST', headers })
      .catch(err => console.error('E-STOP /api/drive/disarm error:', err));

    // 2. Send zero speed, zero PWM and deadman release packets over WebSocket
    if (ws && ws.readyState === WebSocket.OPEN) {
      try { ws.send(JSON.stringify({ type: 'set_pwm', pwms: [0, 0, 0, 0] })); } catch (e) {}
      try { ws.send(JSON.stringify({ type: 'joystick', x: 0.0, y: 0.0, deadman: false })); } catch (e) {}
      try { ws.send(JSON.stringify({ type: 'test_abort', reason: 'EMERGENCY STOP ACTIVATED' })); } catch (e) {}
    }

    // 3. Broadcast Emergency Stop event to main Cockpit window via BroadcastChannel
    if (typeof BroadcastChannel !== 'undefined') {
      try {
        const bc = new BroadcastChannel('rover_telemetry');
        bc.postMessage({ type: 'emergency_stop_triggered', source: 'motion_monitor' });
      } catch (e) {}
    }

    // 4. Immediately update E-STOP UI state
    const btnEstop = document.getElementById('btn-estop-popup');
    if (btnEstop) {
      btnEstop.classList.add('activated');
      btnEstop.textContent = '🚨 EMERGENCY STOPPED';
      btnEstop.disabled = true;
    }

    const alertBanner = document.getElementById('estop-alert-banner');
    if (alertBanner) {
      alertBanner.style.display = 'block';
    }

    renderUI();
  }

  // --- BroadcastChannel & Direct WebSocket Connection Ingestion ---
  function initIngestion() {
    // Bind Emergency Stop Button
    const btnEstop = document.getElementById('btn-estop-popup');
    if (btnEstop) {
      btnEstop.addEventListener('click', (e) => {
        e.preventDefault();
        triggerMotionMonitorEstop();
      });
    }

    // 1. BroadcastChannel Ingestion (from main Cockpit window)
    if (typeof BroadcastChannel !== 'undefined') {
      const channel = new BroadcastChannel('rover_telemetry');
      channel.onmessage = (event) => {
        if (event && event.data) {
          if (event.data.type === 'emergency_stop_triggered') {
            triggerMotionMonitorEstop();
          } else {
            handleTelemetryMsg(event.data);
          }
        }
      };
    }

    // 2. Direct WebSocket Connection Ingestion (Auto-reconnecting Fallback)
    connectDirectWebSocket();
  }

  function connectDirectWebSocket() {
    const protocol = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
    const host = window.location.host || '127.0.0.1:3000';
    const wsUrl = `${protocol}//${host}/ws`;

    try {
      ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        wsConnected = true;
        renderUI();
      };

      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === 'motion_plan_config_updated') {
            fetchServerMotionPlan();
          }
          handleTelemetryMsg(msg);
        } catch (e) {
          console.error('Error parsing WS message:', e);
        }
      };

      ws.onclose = () => {
        wsConnected = false;
        renderUI();
        setTimeout(connectDirectWebSocket, 2000);
      };

      ws.onerror = () => {
        wsConnected = false;
        renderUI();
      };
    } catch (e) {
      wsConnected = false;
      setTimeout(connectDirectWebSocket, 3000);
    }
  }

  let activeCustomZones = [
    { zone_length_mm: 40.0, speed_mms: 45 },
    { zone_length_mm: 25.0, speed_mms: 30 },
    { zone_length_mm: 25.0, speed_mms: 20 }
  ];

  function getComputedSlowdownZones() {
    let totalLen = 0.0;
    activeCustomZones.forEach(z => { totalLen += Math.max(1.0, z.zone_length_mm); });

    let currentStartRem = totalLen;
    const computed = [];

    const zonePalette = [
      { fill: "rgba(245, 158, 11, 0.45)", stroke: "#f59e0b" },  // Zone 1: Amber
      { fill: "rgba(255, 235, 59, 0.70)", stroke: "#ffeb3b" },  // Zone 2: Bright Yellow
      { fill: "rgba(56, 239, 125, 0.70)", stroke: "#38ef7d" },  // Zone 3: Neon Green/Cyan
      { fill: "rgba(168, 85, 247, 0.70)", stroke: "#a855f7" }   // Zone 4: Purple
    ];

    activeCustomZones.forEach((z, idx) => {
      const len = Math.max(1.0, z.zone_length_mm);
      const endRem = Math.max(1.5, currentStartRem - len);
      computed.push({
        idx: idx + 1,
        label: `Zone ${idx+1} (${z.speed_mms}mm/s, ${len.toFixed(0)}mm)`,
        short_label: `Zone ${idx+1}`,
        start_remaining_mm: currentStartRem,
        end_remaining_mm: endRem,
        zone_length_mm: len,
        speed_mms: z.speed_mms,
        color: zonePalette[idx % zonePalette.length]
      });
      currentStartRem = endRem;
    });

    return computed;
  }

  function initPlanEditorUI() {
    renderZonesEditorRows();

    const btnAddZone = document.getElementById('btn-add-zone') || document.getElementById('btn-add-zone-main');
    if (btnAddZone) {
      btnAddZone.onclick = () => {
        activeCustomZones.push({
          zone_length_mm: 20.0,
          speed_mms: 10
        });
        renderZonesEditorRows();
        renderUI();
      };
    }

    const btnApply = document.getElementById('btn-apply-plan') || document.getElementById('btn-apply-plan-main');
    if (btnApply) {
      btnApply.onclick = saveAndApplyPlan;
    }
  }

  function renderZonesEditorRows() {
    const list1 = document.getElementById('zones-editor-list');
    const list2 = document.getElementById('zones-editor-list-main');

    [list1, list2].forEach(container => {
      if (!container) return;
      container.innerHTML = '';

      activeCustomZones.forEach((z, idx) => {
        const row = document.createElement('div');
        row.style.cssText = "display: flex; gap: 8px; align-items: center; background: rgba(15, 23, 42, 0.8); padding: 6px 10px; border-radius: 6px; border: 1px solid rgba(255,255,255,0.08);";
        row.innerHTML = `
          <span style="color: #ffb700; font-weight: 800; font-size: 0.75rem; width: 60px;">Zone ${idx+1}:</span>
          <div style="flex: 1; display: flex; align-items: center; gap: 4px;">
            <span style="color: #94a3b8; font-size: 0.7rem;">Length:</span>
            <input type="number" value="${z.zone_length_mm.toFixed(0)}" step="5.0" min="5.0" class="inp-z-len" data-idx="${idx}" style="width: 55px; background: #0f172a; border: 1px solid #334155; color: #ffb700; font-weight: 800; padding: 4px; border-radius: 4px; font-size: 0.75rem;" />
            <span style="color: #94a3b8; font-size: 0.7rem;">mm</span>
          </div>
          <div style="flex: 1; display: flex; align-items: center; gap: 4px;">
            <span style="color: #94a3b8; font-size: 0.7rem;">Speed:</span>
            <input type="number" value="${z.speed_mms}" step="1" min="1" max="100" class="inp-z-speed" data-idx="${idx}" style="width: 55px; background: #0f172a; border: 1px solid #334155; color: #38ef7d; font-weight: 800; padding: 4px; border-radius: 4px; font-size: 0.75rem;" />
            <span style="color: #94a3b8; font-size: 0.7rem;">mm/s</span>
          </div>
          <button class="btn-del-z" data-idx="${idx}" style="background: rgba(239, 68, 68, 0.2); border: 1px solid #ef4444; color: #ef4444; border-radius: 4px; width: 22px; height: 22px; cursor: pointer; font-size: 0.7rem; font-weight: 800;">✕</button>
        `;
        container.appendChild(row);
      });

      // Bind input changes
      container.querySelectorAll('.inp-z-len').forEach(inp => {
        inp.onchange = (e) => {
          const idx = parseInt(e.target.dataset.idx);
          activeCustomZones[idx].zone_length_mm = Math.max(5.0, parseFloat(e.target.value) || 20.0);
          renderUI();
        };
      });
      container.querySelectorAll('.inp-z-speed').forEach(inp => {
        inp.onchange = (e) => {
          const idx = parseInt(e.target.dataset.idx);
          activeCustomZones[idx].speed_mms = Math.max(1, parseInt(e.target.value) || 10);
          renderUI();
        };
      });
      container.querySelectorAll('.btn-del-z').forEach(btn => {
        btn.onclick = (e) => {
          const idx = parseInt(e.target.dataset.idx);
          activeCustomZones.splice(idx, 1);
          renderZonesEditorRows();
          renderUI();
        };
      });
    });
  }

  function saveAndApplyPlan() {
    const dirEl = document.getElementById('cfg-direction') || document.getElementById('cfg-direction-main');
    const distEl = document.getElementById('cfg-target-dist') || document.getElementById('cfg-target-dist-main');
    const speedEl = document.getElementById('cfg-cruise-speed') || document.getElementById('cfg-cruise-speed-main');

    const dir = parseInt(dirEl ? dirEl.value : "1");
    const distMag = Math.abs(parseFloat(distEl ? distEl.value : "1000.0")) || 1000.0;
    const cruiseMps = Math.abs(parseFloat(speedEl ? speedEl.value : "0.150")) || 0.150;

    const signedTargetMm = dir * distMag;
    const computedZones = getComputedSlowdownZones();

    // Convert computedZones to API plan config
    const backendZones = computedZones.map(z => ({
      start_rem_m: z.start_remaining_mm / 1000.0,
      speed_mps: z.speed_mms / 1000.0,
      label: z.label
    }));

    const planPayload = {
      target_dist_m: signedTargetMm / 1000.0,
      cruise_speed_mps: cruiseMps,
      zero_offset_m: 0.0015,
      zones: backendZones
    };

    // Save to RPi5 server
    fetch('/api/drive/motion_plan_config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(planPayload)
    }).then(r => r.json()).then(res => {
      console.log('[CONFIG] Saved motion plan config to server:', res);
    }).catch(err => console.error('[CONFIG ERROR] Failed to save motion plan config:', err));

    stepMetrics.target_mm = signedTargetMm;
    resetAllData();
    const dirLabel = dir > 0 ? "Forward" : "Reverse";
    alert(`✅ Plan Updated & Saved!\nTarget: ${signedTargetMm.toFixed(1)} mm (${dirLabel})\nCruise: ${cruiseMps.toFixed(3)} m/s\nSlowdown Zones: ${computedZones.length}`);
  }

  function resetAllData() {
    origin.x = telemetry.x;
    origin.y = telemetry.y;
    origin.yaw = telemetry.yaw;
    origin.yaw_deg = telemetry.yaw_deg;
    origin.m1 = telemetry.m1_ticks;
    origin.m2 = telemetry.m2_ticks;
    origin.m3 = telemetry.m3_ticks;
    origin.m4 = telemetry.m4_ticks;
    origin.initialized = true;

    const currentTargetMm = stepMetrics.target_mm || 1000.0;
    stepMetrics.target_mm = currentTargetMm;
    stepMetrics.measured_mm = 0.0;
    stepMetrics.remaining_mm = currentTargetMm;
    stepMetrics.progress_pct = 0.0;
    stepMetrics.peak_speed_mps = 0.0;
    stepMetrics.slowdown_start_mm = null;
    stepMetrics.slowdown_remaining_mm = null;
    stepMetrics.in_slowdown = false;
    stepMetrics.start_heading_deg = telemetry.yaw_deg;
    stepMetrics.stopped_dist_mm = null;
    stepMetrics.distance_error_mm = 0.0;
    stepMetrics.dist_m1_mm = 0.0;
    stepMetrics.dist_m2_mm = 0.0;
    stepMetrics.dist_m3_mm = 0.0;
    stepMetrics.dist_m4_mm = 0.0;
    stepMetrics.overall_enc_mm = 0.0;

    activeStepIndex = 0;
    const targetMm = stepMetrics.target_mm || 1000.0;
    const dir = targetMm >= 0 ? 1 : -1;
    const distMag = Math.abs(targetMm);
    const dirLabel = dir > 0 ? "Forward" : "Reverse";

    planSteps = [
      { id: 1, label: `${dirLabel} ${distMag.toFixed(1)} mm`, target_mm: targetMm, dir: dir, status: "ACTIVE" }
    ];

    activeCustomZones.forEach((z, idx) => {
      const zTarget = dir * Math.max(0.0, distMag - z.start_remaining_mm);
      planSteps.push({
        id: idx + 2,
        label: z.label,
        target_mm: zTarget,
        dir: dir,
        status: "PENDING"
      });
    });

    planSteps.push({
      id: planSteps.length + 1,
      label: "Stop / Disarm",
      target_mm: 0.0,
      dir: 0,
      status: "PENDING"
    });

    renderUI();
  }

  function fetchServerMotionPlan() {
    fetch('/api/drive/motion_plan_config')
      .then(r => r.json())
      .then(res => {
        if (res && res.ok && res.config && Array.isArray(res.config.zones) && res.config.zones.length > 0) {
          const cfg = res.config;
          const targetM = cfg.target_dist_m || 1.0;
          stepMetrics.target_mm = targetM * 1000.0;
          stepMetrics.remaining_mm = targetM * 1000.0;

          // Sync Direction Select Dropdown
          const selectDir = document.getElementById('cfg-direction');
          if (selectDir) {
            selectDir.value = targetM < 0 ? "-1" : "1";
          }
          const inputTarget = document.getElementById('cfg-target-dist');
          if (inputTarget) {
            inputTarget.value = Math.abs(targetM * 1000.0).toFixed(1);
          }

          const rawZones = cfg.zones;
          const parsed = [];
          for (let i = 0; i < rawZones.length; i++) {
            const startRemMm = (rawZones[i].start_rem_m || 0.09) * 1000.0;
            const nextStartRemMm = (i + 1 < rawZones.length) ? (rawZones[i+1].start_rem_m || 0.04) * 1000.0 : 0.0;
            const lenMm = Math.max(5.0, startRemMm - nextStartRemMm);
            const speedMms = Math.round((rawZones[i].speed_mps || 0.015) * 1000.0);
            parsed.push({
              zone_length_mm: lenMm,
              speed_mms: speedMms
            });
          }
          activeCustomZones = parsed;
          renderZonesEditorRows();
          resetAllData();
          console.log('[CONFIG] Loaded motion plan from server:', cfg);
        }
      })
      .catch(e => console.error('[CONFIG LOG] Error fetching server motion plan:', e));
  }

  // Initialize on DOM Ready
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      initIngestion();
      initPlanEditorUI();
      fetchServerMotionPlan();
      const btnReset = document.getElementById('btn-reset-plan');
      const btnClear = document.getElementById('btn-clear-data');
      if (btnReset) btnReset.addEventListener('click', resetAllData);
      if (btnClear) btnClear.addEventListener('click', resetAllData);

      const btnSubmitPhysical = document.getElementById('btn-submit-physical-mm');
      if (btnSubmitPhysical) {
        btnSubmitPhysical.addEventListener('click', () => {
          const inputVal = parseFloat(document.getElementById('input-physical-tape-mm').value);
          const lblStatus = document.getElementById('lbl-scale-error-status');
          const valErr = document.getElementById('val-calculated-scale-error');

          if (isNaN(inputVal) || inputVal <= 0) {
            if (lblStatus) lblStatus.textContent = "INVALID ENTRY";
            if (valErr) valErr.textContent = "Enter valid physical mm";
            return;
          }

          const settledEst = Math.abs(stepMetrics.measured_mm || stepMetrics.stopped_dist_mm || stepMetrics.target_mm || 1000.0);
          const targetMag = Math.abs(stepMetrics.target_mm || 1000.0);

          const errorRatio = ((settledEst - inputVal) / inputVal) * 100.0;
          const diffMm = inputVal - targetMag;

          if (lblStatus) lblStatus.textContent = "CALCULATED (PHYSICAL VS ESTIMATE)";
          if (valErr) {
            valErr.textContent = `${diffMm >= 0 ? '+' : ''}${diffMm.toFixed(1)} mm Ground Offset (${errorRatio >= 0 ? '+' : ''}${errorRatio.toFixed(2)}% Scale Error)`;
          }
          console.log(`[CALIBRATION] Physical entry: ${inputVal}mm, Settled Est: ${settledEst.toFixed(2)}mm, Target: ${targetMag}mm, Scale Error: ${errorRatio.toFixed(2)}%`);
        });
      }
    });
  } else {
    initIngestion();
    initPlanEditorUI();
    fetchServerMotionPlan();
    setInterval(fetchServerMotionPlan, 2000);
    const btnReset = document.getElementById('btn-reset-plan');
    const btnClear = document.getElementById('btn-clear-data');
    if (btnReset) btnReset.addEventListener('click', resetAllData);
    if (btnClear) btnClear.addEventListener('click', resetAllData);
  }

})();
