'use strict';
/**
 * Rover 1 Front Obstacle Safety Zone (ToF-backed Crawl & Stop Gate)
 *
 * Enforces a two-tier safety zone directly in front of the rover for BOTH
 * manual driving (BROWSER / GAMEPAD) and autonomous driving (ROS_AUTONOMY / CALIBRATION_TEST):
 *   - <= 50 mm (FRONT_CRAWL_THRESHOLD_MM): slows forward motion to a crawl (0.03 m/s)
 *   - <= 10 mm (FRONT_STOP_THRESHOLD_MM):  fully stops forward (and in-place) motion (0.0 m/s)
 *   - Reverse motion (vx < 0) remains allowed so the rover can back away from the obstacle.
 *
 * Authoritative Radar Proximity Indicator states:
 *   - PROXIMITY — OFFLINE    (gray,    level='offline'): sensor data unavailable or stale
 *   - PROXIMITY — CLEAR      (neutral, level='clear'):   valid sensor data shows no restriction (> 50 mm)
 *   - PROXIMITY — CRAWL ZONE (yellow,  level='warn'):    obstacle in 11..50 mm crawl zone, not actively reducing motion
 *   - PROXIMITY — SLOWING    (yellow,  level='warn'):    obstacle in 11..50 mm crawl zone, actively reducing commanded motion
 *   - PROXIMITY — STOP ZONE  (red,     level='bad'):     obstacle in <= 10 mm stop zone, stationary or reversing without restriction
 *   - PROXIMITY — STOP       (red,     level='bad'):     obstacle in <= 10 mm stop zone, actively blocking commanded motion
 */

const FRONT_STOP_THRESHOLD_MM = 10;
const FRONT_CRAWL_THRESHOLD_MM = 50;
const DEFAULT_CRAWL_LINEAR_MPS = 0.03;   // Calibrated Case 10 crawl speed (30 mm/s)
const DEFAULT_CRAWL_ANGULAR_RADPS = 0.35; // Calibrated nominal slow spin rate
const MAX_VALID_TOF_MM = 4000;
const DEFAULT_MAX_HOST_AGE_MS = 1500;
const DEFAULT_MAX_CHANNEL_AGE_MS = 1000;

const FRONT_SLOTS = ['front_left_l7cx', 'front_right_l7cx'];

/**
 * Extract the minimum valid distance (in mm, >= 0) from a single ToF multiplexer channel.
 * Returns null if the channel is absent, uninitialized, stale, or has no valid target in range.
 */
function extractChannelMinMm(ch, maxChannelAgeMs = DEFAULT_MAX_CHANNEL_AGE_MS) {
  if (!ch || !ch.present || !ch.init) return null;
  if (typeof ch.age_ms === 'number' && ch.age_ms > maxChannelAgeMs) return null;

  let bestMm = null;

  if (Array.isArray(ch.grid_mm) && ch.grid_mm.length > 0) {
    for (let i = 0; i < ch.grid_mm.length; i++) {
      const d = ch.grid_mm[i];
      if (typeof d === 'number' && isFinite(d) && d >= 0 && d <= MAX_VALID_TOF_MM) {
        if (bestMm === null || d < bestMm) bestMm = d;
      }
    }
  }

  if (bestMm === null && typeof ch.min_mm === 'number' && isFinite(ch.min_mm) && ch.min_mm >= 0 && ch.min_mm <= MAX_VALID_TOF_MM) {
    bestMm = ch.min_mm;
  }

  if (bestMm === null && typeof ch.dist_mm === 'number' && isFinite(ch.dist_mm) && ch.dist_mm >= 0 && ch.dist_mm <= MAX_VALID_TOF_MM) {
    bestMm = ch.dist_mm;
  }

  return bestMm;
}

/**
 * Clamp requested (linearMps, angularRadps) velocity according to the active Front Safety Zone
 * and compute authoritative intervention / indicator status.
 */
function applyFrontSafetyZoneToVelocity(linearMps, angularRadps, zoneStatus) {
  const lin = Number(linearMps) || 0;
  const ang = Number(angularRadps) || 0;

  if (!zoneStatus || zoneStatus.sensorLive === false || zoneStatus.state === 'OFFLINE') {
    return {
      linear: lin,
      angular: ang,
      clamped: false,
      intervening: false,
      intervention: 'NONE',
      action: 'NONE',
      state: 'OFFLINE',
      indicatorCode: 'OFFLINE',
      indicatorLevel: 'offline',
      indicatorText: 'PROXIMITY — OFFLINE',
      minMm: null
    };
  }

  if (zoneStatus.state === 'CLEAR') {
    return {
      linear: lin,
      angular: ang,
      clamped: false,
      intervening: false,
      intervention: 'NONE',
      action: 'NONE',
      state: 'CLEAR',
      indicatorCode: 'CLEAR',
      indicatorLevel: 'clear',
      indicatorText: 'PROXIMITY — CLEAR',
      minMm: zoneStatus.minMm !== undefined ? zoneStatus.minMm : null
    };
  }

  const crawlLin = typeof zoneStatus.crawlSpeedMps === 'number' ? zoneStatus.crawlSpeedMps : DEFAULT_CRAWL_LINEAR_MPS;
  const crawlAng = typeof zoneStatus.crawlAngularRadps === 'number' ? zoneStatus.crawlAngularRadps : DEFAULT_CRAWL_ANGULAR_RADPS;

  if (zoneStatus.state === 'STOP') {
    // Allow reverse motion (lin < -1e-4) so operator or autonomy can back away from the obstacle
    if (lin < -1e-4) {
      const clampedRevAng = Math.max(-crawlAng, Math.min(crawlAng, ang));
      const angClamped = Math.abs(clampedRevAng - ang) > 1e-6;
      return {
        linear: lin,
        angular: clampedRevAng,
        clamped: angClamped,
        intervening: angClamped,
        intervention: angClamped ? 'SLOWING' : 'NONE',
        action: 'REVERSE_ALLOWED',
        state: 'STOP',
        indicatorCode: angClamped ? 'SLOWING' : 'STOP_ZONE',
        indicatorLevel: angClamped ? 'warn' : 'bad',
        indicatorText: angClamped ? 'PROXIMITY — SLOWING' : 'PROXIMITY — STOP ZONE',
        minMm: zoneStatus.minMm
      };
    }
    // Any forward or in-place motion is fully stopped within 10 mm; stationary reports STOP ZONE
    const wasMoving = Math.abs(lin) > 1e-6 || Math.abs(ang) > 1e-6;
    return {
      linear: 0.0,
      angular: 0.0,
      clamped: wasMoving,
      intervening: wasMoving,
      intervention: wasMoving ? 'STOP' : 'NONE',
      action: wasMoving ? 'STOP' : 'STOP_ZONE',
      state: 'STOP',
      indicatorCode: wasMoving ? 'STOP' : 'STOP_ZONE',
      indicatorLevel: 'bad',
      indicatorText: wasMoving ? 'PROXIMITY — STOP' : 'PROXIMITY — STOP ZONE',
      minMm: zoneStatus.minMm
    };
  }

  if (zoneStatus.state === 'CRAWL') {
    if (lin > 1e-4) {
      const newLin = Math.min(lin, crawlLin);
      const scale = lin > crawlLin ? (newLin / lin) : 1.0;
      const scaledAng = ang * scale;
      const newAng = Math.max(-crawlAng, Math.min(crawlAng, scaledAng));
      const clamped = Math.abs(newLin - lin) > 1e-6 || Math.abs(newAng - ang) > 1e-6;
      return {
        linear: newLin,
        angular: newAng,
        clamped,
        intervening: clamped,
        intervention: clamped ? 'SLOWING' : 'NONE',
        action: clamped ? 'CRAWL' : 'CRAWL_MONITOR',
        state: 'CRAWL',
        indicatorCode: clamped ? 'SLOWING' : 'CRAWL_ZONE',
        indicatorLevel: 'warn',
        indicatorText: clamped ? 'PROXIMITY — SLOWING' : 'PROXIMITY — CRAWL ZONE',
        minMm: zoneStatus.minMm
      };
    }
    if (Math.abs(lin) <= 1e-4 && Math.abs(ang) > crawlAng) {
      const newAng = Math.max(-crawlAng, Math.min(crawlAng, ang));
      return {
        linear: 0.0,
        angular: newAng,
        clamped: true,
        intervening: true,
        intervention: 'SLOWING',
        action: 'CRAWL',
        state: 'CRAWL',
        indicatorCode: 'SLOWING',
        indicatorLevel: 'warn',
        indicatorText: 'PROXIMITY — SLOWING',
        minMm: zoneStatus.minMm
      };
    }
    return {
      linear: lin,
      angular: ang,
      clamped: false,
      intervening: false,
      intervention: 'NONE',
      action: 'CRAWL_MONITOR',
      state: 'CRAWL',
      indicatorCode: 'CRAWL_ZONE',
      indicatorLevel: 'warn',
      indicatorText: 'PROXIMITY — CRAWL ZONE',
      minMm: zoneStatus.minMm
    };
  }

  return {
    linear: lin,
    angular: ang,
    clamped: false,
    intervening: false,
    intervention: 'NONE',
    action: 'NONE',
    state: zoneStatus.state || 'CLEAR',
    indicatorCode: 'CLEAR',
    indicatorLevel: 'clear',
    indicatorText: 'PROXIMITY — CLEAR',
    minMm: zoneStatus.minMm
  };
}

/**
 * Evaluate the front safety zone state from the ESP32-S3 ToF state payload and slot mapping.
 */
function evaluateFrontSafetyZone(tofData, mapping = {}, opts = {}) {
  const nowMs = typeof opts.nowMs === 'number' ? opts.nowMs : Date.now();
  const stopThresholdMm = typeof opts.stopThresholdMm === 'number' ? opts.stopThresholdMm : FRONT_STOP_THRESHOLD_MM;
  const crawlThresholdMm = typeof opts.crawlThresholdMm === 'number' ? opts.crawlThresholdMm : FRONT_CRAWL_THRESHOLD_MM;
  const crawlSpeedMps = typeof opts.crawlSpeedMps === 'number' ? opts.crawlSpeedMps : DEFAULT_CRAWL_LINEAR_MPS;
  const crawlAngularRadps = typeof opts.crawlAngularRadps === 'number' ? opts.crawlAngularRadps : DEFAULT_CRAWL_ANGULAR_RADPS;
  const maxHostAgeMs = typeof opts.maxHostAgeMs === 'number' ? opts.maxHostAgeMs : DEFAULT_MAX_HOST_AGE_MS;
  const maxChannelAgeMs = typeof opts.maxChannelAgeMs === 'number' ? opts.maxChannelAgeMs : DEFAULT_MAX_CHANNEL_AGE_MS;
  const reqLin = Number(opts.requestedLinearMps) || 0;
  const reqAng = Number(opts.requestedAngularRadps) || 0;

  const offlineResult = (hostAgeMs = null) => ({
    state: 'OFFLINE',
    active: false,
    sensorLive: false,
    intervening: false,
    intervention: 'NONE',
    action: 'NONE',
    indicatorCode: 'OFFLINE',
    indicatorLevel: 'offline',
    indicatorText: 'PROXIMITY — OFFLINE',
    minMm: null,
    leftMinMm: null,
    rightMinMm: null,
    triggeredBy: null,
    requestedLinearMps: reqLin,
    requestedAngularRadps: reqAng,
    effectiveLinearMps: reqLin,
    effectiveAngularRadps: reqAng,
    stopThresholdMm,
    crawlThresholdMm,
    crawlSpeedMps,
    crawlAngularRadps,
    hostAgeMs,
    updatedAt: nowMs
  });

  if (!tofData || !Array.isArray(tofData.channels)) {
    return offlineResult(null);
  }

  const hostTs = Number(tofData.host_timestamp_ms) || 0;
  const hostAgeMs = hostTs > 0 ? Math.max(0, nowMs - hostTs) : null;
  if (hostAgeMs === null || hostAgeMs > maxHostAgeMs) {
    return offlineResult(hostAgeMs);
  }

  const slotMins = {};
  let anySensorLive = false;

  for (const slot of FRONT_SLOTS) {
    const chIdx = mapping && typeof mapping[slot] === 'number' ? mapping[slot] : null;
    if (chIdx === null) {
      slotMins[slot] = null;
      continue;
    }
    const ch = tofData.channels.find(c => c && c.ch === chIdx);
    if (ch && ch.present && ch.init && (typeof ch.age_ms !== 'number' || ch.age_ms <= maxChannelAgeMs)) {
      anySensorLive = true;
    }
    slotMins[slot] = extractChannelMinMm(ch, maxChannelAgeMs);
  }

  if (!anySensorLive) {
    return offlineResult(hostAgeMs);
  }

  const leftMinMm = slotMins.front_left_l7cx !== undefined ? slotMins.front_left_l7cx : null;
  const rightMinMm = slotMins.front_right_l7cx !== undefined ? slotMins.front_right_l7cx : null;

  let minMm = null;
  let triggeredBy = null;
  if (leftMinMm !== null && rightMinMm !== null) {
    minMm = Math.min(leftMinMm, rightMinMm);
    if (leftMinMm <= crawlThresholdMm && rightMinMm <= crawlThresholdMm) {
      triggeredBy = leftMinMm === rightMinMm ? 'both' : (leftMinMm < rightMinMm ? 'front_left_l7cx' : 'front_right_l7cx');
    } else if (leftMinMm < rightMinMm) {
      triggeredBy = 'front_left_l7cx';
    } else {
      triggeredBy = 'front_right_l7cx';
    }
  } else if (leftMinMm !== null) {
    minMm = leftMinMm;
    triggeredBy = 'front_left_l7cx';
  } else if (rightMinMm !== null) {
    minMm = rightMinMm;
    triggeredBy = 'front_right_l7cx';
  }

  let state = 'CLEAR';
  if (minMm !== null && minMm <= stopThresholdMm) {
    state = 'STOP';
  } else if (minMm !== null && minMm <= crawlThresholdMm) {
    state = 'CRAWL';
  } else {
    triggeredBy = null;
  }

  const velEval = applyFrontSafetyZoneToVelocity(reqLin, reqAng, {
    state,
    sensorLive: true,
    minMm,
    crawlSpeedMps,
    crawlAngularRadps
  });

  return {
    state,
    active: state === 'CRAWL' || state === 'STOP',
    sensorLive: true,
    intervening: velEval.intervening,
    intervention: velEval.intervention,
    action: velEval.action,
    indicatorCode: velEval.indicatorCode,
    indicatorLevel: velEval.indicatorLevel,
    indicatorText: velEval.indicatorText,
    minMm,
    leftMinMm,
    rightMinMm,
    triggeredBy,
    requestedLinearMps: reqLin,
    requestedAngularRadps: reqAng,
    effectiveLinearMps: velEval.linear,
    effectiveAngularRadps: velEval.angular,
    stopThresholdMm,
    crawlThresholdMm,
    crawlSpeedMps,
    crawlAngularRadps,
    hostAgeMs,
    updatedAt: nowMs
  };
}

module.exports = {
  FRONT_STOP_THRESHOLD_MM,
  FRONT_CRAWL_THRESHOLD_MM,
  DEFAULT_CRAWL_LINEAR_MPS,
  DEFAULT_CRAWL_ANGULAR_RADPS,
  FRONT_SLOTS,
  extractChannelMinMm,
  evaluateFrontSafetyZone,
  applyFrontSafetyZoneToVelocity
};
