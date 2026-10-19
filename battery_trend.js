'use strict';
/**
 * Battery state-of-charge trend estimator for the Geekworm X1206 UPS.
 *
 * The X1206 fuel gauge on Rover 1 behaves like a MAX17043 (no CRATE register) and the
 * board's AC-present line is not usable (GPIO6 is driven as an output), so charging /
 * discharging is inferred from the SOC slope — the same approach Rover 2 uses.
 *
 * Least-squares slope over the last `windowMs` of samples, reported in %/hour.
 */
class BatteryTrend {
  constructor(opts = {}) {
    this.windowMs = opts.windowMs || 20 * 60 * 1000;
    this.minSpanMs = opts.minSpanMs || 5 * 60 * 1000;
    this.minSamples = opts.minSamples || 8;
    this.chargeThreshold = opts.chargeThreshold || 1.0;   // %/hr
    this.fullSoc = opts.fullSoc || 95.0;
    this.samples = [];
  }

  add(percentage, tMs = Date.now()) {
    if (typeof percentage !== 'number' || !isFinite(percentage)) return;
    this.samples.push({ t: tMs, p: percentage });
    const cutoff = tMs - this.windowMs;
    while (this.samples.length && this.samples[0].t < cutoff) this.samples.shift();
  }

  spanMs() {
    if (this.samples.length < 2) return 0;
    return this.samples[this.samples.length - 1].t - this.samples[0].t;
  }

  /** @returns {number|null} slope in %/hr, or null when there is not enough history */
  ratePctPerHr() {
    const n = this.samples.length;
    if (n < this.minSamples || this.spanMs() < this.minSpanMs) return null;
    const t0 = this.samples[0].t;
    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const s of this.samples) {
      const x = (s.t - t0) / 3600000; // hours
      sx += x; sy += s.p; sxx += x * x; sxy += x * s.p;
    }
    const den = n * sxx - sx * sx;
    if (Math.abs(den) < 1e-12) return null;
    return (n * sxy - sx * sy) / den;
  }

  snapshot() {
    const rate = this.ratePctPerHr();
    const last = this.samples.length ? this.samples[this.samples.length - 1].p : null;
    let powerState = 'UNKNOWN';
    let minutesRemaining = null;
    if (rate !== null && last !== null) {
      if (last >= this.fullSoc && rate > -this.chargeThreshold) {
        powerState = 'FULL';
      } else if (rate > this.chargeThreshold) {
        powerState = 'CHARGING';
        minutesRemaining = ((100 - last) / rate) * 60;
      } else if (rate < -this.chargeThreshold) {
        powerState = 'DISCHARGING';
        minutesRemaining = (last / -rate) * 60;
      } else {
        powerState = 'STEADY';
      }
    }
    return {
      ratePctPerHr: rate === null ? null : Math.round(rate * 100) / 100,
      powerState,
      minutesRemaining: minutesRemaining === null ? null : Math.round(minutesRemaining),
      trendWindowMin: Math.round(this.spanMs() / 600) / 100,
      trendSamples: this.samples.length
    };
  }
}

module.exports = { BatteryTrend };
