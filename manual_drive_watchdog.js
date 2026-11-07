'use strict';
/**
 * Manual-drive watchdog for browser / gamepad joystick commands.
 *
 * Two independent protections:
 *  1. Socket-close: if the WebSocket that last commanded manual motion closes, the
 *     server must zero the manual target immediately (previously it only logged).
 *  2. Opt-in keepalive: a client that sends `keepalive: true` on its joystick messages
 *     promises to stream at >= ~2 Hz. If no message arrives within `timeoutMs`, the
 *     manual target is zeroed. Legacy clients (old app.js sends once per keydown) do not
 *     set the flag and keep their existing behaviour.
 */
class ManualDriveWatchdog {
  constructor(opts = {}) {
    this.timeoutMs = opts.timeoutMs || 600;
    this.now = opts.now || (() => Date.now());
    this.owner = null;
    this.deadline = 0;
    this.expiredCount = 0;
    this.socketCloseCount = 0;
  }

  /** Record a manual joystick command from `owner` (usually the ws object). */
  noteCommand(owner, keepalive) {
    this.owner = owner || null;
    this.deadline = keepalive === true ? this.now() + this.timeoutMs : 0;
  }

  /** @returns {boolean} true exactly once when an opted-in stream has gone stale. */
  checkExpired() {
    if (this.deadline > 0 && this.now() > this.deadline) {
      this.deadline = 0;
      this.expiredCount++;
      return true;
    }
    return false;
  }

  /** @returns {boolean} true if the closing socket was the active manual commander. */
  socketClosed(owner) {
    if (owner && owner === this.owner) {
      this.owner = null;
      this.deadline = 0;
      this.socketCloseCount++;
      return true;
    }
    return false;
  }

  clear() {
    this.owner = null;
    this.deadline = 0;
  }

  status() {
    return {
      timeoutMs: this.timeoutMs,
      keepaliveArmed: this.deadline > 0,
      expiredCount: this.expiredCount,
      socketCloseCount: this.socketCloseCount
    };
  }
}

module.exports = { ManualDriveWatchdog };
