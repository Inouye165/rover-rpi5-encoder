# Cockpit Staged Migration Plan & Testing Strategy

**Repository:** `C:\Users\Ron\electronic_projects\yahboom-encoder`  
**Branch:** `feature/ros2-encoder-odom-tf`  
**Date:** July 26, 2026  
**Stage:** Stage 1 — Inventory & Analysis  

---

## 1. Staged Implementation Plan (Stages 2 – 9)

The cockpit cleanup and reorganization will follow an 8-stage migration sequence to eliminate risk, maintain continuous motor safety, and guarantee that no tests or live functionality break during transition.

```
Stage 1 (Current)  → Stage 2 Shell  → Stage 3 Read-Only → Stage 4 Drive/Safety
  (Inventory Plan)   (New 5 Tabs)     (Status Panels)     (Manual Driving)
          ↓
Stage 5 Calib UI   → Stage 6 Maint  → Stage 7 Hide Tabs → Stage 8 Dead Code → Stage 9 Backend
  (Consolidate)      (Advanced)       (Verify Parity)     (Frontend Cleanup)  (Deprecate)
```

---

### Stage 2: Create Navigation Shell & Destination Containers
- **Goal:** Introduce the new 5-tab top-level navigation structure (`Drive`, `Autonomy`, `Sensors`, `Calibration`, `Diagnostics`) while preserving all legacy tab containers and DOM IDs completely intact in the DOM.
- **Files Affected:** `public/index.html`, `public/app.js`, `public/style.css`
- **Risk Level:** **Very Low**
- **Motor Safety Concerns:** None (No movement or control logic modified).
- **Automated Tests Required:** `python validate_html.py`, `python validate_tabs.py`, `python test_dom_contract.py`
- **Live Verification Required:** Click each of the 5 new tabs to confirm tab switching without JavaScript console errors.
- **Rollback Strategy:** Revert `index.html` and `app.js` to pre-Stage 2 git commit.

---

### Stage 3: Move Read-Only Status & Diagnostic Panels
- **Goal:** Relocate read-only status components to their target tabs:
  - Move ROS 2 Stack Health & Service Badges to **Tab 2: Autonomy**
  - Move 360° Polar Display & LiDAR Status to **Tab 3: Sensors**
  - Move 3D Rover Attitude & IMU Sensor Readouts to **Tab 3: Sensors**
  - Move 2D Position Canvas & Wheel Telemetry to **Tab 3: Sensors**
- **Files Affected:** `public/index.html`, `public/app.js`
- **Risk Level:** **Low**
- **Motor Safety Concerns:** None (Read-only status elements only).
- **Automated Tests Required:** `python test_dom_contract.py`, `node test_lidar.js`
- **Live Verification Required:** Confirm telemetry, LiDAR polar plot, IMU 3D model, and ROS health badges update smoothly in real-time.
- **Rollback Strategy:** Move HTML elements back to legacy container divs.

---

### Stage 4: Move Manual Drive & Safety Controls
- **Goal:** Relocate driving control room elements to **Tab 1: Drive**:
  - Main Arm/Disarm buttons (`btn-arm-drive`, `btn-disarm-drive`)
  - Gamepad Live Input Status HUD
  - Direction D-Pad & Speed Sliders
  - Emergency Stop Button (`btn-estop`)
  - RPi Camera Feed
  - Single canonical Path Backtracking control panel
- **Files Affected:** `public/index.html`, `public/app.js`
- **Risk Level:** **Moderate**
- **Motor Safety Concerns:** High focus on ensuring Gamepad Deadman switch, Arm/Disarm state machine, and E-Stop functionality remain 100% operational.
- **Automated Tests Required:** `python test_maintenance_safety.py`, `python test_dom_contract.py`
- **Live Verification Required:**
  1. Verify Gamepad connects and live input HUD updates.
  2. Verify Arm / Disarm toggle operates.
  3. Verify E-Stop immediately halts motor outputs.
- **Rollback Strategy:** Revert drive panel markup to legacy dashboard container.

---

### Stage 5: Consolidate Calibration & Repeatability UI
- **Goal:** Unify all physical commissioning tools under **Tab 4: Calibration**:
  - Distance Calibration (2-Meter) Card
  - Rotation Calibration (360°) Card
  - Automatic Closed-Loop Tests (1m Forward, 90° Left, 90° Right)
  - Phase 5 Software Repeatability Verification Panel & Export Tools
- **Files Affected:** `public/index.html`, `public/app.js`
- **Risk Level:** **Moderate**
- **Motor Safety Concerns:** Confirm that the Auto Calibration Confirmation Modal (`#modal-auto-calib-confirm`) triggers before any automated physical floor test.
- **Automated Tests Required:** `python test_auto_calibration.py`, `python test_button_binding.py`, `node test_repeatability_sim.js`
- **Live Verification Required:** Run 1m forward auto-calibration test in elevated state; verify modal prompt and E-stop abort functionality.
- **Rollback Strategy:** Revert calibration panel containers to legacy `tab-motion-cal` and `tab-calibrate`.

---

### Stage 6: Move Maintenance & Advanced Tools
- **Goal:** Relocate specialized maintenance and developer tools:
  - Place Phase 3 Raised-Wheel Maintenance Panel behind a collapsed "Advanced Maintenance" toggle in **Tab 4: Calibration**.
  - Place LiDAR Straight-Line Correction & Orientation Wizard behind a collapsed "Advanced Commissioning" toggle in **Tab 4: Calibration**.
  - Move Raw Driver Logs, Raw Serial Hex Prompt, and Firmware Identity to **Tab 5: Diagnostics**.
- **Files Affected:** `public/index.html`, `public/app.js`
- **Risk Level:** **Low**
- **Motor Safety Concerns:** Verify raised-wheel maintenance safety checklist (`maint-safety-chk`) must still be acknowledged before single-motor test buttons (`btn-m1-fwd`..`btn-m4-rev`) activate.
- **Automated Tests Required:** `python test_maintenance_safety.py`, `python test_dom_contract.py`
- **Live Verification Required:** Expand Advanced Maintenance section; verify raised-wheel acknowledgment unlocks motor buttons and 2.0s auto-stop works.
- **Rollback Strategy:** Expand advanced sections by default or revert container placement.

---

### Stage 7: Hide Legacy Tabs After Parity Verification
- **Goal:** Hide legacy tab buttons (`tab-dashboard`, `tab-imu`, `tab-encoder`, `tab-ros2`, `tab-calibrate`, `tab-motion-cal`, `tab-lidar`) from top navigation bar after confirming complete feature parity across the 5 new tabs.
- **Files Affected:** `public/index.html`
- **Risk Level:** **Low**
- **Motor Safety Concerns:** None (Code remains in DOM, only CSS navigation buttons hidden).
- **Automated Tests Required:** Full test suite execution:
  - `node --check server.js`
  - `node --check public/app.js`
  - `python validate_html.py`
  - `python validate_tabs.py`
  - `python test_dom_contract.py`
  - `python test_auto_calibration.py`
  - `python test_button_binding.py`
  - `python test_maintenance_safety.py`
- **Live Verification Required:** Perform complete user end-to-end walkthrough of all 5 new tabs on live rover.
- **Rollback Strategy:** Re-enable legacy tab buttons in navigation bar.

---

### Stage 8: Remove Dead Frontend Code
- **Goal:** Remove orphaned frontend HTML markup (such as Breakaway Simulation panel and Yahboom Flash config form) and unused JavaScript helper functions only after full dependency and test confirmation.
- **Files Affected:** `public/index.html`, `public/app.js`
- **Risk Level:** **Low**
- **Motor Safety Concerns:** Ensure no active safety handlers or DOM IDs are accidentally deleted.
- **Automated Tests Required:** `python test_dom_contract.py` (guarantees zero missing or unguarded DOM IDs).
- **Live Verification Required:** Run browser console audit; verify 0 JavaScript errors or missing element warnings during runtime.
- **Rollback Strategy:** Git revert Stage 8 commit.

---

### Stage 9: Review Backend Endpoints for Deprecation
- **Goal:** Audit `server.js` endpoints for deprecation. Mark obsolete simulation endpoints (`/api/calibration/simulate/*`) as deprecated or archive them safely while preserving all active APIs (`/api/drive/*`, `/api/maintenance/*`, `/api/calibration/auto/*`, `/api/path/*`).
- **Files Affected:** `server.js`
- **Risk Level:** **Low**
- **Motor Safety Concerns:** Ensure core motor endpoints and safety handlers are never modified.
- **Automated Tests Required:** `node --check server.js`, `python test_maintenance_safety.py`, `python test_auto_calibration.py`
- **Live Verification Required:** Restart server; verify WebSocket broadcasts and REST endpoints respond normally.
- **Rollback Strategy:** Git revert Stage 9 commit.

---

## 2. Comprehensive Testing Strategy

### A. Pre-Implementation Test Design
Before starting UI modification, the test suite must be enhanced to enforce the following invariant safety contracts:

1. **Unique DOM ID Guarantee:** Verify no duplicate `id="..."` attributes exist in `index.html`.
2. **Control Singularity:** Ensure motor control buttons (e.g. Backtracking, E-Stop) exist exactly once in the visible UI.
3. **Safety Confirmation Persistence:** Verify that motor movement actions (Auto 1m, Auto 90°, Arm Drive) retain confirmation modals/switches.
4. **Global E-Stop Accessibility:** Verify `btn-estop` is present and visible across all tab navigation states.
5. **No Double Submissions:** Verify test trigger buttons disable themselves immediately upon click until backend response is received.
6. **Canonical Data Consistency:** Verify UI readouts pull strictly from canonical backend WebSocket broadcasts.
7. **Collapsed Section Integrity:** Verify hidden/collapsed panels (e.g. Advanced Maintenance) continue to receive and update telemetry in the background without throwing DOM errors.

---

### B. Existing Tests & Failure Analysis Across Stages

| Test Script | Tested Features / Contracts | Potential Failure Point During Migration | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| `validate_html.py` | Validates HTML syntax & unclosed tags | Missing tag during HTML restructuring | Run `validate_html.py` after every single file edit. |
| `validate_tabs.py` | Checks `tab-*` ID depths and end tags | Adding new `tab-*` containers | Update parser to recognize new 5-tab container IDs (`tab-drive`, `tab-autonomy`, `tab-sensors`, `tab-calibration`, `tab-diagnostics`). |
| `test_dom_contract.py` | Verifies all DOM IDs accessed by `app.js` exist in `index.html` or are null-safe | Renaming or deleting an HTML `id` while `app.js` still references it | Retain exact DOM IDs during panel relocation; do not rename IDs until Stage 8. |
| `test_button_binding.py` | Verifies `btn-auto-fwd-1m` invokes `promptAutoCalib()` modal flow | Relocating auto-calibration buttons | Ensure button ID and `onclick` attribute remain identical when moved to **Tab 4: Calibration**. |
| `test_maintenance_safety.py` | Enforces 8 maintenance buttons, STOP ALL button, and raised-wheel safety checklist | Moving maintenance panel behind Advanced collapsible section | Maintain exact element IDs (`btn-m1-fwd`..`btn-m4-rev`, `btn-stop-all-maint`) inside the collapsed container. |
| `test_auto_calibration.py` | Verifies closed-loop auto-calib safety rules in `server.js` | Modifying backend auto-calib handlers | Do not modify `server.js` auto-calibration state machine during UI reorganization. |
| `test_repeatability_sim.js` | Tests repeatability statistics calculations | Relocating repeatability panel | Ensure `/api/calibration/repeatability/*` endpoints and table element IDs remain unchanged. |

---

### C. Proposed New Test Suites

1. **`test_navigation_structure.py`:**
   - Verifies that `index.html` contains exactly the 5 new top-level tabs (`tab-drive`, `tab-autonomy`, `tab-sensors`, `tab-calibration`, `tab-diagnostics`).
   - Asserts that every tab button has a matching container div.

2. **`test_panel_ownership.py`:**
   - Maps every panel to its single canonical tab parent container.
   - Asserts zero duplicate panel IDs exist across separate tabs.

3. **`test_global_safety_banner.py`:**
   - Asserts that `btn-estop`, `serial-status`, and `ws-status` remain visible in the fixed header regardless of active tab selection.

4. **`test_advanced_section_behavior.py`:**
   - Verifies that collapsible `<details>` / accordion containers for Advanced Maintenance and Advanced Commissioning properly toggle visibility without breaking DOM updates.

5. **`test_browser_console_cleanliness.js`:**
   - Headless browser test using Puppeteer/Playwright to navigate through all 5 tabs and trigger non-moving UI controls, asserting zero `TypeError` or `Uncaught ReferenceError` exceptions in the console log.
