/**
 * maker_esp32_pro.ino
 * Firmware for the emakefun/NULLLAB Maker ESP32 Pro board.
 * Emulates the Yahboom ROS Expansion Board binary serial protocol
 * while hosting a standalone WiFi WebServer for direct control!
 *
 * Requirements:
 *  - ESP32 Arduino Core
 *  - Board Select: "ESP32 Dev Module" (or similar ESP32-WROOM-32E board)
 *  - DIP Switch: MUST set the "Motor/IO Switch" to "Motor" to enable M2 and M3.
 *  - Power: Connect a 6V-16V battery/power source to the DC Barrel Jack.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED Scrolling state
bool oledOk = false;
String scrollText = "";
int scrollIndex = 0;
unsigned long lastScrollTime = 0;

// ────────────────────────────────────────────────────────────
// WiFi & WebServer Settings
// ────────────────────────────────────────────────────────────
const char* ssid = "Dobby";
const char* password = "sanmina-1";

WebServer server(80);

// ────────────────────────────────────────────────────────────
// GPIO Pin Mapping
// ────────────────────────────────────────────────────────────

// DC Motor pins (IN1, IN2)
const int M1_IN1 = 27;
const int M1_IN2 = 13;
const int M2_IN1 = 4;
const int M2_IN2 = 2;
const int M3_IN1 = 17; // Switch must be in "Motor" position
const int M3_IN2 = 12; // Switch must be in "Motor" position
const int M4_IN1 = 14; // Switch must be in "Motor" position
const int M4_IN2 = 15; // Switch must be in "Motor" position

// Encoder pins (A, B)
const int E1_A = 18;
const int E1_B = 19;
const int E2_A = 5;
const int E2_B = 23;
const int E3_A = 35; // Input only
const int E3_B = 36; // Input only
const int E4_A = 34; // Input only
const int E4_B = 39; // Input only

// Onboard status LED
const int STATUS_LED = -1; // Set to -1 to avoid conflict with M2_IN2 (GPIO 2) on Maker ESP32 Pro

// ────────────────────────────────────────────────────────────
// Global Variables & Telemetry State
// ────────────────────────────────────────────────────────────

// Motor target speeds (-100 to 100)
volatile int targetSpeedM1 = 0;
volatile int targetSpeedM2 = 0;
volatile int targetSpeedM3 = 0;
volatile int targetSpeedM4 = 0;

// Encoder tick counts
volatile int32_t encoderTicksM1 = 0;
volatile int32_t encoderTicksM2 = 0;
volatile int32_t encoderTicksM3 = 0;
volatile int32_t encoderTicksM4 = 0;

// Simulated Battery Voltage
volatile float simulatedVoltage = 12.6; // Start with a full 3S battery (12.6V)

// Speed/RPM calculation state
unsigned long lastSpeedCalcTime = 0;
int32_t prevTicksM1 = 0, prevTicksM2 = 0, prevTicksM3 = 0, prevTicksM4 = 0;
float rpmM1 = 0.0, rpmM2 = 0.0, rpmM3 = 0.0, rpmM4 = 0.0;
float mphM1 = 0.0, mphM2 = 0.0, mphM3 = 0.0, mphM4 = 0.0;

// Speed PID (PI Velocity Controller) configuration & state
unsigned long lastPidTime = 0;
int32_t prevPidTicksM1 = 0, prevPidTicksM2 = 0, prevPidTicksM3 = 0, prevPidTicksM4 = 0;
float errorSumM1 = 0.0, errorSumM2 = 0.0, errorSumM3 = 0.0, errorSumM4 = 0.0;

const float MAX_TICKS_50MS = 257.0; // max physical ticks per 50ms interval (at 100% PWM)
const float KP_SPEED = 0.5;
const float KI_SPEED = 0.15;

// Cross-Coupling Synchronization Gain (set to 0.0 to disable synchronization)
const float KC_SYNC = 0.30;

float getSign(float val) {
  if (val > 0) return 1.0;
  if (val < 0) return -1.0;
  return 0.0;
}

// Keep track of communication activity and safety timeout
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 1000; // stop motors if no commands for 1s

// Timer for sending telemetry
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // 20 Hz stream rate

// ────────────────────────────────────────────────────────────
// Encoder Interrupt Service Routines (ISRs)
// ────────────────────────────────────────────────────────────
// Target position state for each motor (in ticks)
volatile int32_t targetPositionM1 = 0;
volatile int32_t targetPositionM2 = 0;
volatile int32_t targetPositionM3 = 0;
volatile int32_t targetPositionM4 = 0;

volatile bool positionModeM1 = false;
volatile bool positionModeM2 = false;
volatile bool positionModeM3 = false;
volatile bool positionModeM4 = false;

const float TICKS_PER_REV = 937.2; // 11 PPR * 21.3 ratio * 4x multiplier
const float KP_POSITION = 0.4;
const int MIN_POSITION_PWM = 15;
const int MAX_POSITION_PWM = 65;

int calculatePositionSpeed(int32_t current, int32_t target, volatile bool &mode) {
  if (!mode) return 0;
  
  int32_t error = target - current;
  if (abs(error) <= 15) { // Widened from 3 to 15 ticks (~5.7 deg) to stop end-of-move oscillation
    mode = false; // Reached!
    return 0;
  }
  
  float speed = error * KP_POSITION;
  
  // Clamp speed to min/max
  if (speed > 0) {
    if (speed < MIN_POSITION_PWM) speed = MIN_POSITION_PWM;
    if (speed > MAX_POSITION_PWM) speed = MAX_POSITION_PWM;
  } else {
    if (speed > -MIN_POSITION_PWM) speed = -MIN_POSITION_PWM;
    if (speed < -MAX_POSITION_PWM) speed = -MAX_POSITION_PWM;
  }
  
  return (int)speed;
}

// ESP32Encoder Hardware instances using PCNT peripheral
ESP32Encoder encoderM1;
ESP32Encoder encoderM2;
ESP32Encoder encoderM3;
ESP32Encoder encoderM4;

void readEncoderTicks() {
  encoderTicksM1 = (int32_t)encoderM1.getCount();
  encoderTicksM2 = -(int32_t)encoderM2.getCount(); // Invert Direction to match motor alignment (Right Front)
  encoderTicksM3 = (int32_t)encoderM3.getCount();
  encoderTicksM4 = (int32_t)encoderM4.getCount(); // Invert Direction to match motor alignment (Right Rear)
}

// ────────────────────────────────────────────────────────────
// Motor Control Helper
// ────────────────────────────────────────────────────────────
void setMotorSpeed(int pinA, int pinB, int speed) {
  // speed is expected in range -100 to 100
  // Map it to 0-255 PWM duty cycle
  if (speed > 0) {
    int pwm = map(speed, 0, 100, 0, 255);
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    int pwm = map(-speed, 0, 100, 0, 255);
    analogWrite(pinA, 0);
    analogWrite(pinB, pwm);
  } else {
    // Active brake to stop immediately and hold position
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
  }
}

// Drive motor with raw PWM duty cycle (-255 to 255)
void setMotorSpeedPwm(int pinA, int pinB, int pwm) {
  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    // Active brake
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
  }
}

// Run Speed PI Controller loop every 50ms
// Run Speed PI Controller loop every 50ms with Cross-Coupled Motor Synchronization
void updateSpeedPid() {
  unsigned long now = millis();
  if (now - lastPidTime >= 50) {
    lastPidTime = now;
    
    // 1. Calculate actual ticks for all motors in this 50ms interval
    float actualM1 = (float)(encoderTicksM1 - prevPidTicksM1);
    float actualM2 = (float)(encoderTicksM2 - prevPidTicksM2);
    float actualM3 = (float)(encoderTicksM3 - prevPidTicksM3);
    float actualM4 = (float)(encoderTicksM4 - prevPidTicksM4);
    
    prevPidTicksM1 = encoderTicksM1;
    prevPidTicksM2 = encoderTicksM2;
    prevPidTicksM3 = encoderTicksM3;
    prevPidTicksM4 = encoderTicksM4;

    // 2. Compute target ticks based on target speeds
    float targetTicksM1 = (targetSpeedM1 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM2 = (targetSpeedM2 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM3 = (targetSpeedM3 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM4 = (targetSpeedM4 / 100.0) * MAX_TICKS_50MS;

    // 3. Compute normalized actual speeds (ticks per unit target speed)
    float normSpeedM1 = (targetSpeedM1 != 0) ? (actualM1 / (float)targetSpeedM1) : 0.0;
    float normSpeedM2 = (targetSpeedM2 != 0) ? (actualM2 / (float)targetSpeedM2) : 0.0;
    float normSpeedM3 = (targetSpeedM3 != 0) ? (actualM3 / (float)targetSpeedM3) : 0.0;
    float normSpeedM4 = (targetSpeedM4 != 0) ? (actualM4 / (float)targetSpeedM4) : 0.0;

    // 4. Calculate average normalized speed of active motors
    float sumNormSpeed = 0.0;
    int activeCount = 0;
    if (targetSpeedM1 != 0) { sumNormSpeed += normSpeedM1; activeCount++; }
    if (targetSpeedM2 != 0) { sumNormSpeed += normSpeedM2; activeCount++; }
    if (targetSpeedM3 != 0) { sumNormSpeed += normSpeedM3; activeCount++; }
    if (targetSpeedM4 != 0) { sumNormSpeed += normSpeedM4; activeCount++; }
    
    float avgNormSpeed = activeCount > 1 ? (sumNormSpeed / activeCount) : 0.0;

    // 5. Update Motor 1 Control Loop
    if (targetSpeedM1 == 0 && !positionModeM1) {
      errorSumM1 = 0;
      setMotorSpeedPwm(M1_IN1, M1_IN2, 0);
    } else {
      float errorM1 = targetTicksM1 - actualM1;
      if (activeCount > 1 && KC_SYNC > 0.0) {
        float syncErrorM1 = (normSpeedM1 - avgNormSpeed) * (float)targetSpeedM1;
        errorM1 -= KC_SYNC * syncErrorM1;
      }
      errorSumM1 += errorM1;
      errorSumM1 = constrain(errorSumM1, -200.0, 200.0);
      float pwmM1 = (targetSpeedM1 * 2.55) + (KP_SPEED * errorM1) + (KI_SPEED * errorSumM1);
      setMotorSpeedPwm(M1_IN1, M1_IN2, constrain((int)pwmM1, -255, 255));
    }

    // 6. Update Motor 2 Control Loop
    if (targetSpeedM2 == 0 && !positionModeM2) {
      errorSumM2 = 0;
      setMotorSpeedPwm(M2_IN1, M2_IN2, 0);
    } else {
      float errorM2 = targetTicksM2 - actualM2;
      if (activeCount > 1 && KC_SYNC > 0.0) {
        float syncErrorM2 = (normSpeedM2 - avgNormSpeed) * (float)targetSpeedM2;
        errorM2 -= KC_SYNC * syncErrorM2;
      }
      errorSumM2 += errorM2;
      errorSumM2 = constrain(errorSumM2, -200.0, 200.0);
      float pwmM2 = (targetSpeedM2 * 2.55) + (KP_SPEED * errorM2) + (KI_SPEED * errorSumM2);
      // Invert voltage polarity for Front Right motor because it is physically mirrored
      setMotorSpeedPwm(M2_IN1, M2_IN2, -constrain((int)pwmM2, -255, 255));
    }

    // 7. Update Motor 3 Control Loop
    if (targetSpeedM3 == 0 && !positionModeM3) {
      errorSumM3 = 0;
      setMotorSpeedPwm(M3_IN1, M3_IN2, 0);
    } else {
      float errorM3 = targetTicksM3 - actualM3;
      if (activeCount > 1 && KC_SYNC > 0.0) {
        float syncErrorM3 = (normSpeedM3 - avgNormSpeed) * (float)targetSpeedM3;
        errorM3 -= KC_SYNC * syncErrorM3;
      }
      errorSumM3 += errorM3;
      errorSumM3 = constrain(errorSumM3, -200.0, 200.0);
      float pwmM3 = (targetSpeedM3 * 2.55) + (KP_SPEED * errorM3) + (KI_SPEED * errorSumM3);
      setMotorSpeedPwm(M3_IN1, M3_IN2, constrain((int)pwmM3, -255, 255));
    }

    // 8. Update Motor 4 Control Loop
    if (targetSpeedM4 == 0 && !positionModeM4) {
      errorSumM4 = 0;
      setMotorSpeedPwm(M4_IN1, M4_IN2, 0);
    } else {
      float errorM4 = targetTicksM4 - actualM4;
      if (activeCount > 1 && KC_SYNC > 0.0) {
        float syncErrorM4 = (normSpeedM4 - avgNormSpeed) * (float)targetSpeedM4;
        errorM4 -= KC_SYNC * syncErrorM4;
      }
      errorSumM4 += errorM4;
      errorSumM4 = constrain(errorSumM4, -200.0, 200.0);
      float pwmM4 = (targetSpeedM4 * 2.55) + (KP_SPEED * errorM4) + (KI_SPEED * errorSumM4);
      // Invert voltage polarity for Rear Right motor because it is physically mirrored
      setMotorSpeedPwm(M4_IN1, M4_IN2, constrain((int)pwmM4, -255, 255));
    }
  }
}

// Update target velocities and check safety timeouts
void updateMotors() {
  // If position mode is active, override target speeds
  if (positionModeM1) targetSpeedM1 = calculatePositionSpeed(encoderTicksM1, targetPositionM1, positionModeM1);
  if (positionModeM2) targetSpeedM2 = calculatePositionSpeed(encoderTicksM2, targetPositionM2, positionModeM2);
  if (positionModeM3) targetSpeedM3 = calculatePositionSpeed(encoderTicksM3, targetPositionM3, positionModeM3);
  if (positionModeM4) targetSpeedM4 = calculatePositionSpeed(encoderTicksM4, targetPositionM4, positionModeM4);

  // If not in position mode, enforce safety timeout
  if (!positionModeM1 && !positionModeM2 && !positionModeM3 && !positionModeM4) {
    if (targetSpeedM1 != 0 || targetSpeedM2 != 0 || targetSpeedM3 != 0 || targetSpeedM4 != 0) {
      if (millis() - lastCommandTime > SAFETY_TIMEOUT_MS) {
        targetSpeedM1 = 0;
        targetSpeedM2 = 0;
        targetSpeedM3 = 0;
        targetSpeedM4 = 0;
      }
    }
  }
}

// ────────────────────────────────────────────────────────────
// Web HTML UI String
// ────────────────────────────────────────────────────────────
const char htmlContent[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Maker ESP32 Pro Cockpit</title>
  <style>
    :root {
      --bg: #0b0b0e;
      --card-bg: rgba(18, 18, 28, 0.9);
      --accent: #00f2fe;
      --accent-grad: linear-gradient(135deg, #00f2fe, #4facfe);
      --text: #f8fafc;
      --text-muted: #64748b;
      --border: rgba(255, 255, 255, 0.05);
    }
    body {
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg);
      color: var(--text);
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
      box-sizing: border-box;
    }
    .container {
      width: 100%;
      max-width: 500px;
      display: flex;
      flex-direction: column;
      gap: 15px;
    }
    h1 {
      text-align: center;
      font-size: 1.5rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 1px;
      background: var(--accent-grad);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      margin: 10px 0;
    }
    .card {
      background: var(--card-bg);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 4px 20px rgba(0, 0, 0, 0.5);
    }
    h2 {
      margin-top: 0;
      font-size: 0.95rem;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.5px;
      border-bottom: 1px solid var(--border);
      padding-bottom: 6px;
      margin-bottom: 12px;
    }
    .slider-group {
      display: flex;
      flex-direction: column;
      gap: 6px;
      margin-bottom: 12px;
    }
    .slider-group:last-child {
      margin-bottom: 0;
    }
    label {
      font-size: 0.8rem;
      color: #94a3b8;
      display: flex;
      justify-content: space-between;
    }
    .val {
      font-family: monospace;
      color: var(--accent);
      font-weight: bold;
    }
    input[type=range] {
      width: 100%;
      height: 6px;
      background: #1e293b;
      border-radius: 3px;
      outline: none;
      -webkit-appearance: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 16px;
      height: 16px;
      border-radius: 50%;
      background: var(--accent);
      cursor: pointer;
      box-shadow: 0 0 8px var(--accent);
    }
    .btn {
      background: var(--accent-grad);
      border: none;
      color: white;
      padding: 12px;
      font-size: 0.9rem;
      font-weight: bold;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.15s;
    }
    .btn:active {
      transform: scale(0.97);
    }
    .btn.stop {
      background: linear-gradient(135deg, #f43f5e, #e11d48);
      box-shadow: 0 0 10px rgba(244, 63, 94, 0.3);
    }
    .btn-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
    }
    .telemetry-row {
      display: flex;
      justify-content: space-between;
      font-size: 0.85rem;
      border-bottom: 1px solid var(--border);
      padding: 6px 0;
    }
    .telemetry-row:last-child {
      border: none;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>Maker ESP32 Cockpit</h1>
    
    <!-- Drive Controls -->
    <div class="card">
      <h2>Quick Drive</h2>
      <div class="btn-grid">
        <button class="btn" onclick="drive('spinleft')">Spin L</button>
        <button class="btn" onclick="drive('forward')">FWD</button>
        <button class="btn" onclick="drive('spinright')">Spin R</button>
        
        <button class="btn" onclick="drive('left')">LEFT</button>
        <button class="btn stop" onclick="drive('stop')">STOP</button>
        <button class="btn" onclick="drive('right')">RIGHT</button>
        
        <div></div>
        <button class="btn" onclick="drive('reverse')">REV</button>
        <div></div>
      </div>
    </div>

    <!-- Sliders -->
    <div class="card">
      <h2>Manual Motors</h2>
      <div class="slider-group">
        <label>Motor 1 (Left Front) <span id="val-m1" class="val">0</span></label>
        <input type="range" id="m1" min="-100" max="100" value="0" oninput="updateSpeed()">
      </div>
      <div class="slider-group">
        <label>Motor 2 (Right Front) <span id="val-m2" class="val">0</span></label>
        <input type="range" id="m2" min="-100" max="100" value="0" oninput="updateSpeed()">
      </div>
      <div class="slider-group">
        <label>Motor 3 (Left Rear) <span id="val-m3" class="val">0</span></label>
        <input type="range" id="m3" min="-100" max="100" value="0" oninput="updateSpeed()">
      </div>
      <div class="slider-group">
        <label>Motor 4 (Right Rear) <span id="val-m4" class="val">0</span></label>
        <input type="range" id="m4" min="-100" max="100" value="0" oninput="updateSpeed()">
      </div>
    </div>

    <!-- Precise Position Control -->
    <div class="card">
      <h2>Precise Position Control</h2>
      <div style="display: grid; grid-template-columns: 2fr 1fr; gap: 10px; align-items: center; margin-bottom: 12px;">
        <label style="color: #94a3b8; font-size: 0.85rem;">Target Turns:</label>
        <input type="number" id="num-turns" step="0.1" value="1.0" style="background: #1e293b; color: var(--text); border: 1px solid var(--border); border-radius: 6px; padding: 6px; text-align: center; width: 60px; outline: none; font-weight: bold;">
      </div>
      <div class="btn-grid" style="grid-template-columns: repeat(4, 1fr); margin-bottom: 10px;">
        <button class="btn" onclick="turnWheel('m1')">M1 (LF)</button>
        <button class="btn" onclick="turnWheel('m2')">M2 (RF)</button>
        <button class="btn" onclick="turnWheel('m3')">M3 (LR)</button>
        <button class="btn" onclick="turnWheel('m4')">M4 (RR)</button>
      </div>
      <div style="display: flex; gap: 8px;">
        <button class="btn" style="flex: 1.2; background: var(--accent-grad); box-shadow: 0 0 10px rgba(0, 242, 254, 0.2);" onclick="turnWheel('all')">Rotate All Wheels</button>
        <button class="btn" style="flex: 0.8; background: linear-gradient(135deg, #f43f5e, #e11d48); font-weight: bold; box-shadow: 0 0 10px rgba(244, 63, 94, 0.4);" onclick="stopPositionMode()">⚠️ ESTOP ROTATE</button>
      </div>
    </div>

    <!-- Telemetry -->
    <div class="card">
      <h2>Status & Telemetry</h2>
      <div class="telemetry-row" style="margin-bottom: 12px; border: none; padding: 0;">
        <span>Battery:</span>
        <span id="tel-batt" class="val" style="font-size: 1.1rem; color: #ff0055;">Unknown</span>
      </div>
      
      <h2 style="margin-top: 15px;">Motor Feedback</h2>
      <table style="width: 100%; border-collapse: collapse; font-size: 0.85rem; text-align: left;">
        <thead>
          <tr style="color: var(--text-muted); border-bottom: 1px solid var(--border);">
            <th style="padding: 6px 0;">Motor</th>
            <th>Ticks</th>
            <th>RPM</th>
            <th>MPH</th>
          </tr>
        </thead>
        <tbody>
          <tr style="border-bottom: 1px solid var(--border);">
            <td style="padding: 8px 0; font-weight: bold; color: var(--accent);">M1 (LF)</td>
            <td id="tel-e1" class="val">0</td>
            <td id="tel-rpm1" class="val">0.0</td>
            <td id="tel-mph1" class="val">0.00</td>
          </tr>
          <tr style="border-bottom: 1px solid var(--border);">
            <td style="padding: 8px 0; font-weight: bold; color: var(--accent);">M2 (RF)</td>
            <td id="tel-e2" class="val">0</td>
            <td id="tel-rpm2" class="val">0.0</td>
            <td id="tel-mph2" class="val">0.00</td>
          </tr>
          <tr style="border-bottom: 1px solid var(--border);">
            <td style="padding: 8px 0; font-weight: bold; color: var(--accent);">M3 (LR)</td>
            <td id="tel-e3" class="val">0</td>
            <td id="tel-rpm3" class="val">0.0</td>
            <td id="tel-mph3" class="val">0.00</td>
          </tr>
          <tr>
            <td style="padding: 8px 0; font-weight: bold; color: var(--accent);">M4 (RR)</td>
            <td id="tel-e4" class="val">0</td>
            <td id="tel-rpm4" class="val">0.0</td>
            <td id="tel-mph4" class="val">0.00</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>

  <script>
    let sendTimeout = null;
    let keepAliveInterval = null;
    let currentM1=0, currentM2=0, currentM3=0, currentM4=0;

    function drive(dir) {
      let m1=0, m2=0, m3=0, m4=0;
      const spd = 60;
      if (dir === 'forward')  { m1=spd; m2=spd; m3=spd; m4=spd; }
      else if (dir === 'reverse') { m1=-spd; m2=-spd; m3=-spd; m4=-spd; }
      else if (dir === 'left')    { m1=-spd; m2=spd; m3=-spd; m4=spd; }
      else if (dir === 'right')   { m1=spd; m2=-spd; m3=spd; m4=-spd; }
      else if (dir === 'spinleft') { m1=-spd; m2=spd; m3=-spd; m4=spd; }
      else if (dir === 'spinright') { m1=spd; m2=-spd; m3=spd; m4=-spd; }
      
      document.getElementById('m1').value = m1;
      document.getElementById('m2').value = m2;
      document.getElementById('m3').value = m3;
      document.getElementById('m4').value = m4;
      
      document.getElementById('val-m1').innerText = m1;
      document.getElementById('val-m2').innerText = m2;
      document.getElementById('val-m3').innerText = m3;
      document.getElementById('val-m4').innerText = m4;
      
      sendSpeed(m1, m2, m3, m4);
    }

    function updateSpeed() {
      const m1 = document.getElementById('m1').value;
      const m2 = document.getElementById('m2').value;
      const m3 = document.getElementById('m3').value;
      const m4 = document.getElementById('m4').value;
      
      document.getElementById('val-m1').innerText = m1;
      document.getElementById('val-m2').innerText = m2;
      document.getElementById('val-m3').innerText = m3;
      document.getElementById('val-m4').innerText = m4;

      if (sendTimeout) clearTimeout(sendTimeout);
      sendTimeout = setTimeout(() => {
        sendSpeed(m1, m2, m3, m4);
      }, 50);
    }

    function turnWheel(target) {
      const turns = document.getElementById('num-turns').value;
      let query = "";
      if (target === 'all') {
        query = `m1=${turns}&m2=${turns}&m3=${turns}&m4=${turns}`;
      } else {
        query = `${target}=${turns}`;
      }
      fetch(`/api/turn?${query}`)
        .catch(err => console.error(err));
    }

    function stopPositionMode() {
      fetch(`/api/turn?stop=1`)
        .catch(err => console.error(err));
    }

    function sendSpeed(m1, m2, m3, m4) {
      currentM1 = m1;
      currentM2 = m2;
      currentM3 = m3;
      currentM4 = m4;

      fetch(`/api/motor?m1=${m1}&m2=${m2}&m3=${m3}&m4=${m4}`)
        .catch(err => console.error(err));

      // Manage Web keep-alive loop
      if (keepAliveInterval) clearInterval(keepAliveInterval);

      const allZero = (m1 == 0 && m2 == 0 && m3 == 0 && m4 == 0);
      if (!allZero) {
        keepAliveInterval = setInterval(() => {
          fetch(`/api/motor?m1=${currentM1}&m2=${currentM2}&m3=${currentM3}&m4=${currentM4}`)
            .catch(err => console.error(err));
        }, 400); // Send keep-alive every 400ms (safety timeout is 1000ms)
      }
    }

    setInterval(() => {
      fetch('/api/status')
        .then(res => {
          if (!res.ok) throw new Error("Offline");
          return res.json();
        })
        .then(data => {
          if (data.battery <= 2.0) {
            document.getElementById('tel-batt').innerText = "Unknown";
            document.getElementById('tel-batt').style.color = "var(--text-muted)";
          } else {
            document.getElementById('tel-batt').innerText = data.battery.toFixed(2) + " V";
            document.getElementById('tel-batt').style.color = "#39ff14";
          }
          
          document.getElementById('tel-e1').innerText = data.e1;
          document.getElementById('tel-e2').innerText = data.e2;
          document.getElementById('tel-e3').innerText = data.e3;
          document.getElementById('tel-e4').innerText = data.e4;

          document.getElementById('tel-rpm1').innerText = data.rpm1.toFixed(1);
          document.getElementById('tel-rpm2').innerText = data.rpm2.toFixed(1);
          document.getElementById('tel-rpm3').innerText = data.rpm3.toFixed(1);
          document.getElementById('tel-rpm4').innerText = data.rpm4.toFixed(1);

          document.getElementById('tel-mph1').innerText = data.mph1.toFixed(2);
          document.getElementById('tel-mph2').innerText = data.mph2.toFixed(2);
          document.getElementById('tel-mph3').innerText = data.mph3.toFixed(2);
          document.getElementById('tel-mph4').innerText = data.mph4.toFixed(2);
        })
        .catch(err => {
          document.getElementById('tel-batt').innerText = "Offline";
          document.getElementById('tel-batt').style.color = "red";
          document.getElementById('tel-e1').innerText = "Offline";
          document.getElementById('tel-e2').innerText = "Offline";
          document.getElementById('tel-e3').innerText = "Offline";
          document.getElementById('tel-e4').innerText = "Offline";
          document.getElementById('tel-rpm1').innerText = "Offline";
          document.getElementById('tel-rpm2').innerText = "Offline";
          document.getElementById('tel-rpm3').innerText = "Offline";
          document.getElementById('tel-rpm4').innerText = "Offline";
          document.getElementById('tel-mph1').innerText = "Offline";
          document.getElementById('tel-mph2').innerText = "Offline";
          document.getElementById('tel-mph3').innerText = "Offline";
          document.getElementById('tel-mph4').innerText = "Offline";
        });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

// ────────────────────────────────────────────────────────────
// WebServer Route Handlers
// ────────────────────────────────────────────────────────────
void handleRoot() {
  server.send(200, "text/html", htmlContent);
}

void handleMotorApi() {
  // Cancel position mode since user is commanding motor speeds directly
  positionModeM1 = false;
  positionModeM2 = false;
  positionModeM3 = false;
  positionModeM4 = false;

  if (server.hasArg("m1")) targetSpeedM1 = server.arg("m1").toInt();
  if (server.hasArg("m2")) targetSpeedM2 = server.arg("m2").toInt();
  if (server.hasArg("m3")) targetSpeedM3 = server.arg("m3").toInt();
  if (server.hasArg("m4")) targetSpeedM4 = server.arg("m4").toInt();
  
  lastCommandTime = millis(); // Refresh safety timeout
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTurnApi() {
  if (server.hasArg("stop") || server.hasArg("estop")) {
    positionModeM1 = false;
    positionModeM2 = false;
    positionModeM3 = false;
    positionModeM4 = false;
    targetSpeedM1 = 0;
    targetSpeedM2 = 0;
    targetSpeedM3 = 0;
    targetSpeedM4 = 0;
    server.send(200, "application/json", "{\"ok\":true,\"stopped\":true}");
    return;
  }
  if (server.hasArg("m1")) {
    float turns = server.arg("m1").toFloat();
    targetPositionM1 = encoderTicksM1 + (int32_t)(turns * TICKS_PER_REV);
    positionModeM1 = true;
  }
  if (server.hasArg("m2")) {
    float turns = server.arg("m2").toFloat();
    targetPositionM2 = encoderTicksM2 + (int32_t)(turns * TICKS_PER_REV);
    positionModeM2 = true;
  }
  if (server.hasArg("m3")) {
    float turns = server.arg("m3").toFloat();
    targetPositionM3 = encoderTicksM3 + (int32_t)(turns * TICKS_PER_REV);
    positionModeM3 = true;
  }
  if (server.hasArg("m4")) {
    float turns = server.arg("m4").toFloat();
    targetPositionM4 = encoderTicksM4 + (int32_t)(turns * TICKS_PER_REV);
    positionModeM4 = true;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatusApi() {
  String json = "{";
  json += "\"m1\":" + String(targetSpeedM1) + ",";
  json += "\"m2\":" + String(targetSpeedM2) + ",";
  json += "\"m3\":" + String(targetSpeedM3) + ",";
  json += "\"m4\":" + String(targetSpeedM4) + ",";
  json += "\"e1\":" + String(encoderTicksM1) + ",";
  json += "\"e2\":" + String(encoderTicksM2) + ",";
  json += "\"e3\":" + String(encoderTicksM3) + ",";
  json += "\"e4\":" + String(encoderTicksM4) + ",";
  json += "\"rpm1\":" + String(rpmM1, 1) + ",";
  json += "\"rpm2\":" + String(rpmM2, 1) + ",";
  json += "\"rpm3\":" + String(rpmM3, 1) + ",";
  json += "\"rpm4\":" + String(rpmM4, 1) + ",";
  json += "\"mph1\":" + String(abs(mphM1), 2) + ",";
  json += "\"mph2\":" + String(abs(mphM2), 2) + ",";
  json += "\"mph3\":" + String(abs(mphM3), 2) + ",";
  json += "\"mph4\":" + String(abs(mphM4), 2) + ",";
  json += "\"battery\":" + String(simulatedVoltage, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// ────────────────────────────────────────────────────────────
// Protocol Parser (Host -> Board over Serial)
// ────────────────────────────────────────────────────────────
enum ParserState {
  WAIT_HEAD,
  WAIT_DEVICE,
  WAIT_LEN,
  WAIT_PAYLOAD
};

ParserState parserState = WAIT_HEAD;
uint8_t extLen = 0;
uint8_t payloadBuf[64];
uint8_t payloadIdx = 0;

void processPacket() {
  uint8_t funcId = payloadBuf[0];
  uint8_t receivedChecksum = payloadBuf[extLen - 1];

  uint16_t sum = extLen;
  for (int i = 0; i < extLen - 1; i++) {
    sum += payloadBuf[i];
  }
  uint8_t calculatedChecksum = sum & 0xFF;

  if (calculatedChecksum != receivedChecksum) {
    return;
  }

  // Valid packet received via serial!
  lastCommandTime = millis();

  switch (funcId) {
    case 0x10: { // FUNC_MOTOR
      if (extLen >= 6) {
        // Cancel position mode since user is commanding motor speeds directly
        positionModeM1 = false;
        positionModeM2 = false;
        positionModeM3 = false;
        positionModeM4 = false;

        targetSpeedM1 = (int8_t)payloadBuf[1];
        targetSpeedM2 = (int8_t)payloadBuf[2];
        targetSpeedM3 = (int8_t)payloadBuf[3];
        targetSpeedM4 = (int8_t)payloadBuf[4];
      }
      break;
    }

    case 0x02: { // FUNC_BEEP
      if (extLen >= 4) {
        uint16_t duration = payloadBuf[1] | (payloadBuf[2] << 8);
        if (STATUS_LED >= 0) {
          digitalWrite(STATUS_LED, LOW);
          delay(duration / 2);
          digitalWrite(STATUS_LED, HIGH);
        } else {
          // Fallback delay if no status LED
          delay(duration);
        }
      }
      break;
    }
  }
}

void parseSerialInput() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    switch (parserState) {
      case WAIT_HEAD:
        if (b == 0xFF) {
          parserState = WAIT_DEVICE;
        }
        break;

      case WAIT_DEVICE:
        if (b == 0xFC) {
          parserState = WAIT_LEN;
        } else if (b != 0xFF) {
          parserState = WAIT_HEAD;
        }
        break;

      case WAIT_LEN:
        extLen = b;
        if (extLen > sizeof(payloadBuf) || extLen < 2) {
          parserState = WAIT_HEAD;
        } else {
          payloadIdx = 0;
          parserState = WAIT_PAYLOAD;
        }
        break;

      case WAIT_PAYLOAD:
        payloadBuf[payloadIdx++] = b;
        if (payloadIdx >= extLen) {
          processPacket();
          parserState = WAIT_HEAD;
        }
        break;
    }
  }
}

// ────────────────────────────────────────────────────────────
// Telemetry Generator (Board -> Host over Serial)
// ────────────────────────────────────────────────────────────
void sendTelemetryPacket(uint8_t extType, uint8_t *data, uint8_t dataLen) {
  uint8_t extLen = dataLen + 3;
  uint8_t sum = extLen + extType;

  Serial.write(0xFF);
  Serial.write(0xFB);
  Serial.write(extLen);
  Serial.write(extType);

  for (uint8_t i = 0; i < dataLen; i++) {
    Serial.write(data[i]);
    sum += data[i];
  }

  Serial.write(sum & 0xFF);
}

void sendTelemetry() {
  // 1. Encoder packet (TYPE_ENCODER = 0x0D)
  uint8_t encoderData[16];
  memcpy(&encoderData[0],  (const void*)&encoderTicksM1, 4);
  memcpy(&encoderData[4],  (const void*)&encoderTicksM2, 4);
  memcpy(&encoderData[8],  (const void*)&encoderTicksM3, 4);
  memcpy(&encoderData[12], (const void*)&encoderTicksM4, 4);
  sendTelemetryPacket(0x0D, encoderData, 16);

  // Calculate simulated battery consumption
  if (simulatedVoltage > 2.0) {
    float idleDraw = 0.0000014;
    float motorDraw = (abs(targetSpeedM1) + abs(targetSpeedM2) + abs(targetSpeedM3) + abs(targetSpeedM4)) * 0.0000003;
    simulatedVoltage -= (idleDraw + motorDraw);
    if (simulatedVoltage < 6.0) simulatedVoltage = 6.0; // clamp to 2S LiPo cut-off
  } else {
    simulatedVoltage = 0.0;
  }

  // 2. Battery packet (TYPE_BATTERY = 0x0A)
  uint8_t battRaw = (uint8_t)(simulatedVoltage * 10.0);
  uint8_t batteryData[7] = {0, 0, 0, 0, 0, 0, battRaw}; // voltage * 10
  sendTelemetryPacket(0x0A, batteryData, 7);

  // 3. IMU telemetry (TYPE_IMU = 0x0E)
  int16_t leftSpeed = (targetSpeedM1 + targetSpeedM3) / 2;
  int16_t rightSpeed = (targetSpeedM2 + targetSpeedM4) / 2;
  int16_t turnRate = rightSpeed - leftSpeed;
  int16_t gz_raw = turnRate * -10;

  int16_t ax_raw = 0;
  int16_t ay_raw = 0;
  int16_t az_raw = 1000; 

  int16_t gx_raw = 0;
  int16_t gy_raw = 0;
  int16_t mx_raw = 0;
  int16_t my_raw = 0;
  int16_t mz_raw = 0;

  uint8_t imuData[19];
  memcpy(&imuData[0],  &gx_raw, 2);
  memcpy(&imuData[2],  &gy_raw, 2);
  memcpy(&imuData[4],  &gz_raw, 2);
  memcpy(&imuData[6],  &ax_raw, 2);
  memcpy(&imuData[8],  &ay_raw, 2);
  memcpy(&imuData[10], &az_raw, 2);
  memcpy(&imuData[12], &mx_raw, 2);
  memcpy(&imuData[14], &my_raw, 2);
  memcpy(&imuData[16], &mz_raw, 2);
  imuData[18] = 0;

  sendTelemetryPacket(0x0E, imuData, 19);
}

void updateOledScroll() {
  if (!oledOk || scrollText.length() == 0) return;
  unsigned long now = millis();
  if (now - lastScrollTime >= 350) {
    lastScrollTime = now;
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    
    // Draw static centered Line 1: "COCKPIT" (7 chars * 12px = 84px width, offset x=22)
    display.setCursor(22, 0);
    display.print("COCKPIT");
    
    // Draw scrolling Line 2
    display.setCursor(0, 16);
    String visible = "";
    for (int i = 0; i < 10; i++) {
      int idx = (scrollIndex + i) % scrollText.length();
      visible += scrollText[idx];
    }
    display.print(visible);
    display.display();
    
    scrollIndex = (scrollIndex + 1) % scrollText.length();
  }
}

// ────────────────────────────────────────────────────────────
// Setup & Main Loop
// ────────────────────────────────────────────────────────────
void setup() {
  // Configure Status LED
  // Configure Status LED
  if (STATUS_LED >= 0) {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
  }

  // ────────────────────────────────────────────────────────────
  // Verification Step: Flash Onboard LED exactly 3 times
  // ────────────────────────────────────────────────────────────
  if (STATUS_LED >= 0) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(400);
      digitalWrite(STATUS_LED, LOW);
      delay(400);
    }
  }

  // Configure Motor pins
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);

  // Stop all motors initially
  updateMotors();

  // Configure Encoders using ESP32 PCNT Hardware driver
  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoderM1.attachFullQuad(E1_A, E1_B);
  encoderM1.setFilter(1023); // Hardware Glitch Filter (max 1023)

  encoderM2.attachFullQuad(E2_A, E2_B);
  encoderM2.setFilter(1023);

  encoderM3.attachFullQuad(E3_A, E3_B);
  encoderM3.setFilter(1023);

  encoderM4.attachFullQuad(E4_B, E4_A);
  encoderM4.setFilter(1023);

  // Initialize I2C and OLED
  Wire.begin(21, 22);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    // Line 1: Centered "WIFI" (4 chars * 12px = 48px, x = (128-48)/2 = 40)
    display.setCursor(40, 0);
    display.print("WIFI");
    // Line 2: Centered "CONNECT..." (10 chars * 12px = 120px, x = (128-120)/2 = 4)
    display.setCursor(4, 16);
    display.print("CONNECT...");
    display.display();
  }

  // Initialize Serial
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  // Flash LED rapidly while connecting to WiFi
  unsigned long startWifiTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifiTime < 12000) {
    if (STATUS_LED >= 0) {
      digitalWrite(STATUS_LED, HIGH);
      delay(150);
      digitalWrite(STATUS_LED, LOW);
      delay(150);
    } else {
      delay(300);
    }
  }

  // Turn LED Solid on successful connection, off otherwise
  if (WiFi.status() == WL_CONNECTED) {
    if (STATUS_LED >= 0) {
      digitalWrite(STATUS_LED, HIGH);
    }
    // Start MDNS
    if (MDNS.begin("maker-esp32")) {
      MDNS.addService("http", "tcp", 80);
    }
    
    // Set up web routes
    server.on("/", handleRoot);
    server.on("/api/motor", handleMotorApi);
    server.on("/api/status", handleStatusApi);
    server.on("/api/turn", handleTurnApi);
    server.begin();

    if (oledOk) {
      scrollText = "   maker-esp32.local   [IP: " + WiFi.localIP().toString() + "]   ";
    }
  } else {
    if (STATUS_LED >= 0) {
      digitalWrite(STATUS_LED, LOW);
    }
    if (oledOk) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      // Line 1: Centered "WIFI"
      display.setCursor(40, 0);
      display.print("WIFI");
      // Line 2: Centered "FAILED!"
      display.setCursor(22, 16);
      display.print("FAILED!");
      display.display();
    }
  }
  
  lastCommandTime = millis();
}

void loop() {
  // Read ticks from hardware PCNT
  readEncoderTicks();

  // Handle client requests on WebServer
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // Parse incoming commands from the serial port
  parseSerialInput();

  // Handle safety timeouts and calculate motor target speeds
  updateMotors();

  // Run Speed PI Controller loop
  updateSpeedPid();

  // Update OLED horizontal scrolling
  updateOledScroll();

  // Calculate Speed & RPM every 500ms
  unsigned long now = millis();
  if (now - lastSpeedCalcTime >= 500) {
    float dt = (now - lastSpeedCalcTime) / 1000.0;
    if (dt > 0.0) {
      int32_t delta1 = encoderTicksM1 - prevTicksM1;
      int32_t delta2 = encoderTicksM2 - prevTicksM2;
      int32_t delta3 = encoderTicksM3 - prevTicksM3;
      int32_t delta4 = encoderTicksM4 - prevTicksM4;
      
      prevTicksM1 = encoderTicksM1;
      prevTicksM2 = encoderTicksM2;
      prevTicksM3 = encoderTicksM3;
      prevTicksM4 = encoderTicksM4;
      lastSpeedCalcTime = now;
      
      rpmM1 = (delta1 / dt) / TICKS_PER_REV * 60.0;
      rpmM2 = (delta2 / dt) / TICKS_PER_REV * 60.0;
      rpmM3 = (delta3 / dt) / TICKS_PER_REV * 60.0;
      rpmM4 = (delta4 / dt) / TICKS_PER_REV * 60.0;
      
      float rpmToMph = 2.559 * 3.14159265 * 5.0 / 5280.0;
      mphM1 = rpmM1 * rpmToMph;
      mphM2 = rpmM2 * rpmToMph;
      mphM3 = rpmM3 * rpmToMph;
      mphM4 = rpmM4 * rpmToMph;
    }
  }

  // Send telemetry packets at telemetry frequency (20Hz)
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}
