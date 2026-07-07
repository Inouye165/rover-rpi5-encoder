/**
 * @file main.cpp
 * @brief Firmware boilerplate for NULLLAB / emakefun Maker ESP32 Pro board.
 * @details Emulates the compatible binary serial protocol
 * while hosting a local WebServer dashboard.
 * 
 * Hardware Config (Reference: ACTIVE_REVISION.txt):
 *  - DIP Switch: MUST set the "Motor/IO Switch" to "Motor" to enable M2 and M3.
 *  - Power: Connect a 6V-16V battery/power source to the DC Barrel Jack.
 *  - Status LED: -1 (avoid conflict with M2_IN2 / GPIO 2)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Encoder.h>

// ────────────────────────────────────────────────────────────
// Hardware and Configuration Constants
// ────────────────────────────────────────────────────────────

// OLED Display Parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  // OLED I2C Address (default: 0x3C)

// OLED display interface (I2C Bus)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED Scrolling state variables
bool oledOk = false;
String scrollText = "";
int scrollIndex = 0;
unsigned long lastScrollTime = 0;

// WiFi Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Cockpit HTTP Web Server (Port 80)
WebServer server(80);

// ────────────────────────────────────────────────────────────
// ACTIVE REVISION: ESP32 Rover GPIO Pin Maps
// ────────────────────────────────────────────────────────────

// DC Motors Control Pins (PWM / Direction)
const int M1_IN1 = 27;  // Motor 1 (Left Front) control pin 1
const int M1_IN2 = 13;  // Motor 1 (Left Front) control pin 2
const int M2_IN1 = 4;   // Motor 2 (Right Front) control pin 1
const int M2_IN2 = 2;   // Motor 2 (Right Front) control pin 2 (Conflicts with onboard LED!)
const int M3_IN1 = 17;  // Motor 3 (Left Rear) control pin 1 (Must toggle DIP switch to "Motor")
const int M3_IN2 = 12;  // Motor 3 (Left Rear) control pin 2 (Must toggle DIP switch to "Motor")
const int M4_IN1 = 14;  // Motor 4 (Right Rear) control pin 1 (Must toggle DIP switch to "Motor")
const int M4_IN2 = 15;  // Motor 4 (Right Rear) control pin 2 (Must toggle DIP switch to "Motor")

// Incremental Quad Encoders Pins
const int E1_A = 18;    // Encoder 1 Channel A (Left Front)
const int E1_B = 19;    // Encoder 1 Channel B (Left Front)
const int E2_A = 5;     // Encoder 2 Channel A (Right Front)
const int E2_B = 23;    // Encoder 2 Channel B (Right Front)
const int E3_A = 35;    // Encoder 3 Channel A (Left Rear - Input Only)
const int E3_B = 36;    // Encoder 3 Channel B (Left Rear - Input Only)
const int E4_A = 34;    // Encoder 4 Channel A (Right Rear - Input Only)
const int E4_B = 39;    // Encoder 4 Channel B (Right Rear - Input Only)

// Status Indicators
const int STATUS_LED = -1; // -1 to avoid conflict with M2_IN2 (GPIO 2)

// ────────────────────────────────────────────────────────────
// Telemetry & Control Global State Variables
// ────────────────────────────────────────────────────────────

// Volatile targets for Motor PI controller (-100 to 100)
volatile int targetSpeedM1 = 0;
volatile int targetSpeedM2 = 0;
volatile int targetSpeedM3 = 0;
volatile int targetSpeedM4 = 0;

// Volatile tick counts updated by hardware PCNT driver
volatile int32_t encoderTicksM1 = 0;
volatile int32_t encoderTicksM2 = 0;
volatile int32_t encoderTicksM3 = 0;
volatile int32_t encoderTicksM4 = 0;

// Voltage monitoring (simulated or ADC read)
volatile float simulatedVoltage = 12.6; // Full 3S battery (12.6V)

// Telemetry calculation variables (RPM / MPH)
unsigned long lastSpeedCalcTime = 0;
int32_t prevTicksM1 = 0, prevTicksM2 = 0, prevTicksM3 = 0, prevTicksM4 = 0;
float rpmM1 = 0.0, rpmM2 = 0.0, rpmM3 = 0.0, rpmM4 = 0.0;
float mphM1 = 0.0, mphM2 = 0.0, mphM3 = 0.0, mphM4 = 0.0;

// Speed PI (Velocity) controller state
unsigned long lastPidTime = 0;
int32_t prevPidTicksM1 = 0, prevPidTicksM2 = 0, prevPidTicksM3 = 0, prevPidTicksM4 = 0;
float errorSumM1 = 0.0, errorSumM2 = 0.0, errorSumM3 = 0.0, errorSumM4 = 0.0;

const float MAX_TICKS_50MS = 257.0; // Max ticks at 100% PWM per 50ms (at motor shaft)
const float KP_SPEED = 0.5;
const float KI_SPEED = 0.15;
const float KC_SYNC = 0.30;         // Cross-coupling sync gain

// Safety timeouts
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT_MS = 1000; // Estop if communication from Pi fails for >1s

// Telemetry stream timer
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // Send telemetry at 20 Hz (every 50ms)

// Position Control Mode parameters
volatile int32_t targetPositionM1 = 0;
volatile int32_t targetPositionM2 = 0;
volatile int32_t targetPositionM3 = 0;
volatile int32_t targetPositionM4 = 0;

volatile bool positionModeM1 = false;
volatile bool positionModeM2 = false;
volatile bool positionModeM3 = false;
volatile bool positionModeM4 = false;

const float TICKS_PER_REV = 937.2; // 11 PPR * 21.3 Gearbox * 4x Quad evaluation
const float KP_POSITION = 0.4;
const int MIN_POSITION_PWM = 15;
const int MAX_POSITION_PWM = 65;

// Encoder instances using ESP32 PCNT peripheral
ESP32Encoder encoderM1;
ESP32Encoder encoderM2;
ESP32Encoder encoderM3;
ESP32Encoder encoderM4;

// ────────────────────────────────────────────────────────────
// Utility Functions
// ────────────────────────────────────────────────────────────

float getSign(float val) {
  if (val > 0) return 1.0;
  if (val < 0) return -1.0;
  return 0.0;
}

// ────────────────────────────────────────────────────────────
// Position and Feedback Routines
// ────────────────────────────────────────────────────────────

void readEncoderTicks() {
  encoderTicksM1 = (int32_t)encoderM1.getCount();
  encoderTicksM2 = -(int32_t)encoderM2.getCount(); // Mirrored on right front wheel
  encoderTicksM3 = (int32_t)encoderM3.getCount();
  encoderTicksM4 = -(int32_t)encoderM4.getCount(); // Mirrored on right rear wheel
}

int calculatePositionSpeed(int32_t current, int32_t target, volatile bool &mode) {
  if (!mode) return 0;
  
  int32_t error = target - current;
  if (abs(error) <= 3) {
    mode = false; // Reached target position!
    return 0;
  }
  
  float speed = error * KP_POSITION;
  
  // Constrain to motor PWM capabilities
  if (speed > 0) {
    if (speed < MIN_POSITION_PWM) speed = MIN_POSITION_PWM;
    if (speed > MAX_POSITION_PWM) speed = MAX_POSITION_PWM;
  } else {
    if (speed > -MIN_POSITION_PWM) speed = -MIN_POSITION_PWM;
    if (speed < -MAX_POSITION_PWM) speed = -MAX_POSITION_PWM;
  }
  
  return (int)speed;
}

// ────────────────────────────────────────────────────────────
// Low-Level H-Bridge Motor Drivers
// ────────────────────────────────────────────────────────────

void setMotorSpeed(int pinA, int pinB, int speed) {
  // speed maps from -100..100 to 0..255 duty cycle
  if (speed > 0) {
    int pwm = map(speed, 0, 100, 0, 255);
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    int pwm = map(-speed, 0, 100, 0, 255);
    analogWrite(pinA, 0);
    analogWrite(pinB, pwm);
  } else {
    // Active braking to prevent coasting
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
  }
}

void setMotorSpeedPwm(int pinA, int pinB, int pwm) {
  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    // Active braking
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
  }
}

// Cross-Coupled velocity synchronization and loop execution
void updateSpeedPid() {
  unsigned long now = millis();
  if (now - lastPidTime >= 50) {
    lastPidTime = now;
    
    // Calculate current interval delta ticks
    float actualM1 = (float)(encoderTicksM1 - prevPidTicksM1);
    float actualM2 = (float)(encoderTicksM2 - prevPidTicksM2);
    float actualM3 = (float)(encoderTicksM3 - prevPidTicksM3);
    float actualM4 = (float)(encoderTicksM4 - prevPidTicksM4);
    
    prevPidTicksM1 = encoderTicksM1;
    prevPidTicksM2 = encoderTicksM2;
    prevPidTicksM3 = encoderTicksM3;
    prevPidTicksM4 = encoderTicksM4;

    // Convert speed targets to target ticks per 50ms
    float targetTicksM1 = (targetSpeedM1 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM2 = (targetSpeedM2 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM3 = (targetSpeedM3 / 100.0) * MAX_TICKS_50MS;
    float targetTicksM4 = (targetSpeedM4 / 100.0) * MAX_TICKS_50MS;

    // Calculate normalized velocities
    float normSpeedM1 = (targetSpeedM1 != 0) ? (actualM1 / (float)targetSpeedM1) : 0.0;
    float normSpeedM2 = (targetSpeedM2 != 0) ? (actualM2 / (float)targetSpeedM2) : 0.0;
    float normSpeedM3 = (targetSpeedM3 != 0) ? (actualM3 / (float)targetSpeedM3) : 0.0;
    float normSpeedM4 = (targetSpeedM4 != 0) ? (actualM4 / (float)targetSpeedM4) : 0.0;

    float sumNormSpeed = 0.0;
    int activeCount = 0;
    if (targetSpeedM1 != 0) { sumNormSpeed += normSpeedM1; activeCount++; }
    if (targetSpeedM2 != 0) { sumNormSpeed += normSpeedM2; activeCount++; }
    if (targetSpeedM3 != 0) { sumNormSpeed += normSpeedM3; activeCount++; }
    if (targetSpeedM4 != 0) { sumNormSpeed += normSpeedM4; activeCount++; }
    
    float avgNormSpeed = activeCount > 1 ? (sumNormSpeed / activeCount) : 0.0;

    // Motor 1 PI loop
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

    // Motor 2 PI loop (Mirrored physical alignment)
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
      setMotorSpeedPwm(M2_IN1, M2_IN2, -constrain((int)pwmM2, -255, 255)); // Mirrored polarity
    }

    // Motor 3 PI loop
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

    // Motor 4 PI loop (Mirrored physical alignment)
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
      setMotorSpeedPwm(M4_IN1, M4_IN2, -constrain((int)pwmM4, -255, 255)); // Mirrored polarity
    }
  }
}

// Drive speed selection and safety timeout watchdog
void updateMotors() {
  if (positionModeM1) targetSpeedM1 = calculatePositionSpeed(encoderTicksM1, targetPositionM1, positionModeM1);
  if (positionModeM2) targetSpeedM2 = calculatePositionSpeed(encoderTicksM2, targetPositionM2, positionModeM2);
  if (positionModeM3) targetSpeedM3 = calculatePositionSpeed(encoderTicksM3, targetPositionM3, positionModeM3);
  if (positionModeM4) targetSpeedM4 = calculatePositionSpeed(encoderTicksM4, targetPositionM4, positionModeM4);

  // Watchdog timeout to halt wheels if serial command connection is dropped
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
// Web Dashboard HTML UI Template
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
  </style>
</head>
<body>
  <div class="container">
    <h1>Maker ESP32 Cockpit</h1>
    
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
        <button class="btn" style="flex: 1.2;" onclick="turnWheel('all')">Rotate All Wheels</button>
        <button class="btn stop" style="flex: 0.8;" onclick="stopPositionMode()">⚠️ ESTOP ROTATE</button>
      </div>
    </div>

    <div class="card">
      <h2>Status & Telemetry</h2>
      <div class="telemetry-row">
        <span>Battery:</span>
        <span id="tel-batt" class="val">Unknown</span>
      </div>
      <div class="telemetry-row">
        <span>M1 Ticks:</span>
        <span id="tel-e1" class="val">0</span>
      </div>
      <div class="telemetry-row">
        <span>M2 Ticks:</span>
        <span id="tel-e2" class="val">0</span>
      </div>
      <div class="telemetry-row">
        <span>M3 Ticks:</span>
        <span id="tel-e3" class="val">0</span>
      </div>
      <div class="telemetry-row">
        <span>M4 Ticks:</span>
        <span id="tel-e4" class="val">0</span>
      </div>
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
      else if (dir === 'spinleft') { m1=-spd; m2=spd; m3=-spd; m4=spd; }
      else if (dir === 'spinright') { m1=spd; m2=-spd; m3=spd; m4=-spd; }
      
      document.getElementById('m1').value = m1;
      document.getElementById('m2').value = m2;
      document.getElementById('m3').value = m3;
      document.getElementById('m4').value = m4;
      
      updateLabels(m1, m2, m3, m4);
      sendSpeed(m1, m2, m3, m4);
    }

    function updateLabels(m1, m2, m3, m4) {
      document.getElementById('val-m1').innerText = m1;
      document.getElementById('val-m2').innerText = m2;
      document.getElementById('val-m3').innerText = m3;
      document.getElementById('val-m4').innerText = m4;
    }

    function updateSpeed() {
      const m1 = document.getElementById('m1').value;
      const m2 = document.getElementById('m2').value;
      const m3 = document.getElementById('m3').value;
      const m4 = document.getElementById('m4').value;
      
      updateLabels(m1, m2, m3, m4);

      if (sendTimeout) clearTimeout(sendTimeout);
      sendTimeout = setTimeout(() => {
        sendSpeed(m1, m2, m3, m4);
      }, 50);
    }

    function turnWheel(target) {
      const turns = document.getElementById('num-turns').value;
      let query = (target === 'all') 
        ? `m1=${turns}&m2=${turns}&m3=${turns}&m4=${turns}` 
        : `${target}=${turns}`;
      fetch(`/api/turn?${query}`).catch(err => console.error(err));
    }

    function stopPositionMode() {
      fetch(`/api/turn?stop=1`).catch(err => console.error(err));
    }

    function sendSpeed(m1, m2, m3, m4) {
      currentM1 = m1; currentM2 = m2; currentM3 = m3; currentM4 = m4;
      fetch(`/api/motor?m1=${m1}&m2=${m2}&m3=${m3}&m4=${m4}`).catch(err => console.error(err));

      if (keepAliveInterval) clearInterval(keepAliveInterval);
      if (m1 != 0 || m2 != 0 || m3 != 0 || m4 != 0) {
        keepAliveInterval = setInterval(() => {
          fetch(`/api/motor?m1=${currentM1}&m2=${currentM2}&m3=${currentM3}&m4=${currentM4}`).catch(err => console.error(err));
        }, 400);
      }
    }

    setInterval(() => {
      fetch('/api/status')
        .then(res => res.json())
        .then(data => {
          document.getElementById('tel-batt').innerText = data.battery.toFixed(2) + " V";
          document.getElementById('tel-e1').innerText = data.e1;
          document.getElementById('tel-e2').innerText = data.e2;
          document.getElementById('tel-e3').innerText = data.e3;
          document.getElementById('tel-e4').innerText = data.e4;
        }).catch(err => {
          document.getElementById('tel-batt').innerText = "Offline";
        });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

// ────────────────────────────────────────────────────────────
// Cockpit REST API Route Handlers
// ────────────────────────────────────────────────────────────

void handleRoot() {
  server.send(200, "text/html", htmlContent);
}

void handleMotorApi() {
  positionModeM1 = false;
  positionModeM2 = false;
  positionModeM3 = false;
  positionModeM4 = false;

  if (server.hasArg("m1")) targetSpeedM1 = server.arg("m1").toInt();
  if (server.hasArg("m2")) targetSpeedM2 = server.arg("m2").toInt();
  if (server.hasArg("m3")) targetSpeedM3 = server.arg("m3").toInt();
  if (server.hasArg("m4")) targetSpeedM4 = server.arg("m4").toInt();
  
  lastCommandTime = millis(); // Refresh watchdog command timer
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTurnApi() {
  if (server.hasArg("stop") || server.hasArg("estop")) {
    positionModeM1 = false;
    positionModeM2 = false;
    positionModeM3 = false;
    positionModeM4 = false;
    targetSpeedM1 = 0; targetSpeedM2 = 0; targetSpeedM3 = 0; targetSpeedM4 = 0;
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
  json += "\"battery\":" + String(simulatedVoltage, 2);
  json += "}";
  server.send(200, "application/json", json);
}

// ────────────────────────────────────────────────────────────
// Host Binary Serial Packet Processor (Compatible Serial Emulation)
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
    return; // Drop corrupted serial frame
  }

  lastCommandTime = millis(); // Refresh watchdog command timer

  switch (funcId) {
    case 0x10: { // FUNC_MOTOR packet
      if (extLen >= 6) {
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
    case 0x02: { // FUNC_BEEP duration packet
      if (extLen >= 4) {
        uint16_t duration = payloadBuf[1] | (payloadBuf[2] << 8);
        if (STATUS_LED >= 0) {
          digitalWrite(STATUS_LED, LOW);
          delay(duration / 2);
          digitalWrite(STATUS_LED, HIGH);
        } else {
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
        if (b == 0xFF) parserState = WAIT_DEVICE;
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
  // 1. Encoder ticks telemetry (TYPE_ENCODER = 0x0D)
  uint8_t encoderData[16];
  memcpy(&encoderData[0],  (const void*)&encoderTicksM1, 4);
  memcpy(&encoderData[4],  (const void*)&encoderTicksM2, 4);
  memcpy(&encoderData[8],  (const void*)&encoderTicksM3, 4);
  memcpy(&encoderData[12], (const void*)&encoderTicksM4, 4);
  sendTelemetryPacket(0x0D, encoderData, 16);

  // Consume battery charge mathematically
  if (simulatedVoltage > 2.0) {
    float idleDraw = 0.0000014;
    float motorDraw = (abs(targetSpeedM1) + abs(targetSpeedM2) + abs(targetSpeedM3) + abs(targetSpeedM4)) * 0.0000003;
    simulatedVoltage -= (idleDraw + motorDraw);
    if (simulatedVoltage < 6.0) simulatedVoltage = 6.0; // clamp to 2S cut-off voltage
  } else {
    simulatedVoltage = 0.0;
  }

  // 2. Battery telemetry (TYPE_BATTERY = 0x0A)
  uint8_t battRaw = (uint8_t)(simulatedVoltage * 10.0);
  uint8_t batteryData[7] = {0, 0, 0, 0, 0, 0, battRaw}; // voltage * 10
  sendTelemetryPacket(0x0A, batteryData, 7);

  // 3. IMU mock telemetry (TYPE_IMU = 0x0E)
  int16_t leftSpeed = (targetSpeedM1 + targetSpeedM3) / 2;
  int16_t rightSpeed = (targetSpeedM2 + targetSpeedM4) / 2;
  int16_t turnRate = rightSpeed - leftSpeed;
  int16_t gz_raw = turnRate * -10;
  int16_t ax_raw = 0;
  int16_t ay_raw = 0;
  int16_t az_raw = 1000; // gravity
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
    
    // Static Line 1: Header
    display.setCursor(22, 0);
    display.print("COCKPIT");
    
    // Scrolling Line 2: IP Address / Connection
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
// Setup and Loop (Entry Points)
// ────────────────────────────────────────────────────────────

void setup() {
  // Initialization flash checklist sequence
  if (STATUS_LED >= 0) {
    pinMode(STATUS_LED, OUTPUT);
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(400);
      digitalWrite(STATUS_LED, LOW);
      delay(400);
    }
  }

  // Setup pin directions for motor control IN1/IN2 H-bridge
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT);

  // Initialize motor speed commands to 0
  updateMotors();

  // Attach and configure hardware PCNT counters
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoderM1.attachFullQuad(E1_A, E1_B); encoderM1.setFilter(1023);
  encoderM2.attachFullQuad(E2_A, E2_B); encoderM2.setFilter(1023);
  encoderM3.attachFullQuad(E3_A, E3_B); encoderM3.setFilter(1023);
  encoderM4.attachFullQuad(E4_B, E4_A); encoderM4.setFilter(1023);

  // Configure OLED display
  Wire.begin(21, 22); // I2C SDA=GPIO 21, SCL=GPIO 22
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(40, 0); display.print("WIFI");
    display.setCursor(4, 16);  display.print("CONNECT...");
    display.display();
  }

  // Serial link to Raspberry Pi 5
  Serial.begin(115200);

  // Connect to local WiFi network
  WiFi.begin(ssid, password);
  unsigned long startWifiTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifiTime < 12000) {
    if (STATUS_LED >= 0) {
      digitalWrite(STATUS_LED, HIGH); delay(150);
      digitalWrite(STATUS_LED, LOW);  delay(150);
    } else {
      delay(300);
    }
  }

  // Turn LED Solid on successful connection, off otherwise
  if (WiFi.status() == WL_CONNECTED) {
    if (STATUS_LED >= 0) digitalWrite(STATUS_LED, HIGH);
    
    // Register hostname
    if (MDNS.begin("maker-esp32")) {
      MDNS.addService("http", "tcp", 80);
    }
    
    // Register API endpoints
    server.on("/", handleRoot);
    server.on("/api/motor", handleMotorApi);
    server.on("/api/status", handleStatusApi);
    server.on("/api/turn", handleTurnApi);
    server.begin();

    if (oledOk) {
      scrollText = "   maker-esp32.local   [IP: " + WiFi.localIP().toString() + "]   ";
    }
  } else {
    if (STATUS_LED >= 0) digitalWrite(STATUS_LED, LOW);
    if (oledOk) {
      display.clearDisplay();
      display.setCursor(40, 0); display.print("WIFI");
      display.setCursor(22, 16); display.print("FAILED!");
      display.display();
    }
  }
  
  lastCommandTime = millis();
}

void loop() {
  // Update state from hardware PCNT encoder registers
  readEncoderTicks();

  // Handle cockpit client calls
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  // Process host control packages from Raspberry Pi 5
  parseSerialInput();

  // Watchdog timeout and position targeting evaluations
  updateMotors();

  // Velocity PI control step calculations
  updateSpeedPid();

  // Scroll details on display
  updateOledScroll();

  // Calculate speed feedback telemetry variables every 500ms
  unsigned long now = millis();
  if (now - lastSpeedCalcTime >= 500) {
    float dt = (now - lastSpeedCalcTime) / 1000.0;
    if (dt > 0.0) {
      int32_t delta1 = encoderTicksM1 - prevTicksM1;
      int32_t delta2 = encoderTicksM2 - prevTicksM2;
      int32_t delta3 = encoderTicksM3 - prevTicksM3;
      int32_t delta4 = encoderTicksM4 - prevTicksM4;
      
      prevTicksM1 = encoderTicksM1; prevTicksM2 = encoderTicksM2;
      prevTicksM3 = encoderTicksM3; prevTicksM4 = encoderTicksM4;
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

  // Telemetry stream output to serial interface at 20Hz
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}
