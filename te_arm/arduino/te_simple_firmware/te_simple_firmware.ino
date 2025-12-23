/*
 * TE Node - Simple Firmware (手)
 * ================================
 * Arduino Uno + OSEPP TB6612 Motor Shield (TBSHD-01)
 *
 * HARDWARE PIN MAPPING:
 * =====================
 * 
 * SERVO (on SVG header P7):
 *   - Signal: A0 (Analog pin 0)
 *   - Uses ServoTimer2 to avoid Timer1 conflict with motor PWM
 *
 * MOTOR DRIVER (TB6612 - upper chip, M1/M2):
 *   - M1: PWMA=D11, AIN1=D13, AIN2=D12
 *   - M2: PWMB=D10, BIN1=D8,  BIN2=D7
 *
 * MOTOR DRIVER (TB6612 - lower chip, M3/M4):
 *   - M3: PWMA=D6,  AIN1=D4,  AIN2=D9  (or similar - needs verification)
 *   - M4: PWMB=D5,  BIN1=D3,  BIN2=D2  (or similar - needs verification)
 *
 * TIMER NOTES:
 *   - Timer0: pins 5, 6 (used by millis/delay - don't touch)
 *   - Timer1: pins 9, 10 (used by Servo.h - CONFLICTS with M2 PWM!)
 *   - Timer2: pins 3, 11 (used by ServoTimer2 - affects M1 PWM on D11)
 *
 * TRADE-OFF: Using ServoTimer2 disables PWM on D11 (M1).
 *            If you only use M2/M3, this is fine.
 *            M2 (D10) and M3 (D6) remain fully functional.
 *
 * Protocol: Serial JSON at 115200 baud
 */

/*
 * NOTE ON TIMER CONFLICT:
 * Servo.h uses Timer1, which disables PWM on pins 9 and 10.
 * - M2 (PWM=D10) will NOT have speed control (on/off only)
 * - M3 (PWM=D6) WILL work fine (D6 uses Timer0)
 * 
 * If you need M2 speed control, install ServoTimer2 manually.
 */
#include <Servo.h>        // Standard Servo library
#include <ArduinoJson.h>

// =============================================================================
// Pin Configuration - 7 Servos
// =============================================================================

// Arm servos on Analog pins (SVG headers P7-P12)
#define BASE_PIN         A0   // P7 header
#define SHOULDER_PIN     A1   // P8 header
#define ELBOW_PIN        A2   // P9 header
#define WRIST_BEND_PIN   A3   // P10 header
#define WRIST_SWIVEL_PIN A4   // P11 header
#define PINCHER_PIN      A5   // P12 header

// Head servo on Digital pin D2 (SVG position 3 on this shield)
#define HEAD_PAN_PIN     2    // Digital pin 2

#define NUM_SERVOS 7

// MOTOR2 (M2 terminals) - upper TB6612, Motor B
// This is likely your wheel motor - uses Timer1-safe pins
#define M2_PWM  10    // PWM (Timer1 - preserved by ServoTimer2!)
#define M2_IN1  8     // Direction
#define M2_IN2  7     // Direction

// MOTOR1 (M1 terminals) - upper TB6612, Motor A  
// WARNING: D11 PWM may be affected by ServoTimer2 (Timer2)
#define M1_PWM  11
#define M1_IN1  13
#define M1_IN2  12

// MOTOR3 (M3 terminals) - lower TB6612, Motor A (if present)
// These pins are guesses - verify with your hardware
#define M3_PWM  6     // PWM (Timer0 - always works)
#define M3_IN1  4
#define M3_IN2  9

// =============================================================================
// Configuration
// =============================================================================

#define SERIAL_BAUD 115200
#define JSON_BUFFER_SIZE 256

// Motor safety defaults
#define DEFAULT_PWM  120     // gentle power (0-255)
#define DEFAULT_MS   150     // pulse duration in ms

// =============================================================================
// Global State
// =============================================================================

Servo servos[NUM_SERVOS];
int angles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90, 90};

// Servo names for JSON responses
const char* servoNames[NUM_SERVOS] = {
  "base", "shoulder", "elbow", "wrist_bend", "wrist_swivel", "pincher", "head_pan"
};
String inputBuffer = "";

// Motor state tracking
struct MotorState {
  int lastDir;
  int lastPwm;
  int lastMs;
};
MotorState m1State = {0, 0, 0};
MotorState m2State = {0, 0, 0};
MotorState m3State = {0, 0, 0};

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { ; }
  
  // Initialize all 7 servos
  const int pins[NUM_SERVOS] = {BASE_PIN, SHOULDER_PIN, ELBOW_PIN, WRIST_BEND_PIN, WRIST_SWIVEL_PIN, PINCHER_PIN, HEAD_PAN_PIN};
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(pins[i]);
    servos[i].write(90);
    angles[i] = 90;
  }
  
  // Initialize M2 motor pins (your wheel motor)
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  analogWrite(M2_PWM, 0);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  
  // Initialize M1 motor pins (backup/diagnostic)
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  analogWrite(M1_PWM, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  
  // Initialize M3 motor pins (if using lower TB6612)
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  analogWrite(M3_PWM, 0);
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  
  Serial.println("{\"event\":\"startup\",\"node\":\"te_simple\",\"servos\":6,\"motor\":\"M3\"}");
}

// =============================================================================
// Motor Control Functions
// =============================================================================

void motorRun(int pwmPin, int in1Pin, int in2Pin, int dir, int pwm, int ms, MotorState* state) {
  dir = (dir >= 0) ? 1 : -1;
  pwm = constrain(pwm, 0, 255);
  ms = constrain(ms, 10, 2000);
  
  // Set direction
  if (dir > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  
  // Run motor
  analogWrite(pwmPin, pwm);
  delay(ms);
  
  // Stop
  analogWrite(pwmPin, 0);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  
  // Track state
  state->lastDir = dir;
  state->lastPwm = pwm;
  state->lastMs = ms;
}

void motorStop(int pwmPin, int in1Pin, int in2Pin) {
  analogWrite(pwmPin, 0);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}

// Convenience wrappers
void m2Run(int dir, int pwm, int ms) {
  motorRun(M2_PWM, M2_IN1, M2_IN2, dir, pwm, ms, &m2State);
}
void m2Stop() { motorStop(M2_PWM, M2_IN1, M2_IN2); }

void m1Run(int dir, int pwm, int ms) {
  motorRun(M1_PWM, M1_IN1, M1_IN2, dir, pwm, ms, &m1State);
}
void m1Stop() { motorStop(M1_PWM, M1_IN1, M1_IN2); }

void m3Run(int dir, int pwm, int ms) {
  motorRun(M3_PWM, M3_IN1, M3_IN2, dir, pwm, ms, &m3State);
}
void m3Stop() { motorStop(M3_PWM, M3_IN1, M3_IN2); }

// =============================================================================
// Servo Control
// =============================================================================

void moveServo(int idx, int angle) {
  if (idx < 0 || idx >= NUM_SERVOS) return;
  angle = constrain(angle, 0, 180);
  
  int current = angles[idx];
  int step = (angle > current) ? 1 : -1;
  
  while (current != angle) {
    current += step;
    servos[idx].write(current);
    delay(15);
  }
  angles[idx] = angle;
}

// Convenience functions
void moveBase(int angle) { moveServo(0, angle); }
void moveShoulder(int angle) { moveServo(1, angle); }
void moveElbow(int angle) { moveServo(2, angle); }
void moveWristBend(int angle) { moveServo(3, angle); }
void moveWristSwivel(int angle) { moveServo(4, angle); }
void movePincher(int angle) { moveServo(5, angle); }
void moveHeadPan(int angle) { moveServo(6, angle); }

// =============================================================================
// Response Helper
// =============================================================================

void sendResponse(bool ok, const char* error = nullptr) {
  StaticJsonDocument<512> doc;
  doc["ok"] = ok;
  
  if (error) {
    doc["error"] = error;
  }
  
  // All servo angles
  doc["base"] = angles[0];
  doc["shoulder"] = angles[1];
  doc["elbow"] = angles[2];
  doc["wrist_bend"] = angles[3];
  doc["wrist_swivel"] = angles[4];
  doc["pincher"] = angles[5];
  doc["head_pan"] = angles[6];
  
  // Motor state
  JsonObject m3 = doc.createNestedObject("m3");
  m3["dir"] = m3State.lastDir;
  m3["pwm"] = m3State.lastPwm;
  m3["ms"] = m3State.lastMs;
  
  // Joints array for compatibility
  JsonArray joints = doc.createNestedArray("joints");
  for (int i = 0; i < NUM_SERVOS; i++) {
    joints.add(angles[i]);
  }
  doc["enabled"] = true;
  
  serializeJson(doc, Serial);
  Serial.println();
}

// =============================================================================
// Command Processing
// =============================================================================

void processCommand(const String& input) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  DeserializationError err = deserializeJson(doc, input);
  
  if (err) {
    sendResponse(false, "JSON parse error");
    return;
  }
  
  const char* cmd = doc["cmd"];
  if (!cmd) {
    sendResponse(false, "Missing cmd");
    return;
  }
  
  // === STATUS ===
  if (strcmp(cmd, "status") == 0) {
    sendResponse(true);
  }
  
  // === SERVO COMMANDS ===
  else if (strcmp(cmd, "base") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveBase(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "shoulder") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveShoulder(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "elbow") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveElbow(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "wrist_bend") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveWristBend(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "wrist_swivel") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveWristSwivel(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "pincher") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    movePincher(angle);
    sendResponse(true);
  }
  else if (strcmp(cmd, "head_pan") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveHeadPan(angle);
    sendResponse(true);
  }
  // Generic servo by index
  else if (strcmp(cmd, "servo") == 0) {
    int idx = doc["idx"] | -1;
    int angle = doc["angle"] | -1;
    if (idx < 0 || idx >= NUM_SERVOS) { sendResponse(false, "idx 0-5"); return; }
    if (angle < 0 || angle > 180) { sendResponse(false, "angle 0-180"); return; }
    moveServo(idx, angle);
    sendResponse(true);
  }
  
  // === MOTOR M2 (your wheel - primary) ===
  else if (strcmp(cmd, "m2") == 0 || strcmp(cmd, "motor") == 0) {
    int dir = doc["dir"] | 1;
    int pwm = doc["pwm"] | DEFAULT_PWM;
    int ms  = doc["ms"]  | DEFAULT_MS;
    m2Run(dir, pwm, ms);
    sendResponse(true);
  }
  
  else if (strcmp(cmd, "m2_stop") == 0 || strcmp(cmd, "motor_stop") == 0) {
    m2Stop();
    sendResponse(true);
  }
  
  // === MOTOR M1 (diagnostic) ===
  else if (strcmp(cmd, "m1") == 0) {
    int dir = doc["dir"] | 1;
    int pwm = doc["pwm"] | DEFAULT_PWM;
    int ms  = doc["ms"]  | DEFAULT_MS;
    m1Run(dir, pwm, ms);
    sendResponse(true);
  }
  
  else if (strcmp(cmd, "m1_stop") == 0) {
    m1Stop();
    sendResponse(true);
  }
  
  // === MOTOR M3 (diagnostic - lower TB6612) ===
  else if (strcmp(cmd, "m3") == 0) {
    int dir = doc["dir"] | 1;
    int pwm = doc["pwm"] | DEFAULT_PWM;
    int ms  = doc["ms"]  | DEFAULT_MS;
    m3Run(dir, pwm, ms);
    sendResponse(true);
  }
  
  else if (strcmp(cmd, "m3_stop") == 0) {
    m3Stop();
    sendResponse(true);
  }
  
  // === HOME ===
  else if (strcmp(cmd, "home") == 0) {
    for (int i = 0; i < NUM_SERVOS; i++) {
      moveServo(i, 90);
    }
    sendResponse(true);
  }
  
  // === LEGACY MOVE (6-joint) ===
  else if (strcmp(cmd, "move") == 0) {
    JsonArray joints = doc["joints"];
    if (!joints.isNull()) {
      if (joints.size() >= 1) {
        moveBase(constrain(joints[0].as<int>(), 0, 180));
      }
      if (joints.size() >= 2) {
        moveShoulder(constrain(joints[1].as<int>(), 0, 180));
      }
    }
    sendResponse(true);
  }
  
  // === ENABLE/DISABLE (no-op) ===
  else if (strcmp(cmd, "enable") == 0 || strcmp(cmd, "disable") == 0) {
    sendResponse(true);
  }
  
  else {
    sendResponse(false, "Unknown command");
  }
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > JSON_BUFFER_SIZE) {
        inputBuffer = "";
        sendResponse(false, "Buffer overflow");
      }
    }
  }
}

