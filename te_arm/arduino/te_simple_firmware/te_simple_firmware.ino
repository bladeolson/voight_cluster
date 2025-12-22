/*
 * TE Node - Simple Firmware (手)
 * ================================
 * BASE servo + DC Motor on M2 (OSEPP TB6612 Motor+Servo Shield on Uno)
 *
 * Hardware:
 *   - Arduino Uno + OSEPP TB6612 Motor/Servo Shield
 *   - BASE servo on SVG0 (Arduino pin 2)
 *   - DC motor on M2+ / M2- (TB6612 Motor B)
 *
 * Protocol: Serial JSON at 115200 baud
 *
 * Commands:
 *   {"cmd":"status"}                               - Report status
 *   {"cmd":"base","angle":90}                      - Move base servo (0-180)
 *   {"cmd":"m2","dir":-1|1,"pwm":0..255,"ms":50}   - Run DC motor M2 for ms (safe pulse)
 *   {"cmd":"m2_stop"}                              - Stop motor M2
 *   {"cmd":"move","joints":[j0..j5]}               - Legacy 6-joint move (only uses joint 0)
 */

#include <Servo.h>
#include <ArduinoJson.h>

// =============================================================================
// Pin Configuration - OSEPP TBSHD-01 (from schematic)
// =============================================================================

// BASE Servo - SVG header position 0 = Arduino pin 2
#define BASE_SERVO_PIN 2

// MOTOR2 (M2 terminals) on the upper TB6612 (from schematic JP7 mapping):
// - DIR2+  -> Arduino D8
// - DIR2-  -> Arduino D7
// - PWM1B  -> Arduino D10 (PWM)
#define PWMB  10    // PWM1B
#define BIN1  8     // DIR2+
#define BIN2  7     // DIR2-

// MOTOR1 (M1 terminals) on the upper TB6612 (from schematic JP7 mapping):
// - DIR1+  -> Arduino D13
// - DIR1-  -> Arduino D12
// - PWM1A  -> Arduino D11 (PWM)
#define PWMA  11    // PWM1A
#define AIN1  13    // DIR1+
#define AIN2  12    // DIR1-

// =============================================================================
// Temporary pin-scan mode (find correct M2 mapping)
// =============================================================================
// Set to 1 only if pin mapping is unknown.
#define M2_PIN_SCAN_MODE 0

// =============================================================================
// Configuration
// =============================================================================

#define SERIAL_BAUD 115200
#define JSON_BUFFER_SIZE 256

// Motor safety defaults
#define DEFAULT_M2_PWM  120     // gentle power
#define DEFAULT_M2_MS   120     // short pulse

// Pin-scan safety defaults
#define SCAN_PWM  100
#define SCAN_MS   120
#define SCAN_PAUSE_MS 900

// =============================================================================
// Global State
// =============================================================================

Servo baseServo;
int baseAngle = 90;
String inputBuffer = "";
bool m2Running = false;
int lastM2Dir = 0;
int lastM2Pwm = 0;
int lastM2Ms = 0;

bool m1Running = false;
int lastM1Dir = 0;
int lastM1Pwm = 0;
int lastM1Ms = 0;

#if M2_PIN_SCAN_MODE
struct M2Candidate {
  uint8_t stby;
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

// Candidate pins on Arduino Uno we’re willing to test.
// PWM-capable: 3,5,6,9,10,11. Avoid 2 (servo), 0/1 (serial).
const uint8_t _PWM_CANDS[]  = {3, 5, 6, 9, 10, 11};
const uint8_t _STBY_CANDS[] = {4, 7, 8, 12, 13};

// Common direction pin pairs seen on TB6612-based shields.
const uint8_t _DIR_PAIRS[][2] = {
  {7, 8},
  {8, 7},
  {9, 10},
  {10, 9},
  {12, 13},
  {13, 12},
};

// Precomputed candidate list (built at runtime from the arrays above)
M2Candidate _cands[64];
uint8_t _candCount = 0;
uint8_t _candIdx = 0;

void buildCandidates() {
  _candCount = 0;
  for (uint8_t si = 0; si < (sizeof(_STBY_CANDS) / sizeof(_STBY_CANDS[0])); si++) {
    for (uint8_t pi = 0; pi < (sizeof(_PWM_CANDS) / sizeof(_PWM_CANDS[0])); pi++) {
      for (uint8_t di = 0; di < (sizeof(_DIR_PAIRS) / sizeof(_DIR_PAIRS[0])); di++) {
        uint8_t stby = _STBY_CANDS[si];
        uint8_t pwm  = _PWM_CANDS[pi];
        uint8_t in1  = _DIR_PAIRS[di][0];
        uint8_t in2  = _DIR_PAIRS[di][1];
        // Avoid collisions (all pins must be distinct and not the servo pin)
        if (stby == BASE_SERVO_PIN || pwm == BASE_SERVO_PIN || in1 == BASE_SERVO_PIN || in2 == BASE_SERVO_PIN) continue;
        if (stby == pwm || stby == in1 || stby == in2) continue;
        if (pwm == in1 || pwm == in2) continue;
        if (in1 == in2) continue;
        if (_candCount < (sizeof(_cands) / sizeof(_cands[0]))) {
          _cands[_candCount++] = {stby, pwm, in1, in2};
        }
      }
    }
  }
}
#endif

// =============================================================================
// Setup
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) { ; }
  
  // Initialize BASE servo
  baseServo.attach(BASE_SERVO_PIN);
  baseServo.write(baseAngle);
  
#if M2_PIN_SCAN_MODE
  pinMode(LED_BUILTIN, OUTPUT);
  buildCandidates();
  Serial.println("{\"event\":\"startup\",\"node\":\"te_simple\",\"mode\":\"m2_pin_scan\"}");
  Serial.print("M2 scan candidates: ");
  Serial.println(_candCount);
  Serial.println("Watching for motor bump. Note candidate index printed before each pulse.");
  return;
#endif

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  // Motor off initially
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  
  Serial.println("{\"event\":\"startup\",\"node\":\"te_simple\",\"base\":90,\"m2\":true}");
}

// =============================================================================
// DC Motor M2 control (safe pulses)
// =============================================================================

void m2Stop() {
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  m2Running = false;
  // Keep lastM2Dir/lastM2Pwm/lastM2Ms for diagnostics
}

void m2Run(int dir, int pwm, int ms) {
  dir = (dir >= 0) ? 1 : -1;
  pwm = constrain(pwm, 0, 255);
  ms = constrain(ms, 10, 2000);

  // Direction
  if (dir > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMB, pwm);
  m2Running = true;
  lastM2Dir = dir;
  lastM2Pwm = pwm;
  lastM2Ms = ms;

  delay(ms);
  m2Stop();
}

// =============================================================================
// DC Motor M1 control (for diagnosis if motor is on MOTOR1 terminals)
// =============================================================================

void m1Stop() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  m1Running = false;
  // Keep lastM1* for diagnostics
}

void m1Run(int dir, int pwm, int ms) {
  dir = (dir >= 0) ? 1 : -1;
  pwm = constrain(pwm, 0, 255);
  ms = constrain(ms, 10, 2000);

  if (dir > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMA, pwm);
  m1Running = true;
  lastM1Dir = dir;
  lastM1Pwm = pwm;
  lastM1Ms = ms;

  delay(ms);
  m1Stop();
}

// =============================================================================
// BASE Servo Control
// =============================================================================

void moveBase(int angle) {
  angle = constrain(angle, 0, 180);
  
  // Smooth move
  int current = baseAngle;
  int dir = (angle > current) ? 1 : -1;
  
  while (current != angle) {
    current += dir;
    baseServo.write(current);
    delay(10);  // Smooth movement
  }
  
  baseAngle = angle;
}

// =============================================================================
// Command Processing
// =============================================================================

void sendResponse(bool ok, const char* error = nullptr) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  doc["ok"] = ok;
  
  if (error) {
    doc["error"] = error;
  }
  
  doc["base_angle"] = baseAngle;
  doc["m2_running"] = m2Running;
  doc["m2_dir"] = lastM2Dir;
  doc["m2_pwm"] = lastM2Pwm;
  doc["m2_ms"] = lastM2Ms;

  doc["m1_running"] = m1Running;
  doc["m1_dir"] = lastM1Dir;
  doc["m1_pwm"] = lastM1Pwm;
  doc["m1_ms"] = lastM1Ms;
  
  // Legacy joints array (for compatibility)
  JsonArray joints = doc.createNestedArray("joints");
  joints.add(baseAngle);
  for (int i = 1; i < 6; i++) joints.add(90);
  
  doc["enabled"] = true;
  
  serializeJson(doc, Serial);
  Serial.println();
}

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
  
  // === BASE SERVO ===
  else if (strcmp(cmd, "base") == 0) {
    int angle = doc["angle"] | -1;
    if (angle < 0 || angle > 180) {
      sendResponse(false, "angle must be 0-180");
      return;
    }
    moveBase(angle);
    sendResponse(true);
  }

  // === DC Motor M2 ===
  else if (strcmp(cmd, "m2") == 0) {
    int dir = doc["dir"] | 1;
    int pwm = doc["pwm"] | DEFAULT_M2_PWM;
    int ms  = doc["ms"]  | DEFAULT_M2_MS;
    m2Run(dir, pwm, ms);
    sendResponse(true);
  }

  else if (strcmp(cmd, "m2_stop") == 0) {
    m2Stop();
    sendResponse(true);
  }

  // === DC Motor M1 (diagnostic) ===
  else if (strcmp(cmd, "m1") == 0) {
    int dir = doc["dir"] | 1;
    int pwm = doc["pwm"] | DEFAULT_M2_PWM;
    int ms  = doc["ms"]  | DEFAULT_M2_MS;
    m1Run(dir, pwm, ms);
    sendResponse(true);
  }

  else if (strcmp(cmd, "m1_stop") == 0) {
    m1Stop();
    sendResponse(true);
  }
  
  // === LEGACY MOVE (6-joint) ===
  else if (strcmp(cmd, "move") == 0) {
    JsonArray joints = doc["joints"];
    if (!joints.isNull() && joints.size() >= 1) {
      int angle = joints[0].as<int>();
      moveBase(constrain(angle, 0, 180));
    }
    sendResponse(true);
  }
  
  // === HOME ===
  else if (strcmp(cmd, "home") == 0) {
    moveBase(90);
    sendResponse(true);
  }
  
  // === ENABLE/DISABLE (no-op for now) ===
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
#if M2_PIN_SCAN_MODE
  if (_candCount == 0) {
    delay(500);
    return;
  }
  const M2Candidate c = _cands[_candIdx];
  _candIdx = (_candIdx + 1) % _candCount;

  // Configure pins for this candidate
  pinMode(c.stby, OUTPUT);
  pinMode(c.pwm, OUTPUT);
  pinMode(c.in1, OUTPUT);
  pinMode(c.in2, OUTPUT);

  digitalWrite(c.stby, HIGH);

  // Print candidate details
  Serial.print("CAND ");
  Serial.print(_candIdx == 0 ? (_candCount - 1) : (_candIdx - 1));
  Serial.print(" stby=");
  Serial.print(c.stby);
  Serial.print(" pwm=");
  Serial.print(c.pwm);
  Serial.print(" in1=");
  Serial.print(c.in1);
  Serial.print(" in2=");
  Serial.println(c.in2);

  // Forward pulse
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(c.in1, HIGH);
  digitalWrite(c.in2, LOW);
  analogWrite(c.pwm, SCAN_PWM);
  delay(SCAN_MS);
  analogWrite(c.pwm, 0);
  digitalWrite(c.in1, LOW);
  digitalWrite(c.in2, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(SCAN_PAUSE_MS);

  // Reverse pulse
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(c.in1, LOW);
  digitalWrite(c.in2, HIGH);
  analogWrite(c.pwm, SCAN_PWM);
  delay(SCAN_MS);
  analogWrite(c.pwm, 0);
  digitalWrite(c.in1, LOW);
  digitalWrite(c.in2, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(SCAN_PAUSE_MS);

  return;
#endif

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

