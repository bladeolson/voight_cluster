/*
 * TE Node - Robot Arm Firmware (手 - Hand)
 * =========================================
 * 6-DOF Servo Arm Controller for VOIGHT CLUSTER
 * 
 * Protocol: Serial JSON commands at 115200 baud
 * 
 * Commands:
 *   {"cmd":"move","joints":[j0,j1,j2,j3,j4,j5]}  - Move to joint positions (degrees)
 *   {"cmd":"home"}                                - Move to home position
 *   {"cmd":"status"}                              - Report current positions
 *   {"cmd":"enable"}                              - Enable servos (attach)
 *   {"cmd":"disable"}                             - Disable servos (detach)
 *   {"cmd":"grip","value":0-180}                  - Control gripper directly
 * 
 * Response:
 *   {"ok":true,"joints":[j0,j1,j2,j3,j4,j5]}
 *   {"ok":false,"error":"message"}
 */

#include <Servo.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#define NUM_JOINTS 6
#define SERIAL_BAUD 115200
#define JSON_BUFFER_SIZE 256

// Servo pins - adjust to your wiring
// Pin mapping for OSEPP TB6612 SVG headers
// Positions 0-5 on the SVG header = Arduino pins 2-7
const int SERVO_PINS[NUM_JOINTS] = {2, 3, 4, 5, 6, 7};

// Joint names for reference
const char* JOINT_NAMES[NUM_JOINTS] = {
  "base",      // J0 - Base rotation
  "shoulder",  // J1 - Shoulder
  "elbow",     // J2 - Elbow
  "wrist_pitch", // J3 - Wrist pitch
  "wrist_roll",  // J4 - Wrist roll
  "gripper"    // J5 - Gripper
};

// Joint limits (degrees) - adjust to your arm's physical limits
const int JOINT_MIN[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
const int JOINT_MAX[NUM_JOINTS] = {180, 180, 180, 180, 180, 180};

// Home position (degrees)
const int HOME_POS[NUM_JOINTS] = {90, 90, 90, 90, 90, 90};

// Movement speed control (microseconds delay between degree steps)
#define MOVE_DELAY_US 15000  // 15ms per degree step for smooth motion

// -----------------------------------------------------------------------------
// Global State
// -----------------------------------------------------------------------------

Servo servos[NUM_JOINTS];
int currentPos[NUM_JOINTS];      // Current position in degrees
int targetPos[NUM_JOINTS];       // Target position in degrees
bool servosEnabled = false;
String inputBuffer = "";

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  // Initialize positions to home
  for (int i = 0; i < NUM_JOINTS; i++) {
    currentPos[i] = HOME_POS[i];
    targetPos[i] = HOME_POS[i];
  }
  
  // Send startup message
  Serial.println("{\"event\":\"startup\",\"node\":\"te\",\"joints\":6}");
  
  // Auto-enable servos and go to home
  enableServos();
  moveToHome();
}

// -----------------------------------------------------------------------------
// Servo Control Functions
// -----------------------------------------------------------------------------

void enableServos() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(currentPos[i]);
  }
  servosEnabled = true;
}

void disableServos() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    servos[i].detach();
  }
  servosEnabled = false;
}

int clampAngle(int joint, int angle) {
  if (angle < JOINT_MIN[joint]) return JOINT_MIN[joint];
  if (angle > JOINT_MAX[joint]) return JOINT_MAX[joint];
  return angle;
}

void moveToHome() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    targetPos[i] = HOME_POS[i];
  }
  executeSmoothMove();
}

void executeSmoothMove() {
  if (!servosEnabled) {
    enableServos();
  }
  
  // Calculate max steps needed
  int maxSteps = 0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    int steps = abs(targetPos[i] - currentPos[i]);
    if (steps > maxSteps) maxSteps = steps;
  }
  
  // Move incrementally for smooth motion
  for (int step = 0; step < maxSteps; step++) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      if (currentPos[i] < targetPos[i]) {
        currentPos[i]++;
        servos[i].write(currentPos[i]);
      } else if (currentPos[i] > targetPos[i]) {
        currentPos[i]--;
        servos[i].write(currentPos[i]);
      }
    }
    delayMicroseconds(MOVE_DELAY_US);
  }
  
  // Ensure final positions are exact
  for (int i = 0; i < NUM_JOINTS; i++) {
    currentPos[i] = targetPos[i];
    servos[i].write(currentPos[i]);
  }
}

void moveImmediate() {
  if (!servosEnabled) {
    enableServos();
  }
  
  for (int i = 0; i < NUM_JOINTS; i++) {
    currentPos[i] = targetPos[i];
    servos[i].write(currentPos[i]);
  }
}

// -----------------------------------------------------------------------------
// Command Processing
// -----------------------------------------------------------------------------

void sendResponse(bool ok, const char* error = nullptr) {
  StaticJsonDocument<JSON_BUFFER_SIZE> doc;
  doc["ok"] = ok;
  
  if (error != nullptr) {
    doc["error"] = error;
  }
  
  JsonArray joints = doc.createNestedArray("joints");
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints.add(currentPos[i]);
  }
  
  doc["enabled"] = servosEnabled;
  
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
  if (cmd == nullptr) {
    sendResponse(false, "Missing 'cmd' field");
    return;
  }
  
  // Handle commands
  if (strcmp(cmd, "status") == 0) {
    sendResponse(true);
  }
  else if (strcmp(cmd, "home") == 0) {
    moveToHome();
    sendResponse(true);
  }
  else if (strcmp(cmd, "enable") == 0) {
    enableServos();
    sendResponse(true);
  }
  else if (strcmp(cmd, "disable") == 0) {
    disableServos();
    sendResponse(true);
  }
  else if (strcmp(cmd, "move") == 0) {
    JsonArray joints = doc["joints"];
    if (joints.isNull() || joints.size() != NUM_JOINTS) {
      sendResponse(false, "Invalid joints array (need 6 values)");
      return;
    }
    
    for (int i = 0; i < NUM_JOINTS; i++) {
      targetPos[i] = clampAngle(i, joints[i].as<int>());
    }
    
    // Check for smooth vs immediate mode
    bool smooth = doc["smooth"] | true;  // Default to smooth
    if (smooth) {
      executeSmoothMove();
    } else {
      moveImmediate();
    }
    
    sendResponse(true);
  }
  else if (strcmp(cmd, "grip") == 0) {
    int value = doc["value"] | -1;
    if (value < 0 || value > 180) {
      sendResponse(false, "Gripper value must be 0-180");
      return;
    }
    targetPos[5] = clampAngle(5, value);
    currentPos[5] = targetPos[5];
    if (servosEnabled) {
      servos[5].write(currentPos[5]);
    }
    sendResponse(true);
  }
  else {
    sendResponse(false, "Unknown command");
  }
}

// -----------------------------------------------------------------------------
// Main Loop
// -----------------------------------------------------------------------------

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
      
      // Prevent buffer overflow
      if (inputBuffer.length() > JSON_BUFFER_SIZE) {
        inputBuffer = "";
        sendResponse(false, "Input buffer overflow");
      }
    }
  }
}

