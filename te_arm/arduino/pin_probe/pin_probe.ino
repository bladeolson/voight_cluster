/*
 * Pin Probe - Find correct servo and motor pins
 * 
 * This cycles through pins to help identify which one controls what.
 * Watch the serial monitor for which pin is being tested.
 */

#include <Servo.h>

Servo testServo;

// Pins to test for servo (avoiding 0,1 for serial)
const int SERVO_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int NUM_SERVO_PINS = 12;

// Motor test combinations (PWM pin, IN1, IN2)
// PWM must be 3,5,6,9,10,11 on Uno
struct MotorPins {
  int pwm;
  int in1;
  int in2;
};

const MotorPins MOTOR_TESTS[] = {
  // M3 guesses based on schematic (lower TB6612)
  {6, 4, 9},   // My original guess
  {5, 4, 9},   // Alt PWM
  {3, 4, 9},   // Alt PWM
  {6, 3, 4},   // Alt direction pins
  {5, 3, 4},   
  {9, 4, 3},
  {6, 9, 4},   // Swapped direction
  // M1/M2 for comparison (upper TB6612)
  {11, 13, 12}, // M1
  {10, 8, 7},   // M2
};
const int NUM_MOTOR_TESTS = 9;

int currentTest = 0;
String mode = "idle";  // "servo", "motor", "idle"
int servoIdx = 0;
int motorIdx = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println("{\"event\":\"pin_probe_ready\"}");
  Serial.println("Commands:");
  Serial.println("  s = Start servo pin scan (watch for servo movement)");
  Serial.println("  m = Start motor pin scan (watch for wheel movement)");
  Serial.println("  n = Next pin/combo");
  Serial.println("  t = Test current again");
  Serial.println("  x = Stop");
}

void testServoPin(int pin) {
  // Detach from any previous pin
  testServo.detach();
  delay(50);
  
  // Attach to new pin
  testServo.attach(pin);
  
  Serial.print("{\"testing\":\"servo\",\"pin\":");
  Serial.print(pin);
  Serial.println(",\"action\":\"wiggle\"}");
  
  // Wiggle: 90 -> 60 -> 120 -> 90
  testServo.write(90);
  delay(300);
  testServo.write(60);
  delay(400);
  testServo.write(120);
  delay(400);
  testServo.write(90);
  delay(300);
  
  Serial.print("{\"tested\":\"servo\",\"pin\":");
  Serial.print(pin);
  Serial.println("}");
}

void testMotorPins(int pwm, int in1, int in2) {
  // Configure pins
  pinMode(pwm, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  Serial.print("{\"testing\":\"motor\",\"pwm\":");
  Serial.print(pwm);
  Serial.print(",\"in1\":");
  Serial.print(in1);
  Serial.print(",\"in2\":");
  Serial.print(in2);
  Serial.println("}");
  
  // Forward pulse
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 150);
  delay(300);
  
  // Stop
  analogWrite(pwm, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(200);
  
  // Reverse pulse
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(pwm, 150);
  delay(300);
  
  // Stop
  analogWrite(pwm, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  Serial.print("{\"tested\":\"motor\",\"pwm\":");
  Serial.print(pwm);
  Serial.print(",\"in1\":");
  Serial.print(in1);
  Serial.print(",\"in2\":");
  Serial.print(in2);
  Serial.println("}");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 's') {
      mode = "servo";
      servoIdx = 0;
      Serial.println("{\"mode\":\"servo_scan\",\"total_pins\":12}");
      testServoPin(SERVO_PINS[servoIdx]);
    }
    else if (c == 'm') {
      mode = "motor";
      motorIdx = 0;
      Serial.println("{\"mode\":\"motor_scan\",\"total_combos\":9}");
      testMotorPins(MOTOR_TESTS[motorIdx].pwm, 
                    MOTOR_TESTS[motorIdx].in1, 
                    MOTOR_TESTS[motorIdx].in2);
    }
    else if (c == 'n') {
      if (mode == "servo") {
        servoIdx = (servoIdx + 1) % NUM_SERVO_PINS;
        testServoPin(SERVO_PINS[servoIdx]);
      }
      else if (mode == "motor") {
        motorIdx = (motorIdx + 1) % NUM_MOTOR_TESTS;
        testMotorPins(MOTOR_TESTS[motorIdx].pwm,
                      MOTOR_TESTS[motorIdx].in1,
                      MOTOR_TESTS[motorIdx].in2);
      }
    }
    else if (c == 't') {
      if (mode == "servo") {
        testServoPin(SERVO_PINS[servoIdx]);
      }
      else if (mode == "motor") {
        testMotorPins(MOTOR_TESTS[motorIdx].pwm,
                      MOTOR_TESTS[motorIdx].in1,
                      MOTOR_TESTS[motorIdx].in2);
      }
    }
    else if (c == 'x') {
      mode = "idle";
      testServo.detach();
      // Turn off all motor pins
      for (int i = 0; i < NUM_MOTOR_TESTS; i++) {
        analogWrite(MOTOR_TESTS[i].pwm, 0);
        digitalWrite(MOTOR_TESTS[i].in1, LOW);
        digitalWrite(MOTOR_TESTS[i].in2, LOW);
      }
      Serial.println("{\"mode\":\"stopped\"}");
    }
  }
}

