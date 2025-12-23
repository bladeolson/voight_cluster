/*
 * SIMPLE TEST - No JSON, No Server
 * Just wiggles servo on A0 and motor on M3 automatically
 * 
 * Watch: LED blinks, servo wiggles, motor pulses
 */

#include <Servo.h>

Servo myServo;

// Servo on P7 header = A0
#define SERVO_PIN A0

// Motor M3 - TESTING DIFFERENT PIN COMBOS
// Try these one at a time if motor doesn't work
#define MOTOR_PWM 6
#define MOTOR_IN1 4
#define MOTOR_IN2 9

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Servo setup
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  
  // Motor setup
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  
  Serial.println("=== SIMPLE TEST STARTING ===");
  Serial.println("Servo on A0, Motor on D6/D4/D9");
  delay(2000);
}

void loop() {
  // --- BLINK LED ---
  Serial.println("LED ON");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  
  // --- SERVO TEST ---
  Serial.println("SERVO: 90 -> 45");
  myServo.write(45);
  delay(1000);
  
  Serial.println("SERVO: 45 -> 135");
  myServo.write(135);
  delay(1000);
  
  Serial.println("SERVO: 135 -> 90");
  myServo.write(90);
  delay(1000);
  
  // --- MOTOR TEST ---
  Serial.println("MOTOR: Forward pulse");
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 150);
  delay(500);
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN1, LOW);
  delay(500);
  
  Serial.println("MOTOR: Reverse pulse");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  analogWrite(MOTOR_PWM, 150);
  delay(500);
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_IN2, LOW);
  delay(500);
  
  Serial.println("--- CYCLE COMPLETE, waiting 3s ---");
  delay(3000);
}

