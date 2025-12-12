/*
 Project: UMBC Micromouse Capstone - Fall Demo
 File: control_loop.ino
 Description:
 This file implements the main control logic for autonomous navigation, 
 specifically designed for right-wall following along the inner perimeter of a rectangular path. 
 * It integrates sensor data from dual VL6180X Time-of-Flight sensors (Front and Right)
 * Maintains a set distance from the wall and detect corners.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>

/* ===================== PIN DEFINITIONS ===================== */

// Motors (N20 + L298N)
#define PWMA 11   // Left PWM
#define AIN1 6
#define AIN2 7

#define PWMB 10   // Right PWM
#define BIN1 9
#define BIN2 8

// Encoders
#define ENC_L_A 2
#define ENC_L_B 4
#define ENC_R_A 3
#define ENC_R_B 5

// ToF XSHUT pins
#define XSHUT_FRONT 12
#define XSHUT_RIGHT 13

// ToF I2C addresses
#define FRONT_ADDR 0x30
#define RIGHT_ADDR 0x31

// ToF internal register for I2C address
#define VL6180X_SLAVE_ADDRESS 0x212

/* ===================== ROBOT GEOMETRY ===================== */

// Measured / known values
const float WHEEL_DIAMETER_MM = 40.0;   // wheel diameter
const float ENCODER_CPR       = 12.0;   // CPR at motor shaft
const float GEAR_RATIO        = 49.0;   // gearbox ratio
const float TRACK_WIDTH_MM    = 110.0;  // distance between wheel centers

// Derived values (computed in setup)
float wheelCircumference;
float ticksPerWheelRev;
float mmPerTick;
int   TURN_90_TICKS;

/* ===================== ENCODERS ===================== */

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// Left encoder ISR
void ISR_leftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B))
    leftTicks--;
  else
    leftTicks++;
}

// Right encoder ISR
void ISR_rightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    rightTicks++;
  else
    rightTicks--;
}

/* ===================== TOF OBJECTS ===================== */

Adafruit_VL6180X tof_front = Adafruit_VL6180X();
Adafruit_VL6180X tof_right = Adafruit_VL6180X();

/* ===================== BEHAVIOR THRESHOLDS ===================== */

// Front wall detection
const int FRONT_WALL_MM       = 80;  // if closer than this → turn left

// Right wall following
const int RIGHT_TOO_CLOSE_MM  = 40;  
const int RIGHT_TOO_FAR_MM    = 80; 

/* ===================== FORWARD CONTROL PARAMS ===================== */

float Kp_forward = 0.30;   
int forwardLeftBase  = 170; // base PWM left
int forwardRightBase = 160; // base PWM right

/* ===================== PARAMETER COMPUTATION ===================== */

void computeMotionParameters() {
  wheelCircumference = 3.14159265 * WHEEL_DIAMETER_MM;
  ticksPerWheelRev   = ENCODER_CPR * GEAR_RATIO;        // 12 * 49 = 588
  mmPerTick          = wheelCircumference / ticksPerWheelRev;

  float arcLength90 = (3.14159265 * TRACK_WIDTH_MM) / 4.0;
  TURN_90_TICKS = (int)(arcLength90 / mmPerTick);

  Serial.println("=== ROBOT MOTION PARAMETERS ===");
  Serial.print("Wheel circumference (mm): "); Serial.println(wheelCircumference);
  Serial.print("Ticks per wheel rev: ");      Serial.println(ticksPerWheelRev);
  Serial.print("mm per tick: ");              Serial.println(mmPerTick, 5);
  Serial.print("Ticks for 90° turn: ");       Serial.println(TURN_90_TICKS);
  Serial.println("================================");
}

/* ===================== LOW-LEVEL TOF HELPERS ===================== */

void setSensorAddress(uint8_t oldAddress, uint8_t newAddress) {
  Wire.beginTransmission(oldAddress);
  Wire.write((VL6180X_SLAVE_ADDRESS >> 8) & 0xFF); // MSB
  Wire.write(VL6180X_SLAVE_ADDRESS & 0xFF);        // LSB
  Wire.write(newAddress << 1);                     // new address (7-bit << 1)
  Wire.endTransmission();
  delay(5);
}

// Initialize both sensors using XSHUT and give them unique addresses
void initToFSensors() {
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Turn both off
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // ---- FRONT SENSOR ----
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);

  if (!tof_front.begin(0x29)) {
    Serial.println("Front VL6180X not found at 0x29!");
    while (1);
  }
  Serial.println("Front sensor found at 0x29.");
  setSensorAddress(0x29, FRONT_ADDR);
  delay(10);
  if (!tof_front.begin(FRONT_ADDR)) {
    Serial.println("Front sensor failed at new address!");
    while (1);
  }
  Serial.println("Front sensor moved to 0x30");

  // ---- RIGHT SENSOR ----
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);

  if (!tof_right.begin(0x29)) {
    Serial.println("Right VL6180X not found at 0x29!");
    while (1);
  }
  Serial.println("Right sensor found at 0x29.");
  setSensorAddress(0x29, RIGHT_ADDR);
  delay(10);
  if (!tof_right.begin(RIGHT_ADDR)) {
    Serial.println("Right sensor failed at new address!");
    while (1);
  }
  Serial.println("Right sensor moved to 0x31");

  Serial.println("Both ToF sensors initialized.\n");
}

// Wrapper to read ToF sensors
uint8_t readFrontDistance() {
  uint8_t d = tof_front.readRange();
  uint8_t status = tof_front.readRangeStatus();
  if (status != VL6180X_ERROR_NONE) return 255;
  return d;
}

uint8_t readRightDistance() {
  uint8_t d = tof_right.readRange();
  uint8_t status = tof_right.readRangeStatus();
  if (status != VL6180X_ERROR_NONE) return 255;
  return d;
}

/* ===================== MOTOR HELPERS ===================== */

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void steerRightSlight() {
  // Curve toward right wall (left wheel faster)
  analogWrite(PWMA, 150);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, 110);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void steerLeftSlight() {
  // Curve away from right wall (right wheel faster)
  analogWrite(PWMA, 110);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, 150);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

/* ========== FORWARD WITH ENCODER STRAIGHT CORRECTION ========== */

void driveForward_enc() {
  static long lastL = 0;
  static long lastR = 0;

  long dL = leftTicks - lastL;
  long dR = rightTicks - lastR;

  lastL = leftTicks;
  lastR = rightTicks;

  long error = dL - dR;  // positive → left faster

  int leftPWM  = forwardLeftBase  - (Kp_forward * error);
  int rightPWM = forwardRightBase + (Kp_forward * error);

  leftPWM  = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  analogWrite(PWMA, leftPWM);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, rightPWM);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  Serial.print("[FWD] dL: "); Serial.print(dL);
  Serial.print("  dR: ");      Serial.print(dR);
  Serial.print("  err: ");     Serial.print(error);
  Serial.print("  PWM L/R: "); Serial.print(leftPWM);
  Serial.print("/");           Serial.println(rightPWM);
}

/* ========== 90° LEFT TURN USING ENCODERS + GEOMETRY FORMULA ========== */

void turnLeft90_enc() {
  Serial.println("Starting encoder-based 90° left turn...");

  stopMotors();
  delay(40);

  // Reset ticks
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  // Rotate: left backward, right forward
  analogWrite(PWMA, 140);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  analogWrite(PWMB, 140);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  while (abs(leftTicks) < TURN_90_TICKS && abs(rightTicks) < TURN_90_TICKS) {
    Serial.print("[TURN] L: ");
    Serial.print(leftTicks);
    Serial.print("  R: ");
    Serial.println(rightTicks);
  }

  stopMotors();
  delay(80);

  Serial.println("90° turn complete.\n");
}

/* ===================== SETUP ===================== */

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Encoder pins
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_leftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_rightA, CHANGE);

  Serial.println("Booting robot...");

  computeMotionParameters();
  initToFSensors();

  Serial.println("Setup complete.\n");
}

/* ===================== MAIN BEHAVIOR LOOP ===================== */

void loop() {
  // Read sensors
  uint8_t front = readFrontDistance();
  uint8_t right = readRightDistance();

  Serial.print("Front: ");
  Serial.print(front);
  Serial.print(" mm  Right: ");
  Serial.print(right);
  Serial.println(" mm");

  // 1) Corner detection: front wall too close → 90° left turn
  if (front < FRONT_WALL_MM) {
    Serial.println("Corner detected → turn left 90°");
    turnLeft90_enc();
    return;  // skip rest of loop this cycle
  }

  // 2) Right wall following
  if (right > RIGHT_TOO_FAR_MM) {
    // Steer right
    Serial.println("Right wall too far → steer right");
    steerRightSlight();
  }
  else if (right < RIGHT_TOO_CLOSE_MM) {
    // Steer left
    Serial.println("Right wall too close → steer left");
    steerLeftSlight();
  }
  else {
    // Drive straight with encoder correction
    Serial.println("Right wall OK → forward straight");
    driveForward_enc();
  }

  delay(40);  
}
