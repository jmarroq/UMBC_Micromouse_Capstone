/*
 Project: UMBC Micromouse Capstone - Fall Demo
 File: Initial_Demo.ino
 Description:
 * This sketch implements an autonomous control system for a differential drive robot. 
 * It utilizes encoder feedback with a Proportional (P) controller to synchronize wheel speeds,
   correcting for drift to maintain a straight path. 
  * It integrates an Adafruit VL6180X Time-of-Flight (ToF) sensor to continuously monitor the environment,
   automatically halting  the motors when an obstacle is detected within a predefined threshold. */

                                                                                                 

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>

/* ================= MOTOR PINS (NANO) ================= */

// Left motor (A)
#define PWMA 11
#define AIN1 6
#define AIN2 7

// Right motor (B)
#define PWMB 10
#define BIN1 9
#define BIN2 8

/* ================= ENCODER PINS ================= */

// Left encoder
#define ENC_L_A 2
#define ENC_L_B 4

// Right encoder
#define ENC_R_A 3
#define ENC_R_B 5

/* ================= SENSOR CONFIG ================= */

#define WALL_THRESHOLD_MM 130
Adafruit_VL6180X tof;

/* ================= ENCODER COUNTERS ================= */

volatile long leftTicks = 0;
volatile long rightTicks = 0;

/* ========== QUADRATURE ISR (Corrected Direction) ========== */
void ISR_leftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B))
    leftTicks--;
  else
    leftTicks++;
}

void ISR_rightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    rightTicks++;
  else
    rightTicks--;
}

/* ================= TIMERS ================= */

unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 50;

unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 50;

/* ================= CONTROL VARIABLES ================= */

long lastLeftTicks = 0;
long lastRightTicks = 0;

uint16_t distance_mm = 255;

// Tuned values to these fix drift
float Kp = 0.35;          // faster corrections
long bias = 0;           // left wheel lag compensation

// Base PWM per wheel (left is boosted)
int baseLeftPWM  = 130;   
int baseRightPWM = 135;

/* ================= SETUP ================= */

void setupEncoders() {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_leftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_rightA, CHANGE);
}

void setupMotors() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Starting System...");

  if (!tof.begin()) {
    Serial.println("VL6180X NOT FOUND");
    while (1);
  }
  Serial.println("Sensor OK");

  setupEncoders();
  setupMotors();
}

/* ================= MOTOR HELPERS ================= */

void motorsStop() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void motorsForward(int leftPWM, int rightPWM) {
  analogWrite(PWMA, leftPWM);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, rightPWM);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

/* ================= MAIN LOOP ================= */

void loop() {
  unsigned long now = millis();

  /* ---- SENSOR ---- */
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    distance_mm = tof.readRange();
    uint8_t status = tof.readRangeStatus();

    if (status != VL6180X_ERROR_NONE)
      distance_mm = 255;

    Serial.print("DIST: ");
    Serial.println(distance_mm);
  }

  /* ---- STOP IF TOO CLOSE ---- */
  if (distance_mm < WALL_THRESHOLD_MM) {
    motorsStop();
    Serial.println("STOP: Wall Ahead");
    return;
  }

  /* ---- STRAIGHT-LINE CONTROL ---- */
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;

    long dL = leftTicks - lastLeftTicks;
    long dR = rightTicks - lastRightTicks;

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    // Straight-line error with bias compensation
    long error = (dL - dR) + bias;

    int leftPWM  = baseLeftPWM  - (Kp * error);
    int rightPWM = baseRightPWM + (Kp * error);

  
    // Constrain
    leftPWM  = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    motorsForward(leftPWM, rightPWM);

    Serial.print("dL: ");   Serial.print(dL);
    Serial.print(" dR: ");  Serial.print(dR);
    Serial.print(" err: "); Serial.print(error);
    Serial.print(" LPWM: "); Serial.print(leftPWM);
    Serial.print(" RPWM: "); Serial.println(rightPWM);
  }
}
