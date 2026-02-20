#include <Arduino.h>

/* =============================================================
   LAYER 2 — MOVEMENT PRIMITIVES 

/* ------------------- MOTOR PINS ------------------- */

// Left motor
#define PWMA 11
#define AIN1 6
#define AIN2 7

// Right motor
#define PWMB 10
#define BIN1 9
#define BIN2 8

/* ------------------- ENCODER PINS ------------------- */

#define ENC_L_A 2
#define ENC_L_B 4

#define ENC_R_A 3
#define ENC_R_B 5

/* ------------------- ROBOT GEOMETRY PARAMETERS ------------------- */

// Geometry parameters
const float WHEEL_DIAMETER_MM = 40.0;
const float ENCODER_CPR       = 12.0;
const float GEAR_RATIO        = 49.0;
const float TRACK_WIDTH_MM    = 110.0;

// Calculated automatically
float wheelCircumference;
float ticksPerWheelRev;
float mmPerTick;
int TURN_90_TICKS;

/* ------------------- ENCODER COUNTERS ------------------- */

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

/* ------------------- ENCODER ISRs ------------------- */

// LEFT encoder ISR
void ISR_leftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B))
    leftTicks--;
  else
    leftTicks++;
}

// RIGHT encoder ISR
void ISR_rightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B))
    rightTicks++;
  else
    rightTicks--;
}

/* ------------------- AUTO PARAMETER COMPUTATION ------------------- */

void computeMotionParameters() {
  wheelCircumference = 3.14159265 * WHEEL_DIAMETER_MM;
  ticksPerWheelRev   = ENCODER_CPR * GEAR_RATIO;         // 12 × 49 = 588
  mmPerTick          = wheelCircumference / ticksPerWheelRev;

  float arc90 = (3.14159265 * TRACK_WIDTH_MM) / 4.0;
  TURN_90_TICKS = (int)(arc90 / mmPerTick);

  Serial.println("=== MOTION PARAMETERS ===");
  Serial.print("Wheel circumference (mm): "); Serial.println(wheelCircumference);
  Serial.print("Ticks per wheel rev: ");      Serial.println(ticksPerWheelRev);
  Serial.print("mm per tick: ");              Serial.println(mmPerTick, 5);
  Serial.print("90° turn ticks: ");           Serial.println(TURN_90_TICKS);
  Serial.println("==========================\n");
}

/* ------------------- BASIC MOTOR HELPERS ------------------- */

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

/* ------------------- FORWARD USING ENCODER STRAIGHTENING ------------------- */

float Kp_forward = 0.30;
int forwardLeftBase  = 170;
int forwardRightBase = 160;

void driveForward_enc() {
  static long lastL = 0;
  static long lastR = 0;

  long dL = leftTicks - lastL;
  long dR = rightTicks - lastR;

  lastL = leftTicks;
  lastR = rightTicks;

  long error = dL - dR;

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

  Serial.print("[FWD] L/R ticks: ");
  Serial.print(dL); Serial.print("/");
  Serial.print(dR);
  Serial.print("  PWM: ");
  Serial.print(leftPWM); Serial.print("/");
  Serial.println(rightPWM);
}

/* ------------------- 90° TURN USING ENCODERS ------------------- */

void turnLeft90_enc() {
  Serial.println("Starting 90° encoder turn...");

  stopMotors();
  delay(50);

  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();

  // rotate in place
  analogWrite(PWMA, 140);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  analogWrite(PWMB, 140);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  while (abs(leftTicks) < TURN_90_TICKS &&
         abs(rightTicks) < TURN_90_TICKS) {

    Serial.print("[TURN] L=");
    Serial.print(leftTicks);
    Serial.print(" R=");
    Serial.println(rightTicks);
  }

  stopMotors();
  delay(80);

  Serial.println("90° turn complete!\n");
}

/* ------------------- SLIGHT STEERING  ------------------- */

void steerRightSlight() {
  // steer toward the right wall (left wheel faster)
  analogWrite(PWMA, 150);  // Left wheel faster
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, 110);  // Right wheel slower
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void steerLeftSlight() {
  // steer away from right wall (right wheel faster)
  analogWrite(PWMA, 110);  // Left wheel slower
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  analogWrite(PWMB, 150);  // Right wheel faster
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}


/* ------------------- SETUP ------------------- */

void setup() {
  Serial.begin(115200);

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


  computeMotionParameters();
}

/* ------------------- LOOP FOR TESTING ------------------- */

void loop() {

  // 1 second driving forward
  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    driveForward_enc();
  }

  stopMotors();
  delay(300);

  // 90° left turn
  turnLeft90_enc();
  delay(800);
}
