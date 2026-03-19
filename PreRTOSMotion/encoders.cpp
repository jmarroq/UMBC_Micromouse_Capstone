#include "encoders.h"

/* ================= ENCODER PINS ================= */

#define ENC_L_A 23
#define ENC_L_B 22

#define ENC_R_A 41
#define ENC_R_B 40

/* ================= COUNTERS ================= */

volatile long left_ticks = 0;
volatile long right_ticks = 0;

/* ================= ISR ================= */

// encoders.cpp — replace the ISRs with quadrature direction sensing
void leftEncoderISR() {
  if (digitalReadFast(ENC_L_B))
    left_ticks--;
  else
    left_ticks++;
}

void rightEncoderISR() {
  if (digitalReadFast(ENC_R_B))
    right_ticks++;   // note: right is inverted relative to left (motors face opposite)
  else
    right_ticks--;
}

/* ================= INIT ================= */

void encodersInit() {

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);

  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
}

/* ================= ACCESS ================= */

long getLeftTicks() {
  noInterrupts();
  long t = left_ticks;
  interrupts();
  return t;
}

long getRightTicks() {
  noInterrupts();
  long t = right_ticks;
  interrupts();
  return t;
}

void resetEncoders() {
  noInterrupts();
  left_ticks = 0;
  right_ticks = 0;
  interrupts();
}