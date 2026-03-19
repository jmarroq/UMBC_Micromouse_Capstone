#include "encoders.h"

/* ================= ENCODER PINS ================= */

#define ENC_L_A 23   // yellow – interrupt pin
#define ENC_L_B 22   // blue   – direction pin

#define ENC_R_A 41   // interrupt pin
#define ENC_R_B 40   // direction pin

/* ================= COUNTERS ================= */

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

/* ================= ISR ================= */

// BUG FIX: original ISRs only ever incremented. Direction was ignored, so
// turning (which reverses one wheel) still counted up. Turn completion and
// forward distance were both wrong after any reverse-wheel motion.
// Fix: read the B channel on each interrupt to determine direction.

void leftEncoderISR() {
  // Left motor: forward = A rising while B is LOW
  if (digitalReadFast(ENC_L_B))
    left_ticks--;
  else
    left_ticks++;
}

void rightEncoderISR() {
  // Right motor faces opposite direction, so polarity is inverted
  if (digitalReadFast(ENC_R_B))
    right_ticks++;
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

/* ================= RESET ================= */

// Call this at the start of each motion primitive so dL/dR always starts at 0.
// executor.cpp uses prim_start_*_ticks subtraction as an alternative, but
// resetEncoders() is simpler and avoids overflow on very long runs.
void resetEncoders() {
  noInterrupts();
  left_ticks  = 0;
  right_ticks = 0;
  interrupts();
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
