#include "motors.h"

/* ================= MOTOR PINS ================= */

#define PWMA 2
#define AIN1 4
#define AIN2 3

#define PWMB 8
#define BIN1 6
#define BIN2 7

#define STBY 5

void motorsInit() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  stopMotors();
}

void setMotor(int left_pwm, int right_pwm) {
  digitalWrite(STBY, HIGH);

  if (left_pwm >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    left_pwm = -left_pwm;
  }

  if (right_pwm >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    right_pwm = -right_pwm;
  }

  // BUG FIX: Serial.print removed from here.
  // At 10ms control rate this was 100 blocking writes/sec, adding ~1ms of
  // jitter per call and flooding the Serial monitor.
  // Motor values are printed by taskDebug / motionControlUpdateForward at 200ms.
  analogWrite(PWMA, constrain(left_pwm, 0, 255));
  analogWrite(PWMB, constrain(right_pwm, 0, 255));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
