#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void motorsInit();
void setMotor(int left_pwm, int right_pwm);
void stopMotors();

#endif