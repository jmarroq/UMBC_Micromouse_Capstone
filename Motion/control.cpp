#include <Arduino.h>
#include "control.h"
#include "motors.h"
#include "encoders.h"

#define CONTROL_PERIOD_US 1000   // 1 kHz loop

IntervalTimer controlTimer;

static long prevLeftTicks = 0;
static long prevRightTicks = 0;

static float targetLeftVel = 0;
static float targetRightVel = 0;

/* PID parameters */

static float kp = 1.5;
static float ki = 0.05;
static float kd = 0.0;

static float leftIntegral = 0;
static float rightIntegral = 0;

static float prevLeftError = 0;
static float prevRightError = 0;

void control_loop()
{
    long left = get_left_ticks();
    long right = get_right_ticks();

    long leftDelta = left - prevLeftTicks;
    long rightDelta = right - prevRightTicks;

    prevLeftTicks = left;
    prevRightTicks = right;

    float leftVel = leftDelta;
    float rightVel = rightDelta;

    float leftError = targetLeftVel - leftVel;
    float rightError = targetRightVel - rightVel;

    leftIntegral += leftError;
    rightIntegral += rightError;

    float leftOutput =
        kp * leftError +
        ki * leftIntegral +
        kd * (leftError - prevLeftError);

    float rightOutput =
        kp * rightError +
        ki * rightIntegral +
        kd * (rightError - prevRightError);

    prevLeftError = leftError;
    prevRightError = rightError;

    set_motors((int)leftOutput, (int)rightOutput);
}

void control_init()
{
    controlTimer.begin(control_loop, CONTROL_PERIOD_US);
}

void set_target_velocity(int left, int right)
{
    targetLeftVel = left;
    targetRightVel = right;
}
