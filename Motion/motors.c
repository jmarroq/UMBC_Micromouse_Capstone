#include <Arduino.h>
#include "motors.h"

#define L_PWM 5
#define L_DIR 6
#define R_PWM 9
#define R_DIR 10

static void set_motor(int pwmPin, int dirPin, int speed)
{
    if(speed >= 0)
    {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, speed);
    }
    else
    {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, -speed);
    }
}

void motors_init()
{
    pinMode(L_PWM, OUTPUT);
    pinMode(L_DIR, OUTPUT);

    pinMode(R_PWM, OUTPUT);
    pinMode(R_DIR, OUTPUT);
}

void set_motors(int leftSpeed, int rightSpeed)
{
    if(leftSpeed > 255) leftSpeed = 255;
    if(leftSpeed < -255) leftSpeed = -255;

    if(rightSpeed > 255) rightSpeed = 255;
    if(rightSpeed < -255) rightSpeed = -255;

    set_motor(L_PWM, L_DIR, leftSpeed);
    set_motor(R_PWM, R_DIR, rightSpeed);
}


void stop_motors()
{
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
}
