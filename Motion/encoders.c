#include "encoders.h"

/* Encoder pins */

#define ENC_L_A 2
#define ENC_L_B 4

#define ENC_R_A 3
#define ENC_R_B 5

volatile long leftTicks = 0;
volatile long rightTicks = 0;


/* Left encoder interrupt */

void left_encoder_isr(void)
{
    if (digitalReadFast(ENC_L_B))
        leftTicks--;
    else
        leftTicks++;
}


/* Right encoder interrupt */

void right_encoder_isr(void)
{
    if (digitalReadFast(ENC_R_B))
        rightTicks++;
    else
        rightTicks--;
}


/* Initialization */

void encoders_init(void)
{
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);

    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), right_encoder_isr, CHANGE);
}


/* Reset counts */

void reset_encoders(void)
{
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
}


/* Getter functions */

long get_left_ticks(void)
{
    return leftTicks;
}

long get_right_ticks(void)
{
    return rightTicks;
}

