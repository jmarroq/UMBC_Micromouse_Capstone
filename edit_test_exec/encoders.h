#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

void encodersInit();
void resetEncoders();   // reset both counters to 0 (call at start of each primitive)

long getLeftTicks();
long getRightTicks();

#endif
