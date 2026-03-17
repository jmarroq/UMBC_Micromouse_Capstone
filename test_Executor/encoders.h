#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

void encodersInit();

long getLeftTicks();
long getRightTicks();

#endif