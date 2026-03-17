#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <Arduino.h>

void motionControlInit();

void motionControlBeginForward();
void motionControlUpdateForward();

void motionControlBeginTurn();
void motionControlUpdateTurn(int turnDir, float targetTicks, float currentTicks);

#endif