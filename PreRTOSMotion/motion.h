#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

enum PrimitiveType {
  PRIM_IDLE,
  PRIM_FORWARD_ONE_CELL,
  PRIM_TURN_RIGHT_90,
  PRIM_TURN_AROUND
};

void motionInit();
void motionBeginPrimitive(PrimitiveType prim);
void motionUpdate();
bool motionDone();

#endif