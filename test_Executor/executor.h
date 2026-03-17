#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <Arduino.h>

/* ================= DIRECTION ================= */

enum Direction : uint8_t {
  DIR_NORTH = 0,
  DIR_EAST  = 1,
  DIR_SOUTH = 2,
  DIR_WEST  = 3
};

/* ================= PRIMITIVES ================= */

enum PrimitiveType {
  PRIM_IDLE,
  PRIM_FORWARD_ONE_CELL,
  PRIM_TURN_LEFT_90,
  PRIM_TURN_RIGHT_90,
  PRIM_TURN_AROUND
};

void executorInit();
void executorBeginStep(Direction targetDir);
void executorUpdate();
bool executorDone();

PrimitiveType executorGetCurrentPrimitive();
Direction executorGetTargetDirection();

#endif