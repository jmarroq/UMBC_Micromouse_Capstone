#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <Arduino.h>
#include "direction.h"   // ← single source of Direction; removed local duplicate enum

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
Direction     executorGetTargetDirection();

#endif
