#ifndef EXECUTOR_H
#define EXECUTOR_H

#include <Arduino.h>
#include "direction.h"

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
void executorBeginAlignOnly(Direction targetDir);

void executorUpdate();
bool executorDone();

// NEW: true if executor state machine is doing anything
bool executorBusy();

PrimitiveType executorGetCurrentPrimitive();
Direction executorGetTargetDirection();

#endif