#include "executor.h"
#include "direction.h"
#include "encoders.h"
#include "motors.h"
#include "motion_control.h"
#include "robot_state.h"

/* ================= CALIBRATED CONSTANTS ================= */
// Measure these against your physical robot:
//   CELL_MM:       drive one cell, read avgForwardTicks * MM_PER_TICK → adjust
//   MM_PER_TICK:   CELL_MM / tick_count_for_one_cell
//   TURN_TICKS_90: execute a 90° turn, read avgTurnTicks → use that value

static const float CELL_MM        = 180.0f;   // placeholder – measure physically
static const float MM_PER_TICK    = 1.0f;     // placeholder – measure physically
static const float TURN_TICKS_90  = 63.0f;    // placeholder – measure physically
static const float TURN_TICKS_180 = 2.0f * TURN_TICKS_90;

/* ================= STEP PHASE ================= */

enum StepPhase {
  STEP_IDLE,
  STEP_ALIGN,
  STEP_FORWARD
};

/* ================= INTERNAL STATE ================= */

static StepPhase    currentStepPhase    = STEP_IDLE;
static PrimitiveType currentPrimitive   = PRIM_IDLE;
static Direction    activeTargetDir     = NORTH;   // BUG FIX: was DIR_NORTH (now alias)

static long  prim_start_left_ticks  = 0;
static long  prim_start_right_ticks = 0;
static bool  stepDoneFlag           = false;
static float activeTurnTargetTicks  = 0.0f;

/* ================= HELPERS ================= */

static float avgForwardTicks(long dL, long dR) {
  return (dL + dR) / 2.0f;
}

static float avgTurnTicks(long dL, long dR) {
  return (abs(dL) + abs(dR)) / 2.0f;
}

static void startPrimitive(PrimitiveType prim) {
  prim_start_left_ticks  = getLeftTicks();
  prim_start_right_ticks = getRightTicks();
  currentPrimitive       = prim;
  activeTurnTargetTicks  = 0.0f;

  switch (prim) {
    case PRIM_FORWARD_ONE_CELL:
      motionControlBeginForward();
      Serial.println("EXEC START FORWARD");
      break;

    case PRIM_TURN_LEFT_90:
      activeTurnTargetTicks = TURN_TICKS_90;
      motionControlBeginTurn();
      Serial.println("EXEC START TURN LEFT 90");
      break;

    case PRIM_TURN_RIGHT_90:
      activeTurnTargetTicks = TURN_TICKS_90;
      motionControlBeginTurn();
      Serial.println("EXEC START TURN RIGHT 90");
      break;

    case PRIM_TURN_AROUND:
      activeTurnTargetTicks = TURN_TICKS_180;
      motionControlBeginTurn();
      Serial.println("EXEC START TURN 180");
      break;

    case PRIM_IDLE:
    default:
      stopMotors();
      break;
  }
}

// BUG FIX: chooseAlignPrimitive now uses NORTH/EAST/SOUTH/WEST (not DIR_*)
static PrimitiveType chooseAlignPrimitive(Direction currentHeading, Direction targetDir) {
  int diff = ((int)targetDir - (int)currentHeading + 4) % 4;

  if (diff == 0) return PRIM_FORWARD_ONE_CELL;
  if (diff == 1) return PRIM_TURN_RIGHT_90;
  if (diff == 2) return PRIM_TURN_AROUND;
  return PRIM_TURN_LEFT_90;  // diff == 3
}

static void updateHeadingAfterTurn(PrimitiveType prim) {
  switch (prim) {
    case PRIM_TURN_LEFT_90:
      robotState.heading = (Direction)(((int)robotState.heading + 3) % 4);
      break;
    case PRIM_TURN_RIGHT_90:
      robotState.heading = (Direction)(((int)robotState.heading + 1) % 4);
      break;
    case PRIM_TURN_AROUND:
      robotState.heading = (Direction)(((int)robotState.heading + 2) % 4);
      break;
    default:
      break;
  }
}

// BUG FIX: original used NORTH/EAST/SOUTH/WEST while executor.h declared DIR_*
// These now compile correctly because direction.h defines both sets as one enum.
static void updatePoseAfterForward() {
  switch (robotState.heading) {
    case NORTH: robotState.y += 1; break;
    case EAST:  robotState.x += 1; break;
    case SOUTH: robotState.y -= 1; break;
    case WEST:  robotState.x -= 1; break;
  }
}

/* ================= PUBLIC FUNCTIONS ================= */

void executorInit() {
  currentStepPhase      = STEP_IDLE;
  currentPrimitive      = PRIM_IDLE;
  activeTargetDir       = NORTH;
  prim_start_left_ticks  = 0;
  prim_start_right_ticks = 0;
  stepDoneFlag          = false;
  activeTurnTargetTicks  = 0.0f;

  motionControlInit();
}

void executorBeginStep(Direction targetDir) {
  if (currentStepPhase != STEP_IDLE) return;

  activeTargetDir = targetDir;

  // BUG FIX: stepDoneFlag must be cleared here so executorDone() returns false
  // while the new step is running. Without this, the test harness may see a
  // stale true from the previous step and immediately advance state.
  stepDoneFlag = false;

  PrimitiveType firstPrim = chooseAlignPrimitive(robotState.heading, activeTargetDir);

  if (firstPrim == PRIM_FORWARD_ONE_CELL) {
    currentStepPhase = STEP_FORWARD;
    startPrimitive(PRIM_FORWARD_ONE_CELL);
  } else {
    currentStepPhase = STEP_ALIGN;
    startPrimitive(firstPrim);
  }
}

void executorUpdate() {
  long current_left_ticks  = getLeftTicks();
  long current_right_ticks = getRightTicks();

  long dL = current_left_ticks  - prim_start_left_ticks;
  long dR = current_right_ticks - prim_start_right_ticks;

  switch (currentStepPhase) {

    case STEP_IDLE:
      stopMotors();
      currentPrimitive = PRIM_IDLE;
      break;

    case STEP_ALIGN: {
      float turned = avgTurnTicks(dL, dR);

      if (currentPrimitive == PRIM_TURN_LEFT_90) {
        motionControlUpdateTurn(-1, activeTurnTargetTicks, turned);
      } else if (currentPrimitive == PRIM_TURN_RIGHT_90 ||
                 currentPrimitive == PRIM_TURN_AROUND) {
        motionControlUpdateTurn(+1, activeTurnTargetTicks, turned);
      }

      if (turned >= activeTurnTargetTicks) {
        stopMotors();
        updateHeadingAfterTurn(currentPrimitive);

        // Transition: turn done → now drive forward one cell
        currentStepPhase = STEP_FORWARD;
        startPrimitive(PRIM_FORWARD_ONE_CELL);

        Serial.println("EXEC ALIGN COMPLETE -> FORWARD");
      }
      break;
    }

    case STEP_FORWARD: {
      float distance_mm = avgForwardTicks(dL, dR) * MM_PER_TICK;

      motionControlUpdateForward();

      if (distance_mm >= CELL_MM) {
        stopMotors();
        updatePoseAfterForward();

        currentPrimitive = PRIM_IDLE;
        currentStepPhase = STEP_IDLE;
        stepDoneFlag     = true;

        Serial.println("EXEC STEP COMPLETE");
      }
      break;
    }

    default:
      stopMotors();
      currentPrimitive = PRIM_IDLE;
      currentStepPhase = STEP_IDLE;
      break;
  }
}

bool executorDone() {
  return stepDoneFlag;
}

PrimitiveType executorGetCurrentPrimitive() {
  return currentPrimitive;
}

Direction executorGetTargetDirection() {
  return activeTargetDir;
}
