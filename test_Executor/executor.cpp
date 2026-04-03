#include "executor.h"
#include "direction.h"
#include "encoders.h"
#include "motors.h"
#include "motion_control.h"
#include "robot_state.h"

/* ================= CALIBRATED CONSTANTS ================= */

static const float CELL_MM_BASE = 182.0f;
static const float TURN_TICKS_90  = 59.0f;
static const uint16_t FRONT_APPROACH_TARGET_MM = 40;
static const uint16_t FRONT_APPROACH_ENABLE_MM = 150;
static const uint16_t FRONT_TOO_CLOSE_MM       = 30;
static const uint16_t FRONT_BACK_AWAY_TARGET_MM = 40;

static const float LONG_RUN_EXTRA_MM = 0.0f;
static const uint8_t LONG_RUN_THRESHOLD = 3;
static const float MM_PER_TICK    = 1.0f;     
static const float TURN_TICKS_180 = 2.04f * TURN_TICKS_90;



/* ================= STEP PHASE ================= */

enum StepPhase {
  STEP_IDLE,
  STEP_ALIGN,
  STEP_FORWARD,
  STEP_APPROACH_FRONT_WALL,
  STEP_BACK_AWAY_FRONT_WALL
};

/* ================= INTERNAL STATE ================= */

static StepPhase currentStepPhase = STEP_IDLE;
static PrimitiveType currentPrimitive = PRIM_IDLE;
static Direction activeTargetDir = NORTH;

static long prim_start_left_ticks = 0;
static long prim_start_right_ticks = 0;
static bool stepDoneFlag = false;
static float activeTurnTargetTicks = 0.0f;

static Direction lastCompletedStepDir = NORTH;
static uint8_t straightRunCount = 0;

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
  currentPrimitive = prim;
  activeTurnTargetTicks = 0.0f;

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

static PrimitiveType chooseAlignPrimitive(Direction currentHeading, Direction targetDir) {
  int diff = ((int)targetDir - (int)currentHeading + 4) % 4;

  if (diff == 0) return PRIM_FORWARD_ONE_CELL;
  if (diff == 1) return PRIM_TURN_RIGHT_90;
  if (diff == 2) return PRIM_TURN_AROUND;
  return PRIM_TURN_LEFT_90; // diff == 3
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

static void updatePoseAfterForward() {
  switch (robotState.heading) {
    case NORTH:
      robotState.y += 1;
      break;
    case EAST:
      robotState.x += 1;
      break;
    case SOUTH:
      robotState.y -= 1;
      break;
    case WEST:
      robotState.x -= 1;
      break;
  }
}

static bool shouldApproachFrontWall() {
  return robotState.perceptionFrame.wall_front &&
         robotState.sensorFrame.front_valid &&
         (robotState.sensorFrame.front_mm <= FRONT_APPROACH_ENABLE_MM);
}

static bool shouldBackAwayFrontWall() {
  return robotState.perceptionFrame.wall_front &&
         robotState.sensorFrame.front_valid &&
         (robotState.sensorFrame.front_mm < FRONT_TOO_CLOSE_MM);
}

static void updateStraightRunCountAfterStep(Direction stepDir) {
  if (stepDir == lastCompletedStepDir) {
    if (straightRunCount < 255) straightRunCount++;
  } else {
    straightRunCount = 1;
    lastCompletedStepDir = stepDir;
  }
}

/* ================= PUBLIC FUNCTIONS ================= */

void executorInit() {
  currentStepPhase = STEP_IDLE;
  currentPrimitive = PRIM_IDLE;
  activeTargetDir = NORTH;

  prim_start_left_ticks = 0;
  prim_start_right_ticks = 0;
  stepDoneFlag = false;
  activeTurnTargetTicks = 0.0f;

  motionControlInit();
}

void executorBeginStep(Direction targetDir) {
  if (currentStepPhase != STEP_IDLE) return;

  activeTargetDir = targetDir;
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
      }
      else if (currentPrimitive == PRIM_TURN_RIGHT_90) {
        motionControlUpdateTurn(+1, activeTurnTargetTicks, turned);
      }
      else if (currentPrimitive == PRIM_TURN_AROUND) {
        motionControlUpdateTurn(+1, activeTurnTargetTicks, turned);
      }

      if (turned >= activeTurnTargetTicks) {
        stopMotors();
        updateHeadingAfterTurn(currentPrimitive);

        currentStepPhase = STEP_FORWARD;
        startPrimitive(PRIM_FORWARD_ONE_CELL);

        Serial.println("EXEC ALIGN COMPLETE -> FORWARD");
      }
      break;
    }

    case STEP_FORWARD: {
      float distance_mm = avgForwardTicks(dL, dR) * MM_PER_TICK;

      float targetCellMm = CELL_MM_BASE;
      if (straightRunCount >= LONG_RUN_THRESHOLD) {
        targetCellMm += LONG_RUN_EXTRA_MM;
      }

      motionControlUpdateForward();

      if (distance_mm >= targetCellMm) {
        stopMotors();
        updatePoseAfterForward();
        updateStraightRunCountAfterStep(robotState.heading);

        if (shouldBackAwayFrontWall()) {
          currentStepPhase = STEP_BACK_AWAY_FRONT_WALL;
          currentPrimitive = PRIM_IDLE;
          motionControlBeginBackAwayFrontWall();
          Serial.println("EXEC FORWARD COMPLETE -> BACK AWAY");
        }
        else if (shouldApproachFrontWall()) {
          currentStepPhase = STEP_APPROACH_FRONT_WALL;
          currentPrimitive = PRIM_IDLE;
          motionControlBeginFrontApproach();
          Serial.println("EXEC FORWARD COMPLETE -> FRONT APPROACH");
        }
        else {
          currentPrimitive = PRIM_IDLE;
          currentStepPhase = STEP_IDLE;
          stepDoneFlag = true;
          Serial.println("EXEC STEP COMPLETE");
        }
      }
      break;
    }

    case STEP_APPROACH_FRONT_WALL: {
      motionControlUpdateFrontApproach(FRONT_APPROACH_TARGET_MM);

      if (!robotState.sensorFrame.front_valid) {
        stopMotors();
        currentPrimitive = PRIM_IDLE;
        currentStepPhase = STEP_IDLE;
        stepDoneFlag = true;
        Serial.println("EXEC FRONT APPROACH DONE (front invalid)");
      }
      else if (robotState.sensorFrame.front_mm <= FRONT_APPROACH_TARGET_MM) {
        stopMotors();
        currentPrimitive = PRIM_IDLE;
        currentStepPhase = STEP_IDLE;
        stepDoneFlag = true;
        Serial.println("EXEC FRONT APPROACH DONE");
      }
      break;
    }

    case STEP_BACK_AWAY_FRONT_WALL: {
      motionControlUpdateBackAwayFrontWall(FRONT_BACK_AWAY_TARGET_MM);

      if (!robotState.sensorFrame.front_valid) {
        stopMotors();
        currentPrimitive = PRIM_IDLE;
        currentStepPhase = STEP_IDLE;
        stepDoneFlag = true;
        Serial.println("EXEC BACK AWAY DONE (front invalid)");
      }
      else if (robotState.sensorFrame.front_mm >= FRONT_BACK_AWAY_TARGET_MM) {
        stopMotors();
        currentPrimitive = PRIM_IDLE;
        currentStepPhase = STEP_IDLE;
        stepDoneFlag = true;
        Serial.println("EXEC BACK AWAY DONE");
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