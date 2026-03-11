#include "motion.h"
#include "motors.h"
#include "encoders.h"
#include "robot_state.h"

/* ================= CALIBRATED CONSTANTS ================= */

const float CELL_MM        = 300.0f;
const float MM_PER_TICK    = 1.0f;
const float TURN_TICKS_90  = 73.0f;
const float TURN_TICKS_180 = 2.0f * TURN_TICKS_90;

/* ================= CONTROL TIMING ================= */

const float CONTROL_DT = 0.01f;

/* ================= PID STRUCT ================= */

struct PID {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
};

static void pidReset(PID& pid) {
  pid.integral = 0.0f;
  pid.prevError = 0.0f;
}

static float pidUpdate(PID& pid, float error, float dt) {
  pid.integral += error * dt;
  float derivative = (error - pid.prevError) / dt;
  pid.prevError = error;
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

/* ================= PID INSTANCES ================= */

static PID forwardPid = {1.4f, 0.0f, 0.08f, 0.0f, 0.0f};
static PID headingPid = {0.5f, 0.0f, 0.00f, 0.0f, 0.0f};
static PID turnPid    = {2.0f, 0.0f, 0.08f, 0.0f, 0.0f};
static PID wallPid    = {2.0f, 0.0f, 0.08f, 0.0f, 0.0f};

/* ================= INTERNAL STATE ================= */

static PrimitiveType currentPrimitive = PRIM_IDLE;
static long prim_start_left_ticks = 0;
static long prim_start_right_ticks = 0;
static bool primitiveDoneFlag = false;
static float activeTurnTargetTicks = 0.0f;

/* ================= PUBLIC FUNCTIONS ================= */

void motionInit() {
  currentPrimitive = PRIM_IDLE;
  primitiveDoneFlag = false;
  activeTurnTargetTicks = 0.0f;

  pidReset(forwardPid);
  pidReset(headingPid);
  pidReset(turnPid);
  pidReset(wallPid);
}

void motionBeginPrimitive(PrimitiveType prim) {
  prim_start_left_ticks = getLeftTicks();
  prim_start_right_ticks = getRightTicks();

  currentPrimitive = prim;
  primitiveDoneFlag = false;
  activeTurnTargetTicks = 0.0f;

  if (prim == PRIM_FORWARD_ONE_CELL) {
    pidReset(forwardPid);
    pidReset(headingPid);
    pidReset(wallPid);
    Serial.println("START FORWARD");
  }
  else if (prim == PRIM_TURN_RIGHT_90) {
    pidReset(turnPid);
    activeTurnTargetTicks = TURN_TICKS_90;
    Serial.println("START TURN RIGHT 90");
  }
  else if (prim == PRIM_TURN_AROUND) {
    pidReset(turnPid);
    activeTurnTargetTicks = TURN_TICKS_180;
    Serial.println("START TURN 180");
  }
}

void motionUpdate() {
  long current_left_ticks  = getLeftTicks();
  long current_right_ticks = getRightTicks();

  long dL = current_left_ticks  - prim_start_left_ticks;
  long dR = current_right_ticks - prim_start_right_ticks;

  static unsigned long lastPrint = 0;

  switch (currentPrimitive) {

    case PRIM_FORWARD_ONE_CELL: {
      float distance_mm = ((dL + dR) / 2.0f) * MM_PER_TICK;
      float distError   = CELL_MM - distance_mm;

      float forwardCmdF = pidUpdate(forwardPid, distError, CONTROL_DT);
      int forwardCmd = (int)forwardCmdF;

      if (distError > 0.0f) {
        if (forwardCmd < 45)  forwardCmd = 45;
        if (forwardCmd > 140) forwardCmd = 140;
      } else {
        forwardCmd = 0;
      }

      float headingError = (float)(dL - dR);
      float headingCorrF = pidUpdate(headingPid, headingError, CONTROL_DT);
      int headingCorr = (int)headingCorrF;

      int wallCorr = 0;

      if (robotState.perceptionFrame.wall_left &&
          robotState.perceptionFrame.wall_right &&
          robotState.sensorFrame.left_valid &&
          robotState.sensorFrame.right_valid) {

        float wallError =
          (float)robotState.sensorFrame.left_mm -
          (float)robotState.sensorFrame.right_mm;

        float wallCorrF = pidUpdate(wallPid, wallError, CONTROL_DT);
        wallCorr = (int)wallCorrF;
      }

      int left_pwm  = forwardCmd - headingCorr - wallCorr;
      int right_pwm = forwardCmd + headingCorr + wallCorr;

      left_pwm  = constrain(left_pwm,  0, 255);
      right_pwm = constrain(right_pwm, 0, 255);

      setMotor(left_pwm, right_pwm);

      if (millis() - lastPrint > 200) {
        Serial.print("FORWARD dist=");
        Serial.print(distance_mm);
        Serial.print(" errD=");
        Serial.print(distError);
        Serial.print(" cmdF=");
        Serial.print(forwardCmd);
        Serial.print(" dL=");
        Serial.print(dL);
        Serial.print(" dR=");
        Serial.print(dR);
        Serial.print(" errH=");
        Serial.print(headingError);
        Serial.print(" corrH=");
        Serial.print(headingCorr);
        Serial.print(" corrW=");
        Serial.println(wallCorr);
        lastPrint = millis();
      }

      if (distance_mm >= CELL_MM) {
        stopMotors();
        primitiveDoneFlag = true;
        currentPrimitive = PRIM_IDLE;
        Serial.println("FORWARD COMPLETE");
      }
      break;
    }

    case PRIM_TURN_RIGHT_90:
    case PRIM_TURN_AROUND: {
      float turned = (abs(dL) + abs(dR)) / 2.0f;
      float turnError = activeTurnTargetTicks - turned;

      float turnCmdF = pidUpdate(turnPid, turnError, CONTROL_DT);
      int turnCmd = (int)turnCmdF;

      int mag = abs(turnCmd);

      if (turnError > 0.0f) {
        if (mag < 45) mag = 45;
        if (mag > 90) mag = 90;
      } else {
        mag = 0;
      }

      setMotor(mag, -mag);

      if (millis() - lastPrint > 200) {
        Serial.print("TURN ticks=");
        Serial.print(turned);
        Serial.print(" target=");
        Serial.print(activeTurnTargetTicks);
        Serial.print(" err=");
        Serial.print(turnError);
        Serial.print(" cmd=");
        Serial.println(mag);
        lastPrint = millis();
      }

      if (turned >= activeTurnTargetTicks) {
        stopMotors();
        primitiveDoneFlag = true;
        currentPrimitive = PRIM_IDLE;

        if (activeTurnTargetTicks == TURN_TICKS_180) {
          Serial.println("TURN 180 COMPLETE");
        } else {
          Serial.println("TURN 90 COMPLETE");
        }
      }
      break;
    }

    case PRIM_IDLE:
    default:
      stopMotors();
      break;
  }
}

bool motionDone() {
  return primitiveDoneFlag;
}