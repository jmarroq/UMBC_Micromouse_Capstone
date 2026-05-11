#include "motion_control.h"
#include "motors.h"
#include "robot_state.h"

/* ================= CONTROL TIMING ================= */

static const float CONTROL_DT = 0.01f;

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

static PID wallPid = {1.8f, 0.0f, 0.4f, 0.0f, 0.0f};
static PID turnPid = {0.9f, 0.0f, 0.5f, 0.0f, 0.0f};

/* ================= FORWARD SETTINGS ================= */

static const int FORWARD_BASE_PWM = 100;
static const int WALL_CORR_LIMIT  = 10;

static const int TURN_PWM_MIN = 60;
static const int TURN_PWM_MAX = 70;

static const int FRONT_APPROACH_PWM_MIN = 35;
static const int FRONT_APPROACH_PWM_MAX = 70;

static const int BACK_AWAY_PWM = 50;



/* ================= WALL FOLLOW SETTINGS ================= */

// Expected side distance when following one wall.
// Tune this based on your centered readings.
// Good starting range: 55–75 mm.
static const float SIDE_TARGET_MM = 48.0f;


static const int WALL_CORR_SIGN = 1;

/* ================= DEBUG ================= */

static unsigned long lastPrint = 0;

/* ================= WALL CORRECTION HELPER ================= */

static int computeWallCorrection(int limit) {
  bool hasLeft =
    robotState.perceptionFrame.wall_left &&
    robotState.sensorFrame.left_valid;

  bool hasRight =
    robotState.perceptionFrame.wall_right &&
    robotState.sensorFrame.right_valid;

  float error = 0.0f;
  bool useCorrection = false;

  if (hasLeft && hasRight) {
    // Corridor mode:
    // error = 0 means centered between both walls.
    error =
      (float)robotState.sensorFrame.left_mm -
      (float)robotState.sensorFrame.right_mm;

    useCorrection = true;
  }
  else if (hasLeft && !hasRight) {
    // Left-wall mode:
    // Convert one-wall distance into the same style of error.
    //
    // left_mm > SIDE_TARGET_MM  -> too far from left wall
    // left_mm < SIDE_TARGET_MM  -> too close to left wall
    error =
      2.0f * ((float)robotState.sensorFrame.left_mm - SIDE_TARGET_MM);

    useCorrection = true;
  }
  else if (!hasLeft && hasRight) {
    // Right-wall mode:
    // Convert one-wall distance into the same style of error.
    //
    // right_mm > SIDE_TARGET_MM -> too far from right wall
    // right_mm < SIDE_TARGET_MM -> too close to right wall
    error =
      2.0f * (SIDE_TARGET_MM - (float)robotState.sensorFrame.right_mm);

    useCorrection = true;
  }
  else {
    useCorrection = false;
  }

  if (!useCorrection) {
    return 0;
  }

  float wallCorrF = pidUpdate(wallPid, error, CONTROL_DT);
  int wallCorr = (int)wallCorrF;

  wallCorr *= WALL_CORR_SIGN;
  wallCorr = constrain(wallCorr, -limit, limit);

  return wallCorr;
}

/* ================= PUBLIC FUNCTIONS ================= */

void motionControlInit() {
  pidReset(wallPid);
  pidReset(turnPid);
}

void motionControlBeginForward() {
  pidReset(wallPid);
}

void motionControlUpdateForward() {
  int wallCorr = computeWallCorrection(WALL_CORR_LIMIT);

  int left_pwm  = FORWARD_BASE_PWM + wallCorr;
  int right_pwm = FORWARD_BASE_PWM - wallCorr;

  left_pwm  = constrain(left_pwm, 0, 255);
  right_pwm = constrain(right_pwm, 0, 255);

  setMotor(left_pwm, right_pwm);

  if (millis() - lastPrint > 200) {
    Serial.print("CTRL FORWARD L=");
    if (robotState.sensorFrame.left_valid) Serial.print(robotState.sensorFrame.left_mm);
    else Serial.print("X");

    Serial.print(" R=");
    if (robotState.sensorFrame.right_valid) Serial.print(robotState.sensorFrame.right_mm);
    else Serial.print("X");

    Serial.print(" corr=");
    Serial.print(wallCorr);

    Serial.print(" pwmL=");
    Serial.print(left_pwm);

    Serial.print(" pwmR=");
    Serial.println(right_pwm);

    lastPrint = millis();
  }
}

void motionControlBeginTurn() {
  pidReset(turnPid);
}

void motionControlUpdateTurn(int turnDir, float targetTicks, float currentTicks) {
  float turnError = targetTicks - currentTicks;

  float turnCmdF = pidUpdate(turnPid, turnError, CONTROL_DT);
  int turnCmd = (int)turnCmdF;
  int mag = abs(turnCmd);

  if (turnError > 0.0f) {
    if (mag < TURN_PWM_MIN) mag = TURN_PWM_MIN;
    if (mag > TURN_PWM_MAX) mag = TURN_PWM_MAX;
  } else {
    mag = 0;
  }

  int left_pwm  = -turnDir * mag;
  int right_pwm = turnDir * mag;

  setMotor(left_pwm, right_pwm);
}

void motionControlBeginFrontApproach() {
  pidReset(wallPid);
}

void motionControlUpdateFrontApproach(uint16_t targetFrontMm) {
  if (!robotState.sensorFrame.front_valid) {
    stopMotors();
    return;
  }

  float frontError =
    (float)robotState.sensorFrame.front_mm -
    (float)targetFrontMm;

  int basePwm = (int)(1.2f * frontError);

  if (frontError > 0.0f) {
    if (basePwm < FRONT_APPROACH_PWM_MIN) basePwm = FRONT_APPROACH_PWM_MIN;
    if (basePwm > FRONT_APPROACH_PWM_MAX) basePwm = FRONT_APPROACH_PWM_MAX;
  } else {
    basePwm = 0;
  }

  int wallCorr = computeWallCorrection(20);

  int left_pwm  = basePwm + wallCorr;
  int right_pwm = basePwm - wallCorr;

  left_pwm  = constrain(left_pwm, 0, 255);
  right_pwm = constrain(right_pwm, 0, 255);

  setMotor(left_pwm, right_pwm);

  if (millis() - lastPrint > 200) {
    Serial.print("CTRL FRONT APPROACH front=");
    Serial.print(robotState.sensorFrame.front_mm);
    Serial.print(" target=");
    Serial.print(targetFrontMm);
    Serial.print(" corr=");
    Serial.print(wallCorr);
    Serial.print(" cmdL=");
    Serial.print(left_pwm);
    Serial.print(" cmdR=");
    Serial.println(right_pwm);
    lastPrint = millis();
  }
}

void motionControlBeginBackAwayFrontWall() {
  pidReset(wallPid);
}

void motionControlUpdateBackAwayFrontWall(uint16_t targetFrontMm) {
  if (!robotState.sensorFrame.front_valid) {
    stopMotors();
    return;
  }

  int backPwm = 0;

  if (robotState.sensorFrame.front_mm < targetFrontMm) {
    backPwm = BACK_AWAY_PWM;
  }

  int wallCorr = computeWallCorrection(18);

  int left_pwm  = -backPwm + wallCorr;
  int right_pwm = -backPwm - wallCorr;

  left_pwm  = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);

  setMotor(left_pwm, right_pwm);

  if (millis() - lastPrint > 200) {
    Serial.print("CTRL BACK AWAY front=");
    Serial.print(robotState.sensorFrame.front_mm);
    Serial.print(" target=");
    Serial.print(targetFrontMm);
    Serial.print(" corr=");
    Serial.print(wallCorr);
    Serial.print(" cmdL=");
    Serial.print(left_pwm);
    Serial.print(" cmdR=");
    Serial.println(right_pwm);
    lastPrint = millis();
  }
}