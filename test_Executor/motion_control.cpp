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

static PID wallPid = {0.8f, 0.0f, 0.08f, 0.0f, 0.0f};
static PID turnPid = {1.0f, 0.0f, 0.08f, 0.0f, 0.0f};

/* ================= FORWARD SETTINGS ================= */

static const int FORWARD_BASE_PWM = 90;
static const int WALL_CORR_LIMIT  = 60;
static const int TURN_PWM_MIN     = 45;
static const int TURN_PWM_MAX     = 90;

/* ================= DEBUG ================= */

static unsigned long lastPrint = 0;

/* ================= PUBLIC FUNCTIONS ================= */

void motionControlInit() {
  pidReset(wallPid);
  pidReset(turnPid);
}

void motionControlBeginForward() {
  pidReset(wallPid);
}

void motionControlUpdateForward() {
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
  } else {
    wallCorr = 0;
  }

  wallCorr = constrain(wallCorr, -WALL_CORR_LIMIT, WALL_CORR_LIMIT);

  int left_pwm  = FORWARD_BASE_PWM - wallCorr;
  int right_pwm = FORWARD_BASE_PWM + wallCorr;

  left_pwm  = constrain(left_pwm, 0, 255);
  right_pwm = constrain(right_pwm, 0, 255);

  setMotor(left_pwm, right_pwm);

  if (millis() - lastPrint > 200) {
    Serial.print("CTRL FORWARD left=");
    Serial.print(left_pwm);
    Serial.print(" right=");
    Serial.print(right_pwm);
    Serial.print(" wallCorr=");
    Serial.println(wallCorr);
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

  int left_pwm  =  turnDir * mag;
  int right_pwm = -turnDir * mag;

  setMotor(left_pwm, right_pwm);

  if (millis() - lastPrint > 200) {
    Serial.print("CTRL TURN cur=");
    Serial.print(currentTicks);
    Serial.print(" target=");
    Serial.print(targetTicks);
    Serial.print(" cmd=");
    Serial.println(mag);
    lastPrint = millis();
  }
}