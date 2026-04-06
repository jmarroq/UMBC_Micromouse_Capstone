#include <Arduino.h>

#include "Sensors.h"
#include "perception.h"
#include "robot_state.h"
#include "planner_interface.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"

/* ================= TIMING ================= */

static const unsigned long SENSOR_PERIOD_MS  = 20;
static const unsigned long CONTROL_PERIOD_MS = 10;
static const unsigned long DEBUG_PERIOD_MS   = 500;

static unsigned long lastSensorTime  = 0;
static unsigned long lastControlTime = 0;
static unsigned long lastDebugTime   = 0;

/* ================= GLOBALS ================= */

Sensors sensors;
static bool haveSensorFrame = false;
static bool stepStarted     = false;

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Escape maze boot");

  motorsInit();
  encodersInit();
  robotStateInit(robotState);
  plannerInit();
  executorInit();

  if (!sensors.begin()) {
    Serial.println("ERROR: sensors.begin() failed");
    while (1) { delay(100); }
  }

  Serial.println("Setup complete");
}

/* ================= LOOP ================= */

void loop() {
  unsigned long now = millis();

  /* ── Sensor task (every 20 ms) ────────────────────────────────────────────
     Reads VL6180X sensors, updates robotState.sensorFrame and
     robotState.perceptionFrame.  The perceptionFrame is used by both
     motion_control (wall-following during forward motion) and
     plannerUpdateCurrentCell (wall commits).
  */
  if (now - lastSensorTime >= SENSOR_PERIOD_MS) {
    lastSensorTime = now;

    if (sensors.update20ms()) {
      sensors.getLatest(robotState.sensorFrame);
      robotState.perceptionFrame = computePerception(robotState.sensorFrame);
      haveSensorFrame = true;
    }
  }

  /* ── Control task (every 10 ms) ───────────────────────────────────────────
     Advances the executor FSM → PID → motor commands.
     This is the only place motors are commanded during normal motion.
  */
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;
    executorUpdate();
  }

  /* ── Planner task (event-driven) ──────────────────────────────────────────
     Runs every loop() but only does work on state transitions.
     Sequence per cell:
       1. executorDone() → plannerUpdateCurrentCell()
       2. plannerChooseNextDirection()
       3. executorBeginStep()
  */
  if (haveSensorFrame) {

    // Once the robot has exited, stop everything
    if (plannerAtExit()) {
      stopMotors();
      return;
    }

    // Wait until the current step is complete (or no step has started yet)
    bool idle = (executorGetCurrentPrimitive() == PRIM_IDLE);
    bool done = executorDone();

    if (!stepStarted || done) {
      // ── Update the map with walls sensed at the current cell ──────────────
      // plannerUpdateCurrentCell() also:
      //   • detects the exit opening (triggers PHASE_RETURN + reflood)
      //   • transitions PHASE_RETURN → PHASE_SPEEDRUN when back at start
      plannerUpdateCurrentCell();

      // ── Log phase transitions ──────────────────────────────────────────────
      static PlannerPhase lastLoggedPhase = PHASE_EXPLORE;
      if (plannerPhase() != lastLoggedPhase) {
        lastLoggedPhase = plannerPhase();
        Serial.print("MAIN: phase changed to ");
        Serial.println((int)plannerPhase());
      }

      if (plannerAtExit()) {
        stopMotors();
        Serial.println("MAIN: maze solved!");
        return;
      }

      // ── Choose and execute next direction ─────────────────────────────────
      Direction nextDir = plannerChooseNextDirection();

      Serial.print("MAIN: nextDir=");
      Serial.print((int)nextDir);
      Serial.print(" phase=");
      Serial.print((int)plannerPhase());
      Serial.print(" goal=");
      Serial.println(plannerGoalKnown() ? "YES" : "no");

      executorBeginStep(nextDir);
      stepStarted = true;
    }
  }

  /* ── Debug task (every 500 ms) ────────────────────────────────────────────
     One-line summary of sensor values, perception, and pose.
  */
  if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
    lastDebugTime = now;

    Serial.print("DBG  L=");
    Serial.print(robotState.sensorFrame.left_mm);
    Serial.print(" F=");
    Serial.print(robotState.sensorFrame.front_mm);
    Serial.print(" R=");
    Serial.print(robotState.sensorFrame.right_mm);
    Serial.print(" | WL=");
    Serial.print(robotState.perceptionFrame.wall_left);
    Serial.print(" WF=");
    Serial.print(robotState.perceptionFrame.wall_front);
    Serial.print(" WR=");
    Serial.print(robotState.perceptionFrame.wall_right);
    Serial.print(" | pose (");
    Serial.print(robotState.x);
    Serial.print(",");
    Serial.print(robotState.y);
    Serial.print(") hdg=");
    Serial.print((int)robotState.heading);
    Serial.print(" phase=");
    Serial.print((int)plannerPhase());
    Serial.print(" exit=");
    Serial.println(plannerAtExit() ? "YES" : "no");
  }
}
