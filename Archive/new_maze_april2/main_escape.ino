// main_escape.ino  —  4×4 escape maze main loop
//
// Architecture
// ────────────
//   taskSensors  (every 20 ms) — read VL6180X, update PerceptionFrame + WallDetector
//   taskControl  (every 10 ms) — executorUpdate() → PID → motors
//   taskPlanner  (event-driven) — runs on executorDone(), chooses next direction
//   taskDebug    (every 500 ms) — Serial output
//
// Phase flow
// ──────────
//   PHASE_EXPLORE  : left-wall-follow until exit found
//   PHASE_RETURN   : floodfill toward start (gradient reversed)
//   PHASE_SPEEDRUN : floodfill toward exit (optimal path)
//   done           : plannerAtExit() == true → stop

#include <Arduino.h>
#include "sensors.h"
#include "perception.h"
#include "wall_detector.h"
#include "planner_interface.h"
#include "robot_state.h"
#include "executor.h"
#include "encoders.h"
#include "motors.h"

// ── Timing ────────────────────────────────────────────────────────────────────
static const unsigned long SENSOR_PERIOD_MS  = 20;
static const unsigned long CONTROL_PERIOD_MS = 10;
static const unsigned long DEBUG_PERIOD_MS   = 500;

static unsigned long lastSensorTime  = 0;
static unsigned long lastControlTime = 0;
static unsigned long lastDebugTime   = 0;

// ── Globals ───────────────────────────────────────────────────────────────────
Sensors    sensors;
RobotState robotState;

// ── Planner state ─────────────────────────────────────────────────────────────
static bool stepInProgress = false;

// ─────────────────────────────────────────────────────────────────────────────
// taskSensors
// Called every 20 ms.  Reads all three VL6180X sensors through the I2C mux,
// computes the single-frame PerceptionFrame, and feeds the WallDetector vote
// window.  The vote window is what plannerArriveAtCell() uses — not the raw
// single-frame result.
// ─────────────────────────────────────────────────────────────────────────────
void taskSensors() {
    if (!sensors.update20ms()) return;

    sensors.getLatest(robotState.sensorFrame);

    // Fast single-frame perception (used by motion_control for wall-following
    // correction during forward motion — needs to be responsive, not smoothed)
    robotState.perceptionFrame = computePerception(robotState.sensorFrame);

    // Smoothed vote-window perception (used by planner for wall commits)
    wallDetector.update(robotState.sensorFrame);
}

// ─────────────────────────────────────────────────────────────────────────────
// taskControl
// Called every 10 ms.  Advances the executor FSM (PID → motor commands).
// This is the only place motors are commanded during normal operation.
// ─────────────────────────────────────────────────────────────────────────────
void taskControl() {
    executorUpdate();
}

// ─────────────────────────────────────────────────────────────────────────────
// taskPlanner
// Event-driven — runs every loop() but only does work when the executor
// signals that the current step is complete.
//
// Sequence per cell:
//   1. plannerArriveAtCell()     → sense walls, update map, maybe find exit
//   2. choose direction based on phase
//   3. executorBeginStep(dir)    → begin next motion primitive
// ─────────────────────────────────────────────────────────────────────────────
void taskPlanner() {
    // Nothing to do until the current step finishes
    if (stepInProgress && !executorDone()) return;

    // If we've exited the maze, stop everything
    if (plannerAtExit()) {
        stopMotors();
        return;
    }

    // ── Step just completed (or this is the very first step) ─────────────────
    stepInProgress = false;

    // Update the map with the current cell's walls and check for phase changes.
    // Uses wallDetector.getFrame() internally — validated, not raw sensor data.
    plannerArriveAtCell();

    if (plannerAtExit()) {
        stopMotors();
        Serial.println("MAIN: maze solved!");
        return;
    }

    // ── Choose next direction based on phase ──────────────────────────────────
    Direction next;

    switch (plannerPhase()) {

        case PHASE_EXPLORE:
            // Left-wall-follow: explore the maze and discover the exit
            next = plannerLeftWallDirection();
            break;

        case PHASE_RETURN:
            // Exit found — follow floodfill gradient back to start.
            // The gradient was set toward start by mazeOpenExit → mazeReflood.
            // Once we reach start, plannerArriveAtCell() transitions to SPEEDRUN.
            //
            // NOTE: we need to switch the reflood gradient here.
            // mazeOpenExit() called mazeReflood(false) = toward exit.
            // For the return trip we need mazeReflood(true) = toward start.
            // This is done once on phase entry (see below).
            next = plannerChooseNextDirection();
            break;

        case PHASE_SPEEDRUN:
            // Run the optimal path toward exit
            next = plannerChooseNextDirection();
            break;

        default:
            next = robotState.heading;
            break;
    }

    // ── Begin the next step ───────────────────────────────────────────────────
    executorBeginStep(next);
    stepInProgress = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// taskDebug
// Prints a one-line summary at 500 ms intervals.
// ─────────────────────────────────────────────────────────────────────────────
void taskDebug() {
    Serial.print("DBG pose=(");
    Serial.print(robotState.x); Serial.print(",");
    Serial.print(robotState.y); Serial.print(") hdg=");
    Serial.print(robotState.heading);
    Serial.print(" phase=");    Serial.print(plannerPhase());
    Serial.print(" virt=(");    Serial.print(plannerCurrentVrow());
    Serial.print(",");          Serial.print(plannerCurrentVcol());
    Serial.print(") goal=");    Serial.print(mazeGoalKnown() ? "YES" : "no ");
    Serial.print(" exit=");     Serial.print(plannerAtExit()  ? "YES" : "no ");

    uint8_t vl, vf, vr;
    wallDetector.getVoteCounts(vl, vf, vr);
    Serial.print(" votes L="); Serial.print(vl);
    Serial.print(" F=");       Serial.print(vf);
    Serial.print(" R=");       Serial.println(vr);
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("=== Escape maze boot ===");

    motorsInit();
    encodersInit();
    robotStateInit(robotState);
    executorInit();
    plannerInit();

    if (!sensors.begin()) {
        Serial.println("FATAL: sensors.begin() failed");
        while (1) { delay(100); }
    }

    Serial.println("Setup complete — beginning exploration");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    if (now - lastSensorTime >= SENSOR_PERIOD_MS) {
        lastSensorTime = now;
        taskSensors();
    }

    if (now - lastControlTime >= CONTROL_PERIOD_MS) {
        lastControlTime = now;
        taskControl();
    }

    // Planner is event-driven, not timer-driven
    taskPlanner();

    if (now - lastDebugTime >= DEBUG_PERIOD_MS) {
        lastDebugTime = now;
        taskDebug();
    }
}
