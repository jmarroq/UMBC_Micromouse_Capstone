#include <Arduino.h>
#include "motion.h"
#include "encoders.h"
#include "motors.h"
#include "control.h"

void setup()
{
    Serial.begin(115200);

    motors_init();
    encoders_init();
    control_init();
    motion_init();

    enqueue_motion(MOVE_FORWARD_CELL);
    enqueue_motion(TURN_LEFT_90);
    enqueue_motion(MOVE_FORWARD_CELL);
}

void loop()
{
    update_motion();
}


