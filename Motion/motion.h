#ifndef MOTION_H
#define MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

/* Calibration constants */

#define TICKS_PER_CM 47

#define CELL_CENTER_TICKS 372
#define TURN_90_TICKS 147
#define TURN_180_TICKS 294

/* Motion types */

typedef enum
{
    MOVE_FORWARD_CELL,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_180
} MotionType;

/* Public functions */

void motion_init(void);

void enqueue_motion(MotionType type);

void update_motion(void);

#ifdef __cplusplus
}
#endif
#endif

