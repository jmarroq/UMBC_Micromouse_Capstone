#include "motion.h"
#include "encoders.h"
#include "control.h"

#define MOTION_QUEUE_SIZE 32

/* Motion command structure */

typedef struct
{
    MotionType type;
    long targetTicks;
} MotionCommand;


/* Motion queue */

static MotionCommand motionQueue[MOTION_QUEUE_SIZE];

static int queueHead = 0;
static int queueTail = 0;


/* Motion state */

static int motionActive = 0;

static MotionCommand currentMotion;


/* Speed control */

static int baseVelocity = 0;

#define MAX_VELOCITY 25
#define ACCEL_STEP 1


/* Initialize motion system */

void motion_init(void)
{
}


/* Add motion to queue */

void enqueue_motion(MotionType type)
{
    MotionCommand cmd;

    cmd.type = type;

    switch(type)
    {
        case MOVE_FORWARD_CELL:
            cmd.targetTicks = CELL_CENTER_TICKS;
            break;

        case TURN_LEFT_90:
        case TURN_RIGHT_90:
            cmd.targetTicks = TURN_90_TICKS;
            break;

        case TURN_180:
            cmd.targetTicks = TURN_180_TICKS;
            break;
    }

    motionQueue[queueTail] = cmd;
    queueTail = (queueTail + 1) % MOTION_QUEUE_SIZE;
}


/* Start a new motion primitive */

static void start_motion(MotionCommand cmd)
{
    reset_encoders();

    currentMotion = cmd;

    baseVelocity = 0;

    motionActive = 1;
}


/* Acceleration profile */

static void update_velocity(long progress, long target)
{
    if(progress < target / 3)
        baseVelocity += ACCEL_STEP;

    else if(progress > (2 * target) / 3)
        baseVelocity -= ACCEL_STEP;

    if(baseVelocity > MAX_VELOCITY)
        baseVelocity = MAX_VELOCITY;

    if(baseVelocity < 8)
        baseVelocity = 8;
}


/* Main motion update */

void update_motion(void)
{
    if(!motionActive)
    {
        if(queueHead != queueTail)
        {
            MotionCommand cmd = motionQueue[queueHead];

            queueHead = (queueHead + 1) % MOTION_QUEUE_SIZE;

            start_motion(cmd);
        }

        return;
    }

    long left = get_left_ticks();
    long right = get_right_ticks();

    long progress;

    /* Turning progress measured differently */

    if(currentMotion.type == MOVE_FORWARD_CELL)
        progress = (labs(left) + labs(right)) / 2;
    else
        progress = labs(left) + labs(right);

    update_velocity(progress, currentMotion.targetTicks);


    switch(currentMotion.type)
    {

        /* Forward motion with heading correction */

        case MOVE_FORWARD_CELL:
        {
            int error = left - right;

            int correction = error * 0.5;

            set_target_velocity(
                baseVelocity - correction,
                baseVelocity + correction
            );
        }
        break;


        /* Left turn */

        case TURN_LEFT_90:

            set_target_velocity(
                -baseVelocity,
                baseVelocity
            );

        break;


        /* Right turn */

        case TURN_RIGHT_90:

            set_target_velocity(
                baseVelocity,
                -baseVelocity
            );

        break;


        /* 180 turn */

        case TURN_180:

            set_target_velocity(
                baseVelocity,
                -baseVelocity
            );

        break;
    }


    /* Stop condition */

    if(progress >= currentMotion.targetTicks)
    {
        set_target_velocity(0,0);

        motionActive = 0;
    }
}
