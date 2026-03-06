#ifndef MOTORS_H
#define MOTORS_H
#ifdef __cplusplus
extern "C" {
#endif
void motors_init();

void set_motors(int leftSpeed, int rightSpeed);

void stop_motors();

#ifdef __cplusplus
}
#endif
#endif
