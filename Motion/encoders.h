#ifndef ENCODERS_H
#define ENCODERS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

void encoders_init(void);

void reset_encoders(void);

long get_left_ticks(void);

long get_right_ticks(void);
#ifdef __cplusplus
}
#endif
#endif


