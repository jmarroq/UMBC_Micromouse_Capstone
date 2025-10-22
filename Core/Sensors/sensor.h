#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// Sensor identifiers
typedef enum {
    SENSOR_FRONT = 0,
    SENSOR_LEFT,
    SENSOR_RIGHT,
    SENSOR_COUNT
} SensorID_t;

// Single sensor reading
typedef struct {
    uint16_t raw_value;   // ADC raw
    float distance_cm;    // converted and filtered
} SensorReading_t;

// All sensor readings
typedef struct {
    SensorReading_t sensors[SENSOR_COUNT];
} SensorData_t;

// Public Functions for API
void Sensors_Init(void);
void Sensors_Update(void);          // call in main loop or timer
SensorData_t Sensors_GetData(void); // get latest data

#endif
