#include "sensor_manager.h"
#include "sensor_drivers.h"
#include "sensor_filters.h"

static SensorData_t sensor_data;
static SensorID_t current_sensor = SENSOR_FRONT;
static uint8_t phase = 0;  // 0 = enable, 1 = wait, 2 = read
static uint32_t last_tick = 0;

void Sensors_Init(void) {
    SensorDrivers_Init();
}

void Sensors_Update(void) {
    uint32_t now = HAL_GetTick();

    switch (phase) {
        case 0: // Turn ON current sensor
            SensorDrivers_EnableEmitter(current_sensor);
            last_tick = now;
            phase = 1;
            break;

        case 1: // Wait for stabilization
            if (now - last_tick >= 5) { // 5 ms
                phase = 2;
            }
            break;

        case 2: // Read and turn OFF
            sensor_data.raw[current_sensor] = SensorDrivers_ReadADC(current_sensor);
            SensorDrivers_DisableEmitter(current_sensor);

            // Convert and filter
            float dist = SensorFilters_ConvertToDistance(sensor_data.raw[current_sensor]);
            sensor_data.distance_cm[current_sensor] =
                SensorFilters_ApplyAverage(current_sensor, dist);

            // Move to next sensor
            current_sensor = (current_sensor + 1) % SENSOR_COUNT;
            phase = 0;
            break;
    }
}

SensorData_t Sensors_GetData(void) {
    return sensor_data;
}
