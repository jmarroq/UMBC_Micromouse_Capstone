#include "sensor.h"
#include "stm32f4xx_hal.h"

// Internal GPIO/ADC mapping
#define IR_FRONT_GPIO_Port GPIOA
#define IR_FRONT_Pin GPIO_PIN_0
#define IR_LEFT_GPIO_Port  GPIOA
#define IR_LEFT_Pin  GPIO_PIN_1
#define IR_RIGHT_GPIO_Port GPIOA
#define IR_RIGHT_Pin GPIO_PIN_2

// FSM state
static SensorData_t sensor_data;
static SensorID_t current_sensor = SENSOR_FRONT;
static uint8_t phase = 0;  // 0=enable, 1=wait, 2=read
static uint32_t last_tick = 0;


// Internal helper functions (drivers)
static void EnableEmitter(SensorID_t id) {
    switch (id) {
        case SENSOR_FRONT: HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin, GPIO_PIN_SET); break;
        case SENSOR_LEFT:  HAL_GPIO_WritePin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin,  GPIO_PIN_SET); break;
        case SENSOR_RIGHT: HAL_GPIO_WritePin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin, GPIO_PIN_SET); break;
        default: break;
    }
}

static void DisableEmitter(SensorID_t id) {
    switch (id) {
        case SENSOR_FRONT: HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin, GPIO_PIN_RESET); break;
        case SENSOR_LEFT:  HAL_GPIO_WritePin(IR_LEFT_GPIO_Port,  IR_LEFT_Pin,  GPIO_PIN_RESET); break;
        case SENSOR_RIGHT: HAL_GPIO_WritePin(IR_RIGHT_GPIO_Port, IR_RIGHT_Pin, GPIO_PIN_RESET); break;
        default: break;
    }
}

static uint16_t ReadADC(SensorID_t id) {
    // TODO: implement ADC channel selection and read
    return 0;
}


// Internal helper functions (filters)
#define FILTER_SIZE 4
static float last_values[SENSOR_COUNT][FILTER_SIZE] = {0};
static uint8_t index[SENSOR_COUNT] = {0};

static float ConvertToDistance(uint16_t raw_value) {
    return 4800.0f / (raw_value + 20); // placeholder calibration
}

static float ApplyAverage(SensorID_t id, float new_value) {
    last_values[id][index[id]] = new_value;
    index[id] = (index[id] + 1) % FILTER_SIZE;
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += last_values[id][i];
    return sum / FILTER_SIZE;
}


// Public Functions for API
void Sensors_Init(void) {
    // GPIO init
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = IR_FRONT_Pin | IR_LEFT_Pin | IR_RIGHT_Pin;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC init TODO
}

void Sensors_Update(void) {
    uint32_t now = HAL_GetTick();

    switch (phase) {
        case 0: // enable current sensor
            EnableEmitter(current_sensor);
            last_tick = now;
            phase = 1;
            break;

        case 1: // wait for stabilization
            if (now - last_tick >= 5) phase = 2;
            break;

        case 2: // read and disable
            sensor_data.sensors[current_sensor].raw_value = ReadADC(current_sensor);
            DisableEmitter(current_sensor);

            float dist = ConvertToDistance(sensor_data.sensors[current_sensor].raw_value);
            sensor_data.sensors[current_sensor].distance_cm = ApplyAverage(current_sensor, dist);

            current_sensor = (current_sensor + 1) % SENSOR_COUNT;
            phase = 0;
            break;
    }
}

SensorData_t Sensors_GetData(void) {
    return sensor_data;
}
