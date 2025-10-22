// Sensor Manager Header
// Define sensor list, data type, and functions
typedef enum {
    SENSOR_FRONT = 0,
    SENSOR_LEFT,
    SENSOR_RIGHT,
    SENSOR_COUNT
} SensorID_t;

typedef struct {
    uint16_t raw[SENSOR_COUNT];
    float distance_cm[SENSOR_COUNT];
} SensorData_t;

void Sensors_Init(void);
void Sensors_Update(void); 
SensorData_t Sensors_GetData(void);
