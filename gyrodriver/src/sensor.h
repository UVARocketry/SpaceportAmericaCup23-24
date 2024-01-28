
#include "sensorInt.h"

struct GyroData {
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
};

extern SensorInterface<GyroData> gyroSensor;
