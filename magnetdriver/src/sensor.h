
#include "../../sensorInterface/src/sensor.h"

struct MagnetData {
    float x;
    float y;
    float z;
};

extern SensorInterface<MagnetData> magnetSensor;
