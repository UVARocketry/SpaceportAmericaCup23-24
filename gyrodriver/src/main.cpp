#include "sensor.h"
void setup() {
    Serial.begin(115200);
    gyroSensor.doInit(SensorInitOptions());
}

void loop() {
    auto data = gyroSensor.doRead(SensorReadOptions());

    Serial.print("Accel X: ");
    Serial.print(data.accX);
    Serial.print(" \tY: ");
    Serial.print(data.accY);
    Serial.print(" \tZ: ");
    Serial.print(data.accZ);
    Serial.println(" m/s^2 ");

    Serial.print("Gyro X: ");
    Serial.print(data.gyroX);
    Serial.print(" \tY: ");
    Serial.print(data.gyroY);
    Serial.print(" \tZ: ");
    Serial.print(data.gyroZ);
    Serial.println(" radians/s ");

    Serial.print("Temperature ");
    Serial.print(data.temp);
    Serial.println(" deg C");
    delay(1000);
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
