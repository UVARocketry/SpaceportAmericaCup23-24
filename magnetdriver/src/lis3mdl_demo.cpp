// Basic demo for magnetometer readings from Adafruit LIS3MDL

#include "Adafruit_LIS3MDL.h"
#include "sensor.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

void setup() {

    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    magnetSensor.doInit(SensorInitOptions{});
}
void loop() {

    MagnetData data = magnetSensor.doRead(SensorReadOptions{});
    Serial.print("X: ");
    Serial.print(data.x);
    Serial.print(" Y: ");
    Serial.print(data.y);
    Serial.print(" Z: ");
    Serial.println(data.z);
    delay(100);
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
