// #define WProgram_h
// #undef __DEBUG_DISABLE_DUMB__
#include "pins_arduino.h"
#include "sensor.h"
void setup() {
    Serial.begin(9600);
    delay(10000);
    Serial.println("Hello, world!");
    gyroSensor.doInit(SensorInitOptions());
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // auto data = gyroSensor.doRead(SensorReadOptions());
    //
    // Serial.print("Accel X: ");
    // Serial.print(data.accX);
    // Serial.print(" \tY: ");
    // Serial.print(data.accY);
    // Serial.print(" \tZ: ");
    // Serial.print(data.accZ);
    // Serial.println(" m/s^2 ");
    //
    // Serial.print("Gyro X: ");
    // Serial.print(data.gyroX);
    // Serial.print(" \tY: ");
    // Serial.print(data.gyroY);
    // Serial.print(" \tZ: ");
    // Serial.print(data.gyroZ);
    // Serial.println(" radians/s ");
    //
    // Serial.print("Temperature ");
    // Serial.print(data.temp);
    // Serial.println(" deg C");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
}

// int main() {
//     setup();
//     while (1) {
//         loop();
//     }
// }
