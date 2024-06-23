// Basic demo for magnetometer readings from Adafruit LIS3MDL

#include "Adafruit_LIS3MDL.h"
#include "sensor.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Servo.h"
Servo myservo; // create servo object to control a servo
#define LED_L2 15

void setup() {
    myservo.attach(19);
    pinMode(LED_L2, OUTPUT);

    digitalWrite(LED_L2, HIGH);

    // Serial.begin(9600);
    // // delay(10000);
    // // while (!Serial)
    // //     delay(10); // will pause Zero, Leonardo, etc until serial console
    // //     opens
    // Serial.println("Hello!");
    // magnetSensor.doInit(SensorInitOptions{});
}
double step = 1;
double minAng = 0;
double maxAng = 180;
double pos = 0;
void loop() {
    for (int pos = minAng; pos <= maxAng; pos += step) {
        myservo.write(pos);
        delay(15);
    }
    for (int pos = maxAng; pos >= minAng; pos -= step) {
        myservo.write(pos);
        delay(15);
    }
}

// int main() {
//     setup();
//     while (1) {
//         loop();
//     }
// }
