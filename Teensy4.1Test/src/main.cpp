#include "core_pins.h"
#include <Arduino.h>

// Used for the BMP388 sensor definitions
#include <Adafruit_BMP3XX.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_Sensor.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 3

// extra CS pins for the other sensors
#define ADXL375_CS 7
#define LSM_CS 9
#define LIS3MDL_CS 5

#define SEALEVELPRESSURE_HPA (1013.25)

// Adafruit_BMP3XX bmp;

// End of BMP388 sensor definitions

// Define the LED pins
// A4
#define LED_L1 18
// A1
#define LED_L2 15
// A0
#define LED_L3 14
// A11
#define LED_L4 25
// CS3
#define LED_L5 37
// CS2
#define LED_BLUE_RIGHT 36

#define BUZZER 23

// Blink Example, Turns an LED on for one second, then off for one second,
// repeatedly.

// The setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_L1, OUTPUT);
    pinMode(LED_L2, OUTPUT);
    pinMode(LED_L3, OUTPUT);
    // pinMode(LED_L4, OUTPUT);
    // pinMode(LED_L5, OUTPUT);
    pinMode(LED_BLUE_RIGHT, OUTPUT);
    // Serial.begin(9600);
    // while (!Serial) {
    //     delay(10);
    // }
    // Serial.println(bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI));
}

void testAllLeds() {
    loop();
    Serial.println("Testing all LEDs");
    Serial.println("LED_L1");
    digitalWrite(LED_L1, HIGH);
    delay(1000);
    digitalWrite(LED_L1, LOW);
    Serial.println("LED_L2");
    digitalWrite(LED_L2, HIGH);
    delay(1000);
    digitalWrite(LED_L2, LOW);
    Serial.println("LED_L3");
    digitalWrite(LED_L3, HIGH);
    delay(1000);
    digitalWrite(LED_L3, LOW);
    Serial.println("LED_L4");
    digitalWrite(LED_L4, HIGH);
    delay(1000);
    digitalWrite(LED_L4, LOW);
    Serial.println("LED_L5");
    digitalWrite(LED_L5, HIGH);
    delay(1000);
    digitalWrite(LED_L5, LOW);
    Serial.println("LED_BLUE_RIGHT");
    digitalWrite(LED_BLUE_RIGHT, HIGH);
    delay(1000);
    digitalWrite(LED_BLUE_RIGHT, LOW);
}

// the loop function runs over and over again forever
void loop() {
    testAllLeds();
    // int time = 0;
    // while (time < 10000) {
    //     digitalWrite(LED_BLUE_RIGHT, HIGH);
    //     delayMicroseconds(5);
    //     digitalWrite(LED_BLUE_RIGHT, LOW);
    //     delayMicroseconds(10);
    //     time += 15;
    // }
    // digitalWrite(LED_BUILTIN, HIGH);
    // // digitalWrite(BUZZER, HIGH);
    // digitalWrite(LED_L1, HIGH);
    // digitalWrite(LED_L2, HIGH);
    // digitalWrite(LED_L3, HIGH);
    // // digitalWrite(LED_L4, HIGH);
    // // digitalWrite(LED_L5, HIGH);
    // digitalWrite(LED_BLUE_RIGHT, HIGH);
    // delayMicroseconds(100000);
    // digitalWrite(LED_BUILTIN, LOW);
    // // digitalWrite(BUZZER, LOW);
    // delayMicroseconds(100000);
}

int main() {
    setup();
    while (1) {
        loop();
    }
    return 0;
}
