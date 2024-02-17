#include <Arduino.h>

//Used for the BMP388 sensor definitions
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 3

//extra CS pins for the other sensors
#define ADXL375_CS 7
#define LSM_CS 9
#define LIS3MDL_CS 5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

//End of BMP388 sensor definitions

//Define the LED pins
//A4
#define LED_L1 18
//A1
#define LED_L2 15
//A0
#define LED_L3 14
//A11
#define LED_L4 25
//CS3
#define LED_L5 37
//CS2
#define LED_L6 36

//Blink Example, Turns an LED on for one second, then off for one second, repeatedly.

// The setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  /*
  //Setup the onboard LED's
  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L2, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(LED_L4, OUTPUT);
  pinMode(LED_L5, OUTPUT);
  pinMode(LED_L6, OUTPUT);
  */

  /*
  //Try to initialize the BMP388 sensor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  */
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  printf("Hello World\n");
  Serial.println("World Hello");
  delay(1000);

  /*
  //Turn on the onboard LED's
  digitalWrite(LED_L1, HIGH);
  delay(1000);
  digitalWrite(LED_L1, LOW);
  delay(1000);
  printf("LED L1 Turned ON");
  delay(1000);

  digitalWrite(LED_L2, HIGH);
  delay(1000);
  digitalWrite(LED_L2, LOW);
  delay(1000);
  printf("LED L2 Turned ON");
  delay(1000);

  digitalWrite(LED_L3, HIGH);
  delay(1000);
  digitalWrite(LED_L3, LOW);
  delay(1000);
  printf("LED L3 Turned ON");
  delay(1000);

  digitalWrite(LED_L4, HIGH);
  delay(1000);
  digitalWrite(LED_L4, LOW);
  delay(1000);
  printf("LED L4 Turned ON");
  delay(1000);

  digitalWrite(LED_L5, HIGH);
  delay(1000);
  digitalWrite(LED_L5, LOW);
  delay(1000);
  printf("LED L5 Turned ON");
  delay(1000);

  digitalWrite(LED_L6, HIGH);
  delay(1000);
  digitalWrite(LED_L6, LOW);
  delay(1000);
  printf("LED L6 Turned ON");
  delay(1000);
  */


  /*
  // Try to print the temperature, pressure, and altitude from the BMP388 sensor
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
  */
}
