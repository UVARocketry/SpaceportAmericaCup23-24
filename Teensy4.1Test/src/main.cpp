//Used for general definitions
#include <Arduino.h>

//Used for sensor definitions
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#define SCK 13
#define MISO 12
#define MOSI 11

//Used for the BMP388 sensor definitions
#include <Adafruit_BMP3XX.h>
#define BMP_CS 3
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

//Used for ADXL375 sensor definitions
#include <Adafruit_ADXL375.h>
#define ADXL375_CS 7
Adafruit_ADXL375 accel = Adafruit_ADXL375(SCK, MISO, MOSI, ADXL375_CS, 12345);
//Adafruit_ADXL375 accel = Adafruit_ADXL375(ADXL375_CS, &SPI, 12345);

//Used for LIS3MDL sensor definitions
#include <Adafruit_LIS3MDL.h>
#define LIS3MDL_CS 5

//Used for LSM6DSO sensor definitions
#include <Adafruit_LSM6DSOX.h>
#define LSM_CS 9
Adafruit_LSM6DSOX sox;

//Used for SPI Flash chip definitions
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#define SPIFlash_PIN 27
Adafruit_FlashTransport_SPI flashTransport(SPIFlash_PIN, SPI);
Adafruit_SPIFlash flash(&flashTransport);

//Used for the Servo motor definitions
#include <Servo.h>
#define SERVO_PIN 19
Servo servo; 

//Define the onboard LED pins
#define LED_L1 18 //A4
#define LED_L2 15 //A1
#define LED_L3 14 //A0

#define LED_L4 25 //A11
#define LED_L5 37 //CS3
#define LED_L6 36 //CS2


// The setup function runs once when you press reset or power the board
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

}

  /*
  //Setup for the BMP388 sensor
  //Try to initialize the BMP388 sensor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, SCK, MISO, MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //End of setup for BMP388 sensor
  */

  /*
  //Setup for the ADXL375 sensor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("ADXL375 Accelerometer Test"); Serial.println("");

  // Initialise the sensor 
  if(!accel.begin())
  {
    // There was a problem detecting the ADXL375 ... check your connections
    Serial.println("Ooops, no ADXL375 detected ... Check your wiring!");
    while(1);
  }

  // Range is fixed at +-200g

  // Display some basic information on this sensor
  accel.printSensorDetails();
  //Display data rate function not included, if needed see example
  //displayDataRate();
  Serial.println("");
  //End of setup for ADXL375 sensor
  */

  /*
  //Setup for the onboard LED's
  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L2, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(LED_L4, OUTPUT);
  pinMode(LED_L5, OUTPUT);
  pinMode(LED_L6, OUTPUT);
  */

  /*
  //Setup for the Servo motor
  servo.attach(SERVO_PIN);  // attaches the servo on pin 19 to the servo object
  servo.write(0); 
  */

  /*
  //Setup for the Serial Flash Chip sensor
  Serial.println("Adafruit Serial Flash Info example");
  flash.begin();

  Serial.print("JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.print(flash.size() / 1024);
  Serial.println(" KB");
  */

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  printf("Hello World\n");
  delay(1000);
}
  /*
  //Code for the BMP388 sensor
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
  //End of code for the BMP388 sensor
  */

  /*
  //Code for the ADXL375 sensor
  // Get a new sensor event 
  sensors_event_t event;
  accel.getEvent(&event);

  // Display the results (acceleration is measured in m/s^2)
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  delay(500);
  //End of code for the ADXL375 sensor
  */


  /*
  //Code for the Servo motor
  for (int pos = 0; pos <= 180; pos += 1) {  // rotate slowly from 0 degrees to 180 degrees, one by one degree
    // in steps of 1 degree
    servo.write(pos);  // control servo to go to position in variable 'pos'
    delay(10);         // waits 10ms for the servo to reach the position
  }

  for (int pos = 180; pos >= 0; pos -= 1) {  // rotate from 180 degrees to 0 degrees, one by one degree
    servo.write(pos);                        // control servo to go to position in variable 'pos'
    delay(10);                               // waits 10ms for the servo to reach the position
  }
  //End of code for the Servo motor
  */

  /*
  //Code for the Onboard LEDs
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
  //End of code for the Onboard LEDs
  */

