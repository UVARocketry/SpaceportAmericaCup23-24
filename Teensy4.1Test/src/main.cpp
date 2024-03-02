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
Adafruit_ADXL375 accel = Adafruit_ADXL375(ADXL375_CS, &SPI, 12345);

//Used for LIS3MDL sensor definitions
#include <Adafruit_LIS3MDL.h>
#define LIS3MDL_CS 5
Adafruit_LIS3MDL lis3mdl;

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

//Define the Buzzer pin
#define BUZZER_PIN 23

//Define the GPS sensor definitions
#include <TinyGPSPlus.h>
static const uint32_t GPSBaud = 9600;
//For reference, not needed since we are using the hardware serial pins, not software serial
//#define GPS_RXPIN 0
//#define GPS_TXPIN 1
#define GPSSerial Serial1
TinyGPSPlus gps;

//Define RFM95 Radio definitions
//Not that important to test rn
#define RFM95_CS 33

// The setup function runs once when you press reset or power the board
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  /*
  //Setup for the BMP388 sensor
  //Try to initialize the BMP388 sensor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  //if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
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
  //Setup for the LIS3MDL sensor
    Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LIS3MDL test!");
  
  // Try to initialize!
  //if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
  //End of setup for LIS3MDL sensor
  */

  /*
  //Setup for the LSM6DSO sensor
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSOX test!");

  //if (!sox.begin_I2C()) {
  if (!sox.begin_SPI(LSM_CS)) {
    // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );

  // sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);

  // sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  
  //End of setup for the LSM6DSO sensor
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
  //End of setup for the Serial Flash Chip sensor
  */

  /*
  //Setup for the Servo motor
  servo.attach(SERVO_PIN);  // attaches the servo on pin 19 to the servo object
  servo.write(0); 
  //End of setup for the Servo motor
  */

  /*
  //Setup for the onboard LED's
  pinMode(LED_L1, OUTPUT);
  pinMode(LED_L2, OUTPUT);
  pinMode(LED_L3, OUTPUT);
  pinMode(LED_L4, OUTPUT);
  pinMode(LED_L5, OUTPUT);
  pinMode(LED_L6, OUTPUT);
  //End of setup for the onboard LED's
  */
  
  /*
  //Setup for the Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  //Sound the buzzer at 1000 Hz for 1s
  tone(BUZZER_PIN, 1000, 1000); 
  //End of setup for the Buzzer
  */

  /*
  //Setup for the GPS sensor
  Serial.begin(115200);
  GPSSerial.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  //End of setup for the GPS sensor
  */
}


// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  printf("Hello World\n");
  delay(1000);

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
  //Code for the LIS3MDL sensor
  lis3mdl.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("\nX:  "); Serial.print(lis3mdl.x); 
  Serial.print("  \tY:  "); Serial.print(lis3mdl.y); 
  Serial.print("  \tZ:  "); Serial.println(lis3mdl.z); 

  // Or....get a new sensor event, normalized to uTesla 
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  // Display the results (magnetic field is measured in uTesla) 
  Serial.print("\tX: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
  Serial.println(" uTesla ");

  delay(100); 
  Serial.println();
  //End of code for the LIS3MDL sensor
  */

  /*
  //Code for the LSM6DSO sensor
  // Get a new normalized sensor event 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  // Display the results (acceleration is measured in m/s^2) 
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  // Display the results (rotation is measured in rad/s) 
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);
  //End of code for the LSM6DSO sensor
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

  /*
  //Code for the Buzzer
  tone(buzzerPin, 440); // A4
  delay(500);

  tone(buzzerPin, 494); // B4
  delay(500);

  tone(buzzerPin, 523); // C4
  delay(500);

  tone(buzzerPin, 587); // D4
  delay(500);

  tone(buzzerPin, 659); // E4
  delay(500);

  tone(buzzerPin, 698); // F4
  delay(500);

  tone(buzzerPin, 784); // G4
  delay(500);

  noTone(buzzerPin);
  delay(500);
  //End of code for the Buzzer
  */

  /*
  //Code for the GPS sensor
  // This sketch displays information every time a new sentence is correctly encoded.
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read())){
      Serial.print(F("Location: ")); 
    if (gps.location.isValid()){
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    }
    else{
    Serial.print(F("INVALID"));
    }
    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()){
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    }
    else{
    Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()){
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    }
    else{
    Serial.print(F("INVALID"));
    }
    Serial.println();
    }
  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
  //End of code for the GPS sensor
  */
}
