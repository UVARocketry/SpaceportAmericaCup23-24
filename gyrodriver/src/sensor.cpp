#include "sensor.h"
#include <Adafruit_LSM6DSO32.h>
int64_t lastTime = 0;

// For SPI mode, we need a CS pin
#define LSM_CS 9
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

Adafruit_LSM6DSO32 dso32;
void initSensor(SensorInitOptions options, SensorLogger& logger) {
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit LSM6DSO32 test!");

    // if (!dso32.begin_I2C()) {
    // if (!dso32.begin_SPI(LSM_CS)) {
    if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
        Serial.println("Failed to find LSM6DSO32 chip");
        while (1) {
            delay(10);
        }
    }

    Serial.println("LSM6DSO32 Found!");

    dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    Serial.print("Accelerometer range set to: ");
    switch (dso32.getAccelRange()) {
    case LSM6DSO32_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case LSM6DSO32_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case LSM6DSO32_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    case LSM6DSO32_ACCEL_RANGE_32_G:
        Serial.println("+-32G");
        break;
    }

    dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    Serial.print("Gyro range set to: ");
    switch (dso32.getGyroRange()) {
    case LSM6DS_GYRO_RANGE_125_DPS:
        Serial.println("125 degrees/s");
        break;
    case LSM6DS_GYRO_RANGE_250_DPS:
        Serial.println("250 degrees/s");
        break;
    case LSM6DS_GYRO_RANGE_500_DPS:
        Serial.println("500 degrees/s");
        break;
    case LSM6DS_GYRO_RANGE_1000_DPS:
        Serial.println("1000 degrees/s");
        break;
    case LSM6DS_GYRO_RANGE_2000_DPS:
        Serial.println("2000 degrees/s");
        break;
    case ISM330DHCX_GYRO_RANGE_4000_DPS:
        break; // unsupported range for the DSO32
    }

    dso32.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
    Serial.print("Accelerometer data rate set to: ");
    switch (dso32.getAccelDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
    case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
    case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
    case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
    case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
    case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
    case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
    case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
    case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
    case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
    case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
    }

    dso32.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
    Serial.print("Gyro data rate set to: ");
    switch (dso32.getGyroDataRate()) {
    case LSM6DS_RATE_SHUTDOWN:
        Serial.println("0 Hz");
        break;
    case LSM6DS_RATE_12_5_HZ:
        Serial.println("12.5 Hz");
        break;
    case LSM6DS_RATE_26_HZ:
        Serial.println("26 Hz");
        break;
    case LSM6DS_RATE_52_HZ:
        Serial.println("52 Hz");
        break;
    case LSM6DS_RATE_104_HZ:
        Serial.println("104 Hz");
        break;
    case LSM6DS_RATE_208_HZ:
        Serial.println("208 Hz");
        break;
    case LSM6DS_RATE_416_HZ:
        Serial.println("416 Hz");
        break;
    case LSM6DS_RATE_833_HZ:
        Serial.println("833 Hz");
        break;
    case LSM6DS_RATE_1_66K_HZ:
        Serial.println("1.66 KHz");
        break;
    case LSM6DS_RATE_3_33K_HZ:
        Serial.println("3.33 KHz");
        break;
    case LSM6DS_RATE_6_66K_HZ:
        Serial.println("6.66 KHz");
        break;
    }
}

GyroData runSensor(SensorReadOptions options, SensorLogger& logger) {

    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    dso32.getEvent(&accel, &gyro, &temp);

    GyroData data;
    data.accX = accel.acceleration.x;
    data.accY = accel.acceleration.y;
    data.accZ = accel.acceleration.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.y;
    data.gyroZ = gyro.gyro.z;
    data.temp = temp.temperature;
    return data;
}
SensorInterface<GyroData> gyroSensor(initSensor, runSensor);
