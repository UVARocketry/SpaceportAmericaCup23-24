#include "sensor.h"
#include "Adafruit_LIS3MDL.h"
Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 5
void setupMagnetometer(SensorInitOptions options, const SensorLogger& logger) {

    logger.println("Adafruit LIS3MDL test!");

    // Try to initialize!
    if (!lis3mdl.begin_SPI(LIS3MDL_CS)) { // hardware SPI mode
                                          // if (!lis3mdl.begin_SPI(
        //         LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO,
        //         LIS3MDL_MOSI
        //     )) { // soft SPI
        logger.println("Failed to find LIS3MDL chip");
        return;
    }
    logger.println("LIS3MDL Found!");

    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    logger.print("Performance mode set to: ");
    switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE:
        logger.println("Low/");
        break;
    case LIS3MDL_MEDIUMMODE:
        logger.println("Medium");
        break;
    case LIS3MDL_HIGHMODE:
        logger.println("High");
        break;
    case LIS3MDL_ULTRAHIGHMODE:
        logger.println("Ultra-High");
        break;
    }

    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    logger.print("Operation mode set to: ");
    // Single shot mode will complete conversion and go into power down
    switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE:
        logger.println("Continuous");
        break;
    case LIS3MDL_SINGLEMODE:
        logger.println("Single mode");
        break;
    case LIS3MDL_POWERDOWNMODE:
        logger.println("Power-down");
        break;
    }

    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    // You can check the datarate by looking at the frequency of the DRDY
    // pin
    logger.print("Data rate set to: ");
    switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ:
        logger.println("0.625 Hz");
        break;
    case LIS3MDL_DATARATE_1_25_HZ:
        logger.println("1.25 Hz");
        break;
    case LIS3MDL_DATARATE_2_5_HZ:
        logger.println("2.5 Hz");
        break;
    case LIS3MDL_DATARATE_5_HZ:
        logger.println("5 Hz");
        break;
    case LIS3MDL_DATARATE_10_HZ:
        logger.println("10 Hz");
        break;
    case LIS3MDL_DATARATE_20_HZ:
        logger.println("20 Hz");
        break;
    case LIS3MDL_DATARATE_40_HZ:
        logger.println("40 Hz");
        break;
    case LIS3MDL_DATARATE_80_HZ:
        logger.println("80 Hz");
        break;
    case LIS3MDL_DATARATE_155_HZ:
        logger.println("155 Hz");
        break;
    case LIS3MDL_DATARATE_300_HZ:
        logger.println("300 Hz");
        break;
    case LIS3MDL_DATARATE_560_HZ:
        logger.println("560 Hz");
        break;
    case LIS3MDL_DATARATE_1000_HZ:
        logger.println("1000 Hz");
        break;
    }

    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    logger.print("Range set to: ");
    switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS:
        logger.println("+-4 gauss");
        break;
    case LIS3MDL_RANGE_8_GAUSS:
        logger.println("+-8 gauss");
        break;
    case LIS3MDL_RANGE_12_GAUSS:
        logger.println("+-12 gauss");
        break;
    case LIS3MDL_RANGE_16_GAUSS:
        logger.println("+-16 gauss");
        break;
    }

    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(
        false, false, true, // enable z axis
        true,               // polarity
        false,              // don't latch
        true
    ); // enabled!
}

MagnetData
runMagnetometer(SensorReadOptions options, const SensorLogger& logger) {
    sensors_event_t event;
    lis3mdl.getEvent(&event);
    MagnetData data;
    data.x = event.magnetic.x;
    data.y = event.magnetic.y;
    data.z = event.magnetic.z;
    return data;
}

SensorInterface<MagnetData> magnetSensor =
    SensorInterface<MagnetData>(setupMagnetometer, runMagnetometer);
