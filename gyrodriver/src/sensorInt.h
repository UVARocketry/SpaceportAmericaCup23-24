
#include "Arduino.h"

#define DEBUG 1
// Now now before u all kill me for writing a wrapper function for println,
// this is so that we can easily disable output on an actual release build
// Then the optimizer should remove any trace of this class from the binary
class SensorLogger {
public:
    template <typename T>
    void print(T t) {
#if DEBUG
        Serial.print(t);
#endif
    }
    template <typename T>
    void println(T t) {
#if DEBUG
        Serial.println(t);
#endif
    }
};
// This is for whatever options we want to pass to all sensors,
// like the SPI pins, I2C pins, etc.
class SensorInitOptions {
public:
};
class SensorReadOptions {
public:
};

// This is the interface that all sensors can plug functions into
template <class Data>
class SensorInterface {
public:
private:
    // Make them constants
    void (*const init)(SensorInitOptions, SensorLogger&);
    Data (*const read)(SensorReadOptions, SensorLogger&);
    SensorLogger logger;

public:
    // Delete the default constructor bc we dont want it to accidentally be
    // called
    SensorInterface() = delete;
    SensorInterface(
        void (*init)(SensorInitOptions, SensorLogger&),
        Data (*read)(SensorReadOptions, SensorLogger&)
    )
      : init(init), read(read) {
        logger = SensorLogger();
    }

    // If this were OCaml, we could just have partial function application, but
    // unfortunately we're not, so we have to do this
    void doInit(SensorInitOptions options) {
        init(options, logger);
    }
    Data doRead(SensorReadOptions options) {
        return read(options, logger);
    }
};
