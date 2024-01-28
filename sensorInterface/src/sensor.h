
#include "Arduino.h"
class SensorLogger {
public:
    template <typename T>
    void print(T t) {
        Serial.print(t);
    }
    template <typename T>
    void println(T t) {
        Serial.println(t);
    }
};
class SensorInitOptions {
public:
};
class SensorReadOptions {
public:
};
template <class Data>
class SensorInterface {
public:
private:
    void (*const init)(SensorInitOptions, SensorLogger&);
    Data (*const read)(SensorReadOptions, SensorLogger&);
    SensorLogger logger;
    SensorInterface() = delete;

public:
    SensorInterface(
        void (*init)(SensorInitOptions, SensorLogger&),
        Data (*read)(SensorReadOptions, SensorLogger&)
    )
      : init(init), read(read) {
        logger = SensorLogger();
    }

    void doInit(SensorInitOptions options) {
        init(options, logger);
    }
    Data doRead(SensorReadOptions options) {
        return read(options, logger);
    }
};
