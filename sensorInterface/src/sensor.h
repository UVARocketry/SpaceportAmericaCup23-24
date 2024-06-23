
#include "Arduino.h"
class SensorLogger {
public:
    template <typename T>
    void print(T t) const {
        Serial.print(t);
    }
    template <typename T>
    void println(T t) const {
        Serial.println(t);
    }
};
class SensorInitOptions {};
class SensorReadOptions {};
template <class Data>
class SensorInterface {
public:
private:
    void (*const init)(SensorInitOptions, const SensorLogger&);
    Data (*const read)(SensorReadOptions, const SensorLogger&);
    const SensorLogger logger;
    SensorInterface() = delete;

public:
    SensorInterface(
        void (*init)(SensorInitOptions, const SensorLogger&),
        Data (*read)(SensorReadOptions, const SensorLogger&)
    )
      : init(init), read(read), logger(SensorLogger()) {
    }

    void doInit(SensorInitOptions options) const {
        init(options, logger);
    }
    Data doRead(SensorReadOptions options) const {
        return read(options, logger);
    }
};
