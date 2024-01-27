
#include "sensor.h"
void setup() {
    initSensor();
}

void loop() {
    runSensor();
}

int main() {
    setup();
    while (1) {
        loop();
    }
}
