/*
#include <Arduino.h>
#include "IRSensor.h"

#define SENSOR_POLL_INTERVAL_MS 25

IRSensor irSensor;
unsigned long lastPollTime = 0;

void setup() {
    Serial.begin(115200);
    irSensor.begin();
}

void loop() {
    if (millis() - lastPollTime >= SENSOR_POLL_INTERVAL_MS) {
        irSensor.poll();
        irSensor.handleAvoidance();
        lastPollTime = millis();
    }

    // ... Insert rest of drone flight control logic here ...
}
*/