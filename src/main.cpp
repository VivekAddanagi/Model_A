#include <Arduino.h>

#include "CommManager.h"

CommManager commManager;

void setup() {
    Serial.begin(115200);
    commManager.begin();
}

void loop() {
    commManager.update();

    if (commManager.hasValidData()) {
        RXData rx = commManager.getControlData();
        // Use rx.throttle, rx.pitch, etc.
    }
}
