#include <Arduino.h>
#include "CommManager.h"

ComManager com;

void setup() {
    Serial.begin(115200);
    com.begin();
}

void loop() {
    com.update();

    if (com.hasNewData()) {
        Serial.printf("[MAIN] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d TO=%d FS=%d PH=%d VID=%d\n",
                      com.yaw, com.pitch, com.roll, com.throttle, com.mode,
                      com.takeoff, com.failsafe, com.photo, com.video);
    }

    delay(20); // Adjust as needed
}
