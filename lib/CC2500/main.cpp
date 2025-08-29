/*
#include <Arduino.h>
#include "CC2500Receiver.h"

#define CS_PIN    10
#define SCK_PIN   12
#define MISO_PIN  13
#define MOSI_PIN  11

CC2500Receiver receiver(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN);

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("=== CC2500 Receiver Starting ===");
    receiver.begin();
}

void loop() {
    if (receiver.receivePacket()) {
        int8_t yaw, pitch, roll;
        uint8_t throttle, mode, takeoff, failsafe, photo, video;
        if (receiver.getLatestControlData(yaw, pitch, roll, throttle, mode, takeoff, failsafe, photo, video)) {
            // Already printed by internal debug
        }
    } else {
        Serial.println("[INFO] No packet received.");
    }

    if (receiver.isTimedOut(200)) {
        Serial.println("[FAILSAFE] No packet received within 200ms!");
    }

    delay(20);
}

*/