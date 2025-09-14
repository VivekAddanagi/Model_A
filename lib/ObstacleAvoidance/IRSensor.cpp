#include <Arduino.h>
#include "IRSensor.h"

// === GPIO setup ===
#define EMITTER_FRONT  40
#define EMITTER_RIGHT  41
#define EMITTER_BACK   38
#define EMITTER_LEFT   39
#define IR_RECEIVER    1  // ADC pin

// Timing constants
#define IR_SETTLE_TIME_MICROS  200
#define IR_READ_DELAY_MICROS   300
#define NUM_DIRECTIONS 4

// Local emitter pin map (not exposed outside this file)
static const uint8_t emitterPins[NUM_DIRECTIONS] = {
    EMITTER_FRONT, EMITTER_RIGHT, EMITTER_BACK, EMITTER_LEFT
};

// === Public Methods ===

void IRSensor::begin() {
    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        pinMode(emitterPins[i], OUTPUT);
        digitalWrite(emitterPins[i], LOW);
    }
    analogReadResolution(12); // ESP32 supports 12-bit ADC
}

void IRSensor::poll() {
    for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
        digitalWrite(emitterPins[dir], HIGH);
        delayMicroseconds(IR_SETTLE_TIME_MICROS);

        // Take 3 readings and average
        int sum = 0;
        for (int i = 0; i < 3; i++) {
            sum += analogRead(IR_RECEIVER);
            delayMicroseconds(IR_READ_DELAY_MICROS);
        }
        readings[dir] = sum / 3;

        digitalWrite(emitterPins[dir], LOW);
    }
}

float IRSensor::getDistance(Direction dir) const {
    // Currently returns raw ADC value (0–4095).
    // To convert to cm, you’ll need calibration.
    return readings[dir];
}

bool IRSensor::isNear(Direction dir, int threshold) const {
    return readings[dir] > threshold;
}

void IRSensor::handleAvoidance() {
    const int dangerThreshold = 2000; // Tune via calibration

    if (readings[FRONT] > dangerThreshold) {
        Serial.println("Obstacle Ahead: Slowing forward motion");
    }
    if (readings[LEFT] > dangerThreshold) {
        Serial.println("Obstacle Left: Nudge Right");
    }
    if (readings[RIGHT] > dangerThreshold) {
        Serial.println("Obstacle Right: Nudge Left");
    }
    if (readings[BACK] > dangerThreshold) {
        Serial.println("Obstacle Behind: Stop reverse motion");
    }
}
