#include <Arduino.h>

// GPIO setup
#define EMITTER_FRONT  40
#define EMITTER_RIGHT  41
#define EMITTER_BACK   38
#define EMITTER_LEFT   39
#define IR_RECEIVER    1  // ADC pin

#define IR_SETTLE_TIME_MICROS  200
#define IR_READ_DELAY_MICROS   300
#define SENSOR_POLL_INTERVAL_MS 25
#define NUM_DIRECTIONS 4

enum Direction { FRONT, RIGHT, BACK, LEFT };

uint8_t emitterPins[NUM_DIRECTIONS] = {EMITTER_FRONT, EMITTER_RIGHT, EMITTER_BACK, EMITTER_LEFT};
int irValues[NUM_DIRECTIONS];

unsigned long lastPollTime = 0;

// Initialize pins
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_DIRECTIONS; i++) {
    pinMode(emitterPins[i], OUTPUT);
    digitalWrite(emitterPins[i], LOW);
  }
  analogReadResolution(12); // ESP32 supports up to 12 bits
}

// Scanning logic
void scanIR() {
  for (int dir = 0; dir < NUM_DIRECTIONS; dir++) {
    digitalWrite(emitterPins[dir], HIGH);
    delayMicroseconds(IR_SETTLE_TIME_MICROS);

    // Take 3 readings and average
    int sum = 0;
    for (int i = 0; i < 3; i++) {
      sum += analogRead(IR_RECEIVER);
      delayMicroseconds(IR_READ_DELAY_MICROS);
    }
    irValues[dir] = sum / 3;

    digitalWrite(emitterPins[dir], LOW);
  }
}

// Decide avoidance action
void handleAvoidance() {
  const int dangerThreshold = 2000; // Example threshold - tune via calibration

  if (irValues[FRONT] > dangerThreshold) {
    Serial.println("Obstacle Ahead: Slowing forward motion");
    // Here: reduce throttle, block forward input, or hover
  }

  if (irValues[LEFT] > dangerThreshold) {
    Serial.println("Obstacle Left: Nudge Right");
  }

  if (irValues[RIGHT] > dangerThreshold) {
    Serial.println("Obstacle Right: Nudge Left");
  }

  if (irValues[BACK] > dangerThreshold) {
    Serial.println("Obstacle Behind: Stop reverse motion");
  }
}

void loop() {
  if (millis() - lastPollTime >= SENSOR_POLL_INTERVAL_MS) {
    scanIR();
    handleAvoidance();
    lastPollTime = millis();
  }

  // ... Insert rest of flight logic here ...
}
