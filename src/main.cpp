#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"
#include "SensorManager.h"
#include "FlightController.h"
#include "DroneLEDController.h"

// 🟢 Front LED on GPIO 5, 🔴 Rear LED on GPIO 43
DroneLEDController ledController(5, 43);

// Drone state tracking
DroneState currentState = STATE_INIT;
FlightMode currentMode = MODE_STABLE;
bool recording = false;
bool photoFlash = false;

// === Global Managers ===
ComManager comManager;
SensorManager sensorManager;
FlightController flightController(&sensorManager, &comManager);

// Prototypes
FlightMode select_mode();
void print_mode_configuration(FlightMode mode);
void apply_bmi323_mode(FlightMode mode);
void apply_bmp390_mode(FlightMode mode);
void run_calibration_sequence_startup();

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(3000);

    // 🔹 Step 1: Initialize LEDs and show INIT state
    ledController.begin();
    currentState = STATE_INIT;
    ledController.update(currentState, currentMode, recording, photoFlash);

    // Optional: blink for 2 seconds while waiting for mode selection
    unsigned long initStart = millis();
    while (millis() - initStart < 2000) {
        ledController.update(currentState, currentMode, recording, photoFlash);
        delay(50);
    }

    // 🔹 Step 2: Initialize CC2500 receiver
    comManager.begin();
    delay(20);

    if (!comManager.selfTest()) {
        Serial.println("[ERROR] CC2500 self-test failed!");
    }

    delay(50);

    // 🔹 Step 3: BMI323 & BMP390 Init
    bmi323_init();
    delay(50);

    if (!bmp390_init_all()) {
        Serial.println("[BMP390] Init failed!");
        return;
    }

    // 🔹 Step 4: FlightController init (motors)
    flightController.begin();

    Serial.println(F("\nDrone Mode Selector Starting..."));

    // 🔹 Step 5: Mode selection
    FlightMode selected = select_mode();
    Serial.println("[DEBUG] Mode selected OK");

    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);

    // 🔹 Step 6: Update LEDs with selected mode
    currentMode = selected;
    ledController.update(currentState, currentMode, recording, photoFlash);

    print_mode_configuration(selected);
    run_calibration_sequence_startup();

    Serial.println("[ERROR] BMI323 FIFO setup started ");

    if (!bmi323_setup_fifo()) {
        Serial.println("[ERROR] BMI323 FIFO setup failed");
        return;
    }

    if (!bmp390_begin()) {
        Serial.println("[ERROR] BMP390 setup failed");
    }

    delay(1000);
    Serial.println(F("Flight mode configuration applied."));
}


void loop() {
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    float dt = (now - last_ms) * 0.001f;
    last_ms = now;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f; // safe fallback

    // Update sensors + EKF
    sensorManager.update();

    delayMicroseconds(500); // small delay to allow FIFO to fill
    // Update RC inputs
    comManager.update();

    delayMicroseconds(500); // small delay to allow FIFO to fill

    // Map RC input to flight controller setpoints if new data received
    if (comManager.hasNewData()) {
        flightController.roll_set  = map(comManager.roll,   -100, 100, -30, 30); // degrees
        flightController.pitch_set = map(comManager.pitch,  -100, 100, -30, 30); // degrees
        flightController.yaw_set   = map(comManager.yaw,    -100, 100, -90, 90); // degrees
        flightController.alt_set   = map(comManager.throttle, 0, 255, 0, 10);   // meters
    }

    // Update flight controller with dt
    flightController.update(dt);

    // 🔹 Step 5: Update LEDs each loop
    ledController.update(currentState, currentMode, recording, photoFlash);
    photoFlash = false; // reset after one update

    delay(5); // maintain sensor update rate
}
