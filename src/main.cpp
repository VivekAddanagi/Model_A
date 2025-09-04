#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"
#include "SensorManager.h"
#include "FlightController.h"

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

    // Initialize CC2500 receiver
    comManager.begin();

    delay(50);
    
    // BMI323 Init
    bmi323_init();

    delay(50);

    // BMP390 Init
    if (!bmp390_init_all()) {
        Serial.println("[BMP390] Init failed!");
        return;
    }

   // sensorManager.begin(101325); // default sea level pressure

    // FlightController init
    flightController.begin(); // sets up motors

    Serial.println(F("\nDrone Mode Selector Starting..."));
    FlightMode selected = select_mode();
    Serial.println("[DEBUG] Mode selected OK");

    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);
    print_mode_configuration(selected);
    run_calibration_sequence_startup();
   // delay(500);

    Serial.println("[ERROR] BMI323 FIFO setup started ");

// BMI323 FIFO
    if (!bmi323_setup_fifo()) {
    Serial.println("[ERROR] BMI323 FIFO setup failed");
    return;
}
    
    // BMP390 Init (AFTER mode selection!)
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

    // Update RC inputs
    comManager.update();

    // Map RC input to flight controller setpoints if new data received
    if (comManager.hasNewData()) {
        flightController.roll_set  = map(comManager.roll,   -100, 100, -30, 30); // degrees
        flightController.pitch_set = map(comManager.pitch,  -100, 100, -30, 30); // degrees
        flightController.yaw_set   = map(comManager.yaw,    -100, 100, -90, 90); // degrees
        flightController.alt_set   = map(comManager.throttle, 0, 255, 0, 10);   // meters
    }

    // Update flight controller with dt
    flightController.update(dt);  // only dt
    delay(4); // maintain sensor update rate
}
