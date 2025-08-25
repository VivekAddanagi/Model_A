#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"
#include "SensorManager.h"

// === Global Managers ===
ComManager comManager;
SensorManager sensorManager;

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

    //comManager.begin();

    bmi323_init();
   // BMP390 Init 
   if (!bmp390_init_all()) {
    Serial.println("[BMP390] Init failed!");
    return;
}


    Serial.println(F("\nDrone Mode Selector Starting..."));
    FlightMode selected = select_mode();
    Serial.println("[DEBUG] Mode selected OK");

    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);
    print_mode_configuration(selected);
    run_calibration_sequence_startup();

    // BMI323 FIFO
    bmi323_setup_fifo();

    // BMP390 Init (AFTER mode selection!)
    if (!bmp390_begin()) {
        Serial.println("[ERROR] BMP390 setup failed");
    }

    delay(1000);
    Serial.println(F("Flight mode configuration applied."));
}

void loop() {
    sensorManager.update(); // calls bmp390_process_fifo() internally
    //comManager.update();
    delay(2);
}
