#include <Arduino.h>
#include "bmi323.h"   // Existing BMI323 library
#include "bmp390.h"   // Existing BMP390 library
#include "ComManager.h" // Add ComManager header

// === Function prototypes ===
FlightMode select_mode();
void print_mode_configuration(FlightMode mode);
void apply_bmi323_mode(FlightMode mode);
void apply_bmp390_mode(FlightMode mode);
// Forward declare the manager (from calibration_sequence.cpp)
void run_calibration_sequence_startup();

// === Global ComManager instance ===
ComManager comManager;

// === Setup ===
void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(3000); // Allow time for Serial to initialize
    // Start the communication manager (CC2500 receiver)
    comManager.begin();
    

    // Initialize sensors using your existing init functions
    bmp390_init_all(); // <- your library's init function
    bmi323_init();     // <- your library's init function

    Serial.println(F("\nDrone Mode Selector Starting..."));

    FlightMode selected = select_mode();

    // Apply both sensor configs
    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);

    // Print the applied configuration
    print_mode_configuration(selected);

        // Run unified calibration decision & flow
    run_calibration_sequence_startup();

    // Print the applied configuration
   // print_mode_configuration(selected);

   // Setup FIFO for continuous reading
    bmi323_setup_fifo();


    Serial.println(F("Flight mode configuration applied."));
}

// === Loop ===
void loop() {
    // 1. Check FIFO for new data
    bmi323_read_fifo();

    // 2. Optional: update communication / telemetry
    comManager.update();

    // 3. Add a small delay to avoid starving SPI bus (optional)
    delay(1);
}
