#include <Arduino.h>
#include "bmi323.h"

// === Define global variables here ===
FlightMode current_mode;
const FlightModeConfig* current_config = nullptr;

float estimated_pitch = 0.0f;
float estimated_roll = 0.0f;
float estimated_yaw = 0.0f;  

unsigned long last_update_time = 0;

bmi323_data_t sensor_data;
GyroCalibration gyro_cal;
AccelCalibration accel_cal;



// Setup
// ------------------------
void setup() {
    delay(3000);
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing BMI323...");

    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
        while (1);
    }

    delay(200);

    if (!bmi323_run_selftest()) {
        Serial.println("[WARN] Self-test failed.");
    }

    bmi323_set_axis_remap(0x00);
    delay(200);

    if (user_requested_recalibration()) {
        clear_calibration_flash();
        perform_calibration_sequence();
    } else {
        if (load_calibration_from_flash(gyro_cal, accel_cal)) {
            apply_gyro_calibration(&gyro_cal);
            apply_accel_calibration(&accel_cal);
        } else {
            perform_calibration_sequence();
        }
    }

    print_calibration_info();

    current_mode = select_flight_mode();

    switch (current_mode) {
        case MODE_STABLE: current_config = &stable_config; break;
        case MODE_HOVER:  current_config = &hover_config;  break;
        case MODE_CRUISE: current_config = &cruise_config; break;
    }

    Serial.println("Flight mode configured. Starting control loop...");
    bmi323_setup_fifo();
}

// ------------------------
// Loop
// ------------------------
void loop() {
    bmi323_read_fifo();  // Reads, computes, and prints all logic per frame
    delay(10);           // Optional, reduce if missing frames
}


