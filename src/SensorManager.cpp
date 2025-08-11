#include "SensorManager.h"
#include "Config.h"
#include "bmi323.h"


SensorManager::SensorManager() {}

bool SensorManager::begin() {
    Serial.println("[SENSOR] Initializing BMI323...");
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
    return true;
}

void SensorManager::update() {
    bmi323_read_fifo();
    // Could add BMP390, CC2500 polling here later
}

void SensorManager::calibrateBMI323(bool force) {
    if (force) {
        perform_calibration_sequence();
    }
}

void SensorManager::printBMI323Data() {
    Serial.printf("[BMI323] Pitch: %.2f | Roll: %.2f | Yaw: %.2f\n",
                  estimated_pitch, estimated_roll, estimated_yaw);
}


FlightMode SensorManager::select_flight_mode() {
    Serial.println("Select flight mode:");
    Serial.println("1 - Stable");
    Serial.println("2 - Hover");
    Serial.println("3 - Cruise");
    
    while(!Serial.available());
    char input = Serial.read();
    
    switch(input) {
        case '1': return MODE_STABLE;
        case '2': return MODE_HOVER;
        case '3': return MODE_CRUISE;
        default: return MODE_STABLE;
    }
}

void SensorManager::validate_config() {
    if(current_config == nullptr) {
        Serial.println("[ERROR] No flight mode configured!");
        current_config = &stable_config; // Fallback to stable mode
    }
}