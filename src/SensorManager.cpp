#include "SensorManager.h"

SensorManager::SensorManager() {}

bool SensorManager::begin() {
    Serial.println("[SENSOR] Initializing BMI323...");

    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed!");
        return false;
    }

    // Load or perform calibration
    if (!load_calibration_from_flash(gyro_cal, accel_cal) || user_requested_recalibration()) {
        Serial.println("[CALIB] Calibration data missing or user requested recalibration.");
        calibrateBMI323(true);
    } else {
        apply_gyro_calibration(&gyro_cal);
        apply_accel_calibration(&accel_cal);
    }

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
