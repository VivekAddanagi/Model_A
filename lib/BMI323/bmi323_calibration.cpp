#include "bmi323.h"
#include <math.h>
#include <Arduino.h>
#include <Preferences.h>


static Preferences prefs;
#define GYRO_SAMPLES 200
#define ACCEL_SAMPLES 100

void wait_for_user_confirmation() {
    Serial.println("Place the drone level and press any key in Serial Monitor...");
    while (!Serial.available()) {
        delay(10);
    }
    while (Serial.available()) Serial.read();  // Clear buffer
}

bool bmi323_quick_gyro_calibrate(GyroCalibration* cal) {
    // 1. Ensure sensor is stable
    float temp_x = 0, temp_y = 0, temp_z = 0;

       // --- Ignore first 5 readings ---
    for (int i = 0; i < 5; i++) {
        bmi323_data_t dummy;
        bmi323_read(&dummy);
        delay(10);
    }
    
    // 2. Collect samples
    for(int i=0; i<GYRO_SAMPLES; i++) {
        bmi323_data_t data;
        if(!bmi323_read(&data)) return false;
        
        temp_x += data.gx;
        temp_y += data.gy;
        temp_z += data.gz;
        delay(10); // 100Hz sampling
    }

    // 3. Calculate biases
    cal->bias_x = temp_x / GYRO_SAMPLES;
    cal->bias_y = temp_y / GYRO_SAMPLES;
    cal->bias_z = temp_z / GYRO_SAMPLES;

    return true;
}

bool bmi323_z_accel_calibrate(AccelCalibration* cal) {
    // 1. Assume level surface (Z-axis should be +1g)
    float temp_z = 0;

    // --- Ignore first 5 readings ---
    for (int i = 0; i < 5; i++) {
        bmi323_data_t dummy;
        bmi323_read(&dummy);
        delay(20);
    }
    
    // 2. Collect samples
    for(int i=0; i<ACCEL_SAMPLES; i++) {
        bmi323_data_t data;
        if(!bmi323_read(&data)) return false;
        
        temp_z += data.az / 4096.0f; // Convert to g
        delay(20);
    }

    // 3. Calculate offset from ideal 1g
    cal->z_offset = (temp_z / ACCEL_SAMPLES) - 1.0f;
    return true;
}

void apply_gyro_calibration(const GyroCalibration* cal) {
    int16_t ox = (int16_t)(cal->bias_x * 16.384f);
    int16_t oy = (int16_t)(cal->bias_y * 16.384f);
    int16_t oz = (int16_t)(cal->bias_z * 16.384f);

    bmi323_writeRegister(0x66, ox); // GYR_DP_OFF_X
    bmi323_writeRegister(0x68, oy); // GYR_DP_OFF_Y
    bmi323_writeRegister(0x6A, oz); // GYR_DP_OFF_Z
}

void apply_accel_calibration(const AccelCalibration* cal) {
    int16_t oz = (int16_t)(cal->z_offset / 0.00003052f); // offset in LSBs
    bmi323_writeRegister(0x64, oz); // ACC_DP_OFF_Z
}

bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal) {
    prefs.begin("bmi323", true);  // Read-only
    if (!prefs.isKey("gyro_x")) {
        prefs.end();
        Serial.println("[FLASH] No calibration found.");
        return false;
    }

    gyro_cal.bias_x = prefs.getFloat("gyro_x");
    gyro_cal.bias_y = prefs.getFloat("gyro_y");
    gyro_cal.bias_z = prefs.getFloat("gyro_z");
    accel_cal.z_offset = prefs.getFloat("accel_z");

    prefs.end();

    Serial.println("[FLASH] Calibration loaded from NVS:");
    Serial.printf("  Gyro Bias: X=%.2f Y=%.2f Z=%.2f\n", gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    Serial.printf("  Accel Z Offset: %.3f\n", accel_cal.z_offset);
    return true;
}

void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal) {
    prefs.begin("bmi323", false);  // Read-write
    prefs.putFloat("gyro_x", gyro_cal.bias_x);
    prefs.putFloat("gyro_y", gyro_cal.bias_y);
    prefs.putFloat("gyro_z", gyro_cal.bias_z);
    prefs.putFloat("accel_z", accel_cal.z_offset);
    prefs.end();
    Serial.println("[FLASH] Calibration saved to NVS.");
}

void clear_calibration_flash() {
    prefs.begin("bmi323", false);
    prefs.clear();
    prefs.end();
    Serial.println("[FLASH] Calibration erased from NVS.");
}

bool user_requested_recalibration() {
    Serial.println("Press 'c' to force calibration (within 5 seconds)...");

    unsigned long start = millis();
    while (millis() - start < 5000) {
        if (Serial.available()) {
            char ch = Serial.read();
            if (ch == 'c' || ch == 'C') return true;
        }
        delay(10);
    }
    return false;
}

void perform_calibration_sequence() {
    wait_for_user_confirmation();  // Ask user to place flat

    if (!bmi323_quick_gyro_calibrate(&gyro_cal)) {
        Serial.println("[ERROR] Gyro calibration failed.");
        while (1);
    }

    if (!bmi323_z_accel_calibrate(&accel_cal)) {
        Serial.println("[ERROR] Accel Z calibration failed.");
        while (1);
    }

    save_calibration_to_flash(gyro_cal, accel_cal);
    apply_gyro_calibration(&gyro_cal);
    apply_accel_calibration(&accel_cal);

    Serial.println("[CAL] Calibration completed and saved.");
}

void print_calibration_info() {
    Serial.println("Calibration Results:");
    Serial.printf("  Gyro Bias: X=%.2f Y=%.2f Z=%.2f\n",
                  gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    Serial.printf("  Accel Z Offset: %.3f\n", accel_cal.z_offset);
}
