#include <EEPROM.h>
#include <time.h>
#include "bmp390.h"

// Global variable to hold pressure offset loaded from EEPROM or calibration
float pressure_offset = 0.0f;

/**
 * Calibrates the pressure sensor by taking multiple readings and storing the average
 * pressure and temperature in EEPROM with a timestamp.
 */
int bmp390_calibrate_offset() {
    Serial.println("[CALIB] Starting zero-offset calibration...");
    delay(1000);  // Let sensor settle after power-up or reset

    float sum_p = 0.0f, sum_t = 0.0f;
    uint8_t valid_samples = 0;
    float p, t;

    for (int i = 0; i < 40; i++) {
        if (bmp390_force_measurement_both(&p, &t) == 0) {
            sum_p += p;
            sum_t += t;
            valid_samples++;
        } else {
            Serial.printf("[CALIB] Sample %02d: [Invalid]\n", i);
        }
        delay(25);  // Delay between readings
    }

    if (valid_samples < 30) {
        Serial.println("[ERROR] Not enough valid samples for calibration.");
        return -1;
    }

    float avg_p = sum_p / valid_samples;
    float avg_t = sum_t / valid_samples;

    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) {
        Serial.println("[ERROR] EEPROM.begin() failed.");
        return -1;
    }

    uint32_t timestamp = (uint32_t)time(NULL);
    if (timestamp == 0) timestamp = 1;  // Fallback if RTC not set

    BMP390_CalibrationData calib = {
        .pressure_offset = avg_p,
        .calibration_temperature = avg_t,
        .timestamp = timestamp
    };

    EEPROM.put(CALIB_EEPROM_ADDR, calib);
    if (!EEPROM.commit()) {
        Serial.println("[ERROR] EEPROM commit failed.");
        EEPROM.end();
        return -1;
    }

    EEPROM.end();

    pressure_offset = avg_p;
    Serial.printf("[CALIB] Calibration complete: P=%.2f Pa | T=%.2f °C\n", avg_p, avg_t);
    return 0;
}

/**
 * Loads pressure calibration data from EEPROM and checks if it is still valid.
 * If valid, the pressure offset is applied.
 */
int bmp390_apply_calibration() {
    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) {
        Serial.println("[ERROR] EEPROM.begin() failed.");
        return -1;
    }

    BMP390_CalibrationData calib;
    EEPROM.get(CALIB_EEPROM_ADDR, calib);
    EEPROM.end();

    uint32_t now = (uint32_t)time(NULL);
    if (now == 0 || (now - calib.timestamp > CALIB_VALID_DURATION)) {
        Serial.println("[CALIB] Calibration data is invalid or expired.");
        return -1;
    }

    pressure_offset = calib.pressure_offset;
    Serial.printf("[CALIB] Loaded from EEPROM: P=%.2f Pa | T=%.2f °C\n",
                  calib.pressure_offset, calib.calibration_temperature);
    return 0;
}

/**
 * Returns the pressure offset from calibration. This is used to compute relative altitude.
 */
float bmp390_get_ground_pressure() {
    return pressure_offset;
}
