#include <Preferences.h>
#include <time.h>
#include "bmp390.h"

// Global variable to hold pressure offset loaded from NVS or calibration
float pressure_offset = 0.0f;

// NVS namespace and keys
#define BMP390_PREFS_NS       "bmp390"
#define BMP390_TS_KEY         "ts"
#define BMP390_PRES_KEY       "pres"
#define BMP390_TEMP_KEY       "temp"

// ==================== Calibration ====================
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

    uint32_t timestamp = (uint32_t)time(NULL);
    if (timestamp == 0) timestamp = 1;  // Fallback if RTC not set

    // Save to NVS
    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, false)) {
        Serial.println("[ERROR] Failed to open NVS for BMP390 calibration.");
        return -1;
    }

    prefs.putUInt(BMP390_TS_KEY, timestamp);
    prefs.putFloat(BMP390_PRES_KEY, avg_p);
    prefs.putFloat(BMP390_TEMP_KEY, avg_t);
    prefs.end();

    pressure_offset = avg_p;
    Serial.printf("[CALIB] Calibration complete: P=%.2f Pa | T=%.2f °C\n", avg_p, avg_t);
    return 0;
}

// ==================== Apply Saved Calibration ====================
int bmp390_apply_calibration() {
    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, true)) {
        Serial.println("[ERROR] Failed to open NVS for reading BMP390 calibration.");
        return -1;
    }

    if (!prefs.isKey(BMP390_TS_KEY)) {
        Serial.println("[CALIB] No saved BMP390 calibration found.");
        prefs.end();
        return -1;
    }

    uint32_t timestamp = prefs.getUInt(BMP390_TS_KEY, 0);
    float saved_p = prefs.getFloat(BMP390_PRES_KEY, 0.0f);
    float saved_t = prefs.getFloat(BMP390_TEMP_KEY, 0.0f);
    prefs.end();

    uint32_t now = (uint32_t)time(NULL);
    if (now == 0 || (now - timestamp > CALIB_VALID_DURATION)) {
        Serial.println("[CALIB] Calibration data is invalid or expired.");
        return -1;
    }

    pressure_offset = saved_p;
    Serial.printf("[CALIB] Loaded from NVS: P=%.2f Pa | T=%.2f °C\n", saved_p, saved_t);
    return 0;
}

// ==================== Getter ====================
float bmp390_get_ground_pressure() {
    return pressure_offset;
}
