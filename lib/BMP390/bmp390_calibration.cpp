#include <Preferences.h>
#include <time.h>
#include "bmp390.h"

// Global calibration variables
 float pressure_offset = 101325.0f;   // Default sea-level pressure
 float sea_level_offset = 101325.0f;  // Calibration reference for altitude

// NVS namespace and keys
#define BMP390_PREFS_NS       "bmp390"
#define BMP390_TS_KEY         "ts"
#define BMP390_PRES_KEY       "pres"
#define BMP390_TEMP_KEY       "temp"
#define BMP390_SEALEVEL_KEY "bmp390_sealevel"  // <-- NEW: sea-level reference

// ==================== Calibration ====================
// ==================== Globals ====================
static float filtered_rel_alt = 0.0f;
static float filtered_abs_alt = 0.0f;
const float ALT_ALPHA = 0.1f; // smoothing factor (0.05–0.2 recommended)

// ==================== Calibration ====================
int bmp390_calibrate_offset(float sea_level_pressure) {
    Serial.println("[CALIB] Starting BMP390 calibration...");
    delay(1000);

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
        delay(25);
    }

    if (valid_samples < 30) {
        Serial.println("[ERROR] Not enough valid samples for calibration.");
        return -1;
    }

    float avg_p = sum_p / valid_samples;
    float avg_t = sum_t / valid_samples;

    // Save offsets
    sea_level_offset = sea_level_pressure;  // <-- FIXED (now valid)
    pressure_offset  = avg_p;

    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, false)) {
        Serial.println("[ERROR] Failed to open NVS for BMP390 calibration.");
        return -1;
    }
    prefs.putFloat(BMP390_PRES_KEY, pressure_offset);
    prefs.putFloat(BMP390_SEALEVEL_KEY, sea_level_offset);
    prefs.putFloat(BMP390_TEMP_KEY, avg_t);
    prefs.end();

    Serial.printf("[CALIB] Complete: Ground P=%.2f Pa | SeaLevelRef=%.2f Pa | T=%.2f °C\n",
                  pressure_offset, sea_level_offset, avg_t);
    return 0;
}


// ==================== Getters ====================
float bmp390_get_ground_pressure() {
    return pressure_offset;
}

float bmp390_get_relative_altitude(float pressure_now) {
    float raw_alt = 44330.0f * (1.0f - pow(pressure_now / pressure_offset, 0.1903f));
    filtered_rel_alt = ALT_ALPHA * raw_alt + (1.0f - ALT_ALPHA) * filtered_rel_alt;
    return filtered_rel_alt;
}

float bmp390_get_absolute_altitude(float pressure_now) {
    // Uses calibrated sea-level reference
    float raw_alt = 44330.0f * (1.0f - pow(pressure_now / sea_level_offset, 0.1903f));
    return raw_alt;
}


float bmp390_altitude_from_ground(float pressure, float ground_pressure) {
    return 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));
}

float bmp390_calculate_altitude(float pressure_pa) {
    float reference_pressure = 101325.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / reference_pressure, 1.0f / 5.255f));
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

