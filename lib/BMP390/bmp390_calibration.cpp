// bmp390_calibration.cpp
#include <Preferences.h>
#include <time.h>
#include <algorithm>
#include <vector>
#include <math.h>
#include "bmp390.h"

// ---------- Globals ----------
float pressure_offset = 101325.0f;   // ground pressure at calibration site
float sea_level_offset = 101325.0f;  // sea-level equivalent reference


static float filtered_rel_alt = 0.0f;
static float filtered_abs_alt = 0.0f;
const float ALT_ALPHA = 0.1f;

// ---------- NVS ----------
#define BMP390_PREFS_NS       "bmp390"
#define BMP390_TS_KEY         "ts"
#define BMP390_PRES_KEY       "pres"
#define BMP390_TEMP_KEY       "temp"
#define BMP390_SEALEVEL_KEY   "sealevel"

// If not included elsewhere:
#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION   86400UL
#endif

static uint32_t now_secs_bmp() {
    uint32_t t = (uint32_t)time(NULL);
    if (t == 0) {
        t = millis() / 1000UL;
        if (t == 0) t = 1;
    }
    return t;
}

// ---------- Baro formulas ----------
static inline float sealevel_from_ground(float P_ground, float altitude_m) {
    return P_ground / powf(1.0f - altitude_m / 44330.0f, 5.255f);
}

static inline float altitude_from_p_and_p0(float P, float P0) {
    return 44330.0f * (1.0f - powf(P / P0, 0.1903f));
}

// ---------- Robust average with outlier rejection ----------
static bool take_pressure_temp_average(float& avg_p, float& avg_t) {
    const int N = 150;             // number of attempts
    const int MIN_VALID = 100;     // minimum valid samples required
    std::vector<float> ps; ps.reserve(N);
    std::vector<float> ts; ts.reserve(N);

    float p, t;
    for (int i = 0; i < N; ++i) {
        if (bmp390_force_measurement_both(&p, &t) == 0) {
            ps.push_back(p);
            ts.push_back(t);
        }
        delay(10);
    }
    if ((int)ps.size() < MIN_VALID) return false;

    // Trimmed mean (remove 10% high/low)
    auto trim_mean = [](std::vector<float>& v) {
        std::sort(v.begin(), v.end());
        size_t n = v.size();
        size_t trim = (size_t)(0.10f * n);
        if (2 * trim >= n) trim = 0;
        double sum = 0.0;
        for (size_t i = trim; i < n - trim; ++i) sum += v[i];
        return (float)(sum / (double)(n - 2 * trim));
    };

    avg_p = trim_mean(ps);
    avg_t = trim_mean(ts);
    return true;
}

// ==================== Calibration (Option A): pass sea-level pressure (Pa) ====================
int bmp390_calibrate_offset(float sea_level_pressure_pa) {
    Serial.println("[CALIB] Starting BMP390 calibration...");
    delay(100);

    float avg_p = 0.0f, avg_t = 0.0f;
    if (!take_pressure_temp_average(avg_p, avg_t)) {
        Serial.println("[ERROR] Not enough valid samples for calibration.");
        return -1;
    }

    pressure_offset = avg_p;

    // If valid sea-level pressure passed, use it; otherwise, fallback to measured ground
    sea_level_offset = (sea_level_pressure_pa > 80000.0f && sea_level_pressure_pa < 110000.0f)
                        ? sea_level_pressure_pa
                        : avg_p;

    // Save to NVS
    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, false)) {
        Serial.println("[ERROR] Failed to open NVS for BMP390 calibration.");
        return -1;
    }
    prefs.putUInt(BMP390_TS_KEY, now_secs_bmp());
    prefs.putFloat(BMP390_PRES_KEY, pressure_offset);
    prefs.putFloat(BMP390_SEALEVEL_KEY, sea_level_offset);
    prefs.putFloat(BMP390_TEMP_KEY, avg_t);
    prefs.end();

    Serial.printf("[CALIB] Complete: Ground P=%.2f Pa | SeaLevelRef=%.2f Pa | T=%.2f °C\n",
                  pressure_offset, sea_level_offset, avg_t);
    return 0;
}

// ==================== Calibration (Option B): pass known field altitude (m) ====================
int bmp390_calibrate_with_altitude(float field_altitude_m) {
    Serial.println("[CALIB] Starting BMP390 calibration (with altitude)...");
    delay(100);

    float avg_p = 0.0f, avg_t = 0.0f;
    if (!take_pressure_temp_average(avg_p, avg_t)) {
        Serial.println("[ERROR] Not enough valid samples for calibration.");
        return -1;
    }

    pressure_offset = avg_p;
    sea_level_offset = sealevel_from_ground(avg_p, field_altitude_m);

    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, false)) {
        Serial.println("[ERROR] Failed to open NVS for BMP390 calibration.");
        return -1;
    }
    prefs.putUInt(BMP390_TS_KEY, now_secs_bmp());
    prefs.putFloat(BMP390_PRES_KEY, pressure_offset);
    prefs.putFloat(BMP390_SEALEVEL_KEY, sea_level_offset);
    prefs.putFloat(BMP390_TEMP_KEY, avg_t);
    prefs.end();

    Serial.printf("[CALIB] Complete: Ground P=%.2f Pa | SeaLevelRef=%.2f Pa | T=%.2f °C | Alt=%.2f m\n",
                  pressure_offset, sea_level_offset, avg_t, field_altitude_m);
    return 0;
}

// ==================== Calibration (Option C): Auto sea-level from current ground ====================
int bmp390_calibrate_auto_sealevel() {
    Serial.println("[CALIB] Starting BMP390 auto sea-level calibration...");

    float avg_p = 0.0f, avg_t = 0.0f;
    if (!take_pressure_temp_average(avg_p, avg_t)) {
        Serial.println("[ERROR] Not enough valid samples for calibration.");
        return -1;
    }

    pressure_offset = avg_p;

    // Assume ground altitude = 0 m for takeoff
    float ground_alt_m = 0.0f;
    sea_level_offset = sealevel_from_ground(pressure_offset, ground_alt_m);

    Preferences prefs;
    if (!prefs.begin(BMP390_PREFS_NS, false)) {
        Serial.println("[ERROR] Failed to open NVS for BMP390 calibration.");
        return -1;
    }
    prefs.putUInt(BMP390_TS_KEY, now_secs_bmp());
    prefs.putFloat(BMP390_PRES_KEY, pressure_offset);
    prefs.putFloat(BMP390_SEALEVEL_KEY, sea_level_offset);
    prefs.putFloat(BMP390_TEMP_KEY, avg_t);
    prefs.end();

    Serial.printf("[CALIB] Complete: Ground P=%.2f Pa | SeaLevelRef=%.2f Pa | T=%.2f °C\n",
                  pressure_offset, sea_level_offset, avg_t);
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
    float saved_p  = prefs.getFloat(BMP390_PRES_KEY, 0.0f);
    float saved_sl = prefs.getFloat(BMP390_SEALEVEL_KEY, 0.0f);
    float saved_t  = prefs.getFloat(BMP390_TEMP_KEY, 0.0f);
    prefs.end();

    uint32_t now = now_secs_bmp();
    if (now == 0 || (now - timestamp > CALIB_VALID_DURATION)) {
        Serial.println("[CALIB] Calibration data is invalid or expired.");
        return -1;
    }

    if (saved_p < 80000.0f || saved_p > 110000.0f) {
        Serial.println("[CALIB] Saved ground pressure out of range.");
        return -1;
    }
    if (saved_sl < 80000.0f || saved_sl > 110000.0f) {
        Serial.println("[CALIB] Saved sea-level pressure out of range.");
        return -1;
    }

    pressure_offset  = saved_p;
    sea_level_offset = saved_sl;

    Serial.printf("[CALIB] Loaded from NVS: GroundP=%.2f Pa | SeaLevelRef=%.2f Pa | T=%.2f °C\n",
                  saved_p, saved_sl, saved_t);
    return 0;
}

// ==================== Getters / Altitude ====================
float bmp390_get_ground_pressure() { return pressure_offset; }

float bmp390_get_relative_altitude(float pressure_now) {
    float raw_alt = altitude_from_p_and_p0(pressure_now, pressure_offset);
    filtered_rel_alt = ALT_ALPHA * raw_alt + (1.0f - ALT_ALPHA) * filtered_rel_alt;
    return filtered_rel_alt;
}

float bmp390_get_absolute_altitude(float pressure_now) {
    float raw_alt = altitude_from_p_and_p0(pressure_now, sea_level_offset);
    filtered_abs_alt = ALT_ALPHA * raw_alt + (1.0f - ALT_ALPHA) * filtered_abs_alt;
    return filtered_abs_alt;
}

// Convenience functions
float bmp390_altitude_from_ground(float pressure, float ground_pressure) {
    return altitude_from_p_and_p0(pressure, ground_pressure);
}

float bmp390_calculate_altitude(float pressure_pa) {
    const float P0 = 101325.0f;
    return altitude_from_p_and_p0(pressure_pa, P0);
}
