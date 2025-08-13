// calibration_sequence.cpp
#include <Arduino.h>
#include <time.h>
#include <Preferences.h>

#include "bmi323.h"
#include "bmp390.h"

// ===================== Config =====================
#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION   86400UL   // 24 hours
#endif

#ifndef LONG_EXPIRE_MULTIPLIER
#define LONG_EXPIRE_MULTIPLIER 7UL       // 7 days
#endif

#define LONG_EXPIRE_DURATION   (CALIB_VALID_DURATION * LONG_EXPIRE_MULTIPLIER)

// ===================== External variables from other files =====================
extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;
extern float pressure_offset;

extern bool bmi323_quick_gyro_calibrate(GyroCalibration* cal);
extern bool bmi323_accel_calibrate_all(AccelCalibration* cal);
extern void apply_gyro_calibration(const GyroCalibration* cal);
extern void apply_accel_calibration(const AccelCalibration* cal);
extern int bmp390_calibrate_offset(void);
extern int bmp390_apply_calibration(void);

extern void wait_for_user_confirmation();

// ===================== Helpers =====================
static uint32_t now_secs() {
    uint32_t t = (uint32_t)time(NULL);
    if (t == 0) {
        t = millis() / 1000UL;
        if (t == 0) t = 1;
    }
    return t;
}

// ===================== Preferences Keys (Unified) =====================
#define CALIB_PREFS_NS      "calib"
#define CALIB_TS_KEY        "ts"
#define CALIB_GYRO_KEY      "gyro"
#define CALIB_ACCEL_KEY     "accel"
#define CALIB_PRESSURE_KEY  "pressure"

// ===================== Unified Save/Load =====================
static bool save_all_calibration(const GyroCalibration& gyro,
                                 const AccelCalibration& accel,
                                 float pressure) {
    Preferences prefs;
    if (!prefs.begin(CALIB_PREFS_NS, false)) return false;

    prefs.putUInt(CALIB_TS_KEY, now_secs());
    prefs.putBytes(CALIB_GYRO_KEY, &gyro, sizeof(gyro));
    prefs.putBytes(CALIB_ACCEL_KEY, &accel, sizeof(accel));
    prefs.putFloat(CALIB_PRESSURE_KEY, pressure);

    prefs.end();
    return true;
}

static bool load_all_calibration(GyroCalibration& gyro,
                                 AccelCalibration& accel,
                                 float& pressure) {
    Preferences prefs;
    if (!prefs.begin(CALIB_PREFS_NS, true)) return false;
    if (!prefs.isKey(CALIB_TS_KEY)) { prefs.end(); return false; }

    prefs.getBytes(CALIB_GYRO_KEY, &gyro, sizeof(gyro));
    prefs.getBytes(CALIB_ACCEL_KEY, &accel, sizeof(accel));
    pressure = prefs.getFloat(CALIB_PRESSURE_KEY, 0.0f);

    prefs.end();
    return true;
}

static uint32_t read_calibration_timestamp() {
    Preferences prefs;
    prefs.begin(CALIB_PREFS_NS, true);
    uint32_t ts = prefs.getUInt(CALIB_TS_KEY, 0);
    prefs.end();
    return ts;
}

static void erase_all_calibration() {
    Preferences prefs;
    prefs.begin(CALIB_PREFS_NS, false);
    prefs.clear(); // Wipes all keys in this namespace
    prefs.end();
}

// ===================== Print calibration values =====================
static void print_calibration_data() {
    Serial.println(F("=== Current Calibration Data ==="));
    Serial.printf("BMI323 Gyro Bias: X=%.6f Y=%.6f Z=%.6f\n",
                  gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    Serial.printf("Accel bias (x, y, z): %.4f, %.4f, %.4f m/s^2\n",
                  accel_cal.bias_x, accel_cal.bias_y, accel_cal.bias_z);
    Serial.printf("BMP390 Pressure Offset: %.3f Pa\n", pressure_offset);
    Serial.println(F("================================"));
}

// ===================== Main Calibration Flow =====================
void run_calibration_sequence_startup() {
    Serial.println(F("[CALIB] Performing fresh calibration before flight..."));
    wait_for_user_confirmation();

    // Gyro calibration
    if (bmi323_quick_gyro_calibrate(&gyro_cal))
        Serial.println(F("[BMI323] Gyro calibrated."));
    else
        Serial.println(F("[ERROR] Gyro calibration failed."));

    // Accel calibration
    if (bmi323_accel_calibrate_all(&accel_cal))
        Serial.println(F("[BMI323] Accelerometer (XYZ) calibrated."));
    else
        Serial.println(F("[ERROR] Accelerometer calibration failed."));

    // BMP390 calibration
    if (bmp390_calibrate_offset() == 0)
        Serial.println(F("[BMP390] Pressure sensor calibrated."));
    else
        Serial.println(F("[ERROR] BMP390 calibration failed."));

    // Save calibration (optional – so we can view later if needed)
    save_all_calibration(gyro_cal, accel_cal, pressure_offset);

    // Apply calibration
    apply_gyro_calibration(&gyro_cal);
    apply_accel_calibration(&accel_cal);
    bmp390_apply_calibration();

    // Show results
    print_calibration_data();
    Serial.println(F("[CAL] Calibration sequence completed."));
}

