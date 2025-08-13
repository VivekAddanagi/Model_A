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

// ===================== Preferences Keys =====================
// BMI323
#define BMI323_PREFS_NS "bmi323"
#define BMI323_TS_KEY   "ts"
#define BMI323_GYRO_KEY "gyro"
#define BMI323_ACCEL_KEY "accel"

// BMP390
#define BMP390_PREFS_NS "bmp390"
#define BMP390_TS_KEY   "ts"
#define BMP390_OFFSET_KEY "offset"

// ===================== Externals from your code =====================
extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;
extern float pressure_offset;

extern bool bmi323_quick_gyro_calibrate(GyroCalibration* cal);
extern bool bmi323_accel_calibrate_all(AccelCalibration* cal) ;
extern void apply_gyro_calibration(const GyroCalibration* cal);
extern void apply_accel_calibration(const AccelCalibration* cal);

extern bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal);
extern void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal);
extern void clear_calibration_flash();

extern int bmp390_calibrate_offset(void);
extern int bmp390_apply_calibration(void);

// ===================== Helpers =====================
static uint32_t now_secs() {
    uint32_t t = (uint32_t)time(NULL);
    if (t == 0) {
        t = millis() / 1000UL;
        if (t == 0) t = 1;
    }
    return t;
}

// ----- BMI323 -----
static uint32_t bmi323_read_timestamp() {
    Preferences prefs;
    if (!prefs.begin(BMI323_PREFS_NS, true)) {
        return 0; // No calibration stored yet
    }
    uint32_t ts = prefs.getUInt(BMI323_TS_KEY, 0);
    prefs.end();
    return ts;
}

static void bmi323_write_timestamp(uint32_t ts) {
    Preferences prefs;
    prefs.begin(BMI323_PREFS_NS, false);
    prefs.putUInt(BMI323_TS_KEY, ts);
    prefs.end();
}
static void bmi323_erase_calibration() {
    clear_calibration_flash();
    Preferences prefs;
    prefs.begin(BMI323_PREFS_NS, false);
    prefs.remove(BMI323_TS_KEY);
    prefs.remove(BMI323_GYRO_KEY);
    prefs.remove(BMI323_ACCEL_KEY);
    prefs.end();
}

// ----- BMP390 -----
static uint32_t bmp390_read_timestamp() {
    Preferences prefs;
    prefs.begin(BMP390_PREFS_NS, true);
    uint32_t ts = prefs.getUInt(BMP390_TS_KEY, 0);
    prefs.end();
    return ts;
}
static void bmp390_write_timestamp(uint32_t ts) {
    Preferences prefs;
    prefs.begin(BMP390_PREFS_NS, false);
    prefs.putUInt(BMP390_TS_KEY, ts);
    prefs.end();
}
static void bmp390_save_calibration(float offset) {
    Preferences prefs;
    prefs.begin(BMP390_PREFS_NS, false);
    prefs.putFloat(BMP390_OFFSET_KEY, offset);
    prefs.putUInt(BMP390_TS_KEY, now_secs());
    prefs.end();
}
static bool bmp390_load_calibration(float &offset) {
    Preferences prefs;
    prefs.begin(BMP390_PREFS_NS, true);
    if (!prefs.isKey(BMP390_OFFSET_KEY)) {
        prefs.end();
        return false;
    }
    offset = prefs.getFloat(BMP390_OFFSET_KEY, 0.0f);
    prefs.end();
    return true;
}
static void bmp390_erase_calibration() {
    Preferences prefs;
    prefs.begin(BMP390_PREFS_NS, false);
    prefs.remove(BMP390_TS_KEY);
    prefs.remove(BMP390_OFFSET_KEY);
    prefs.end();
}

// ----- Print calibration values -----
static void print_calibration_data() {
    Serial.println(F("=== Current Calibration Data ==="));

    Serial.printf("BMI323 Gyro Bias: X=%.6f Y=%.6f Z=%.6f\n",
                  gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
   Serial.printf("Accel bias (x, y, z): %.4f, %.4f, %.4f m/s^2\n",
              accel_cal.bias_x,
              accel_cal.bias_y,
              accel_cal.bias_z);


    Serial.printf("BMP390 Pressure Offset: %.3f Pa\n", pressure_offset);
    Serial.println(F("================================"));
}

// ===================== Main Calibration Flow =====================
void run_calibration_sequence_startup() {
    uint32_t now = now_secs();

    // Always calibrate BMI323
    Serial.println(F("[BMI323] Always calibrating at startup..."));
    wait_for_user_confirmation();
    if (bmi323_quick_gyro_calibrate(&gyro_cal))
        Serial.println(F("[BMI323] Gyro calibrated."));
   if (bmi323_accel_calibrate_all(&accel_cal))
    Serial.println(F("[BMI323] Accelerometer (XYZ) calibrated."));
else
    Serial.println(F("[ERROR] Accelerometer calibration failed."));

    save_calibration_to_flash(gyro_cal, accel_cal);
    bmi323_write_timestamp(now);
    apply_gyro_calibration(&gyro_cal);
    apply_accel_calibration(&accel_cal);

    // BMP390 — check if stale
    uint32_t bmp_ts = bmp390_read_timestamp();
    uint32_t bmp_age = (bmp_ts > 0 && now >= bmp_ts) ? (now - bmp_ts) : UINT32_MAX;
    bool bmp_stale = (bmp_age == UINT32_MAX) || (bmp_age > CALIB_VALID_DURATION);
    bool bmp_too_long = (bmp_age != UINT32_MAX) && (bmp_age > LONG_EXPIRE_DURATION);

    if (bmp_too_long) {
        Serial.println(F("[BMP390] Calibration too old -> erasing."));
        bmp390_erase_calibration();
        bmp_stale = true;
    }

    if (bmp_stale) {
        Serial.println(F("[BMP390] Calibrating..."));
        if (bmp390_calibrate_offset() == 0) {
            bmp390_save_calibration(pressure_offset);
            Serial.println(F("[BMP390] Calibration completed."));
        } else {
            Serial.println(F("[ERROR] BMP390 calibration failed — trying saved data..."));
            bmp390_load_calibration(pressure_offset);
        }
    } else {
        if (bmp390_load_calibration(pressure_offset)) {
            Serial.println(F("[BMP390] Using saved calibration."));
        } else {
            Serial.println(F("[BMP390] No saved calibration, calibrating now..."));
            bmp390_calibrate_offset();
            bmp390_save_calibration(pressure_offset);
        }
    }

    // Print final calibration data
    print_calibration_data();

    Serial.println(F("[CAL] Startup calibration sequence completed."));
}
