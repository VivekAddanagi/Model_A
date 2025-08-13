// calibration_sequence.cpp
#include <Arduino.h>
#include <time.h>
#include <EEPROM.h>
#include <Preferences.h>

#include "bmi323.h"
#include "bmp390.h"

// ===================== Config (can be overridden via Config.h) =====================
#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION   86400UL   // 24 hours (stale threshold)
#endif

#ifndef LONG_EXPIRE_MULTIPLIER
#define LONG_EXPIRE_MULTIPLIER 7UL       // "too long" threshold = 7 * CALIB_VALID_DURATION
#endif

#define LONG_EXPIRE_DURATION   (CALIB_VALID_DURATION * LONG_EXPIRE_MULTIPLIER)

// BMI323 timestamp key (alongside your existing prefs keys)
#ifndef BMI323_PREFS_NS
#define BMI323_PREFS_NS "bmi323"
#endif
#ifndef BMI323_TS_KEY
#define BMI323_TS_KEY   "ts"
#endif

// ===================== Externals from your code =====================
// BMI323 calib state (declared extern in bmi323.h)
extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;

// BMP390: global ground pressure offset is set inside bmp390_* code
extern float pressure_offset;

// Your existing functions (from your codebase)
extern bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal);
extern void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal);
extern void clear_calibration_flash();

extern bool bmi323_quick_gyro_calibrate(GyroCalibration* cal);
extern bool bmi323_z_accel_calibrate(AccelCalibration* cal);
extern void apply_gyro_calibration(const GyroCalibration* cal);
extern void apply_accel_calibration(const AccelCalibration* cal);

extern int  bmp390_calibrate_offset(void);
extern int  bmp390_apply_calibration(void);

// ===================== Helpers =====================

static uint32_t now_secs() {
    uint32_t t = (uint32_t)time(NULL);
    if (t == 0) {
        // Fallback if RTC not set: derive seconds from millis (coarse)
        t = millis() / 1000UL;
        if (t == 0) t = 1;
    }
    return t;
}

// ----- BMI323 timestamp storage in Preferences -----
static uint32_t bmi323_read_timestamp() {
    Preferences prefs;
    prefs.begin(BMI323_PREFS_NS, true);
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

// Erase BMI323 calibration (your clear function) + timestamp
static void bmi323_erase_calibration() {
    clear_calibration_flash();
    Preferences prefs;
    prefs.begin(BMI323_PREFS_NS, false);
    prefs.remove(BMI323_TS_KEY);
    prefs.end();
}

// ----- BMP390 timestamp handling in EEPROM blob -----
static uint32_t bmp390_peek_timestamp() {
    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) return 0;
    BMP390_CalibrationData calib;
    EEPROM.get(CALIB_EEPROM_ADDR, calib);
    EEPROM.end();
    return calib.timestamp;
}

// Invalidate BMP390 EEPROM by zeroing timestamp
static void bmp390_invalidate_eeprom() {
    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) return;
    BMP390_CalibrationData calib;
    EEPROM.get(CALIB_EEPROM_ADDR, calib);
    calib.timestamp = 0;
    EEPROM.put(CALIB_EEPROM_ADDR, calib);
    EEPROM.commit();
    EEPROM.end();
}

// Ask user whether to recalibrate, with a 5-second window.
// Also display ages to help decide.
static bool prompt_user_recalibration(uint32_t bmi_age, uint32_t bmp_age, bool bmi_stale, bool bmp_stale) {
    Serial.println();
    Serial.println(F("=== Calibration Decision ==="));
    Serial.printf("BMI323 age: %lus (%s)\n", (unsigned long)bmi_age, bmi_stale ? "stale" : "ok");
    Serial.printf("BMP390 age: %lus (%s)\n", (unsigned long)bmp_age, bmp_stale ? "stale" : "ok");
    Serial.println(F("Press 'c' to recalibrate, 'u' to use saved, or wait 5s (stale => auto-calibrate)."));

    uint32_t start = millis();
    while (millis() - start < 5000) {
        if (Serial.available()) {
            char ch = Serial.read();
            if (ch == 'c' || ch == 'C') return true;
            if (ch == 'u' || ch == 'U') return false;
        }
        delay(10);
    }

    // Auto-decision: if either is stale, recalibrate; else use saved
    return (bmi_stale || bmp_stale);
}

static void apply_all_if_available() {
    // BMP390 apply: uses EEPROM validity check inside
    if (bmp390_apply_calibration() == 0) {
        Serial.println(F("[BMP390] Applied saved calibration."));
    } else {
        Serial.println(F("[BMP390] No valid saved calibration."));
    }

    // BMI323 apply: load from NVS and apply
    GyroCalibration gtmp;
    AccelCalibration atmp;
    if (load_calibration_from_flash(gtmp, atmp)) {
        apply_gyro_calibration(&gtmp);
        apply_accel_calibration(&atmp);
        gyro_cal = gtmp;
        accel_cal = atmp;
        Serial.println(F("[BMI323] Applied saved calibration."));
    } else {
        Serial.println(F("[BMI323] No saved calibration."));
    }
}

// Perform full calibration for both sensors
static bool recalibrate_all_and_save() {
    Serial.println(F("\n[CAL] Starting calibration for both sensors..."));

    // --- BMI323 ---
    wait_for_user_confirmation(); // your prompt: place flat and press a key

    if (!bmi323_quick_gyro_calibrate(&gyro_cal)) {
        Serial.println(F("[ERROR] BMI323 gyro calibration failed."));
        return false;
    }
    if (!bmi323_z_accel_calibrate(&accel_cal)) {
        Serial.println(F("[ERROR] BMI323 accel-Z calibration failed."));
        return false;
    }
    save_calibration_to_flash(gyro_cal, accel_cal);
    bmi323_write_timestamp(now_secs());  // record timestamp for age tracking
    apply_gyro_calibration(&gyro_cal);
    apply_accel_calibration(&accel_cal);
    Serial.println(F("[BMI323] Calibration completed, saved, and applied."));

    // --- BMP390 ---
    if (bmp390_calibrate_offset() != 0) {
        Serial.println(F("[ERROR] BMP390 pressure calibration failed."));
        return false;
    }
    // bmp390_calibrate_offset() saves EEPROM with timestamp internally and sets pressure_offset
    Serial.println(F("[BMP390] Calibration completed, saved, and applied."));

    return true;
}

// ===================== Public entry point =====================
// Call from setup() after sensor init.
void run_calibration_sequence_startup() {
    // Ages
    uint32_t now = now_secs();

    // BMI323 age
    uint32_t bmi_ts = bmi323_read_timestamp();
    uint32_t bmi_age = (bmi_ts > 0 && now >= bmi_ts) ? (now - bmi_ts) : UINT32_MAX;

    // BMP390 age
    uint32_t bmp_ts = bmp390_peek_timestamp();
    uint32_t bmp_age = (bmp_ts > 0 && now >= bmp_ts) ? (now - bmp_ts) : UINT32_MAX;

    bool bmi_stale = (bmi_age == UINT32_MAX) || (bmi_age > CALIB_VALID_DURATION);
    bool bmp_stale = (bmp_age == UINT32_MAX) || (bmp_age > CALIB_VALID_DURATION);

    bool bmi_too_long = (bmi_age != UINT32_MAX) && (bmi_age > LONG_EXPIRE_DURATION);
    bool bmp_too_long = (bmp_age != UINT32_MAX) && (bmp_age > LONG_EXPIRE_DURATION);

    // If "too long", erase first so we don't accidentally use stale
    if (bmi_too_long) {
        Serial.println(F("[BMI323] Calibration too old -> erasing."));
        bmi323_erase_calibration();
    }
    if (bmp_too_long) {
        Serial.println(F("[BMP390] Calibration too old -> erasing."));
        bmp390_invalidate_eeprom();
    }

    // If both fresh and user chooses to keep, we apply and exit
    bool user_wants_recal = prompt_user_recalibration(bmi_age, bmp_age, bmi_stale, bmp_stale);
    if (!user_wants_recal && !bmi_stale && !bmp_stale) {
        Serial.println(F("[CAL] Using saved calibrations."));
        apply_all_if_available();
        return;
    }

    // Either user requested recalibration or one/both are stale
    if (!recalibrate_all_and_save()) {
        Serial.println(F("[CAL] Recalibration failed. Attempting to apply any saved calibrations..."));
        apply_all_if_available();
        return;
    }

    Serial.println(F("[CAL] Recalibration done."));
}
