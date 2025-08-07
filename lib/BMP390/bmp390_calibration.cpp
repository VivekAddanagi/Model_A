#include <EEPROM.h>
#include <time.h>
#include "bmp390.h"


static float pressure_offset = 0.0f;

int bmp390_calibrate_offset() {
    Serial.println("[CALIB] Starting zero-offset calibration...");
    delay(1000);  // Allow sensor to settle

    float sum_p = 0.0, sum_t = 0.0;
    uint8_t valid = 0;
    float p, t;

    for (int i = 0; i < 40; i++) {
        if (bmp390_force_measurement_both(&p, &t) == 0) {
            sum_p += p;
            sum_t += t;
            valid++;
            
        } else {
            Serial.printf("[CALIB] Sample %02d: [Invalid]\n", i);
        }
        delay(25);  // time between samples
    }

    if (valid < 30) {
        Serial.println("[ERROR] Not enough valid samples.");
        return -1;
    }

    float avg_p = sum_p / valid;
    float avg_t = sum_t / valid;

    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) return -1;

    uint32_t now = (uint32_t)time(NULL);
    if (now == 0) now = 1;  // Fallback if RTC not set

    BMP390_CalibrationData calib = { avg_p, avg_t, now };
    EEPROM.put(CALIB_EEPROM_ADDR, calib);
    if (!EEPROM.commit()) {
        Serial.println("[ERROR] EEPROM commit failed.");
        EEPROM.end();
        return -1;
    }

    EEPROM.end();
    pressure_offset = avg_p;
    Serial.printf("[CALIB] Saved: P=%.2f Pa | T=%.2f °C\n", avg_p, avg_t);
    return 0;
}

int bmp390_apply_calibration() {
    if (!EEPROM.begin(BMP390_EEPROM_SIZE)) return -1;
    BMP390_CalibrationData calib;
    EEPROM.get(CALIB_EEPROM_ADDR, calib);
    EEPROM.end();

    uint32_t now = (uint32_t)time(NULL);
    if (now == 0 || (now - calib.timestamp > CALIB_VALID_DURATION)) {
        Serial.println("[CALIB] Calibration data invalid or expired.");
        return -1;
    }

    pressure_offset = calib.pressure_offset;
    Serial.printf("[CALIB] Loaded: P=%.2f Pa | T=%.2f °C\n", calib.pressure_offset, calib.calibration_temperature);
    return 0;
}

float bmp390_get_ground_pressure() {
    return pressure_offset;
}
