#include "SensorManager.h"
#include "Config.h"
#include <EEPROM.h>

// -------------------- Constructor --------------------
SensorManager::SensorManager() {}

// -------------------- Begin / init --------------------
bool SensorManager::begin() {
    Serial.println("[SENSOR] Initializing BMI323...");
    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
        while (1);
    }

    delay(200);

    if (!bmi323_run_selftest()) {
        Serial.println("[WARN] BMI323 self-test failed.");
    }

    bmi323_set_axis_remap(0x00);
    delay(200);

    bmi323_setup_fifo();

    return true;
}

// -------------------- Update (called frequently) --------------------
void SensorManager::update() {
    // --- BMI323 update ---
    bmi323_read_fifo();

    // --- Optional: BMP390 FIFO processing ---
    process_bmp390_fifo();
}

// -------------------- Public getters --------------------
void SensorManager::printBMI323Data() {
    Serial.printf("[BMI323] Pitch: %.2f | Roll: %.2f | Yaw: %.2f\n",
                  estimated_pitch, estimated_roll, estimated_yaw);
}

// -------------------- Existing helpers preserved --------------------
void SensorManager::validate_config() {
    if(current_config == nullptr) {
        Serial.println("[ERROR] No flight mode configured!");
        current_config = &stable_config; // Fallback to stable mode
    }
}

// -------------------- BMP390 FIFO processing --------------------
void SensorManager::process_bmp390_fifo() {
    static unsigned long last_read_ms = 0;
    const unsigned long READ_INTERVAL_MS = 10; // ~100 Hz

    bool do_read = fifo_data_ready || (millis() - last_read_ms >= READ_INTERVAL_MS);
    if (!do_read) return;

    last_read_ms = millis();
    fifo_data_ready = false;

    // Check FIFO overflow
    if (bmp390_check_fifo_overflow() > 0) {
        Serial.println("[WARN] BMP390 FIFO Overflow detected");
    }

    int ret = bmp390_read_fifo_data(fifo_data, FIFO_BUFFER_SIZE, &frames_available);
    if (ret != 0) {
        Serial.println("[ERROR] BMP390 FIFO read failed");
        return;
    }

    if (frames_available == 0) return;

    for (int i = 0; i < frames_available; ++i) {
        if (!fifo_data[i].pressure_valid || !fifo_data[i].temperature_valid) continue;

        float pressure = fifo_data[i].pressure;
        float temperature = fifo_data[i].temperature;
        float alt_ground = bmp390_altitude_from_ground(pressure, bmp390_get_ground_pressure());
        float alt_sea   = bmp390_calculate_altitude(pressure);

        // Use millis() as software timestamp per frame
        unsigned long frame_time = millis();

        Serial.printf(
            "TEMP: %.2f °C | P: %.2f Pa | AltG: %.2f m | AltS: %.2f m | Time: %lu\n",
            temperature, pressure, alt_ground, alt_sea, frame_time
        );
    }
}

// EOF
