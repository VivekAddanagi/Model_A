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
        Serial.println("[WARN] Self-test failed.");
    }

    bmi323_set_axis_remap(0x00);
    delay(200);

    if (user_requested_recalibration()) {
        clear_calibration_flash();
        perform_calibration_sequence();
    } else {
        if (load_calibration_from_flash(gyro_cal, accel_cal)) {
            apply_gyro_calibration(&gyro_cal);
            apply_accel_calibration(&accel_cal);
        } else {
            perform_calibration_sequence();
        }
    }

    print_calibration_info();

    current_mode = select_flight_mode();

    switch (current_mode) {
        case MODE_STABLE: current_config = &stable_config; break;
        case MODE_HOVER:  current_config = &hover_config;  break;
        case MODE_CRUISE: current_config = &cruise_config; break;
    }

    Serial.println("Flight mode configured. Starting control loop...");
    bmi323_setup_fifo();
   

    // ---------------- BMP390 Initialization & continuous FIFO ----------------
    Serial.println("[SENSOR] Initializing BMP390...");
    if (bmp390_init_all() != 0) {
        Serial.println("[WARN] BMP390 init failed (sensor not found or error). BMP390 disabled.");
        bmp_present = false;
        return true; // don't fail the whole system — keep IMU working
    }

    bmp_present = true;

    // Try to apply stored calibration. If it fails, keep operating but log.
    if (bmp390_apply_calibration() != 0) {
        Serial.println("[CALIB] No valid BMP390 calibration in EEPROM.");
        // Optionally we could auto-calibrate here if system is on the ground:
        // bmp390_calibrate_offset();
    } else {
        Serial.printf("[CALIB] Applied ground pressure offset: %.2f Pa\n", bmp390_get_ground_pressure());
    }

    // Initialize interrupts and FIFO handling
    bmp390_fifo_init();

    // Apply an initial flight/profile mode to BMP390 (use current_mode mapping)
    BMP390_set_flight_mode((bmp390_mode_t)current_mode);

    // Start FIFO continuous mode; enable pressure & temperature
    if (bmp390_start_fifo_continuous_mode(true, true) != 0) {
        Serial.println("[WARN] Failed to start BMP390 FIFO continuous mode. Trying forced-mode fallback.");
        // As a fallback, leave BMP390 in sleep and rely on forced reads later.
    } else {
        Serial.println("[SENSOR] BMP390 FIFO continuous mode started.");
    }

    // initialize filtered pressure to a sensible value (use calibration if available)
    filtered_pressure_pa = bmp390_get_ground_pressure();
    last_bmp_valid_ms = millis();

    return true;
}

// -------------------- Update (called frequently) --------------------
void SensorManager::update() {
    // Keep existing BMI323 update flow
    // (Original update only read BMI FIFO; keep that)
    bmi323_read_fifo();

    // Handle BMP390 FIFO if an interrupt flagged data ready
    if (bmp_present && fifo_data_ready) {
        // Clear the flag as early as possible
        noInterrupts();
        fifo_data_ready = false;
        interrupts();

        processBMP390Fifo();
    }

    handleBMP390Watchdog();
}

// -------------------- BMP390 FIFO processing --------------------
void SensorManager::processBMP390Fifo() {
    // Read FIFO frames into local buffer
    if (fifo_data_ready) {
        fifo_data_ready = false;

        //Serial.println("[LOOP] FIFO interrupt received");

        // Optional: check FIFO overflow
        if (bmp390_check_fifo_overflow() > 0) {
            Serial.println("[WARN] FIFO Overflow detected");
        }

        // Read FIFO frames
        int ret = bmp390_read_fifo_data(fifo_data, FIFO_BUFFER_SIZE, &frames_available);
       // Serial.printf("[DEBUG] bmp390_read_fifo_data returned: %d | Frames Available: %d\n", ret, frames_available);

        if (ret != 0) {
            Serial.println("[ERROR] Failed to read FIFO data");
            return;
        }

        // Print valid data
       float sum_p = 0, sum_t = 0;
int count = 0;

for (int i = 0; i < frames_available; ++i) {
    if (fifo_data[i].pressure_valid && fifo_data[i].temperature_valid) {
        sum_p += fifo_data[i].pressure;
        sum_t += fifo_data[i].temperature;
        count++;
    }
}

if (count > 0) {
    float avg_p = sum_p / count;
    float avg_t = sum_t / count;
    float alt_ground = bmp390_altitude_from_ground(avg_p, bmp390_get_ground_pressure());
    float alt_sea = bmp390_calculate_altitude(avg_p);
    unsigned long now = millis();

    Serial.printf("[%lu ms] [AVG] P=%.2f Pa | T=%.2f °C | AltG=%.2f m | AltS=%.2f m\n",
        now, avg_p, avg_t, alt_ground, alt_sea);
}

    }
}

// -------------------- Filtering --------------------
void SensorManager::updateFilteredPressure(float new_p) {
    // Simple exponential smoothing: filtered = alpha*new + (1-alpha)*prev
    if (filtered_pressure_pa <= 1e-6f) {
        // First valid sample — initialize filter directly
        filtered_pressure_pa = new_p;
    } else {
        filtered_pressure_pa = pressure_filter_alpha * new_p + (1.0f - pressure_filter_alpha) * filtered_pressure_pa;
    }
}

// -------------------- Watchdog / Recovery --------------------
void SensorManager::handleBMP390Watchdog() {
    if (!bmp_present) return;

    uint32_t now = millis();
    if ((now - last_bmp_valid_ms) > bmp_stale_ms) {
        // Data is stale — take remedial actions: try forced read periodically
        static uint32_t last_attempt_ms = 0;
        if ((now - last_attempt_ms) > 200) { // try forced read every 200ms
            last_attempt_ms = now;
            float p = 0.0f, t = 0.0f;
            if (bmp390_force_measurement_both(&p, &t) == 0) {
                noInterrupts();
                latest_pressure_pa = p;
                latest_temperature_c = t;
                bmp_pressure_valid = true;
                bmp_temp_valid = true;
                last_bmp_valid_ms = millis();
                updateFilteredPressure(p);
                float ground_p = bmp390_get_ground_pressure();
                if (ground_p > 0.0f) {
                    latest_altitude_m = bmp390_altitude_from_ground(filtered_pressure_pa, ground_p);
                } else {
                    latest_altitude_m = bmp390_calculate_altitude(filtered_pressure_pa);
                }
                interrupts();
                Serial.println("[BMP] Forced measurement fallback succeeded.");
            } else {
                Serial.println("[BMP] Forced measurement fallback failed.");
            }
        }
    }
}

// -------------------- Public getters --------------------
bool SensorManager::isBMP390Present() const {
    return bmp_present;
}

bool SensorManager::isPressureValid() const {
    // brief critical section to read volatile flag safely
    bool v;
    noInterrupts();
    v = bmp_pressure_valid;
    interrupts();
    return v;
}

float SensorManager::getPressure() const {
    float p;
    noInterrupts();
    p = latest_pressure_pa;
    interrupts();
    return p;
}

float SensorManager::getTemperature() const {
    float t;
    noInterrupts();
    t = latest_temperature_c;
    interrupts();
    return t;
}

float SensorManager::getAltitude() const {
    float a;
    noInterrupts();
    a = latest_altitude_m;
    interrupts();
    return a;
}

void SensorManager::requestBMP390Recalibration() {
    if (!bmp_present) return;
    Serial.println("[CALIB] Performing BMP390 calibration (EEPROM store)...");
    if (bmp390_calibrate_offset() == 0) {
        Serial.println("[CALIB] BMP390 recalibration complete and saved.");
    } else {
        Serial.println("[CALIB] BMP390 recalibration failed.");
    }
}

// -------------------- Print helpers --------------------
void SensorManager::printBMI323Data() {
    Serial.printf("[BMI323] Pitch: %.2f | Roll: %.2f | Yaw: %.2f\n",
                  estimated_pitch, estimated_roll, estimated_yaw);
}

void SensorManager::printBMP390Data() {
    if (!bmp_present) {
        Serial.println("[BMP] Not present.");
        return;
    }
    bool v = isPressureValid();
    Serial.printf("[BMP] Pressure: %.2f Pa | Temp: %.2f C | Alt: %.2f m | Valid=%s\n",
                  getPressure(), getTemperature(), getAltitude(), v ? "YES" : "NO");
}

// -------------------- Existing helpers preserved --------------------
FlightMode SensorManager::select_flight_mode() {
    Serial.println("Select flight mode:");
    Serial.println("1 - Stable");
    Serial.println("2 - Hover");
    Serial.println("3 - Cruise");
    
    while(!Serial.available());
    char input = Serial.read();
    
    switch(input) {
        case '1': return MODE_STABLE;
        case '2': return MODE_HOVER;
        case '3': return MODE_CRUISE;
        default: return MODE_STABLE;
    }
}

void SensorManager::validate_config() {
    if(current_config == nullptr) {
        Serial.println("[ERROR] No flight mode configured!");
        current_config = &stable_config; // Fallback to stable mode
    }
}

// EOF
