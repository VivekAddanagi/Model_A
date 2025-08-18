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

    // --- BMP390 update ---
    process_bmp390_fifo();

    // --- Get latest sensor values ---
   float ax = latest_ax;
   float ay = latest_ay;
   float az = latest_az;

   float gx = latest_gx;
   float gy = latest_gy;
   float gz = latest_gz;


    float roll  = estimated_roll;
    float pitch = estimated_pitch;

    float baro_alt = latest_altitude_m;   // from BMP390 FIFO handler
    static uint32_t last_update_ms = millis();
    uint32_t now = millis();
    float dt = (now - last_update_ms) / 1000.0f;
    last_update_ms = now;

    // --- Fuse for altitude ---
    updateAltitude(ax, ay, az, roll, pitch, baro_alt, dt);

    // --- Debug print ---
    Serial.printf("ALT_EST: %.2f m | ALT_BARO: %.2f m\n", alt_est, baro_alt);
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

    if (!fifo_data_ready && millis() - last_read_ms < READ_INTERVAL_MS) return;
    last_read_ms = millis();
    fifo_data_ready = false;

    if (bmp390_check_fifo_overflow() > 0)
        Serial.println("[WARN] BMP390 FIFO Overflow detected");

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

        // Use **millis() as software timestamp**, same reference as BMI323
        unsigned long frame_time = millis();

        Serial.printf(
            "TEMP: %.2f °C | P: %.2f Pa | AltG: %.2f m | AltS: %.2f m | Time: %lu\n",
            temperature, pressure, alt_ground, alt_sea, frame_time
        );
    }
}

// EOF

// Complementary filter for altitude
void SensorManager::updateAltitude(float ax, float ay, float az,
                                   float roll, float pitch,
                                   float baro_alt, float dt) {
    const float ALPHA_ALT = 0.98f;
    const float G = 9.80665f;

    // Rotate accel into earth Z
    float sinR = sinf(roll * DEG_TO_RAD);
    float cosR = cosf(roll * DEG_TO_RAD);
    float sinP = sinf(pitch * DEG_TO_RAD);
    float cosP = cosf(pitch * DEG_TO_RAD);

    float acc_earth_z = cosP * cosR * az +
                        cosP * sinR * ay -
                        sinP * ax;

    // convert g->m/s², subtract gravity
    acc_earth_z = acc_earth_z * G - G;

    // integrate velocity + altitude
    vel_z   += acc_earth_z * dt;
    float alt_acc = alt_est + vel_z * dt;

    // complementary filter with barometer
    alt_est = ALPHA_ALT * alt_acc + (1.0f - ALPHA_ALT) * baro_alt;
}
