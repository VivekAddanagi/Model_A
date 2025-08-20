#include "SensorManager.h"
#include "Config.h"
#include <EEPROM.h>

// If these live in other translation units, keep them extern here:
extern bmp390_fifo_data_t fifo_data[FIFO_BUFFER_SIZE];
extern uint16_t frames_available;
extern volatile bool fifo_data_ready;

// latest_altitude_m is consumed in update(); declare extern if defined elsewhere.
// If it's NOT defined anywhere yet, define it once (without extern) in a single .cpp.
extern float latest_altitude_m;

// -------------------- Constructor --------------------
SensorManager::SensorManager() {}

// -------------------- Begin / init --------------------
bool SensorManager::begin() {
    Serial.println("[SENSOR] Initializing BMI323...");
    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
        while (1) { /* halt */ }
    }

    delay(200);

    if (!bmi323_run_selftest()) {
        Serial.println("[WARN] BMI323 self-test failed.");
    }

    // axis map as needed for your board
    bmi323_set_axis_remap(0x00);
    delay(200);

    // start BMI323 FIFO
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

    // NOTE: latest_altitude_m is set in process_bmp390_fifo() from AltG (relative to ground).
    float baro_alt = latest_altitude_m;

    static uint32_t last_update_ms = millis();
    uint32_t now = millis();
    float dt = (now - last_update_ms) * 0.001f;
    last_update_ms = now;

    // Guard dt (avoid huge steps on first loop / serial pauses)
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.01f; // safe fallback (100 Hz)
    }

    // --- Fuse for altitude ---
    updateAltitude(ax, ay, az, roll, pitch, baro_alt, dt);

    // --- Debug print (both are relative to ground) ---
    Serial.printf("ALT_EST: %.2f m | ALT_BARO: %.2f m\n", alt_est, baro_alt);
}

// -------------------- Public getters --------------------
void SensorManager::printBMI323Data() {
    Serial.printf("[BMI323] Pitch: %.2f | Roll: %.2f | Yaw: %.2f\n",
                  estimated_pitch, estimated_roll, estimated_yaw);
}

// -------------------- Existing helpers preserved --------------------
void SensorManager::validate_config() {
    if (current_config == nullptr) {
        Serial.println("[ERROR] No flight mode configured!");
        current_config = &stable_config; // Fallback to stable mode
    }
}

// -------------------- BMP390 FIFO processing --------------------
void SensorManager::process_bmp390_fifo() {
    static unsigned long last_read_ms = 0;
    const unsigned long READ_INTERVAL_MS = 10; // ~100 Hz

    if (!fifo_data_ready && (millis() - last_read_ms < READ_INTERVAL_MS)) return;
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

        float pressure    = fifo_data[i].pressure;
        float temperature = fifo_data[i].temperature;

        // AltG: altitude relative to *ground pressure* (already zero-referenced)
        float alt_ground  = bmp390_altitude_from_ground(pressure, bmp390_get_ground_pressure());
        // AltS: altitude from sea-level formula (absolute)
        float alt_sea     = bmp390_calculate_altitude(pressure);

        // Store *relative* altitude for fusion (this was missing before)
        latest_altitude_m = alt_ground;

        // Optional: timestamp for correlation
        unsigned long frame_time = millis();

        Serial.printf(
            "TEMP: %.2f °C | P: %.2f Pa | AltG: %.2f m | AltS: %.2f m | Time: %lu\n",
            temperature, pressure, alt_ground, alt_sea, frame_time
        );
    }
}

// ==================== Altitude Complementary Filter ====================

// State variables (single definition here; do NOT duplicate in other files)
float alt_est = 0.0f;   // Estimated altitude (relative to ground)
float vel_z   = 0.0f;   // Vertical velocity estimate (m/s)

// Bias & filters
float acc_z_bias = 0.0f;     // learned on ground (tune/estimate separately)
float baro_lpf   = NAN;      // low-pass of baro altitude (relative)

// Robust complementary filter with innovation gate
void SensorManager::updateAltitude(float ax, float ay, float az,
                                   float roll, float pitch,
                                   float alt_baro, float dt) {
    constexpr float ALPHA_ALT = 0.97f;   // more weight on accel prediction
    constexpr float G         = 9.80665f;

    // Smooth baro (alt_baro is already relative to ground)
    if (isnan(baro_lpf)) baro_lpf = alt_baro;
    baro_lpf += 0.1f * (alt_baro - baro_lpf);  // simple 1st-order LPF

    // Rotate body accel into earth Z (using roll/pitch only)
    float sinR = sinf(roll * DEG_TO_RAD);
    float cosR = cosf(roll * DEG_TO_RAD);
    float sinP = sinf(pitch * DEG_TO_RAD);
    float cosP = cosf(pitch * DEG_TO_RAD);

    float acc_earth_z_g =  cosP * cosR * az
                         + cosP * sinR * ay
                         - sinP * ax;

    // Convert g -> m/s² and remove gravity & learned bias (1g)
    float acc_earth_z = (acc_earth_z_g - 1.0f - acc_z_bias) * G;

    // Integrate velocity with clamp to prevent runaway drift
    vel_z += acc_earth_z * dt;
    vel_z = constrain(vel_z, -5.0f, 5.0f);

    // Predict altitude from velocity integration
    float alt_acc = alt_est + vel_z * dt;

    // Innovation gate: ignore insane baro jumps
    float innov = baro_lpf - alt_acc;
    constexpr float gate = 3.0f; // meters
    float k = (fabsf(innov) < gate) ? (1.0f - ALPHA_ALT) : 0.0f;

    // Fuse prediction with (low-passed) baro
    alt_est = (1.0f - k) * alt_acc + k * baro_lpf;
}
