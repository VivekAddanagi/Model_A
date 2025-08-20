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

// ==================== Altitude Complementary Filter ====================

// State variables (single definition here; do NOT duplicate in other files)
float alt_est = 0.0f;   // Estimated altitude (relative to ground)
float vel_z   = 0.0f;   // Vertical velocity estimate (m/s)

// Bias & filters
float acc_z_bias = accel_cal.bias_z;     // learned on ground (tune/estimate separately)
float baro_lpf   = NAN;      // low-pass of baro altitude (relative)

// Robust complementary filter with averaged seeding
void SensorManager::updateAltitude(float ax, float ay, float az,
                                   float roll, float pitch,
                                   float alt_baro, float dt)
{
    // ---- constants ----
    constexpr float G = 9.80665f;
    // stationary detector thresholds
    constexpr float GYRO_STILL_DPS   = 0.5f;   // per-axis or norm threshold
    constexpr float ACC_MAG_TOL_G    = 0.05f;  // |mag-1g| tolerance
    // bias learning & velocity leak
    constexpr float BIAS_BETA        = 0.02f;  // bias IIR when stationary
    constexpr float LAMBDA_STOP      = 1.5f;   // vel leak (s^-1) when still
    constexpr float LAMBDA_MOVE      = 0.2f;   // vel leak when moving
    // complementary filter weights
    constexpr float ALPHA_STOP       = 0.90f;  // more baro when still
    constexpr float ALPHA_MOVE       = 0.96f;  // more accel when moving
    // innovation gate & snapback
    constexpr float INNOV_GATE_M     = 3.0f;   // meters
    constexpr float SNAP_THRESH_M    = 0.5f;   // snap to baro when still

    // dt guard
    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

    // ---- Initialization with averaging (~1s of baro samples) ----
    static bool initialized = false;
    if (!initialized) {
        static int count = 0;
        static float sum = 0.0f;
        sum += alt_baro;
        count++;
        if (count >= 100) { // ~1s at 100 Hz
            float avg = sum / count;
            baro_lpf = avg;
            alt_est  = avg;
            vel_z    = 0.0f;
            initialized = true;
            Serial.printf("[ALT INIT] seeded at %.2f m\n", avg);
        }
        return; // skip fusion until initialized
    }

    // ---- baro low-pass (AltG: relative to ground) ----
    baro_lpf += 0.1f * (alt_baro - baro_lpf);

    // ---- rotate body accel into earth Z (roll/pitch only) ----
    float sinR = sinf(roll * DEG_TO_RAD);
    float cosR = cosf(roll * DEG_TO_RAD);
    float sinP = sinf(pitch * DEG_TO_RAD);
    float cosP = cosf(pitch * DEG_TO_RAD);

    float acc_earth_z_g =  cosP * cosR * az
                         + cosP * sinR * ay
                         - sinP * ax;     // still in g

    // ---- stationary detection ----
    float acc_mag_g = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_norm = sqrtf(latest_gx*latest_gx +
                            latest_gy*latest_gy +
                            latest_gz*latest_gz);
    bool acc_ok   = fabsf(acc_mag_g - 1.0f) < ACC_MAG_TOL_G;
    bool gyro_ok  = gyro_norm < GYRO_STILL_DPS;
    static int still_count = 0;
    static int move_count  = 0;
    if (acc_ok && gyro_ok) { still_count++; move_count = 0; }
    else { move_count++; still_count = 0; }
    bool stationary = (still_count >= 5);   // ~50 ms at 100 Hz

    // ---- bias learning when stationary ----
    if (stationary) {
        float err = (acc_earth_z_g - 1.0f) - acc_z_bias;
        acc_z_bias += BIAS_BETA * err;          
        acc_z_bias = constrain(acc_z_bias, -0.05f, 0.05f);
    }

    // ---- remove gravity & bias, convert to m/s^2 ----
    float acc_earth_z = (acc_earth_z_g - 1.0f - acc_z_bias) * G;

    // ---- integrate velocity with leak ----
    float lambda = stationary ? LAMBDA_STOP : LAMBDA_MOVE;
    vel_z += acc_earth_z * dt;
    vel_z -= lambda * vel_z * dt;           
    vel_z = constrain(vel_z, -2.0f, 2.0f);

    // ---- predict altitude ----
    float alt_acc = alt_est + vel_z * dt;

    // ---- complementary fusion (adaptive alpha) ----
    float alpha = stationary ? ALPHA_STOP : ALPHA_MOVE;
    float innov = baro_lpf - alt_acc;
    float k = (fabsf(innov) < INNOV_GATE_M) ? (1.0f - alpha) : 0.0f;

    alt_est = (1.0f - k) * alt_acc + k * baro_lpf;

    // ---- snapback when landed/still and diverged ----
    if (stationary && fabsf(alt_est - baro_lpf) > SNAP_THRESH_M) {
        alt_est = baro_lpf;
        vel_z   = 0.0f;
    }
}
