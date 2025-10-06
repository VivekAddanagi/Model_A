#include "SensorManager.h"
#include "Config.h"
#include "bmp390.h"
#include <Preferences.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <math.h>
#include <vector>
#include <algorithm>

// near the top of SensorManager.cpp with other globals
extern float estimated_roll;
extern float estimated_pitch;
extern float estimated_yaw; // required for full rotation from body -> earth

// exported velocity state (m/s)
float vel_x = 0.0f;
float vel_y = 0.0f;
float vel_z = 0.0f; // you'll already be setting vel_z from EKF

// Add simple getters (if you prefer them as member functions, add declarations in SensorManager.h)
float SensorManager::getVelX() const { return vel_x; }
float SensorManager::getVelY() const { return vel_y; }
float SensorManager::getVelZ() const { return vel_z; }



// If these live in other translation units, keep them extern here:
extern bmp390_fifo_data_t fifo_data[BMP390_FIFO_BUFFER_SIZE];
extern uint16_t frames_available;

extern float pressure_offset;
 float avg_t;

// Latest altitude for EKF
float latest_altitude_m = 0.0f;

void apply_bmi323_mode(FlightMode mode);
void apply_bmp390_mode(FlightMode mode);
void run_calibration_sequence_startup();

// -------------------- Constructor --------------------
SensorManager::SensorManager() {}

const char* SensorManager::getStatusString() const {
    switch (status) {
        case SENSOR_STATUS_OK:    return "OK";
        case SENSOR_STATUS_ERROR: return "ERROR";
        default:                  return "UNKNOWN";
    }
}


// -------------------- Begin / init --------------------
bool SensorManager::begin(float sea_level_pressure, FlightMode mode) {
   // Serial.println("[SENSOR] === SensorManager Begin Sequence ===");

    bool bmi_ok = true;
    bool bmp_ok = true;

    // 1️⃣ Initialize BMI323
    //Serial.println("[SENSOR] Initializing BMI323...");
    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
        bmi_ok = false;
    } else if (!bmi323_run_selftest()) {
        Serial.println("[WARN] BMI323 self-test failed.");
        // still allow flight but degraded
    } else {
       // Serial.println("[SENSOR] BMI323 self-test passed.");
    }

    // 2️⃣ Initialize BMP390
   // Serial.println("[SENSOR] Initializing BMP390...");
    if (!bmp390_init_all()) {
        Serial.println("[ERROR] BMP390 init failed!");
        bmp_ok = false;
    }

    // 3️⃣ If either sensor failed → mark error & return
    if (!bmi_ok || !bmp_ok) {
        status = SENSOR_STATUS_ERROR;
        Serial.println("[SENSOR] === SensorManager Begin FAILED ===");
        return false;
    }

    // 4️⃣ Apply configs + calibration
    apply_bmi323_mode(mode);
    apply_bmp390_mode(mode);
    run_calibration_sequence_startup();

    // 5️⃣ Setup FIFO + start BMP390
    if (!bmi323_setup_fifo()) {
        Serial.println("[ERROR] BMI323 FIFO setup failed!");
        status = SENSOR_STATUS_ERROR;
        return false;
    }

    if (!bmp390_begin()) {
        Serial.println("[ERROR] BMP390 begin failed!");
        status = SENSOR_STATUS_ERROR;
        return false;
    }

    // ✅ All good
    status = SENSOR_STATUS_OK;
    Serial.println("[SENSOR] === SensorManager Begin SUCCESS ===");
    return true;
}


// -------------------- Update (called frequently) --------------------
void SensorManager::update() {
    bmi323_read_fifo();
    delayMicroseconds(500); 
    process_bmp390_fifo();

    float ax = latest_ax;
    float ay = latest_ay;
    float az = latest_az;
    float roll  = estimated_roll;
    float pitch = estimated_pitch;
    float baro_alt = latest_altitude_m;

    static uint32_t last_update_ms = millis();
    uint32_t now = millis();
    float dt = (now - last_update_ms) * 0.001f;
    last_update_ms = now;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f; 

    updateAltitude(ax, ay, az, roll, pitch, baro_alt, dt);

    // --- Print velocities every 1000 ms ---
    static uint32_t last_print_ms = 0;
    const uint32_t PRINT_INTERVAL_MS = 1000; // 1 seconds
    if (millis() - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = millis();
        Serial.printf("[VELOCITY] vx=%.3f m/s, vy=%.3f m/s, vz=%.3f m/s\n", 
                      getVelX(), getVelY(), getVelZ());
    }
}




// -------------------- BMP390 FIFO processing (stabilized) --------------------
void SensorManager::process_bmp390_fifo() {
    static unsigned long last_read_ms = 0;
    static unsigned long last_successful_read_ms = 0;
    const unsigned long READ_INTERVAL_MS = 10; // 100 Hz sensor processing

    // === Two-stage low-pass filter states ===
    static float p_fast  = pressure_offset; // fast LPF state
    static float p_slow  = pressure_offset; // slow LPF state
    static float t_filt  = 29.0f;           // filtered temperature

    // Constants
    const float ALPHA_FAST = 0.05f;  // fast response
    const float ALPHA_SLOW = 0.01f;  // slow response
    const float TEMP_COEFF = 0.12f;  // Pa/°C temperature compensation

    if (!fifo_data_ready && (millis() - last_read_ms < READ_INTERVAL_MS)) return;

    unsigned long now = millis();
    last_read_ms = now; // tracks "attempts"
    fifo_data_ready = false;

    if (bmp390_check_fifo_overflow() > 0)
        Serial.println("[WARN] BMP390 FIFO Overflow detected");

    // === Read FIFO ===
    uint16_t frames_available = 0;
    int ret = bmp390_read_fifo_data(fifo_data, BMP390_FIFO_BUFFER_SIZE, &frames_available);
    if (ret != 0) {
        Serial.println("[ERROR] BMP390 FIFO read failed");
        bmp390_write(BMP390_REG_CMD, BMP390_CMD_FIFO_FLUSH);
        delay(10);
        return;
    }
    if (frames_available == 0) return;

    // === Collect valid frames ===
    std::vector<float> pressures, temps;
    pressures.reserve(frames_available);
    temps.reserve(frames_available);

    for (int i = 0; i < frames_available; ++i) {
        if (!fifo_data[i].pressure_valid || !fifo_data[i].temperature_valid) continue;
        pressures.push_back(fifo_data[i].pressure);
        temps.push_back(fifo_data[i].temperature);
    }
    if (pressures.empty()) return;

    // === Trimmed mean for pressure ===
    std::sort(pressures.begin(), pressures.end());
    size_t trim = pressures.size() / 10;
    float pressure_sum = 0;
    for (size_t i = trim; i < pressures.size() - trim; ++i)
        pressure_sum += pressures[i];
    float pressure_avg = pressure_sum / (pressures.size() - 2 * trim);

    // === Simple average for temperature ===
    float temp_sum = 0;
    for (auto t : temps) temp_sum += t;
    float temp_avg = temp_sum / temps.size();

    // === Temperature-compensated cascaded LPF ===
    float p_comp = pressure_avg + TEMP_COEFF * (temp_avg - t_filt);

    p_fast = ALPHA_FAST * p_comp + (1.0f - ALPHA_FAST) * p_fast;   // fast LPF
    p_slow = ALPHA_SLOW * p_fast + (1.0f - ALPHA_SLOW) * p_slow;   // cascade into slow LPF
    t_filt = ALPHA_FAST * temp_avg + (1.0f - ALPHA_FAST) * t_filt; // temp LPF

    float pressure_filtered = p_slow; // final filtered output

    // === Altitude calculations ===
    float alt_ground = bmp390_altitude_from_ground(pressure_filtered, bmp390_get_ground_pressure());
    float alt_sea    = bmp390_calculate_altitude(pressure_filtered);
    latest_altitude_m = alt_ground;

    // === Auto-update ground pressure (if stationary) ===
    static unsigned long last_offset_update = 0;
    if (millis() - last_offset_update > 60000) { // every 1 min
        pressure_offset = pressure_filtered;
        last_offset_update = millis();
    }

    // === Measure gap between successful reads ===
    unsigned long read_gap = now - last_successful_read_ms;
    last_successful_read_ms = now;

    // === Throttled debug print ===
    static unsigned long last_print_ms = 0;
    const unsigned long PRINT_INTERVAL_MS = 1500; // print every 1.5 s

    if (millis() - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = millis();
        Serial.printf(
            "[BMP390] Gap:%lu ms | TEMP: %.2f °C | P: %.2f Pa | AltG: %.2f m | AltS: %.2f m | Frames=%d\n",
            read_gap, t_filt, pressure_filtered, alt_ground, alt_sea, (int)pressures.size()
        );
    }
}


// -------------------- Altitude EKF --------------------
static float h = 0.0f;   // altitude
static float v = 0.0f;   // vertical velocity
static float b = 0.0f;   // accel bias
static float P[3][3] = { {1,0,0},{0,1,0},{0,0,0.01} };

void SensorManager::updateAltitude(float ax, float ay, float az,
                                   float roll, float pitch,
                                   float alt_baro, float dt) {
    constexpr float G = 9.80665f;
    static bool initialized = false;

    if (!initialized) {
        h = alt_baro;
        v = 0.0f;
        b = 0.0f;
        vel_x = vel_y = vel_z = 0.0f; // seed horizontals to zero
        initialized = true;
        Serial.printf("[ALT EKF INIT] seeded at %.2f m\n", alt_baro);
        return;
    }

    // Precompute sin/cos
    float sinR = sinf(roll * DEG_TO_RAD);
    float cosR = cosf(roll * DEG_TO_RAD);
    float sinP = sinf(pitch * DEG_TO_RAD);
    float cosP = cosf(pitch * DEG_TO_RAD);

    // Use current yaw estimate (extern)
    float yaw = estimated_yaw;
    float sinY = sinf(yaw * DEG_TO_RAD);
    float cosY = cosf(yaw * DEG_TO_RAD);

    // --- Rotate accelerometer (body -> earth frame), measured in g
    // Using standard rotation: Rz(yaw) * Ry(pitch) * Rx(roll)
    // ax,ay,az are in g (you later multiply by G)
    float acc_e_x_g = ax * (cosP * cosY)
                    + ay * (sinR * sinP * cosY - cosR * sinY)
                    + az * (cosR * sinP * cosY + sinR * sinY);

    float acc_e_y_g = ax * (cosP * sinY)
                    + ay * (sinR * sinP * sinY + cosR * cosY)
                    + az * (cosR * sinP * sinY - sinR * cosY);

    float acc_e_z_g = ax * (-sinP)
                    + ay * (cosP * sinR)
                    + az * (cosP * cosR);

    // --- Vertical EKF prediction (unchanged except use acc_e_z_g)
    float h_pred = h + v*dt;
    float v_pred = v + ((acc_e_z_g - 1.0f - b)*G)*dt; // subtract gravity (1g) and bias
    float b_pred = b;

    // Jacobian / covariance propagation (your existing code)
    float F[3][3] = {{1, dt, 0},{0,1,-G*dt},{0,0,1}};
    float Q[3][3] = {{0.001f,0,0},{0,0.01f,0},{0,0,0.00001f}};
    float Pnew[3][3] = {0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) for(int l=0;l<3;l++)
        Pnew[i][j] += F[i][k]*P[k][l]*F[j][l];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        Pnew[i][j] += Q[i][j];

    h = h_pred; v = v_pred; b = b_pred;
    memcpy(P, Pnew, sizeof(P));

    // Measurement update (unchanged)
    float z = alt_baro;
    float y = z - h;
    constexpr float R = 0.25f;
    float S = P[0][0] + R;
    float K[3] = { P[0][0]/S, P[1][0]/S, P[2][0]/S };

    h += K[0]*y; v += K[1]*y; b += K[2]*y;

    // Covariance update
    float Pupd[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        Pupd[i][j] = P[i][j] - K[i]*P[0][j];
    memcpy(P, Pupd, sizeof(Pupd));

    // Export vertical
    alt_est = h;
    vel_z = v;

    // --- Horizontal integration (simple inertial integration of earth-frame accel)
    // Convert earth-frame accel from g -> m/s^2
    float acc_e_x = acc_e_x_g * G;
    float acc_e_y = acc_e_y_g * G;

    // Remove any mean bias if you keep an estimate; here we don't have horizontal bias estimate,
    // so this is raw integration (will drift). Consider adding zero-velocity updates or external aids.
    vel_x += acc_e_x * dt;
    vel_y += acc_e_y * dt;

    // Optional: small velocity damping to keep runaway in check (tiny value)
    const float vel_damping = 0.9995f;
    vel_x *= vel_damping;
    vel_y *= vel_damping;
}



// -------------------- Getter for altitude --------------------
float SensorManager::getAltitude() const {
    // Option A: Use EKF estimate (smoother, better for flight control)
    return alt_est;

    // Option B: Use raw baro altitude if you prefer
    // return latest_altitude_m;
}
