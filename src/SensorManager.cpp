#include "SensorManager.h"
#include "Config.h"
#include "bmp390.h"
#include <Preferences.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <math.h>

// If these live in other translation units, keep them extern here:
extern bmp390_fifo_data_t fifo_data[BMP390_FIFO_BUFFER_SIZE];
extern uint16_t frames_available;


// Latest altitude for EKF
float latest_altitude_m = 0.0f;

// -------------------- Constructor --------------------
SensorManager::SensorManager() {}

// -------------------- Begin / init --------------------
bool SensorManager::begin(float sea_level_pressure) {
    Serial.println("[SENSOR] Initializing BMI323...");
    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
        while (1) { /* halt */ }
    }

    delay(200);

    if (!bmi323_run_selftest()) {
        Serial.println("[WARN] BMI323 self-test failed.");
    }

    bmi323_set_axis_remap(0x00); // Adjust as needed
    delay(200);

    bmi323_setup_fifo();

    // -------------------- BMP390 Calibration --------------------
    if (!bmp390_apply_calibration()) {
        Serial.println("[BMP390] No saved calibration found. Running fresh calibration...");
        if (bmp390_calibrate_offset(sea_level_pressure) == 0) {
            Serial.println("[BMP390] Calibration complete.");
        } else {
            Serial.println("[ERROR] BMP390 calibration failed!");
        }
    } else {
        Serial.println("[BMP390] Calibration loaded from NVS.");
    }

    return true;
}

// -------------------- Update (called frequently) --------------------
void SensorManager::update() {
    bmi323_read_fifo();
   // process_bmp390_fifo();

    // EKF and sensor fusion
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
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f; // safe fallback

    updateAltitude(ax, ay, az, roll, pitch, baro_alt, dt);
}

// -------------------- BMP390 FIFO processing --------------------
void SensorManager::process_bmp390_fifo() {
    static unsigned long last_read_ms = 0;
    const unsigned long READ_INTERVAL_MS = 0; // 100 Hz

    if (!fifo_data_ready && (millis() - last_read_ms < READ_INTERVAL_MS)) return;
    last_read_ms = millis();
    fifo_data_ready = false;

    if (bmp390_check_fifo_overflow() > 0)
        Serial.println("[WARN] BMP390 FIFO Overflow detected");

    int ret = bmp390_read_fifo_data(fifo_data, BMP390_FIFO_BUFFER_SIZE, &frames_available);
    if (ret != 0) {
        Serial.println("[ERROR] BMP390 FIFO read failed");
        return;
    }

    if (frames_available == 0) return;

    for (int i = 0; i < frames_available; ++i) {
        if (!fifo_data[i].pressure_valid || !fifo_data[i].temperature_valid) continue;

        float pressure    = fifo_data[i].pressure;
        float temperature = fifo_data[i].temperature;

        float alt_ground  = bmp390_altitude_from_ground(pressure, bmp390_get_ground_pressure());
        float alt_sea     = bmp390_calculate_altitude(pressure);

        latest_altitude_m = alt_ground;

        // Optional: throttle debug prints
        static uint32_t last_debug = 0;
        if (millis() - last_debug > 0) {
            last_debug = millis();

            // Add timestamp printing similar to BMI323 FIFO
            Serial.printf(
                "ms:%lu | TEMP: %.2f °C | P: %.2f Pa | AltG: %.2f m | AltS: %.2f m\n",
                millis(), temperature, pressure, alt_ground, alt_sea
            );
        }
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
        initialized = true;
        Serial.printf("[ALT EKF INIT] seeded at %.2f m\n", alt_baro);
        return;
    }

    // Rotate accel into earth Z
    float sinR = sinf(roll * DEG_TO_RAD);
    float cosR = cosf(roll * DEG_TO_RAD);
    float sinP = sinf(pitch * DEG_TO_RAD);
    float cosP = cosf(pitch * DEG_TO_RAD);
    float acc_earth_z_g =  cosP * cosR * az + cosP * sinR * ay - sinP * ax;

    // Prediction
    float h_pred = h + v*dt;
    float v_pred = v + ((acc_earth_z_g - 1.0f - b)*G)*dt;
    float b_pred = b;

    // Jacobian
    float F[3][3] = {{1, dt, 0},{0,1,-G*dt},{0,0,1}};
    float Q[3][3] = {{0.001f,0,0},{0,0.01f,0},{0,0,0.00001f}};
    float Pnew[3][3] = {0};
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) for(int l=0;l<3;l++)
        Pnew[i][j] += F[i][k]*P[k][l]*F[j][l];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++)
        Pnew[i][j] += Q[i][j];

    h = h_pred; v = v_pred; b = b_pred;
    memcpy(P, Pnew, sizeof(P));

    // Measurement update
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

    // Export results
    alt_est = h;
    vel_z = v;
}
