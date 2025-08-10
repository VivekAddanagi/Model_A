/*// src/SensorManager.cpp
#include "SensorManager.h"
#include "Config.h"
#include "bmi323.h"
#include "bmp390.h"
#include <Preferences.h>

// optional: preferences instance if needed by BMI323 lib
static Preferences prefs;

// === Init Function ===
void SensorManager::init() {
    // --- BMI323 ---
    if (!bmi323_init()) {
        Serial.println("[ERROR] BMI323 init failed.");
    } else {
        Serial.println("[INFO] BMI323 ready.");
        // Setup FIFO if the library supports it (improves efficiency)
        bmi323_setup_fifo();
    }

    // Try to load BMI323 calibrations from flash if library provides helper
    if (load_calibration_from_flash(gyro_cal, accel_cal)) {
        apply_gyro_calibration(&gyro_cal);
        apply_accel_calibration(&accel_cal);
        Serial.println("[INFO] BMI323 calibration loaded from flash.");
    } else {
        Serial.println("[INFO] BMI323 calibration not present.");
    }

    // --- BMP390 ---
    if (bmp390_init_all() != 0) {
        Serial.println("[ERROR] BMP390 init failed.");
    } else {
        Serial.println("[INFO] BMP390 ready.");
    }

    // Load pressure calibration
    if (bmp390_apply_calibration() != 0) {
        Serial.println("[WARN] BMP390 calibration not found or expired.");
    } else {
        Serial.println("[INFO] BMP390 calibration loaded.");
    }

    // --- IR Sensor Pins: use Config.h macros if available ---
    pinMode(IR_EMITTER_FRONT, OUTPUT); digitalWrite(IR_EMITTER_FRONT, LOW);
    pinMode(IR_EMITTER_BACK, OUTPUT);  digitalWrite(IR_EMITTER_BACK, LOW);
    pinMode(IR_EMITTER_LEFT, OUTPUT);  digitalWrite(IR_EMITTER_LEFT, LOW);
    pinMode(IR_EMITTER_RIGHT, OUTPUT); digitalWrite(IR_EMITTER_RIGHT, LOW);

    // Analog receiver pin set by Config.h (IR_RECEIVER)
    analogReadResolution(12); // ESP32 ADC resolution

    // Clear initial values
    memset(obstacleDistances, 0, sizeof(obstacleDistances));
    currentAltitude = 0.0f;
    currentPressure = 0.0f;

    // Initialize timestamps
    lastIMUUpdate = millis();
    lastBaroUpdate = millis();
    lastIRUpdate = millis();
}

// === Update Wrapper ===
void SensorManager::update() {
    uint32_t now = millis();

    if ((now - lastIMUUpdate) >= IMU_UPDATE_INTERVAL_MS) {
        updateIMU();
        lastIMUUpdate = now;
    }

    if ((now - lastBaroUpdate) >= BARO_UPDATE_INTERVAL_MS) {
        updateBarometer();
        lastBaroUpdate = now;
    }

    if ((now - lastIRUpdate) >= IR_UPDATE_INTERVAL_MS) {
        updateIR();
        lastIRUpdate = now;
    }
}

// === IMU ===
void SensorManager::updateIMU() {
    bmi323_data_t raw = {};
    if (bmi323_read(&raw)) {
        // convert raw int16 to engineering units (match your earlier scaling)
        // accelerometer -> g (assuming 4096 LSB/g)
        imuData.ax = raw.ax / 4096.0f;
        imuData.ay = raw.ay / 4096.0f;
        imuData.az = raw.az / 4096.0f;

        // gyroscope -> deg/s (assuming 16.384 LSB/(deg/s))
        imuData.gx = raw.gx / 16.384f - gyro_cal.bias_x;
        imuData.gy = raw.gy / 16.384f - gyro_cal.bias_y;
        imuData.gz = raw.gz / 16.384f - gyro_cal.bias_z;

        // Optionally update orientation using library's update_orientation if desired.
        // update_orientation(imuData.ax, imuData.ay, imuData.az, imuData.gx, imuData.gy, imuData.gz);
    } else {
        // fallback: attempt to read FIFO (if enabled)
        bmi323_read_fifo(); // this will update orientation internally if implemented
    }
}

IMUData SensorManager::getIMU() const{
    return imuData;
}

void SensorManager::calibrateIMU() {
    Serial.println("[CALIB] Starting IMU gyro calibration (quick)...");
    GyroCalibration tempGyro{};
    if (bmi323_quick_gyro_calibrate(&tempGyro)) {
        gyro_cal = tempGyro;
        apply_gyro_calibration(&gyro_cal);
        // Save calibration to flash if helper exists
        save_calibration_to_flash(gyro_cal, accel_cal);
        Serial.println("[CALIB] IMU gyro calibration saved.");
    } else {
        Serial.println("[CALIB] IMU gyro calibration failed.");
    }

    Serial.println("[CALIB] Starting Z-axis accel calibration...");
    AccelCalibration aCal{};
    if (bmi323_z_accel_calibrate(&aCal)) {
        accel_cal = aCal;
        apply_accel_calibration(&accel_cal);
        save_calibration_to_flash(gyro_cal, accel_cal);
        Serial.println("[CALIB] Accel calibration saved.");
    } else {
        Serial.println("[CALIB] Accel calibration failed.");
    }
}

// === BMP390 ===
void SensorManager::updateBarometer() {
    float pressure = 0.0f, temp = 0.0f;
    if (bmp390_force_measurement_both(&pressure, &temp) == 0) {
        currentPressure = pressure;
        // Use ground/offset from bmp390 calibration
        float groundP = bmp390_get_ground_pressure();
        if (groundP <= 0.0f) groundP = 101325.0f; // fallback
        currentAltitude = bmp390_altitude_from_ground(pressure, groundP);
    } else {
        // Could try reading FIFO frames if continuous mode is enabled
        // bmp390_read_fifo_data(...)
    }
}

float SensorManager::getPressure() const {
    return currentPressure;
}

float SensorManager::getAltitude() const {
    return currentAltitude;
}

void SensorManager::calibrateBarometer() {
    if (bmp390_calibrate_offset() == 0) {
        Serial.println("[CALIB] Barometer calibration saved to EEPROM.");
    } else {
        Serial.println("[CALIB] Barometer calibration failed.");
    }
}

// === IR ===
void SensorManager::updateIR() {
    // Simple per-direction emitter activate + averaged analog read
    const uint8_t readCycles = 3;
    uint32_t sum;

    // FRONT
    digitalWrite(IR_EMITTER_FRONT, HIGH);
    delayMicroseconds(IR_SETTLE_TIME_US);
    sum = 0;
    for (uint8_t i=0;i<readCycles;i++) {
        sum += analogRead(IR_RECEIVER_PIN);
        delayMicroseconds(IR_READ_DELAY_US);
    }
    obstacleDistances[0] = float(sum) / readCycles;
    digitalWrite(IR_EMITTER_FRONT, LOW);

    // BACK
    digitalWrite(IR_EMITTER_BACK, HIGH);
    delayMicroseconds(IR_SETTLE_TIME_US);
    sum = 0;
    for (uint8_t i=0;i<readCycles;i++) {
        sum += analogRead(IR_RECEIVER_PIN);
        delayMicroseconds(IR_READ_DELAY_US);
    }
    obstacleDistances[1] = float(sum) / readCycles;
    digitalWrite(IR_EMITTER_BACK, LOW);

    // LEFT
    digitalWrite(IR_EMITTER_LEFT, HIGH);
    delayMicroseconds(IR_SETTLE_TIME_US);
    sum = 0;
    for (uint8_t i=0;i<readCycles;i++) {
        sum += analogRead(IR_RECEIVER_PIN);
        delayMicroseconds(IR_READ_DELAY_US);
    }
    obstacleDistances[2] = float(sum) / readCycles;
    digitalWrite(IR_EMITTER_LEFT, LOW);

    // RIGHT
    digitalWrite(IR_EMITTER_RIGHT, HIGH);
    delayMicroseconds(IR_SETTLE_TIME_US);
    sum = 0;
    for (uint8_t i=0;i<readCycles;i++) {
        sum += analogRead(IR_RECEIVER_PIN);
        delayMicroseconds(IR_READ_DELAY_US);
    }
    obstacleDistances[3] = float(sum) / readCycles;
    digitalWrite(IR_EMITTER_RIGHT, LOW);
}

float SensorManager::getObstacleDistance(Direction dir) const {
    switch (dir) {
        case Direction::FRONT: return obstacleDistances[0];
        case Direction::BACK:  return obstacleDistances[1];
        case Direction::LEFT:  return obstacleDistances[2];
        case Direction::RIGHT: return obstacleDistances[3];
        default: return 0.0f;
    }
}

bool SensorManager::isObstacleNear(Direction dir) const {
    float val = getObstacleDistance(dir);
    // Note: IR returns ADC counts — higher means closer in your code
    return val > IR_DANGER_THRESHOLD;
}
*/