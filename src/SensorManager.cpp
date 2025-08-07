#include "SensorManager.h"

// === Init Function ===
void SensorManager::init() {
    // --- BMI323 ---
    if (bmi323_init_spi() != 0) {
        Serial.println("[ERROR] BMI323 init failed.");
    } else {
        Serial.println("[INFO] BMI323 ready.");
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
    }

    // --- IR Sensor Pins ---
    pinMode(IR_FRONT_PIN, INPUT);
    pinMode(IR_BACK_PIN, INPUT);
    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);

    // Clear initial values
    memset(obstacleDistances, 0, sizeof(obstacleDistances));
    currentAltitude = 0;
    currentPressure = 0;
}

// === Update Wrapper ===
void SensorManager::update() {
    uint32_t now = millis();

    if (now - lastIMUUpdate >= IMU_UPDATE_INTERVAL_MS) {
        updateIMU();
        lastIMUUpdate = now;
    }

    if (now - lastBaroUpdate >= BARO_UPDATE_INTERVAL_MS) {
        updateBarometer();
        lastBaroUpdate = now;
    }

    if (now - lastIRUpdate >= IR_UPDATE_INTERVAL_MS) {
        updateIR();
        lastIRUpdate = now;
    }
}

// === IMU ===
void SensorManager::updateIMU() {
    float ax, ay, az, gx, gy, gz;
    if (bmi323_get_fifo_data_filtered(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
        imuData.ax = ax;
        imuData.ay = ay;
        imuData.az = az;
        imuData.gx = gx;
        imuData.gy = gy;
        imuData.gz = gz;
    }
}

IMUData SensorManager::getIMU() {
    return imuData;
}

void SensorManager::calibrateIMU() {
    Serial.println("[CALIB] Starting IMU calibration...");
    bmi323_run_self_calibration(); // Implemented in your driver
    Serial.println("[CALIB] IMU calibration complete.");
}

// === BMP390 ===
void SensorManager::updateBarometer() {
    float pressure, temp;
    if (bmp390_force_measurement_both(&pressure, &temp) == 0) {
        currentPressure = pressure;
        currentAltitude = bmp390_altitude_from_ground(pressure, bmp390_get_ground_pressure());
    }
}

float SensorManager::getPressure() {
    return currentPressure;
}

float SensorManager::getAltitude() {
    return currentAltitude;
}

void SensorManager::calibrateBarometer() {
    bmp390_calibrate_offset();
    Serial.println("[CALIB] Barometer calibration saved.");
}

// === IR ===
void SensorManager::updateIR() {
    obstacleDistances[0] = analogRead(IR_FRONT_PIN); // FRONT
    obstacleDistances[1] = analogRead(IR_BACK_PIN);  // BACK
    obstacleDistances[2] = analogRead(IR_LEFT_PIN);  // LEFT
    obstacleDistances[3] = analogRead(IR_RIGHT_PIN); // RIGHT
}

float SensorManager::getObstacleDistance(Direction dir) {
    switch (dir) {
        case Direction::FRONT: return obstacleDistances[0];
        case Direction::BACK:  return obstacleDistances[1];
        case Direction::LEFT:  return obstacleDistances[2];
        case Direction::RIGHT: return obstacleDistances[3];
    }
    return 0;
}

bool SensorManager::isObstacleNear(Direction dir) {
    float dist = getObstacleDistance(dir);
    return dist < OBSTACLE_THRESHOLD_CM;
}
