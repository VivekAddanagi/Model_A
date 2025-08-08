#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "types.h"
#include "Config.h"
#include "bmi323.h"
#include "bmp390.h"

// SensorManager provides unified access to IMU, barometer, IR avoidance
class SensorManager {
public:
    SensorManager() = default;
    ~SensorManager() = default;

    // init sensors, load calibrations from flash
    void init();

    // non-blocking update; call in loop()
    void update();

    // getters (copy small structs)
    IMUData getIMU() const;
    float getAltitude() const;
    float getPressure() const;

    // IR obstacle API (true if closer than threshold)
    bool isObstacleNear(Direction dir) const;
    float getObstacleDistance(Direction dir) const;

    // Calibration routines (interactive or automated)
    void calibrateIMU();
    void calibrateBarometer();

private:
    // helpers
    void updateIMU();
    void updateBarometer();
    void updateIR();

    // state
    IMUData imuData{};
    float obstacleDistances[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // FRONT,BACK,LEFT,RIGHT
    float currentAltitude = 0.0f;
    float currentPressure = 0.0f;

    uint32_t lastIMUUpdate = 0;
    uint32_t lastBaroUpdate = 0;
    uint32_t lastIRUpdate = 0;
};

#endif // SENSOR_MANAGER_H
