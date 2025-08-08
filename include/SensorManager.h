#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "types.h"       // Shared data structures
#include "Config.h"      // Centralized pin assignments & settings
#include "bmi323.h"      // IMU driver
#include "bmp390.h"      // Barometer driver

// SensorManager.h

class SensorManager {
public:
    void init();
    void update();
    IMUData getIMU();
    float getAltitude();
    float getPressure();
    bool isObstacleNear(Direction dir);
    float getObstacleDistance(Direction dir);
    void calibrateIMU();
    void calibrateBarometer();

private:
    // Internal helper functions
    void updateIMU();
    void updateBarometer();
    void updateIR();

    // Internal state variables
    IMUData imuData{};
    float obstacleDistances[4] = {0}; // FRONT, BACK, LEFT, RIGHT
    float currentAltitude = 0.0f;
    float currentPressure = 0.0f;

    uint32_t lastIMUUpdate = 0;
    uint32_t lastBaroUpdate = 0;
    uint32_t lastIRUpdate = 0;
};

#endif
