#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

// Include sensor drivers
#include "bmi323.h"
#include "bmp390.h"

// === IR Sensor Pins ===
#define IR_FRONT_PIN  32
#define IR_BACK_PIN   33
#define IR_LEFT_PIN   34
#define IR_RIGHT_PIN  35

// === Threshold for obstacle detection ===
#define OBSTACLE_THRESHOLD_CM  30.0f  // Adjust as needed

// === Update Intervals ===
#define IMU_UPDATE_INTERVAL_MS      5    // 200 Hz
#define BARO_UPDATE_INTERVAL_MS     40   // 25 Hz
#define IR_UPDATE_INTERVAL_MS       20   // 50 Hz

// === IMU Output Struct ===
struct IMUData {
    float ax, ay, az;
    float gx, gy, gz;
};

// === Obstacle Directions ===
enum class Direction {
    FRONT,
    BACK,
    LEFT,
    RIGHT
};

class SensorManager {
public:
    void init();                    // Init all sensors & load calibration
    void update();                  // Periodic non-blocking update

    IMUData getIMU();               // Latest IMU values
    float getAltitude();            // Relative altitude in meters
    float getPressure();            // Absolute pressure in Pa

    float getObstacleDistance(Direction dir);    // Raw analog value
    bool isObstacleNear(Direction dir);          // Threshold check

    void calibrateIMU();            // Starts BMI323 gyro/accel calibration
    void calibrateBarometer();      // Saves baseline ground pressure to EEPROM

private:
    // Sensor state
    IMUData imuData;
    float currentPressure;
    float currentAltitude;
    float obstacleDistances[4]; // Index 0: FRONT, 1: BACK, 2: LEFT, 3: RIGHT

    // Timestamps
    uint32_t lastIMUUpdate = 0;
    uint32_t lastBaroUpdate = 0;
    uint32_t lastIRUpdate = 0;

    void updateIMU();
    void updateBarometer();
    void updateIR();
};

#endif // SENSOR_MANAGER_H
