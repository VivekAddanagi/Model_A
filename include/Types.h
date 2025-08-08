#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>

// Direction enum (class-style avoids name collisions)
enum class Direction {
    FRONT,
    BACK,
    LEFT,
    RIGHT
};

// IMU combined data
struct IMUData {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
};

// Altimeter / barometer processed values
struct AltimeterData {
    float altitude = 0.0f;
    float pressure = 0.0f;
    float temperature = 0.0f;
};

// RX control packet (match CC2500 payload fields)
struct RXData {
    int16_t throttle = 0;
    int16_t pitch    = 0;
    int16_t roll     = 0;
    int16_t yaw      = 0;

    uint8_t mode     = 0;
    bool takeoff     = false;
    bool failsafe    = false;
    bool photo       = false;
    bool video       = false;
};

// Simple calibration containers
struct IMUCalibration {
    float gyroOffsetX = 0;
    float gyroOffsetY = 0;
    float gyroOffsetZ = 0;
    float accelOffsetX = 0;
    float accelOffsetY = 0;
    float accelOffsetZ = 0;
};

struct BaroCalibration {
    float pressureOffset = 0;
    float altitudeOffset = 0;
};

#endif // TYPES_H
