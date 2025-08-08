#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>  // For standard integer types

// ============================
// Direction Enum for Obstacle Sensors
// ============================
enum class Direction {
    FRONT,
    BACK,
    LEFT,
    RIGHT
};

// ============================
// IMU Data (Accelerometer + Gyro)
// ============================
struct IMUData {
    float ax;  // Acceleration X (m/s²)
    float ay;  // Acceleration Y
    float az;  // Acceleration Z
    float gx;  // Gyro X (deg/s)
    float gy;  // Gyro Y
    float gz;  // Gyro Z
};

// ============================
// Altimeter / Barometer Data
// ============================
struct AltimeterData {
    float altitude;     // meters
    float pressure;     // Pascals
    float temperature;  // Celsius
};

// ============================
// RX Control Data from Remote
// ============================
struct RXData {
    int16_t throttle;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;

    uint8_t mode;      // flight mode
    bool takeoff;      // takeoff command
    bool failsafe;     // failsafe flag
    bool photo;        // take photo command
    bool video;        // start/stop video command
};

// ============================
// IMU Calibration Data
// ============================
struct IMUCalibration {
    float gyroOffsetX;
    float gyroOffsetY;
    float gyroOffsetZ;
    float accelOffsetX;
    float accelOffsetY;
    float accelOffsetZ;
};

// ============================
// Barometer Calibration Data
// ============================
struct BaroCalibration {
    float pressureOffset;
    float altitudeOffset;
};

#endif // TYPES_H
