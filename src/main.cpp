#include <Arduino.h>
#include "CommManager.h"
#include "SensorManager.h"

CommManager commManager;
SensorManager sensors;

void setup() {
    Serial.begin(115200);

    // Initialize communication and sensors
    commManager.begin();
    sensors.init();
}

void loop() {
    // === Update RX + Sensors (non-blocking) ===
    commManager.update();
    sensors.update();

    // === Get RX control data if available ===
    if (commManager.hasValidData()) {
        RXData rx = commManager.getControlData();

        // You can use: rx.throttle, rx.pitch, rx.roll, rx.yaw
        Serial.printf("[RX] Throttle: %d | Pitch: %d | Roll: %d | Yaw: %d\n",
                      rx.throttle, rx.pitch, rx.roll, rx.yaw);
    }

    // === Get Sensor data ===
    IMUData imu = sensors.getIMU();
    float altitude = sensors.getAltitude();
    bool frontBlocked = sensors.isObstacleNear(Direction::FRONT);

    // === Debug print ===
    Serial.printf("[SENSOR] Alt: %.2f m | Ax: %.2f | Front Obstacle: %s\n",
                  altitude, imu.ax, frontBlocked ? "YES" : "NO");

    // === Minimal delay (for serial stability) ===
    delay(5);
}
