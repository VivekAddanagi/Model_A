#include <Arduino.h>
#include "TimerManager.h"
#include "CommManager.h"
#include "SensorManager.h"

// === Global instances ===
TimerManager timer;
CommManager commManager;
SensorManager sensors;

// === Periodic task: Read sensors ===
void readSensors() {
    sensors.update();
}

// === Periodic task: Update communication (RX) ===
void updateComm() {
    commManager.update();

    if (commManager.hasValidData()) {
        RXData rx = commManager.getControlData();
        Serial.printf("[RX] Throttle: %d | Pitch: %d | Roll: %d | Yaw: %d\n",
                      rx.throttle, rx.pitch, rx.roll, rx.yaw);
    }
}

// === Periodic task: Print sensor data for debugging ===
void debugSensors() {
    IMUData imu = sensors.getIMU();
    float altitude = sensors.getAltitude();
    bool frontBlocked = sensors.isObstacleNear(Direction::FRONT);

    Serial.printf("[SENSOR] Alt: %.2f m | Ax: %.2f | Front Obstacle: %s\n",
                  altitude, imu.ax, frontBlocked ? "YES" : "NO");
}

void setup() {
    Serial.begin(115200);

    // Init modules
    commManager.begin();
    sensors.init();

    // === Register periodic tasks ===
    timer.addTask(updateComm, 10);     // Every 10 ms — RX updates
    timer.addTask(readSensors, 10);    // Every 10 ms — Sensor updates
    timer.addTask(debugSensors, 100);  // Every 100 ms — Debug output
}

void loop() {
    timer.update(); // Runs all tasks in millis()-based scheduler
}
