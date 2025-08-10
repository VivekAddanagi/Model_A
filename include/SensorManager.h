#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"
#include "bmi323.h"

class SensorManager {
public:
    SensorManager();
    void printBMI323Data();
    bool begin();
    void update();
    void calibrateBMI323(bool force = false);
    
    
    // Add these method declarations
    FlightMode select_flight_mode();
    void validate_config();

    float getPitch() const { return estimated_pitch; }
    float getRoll() const { return estimated_roll; }
    float getYaw() const { return estimated_yaw; }

private:
    // Remove duplicate declarations (they were in both public and private)
    FlightMode current_mode;
    const FlightModeConfig* current_config;
    
    // Configuration instances
    FlightModeConfig stable_config = {true, true, true, false, 1.0f, 0.1f, 0.05f, 15.0f};
    FlightModeConfig hover_config = {true, true, true, true, 1.2f, 0.15f, 0.08f, 25.0f};
    FlightModeConfig cruise_config = {true, false, false, false, 0.8f, 0.05f, 0.03f, 35.0f};
    
    // Add these if they're used in your implementation
    float estimated_pitch = 0;
    float estimated_roll = 0;
    float estimated_yaw = 0;
};

#endif // SENSOR_MANAGER_H