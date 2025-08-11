#pragma once
#include <Arduino.h>
#include "bmi323.h"   // Already defines FlightModeConfig and FlightMode

class SensorManager {
public:
    SensorManager();
    bool begin();
    void update();
    void calibrateBMI323(bool force);
    void printBMI323Data();

    // Newly added declarations so cpp matches header
    FlightMode select_flight_mode();
    void validate_config();

private:
    FlightMode current_mode;
    FlightModeConfig* current_config;  // Pointer to current mode config

    // Mode configurations
    FlightModeConfig stable_config;
    FlightModeConfig hover_config;
    FlightModeConfig cruise_config;
};
