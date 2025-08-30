#pragma once

#include <Arduino.h>
#include "bmi323.h"   // Already defines FlightModeConfig and FlightMode
#include "Config.h"
#include "bmp390.h"

class SensorManager {
public:
    SensorManager();
    bool begin(float sea_level_pressure);
    void update();
    void calibrateBMI323(bool force);
    void printBMI323Data();
    void process_bmp390_fifo();  // Declare BMP390 FIFO handler as a member
    // Newly added declarations so cpp matches header
    FlightMode select_flight_mode();
    void validate_config();

    // BMP390 related API
    bool isBMP390Present() const;
    bool isPressureValid() const;
    float getPressure() const;          // Pa
    float getTemperature() const;       // °C
    float getAltitude() const;          // meters (relative to ground calibration if available)
    void requestBMP390Recalibration();  // trigger recalibration (EEPROM + runtime)

    // Debug prints
    void printBMP390Data();
     // helper for altitude filter
    void updateAltitude(float ax, float ay, float az,
                        float roll, float pitch,
                        float baro_alt, float dt);
     float alt_est = 0.0f;

private:
    // ------- existing BMI323-related members (preserve original names) -------
    // (Assumed types/variables referenced in original SensorManager.cpp)
    FlightMode current_mode;
    FlightModeConfig stable_config;
    FlightModeConfig hover_config;
    FlightModeConfig cruise_config;
    FlightModeConfig* current_config = nullptr;

    float estimated_pitch = 0.0f;
    float estimated_roll  = 0.0f;
    float estimated_yaw   = 0.0f;

    // ------- BMP390 members -------
    volatile bool bmp_present = false;
    volatile bool bmp_pressure_valid = false;
    volatile bool bmp_temp_valid = false;

    // Latest sensor values (protected via brief critical sections)
    float latest_pressure_pa = 0.0f;
    float latest_temperature_c = 0.0f;
    float latest_altitude_m = 0.0f;
    uint32_t last_bmp_sample_ms = 0;

    // Filtering & watchdog
    float pressure_filter_alpha = 0.35f; // smoothing factor (0..1), higher => more responsive
    float filtered_pressure_pa = 0.0f;
    uint32_t bmp_stale_ms = 250;     // considered stale after this (ms)
    uint32_t last_bmp_valid_ms = 0;

    // FIFO parsing buffer
    static const size_t MAX_FIFO_FRAMES = 32;
    bmp390_fifo_data_t fifo_frames[MAX_FIFO_FRAMES];

    bmp390_fifo_data_t fifo_data[BMP390_FIFO_BUFFER_SIZE];
    uint16_t frames_available = 0;

    // internal helpers
    void processBMP390Fifo();
    void handleBMP390Watchdog();
    void updateFilteredPressure(float new_p);

 

    // Altitude fusion state
  
    float vel_z   = 0.0f;



    
};
