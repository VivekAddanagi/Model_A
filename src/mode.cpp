#include <Arduino.h>
#include "bmi323.h"   // Existing BMI323 library (already has FlightMode enum)
#include "bmp390.h"   // Existing BMP390 library


// === BMI323 & BMP390 Profiles ===
// (Copied from your existing configurations, but stored here for runtime selection)
static bmp390_profile_t bmp390_profiles[] = {
    { 0x03, 0x01, 0x03, 0x03 }, // MODE_STABLE
    { 0x04, 0x01, 0x07, 0x03 }, // MODE_HOVER
    { 0x05, 0x01, 0x01, 0x03 }  // MODE_CRUISE
};

static FlightModeConfig flight_mode_configs[] = {
    { true, true, true, false, 1.2f, 1.2f, 1.0f, 0.0f }, // MODE_STABLE
    { true, true, true, true,  1.0f, 1.0f, 1.0f, 1.5f }, // MODE_HOVER
    { false, false, true, false, 0.5f, 0.5f, 0.8f, 0.0f } // MODE_CRUISE
};

// === Function to Select Mode ===
FlightMode select_mode() {
   // Serial.println(F("\nSelect Flight Mode: 1-STABLE 2-HOVER 3-CRUISE"));
   // Serial.print(F("Enter mode (1-3): "));

    unsigned long start = millis();
    while (millis() - start < 100) {  // wait 5 seconds max
        if (Serial.available()) {
            char ch = Serial.read();
           // Serial.printf("[DEBUG] Got char: %c\n", ch);
            switch (ch) {
                case '1': return MODE_STABLE;
                case '2': return MODE_HOVER;
                case '3': return MODE_CRUISE;
                default: Serial.print(F("Invalid input. Enter 1, 2, or 3: "));
            }
        }
        delay(10);
    }

    Serial.println("No input, defaulting to MODE_STABLE");
    return MODE_STABLE;
}

// === Apply BMI323 Mode ===
void apply_bmi323_mode(FlightMode mode) {
    current_mode = mode;
    current_config = &flight_mode_configs[mode];

    // Keep only configuration prints
  //  Serial.printf("\n=== Applying BMI323 Mode: %s ===\n", 
                 // mode == MODE_STABLE ? "STABLE" :
                 // mode == MODE_HOVER  ? "HOVER"  : "CRUISE");

   // Serial.printf("Pitch stabilization: %s\n", current_config->stabilize_pitch ? "ENABLED" : "DISABLED");
    //Serial.printf("Roll stabilization : %s\n", current_config->stabilize_roll  ? "ENABLED" : "DISABLED");
   // Serial.printf("Yaw stabilization  : %s\n", current_config->stabilize_yaw   ? "ENABLED" : "DISABLED");
   // Serial.printf("Altitude hold     : %s\n", current_config->hold_altitude   ? "ENABLED" : "DISABLED");

   // Serial.printf("Pitch Gain        : %.2f\n", current_config->pitch_gain);
   // Serial.printf("Roll Gain         : %.2f\n", current_config->roll_gain);
   // Serial.printf("Yaw Gain          : %.2f\n", current_config->yaw_gain);
   // Serial.printf("Altitude Gain     : %.2f\n", current_config->altitude_gain);

   // Serial.println(F("=== BMI323 Mode Applied ==="));
}

// === Apply BMP390 Config ===
void apply_bmp390_mode(FlightMode mode) {
    bmp390_profile_t profile = bmp390_profiles[mode];

    uint8_t osr = (profile.osr_t << 3) | profile.osr_p;
    uint8_t iir = (profile.iir_coeff << 1);
    uint8_t odr = profile.odr_sel;

    bmp390_write(BMP390_REG_OSR, osr);
    bmp390_write(BMP390_REG_CONFIG, iir);
    bmp390_write(BMP390_REG_ODR, odr);

    // ✅ After FIFO init, set DATA_SELECT = filtered (01b)
    uint8_t cfg2 = 0;
    bmp390_read(BMP390_FIFO_CONFIG2, &cfg2, 1);
    cfg2 &= ~(0b11 << 0);   // clear data_select bits
    cfg2 |=  (0b01 << 0);   // 01 = filtered data
    bmp390_write(BMP390_FIFO_CONFIG2, cfg2);

   // Serial.println(F("[BMP390] FIFO DATA_SELECT set to filtered data"));
}


// === Print Configuration ===
void print_mode_configuration(FlightMode mode) {
    Serial.println(F("\n--- Current Mode Configuration ---"));

    // BMP390 settings
    bmp390_profile_t bmp = bmp390_profiles[mode];
    Serial.printf("BMP390:\n  osr_p: 0x%02X\n  osr_t: 0x%02X\n  iir_coeff: 0x%02X\n  odr_sel: 0x%02X\n",
                  bmp.osr_p, bmp.osr_t, bmp.iir_coeff, bmp.odr_sel);

    // Flight mode settings
    FlightModeConfig cfg = flight_mode_configs[mode];
    Serial.println(F("FlightModeConfig:"));
    Serial.printf("  stabilize_pitch: %s\n", cfg.stabilize_pitch ? "true" : "false");
    Serial.printf("  stabilize_roll : %s\n", cfg.stabilize_roll ? "true" : "false");
    Serial.printf("  stabilize_yaw  : %s\n", cfg.stabilize_yaw ? "true" : "false");
    Serial.printf("  hold_altitude  : %s\n", cfg.hold_altitude ? "true" : "false");
    Serial.printf("  pitch_gain     : %.2f\n", cfg.pitch_gain);
    Serial.printf("  roll_gain      : %.2f\n", cfg.roll_gain);
    Serial.printf("  yaw_gain       : %.2f\n", cfg.yaw_gain);
    Serial.printf("  altitude_gain  : %.2f\n", cfg.altitude_gain);
    Serial.println(F("----------------------------------"));
}