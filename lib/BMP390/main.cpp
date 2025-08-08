/*
#include <Arduino.h>
#include "bmp390.h"

bmp390_fifo_data_t fifo_data[FIFO_BUFFER_SIZE];
uint16_t frames_available = 0;

void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== BMP390 System Start ===");

    if (bmp390_init_all() != 0) {
        Serial.println("[ERROR] BMP390 Init failed.");
        return;
    }

    // === Mode Selection & Configuration ===
    bmp390_mode_t selected_mode = get_user_selected_mode();
    BMP390_set_flight_mode(selected_mode);
    BMP390_print_raw_before_fifo();

    // === Calibration ===
    char input;
    Serial.print("Use existing calibration? (y/n): ");
    while (!Serial.available()) delay(10);
    input = Serial.read();

    if (input == 'y' || input == 'Y') {
        if (bmp390_apply_calibration() != 0) {
            Serial.println("[INFO] Recalibrating...");
            bmp390_calibrate_offset();
        }
    } else {
        bmp390_calibrate_offset();
    }

    // === FIFO Initialization ===
    if (bmp390_fifo_init() != 0 || bmp390_start_fifo_continuous_mode(true, true) != 0) {
        Serial.println("[ERROR] FIFO init failed.");
        return;
    }

   
    // ✅ Let FIFO fill with data after enabling
    //Serial.println("[INFO] Waiting 1000ms for FIFO to accumulate frames...");
    delay(1000);
}

void loop() {
    if (fifo_data_ready) {
        fifo_data_ready = false;

        //Serial.println("[LOOP] FIFO interrupt received");

        // Optional: check FIFO overflow
        if (bmp390_check_fifo_overflow() > 0) {
            Serial.println("[WARN] FIFO Overflow detected");
        }

        // Read FIFO frames
        int ret = bmp390_read_fifo_data(fifo_data, FIFO_BUFFER_SIZE, &frames_available);
       // Serial.printf("[DEBUG] bmp390_read_fifo_data returned: %d | Frames Available: %d\n", ret, frames_available);

        if (ret != 0) {
            Serial.println("[ERROR] Failed to read FIFO data");
            return;
        }

        // Print valid data
       float sum_p = 0, sum_t = 0;
int count = 0;

for (int i = 0; i < frames_available; ++i) {
    if (fifo_data[i].pressure_valid && fifo_data[i].temperature_valid) {
        sum_p += fifo_data[i].pressure;
        sum_t += fifo_data[i].temperature;
        count++;
    }
}

if (count > 0) {
    float avg_p = sum_p / count;
    float avg_t = sum_t / count;
    float alt_ground = bmp390_altitude_from_ground(avg_p, bmp390_get_ground_pressure());
    float alt_sea = bmp390_calculate_altitude(avg_p);
    unsigned long now = millis();

    Serial.printf("[%lu ms] [AVG] P=%.2f Pa | T=%.2f °C | AltG=%.2f m | AltS=%.2f m\n",
        now, avg_p, avg_t, alt_ground, alt_sea);
}

    }

    // Small delay to prevent spamming if no interrupt received
    delay(100);
}

*/