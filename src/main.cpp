#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"

// === Global FIFO buffer for BMP390 ===
bmp390_fifo_data_t fifo_data[FIFO_BUFFER_SIZE];
uint16_t frames_available = 0;

// If your bmp390.h exposes this flag, we use it.
// (Your existing code already referenced it without local definition)
extern volatile bool fifo_data_ready;

// === Global ComManager instance ===
ComManager comManager;

// === Function prototypes === 
FlightMode select_mode(); 
void print_mode_configuration(FlightMode mode); 
void apply_bmi323_mode(FlightMode mode); 
void apply_bmp390_mode(FlightMode mode); 
// Forward declare the manager (from calibration_sequence.cpp) 
void run_calibration_sequence_startup();

// Optional: If your project defines BMP390_INT_PIN, attach the FIFO ISR here
#ifdef BMP390_INT_PIN
void IRAM_ATTR bmp390_fifo_isr() {
    fifo_data_ready = true;
}
#endif

// === Setup ===
void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(3000);

    // Start the communication manager (CC2500 receiver)
    comManager.begin();

    // Initialize sensors
    bmp390_init_all();
    bmi323_init();

    Serial.println(F("\nDrone Mode Selector Starting..."));
    FlightMode selected = select_mode();

    // Apply selected modes
    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);

    // Calibrate (your unified flow)
    run_calibration_sequence_startup();

    // Initialize BMI323 FIFO
    bmi323_setup_fifo();

    // Initialize BMP390 FIFO for continuous pressure + temperature
    if (bmp390_fifo_init() != 0 || bmp390_start_fifo_continuous_mode(true, true) != 0) {
        Serial.println("[ERROR] BMP390 FIFO init failed.");
    }

    // Attach BMP390 FIFO interrupt if pin is available in your build
    #ifdef BMP390_INT_PIN
    pinMode(BMP390_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BMP390_INT_PIN), bmp390_fifo_isr, RISING);
    #endif

    // Let BMP390 FIFO accumulate some frames
    delay(1000);

    Serial.println(F("Flight mode configuration applied."));
}

// === Loop ===
void loop() {
    // --- BMI323 continuous reading ---
    bmi323_read_fifo();

    // --- BMP390 continuous FIFO reading ---
    // Prefer interrupt if available; also include a polling fallback so we don't get stuck
    static unsigned long last_bmp390_read_ms = 0;
    const unsigned long BMP390_READ_INTERVAL_MS = 20; // ~50 Hz

    bool do_bmp390_read = false;

    // If ISR flagged ready, read now
    if (fifo_data_ready) {
        do_bmp390_read = true;
    }
    // Otherwise, poll at a safe rate to avoid missing data when ISR isn't attached
    else if (millis() - last_bmp390_read_ms >= BMP390_READ_INTERVAL_MS) {
        do_bmp390_read = true;
    }

    if (do_bmp390_read) {
        last_bmp390_read_ms = millis();
        fifo_data_ready = false; // clear flag if it was set

        // Optional: check FIFO overflow
        if (bmp390_check_fifo_overflow() > 0) {
            Serial.println("[WARN] BMP390 FIFO Overflow detected");
        }

        int ret = bmp390_read_fifo_data(fifo_data, FIFO_BUFFER_SIZE, &frames_available);
        if (ret != 0) {
            Serial.println("[ERROR] BMP390 FIFO read failed");
        } else if (frames_available > 0) {
            float sum_p = 0.0f, sum_t = 0.0f;
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

                Serial.printf(
                    "[%lu ms] BMP390 AVG: P=%.2f Pa | T=%.2f °C | AltG=%.2f m | AltS=%.2f m\n",
                    now, avg_p, avg_t, alt_ground, alt_sea
                );
            }
        }
    }

    // --- Update CC2500 telemetry ---
    comManager.update();

    // --- Prevent SPI starvation ---
    delay(1);
}
