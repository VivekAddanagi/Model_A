#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"
#include "SensorManager.h"

// === Global FIFO buffer for BMP390 ===
bmp390_fifo_data_t fifo_data[BMP390_FIFO_BUFFER_SIZE];
uint16_t frames_available = 0;

// If your bmp390.h exposes this flag, we use it.
// (Your existing code already referenced it without local definition)
extern volatile bool fifo_data_ready;



// === Global ComManager instance ===
ComManager comManager;
SensorManager sensorManager;

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
   Serial.println("[DEBUG] Calling select_mode...");
   FlightMode selected = select_mode();
   Serial.println("[DEBUG] Mode selected OK");

    // Apply selected modes
    apply_bmi323_mode(selected);
    apply_bmp390_mode(selected);
    print_mode_configuration(selected);
    // Calibrate (your unified flow)
    run_calibration_sequence_startup();
    
    // Initialize BMI323 FIFO
    bmi323_setup_fifo();


    // Initialize BMP390 FIFO for continuous pressure + temperature
    if (bmp390_fifo_init() != 0 || bmp390_start_fifo_continuous_mode(true, true) != 0) {
        Serial.println("[ERROR] BMP390 FIFO init failed.");
    }
    delay(20);  // wait for first sample(s) to accumulate
     
    // Attach BMP390 FIFO interrupt if pin is available in your build
    #ifdef BMP390_INT_PIN
    pinMode(BMP390_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BMP390_INT_PIN), bmp390_fifo_isr, RISING);
    #endif

    // Let BMP390 FIFO accumulate some frames
    delay(1000);

    Serial.println(F("Flight mode configuration applied."));
}

void loop() {
         // poll FIFO every loop
   sensorManager.update(); // Handle BMP390 FIFO
    //comManager.update();    // Update CC2500 telemetry
    delay(2);               // Prevent SPI starvation
}
