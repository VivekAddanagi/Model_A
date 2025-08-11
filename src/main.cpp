#include <Arduino.h>
#include "ComManager.h"
#include "SensorManager.h"

ComManager com;
SensorManager sensors;

// Print interval for BMP390 telemetry
static const uint32_t BMP_PRINT_INTERVAL_MS = 200; // 5 Hz

void setup() {
    Serial.begin(DEBUG_SERIAL_BAUD);
    while (!Serial) { delay(10); }
    delay(500); // Allow time for Serial to initialize

    Serial.println("[SYSTEM] Startup...");

    // Initialize communications (radio)
    if(!com.begin()) {  // Modify your ComManager::begin() to return bool
        Serial.println("[ERROR] Failed to initialize radio!");
        while(1) { delay(100); } // Halt if radio fails
    }

    // Initialize sensors; this now also initializes BMP390 (continuous FIFO mode)
    if(!sensors.begin()) {
        Serial.println("[ERROR] Failed to initialize sensors!");
        while(1) { delay(100); } // Halt if sensors fail
    }

    Serial.println("[SYSTEM] Initialization complete");
}

void loop() {
    static uint32_t lastBmpPrint = 0;
    uint32_t now = millis();

    // Update both systems
    com.update();
    delay(10); // Small delay to avoid flooding Serial
    sensors.update();

    // Print BMI323 & BMP390 data periodically for logging and debugging
    if (now - lastBmpPrint >= BMP_PRINT_INTERVAL_MS) {
        lastBmpPrint = now;
        sensors.printBMI323Data();
        sensors.printBMP390Data();
    }

    // Small delay to avoid hammering CPU (control loops may override)
    delay(10);
}
