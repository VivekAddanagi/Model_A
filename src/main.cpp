#include <Arduino.h>
#include "ComManager.h"
#include "SensorManager.h"

ComManager com;
SensorManager sensors;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(5000); // Allow time for Serial to initialize
    
    // Initialize both systems


   if(!com.begin()) {  // Modify your ComManager::begin() to return bool
        Serial.println("[ERROR] Failed to initialize radio!");
     while(1); // Halt if radio fails
    }

    if(!sensors.begin()) {
        Serial.println("[ERROR] Failed to initialize sensors!");
        while(1); // Halt if sensors fail
    }
    
    
    
    Serial.println("[SYSTEM] Initialization complete");
}

void loop() {
    // Update both systems
    sensors.update();
    com.update();

    // Handle incoming radio data
    if (com.hasNewData()) {
        Serial.printf("[RADIO] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d\n",
                     com.yaw, com.pitch, com.roll, com.throttle, com.mode);
    }

    // Print sensor data periodically
    static uint32_t lastSensorPrint = 0;
    if(millis() - lastSensorPrint > 100) { // 10Hz update
        sensors.printBMI323Data();
        lastSensorPrint = millis();
    }

    // Reduced delay for better responsiveness
    delay(10);
}