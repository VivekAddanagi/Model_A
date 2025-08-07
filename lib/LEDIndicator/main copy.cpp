#include <Arduino.h>
#include "DroneLEDController.h"

// 🟢 Front LED on GPIO 5, 🔴 Rear LED on GPIO 43 
DroneLEDController ledController(5, 43);

DroneState currentState = STATE_INIT;
FlightMode currentMode = MODE_STABLE;
bool recording = false;
bool photoFlash = false;

void setup() {
  Serial.begin(115200);
  delay(2000);  // Allow Serial to settle

  ledController.begin();

  Serial.println("Drone LED Simulator Ready. Press keys:");
  Serial.println("[i] Init   [a] Armed   [t] Takeoff   [f] In-Flight   [e] Failsafe");
  Serial.println("[1] Stable Mode   [2] Hover   [3] Cruise");
  Serial.println("[p] Photo Flash   [v] Toggle Video");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();  // Read single character
    Serial.print("Received: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'i':
        currentState = STATE_INIT;
        Serial.println("🟡 State → INIT (Both LEDs blinking slowly @ 500ms)");
        break;

      case 'a':
        currentState = STATE_ARMED;
        Serial.println("🔓 State → ARMED (Both LEDs solid ON)");
        break;

      case 't':
        currentState = STATE_TAKEOFF;
        Serial.println("✈️ State → TAKEOFF (Both LEDs blinking fast @ 200ms)");
        break;

      case 'f':
        currentState = STATE_IN_FLIGHT;
        Serial.println("🛫 State → IN_FLIGHT (Use 1:Stable, 2:Hover, 3:Cruise)");
        break;

      case 'e':
        currentState = STATE_FAILSAFE;
        Serial.println("🚨 State → FAILSAFE (Red LED fast blinking @ 100ms)");
        break;

      case '1':
        currentMode = MODE_STABLE;
        Serial.println("🟢 Mode → STABLE (Green ON, Red OFF)");
        break;

      case '2':
        currentMode = MODE_HOVER;
        Serial.println("🔵 Mode → HOVER (Green blinking 1s, Red OFF)");
        break;

      case '3':
        currentMode = MODE_CRUISE;
        Serial.println("🌊 Mode → CRUISE (Green PWM breathing, Red OFF)");
        break;

      case 'p':
        photoFlash = true;
        Serial.println("📸 Photo → Green flashes for 100ms");
        break;

      case 'v':
        recording = !recording;
        if (recording) {
          Serial.println("🎥 Video Recording STARTED:");
          if (currentMode == MODE_CRUISE)
            Serial.println("    🔴 Red: ON   🟢 Green: Breathing (PWM)");
          else
            Serial.println("    🔴 Red: ON   🟢 Green: Slow blink (1s)");
        } else {
          Serial.println("🎥 Video Recording STOPPED:");
          Serial.println("    LEDs return to previous state behavior");
        }
        break;

      default:
        Serial.println("⚠️  Unknown command. Valid: i a t f e 1 2 3 p v");
    }
  }

  ledController.update(currentState, currentMode, recording, photoFlash);
  photoFlash = false;
}


