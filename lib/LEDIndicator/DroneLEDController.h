#ifndef DRONE_LED_CONTROLLER_H
#define DRONE_LED_CONTROLLER_H

#include <Arduino.h>
#include "bmi323.h"   // ✅ Use the FlightMode enum already defined here
#include "bmp390.h"   // ✅ Use bmp390_profile_t if needed

// Drone States
enum DroneState {
  STATE_INIT,
  STATE_ARMED,
  STATE_TAKEOFF,
  STATE_IN_FLIGHT,
  STATE_FAILSAFE,
  STATE_DISARMED   // <-- add this
};



class DroneLEDController {
public:
  DroneLEDController(uint8_t frontPin, uint8_t rearPin);

  void begin();
  void update(DroneState state, FlightMode mode, bool isRecording, bool photoTaken);

private:
  uint8_t frontLEDPin;
  uint8_t rearLEDPin;
  uint8_t brightness = 0;

  DroneState currentState;
  FlightMode currentMode;
  bool recording;
  bool photoFlash;

  unsigned long lastUpdate;
  unsigned long blinkInterval;
  bool ledState;

  void applyStateBehavior();
  void setLED(uint8_t pin, bool state);
  void breathingEffect(uint8_t pin);
  void photoFlashHandler();
};

#endif
