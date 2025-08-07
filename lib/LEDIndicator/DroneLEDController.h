#ifndef DRONE_LED_CONTROLLER_H
#define DRONE_LED_CONTROLLER_H

#include <Arduino.h>

// Drone States
enum DroneState {
  STATE_INIT,
  STATE_ARMED,
  STATE_TAKEOFF,
  STATE_IN_FLIGHT,
  STATE_FAILSAFE
};

// Flight Modes
enum FlightMode {
  MODE_STABLE,
  MODE_HOVER,
  MODE_CRUISE
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
