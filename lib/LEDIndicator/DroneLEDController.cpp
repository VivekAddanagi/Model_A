#include "DroneLEDController.h"

DroneLEDController::DroneLEDController(uint8_t frontPin, uint8_t rearPin)
  : frontLEDPin(frontPin), rearLEDPin(rearPin), currentState(STATE_INIT), currentMode(MODE_STABLE),
    recording(false), photoFlash(false), lastUpdate(0), blinkInterval(500), ledState(false) {}

void DroneLEDController::begin() {
  pinMode(frontLEDPin, OUTPUT);
  pinMode(rearLEDPin, OUTPUT);
  digitalWrite(frontLEDPin, LOW);
  digitalWrite(rearLEDPin, LOW);

  // Setup PWM for breathing effect (front LED only)
  ledcSetup(0, 5000, 8);         // Channel 0, 5kHz, 8-bit resolution
  ledcAttachPin(frontLEDPin, 0); // Front LED controlled by PWM
}


void DroneLEDController::update(DroneState state, FlightMode mode, bool isRecording, bool photoTaken) {
  currentState = state;
  currentMode = mode;
  recording = isRecording;

  if (photoTaken) {
    photoFlash = true;
  }

  photoFlashHandler();
  applyStateBehavior();
}

void DroneLEDController::photoFlashHandler() {
  static unsigned long photoStart = 0;
  if (photoFlash) {
    digitalWrite(frontLEDPin, HIGH);
    photoStart = millis();
    photoFlash = false;
  }

  if (millis() - photoStart < 100) return;
  else digitalWrite(frontLEDPin, LOW);
}

void DroneLEDController::applyStateBehavior() {
  unsigned long currentMillis = millis();
  static unsigned long recBlink = 0;
  static bool recLedState = false;

  // Detach PWM unless cruise
  bool isCruise = (currentState == STATE_IN_FLIGHT && currentMode == MODE_CRUISE);
  if (!isCruise) {
    ledcDetachPin(frontLEDPin);
  }

  switch (currentState) {

    case STATE_INIT:
      blinkInterval = 500;
      if (currentMillis - lastUpdate >= blinkInterval) {
        ledState = !ledState;
        setLED(frontLEDPin, ledState);
        setLED(rearLEDPin, ledState);
        lastUpdate = currentMillis;
      }
      break;

    case STATE_ARMED:
    setLED(frontLEDPin, true);
    setLED(rearLEDPin, true);
    break;

case STATE_DISARMED:
    blinkInterval = 500; // slow blink
    if (millis() - lastUpdate >= blinkInterval) {
        ledState = !ledState;
        setLED(frontLEDPin, ledState);
        setLED(rearLEDPin, LOW);
        lastUpdate = millis();
    }
    break;


    case STATE_TAKEOFF:
      blinkInterval = 200;
      if (currentMillis - lastUpdate >= blinkInterval) {
        ledState = !ledState;
        setLED(frontLEDPin, ledState);
        setLED(rearLEDPin, ledState);
        lastUpdate = currentMillis;
      }
      break;

    case STATE_IN_FLIGHT:
      switch (currentMode) {
        case MODE_STABLE:
          setLED(frontLEDPin, true);
          setLED(rearLEDPin, false);
          break;

        case MODE_HOVER:
          blinkInterval = 1000;
          if (currentMillis - lastUpdate >= blinkInterval) {
            ledState = !ledState;
            setLED(frontLEDPin, ledState);
            setLED(rearLEDPin, false);
            lastUpdate = currentMillis;
          }
          break;

        case MODE_CRUISE:
          ledcAttachPin(frontLEDPin, 0);
          breathingEffect(frontLEDPin);
          setLED(rearLEDPin, false);
          break;
      }
      break;

    case STATE_FAILSAFE:
      blinkInterval = 100;
      if (currentMillis - lastUpdate >= blinkInterval) {
        ledState = !ledState;
        setLED(frontLEDPin, false);
        setLED(rearLEDPin, ledState);
        lastUpdate = currentMillis;
      }
      break;
  }

  // 📸 PHOTO OVERRIDE (flash green LED 100ms)
  static unsigned long photoStart = 0;
  if (photoFlash) {
    setLED(frontLEDPin, true);
    photoStart = millis();
    photoFlash = false;
  }

  if (millis() - photoStart < 100) {
    setLED(frontLEDPin, true);  // keep green ON
  } else if (!isCruise && currentState != STATE_FAILSAFE) {
    // Let applyStateBehavior reset it otherwise
    if (currentState == STATE_INIT || currentState == STATE_TAKEOFF || currentMode == MODE_HOVER)
      ; // Already handled
    else if (recording && currentMode != MODE_CRUISE)
      ; // Let recording override it
    else
      setLED(frontLEDPin, false);
  }

  // 🎥 VIDEO OVERRIDE
  if (recording && currentState == STATE_IN_FLIGHT && currentMode != MODE_CRUISE) {
    digitalWrite(rearLEDPin, HIGH);  // Solid red
    if (currentMillis - recBlink >= 1000) {
      recLedState = !recLedState;
      digitalWrite(frontLEDPin, recLedState);  // Toggle green
      recBlink = currentMillis;
    }
  }
}


void DroneLEDController::setLED(uint8_t pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

// Basic breathing effect using PWM
void DroneLEDController::breathingEffect(uint8_t pin) {
  static int fadeAmount = 5;
  static unsigned long lastFade = 0;

  if (millis() - lastFade >= 20) {
    brightness += fadeAmount;
    if (brightness <= 5 || brightness >= 250) fadeAmount = -fadeAmount;

    ledcWrite(0, brightness);  // Use class member
    lastFade = millis();
    //Serial.println(brightness);

  }
}


