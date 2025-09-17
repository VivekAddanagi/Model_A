#include "DroneLEDController.h"

DroneLEDController::DroneLEDController(uint8_t frontPin, uint8_t rearPin)
  : frontLEDPin(frontPin),
    rearLEDPin(rearPin),
    currentState(STATE_INIT),
    currentMode(MODE_STABLE),
    recording(false),
    photoFlash(false),
    lastUpdate(0),
    blinkInterval(500),
    ledState(false),
    pwmActive(false),     // init to false
    recLedState(false),   // init to false
    brightness(0)         // start brightness at 0
{}


void DroneLEDController::begin() {
  pinMode(frontLEDPin, OUTPUT);
  pinMode(rearLEDPin, OUTPUT);
  digitalWrite(frontLEDPin, LOW);
  digitalWrite(rearLEDPin, LOW);

  // Setup PWM for breathing effect (front LED only) using a dedicated channel
  constexpr uint32_t freq = 5000;
  constexpr uint8_t resolution_bits = 8;
  ledcSetup(LEDC_CHANNEL, freq, resolution_bits);         // use LEDC_CHANNEL (4)
  // Attach but don't consider pwm active yet - we'll attach when entering cruise
  // If you prefer it attached immediately uncomment the next line:
  // ledcAttachPin(frontLEDPin, LEDC_CHANNEL);

  pwmActive = false;
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
    photoFlash = false; // consume the trigger
  }

  if (photoStart != 0 && millis() - photoStart >= 100) {
    digitalWrite(frontLEDPin, LOW);
    photoStart = 0; // reset
  }
}

void DroneLEDController::applyStateBehavior() {
    unsigned long currentMillis = millis();
    static unsigned long recBlink = 0;
    static unsigned long lastBlinkUpdate = 0;
    static bool blinkLedState = false;

    bool isCruise = (currentState == STATE_IN_FLIGHT && currentMode == MODE_CRUISE);

    // Attach/detach LEDC pin when entering/leaving cruise mode
    if (isCruise && !pwmActive) {
        ledcAttachPin(frontLEDPin, LEDC_CHANNEL);
        pwmActive = true;
    } else if (!isCruise && pwmActive) {
        ledcDetachPin(frontLEDPin);
        pwmActive = false;
        // ensure front LED off when detaching
        digitalWrite(frontLEDPin, LOW);
    }

    switch (currentState) {
        case STATE_INIT:
            if (currentMillis - lastBlinkUpdate >= 500) {
                blinkLedState = !blinkLedState;
                setLED(frontLEDPin, blinkLedState);
                setLED(rearLEDPin, blinkLedState);
                lastBlinkUpdate = currentMillis;
            }
            break;

        case STATE_ARMED:
            setLED(frontLEDPin, HIGH);
            setLED(rearLEDPin, HIGH);
            break;

        case STATE_DISARMED:
            if (currentMillis - lastBlinkUpdate >= 500) {
                blinkLedState = !blinkLedState;
                setLED(frontLEDPin, blinkLedState);
                setLED(rearLEDPin, LOW);
                lastBlinkUpdate = currentMillis;
            }
            break;

        case STATE_TAKEOFF:
            if (currentMillis - lastBlinkUpdate >= 200) {
                blinkLedState = !blinkLedState;
                setLED(frontLEDPin, blinkLedState);
                setLED(rearLEDPin, blinkLedState);
                lastBlinkUpdate = currentMillis;
            }
            break;

        case STATE_IN_FLIGHT:
            switch (currentMode) {
                case MODE_STABLE:
                    setLED(frontLEDPin, HIGH);
                    setLED(rearLEDPin, LOW);
                    break;

                case MODE_HOVER:
                    if (currentMillis - lastBlinkUpdate >= 1000) {
                        blinkLedState = !blinkLedState;
                        setLED(frontLEDPin, blinkLedState);
                        setLED(rearLEDPin, LOW);
                        lastBlinkUpdate = currentMillis;
                    }
                    break;

                case MODE_CRUISE:
                    breathingEffect(frontLEDPin);   // PWM breathing using LEDC_CHANNEL
                    setLED(rearLEDPin, LOW);
                    break;
            }
            break;

        case STATE_FAILSAFE:
            if (currentMillis - lastBlinkUpdate >= 100) {
                blinkLedState = !blinkLedState;
                setLED(frontLEDPin, LOW);
                setLED(rearLEDPin, blinkLedState);
                lastBlinkUpdate = currentMillis;
            }
            break;
         case STATE_OBSTACLE_BLOCK:
    if (currentMillis - lastBlinkUpdate >= 100) {  // fast blink
        blinkLedState = !blinkLedState;
        setLED(frontLEDPin, blinkLedState);
        setLED(rearLEDPin, blinkLedState);
        lastBlinkUpdate = currentMillis;
    }
    break;
   
    }

    // NOTE: photoFlashHandler() is called only from update(), not here, to avoid double-processing.

    // Recording indicator: rear LED solid + front LED blink if recording in flight (non-cruise)
    if (recording && currentState == STATE_IN_FLIGHT && currentMode != MODE_CRUISE) {
        digitalWrite(rearLEDPin, HIGH);
        if (currentMillis - recBlink >= 1000) {
            recLedState = !recLedState;
            digitalWrite(frontLEDPin, recLedState);
            recBlink = currentMillis;
        }
    }
}


void DroneLEDController::setLED(uint8_t pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
  // Debug print to help trace behavior on serial monitor
 // Serial.printf("[LED] Pin %d -> %s\n", pin, state ? "HIGH" : "LOW");
}



void DroneLEDController::breathingEffect(uint8_t pin) {
  static int fadeAmount = 5;
  static unsigned long lastFade = 0;

  if (millis() - lastFade >= 20) {
    int tmp = (int)brightness + fadeAmount;
    if (tmp <= 5 || tmp >= 250) fadeAmount = -fadeAmount;
    brightness = (uint8_t)constrain(tmp, 0, 255);

    // Write to the chosen LEDC channel
    ledcWrite(LEDC_CHANNEL, brightness);
    lastFade = millis();
  }
}


