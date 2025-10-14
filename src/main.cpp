
#include <Arduino.h>
#include "bmi323.h"
#include "bmp390.h"
#include "ComManager.h"
#include "SensorManager.h"
#include "FlightController.h"
#include "DroneLEDController.h"
#include "IRSensor.h"
#include "GeofenceRSSI.h"
#include "TelemetryTx.h"

IRSensor irSensor;
GeofenceRSSI geofence; // default-constructed

// 🟢 Front LED on GPIO 5, 🔴 Rear LED on GPIO 43
DroneLEDController ledController(5, 43);

// Drone state tracking
DroneState currentState = STATE_INIT;
FlightMode currentMode = MODE_STABLE;
bool recording = false;
bool photoFlash = false;

// === Global Managers ===
ComManager comManager;
SensorManager sensorManager;
FlightController flightController(&sensorManager, &comManager);
TelemetryTx telemetry(&comManager, &sensorManager, &flightController, &irSensor);


// Prototypes
FlightMode select_mode();
void print_mode_configuration(FlightMode mode);

void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0) < 200) {
        delay(10); // wait up to 200 ms for serial, then continue
    }

    delay(3000);

    // 🔹 Measure setup start time
    unsigned long setup_start = millis();

    // 🔹 Step 1: Initialize LEDs and show INIT state
    ledController.begin();
    currentState = STATE_INIT;
    ledController.update(currentState, currentMode, recording, photoFlash);

    // Optional: blink for 0.5 s while waiting for mode selection
    unsigned long initStart = millis();
    while (millis() - initStart < 500) {
        ledController.update(currentState, currentMode, recording, photoFlash);
        delay(5);
    }

    // 🔹 Step 2: Initialize CC2500 receiver
    comManager.begin();
    delay(2);

   
    // 🔹 Step 5: Mode selection
    FlightMode selected = select_mode();
   // Serial.println("[DEBUG] Mode selected OK");

    // 🔹 Step 6: SensorManager handles BMI323 + BMP390 sequence
    if (!sensorManager.begin(1013.25f, selected)) {  // pass sea level pressure + mode
        Serial.println("[ERROR] SensorManager init failed!");
        return;
    }

    // 🔹 Step 7: Update LEDs with selected mode
    currentMode = selected;
    ledController.update(currentState, currentMode, recording, photoFlash);

     // 🔹 Step 3: FlightController init (motors)
   flightController.begin();

    // 🔹 Step 4: Initialize IR sensors
    //irSensor.begin();
   // Serial.println("[IR] Sensors initialized.");


    // 🔹 Step 8: Start telemetry
   // telemetry.begin();

    // 🔹 Measure and print setup time
    unsigned long setup_end = millis();
    Serial.printf("[SETUP] Completed successfully in %lu ms\n", setup_end - setup_start);

    esp_reset_reason_t reason = esp_reset_reason();
Serial.printf("[BOOT] Reset reason: %d\n", reason);

}

void loop() {

    static uint32_t last_ms = millis();
    uint32_t now = millis();
    float dt = (now - last_ms) * 0.001f;
    last_ms = now;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.01f; // safe fallback

    // --- Timing variables ---
    uint32_t t_start = micros();
    uint32_t t_sensors, t_com, t_fc;

    // Update RC inputs
    comManager.update();
    t_com = micros();

    // Update sensors + EKF
    sensorManager.update();
    t_sensors = micros();

    // Map RC input to flight controller setpoints if new data received
    if (comManager.hasNewData()) {
        flightController.roll_set  = map(comManager.roll,   -100, 100, -30, 30); // degrees
        flightController.pitch_set = map(comManager.pitch,  -100, 100, -30, 30); // degrees
        flightController.yaw_set   = map(comManager.yaw,    -100, 100, -90, 90); // degrees
        flightController.alt_set   = map(comManager.throttle, 0, 255, 0, 10);   // meters
    }

    // enforce geofence — reads comManager RSSI and sensor altitude, may modify setpoints
    // geofence.update(&comManager, &sensorManager, &flightController);

    // Update flight controller with dt
    flightController.update(dt);
  //  t_fc = micros();

   // telemetry.update(); // send telemetry periodically

    // --- Calculate timings ---
    uint32_t dur_com     = t_com - t_start;
    uint32_t dur_sensors = t_sensors - t_com;
    uint32_t dur_fc      = t_fc - t_com;
    uint32_t dur_total   = t_fc - t_start;

    // Print every 1 s to avoid spamming serial
   // static uint32_t last_dbg = millis();
   // if (millis() - last_dbg > 1000) {
       // Serial.printf("[TIME] Sensors: %lu us | Com: %lu us | FC: %lu us | Total: %lu us\n",
                    //  dur_sensors, dur_com, dur_fc, dur_total);
       // last_dbg = millis();
   // }

    delay(6); // maintain sensor update rate
}



/*


#include <Arduino.h>
#include <driver/mcpwm.h>

#define MOTOR_PIN 2
#define PWM_FREQ 20000
#define PWM_RES 100.0f
#define THR_MIN 1000.0f
#define THR_MAX 2200.0f   // allow slight overdrive

float throttle_us = THR_MIN;
bool atMax = false;

static inline float usToDuty(float us) {
  us = constrain(us, THR_MIN, THR_MAX);
  // Map 1000..2200 -> 0..100%
  return (us - THR_MIN) * PWM_RES / (THR_MAX - THR_MIN);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== OVERDRIVE RAMP TEST (up to 110%) ===");

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PIN);
  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQ;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void loop() {
  if (!atMax) {
    throttle_us += 20;        // ramp step
    if (throttle_us >= THR_MAX) {
      throttle_us = THR_MAX;
      atMax = true;
      Serial.println("Reached overdrive max (110%) — HOLDING. Stop if temperature/current high.");
    }
    float duty = usToDuty(throttle_us);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    Serial.printf("Throttle: %.0f us -> Duty: %.1f%%\n", throttle_us, duty);
    delay(50);
  }
}

*/