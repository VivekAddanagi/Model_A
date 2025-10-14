#include "FlightController.h"
#include <driver/mcpwm.h>
#include "soc/mcpwm_periph.h"
#include "DroneLEDController.h"
#include "IRSensor.h"


// FlightController.cpp
extern DroneState currentState;
extern FlightMode currentMode;
extern bool recording;
extern bool photoFlash;
extern DroneLEDController ledController;
extern IRSensor irSensor;  // declare external reference (defined in main.cpp)



float FlightController::PID_Update(PID &pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // --- Integral with clamping ---
    pid.integral += error * dt;
#ifdef PID_INTEGRAL_LIMITS
    pid.integral = constrain(pid.integral, pid.integral_min, pid.integral_max);
#endif

    // --- Derivative with guard ---
    float derivative = 0.0f;
    if (dt > 1e-6f) {
        derivative = (error - pid.prev_error) / dt;
    }
    pid.prev_error = error;

    float out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    float constrained_out = constrain(out, pid.output_min, pid.output_max);

    // Debug PID
    static unsigned long last_pid_dbg = 0;
    if (millis() - last_pid_dbg > 200) {
       // Serial.printf("[PID] SP: %.2f, Meas: %.2f, Err: %.2f, Out: %.2f\n",
                    //  setpoint, measured, error, constrained_out);
       // last_pid_dbg = millis();
    }

    return constrained_out;
}




// ===================== MCPWM CONFIG =====================
#define PWM_FREQ 20000  // 20 kHz for brushed motors
#define PWM_RES  100.0f // duty is 0..100%

// Each motor uses a different operator on the same timer
// M1 -> MCPWM0A, M2 -> MCPWM0B, M3 -> MCPWM1A, M4 -> MCPWM1B

// Helper: convert µs (1000–2000) to duty %
static inline float usToDuty(float us) {
    us = constrain(us, 1000.0f, 2000.0f);
    return (us - 1000.0f) * PWM_RES / 1000.0f;  // 1000 → 0%, 2000 → 100%
}

// -------------------- Write PWM to motors --------------------
void FlightController::writeMotors(float m1, float m2, float m3, float m4) {
    
    float d1 = usToDuty(m1);
    float d2 = usToDuty(m2);
    float d3 = usToDuty(m3);
    float d4 = usToDuty(m4);

    // Apply min spin floor if armed
    const float min_spin_pct = (com->armed && !com->failsafe && !com->takeoff) ? 5.0f : 0.0f;
    d1 = max(d1, min_spin_pct);
    d2 = max(d2, min_spin_pct);
    d3 = max(d3, min_spin_pct);
    d4 = max(d4, min_spin_pct);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, d1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, d2);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, d3);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, d4);

    _lastMotorDuty[0] = d1;
_lastMotorDuty[1] = d2;
_lastMotorDuty[2] = d3;
_lastMotorDuty[3] = d4;


    static unsigned long last_motor_dbg = 0;
    if (millis() - last_motor_dbg > 600) {
        Serial.printf("[MOTORS] M1: %.1f%% | M2: %.1f%% | M3: %.1f%% | M4: %.1f%%\n", d1, d2, d3, d4);
        last_motor_dbg = millis();
    }   
}

// -------------------- Initialize motors --------------------
void FlightController::begin() {
    // GPIO init
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_PIN_1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_PIN_2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR_PIN_3);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MOTOR_PIN_4);

    // Configure PWM (same for both timers)
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQ;
    pwm_config.cmpr_a = 0;  // duty cycle %
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    // Idle at safe low
    writeMotors(1000, 1000, 1000, 1000);
    // Debug pin to scope LED updates
    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);
    Serial.println("[FC] Motors initialized on MCPWM @ 20kHz.");
}

void FlightController::update(float dt) {

    // --- Sensor readings ---
    float roll  = estimated_roll;
    float pitch = estimated_pitch;
    float yaw   = estimated_yaw;
    float alt   = sensor->alt_est;

    // --- Enforce Altitude Fence ---
    const float ALT_MAX = 5.0f;  // Maximum allowed altitude in meters
    if (alt > ALT_MAX) {
        Serial.println("[FC] Altitude fence active! Preventing climb above 5 m.");
        alt_set = min(alt_set, ALT_MAX);
    }

    // --- PID outputs ---
    float roll_cmd  = PID_Update(pid_roll,  roll_set,  roll,  dt);
    float pitch_cmd = PID_Update(pid_pitch, pitch_set, pitch, dt);
    float yaw_cmd   = PID_Update(pid_yaw,   yaw_set,   yaw,   dt);
    float alt_cmd   = PID_Update(pid_alt,   alt_set,   alt,   dt);

    // Poll IR sensors regularly
   // irSensor.poll();

   /*

    if (irSensor.isNear(FRONT, irSensor.getThreshold())) {
        Serial.println("[AVOID] Obstacle ahead! Limiting forward pitch.");
        pitch_cmd = min(pitch_cmd, 0.0f);
    }

    if (irSensor.isNear(LEFT, irSensor.getThreshold())) {
        Serial.println("[AVOID] Obstacle left! Nudging right.");
        roll_cmd = abs(roll_cmd);
    }

    if (irSensor.isNear(RIGHT, irSensor.getThreshold())) {
        Serial.println("[AVOID] Obstacle right! Nudging left.");
        roll_cmd = -abs(roll_cmd);
    }

    if (irSensor.isNear(BACK, irSensor.getThreshold())) {
        Serial.println("[AVOID] Obstacle behind! Blocking reverse motion.");
        pitch_cmd = max(pitch_cmd, 0.0f);
    }
     
    */
   
    // --- Motor mixing ---
  auto mixMotors = [&](float base) {
    float M1 = constrain(base + pitch_cmd + roll_cmd - yaw_cmd, 1000.0f, 2000.0f);
    float M2 = constrain(base + pitch_cmd - roll_cmd + yaw_cmd, 1000.0f, 2000.0f);
    float M3 = constrain(base - pitch_cmd - roll_cmd - yaw_cmd, 1000.0f, 2000.0f);
    float M4 = constrain(base - pitch_cmd + roll_cmd + yaw_cmd, 1000.0f, 2000.0f);
   // Serial.printf("[MIX] Base=%.1f | M1=%.1f | M2=%.1f | M3=%.1f | M4=%.1f\n",
             // base, M1, M2, M3, M4);

    writeMotors(M1, M2, M3, M4);
};




    static float throttleOverride = 1000;
    static uint32_t landedTimer   = 0;

    if (com->failsafe) {
        currentState = STATE_FAILSAFE;

        if (alt > 0.2f) {
            throttleOverride = max(1100.0f, throttleOverride - 0.5f);
            landedTimer = millis();
        } else {
            if (millis() - landedTimer > 2000) {
                throttleOverride = 1000;
                com->armed = false;
                lockoutActive = true;
                Serial.println("[FC] FAILSAFE: Landed, auto-disarmed, lockout enabled.");
                writeMotors(1000, 1000, 1000, 1000);
                digitalWrite(7, HIGH);
                ledController.update(currentState, currentMode, recording, photoFlash);
                digitalWrite(7, LOW);
                photoFlash = false;
                return;
            }
        }

        float roll_cmd_fs  = 0.0f;
        float pitch_cmd_fs = 0.0f;

        auto adaptiveTilt = [&](Direction dir, int8_t sign) {
            float dist = irSensor.getDistance(dir);
            float threshold = irSensor.getThreshold();
            
            if (dist < threshold) {
                float strength = map(dist, threshold, 0, 0, 100);
                return sign * constrain(strength, 20.0f, 100.0f);
            }
            return 0.0f;
        };

        pitch_cmd_fs += adaptiveTilt(FRONT, -1);
        pitch_cmd_fs += adaptiveTilt(BACK,  1);
        roll_cmd_fs  += adaptiveTilt(LEFT,  1);
        roll_cmd_fs  += adaptiveTilt(RIGHT, -1);

        auto mixMotorsFailsafe = [&](float base) {
            float M1 = base + pitch_cmd_fs + roll_cmd_fs;
            float M2 = base + pitch_cmd_fs - roll_cmd_fs;
            float M3 = base - pitch_cmd_fs - roll_cmd_fs;
            float M4 = base - pitch_cmd_fs + roll_cmd_fs;
            writeMotors(M1, M2, M3, M4);
        };

        mixMotorsFailsafe(throttleOverride);

        Serial.printf("[FC] FAILSAFE DESCENT: Alt=%.2f m, Thr=%.0f, RollCmd=%.1f, PitchCmd=%.1f\n",
                      alt, throttleOverride, roll_cmd_fs, pitch_cmd_fs);

        digitalWrite(7, HIGH);
        ledController.update(currentState, currentMode, recording, photoFlash);
        digitalWrite(7, LOW);
        photoFlash = false;
        return;
    }

    // --- OBSTACLE LOCKOUT SAFETY ---
    bool obstacleBlocking = (
        irSensor.isNear(FRONT, irSensor.getThreshold()) ||
        irSensor.isNear(LEFT,  irSensor.getThreshold()) ||
        irSensor.isNear(RIGHT, irSensor.getThreshold()) ||
        irSensor.isNear(BACK,  irSensor.getThreshold())
    );

    if (obstacleBlocking) {
        if (!com->armed) {
            Serial.println("[SAFETY] Obstacle detected! ARM blocked.");
            com->armed = false;
            currentState = STATE_OBSTACLE_BLOCK;
            digitalWrite(7, HIGH);
            ledController.update(currentState, currentMode, recording, photoFlash);
            digitalWrite(7, LOW);
            photoFlash = false;
            return;
        } else if (com->armed && com->throttle < 10) {
            Serial.println("[SAFETY] Obstacle detected! TAKEOFF blocked.");
            writeMotors(1000, 1000, 1000, 1000);
            currentState = STATE_OBSTACLE_BLOCK;
            digitalWrite(7, HIGH);
            ledController.update(currentState, currentMode, recording, photoFlash);
            digitalWrite(7, LOW);
            photoFlash = false;
            return;
        }
    }

    // --- DISARMED ---
    if (!com->armed) {
        currentState = STATE_DISARMED;
        throttleOverride = 1000;
        writeMotors(1000, 1000, 1000, 1000);

        if (lockoutActive) {
            bool armGesture = (com->throttle < 10 && com->yaw > 100 && com->pitch < -100 && com->roll < -100);
            if (armGesture) {
                lockoutActive = false;
                Serial.println("[FC] Lockout cleared. Stick gesture detected. Ready for ARM.");
            }
        }

        digitalWrite(7, HIGH);
        ledController.update(currentState, currentMode, recording, photoFlash);
        digitalWrite(7, LOW);
        photoFlash = false;
        return;
    }

    /*
    //add after gesture arm enabled 
    // --- Prevent ARM during lockout ---
    if (lockoutActive && com->armed) {
        currentState = STATE_ARMED;
        com->armed = false;
        writeMotors(1000, 1000, 1000, 1000);
        Serial.println("[FC] ARM blocked due to LOCKOUT.");
        digitalWrite(7, HIGH);
        ledController.update(currentState, currentMode, recording, photoFlash);
        digitalWrite(7, LOW);
        photoFlash = false;
        return;
    }

    */

    // --- NORMAL FLIGHT ---
    float base_throttle = map(com->throttle, 0, 255, 1000, 2000);
    if (base_throttle < 1100) base_throttle = 1100;

   // Serial.printf("[THROTTLE DEBUG] Raw=%d | Mapped=%.1f\n", com->throttle, base_throttle);


    mixMotors(base_throttle);

    if (com->armed && !com->failsafe && com->throttle > 0) {
        currentState = STATE_IN_FLIGHT;
    } else if (com->armed) {
        currentState = STATE_ARMED;
    }

    digitalWrite(7, HIGH);
    ledController.update(currentState, currentMode, recording, photoFlash);
    digitalWrite(7, LOW);
    photoFlash = false;

    static unsigned long last_status_dbg = 0;
    if (millis() - last_status_dbg > 1000) {
        Serial.printf("[FC STATUS] ARM=%d, FS=%d, Alt=%.2f m, Thr=%d, State=%d\n",
                      com->armed, com->failsafe, alt, com->throttle, currentState);
        last_status_dbg = millis();
    }
}






// TODO: In failsafe descent, add horizontal drift compensation from IMU (gyro/accel) 
//       to ensure drone doesn’t tilt too much from avoidance push and topple.

// TODO: Add adaptive throttle reduction rate in failsafe 
//       (faster descent if high altitude, slower near ground).

// TODO: Consider adding obstacle height estimation 
//       (use multiple IR sensors angled down + barometer) 
//       so drone can avoid ground obstacles during descent, not just side walls.

// TODO: Add configurable safety margins (user can set IR sensor threshold 
//       depending on indoor/outdoor flying space).

// TODO: Expand stick gesture system (e.g., different gestures for ARM clear, 
//       emergency motor cut, mode switching) for more pilot control.

// TODO: Add "soft takeoff ramp" (gradual motor throttle ramp on takeoff) 
//       to prevent sudden jerk when arming with throttle > 0.

// TODO: Add watchdog for sensor polling (if IRSensor.poll() or IMU fails to update, 
//       enter failsafe immediately).

// TODO: Improve obstacle avoidance logic: instead of blocking/forcing tilt instantly, 
//       use smoothing filter (low-pass) to avoid sudden jerks near walls.

// TODO: Add non-linear adaptive tilt response for failsafe avoidance 
//       (gentle far from obstacle, exponential stronger response when very close).

// TODO: Save failsafe events into flight log (blackbox or SD card) 
//       for post-flight debugging and tuning.

// TODO: Allow failsafe recovery if RC signal restored mid-flight 
//       (but only if altitude > safe threshold and no nearby obstacles).

// TODO: Add battery failsafe (low voltage auto-land) 
//       integrated with existing failsafe descent logic.

// TODO: Add motor fault detection (if one motor output differs too much, 
//       trigger controlled shutdown or auto-land).

// TODO : MAX TILT LIMITS BASED ON TESTING ,Throttle smoothing for safety ,battery voltage monitoring ,Failsafe recovery logic ,Data logging for analysis














/* Further correction recommendations
 Timer usage awareness

You are using:

M1/M2 → MCPWM_TIMER_0

M3/M4 → MCPWM_TIMER_1

That’s fine — they will run at the same frequency (20 kHz) but each pair shares a timer → so duty updates are simultaneous within a pair, not across all four.

No immediate bug, just good to document:

// NOTE: Motors on same timer update simultaneously (M1+M2, M3+M4).
// Acceptable for quad, but not fully independent channels.


If you prefer motors to stay idle until takeoff, keep ARM=1 + THR=0 as you have; your code is already holding 1000 µs.

If you want a tiny “armed idle spin,” set a floor (e.g., min_spin = 1050) only when armed and not in failsafe/takeoff.

Consider a stick deadband (e.g., ±3) on estimated angles to quiet small PID outputs in logs.

Keep an eye on actual hover throttle and tune your takeoff ramp cap (e.g., ~1400–1500) accordingly.
*/