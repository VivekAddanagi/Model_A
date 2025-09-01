#include "FlightController.h"
#include <driver/mcpwm.h>
#include "soc/mcpwm_periph.h"

// extern ComManager comManager;
// -------------------- PID update --------------------
float FlightController::PID_Update(PID &pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid.integral += error * dt;
    float derivative = (error - pid.prev_error) / dt;
    pid.prev_error = error;

    float out = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    float constrained_out = constrain(out, pid.output_min, pid.output_max);

    // Debug PID
    static unsigned long last_pid_dbg = 0;
    if (millis() - last_pid_dbg > 200) {
        Serial.printf("[PID] SP: %.2f, Meas: %.2f, Err: %.2f, Out: %.2f\n", 
                      setpoint, measured, error, constrained_out);
        last_pid_dbg = millis();
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

    static unsigned long last_motor_dbg = 0;
    if (millis() - last_motor_dbg > 200) {
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
    Serial.println("[FC] Motors initialized on MCPWM @ 20kHz.");
}

// -------------------- Flight loop update --------------------
void FlightController::update(float dt) {
    // --- Sensor readings ---
    float roll  = estimated_roll;
    float pitch = estimated_pitch;
    float yaw   = estimated_yaw;
    float alt   = sensor->alt_est;   // estimated altitude from barometer/IMU

    // --- PID outputs ---
    float roll_cmd  = PID_Update(pid_roll,  roll_set,  roll,  dt);
    float pitch_cmd = PID_Update(pid_pitch, pitch_set, pitch, dt);
    float yaw_cmd   = PID_Update(pid_yaw,   yaw_set,   yaw,   dt);
    float alt_cmd   = PID_Update(pid_alt,   alt_set,   alt,   dt);

    // ==============================
    // Motor mixing function
    auto mixMotors = [&](float base) {
        float M1 = base + pitch_cmd + roll_cmd - yaw_cmd; // Front Left CCW
        float M2 = base + pitch_cmd - roll_cmd + yaw_cmd; // Front Right CW
        float M3 = base - pitch_cmd - roll_cmd - yaw_cmd; // Rear Right CCW
        float M4 = base - pitch_cmd + roll_cmd + yaw_cmd; // Rear Left CW
        writeMotors(M1, M2, M3, M4);
    };

    // ==============================
    // Safety Logic (ARM, FAILSAFE, TAKEOFF)
    // ==============================
    static float throttleOverride = 1000; // used for ramp control
    static uint32_t landedTimer   = 0;    // track time on ground

    // FAILSAFE DESCENT
    if (com->failsafe) {
        if (alt > 0.2f) {   // above 20 cm
            throttleOverride -= 0.5f;  // gentle descent
            if (throttleOverride < 1100) throttleOverride = 1100; // small lift
            landedTimer = millis(); // reset timer while airborne
        } else {
            if (millis() - landedTimer > 2000) { // 2 sec stable on ground
                throttleOverride = 1000;
                com->armed = false; // auto-disarm
                lockoutActive = true; // enable lockout after failsafe
                Serial.println("[FC] FAILSAFE: Landed, auto-disarmed, lockout enabled.");
                writeMotors(1000, 1000, 1000, 1000);
                return;
            }
        }

        mixMotors(throttleOverride); // stabilized descent
        Serial.printf("[FC] FAILSAFE DESCENT: Alt=%.2f m, Thr=%.0f\n", alt, throttleOverride);
        return;
    }

    // ==============================
    // DISARMED: check lockout
    // ==============================
    if (!com->armed) {
        throttleOverride = 1000;
        writeMotors(1000, 1000, 1000, 1000);

        // Unlock only if stick-arming gesture detected
        if (lockoutActive) {
            bool armGesture = (com->throttle < 10 && com->yaw > 90);  // Throttle low + yaw right
            if (armGesture) {
                lockoutActive = false;
                Serial.println("[FC] Lockout cleared. Stick gesture detected. Ready for ARM.");
            }
        }
        return;
    }

    // ==============================
    // Prevent ARM during lockout
    // ==============================
    if (lockoutActive && com->armed) {
        com->armed = false; // force disarm
        writeMotors(1000, 1000, 1000, 1000);
        Serial.println("[FC] ARM blocked due to LOCKOUT. Perform stick gesture to reset.");
        return;
    }

    // TAKEOFF ASCENT
    if (com->takeoff) {
        throttleOverride += 1.5f;
        if (throttleOverride > 1400) throttleOverride = 1400; // cap near hover
        mixMotors(throttleOverride);
        Serial.printf("[FC] TAKEOFF ASCENT: Alt=%.2f m, Thr=%.0f\n", alt, throttleOverride);
        return;
    }

   // Normal flight mode
float M1 = alt_cmd + pitch_cmd + roll_cmd - yaw_cmd;
float M2 = alt_cmd + pitch_cmd - roll_cmd + yaw_cmd;
float M3 = alt_cmd - pitch_cmd - roll_cmd - yaw_cmd;
float M4 = alt_cmd - pitch_cmd + roll_cmd + yaw_cmd;

// Constrain before sending to writeMotors()
M1 = constrain(M1, 1000.0f, 2000.0f);
M2 = constrain(M2, 1000.0f, 2000.0f);
M3 = constrain(M3, 1000.0f, 2000.0f);
M4 = constrain(M4, 1000.0f, 2000.0f);



throttleOverride = 1000; // reset when flying normally
writeMotors(M1, M2, M3, M4);

    // Debug overall status
    static unsigned long last_status_dbg = 0;
    if (millis() - last_status_dbg > 500) {
        Serial.printf("[FC STATUS] ARM=%d, FS=%d, TO=%d, Alt=%.2f m, Thr=%.0f\n", 
                      com->armed, com->failsafe, com->takeoff, alt, throttleOverride);
        last_status_dbg = millis();
    }

}


















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