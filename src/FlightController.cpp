#include "FlightController.h"

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

// -------------------- Write PWM to motors --------------------
void FlightController::writeMotors(float m1, float m2, float m3, float m4) {
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);

    analogWrite(MOTOR_PIN_1, m1);
    analogWrite(MOTOR_PIN_2, m2);
    analogWrite(MOTOR_PIN_3, m3);
    analogWrite(MOTOR_PIN_4, m4);

    // Debug motors
    static unsigned long last_motor_dbg = 0;
    if (millis() - last_motor_dbg > 200) {
        Serial.printf("[MOTORS] M1: %.0f | M2: %.0f | M3: %.0f | M4: %.0f\n", m1, m2, m3, m4);
        last_motor_dbg = millis();
    }
}

// -------------------- Initialize motors --------------------
void FlightController::begin() {
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(MOTOR_PIN_3, OUTPUT);
    pinMode(MOTOR_PIN_4, OUTPUT);

    // Idle PWM safe
    writeMotors(1000, 1000, 1000, 1000);
    Serial.println("[FC] Motors initialized and set to idle.");
}

// -------------------- Flight loop update --------------------
void FlightController::update(float dt) {
    // --- Sensor readings ---
float roll  = estimated_roll;
float pitch = estimated_pitch;
float yaw   = estimated_yaw;
float alt   = sensor->alt_est;

    // Debug sensor readings
    static unsigned long last_sensor_dbg = 0;
    if (millis() - last_sensor_dbg > 200) {
        Serial.printf("[SENSORS] Roll: %.2f | Pitch: %.2f | Yaw: %.2f | Alt: %.2f\n",
                      roll, pitch, yaw, alt);
        last_sensor_dbg = millis();
    }

    // --- PID outputs ---
    float roll_cmd  = PID_Update(pid_roll,  roll_set,  roll,  dt);
    float pitch_cmd = PID_Update(pid_pitch, pitch_set, pitch, dt);
    float yaw_cmd   = PID_Update(pid_yaw,   yaw_set,   yaw,   dt);
    float alt_cmd   = PID_Update(pid_alt,   alt_set,   alt,   dt);

    // --- Motor mixing (X quad, brushed motors) ---
    float M1 = alt_cmd + pitch_cmd + roll_cmd - yaw_cmd; // Front Left CCW
    float M2 = alt_cmd + pitch_cmd - roll_cmd + yaw_cmd; // Front Right CW
    float M3 = alt_cmd - pitch_cmd - roll_cmd - yaw_cmd; // Rear Right CCW
    float M4 = alt_cmd - pitch_cmd + roll_cmd + yaw_cmd; // Rear Left CW

    writeMotors(M1, M2, M3, M4);
}