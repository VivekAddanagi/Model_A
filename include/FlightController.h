#pragma once
#include "SensorManager.h"
#include <Arduino.h>

// Quad motor pins (assigned as per your hardware)
#define MOTOR_PIN_1 4    // Front Left - CCW
#define MOTOR_PIN_2 2    // Front Right - CW
#define MOTOR_PIN_3 42   // Rear Right - CCW
#define MOTOR_PIN_4 44   // Rear Left - CW

struct PID {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float output_min;
    float output_max;
};

class FlightController {
public:
    FlightController(SensorManager* sm) : sensor(sm) {}

    void begin();
    void update(float dt);

    // Setpoints
    float roll_set  = 0.0f;
    float pitch_set = 0.0f;
    float yaw_set   = 0.0f;
    float alt_set   = 1.5f; // meters

private:
    SensorManager* sensor;   // <-- pointer to SensorManager

    PID pid_roll  = {6.0f, 0.0f, 0.5f, 0,0,-500,500};
    PID pid_pitch = {6.0f, 0.0f, 0.5f, 0,0,-500,500};
    PID pid_yaw   = {2.0f, 0.0f, 0.1f, 0,0,-500,500};
    PID pid_alt   = {2.0f, 0.5f, 1.0f, 0,0,-400,400};

    float PID_Update(PID &pid, float setpoint, float measured, float dt);
    void writeMotors(float m1, float m2, float m3, float m4);
};
