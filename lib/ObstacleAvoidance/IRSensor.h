#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

// Directions (front, right, back, left)
enum Direction { FRONT, RIGHT, BACK, LEFT };

class IRSensor {
public:
    IRSensor() = default;

    void begin();             // Configure pins, ADC resolution
    void poll();              // Scan all emitters
    float getDistance(Direction dir) const;  // Return raw ADC value
    bool isNear(Direction dir, int threshold = 2000) const; 
    void handleAvoidance();   // Print/log or adjust motors

private:
    int readings[4] = {0, 0, 0, 0};  // Last sensor readings
};

#endif // IR_SENSOR_H
