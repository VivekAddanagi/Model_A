#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

// === GPIO setup ===
#define EMITTER_FRONT  40
#define EMITTER_RIGHT  41
#define EMITTER_BACK   38
#define EMITTER_LEFT   39
#define IR_RECEIVER    1  // ADC pin

// Directions (front, right, back, left)
enum Direction { FRONT, RIGHT, BACK, LEFT };

// Default danger threshold (can be overridden)
#define IR_DANGER_THRESHOLD 2000

class IRSensor {
public:
    IRSensor() = default;

    void begin();             
    void poll();              
    float getDistance(Direction dir) const;
    bool isNear(Direction dir, int threshold = -1) const; 
    void handleAvoidance();   

    // Threshold config API
    void setThreshold(int threshold);
    int  getThreshold() const;
    uint8_t getObstacleFlags() const;  // default threshold

private:
    int readings[4] = {0,0,0,0};
    uint8_t emitterPins[4] = { EMITTER_FRONT, EMITTER_BACK, EMITTER_LEFT, EMITTER_RIGHT };
    int dangerThreshold = IR_DANGER_THRESHOLD;  
};

#endif // IR_SENSOR_H
