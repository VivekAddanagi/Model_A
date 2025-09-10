#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>



class IRSensor {
public:
    IRSensor();
    void begin();             // sets pins, ADC resolution
    void poll();              // take one scan of all emitters (non-blocking caller)
    float getDistance(Direction dir) const;
    bool isNear(Direction dir, int threshold = IR_DANGER_THRESHOLD) const;

private:
    int readings[4] = {0,0,0,0};
    uint8_t emitterPins[4] = { IR_EMITTER_FRONT, IR_EMITTER_BACK, IR_EMITTER_LEFT, IR_EMITTER_RIGHT };

    void measureDirection(uint8_t idx);
};

#endif // IR_SENSOR_H
