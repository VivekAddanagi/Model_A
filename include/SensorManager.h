#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include "Config.h"
#include "bmi323.h"

class SensorManager {
public:
    SensorManager();

    bool begin();
    void update();
    void calibrateBMI323(bool force = false);

    float getPitch() const { return estimated_pitch; }
    float getRoll()  const { return estimated_roll;  }
    float getYaw()   const { return estimated_yaw;   }

private:
    void printBMI323Data();
};

#endif // SENSOR_MANAGER_H
