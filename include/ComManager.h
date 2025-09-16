#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <Arduino.h>
#include "CC2500Receiver.h"

class ComManager {
public:
    ComManager();
    bool selfTest();

    bool begin();              // Initialize comms
    void update();             // Call frequently to check for new packets
    bool hasNewData() const;   // Check if last packet is valid
    // ComManager.h (add to public section)
int8_t getRSSI() const;


    // Access to latest control data
    int8_t yaw, pitch, roll;
    uint8_t throttle, mode, armed , takeoff, failsafe, photo, video;

private:
    CC2500Receiver _cc2500;
    bool _newData;
};

#endif // COM_MANAGER_H
