#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <Arduino.h>
#include "CC2500Receiver.h"
#include "Config.h"

// Structure to hold control data from the receiver
struct RXData {
    int8_t yaw;
    int8_t pitch;
    int8_t roll;
    uint8_t throttle;
    uint8_t mode;
    bool takeoff;
    bool failsafe;
    bool photo;
    bool video;
};

class CommManager {
public:
    CommManager();
    void begin();
    void update();

    // Returns true if a new, valid packet is available
    bool hasValidData() const;

    // Gets the latest RX control data
    RXData getControlData() const;

    // Optional: Check if receiver timeout occurred
    bool isSignalLost(uint32_t timeoutMs = 300) const;

private:
    CC2500Receiver _receiver;
    RXData _latestData;
    bool _validPacket;
};

#endif // COMM_MANAGER_H
