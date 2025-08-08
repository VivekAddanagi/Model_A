#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <Arduino.h>
#include "types.h"
#include "Config.h"
#include "CC2500Receiver.h"

class CommManager {
public:
    CommManager();                // Constructor
    void begin();                  // Initialize CC2500
    void update();                 // Read incoming data
    bool hasValidData() const;     // Data available & valid
    RXData getControlData() const; // Latest control input
    bool isSignalLost(uint32_t timeoutMs) const; // Link loss detection
    void sendData();               // Optional telemetry back

private:
    CC2500Receiver _receiver;
    RXData _latestData{};
    bool _validPacket = false;
    uint32_t _lastPacketTime = 0;
};

#endif
