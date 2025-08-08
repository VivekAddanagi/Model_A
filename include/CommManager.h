#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include "CC2500Receiver.h"
#include "types.h"

class CommManager {
public:
    CommManager();
    void begin();
    void update();
    bool hasValidData() const;
    RXData getControlData() const;
    bool isSignalLost(uint32_t timeoutMs = 200) const;

private:
    CC2500Receiver _receiver;   // Now an object, not a pointer
    RXData _latestData;
    bool _validPacket;
};

#endif // COMM_MANAGER_H
