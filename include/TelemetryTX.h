#ifndef TELEMETRY_TX_H
#define TELEMETRY_TX_H

#include <Arduino.h>
#include "SensorManager.h"
#include "ComManager.h"
#include "CC2500Receiver.h"  // we’ll reuse same radio driver

class TelemetryTx {
public:
    TelemetryTx(ComManager* com, SensorManager* sensors);

    void begin();
    void update();

private:
    ComManager* _com;
    SensorManager* _sensors;
    uint8_t _packetCounter = 0;
    unsigned long _lastSend = 0;

    void sendTelemetry();
    uint8_t calcChecksum(const uint8_t* data, uint8_t len);
};

#endif
