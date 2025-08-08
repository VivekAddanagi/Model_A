#include "CommManager.h"
#include "Config.h"
#include <string.h> // for memset

CommManager::CommManager()
    : _receiver(CC2500_CS_PIN, SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN),
      _validPacket(false) 
{
    memset(&_latestData, 0, sizeof(_latestData));
}

void CommManager::begin() {
    _receiver.begin();
}

void CommManager::update() {
    if (_receiver.receivePacket()) {
        int8_t yaw, pitch, roll;
        uint8_t throttle, mode, takeoff, failsafe, photo, video;

        if (_receiver.getLatestControlData(yaw, pitch, roll, throttle, mode, takeoff, failsafe, photo, video)) {
            _latestData.yaw      = yaw;
            _latestData.pitch    = pitch;
            _latestData.roll     = roll;
            _latestData.throttle = throttle;
            _latestData.mode     = mode;
            _latestData.takeoff  = takeoff;
            _latestData.failsafe = failsafe;
            _latestData.photo    = photo;
            _latestData.video    = video;
            _validPacket = true;
        } else {
            _validPacket = false;
        }
    }
}

bool CommManager::hasValidData() const {
    return _validPacket;
}

RXData CommManager::getControlData() const {
    return _latestData;
}

bool CommManager::isSignalLost(uint32_t timeoutMs) const {
    return _receiver.isTimedOut(timeoutMs);
}
