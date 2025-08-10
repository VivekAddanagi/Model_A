#include "CommManager.h"

ComManager::ComManager() 
    : _cc2500(CC2500_CS_PIN, CC2500_GDO0_PIN, SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN),
      _newData(false) {}

bool ComManager::begin() {
    Serial.println("[ComManager] Starting CC2500 receiver...");
    _cc2500.begin();
    return true;
}

void ComManager::update() {
    if (_cc2500.receivePacket()) {
        if (_cc2500.getLatestControlData(yaw, pitch, roll, throttle, mode, takeoff, failsafe, photo, video)) {
            _newData = true;
        } else {
            _newData = false;
        }
    } else {
        _newData = false;
    }
}

bool ComManager::hasNewData() const {
    return _newData;
}
