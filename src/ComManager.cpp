#include "ComManager.h"
#include "CC2500Receiver.h"

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
        if (_cc2500.getLatestControlData(yaw, pitch, roll, throttle, mode, armed , takeoff, failsafe, photo, video)) {
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

bool ComManager::selfTest() {
    // Read PARTNUM (0x30)
    uint8_t part = _cc2500._readRegister(0x30);
    Serial.printf("[CC2500 SELFTEST] PARTNUM=0x%02X\n", part);

    // Check if it’s the expected 0x80
    if (part != 0x80) {
        Serial.println("[CC2500 SELFTEST] Unexpected PARTNUM! Check wiring.");
        return false;
    }

    // Try one receive attempt
    if (_cc2500.receivePacket()) {
        Serial.println("[CC2500 SELFTEST] Packet received.");
    } else {
        Serial.println("[CC2500 SELFTEST] No packet (but chip is alive).");
    }

    return true;
}
