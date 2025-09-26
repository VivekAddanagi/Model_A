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

int8_t ComManager::getRSSI() const {
    return _cc2500.getLastRSSI();
}

bool ComManager::sendTelemetryPacket(const uint8_t* data, uint8_t len) {
    if (len == 0) {
        Serial.println("[ComManager ERROR] sendTelemetryPacket called with len=0");
        return false;
    }

   // Serial.printf("[ComManager DEBUG] TX len=%u\n", len);

    // --- Enter IDLE ---
    _cc2500._strobeCommand(0x36); // SIDLE
    delayMicroseconds(50);
    uint8_t marcState = _cc2500._readRegister(0x35) & 0x1F;
    if (marcState != 0x01) { // IDLE
        Serial.printf("[ComManager ERROR] SIDLE failed, MARCSTATE=0x%02X\n", marcState);
        return false;
    }
   // Serial.printf("[ComManager DEBUG] After SIDLE, MARCSTATE=0x%02X\n", marcState);

    // --- Flush TX FIFO ---
    _cc2500._strobeCommand(0x3B); // SFTX
    delayMicroseconds(10);

    // --- Write TX FIFO ---
    digitalWrite(CC2500_CS_PIN, LOW);
    SPI.transfer(0x7F | 0x40); // burst write TX FIFO
    for (uint8_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }
    digitalWrite(CC2500_CS_PIN, HIGH);

    // Verify TX FIFO count
    uint8_t txBytes = _cc2500._readRegister(0x3A) & 0x7F; // TXBYTES
   // Serial.printf("[ComManager DEBUG] TX FIFO now has %u bytes\n", txBytes);
    if (txBytes != len) {
        Serial.printf("[ComManager ERROR] TX FIFO mismatch: expected=%u got=%u\n", len, txBytes);
        return false;
    }

    // --- Start TX ---
    _cc2500._strobeCommand(0x35); // STX
    delayMicroseconds(10);
    marcState = _cc2500._readRegister(0x35) & 0x1F;

    if (marcState == 0x16) { // TXFIFO_UNDERFLOW
        Serial.println("[ComManager ERROR] TXFIFO underflow immediately after STX!");
        return false;
    }

    if (marcState != 0x13 && marcState != 0x08) {
        // Allow TX (0x13) and STARTCAL (0x08) when auto-calibration is enabled
        Serial.printf("[ComManager ERROR] Unexpected MARCSTATE after STX=0x%02X\n", marcState);
        return false;
    }
   // Serial.printf("[ComManager DEBUG] After STX, MARCSTATE=0x%02X\n", marcState);

    // --- Wait for GDO0 HIGH (sync word sent) ---
    unsigned long start = micros();
    while (digitalRead(CC2500_GDO0_PIN) == LOW && (micros() - start) < 10000);
    if (digitalRead(CC2500_GDO0_PIN) == HIGH) {
      //  Serial.printf("[ComManager DEBUG] GDO0 went HIGH after %lu us\n", micros() - start);
    } else {
        Serial.println("[ComManager ERROR] Timeout waiting for GDO0 HIGH");
        return false;
    }

    // --- Wait for GDO0 LOW (end of packet) ---
    start = micros();
    while (digitalRead(CC2500_GDO0_PIN) == HIGH && (micros() - start) < 50000);
    if (digitalRead(CC2500_GDO0_PIN) == LOW) {
       // Serial.printf("[ComManager DEBUG] GDO0 went LOW after %lu us\n", micros() - start);
    } else {
        Serial.println("[ComManager ERROR] Timeout waiting for GDO0 LOW");
        return false;
    }

    // --- Back to RX ---
    _cc2500._strobeCommand(0x34); // SRX
    delayMicroseconds(50);
    marcState = _cc2500._readRegister(0x35) & 0x1F;

    if (marcState == 0x16) { // TXFIFO_UNDERFLOW (should not happen here, but check)
        Serial.println("[ComManager ERROR] TXFIFO underflow detected after TX!");
        return false;
    }

    // Accept valid post-TX states
    if (marcState != 0x01 && // IDLE
        marcState != 0x0D && // RX
        marcState != 0x12 && // FSTXON
        marcState != 0x08)   // STARTCAL (if calibration on RX entry)
    {
        Serial.printf("[ComManager ERROR] Unexpected MARCSTATE after SRX=0x%02X\n", marcState);
        return false;
    }

   // Serial.printf("[ComManager DEBUG] After SRX, MARCSTATE=0x%02X\n", marcState);

    return true;
}
