#include "TelemetryTx.h"

TelemetryTx::TelemetryTx(ComManager* com, SensorManager* sensors)
    : _com(com), _sensors(sensors) {}

void TelemetryTx::begin() {
    Serial.println("[TelemetryTx] Initialized");
}

void TelemetryTx::update() {
    const unsigned long SEND_INTERVAL_MS = 1000; // 20 Hz
    unsigned long now = millis();

    if (now - _lastSend >= SEND_INTERVAL_MS) {
        _lastSend = now;
       // sendTelemetry();
         sendDummyTelemetry();
    }
}
/*
void TelemetryTx::sendTelemetry() {
    uint8_t payload[19] = {0}; // payload excluding length byte
    payload[0] = _packetCounter++;  // counter

    // Altitude (m → cm)
    int16_t alt_cm = (int16_t)(_sensors->getAltitudeMeters() * 100);
    payload[1] = alt_cm & 0xFF;
    payload[2] = (alt_cm >> 8) & 0xFF;

    // Gyroscope
    int16_t gx = (int16_t)(_sensors->getGyroX() * 100);
    int16_t gy = (int16_t)(_sensors->getGyroY() * 100);
    int16_t gz = (int16_t)(_sensors->getGyroZ() * 100);
    payload[3] = gx & 0xFF; payload[4] = gx >> 8;
    payload[5] = gy & 0xFF; payload[6] = gy >> 8;
    payload[7] = gz & 0xFF; payload[8] = gz >> 8;

    // Accelerometer
    int16_t ax = (int16_t)(_sensors->getAccelX() * 1000);
    int16_t ay = (int16_t)(_sensors->getAccelY() * 1000);
    int16_t az = (int16_t)(_sensors->getAccelZ() * 1000);
    payload[9]  = ax & 0xFF; payload[10] = ax >> 8;
    payload[11] = ay & 0xFF; payload[12] = ay >> 8;
    payload[13] = az & 0xFF; payload[14] = az >> 8;

    // Errors, battery, RSSI
    uint8_t errors = _com->failsafe ? 0x01 : 0x00;
    payload[15] = errors;
    payload[16] = 0; // battery %
    payload[17] = (uint8_t)_com->getRSSI();

    // Checksum
    payload[18] = calcChecksum(payload, 18);

    uint8_t packetLength = sizeof(payload); // 19 bytes
    uint8_t buf[20];
    buf[0] = packetLength; // length byte for variable-length mode
    memcpy(&buf[1], payload, packetLength);

    // Send to CC2500
    if (_com->sendTelemetryPacket(buf, packetLength + 1)) {
        Serial.println("[TelemetryTx] Packet sent OK (variable length)");
    } else {
        Serial.println("[TelemetryTx] ERROR sending packet");
    }
}

*/

uint8_t TelemetryTx::calcChecksum(const uint8_t* data, uint8_t len) {
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}

void TelemetryTx::sendDummyTelemetry() {
    static unsigned long lastSuccessTime = 0; // store last successful send
    unsigned long now = millis();

    uint8_t payload[19] = {0};
    payload[0] = _packetCounter++;

    int16_t alt_cm = (int16_t)(_sensors->getAltitudeMeters() * 100);
    payload[1] = alt_cm & 0xFF;
    payload[2] = (alt_cm >> 8) & 0xFF;

    // Gyroscope
    int16_t gx = (int16_t)(_sensors->getGyroX() * 100);
    int16_t gy = (int16_t)(_sensors->getGyroY() * 100);
    int16_t gz = (int16_t)(_sensors->getGyroZ() * 100);
    payload[3] = gx & 0xFF; payload[4] = gx >> 8;
    payload[5] = gy & 0xFF; payload[6] = gy >> 8;
    payload[7] = gz & 0xFF; payload[8] = gz >> 8;

    // Accelerometer
    int16_t ax = (int16_t)(_sensors->getAccelX() * 1000);
    int16_t ay = (int16_t)(_sensors->getAccelY() * 1000);
    int16_t az = (int16_t)(_sensors->getAccelZ() * 1000);
    payload[9]  = ax & 0xFF; payload[10] = ax >> 8;
    payload[11] = ay & 0xFF; payload[12] = ay >> 8;
    payload[13] = az & 0xFF; payload[14] = az >> 8;

    // Errors, battery, RSSI
    uint8_t errors = _com->failsafe ? 0x01 : 0x00;
    payload[15] = errors;
    payload[16] = 0; // battery %
    payload[17] = (uint8_t)_com->getRSSI();
    payload[18] = calcChecksum(payload, 18);

    uint8_t packetLength = sizeof(payload);
    uint8_t buf[20];
    buf[0] = packetLength;
    memcpy(&buf[1], payload, packetLength);

    // --- DEBUG GDO0 before TX ---
    Serial.printf("[TelemetryTx DEBUG] GDO0 before send: %d\n", digitalRead(CC2500_GDO0_PIN));

    if (_com->sendTelemetryPacket(buf, packetLength + 1)) {
        Serial.println("[TelemetryTx] Dummy packet sent OK");

        // --- Print time since last successful transmission ---
        if (lastSuccessTime != 0) {
            unsigned long dt = now - lastSuccessTime;
            Serial.printf("[TelemetryTx] Time since last successful send: %lu ms\n", dt);
        }
        lastSuccessTime = now;

    } else {
        Serial.println("[TelemetryTx] ERROR sending dummy packet");
    }

    // --- DEBUG GDO0 after TX ---
    Serial.printf("[TelemetryTx DEBUG] GDO0 after send: %d\n", digitalRead(CC2500_GDO0_PIN));

    // Optional: dump bytes
    Serial.print("[TelemetryTx] Dummy packet bytes: ");
    for (uint8_t i = 0; i < packetLength + 1; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
}
