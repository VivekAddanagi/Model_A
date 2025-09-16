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
        sendTelemetry();
    }
}

void TelemetryTx::sendTelemetry() {
    uint8_t buf[20] = {0};
    buf[0] = 0xA5;              // start byte
    buf[1] = _packetCounter++;  // counter

    // Altitude (m → cm)
    int16_t alt_cm = (int16_t)(_sensors->getAltitudeMeters() * 100);
    buf[2] = alt_cm & 0xFF;
    buf[3] = (alt_cm >> 8) & 0xFF;

    // Gyroscope (scaled *100)
    int16_t gx = (int16_t)(_sensors->getGyroX() * 100);
    int16_t gy = (int16_t)(_sensors->getGyroY() * 100);
    int16_t gz = (int16_t)(_sensors->getGyroZ() * 100);
    buf[4] = gx & 0xFF; buf[5] = gx >> 8;
    buf[6] = gy & 0xFF; buf[7] = gy >> 8;
    buf[8] = gz & 0xFF; buf[9] = gz >> 8;

    // Accelerometer (scaled *1000)
    int16_t ax = (int16_t)(_sensors->getAccelX() * 1000);
    int16_t ay = (int16_t)(_sensors->getAccelY() * 1000);
    int16_t az = (int16_t)(_sensors->getAccelZ() * 1000);
    buf[10] = ax & 0xFF; buf[11] = ax >> 8;
    buf[12] = ay & 0xFF; buf[13] = ay >> 8;
    buf[14] = az & 0xFF; buf[15] = az >> 8;

    // Error flags
    uint8_t errors = 0;
    if (_com->failsafe) errors |= 0x01;
    buf[16] = errors;

    // Battery % (future placeholder)
    buf[17] = 0;

    // RSSI from last control packet
    buf[18] = (uint8_t)_com->getRSSI();

    // Checksum
    buf[19] = calcChecksum(buf, 19);

    // Debug print human-readable values
    Serial.printf("[TelemetryTx] Packet #%u | Alt: %.2f m | Gyro: (%.2f, %.2f, %.2f) dps | Accel: (%.3f, %.3f, %.3f) g | Errors: 0x%02X | RSSI: %d dBm\n",
                  buf[1],
                  alt_cm / 100.0f,
                  gx / 100.0f, gy / 100.0f, gz / 100.0f,
                  ax / 1000.0f, ay / 1000.0f, az / 1000.0f,
                  errors,
                  (int8_t)buf[18]);

    // Debug print raw packet bytes
   // Serial.print("[TelemetryTx] Raw: ");
   // for (uint8_t i = 0; i < sizeof(buf); i++) {
       // Serial.printf("%02X ", buf[i]);
  //  }
  //  Serial.println();

    // Hand off to ComManager
    if (_com->sendTelemetryPacket(buf, sizeof(buf))) {
        Serial.println("[TelemetryTx] Packet sent OK");
    } else {
        Serial.println("[TelemetryTx] ERROR sending packet");
    }
}

uint8_t TelemetryTx::calcChecksum(const uint8_t* data, uint8_t len) {
    uint8_t chk = 0;
    for (uint8_t i = 0; i < len; i++) {
        chk ^= data[i];
    }
    return chk;
}
