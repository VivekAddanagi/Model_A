#include "TelemetryTx.h"
#include "FlightController.h"


// Status codes
#define CODE_LOW_BATTERY   1   // Error
#define CODE_MOTOR_FAIL    2   // Error
#define CODE_GPS_WEAK      3   // Warning
#define CODE_CALIBRATION   4   // Warning
#define CODE_INFO_STARTUP  5   // Info
#define CODE_INFO_LANDED   6   // Info

// Severity levels
#define SEVERITY_INFO      0
#define SEVERITY_WARNING   1
#define SEVERITY_CRITICAL  2


TelemetryTx::TelemetryTx(ComManager* com, SensorManager* sensors, FlightController* fc)
    : _com(com), _sensors(sensors), _fc(fc) {}


void TelemetryTx::begin() {
    Serial.println("[TelemetryTx] Initialized");
}

void TelemetryTx::update() {
    const unsigned long SEND_INTERVAL_MS = 1000; // 20 Hz
    unsigned long now = millis();

    if (now - _lastSend >= SEND_INTERVAL_MS) {
        _lastSend = now;
        sendTelemetry();
        // sendDummyTelemetry();
    }
}
void TelemetryTx::sendTelemetry() {
    static unsigned long lastSuccessTime = 0; // track last successful send
    unsigned long now = millis();

    uint8_t payload[34] = {0}; // increased payload size for roll/pitch/yaw + timestamp + motors
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

    // Euler angles (roll, pitch, yaw)
    int16_t roll  = (int16_t)(estimated_roll  * 100);
    int16_t pitch = (int16_t)(estimated_pitch * 100);
    int16_t yaw   = (int16_t)(estimated_yaw   * 100);
    payload[15] = roll & 0xFF;    payload[16] = roll >> 8;
    payload[17] = pitch & 0xFF;   payload[18] = pitch >> 8;
    payload[19] = yaw & 0xFF;     payload[20] = yaw >> 8;

    // Status (error/warning/info)
    uint8_t statusCode = 0;
    uint8_t severity = SEVERITY_INFO;
    if (_com->failsafe) {
        statusCode = CODE_MOTOR_FAIL;
        severity = SEVERITY_CRITICAL;
    } else {
        statusCode = CODE_INFO_STARTUP;
        severity = SEVERITY_INFO;
    }
    payload[21] = statusCode;
    payload[22] = severity;

    // Battery % placeholder
    payload[23] = 0;

    // RSSI
    payload[24] = (uint8_t)_com->getRSSI();

    // Timestamp (milliseconds, 4 bytes)
    uint32_t t_ms = now;
    payload[25] = t_ms & 0xFF;
    payload[26] = (t_ms >> 8) & 0xFF;
    payload[27] = (t_ms >> 16) & 0xFF;
    payload[28] = (t_ms >> 24) & 0xFF;

    // --- Add motor duty cycles (scaled 0–100%) ---
    payload[29] = (uint8_t)_fc->_lastMotorDuty[0]; // M1 %
    payload[30] = (uint8_t)_fc->_lastMotorDuty[1]; // M2 %
    payload[31] = (uint8_t)_fc->_lastMotorDuty[2]; // M3 %
    payload[32] = (uint8_t)_fc->_lastMotorDuty[3]; // M4 %

    // Checksum
    payload[33] = calcChecksum(payload, 33); // last byte = checksum

    uint8_t packetLength = 34;
    uint8_t buf[35];
    buf[0] = packetLength; // length byte
    memcpy(&buf[1], payload, packetLength);

    if (_com->sendTelemetryPacket(buf, packetLength + 1)) {
        lastSuccessTime = now;
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

/*
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
*/