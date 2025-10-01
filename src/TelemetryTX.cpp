#include "TelemetryTx.h"
#include "FlightController.h"
#include "ComManager.h"
#include "DroneLEDController.h"

extern DroneState currentState;


// Status codes
#define CODE_LOW_BATTERY   1   // Error
#define CODE_MOTOR_FAIL    2   // Error
#define CODE_GPS_WEAK      3   // Warning
#define CODE_CALIBRATION   4   // Warning
#define CODE_INFO_STARTUP  5   // Info
#define CODE_INFO_LANDED   6   // Info
#define CODE_SENSOR_FAIL   7   // Error
#define CODE_SENSOR_OK     8   // Info
#define CODE_CC2500_FAIL   9   // Error
#define CODE_CC2500_OK     10  // Info


// Severity levels
#define SEVERITY_INFO      0
#define SEVERITY_WARNING   1
#define SEVERITY_CRITICAL  2




TelemetryTx::TelemetryTx(ComManager* com, SensorManager* sensors, FlightController* fc, IRSensor* ir)
    : _com(com), _sensors(sensors), _fc(fc), _irSensor(ir) {}


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
    static unsigned long lastSuccessTime = 0;
    unsigned long now = millis();

    uint8_t payload[42] = {0}; // increased payload to include velocities
    payload[0] = _packetCounter++;

    // --- Existing fields ---
    int16_t alt_cm = (int16_t)(_sensors->getAltitudeMeters() * 100);
    payload[1] = alt_cm & 0xFF;
    payload[2] = (alt_cm >> 8) & 0xFF;

    int16_t gx = (int16_t)(_sensors->getGyroX() * 100);
    int16_t gy = (int16_t)(_sensors->getGyroY() * 100);
    int16_t gz = (int16_t)(_sensors->getGyroZ() * 100);
    payload[3] = gx & 0xFF; payload[4] = gx >> 8;
    payload[5] = gy & 0xFF; payload[6] = gy >> 8;
    payload[7] = gz & 0xFF; payload[8] = gz >> 8;

    int16_t ax = (int16_t)(_sensors->getAccelX() * 1000);
    int16_t ay = (int16_t)(_sensors->getAccelY() * 1000);
    int16_t az = (int16_t)(_sensors->getAccelZ() * 1000);
    payload[9]  = ax & 0xFF; payload[10] = ax >> 8;
    payload[11] = ay & 0xFF; payload[12] = ay >> 8;
    payload[13] = az & 0xFF; payload[14] = az >> 8;

    int16_t roll  = (int16_t)(estimated_roll  * 100);
    int16_t pitch = (int16_t)(estimated_pitch * 100);
    int16_t yaw   = (int16_t)(estimated_yaw   * 100);
    payload[15] = roll & 0xFF;    payload[16] = roll >> 8;
    payload[17] = pitch & 0xFF;   payload[18] = pitch >> 8;
    payload[19] = yaw & 0xFF;     payload[20] = yaw >> 8;

    // --- Status ---
    uint8_t statusCode = CODE_INFO_STARTUP;
    uint8_t severity = SEVERITY_INFO;
    if (!_com->isHealthy()) { statusCode = CODE_CC2500_FAIL; severity = SEVERITY_CRITICAL; }
    else if (_com->failsafe) { statusCode = CODE_MOTOR_FAIL; severity = SEVERITY_CRITICAL; }
    else if (_sensors->getStatus() == SENSOR_STATUS_ERROR) { statusCode = CODE_SENSOR_FAIL; severity = SEVERITY_CRITICAL; }
    else if (_sensors->getStatus() == SENSOR_STATUS_OK) { statusCode = CODE_SENSOR_OK; severity = SEVERITY_INFO; }
    payload[21] = statusCode;
    payload[22] = severity;

    payload[23] = (uint8_t)currentState;
    payload[24] = 0; // battery placeholder
    payload[25] = _irSensor->getObstacleFlags();
    payload[26] = (uint8_t)_com->getRSSI();

    uint32_t t_ms = now;
    payload[27] = t_ms & 0xFF;
    payload[28] = (t_ms >> 8) & 0xFF;
    payload[29] = (t_ms >> 16) & 0xFF;
    payload[30] = (t_ms >> 24) & 0xFF;

    payload[31] = (uint8_t)_fc->_lastMotorDuty[0];
    payload[32] = (uint8_t)_fc->_lastMotorDuty[1];
    payload[33] = (uint8_t)_fc->_lastMotorDuty[2];
    payload[34] = (uint8_t)_fc->_lastMotorDuty[3];

    // --- Add velocities (scaled to cm/s) ---
    int16_t vx_cm = (int16_t)(_sensors->getVelX() * 100.0f);
    int16_t vy_cm = (int16_t)(_sensors->getVelY() * 100.0f);
    int16_t vz_cm = (int16_t)(_sensors->getVelZ() * 100.0f);
    payload[35] = vx_cm & 0xFF; payload[36] = vx_cm >> 8;
    payload[37] = vy_cm & 0xFF; payload[38] = vy_cm >> 8;
    payload[39] = vz_cm & 0xFF; payload[40] = vz_cm >> 8;

    // Checksum
    payload[41] = calcChecksum(payload, 41);

    // Send packet
    uint8_t buf[42];
    buf[0] = sizeof(payload);
    memcpy(&buf[1], payload, sizeof(payload));
    if (_com->sendTelemetryPacket(buf, sizeof(buf))) {
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