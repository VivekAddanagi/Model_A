#include "CC2500Receiver.h"
#include <Arduino.h>

#define SRX    0x34
#define SIDLE  0x36
#define SFRX   0x3A
#define RX_FIFO_BURST 0xFF

#if defined(ESP32)
int getPinMode(uint8_t pin) {
    uint32_t mode = (GPIO.pin[pin].val >> 8) & 0x07;
    return mode;
}
#endif

CC2500Receiver::CC2500Receiver(uint8_t csPin, uint8_t gdo0Pin,
                               uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin)
    : _cs(csPin), _gdo0(gdo0Pin), _sck(sckPin), _miso(misoPin), _mosi(mosiPin),
      _lastPacketTime(0), _hasValidPacket(false) {}

void CC2500Receiver::begin() {
    Serial.println("[CC2500] Initializing SPI and GPIO...");
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CC2500_CS_PIN);

    pinMode(_cs, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(CC2500_GDO0_PIN, INPUT_PULLUP);

#if defined(ESP32)
    Serial.printf("[DEBUG] GDO0 pin mode: %d, level: %d\n",
                  getPinMode(CC2500_GDO0_PIN),
                  digitalRead(CC2500_GDO0_PIN));
#endif

    digitalWrite(_cs, HIGH);

    _reset();
    _configureRadio();
    _printConfigSummary();
    _loadPATable();
    _strobeCommand(SRX);
    Serial.println("[CC2500] Initialization complete.");
}

bool CC2500Receiver::receivePacket() {
    uint8_t len = 0;
    bool crcOk;
    delayMicroseconds(20);
    uint8_t state = _readMARCState();
    Serial.printf("[DEBUG] MARCSTATE = 0x%02X\n", state);

    if (_readRXFIFO(_packet, len, crcOk)) {
        if (_verifyPacket(_packet, len, crcOk)) {
            _lastPacketTime = millis();
            _hasValidPacket = true;
            return true;
        }
    }

    Serial.println("[CC2500] Invalid packet or CRC failed.");
    _strobeCommand(SIDLE);
    _strobeCommand(SFRX);
    _strobeCommand(SRX);
    delayMicroseconds(20);
    return false;
}

bool CC2500Receiver::getLatestControlData(int8_t& yaw, int8_t& pitch, int8_t& roll,
                                          uint8_t& throttle, uint8_t& mode, uint8_t& takeoff,
                                          uint8_t& failsafe, uint8_t& photo, uint8_t& video) {
    if (!_hasValidPacket) return false;

    Serial.print("[RX PACKET] ");
    for (int i = 0; i < CC2500_DATA_BYTES; i++) {
        Serial.printf("0x%02X ", _packet[i]);
    }
    Serial.println();

    yaw      = _packet[1];
    pitch    = _packet[2];
    roll     = _packet[3];
    throttle = _packet[4];
    mode     = _packet[5];
    takeoff  = _packet[6];
    failsafe = _packet[7];
    photo    = _packet[8];
    video    = _packet[9];

    Serial.printf("[RX DATA] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d TO=%d FS=%d PH=%d VID=%d\n",
                  yaw, pitch, roll, throttle, mode, takeoff, failsafe, photo, video);
    return true;
}

bool CC2500Receiver::isTimedOut(uint32_t timeoutMs) const {
    return (millis() - _lastPacketTime) > timeoutMs;
}

void CC2500Receiver::_reset() {
    Serial.println("[RESET] Sending SRES...");

    // Ensure CS is high before starting transaction
    digitalWrite(_cs, HIGH);
    delayMicroseconds(100);

    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);
    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        Serial.println("[RESET] Chip not ready before reset!");
        return;
    }

    SPI.transfer(0x30); // SRES
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    delay(1);
    if (!_waitForChipReady()) {
        Serial.println("[RESET] Chip not ready after reset!");
        return;
    }
    Serial.println("[RESET] Complete.");
}

bool CC2500Receiver::_waitForChipReady() {
    uint32_t timeout = micros() + 1000;
    while (digitalRead(_miso)) {
        if (micros() > timeout) {
            Serial.println("[WAIT] Timeout waiting for MISO to go low.");
            return false;
        }
    }
    return true;
}

void CC2500Receiver::_writeRegister(uint8_t addr, uint8_t value) {
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    SPI.transfer(addr & 0x3F);
    SPI.transfer(value);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}

uint8_t CC2500Receiver::_readRegister(uint8_t addr) {
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return 0xFF;
    }

    uint8_t header = (addr >= 0x30) ? (addr | 0xC0) : (addr | 0x80);
    SPI.transfer(header);
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return value;
}

void CC2500Receiver::_strobeCommand(uint8_t cmd) {
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    SPI.transfer(cmd);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}

void CC2500Receiver::_loadPATable() {
    const uint8_t paTable[1] = { 0xFF };

    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    SPI.transfer(0x7E | 0x40);
    SPI.transfer(paTable[0]);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    Serial.println("[PA TABLE] Loaded.");
}

bool CC2500Receiver::_readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out) {
    delayMicroseconds(100);

    uint8_t state = _readMARCState();
    Serial.printf("[DEBUG] MARCSTATE = 0x%02X\n", state);

    // Wait for GDO0 to indicate end of packet (RX complete)
    unsigned long start = millis();
    while (digitalRead(CC2500_GDO0_PIN) == LOW) {
        if (millis() - start > 100) {
            Serial.println("[TIMEOUT] GDO0 didn't trigger (RX complete not signaled)");
            return false;
        }
    }

    // Read RXBYTES twice to verify FIFO content (these use _readRegister which is transaction-safe)
    uint8_t bytes1, bytes2;
    do {
        bytes1 = _readRegister(0x3B) & 0x7F;
        bytes2 = _readRegister(0x3B) & 0x7F;
    } while (bytes1 != bytes2);

    if (bytes1 != CC2500_PACKET_SIZE) {
        Serial.printf("[FIFO] Unexpected packet size: %u bytes\n", bytes1);
        return false;
    }

    // Read FIFO in a single transaction
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return false;
    }

    SPI.transfer(RX_FIFO_BURST);
    for (uint8_t i = 0; i < CC2500_PACKET_SIZE; ++i) {
        buffer[i] = SPI.transfer(0x00);
    }

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    uint8_t rssi     = buffer[10];
    uint8_t lqi_crc  = buffer[11];
    int8_t rssi_dbm;
    uint8_t lqi;
    _processStatusBytes(rssi, lqi_crc, rssi_dbm, lqi, crcOk_out);

    Serial.printf("[RX] RSSI: %ddBm, LQI: %u, CRC: %s\n", rssi_dbm, lqi, crcOk_out ? "OK" : "FAIL");
    Serial.printf("[DEBUG] Expected: %d bytes, Got: %d bytes\n", CC2500_PACKET_SIZE, bytes1);

    len = CC2500_PACKET_SIZE;
    return true;
}

void CC2500Receiver::_processStatusBytes(uint8_t rssiByte, uint8_t lqiByte,
                                         int8_t& rssi_dBm, uint8_t& lqi, bool& crcOk) {
    if (rssiByte >= 128)
        rssi_dBm = ((int16_t)rssiByte - 256) / 2 - RSSI_OFFSET_250K;
    else
        rssi_dBm = rssiByte / 2 - RSSI_OFFSET_250K;

    lqi = lqiByte & 0x7F;
    crcOk = (lqiByte & 0x80);
}

bool CC2500Receiver::_verifyPacket(const uint8_t* data, uint8_t len, bool crcOk) {
    if (!crcOk || len != CC2500_PACKET_SIZE || data[0] != CC2500_START_BYTE) {
        Serial.println("[VERIFY] Packet failed verification.");
        return false;
    }
    return true;
}

uint8_t CC2500Receiver::_readMARCState() {
    return _readRegister(0x35) & 0x1F;
}

void CC2500Receiver::_configureRadio() {
    Serial.println("[CONFIG] Setting registers...");
    _writeRegister(0x00, 0x06);
    _writeRegister(0x02, 0x07);
    _writeRegister(0x03, 0x07);
    _writeRegister(0x04, 0xD3);
    _writeRegister(0x05, 0x91);
    _writeRegister(0x06, CC2500_DATA_BYTES);
    _writeRegister(0x07, 0x04);
    _writeRegister(0x08, 0x44);
    _writeRegister(0x0A, 0x07);
    _writeRegister(0x0B, 0x06);
    _writeRegister(0x0C, 0x00);
    _writeRegister(0x0D, 0x5D);
    _writeRegister(0x0E, 0x93);
    _writeRegister(0x0F, 0xB1);
    _writeRegister(0x10, 0xF8);
    _writeRegister(0x11, 0x93);
    _writeRegister(0x12, 0x13);
    _writeRegister(0x13, 0x22);
    _writeRegister(0x14, 0xF8);
    _writeRegister(0x15, 0x15);
    _writeRegister(0x17, 0x3C);
    _writeRegister(0x18, 0x18);
    _writeRegister(0x19, 0x1D);
    _writeRegister(0x1A, 0x1C);
    _writeRegister(0x1B, 0xC7);
    _writeRegister(0x1C, 0x00);
    _writeRegister(0x1D, 0xB0);
    _writeRegister(0x21, 0xB6);
    _writeRegister(0x22, 0x10);
    _writeRegister(0x23, 0xEA);
    _writeRegister(0x24, 0x2A);
    _writeRegister(0x25, 0x00);
    _writeRegister(0x26, 0x1F);
    Serial.println("[CONFIG] Configuration complete.");
}

void CC2500Receiver::_printConfigSummary() {
    Serial.printf("CHANNR: 0x%02X\n", _readRegister(0x0A));
    Serial.printf("MDMCFG2: 0x%02X\n", _readRegister(0x12));
    Serial.printf("PKTCTRL1: 0x%02X, PKTCTRL0: 0x%02X\n", _readRegister(0x07), _readRegister(0x08));
    Serial.printf("DEVIATN: 0x%02X\n", _readRegister(0x15));
    Serial.printf("SYNC: 0x%02X 0x%02X\n", _readRegister(0x04), _readRegister(0x05));
}

void CC2500Receiver::_select() { digitalWrite(_cs, LOW); }
void CC2500Receiver::_deselect() { digitalWrite(_cs, HIGH); }

uint8_t CC2500Receiver::_calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) sum ^= data[i];
    return sum;
}

void CC2500Receiver::_debugPrintPacket(const uint8_t* data, uint8_t len) {
    Serial.print("[DEBUG] Raw Packet: ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
    Serial.println();
}
