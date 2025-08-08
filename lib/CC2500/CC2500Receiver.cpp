#include "CC2500Receiver.h"
#include <Arduino.h>

#define SRX    0x34
#define SIDLE  0x36
#define SFRX   0x3A
#define RX_FIFO_BURST 0xFF

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
    delayMicroseconds(20);  // Allow SPI  settle
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

    // Print full raw packet first
    Serial.print("[RX PACKET] ");
for (int i = 0; i < CC2500_DATA_BYTES; i++) {
    Serial.printf("0x%02X ", _packet[i]);
}
Serial.println();

// Extract values (only from first 10 bytes) 
yaw      = _packet[1];
pitch    = _packet[2];
roll     = _packet[3];
throttle = _packet[4];
mode     = _packet[5];
takeoff  = _packet[6];
failsafe = _packet[7];
photo    = _packet[8];
video    = _packet[9];

    // Print decoded fields
    Serial.printf("[RX DATA] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d TO=%d FS=%d PH=%d VID=%d\n",
        yaw, pitch, roll, throttle, mode, takeoff, failsafe, photo, video);

    return true;
}

bool CC2500Receiver::isTimedOut(uint32_t timeoutMs) const {
    return (millis() - _lastPacketTime) > timeoutMs;
}


void CC2500Receiver::_reset() {
    Serial.println("[RESET] Sending SRES...");
    _deselect(); delayMicroseconds(100);
    _select(); if (!_waitForChipReady()) return;
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    SPI.transfer(0x30);  // SRES
    SPI.endTransaction();
    _deselect();
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
    _select();
    _waitForChipReady();
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    SPI.transfer(addr & 0x3F);
    SPI.transfer(value);
    SPI.endTransaction();
    _deselect();
}

uint8_t CC2500Receiver::_readRegister(uint8_t addr) {
    _select();
    if (!_waitForChipReady()) return 0xFF;
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    uint8_t header = (addr >= 0x30) ? (addr | 0xC0) : (addr | 0x80);
    SPI.transfer(header);
    uint8_t value = SPI.transfer(0x00);
    SPI.endTransaction();
    _deselect();
    return value;
}

void CC2500Receiver::_strobeCommand(uint8_t cmd) {
    _select();
    _waitForChipReady();
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    SPI.transfer(cmd);
    SPI.endTransaction();
    _deselect();
}

void CC2500Receiver::_loadPATable() {
    const uint8_t paTable[1] = { 0xFF };
    _select();
    _waitForChipReady();
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    SPI.transfer(0x7E | 0x40);
    SPI.transfer(paTable[0]);
    SPI.endTransaction();
    _deselect();
    Serial.println("[PA TABLE] Loaded.");
}

bool CC2500Receiver::_readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out) {
    delayMicroseconds(100); // Allow SPI and FIFO sync

    uint8_t state = _readMARCState();
    Serial.printf("[DEBUG] MARCSTATE = 0x%02X\n", state);

    // ✅ Wait for end of packet via GDO0
    unsigned long start = millis();
    while (digitalRead(CC2500_GDO0_PIN) == LOW) {
        if (millis() - start > 100) {
            Serial.println("[TIMEOUT] GDO0 didn't trigger (RX complete not signaled)");
            return false;
        }
    }

    // Read RXBYTES twice to verify FIFO content
    uint8_t bytes1, bytes2;
    do {
        bytes1 = _readRegister(0x3B) & 0x7F;
        bytes2 = _readRegister(0x3B) & 0x7F;
    } while (bytes1 != bytes2);

    if (bytes1 != CC2500_PACKET_SIZE) {
        Serial.printf("[FIFO] Unexpected packet size: %u bytes\n", bytes1);
        return false;
    }

    _select();
    if (!_waitForChipReady()) {
        _deselect(); return false;
    }

    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    SPI.transfer(RX_FIFO_BURST);

    // Read exactly 12 bytes: 10 data + 2 status
    for (uint8_t i = 0; i < CC2500_PACKET_SIZE; ++i) {
        buffer[i] = SPI.transfer(0x00);
    }

    SPI.endTransaction();
    _deselect();

    // Extract status bytes
    uint8_t rssi     = buffer[10];
    uint8_t lqi_crc  = buffer[11];

    // Decode RSSI, LQI, CRC
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

    _writeRegister(0x00, 0x06); // IOCFG2
    _writeRegister(0x02, 0x07); // IOCFG0: GDO0 = Sync Word received ,Rx complete

    _writeRegister(0x03, 0x07); // FIFOTHR

    _writeRegister(0x04, 0xD3); // SYNC1
    _writeRegister(0x05, 0x91); // SYNC0

    _writeRegister(0x06, CC2500_DATA_BYTES); // PKTLEN = 10
    _writeRegister(0x07, 0x04);  // PKTCTRL1: CRC_AUTOFLUSH=1, APPEND_STATUS=1, ADR_CHK=0

    _writeRegister(0x08, 0x44); // PKTCTRL0: Fixed length, CRC enabled, whiteing on

   
    _writeRegister(0x0A, 0x07); // CHANNR = 7

    _writeRegister(0x0B, 0x06); // FSCTRL1: IF
    _writeRegister(0x0C, 0x00); // FSCTRL0

 _writeRegister(0x0D, 0x5D); // FREQ2: Set high byte for 2433 MHz [2, 4]
_writeRegister(0x0E, 0x93); // FREQ1: Set middle byte for 2433 MHz [3, 4]
_writeRegister(0x0F, 0xB1); // FREQ0: Set low byte for 2433 MHz [3, 4]

    _writeRegister(0x10, 0xF8); // MDMCFG4 58 kHz
    _writeRegister(0x11, 0x93); // MDMCFG3
    _writeRegister(0x12, 0x13); // MDMCFG2: GFSK, 32-bit sync
    _writeRegister(0x13, 0x22); // MDMCFG1
    _writeRegister(0x14, 0xF8); // MDMCFG0

    _writeRegister(0x15, 0x15); // DEVIATN

    _writeRegister(0x17, 0x3C); // RX stays in RX after receiving, CCA_MODE = 11

    _writeRegister(0x18, 0x18); // MCSM0: FS_AUTOCAL (idle→RX/TX)

    _writeRegister(0x19, 0x1D); // FOCCFG
    _writeRegister(0x1A, 0x1C); // BSCFG
    _writeRegister(0x1B, 0xC7); // AGCCTRL2
    _writeRegister(0x1C, 0x00); // AGCCTRL1
    _writeRegister(0x1D, 0xB0); // AGCCTRL0

    _writeRegister(0x21, 0xB6); // FREND1
    _writeRegister(0x22, 0x10); // FREND0

    _writeRegister(0x23, 0xEA); // FSCAL3
    _writeRegister(0x24, 0x2A); // FSCAL2
    _writeRegister(0x25, 0x00); // FSCAL1 (optional)
    _writeRegister(0x26, 0x1F); // FSCAL0

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
