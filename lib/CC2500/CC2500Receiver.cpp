#include "CC2500Receiver.h"
#include <Arduino.h>
#include <SPI.h>

static SPISettings cc2500_spi_settings(CC2500_SPI_SPEED, MSBFIRST,CC2500_SPI_MODE ); // CC2500 often mode 0, can try mode 3 if needed


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


bool CC2500Receiver::_waitForChipReady() {
    uint32_t start = micros();
    uint32_t timeout = 5000; // 5ms
    
    while(micros() - start < timeout) {
        if(digitalRead(_miso) == LOW) {
            // Only print debug if waiting took significant time
            if(micros() - start > 100) {
                Serial.printf("[CC2500 DEBUG] MISO ready after %d μs\n", micros() - start);
            }
            return true;
        }
        delayMicroseconds(10);
    }
    
    Serial.printf("[CC2500 ERROR] Timeout waiting for MISO (state: %d)\n", digitalRead(_miso));
    return false;
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

    Serial.println("[CC2500 PA TABLE] Loaded.");
}

void CC2500Receiver::_reset() {
    Serial.println("[CC2500 RESET] Starting enhanced reset sequence...");
    
    // 1. Put chip in IDLE state first
    _strobeCommand(SIDLE);
    delayMicroseconds(100);
    
    // 2. Flush RX FIFO
    _strobeCommand(SFRX);
    delayMicroseconds(100);
    
    // 3. Hardware reset pulse
    digitalWrite(_cs, HIGH);
    delay(1);
    digitalWrite(_cs, LOW);
    delayMicroseconds(100);
    digitalWrite(_cs, HIGH);
    delay(5);  // Extended delay for full reset
    
    // 4. Software reset command with proper MISO handling
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, CC2500_SPI_MODE)); // Slow speed for reset
    digitalWrite(_cs, LOW);
    
    // Wait for MISO to go low (with inverted logic check if needed)
    uint32_t timeout = micros() + 5000; // 5ms timeout
    while(digitalRead(_miso) != LOW) {  // Change to HIGH if your hardware inverts
        if(micros() > timeout) {
            Serial.println("[CC2500 WARNING] MISO not low after CS asserted");
            break;
        }
    }
    
    SPI.transfer(0x30); // SRES
    delayMicroseconds(100);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    
    // 5. Post-reset stabilization
    delay(10);
    
    // 6. Verify reset completed
    uint8_t partnum = _readRegister(0x30);
    if(partnum != 0x80) {
        Serial.printf("[CC2500 ERROR] Reset failed - PARTNUM: 0x%02X (expected 0x80)\n", partnum);
    } else {
        Serial.println("[CC2500 RESET] Successful");
    }
}

void CC2500Receiver::begin() {
    Serial.println("[CC2500] Initializing SPI and GPIO...");
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CC2500_CS_PIN);

    pinMode(_cs, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(CC2500_GDO0_PIN, INPUT);

#if defined(ESP32)
    Serial.printf("[CC2500 DEBUG] GDO0 pin mode: %d, level: %d\n",
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

bool CC2500Receiver::getLatestControlData(int8_t& yaw, int8_t& pitch, int8_t& roll,
                                          uint8_t& throttle, uint8_t& mode, uint8_t& armed,
                                          uint8_t& takeoff, uint8_t& failsafe,
                                          uint8_t& photo, uint8_t& video) {
    if (!_hasValidPacket) return false;

    Serial.print("[RX PACKET] ");
    for (int i = 0; i < CC2500_DATA_BYTES; i++) {
        Serial.printf("0x%02X ", _packet[i]);
    }
    Serial.println();

   yaw      = (int8_t)_packet[1];
pitch    = (int8_t)_packet[2];
roll     = (int8_t)_packet[3];
throttle = _packet[4];
mode     = _packet[5];
armed    = _packet[6];
takeoff  = _packet[7];
failsafe = _packet[8];
photo    = _packet[9];
video    = _packet[10];

Serial.printf("[RX DATA] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d ARM=%d TO=%d FS=%d PH=%d VID=%d\n",
              yaw, pitch, roll, throttle, mode, armed, takeoff, failsafe, photo, video);

    return true;
}

bool CC2500Receiver::isTimedOut(uint32_t timeoutMs) const {
    return (millis() - _lastPacketTime) > timeoutMs;
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

uint8_t CC2500Receiver::_readMARCState() {
    return _readRegister(0x35) & 0x1F;
}


bool CC2500Receiver::_readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out) {
    // Wait for full packet
    unsigned long t0 = millis();
    uint8_t rxBytes = 0;
    while (millis() - t0 < 500) {
        rxBytes = _readRegister(0x3B) & 0x7F;
        if (rxBytes >= CC2500_PACKET_SIZE) break;
        delay(1);
    }

    if (rxBytes < CC2500_PACKET_SIZE) return false;

    // Read exactly one packet
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

    // Process RSSI and LQI
    uint8_t rssi     = buffer[11];
    uint8_t lqi_crc  = buffer[12];
    int8_t rssi_dbm;
    uint8_t lqi;
    _processStatusBytes(rssi, lqi_crc, rssi_dbm, lqi, crcOk_out);

    len = CC2500_PACKET_SIZE;
    return true;   // 🔹 no flush here
}

bool CC2500Receiver::receivePacket() {
    bool anyPacketReceived = false;
    uint8_t len = 0;
    bool crcOk;

    // Track print timing
    static unsigned long lastPrintTime = 0;
    static long rssiSum = 0;
    static long lqiSum = 0;
    static uint16_t rssiCount = 0;

    // Failure counters
    static uint32_t crcFails = 0;
    static uint32_t lenFails = 0;
    static uint32_t startByteFails = 0;

    while (true) {
        if (!_readRXFIFO(_packet, len, crcOk)) break;
        if (!_verifyPacket(_packet, len, crcOk)) {
            if (!crcOk) crcFails++;
            else if (len != CC2500_PACKET_SIZE) lenFails++;
            else if (_packet[0] != CC2500_START_BYTE) startByteFails++;
            continue;
        }

        // Packet is valid → update counters
        _receivedPackets++;
        _expectedPackets++;

        unsigned long currentTime = millis();
        unsigned long gap = (_lastPacketTime == 0) ? 0 : (currentTime - _lastPacketTime);
        _lastPacketTime = currentTime;

        // Infer missed packets from gap
        if (gap > (24 + 12)) {   // tolerance = 1.5 × interval
            uint16_t missed = (gap / 24) - 1;
            _lostPackets += missed;
            _expectedPackets += missed;
        }

        _hasValidPacket = true;
        anyPacketReceived = true;

        // Extract payload
        int8_t yaw   = (int8_t)_packet[1];
        int8_t pitch = (int8_t)_packet[2];
        int8_t roll  = (int8_t)_packet[3];
        uint8_t thr  = _packet[4];
        uint8_t mode = _packet[5];
        bool arm     = _packet[6];
        bool to      = _packet[7];
        bool fs      = _packet[8];
        bool ph      = _packet[9];
        bool vid     = _packet[10];

        int8_t rssi_dbm;
        uint8_t lqi;
        _processStatusBytes(_packet[11], _packet[12], rssi_dbm, lqi, crcOk);

        // Accumulate RSSI/LQI for averaging
        rssiSum += rssi_dbm;
        lqiSum += lqi;
        rssiCount++;

        // Print once every 500 ms
        if (currentTime - lastPrintTime >= 500) {
            lastPrintTime = currentTime;

            float lossRate = (_expectedPackets > 0) ? 
                              (100.0f * _lostPackets / _expectedPackets) : 0.0f;

            int avgRssi = (rssiCount > 0) ? (rssiSum / rssiCount) : 0;
            int avgLqi  = (rssiCount > 0) ? (lqiSum / rssiCount) : 0;

            Serial.printf("[RX STATS] OK=%lu FAIL=%lu LOSS=%.1f%% "
                          "RSSI(avg)=%d dBm LQI(avg)=%u "
                          "| CRC_FAIL=%lu LEN_FAIL=%lu SB_FAIL=%lu\n",
                          _receivedPackets, _lostPackets, lossRate,
                          avgRssi, avgLqi,
                          crcFails, lenFails, startByteFails);

            // Reset accumulators
            rssiSum = 0;
            lqiSum = 0;
            rssiCount = 0;

            /*
            // Optional detailed packet print
            Serial.printf("[RX DATA] YAW=%d PITCH=%d ROLL=%d THR=%u MODE=%u ARM=%u TO=%u FS=%u PH=%u VID=%u "
                          "| RSSI=%ddBm LQI=%u Δt=%lu ms | Loss=%lu/%lu (%.1f%%)\n",
                          yaw, pitch, roll, thr, mode, arm, to, fs, ph, vid,
                          rssi_dbm, lqi, gap,
                          _lostPackets, _expectedPackets, lossRate);
            */
        }
    }

    // Flush FIFO once at end
    _strobeCommand(SIDLE);
    _strobeCommand(SFRX);
    _strobeCommand(SRX);

    return anyPacketReceived;
}

bool CC2500Receiver::_verifyPacket(const uint8_t* data, uint8_t len, bool crcOk) {
    if (!crcOk || len != CC2500_PACKET_SIZE || data[0] != CC2500_START_BYTE) {
        // Minimal debug: failures are counted, not printed per packet
        /*
        Serial.printf("[VERIFY DEBUG] Len=%u, CRC_OK=%d, StartByte=0x%02X\n", len, crcOk, data[0]);
        Serial.print("[VERIFY DATA] ");
        for (int i = 0; i < len; i++) {
            Serial.printf("0x%02X ", data[i]);
        }
        Serial.println();
        */
        return false;
    }
    return true;
}


void CC2500Receiver::_configureRadio() {
    Serial.println("[CC2500 CONFIG] Setting registers...");
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
    Serial.println("[CC2500 CONFIG] Configuration complete.");
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
    Serial.print("[CC2500 DEBUG] Raw Packet: ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
    Serial.println();
}
