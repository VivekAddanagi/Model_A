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
    // Ensure BMI CS is de-asserted before asserting CC2500 CS
#ifdef BMI323_CS_PIN
    digitalWrite(BMI323_CS_PIN, HIGH);
#endif
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    uint8_t status = SPI.transfer(addr & 0x3F); // capture status byte (returned during header)
    (void)status; // currently unused but read per CC2500 SPI rules
    SPI.transfer(value);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}


uint8_t CC2500Receiver::_readRegister(uint8_t addr) {
    // Ensure BMI CS is de-asserted before asserting CC CS
#ifdef BMI323_CS_PIN
    digitalWrite(BMI323_CS_PIN, HIGH);
#endif
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return 0xFF;
    }

    uint8_t header = (addr >= 0x30) ? (addr | 0xC0) : (addr | 0x80);
    uint8_t status = SPI.transfer(header); // status byte returned simultaneously
    (void)status; // keep for later debugging/interpretation if needed
    uint8_t value = SPI.transfer(0x00);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return value;
}

void CC2500Receiver::_strobeCommand(uint8_t cmd) {
    // Ensure BMI CS is de-asserted before asserting CC CS
#ifdef BMI323_CS_PIN
    digitalWrite(BMI323_CS_PIN, HIGH);
#endif
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    uint8_t status = SPI.transfer(cmd); // capture status byte
    (void)status;

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
}


void CC2500Receiver::_loadPATable() {
    const uint8_t paTable[1] = { 0xFF };

#ifdef BMI323_CS_PIN
    digitalWrite(BMI323_CS_PIN, HIGH);
#endif
    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);

    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        return;
    }

    uint8_t status = SPI.transfer(0x7E | 0x40);
    (void)status;
    SPI.transfer(paTable[0]);

    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

   // Serial.println("[CC2500 PA TABLE] Loaded.");
}

void CC2500Receiver::_reset() {
   // Serial.println("[CC2500 RESET] Starting enhanced reset sequence...");
    
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
#ifdef BMI323_CS_PIN
    digitalWrite(BMI323_CS_PIN, HIGH);
#endif
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
    
    uint8_t status = SPI.transfer(0x30); // SRES (capture status)
    (void)status;
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
       // Serial.println("[CC2500 RESET] Successful");
    }
}


void CC2500Receiver::begin() {
   // Serial.println("[CC2500] Initializing SPI and GPIO...");
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
  //  _printConfigSummary();
    _loadPATable();
    _strobeCommand(SRX);
   // Serial.println("[CC2500] Initialization complete.");
}

bool CC2500Receiver::getLatestControlData(int8_t& yaw, int8_t& pitch, int8_t& roll,
                                          uint8_t& throttle, uint8_t& mode, uint8_t& armed,
                                          uint8_t& takeoff, uint8_t& failsafe,
                                          uint8_t& photo, uint8_t& video) {
    if (!_hasValidPacket) return false;

    /*
    Serial.print("[RX PACKET] ");
    for (int i = 0; i < CC2500_DATA_BYTES; i++) {
        Serial.printf("0x%02X ", _packet[i]);
    }
    Serial.println();
    */

    yaw      = (int8_t)_packet[2];
    pitch    = (int8_t)_packet[3];
    roll     = (int8_t)_packet[4];
    throttle = _packet[5];
    mode     = _packet[6];
    armed    = _packet[7];
    takeoff  = _packet[8];
    failsafe = _packet[9];
    photo    = _packet[10];
    video    = _packet[11];

    
   // Serial.printf("[RX DATA] YAW=%d PITCH=%d ROLL=%d THR=%d MODE=%d ARM=%d TO=%d FS=%d PH=%d VID=%d\n",
                //  yaw, pitch, roll, throttle, mode, armed, takeoff, failsafe, photo, video);
    

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

bool CC2500Receiver::_readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out ,
     int8_t& rssi_dbm_out, uint8_t& lqi_out) {
    uint8_t rxBytes = _readRegister(0x3B) & 0x7F;
    if (rxBytes == 0) {
      // Serial.println("[DEBUG RXFIFO] No bytes available in RX FIFO");
        return false;
    }
    // Serial.printf("[DEBUG RXFIFO] RXBYTES=%u\n", rxBytes);

    SPI.beginTransaction(SPISettings(CC2500_SPI_SPEED, MSBFIRST, CC2500_SPI_MODE));
    digitalWrite(_cs, LOW);
    if (!_waitForChipReady()) {
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
       // Serial.println("[DEBUG RXFIFO] Chip not ready");
        return false;
    }

    SPI.transfer(RX_FIFO_BURST);

    // First byte = payload length
    uint8_t pktLen = SPI.transfer(0x00);
   // Serial.printf("[DEBUG RXFIFO] Length byte received=%u\n", pktLen);

    // ✅ Safety check against buffer overflow (payload-based)
    const uint8_t MAX_PAYLOAD = sizeof(_packet) - 1; // exclude length byte
    if (pktLen == 0 || pktLen > MAX_PAYLOAD) {
      //  Serial.printf("[ERROR RXFIFO] Invalid pktLen=%u (max=%u), flushing FIFO!\n",
                     // pktLen, MAX_PAYLOAD);
        digitalWrite(_cs, HIGH);
        SPI.endTransaction();
        _strobeCommand(SIDLE);
        _strobeCommand(SFRX);
        _strobeCommand(SRX);
        return false;
    }

    // Store length byte
    buffer[0] = pktLen; 
   // Serial.printf("[DEBUG RXFIFO] Reading %u payload bytes...\n", pktLen);

    // Read payload
    for (uint8_t i = 1; i <= pktLen; i++) {
        buffer[i] = SPI.transfer(0x00);
    }

    // Status bytes (always 2: RSSI + LQI/CRC)
    uint8_t rssi = SPI.transfer(0x00);
    uint8_t lqi_crc = SPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();

    // Debug print entire packet
   // Serial.print("[DEBUG RXFIFO] Raw packet: ");
   // for (uint8_t i = 0; i <= pktLen; i++) {
       // Serial.printf("0x%02X ", buffer[i]);
 //   }
   // Serial.printf("| RSSI=0x%02X LQI/CRC=0x%02X\n", rssi, lqi_crc);

  // Process status and pass up
_processStatusBytes(rssi, lqi_crc, rssi_dbm_out, lqi_out, crcOk_out);

// Serial.printf("[DEBUG RXFIFO] RSSI=%d dBm LQI=%u CRC_OK=%d\n",
           //   rssi_dbm_out, lqi_out, crcOk_out);


    // Reported length = 1 (length byte) + payload
    len = pktLen + 1;
   // Serial.printf("[DEBUG RXFIFO] Final reported length=%u (len byte + payload)\n", len);

    return true;
}



bool CC2500Receiver::receivePacket() {
    if (!_waitForPacketGDO0()) return false;
    bool anyPacketReceived = false;
    uint8_t len = 0;
    bool crcOk;

    // Track print timing (static persists across calls)
    static unsigned long lastPrintTime = 0;
    static long rssiSum = 0;
    static long lqiSum = 0;
    static uint16_t rssiCount = 0;

    // Failure counters (persistent)
    static uint32_t crcFails = 0;
    static uint32_t lenFails = 0;
    static uint32_t startByteFails = 0;

   int8_t rssi_dbm;
   uint8_t lqi;

if (!_readRXFIFO(_packet, len, crcOk, rssi_dbm, lqi)) {
    Serial.println("[DEBUG RECEIVE] No packet read from FIFO");
    return false;
}

  //  Serial.printf("[DEBUG RECEIVE] Packet length=%u, CRC=%d\n", len, crcOk);

    if (!_verifyPacket(_packet, len, crcOk)) {
        Serial.printf("[DEBUG RECEIVE] Packet verify failed (len=%u, crc=%d, start=0x%02X)\n",
                      len, crcOk, _packet[1]);
        return false; // invalid packet, skip
    }

    // ✅ Valid packet
    _receivedPackets++;
    _expectedPackets++;

    unsigned long currentTime = millis();
    unsigned long gap = (_lastPacketTime == 0) ? 0 : (currentTime - _lastPacketTime);
    _lastPacketTime = currentTime;

    // Detect missed packets (gap > 1.5 × interval)
    if (gap > (25 + 12)) {
        uint16_t missed = (gap / 25) - 1;
        _lostPackets += missed;
        _expectedPackets += missed;
    }

    _hasValidPacket = true;
    anyPacketReceived = true;

    // Extract payload
    int8_t yaw   = (int8_t)_packet[2];
    int8_t pitch = (int8_t)_packet[3];
    int8_t roll  = (int8_t)_packet[4];
    uint8_t thr  = _packet[5];
    uint8_t mode = _packet[6];
    bool arm     = _packet[7];
    bool to      = _packet[8];
    bool fs      = _packet[9];
    bool ph      = _packet[10];
    bool vid     = _packet[11];

  // Status bytes
// int8_t rssi_dbm;
// uint8_t lqi;
// _processStatusBytes(_packet[12], _packet[13], rssi_dbm, lqi, crcOk);

// Store last RSSI (for external modules)
_lastRssiDbm = rssi_dbm;

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
                      "RSSI(avg)=%d dBm LQI(avg)=%u Δt=%lu ms "
                      "| CRC_FAIL=%lu LEN_FAIL=%lu SB_FAIL=%lu\n",
                      _receivedPackets, _lostPackets, lossRate,
                      avgRssi, avgLqi, gap,
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
   

    // Flush FIFO only if stale data detected
if (_isFifoStale()) {
    _strobeCommand(SIDLE);
    _strobeCommand(SFRX);
    _strobeCommand(SRX);
    }


    return anyPacketReceived;
}

bool CC2500Receiver::_waitForPacketGDO0() {
    uint32_t timeoutMs = 50; // internal timeout
    uint32_t start = millis();
    while (digitalRead(_gdo0) == LOW) { // Wait for GDO0 to go HIGH
        if (millis() - start > timeoutMs) {
            Serial.println("[DEBUG] GDO0 timeout");
            return false; // timeout
        }
    }
    return true; // packet ready
}


bool CC2500Receiver::_isFifoStale() {
    uint8_t rxBytes = _readRegister(CC2500_RXBYTES);
    bool overflow = rxBytes & 0x80;
    uint8_t numBytes = rxBytes & 0x7F;

    uint8_t maxLen = _readRegister(0x06); // PKTLEN
    uint8_t allowedMax = maxLen + 1 + 2;

   // Serial.printf("[DEBUG FIFO] RXBYTES=%u (overflow=%d), allowedMax=%u\n",
                //  numBytes, overflow, allowedMax);

    return (overflow || (numBytes > allowedMax));
}

bool CC2500Receiver::_verifyPacket(const uint8_t* data, uint8_t len, bool crcOk) {
    if (!crcOk) {
        Serial.println("[DEBUG VERIFY] CRC failed");
        return false;
    }

    uint8_t pktLen = data[0]; // first byte is length
    if (len < pktLen + 1) {
        Serial.printf("[DEBUG VERIFY] Length mismatch (len=%u, pktLen=%u)\n", len, pktLen);
        return false;
    }

    // Optional start byte check
     if (data[1] != CC2500_START_BYTE) {
        Serial.printf("[DEBUG VERIFY] Start byte mismatch (got=0x%02X)\n", data[1]);
         return false;
     }

    return true;
}

void CC2500Receiver::_configureRadio() {
   // Serial.println("[CC2500 CONFIG] Setting registers...");
   // _writeRegister(0x00, 0x06);
    _writeRegister(0x02, 0x06);
    _writeRegister(0x03, 0x07);
    _writeRegister(0x04, 0xD3);
    _writeRegister(0x05, 0x91);
    _writeRegister(0x06, 50);
    _writeRegister(0x07, 0x04);
    _writeRegister(0x08, 0x45);
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
    _writeRegister(0x2C, 0x81);
    _writeRegister(0x2D, 0x35);
   // Serial.println("[CC2500 CONFIG] Configuration complete.");
}

/*

bool CC2500Receiver::_verifyPacket(const uint8_t* data, uint8_t len, bool crcOk) {
    if (!crcOk) return false;

    uint8_t pktLen = data[0]; // first byte is length
   // if (len != pktLen + 1) return false; // mismatch
   if (len < pktLen + 1) return false;  // must have enough bytes
  // if (data[1] != CC2500_START_BYTE) return false; // check start byte

    return true;
}
*/


void CC2500Receiver::_printConfigSummary() {
    Serial.printf("CHANNR: 0x%02X\n", _readRegister(0x0A));
    Serial.printf("MDMCFG2: 0x%02X\n", _readRegister(0x12));
    Serial.printf("PKTCTRL1: 0x%02X, PKTCTRL0: 0x%02X\n", _readRegister(0x07), _readRegister(0x08));
    Serial.printf("DEVIATN: 0x%02X\n", _readRegister(0x15));
    Serial.printf("SYNC: 0x%02X 0x%02X\n", _readRegister(0x04), _readRegister(0x05));
}

void CC2500Receiver::_select() { digitalWrite(_cs, LOW); }
void CC2500Receiver::_deselect() { digitalWrite(_cs, HIGH); }


/*
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
*/

/*
bool CC2500Receiver::_isFifoStale() {
    uint8_t rxBytes = _readRegister(CC2500_RXBYTES);  
    // Bit 7 = RX FIFO overflow flag
    // Bits 6:0 = number of bytes in RX FIFO
    bool overflow = rxBytes & 0x80;
    uint8_t numBytes = rxBytes & 0x7F;

    // Stale if overflow OR if unexpected size
    return (overflow || (numBytes > CC2500_PACKET_SIZE));
}
    */