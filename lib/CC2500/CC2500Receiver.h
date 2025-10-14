#ifndef CC2500_RECEIVER_H
#define CC2500_RECEIVER_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include "Config.h" // Central config for pin definitions

// =============================== Pin Definitions ===============================
// Use Config.h values if provided, else fallback to defaults

// RX FIFO wait timeout in ms (keep small for drone use!)
#define CC2500_RX_TIMEOUT 5


#ifndef CC2500_CS_PIN
#define CC2500_CS_PIN 10
#endif

#ifndef CC2500_GDO0_PIN
#define CC2500_GDO0_PIN 14
#endif

#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN 12
#endif

#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN 13
#endif

#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN 11
#endif

// =============================== SPI Settings ===============================
#ifndef CC2500_SPI_SPEED
#define CC2500_SPI_SPEED 4000000
#endif

#ifndef CC2500_SPI_MODE
#define CC2500_SPI_MODE SPI_MODE0
#endif

// =============================== Packet Structure ===============================
#define CC2500_DATA_BYTES     11
#define CC2500_PACKET_SIZE    13  // 11 data + 2 status bytes

#define CC2500_START_BYTE     0x88
#define CC2500_RX_STATUS_SIZE 2
#define RSSI_OFFSET_250K      72
#define CC2500_RXBYTES 0x3B


// =============================== CC2500 Receiver Class ===============================
class CC2500Receiver {
public:
    // Constructor uses central config defaults
    CC2500Receiver(uint8_t csPin   = CC2500_CS_PIN,
                   uint8_t gdo0Pin = CC2500_GDO0_PIN,
                   uint8_t sckPin  = SPI_SCK_PIN,
                   uint8_t misoPin = SPI_MISO_PIN,
                   uint8_t mosiPin = SPI_MOSI_PIN);

    // Setup & operation
    void begin();
    bool receivePacket();
    uint8_t _readRegister(uint8_t addr);
    void _strobeCommand(uint8_t cmd);

    // Data access
    bool getLatestControlData(int8_t& yaw, int8_t& pitch, int8_t& roll,
                                          uint8_t& throttle, uint8_t& mode, uint8_t& armed,
                                          uint8_t& takeoff, uint8_t& failsafe,
                                          uint8_t& photo, uint8_t& video);
    bool isTimedOut(uint32_t timeoutMs = 200) const;

    // Frequency/channel tuning
    void setFrequency(uint32_t freq);
    void setChannel(uint8_t ch);
    // CC2500Receiver.h (add to public section)
    int8_t getLastRSSI() const { return _lastRssiDbm; }


private:
    // Pins
    uint8_t _cs, _gdo0, _sck, _miso, _mosi;
    bool _hasValidPacket = false;
    uint32_t _lastPacketTime = 0;   // tracks last valid packet timestamp
    uint32_t _expectedPackets = 0;  // expected count (based on TX interval)
    uint32_t _receivedPackets = 0;  // successfully received
    uint32_t _lostPackets = 0;      // estimated lost
    uint8_t _packet[CC2500_PACKET_SIZE];

    struct ControlPacket {
        int8_t yaw;
        int8_t pitch;
        int8_t roll;
        uint8_t throttle;
        uint8_t mode;
        bool takeoff;
        bool failsafe;
        bool photo;
        uint8_t video;
    } _lastPacket;

    // Internal helpers
    void _reset();
    bool _waitForChipReady();
    void _writeRegister(uint8_t addr, uint8_t value);
    void _configureRadio();
    void _loadPATable();
    bool _readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out ,
                              int8_t& rssi_dbm_out, uint8_t& lqi_out);
    bool _verifyPacket(const uint8_t* data, uint8_t len);
    bool _verifyPacket(const uint8_t* data, uint8_t len, bool crcOk);
    void _processStatusBytes(uint8_t rssiByte, uint8_t lqiByte, int8_t& rssi_dBm, uint8_t& lqi, bool& crcOk);
    uint8_t _calculateChecksum(const uint8_t* data, size_t length);
    void _debugPrintPacket(const uint8_t* data, uint8_t len);
    uint8_t _readMARCState();
    void _printConfigSummary();
    bool _waitForPacketGDO0();    // class member (in CC2500Receiver.h private section)
 int8_t _lastRssiDbm = -127; // add this as a private field

  bool _isFifoStale();

    
    // SPI helpers
    void _select();
    void _deselect();

    // inside class CC2500Receiver { ... private: for reset of cc2500 on Air 
uint8_t _gdo0TimeoutCount = 0;
static constexpr uint8_t GDO0_TIMEOUT_THRESHOLD = 5;

void _handleGDO0Timeouts();


        
};

#endif // CC2500_RECEIVER_H
