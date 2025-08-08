#ifndef CC2500_RECEIVER_H
#define CC2500_RECEIVER_H

#include <Arduino.h>
#include <SPI.h>

// SPI Pin Definitions

#ifndef CS_PIN
#define CS_PIN 8
#endif


#define SCK_PIN   12
#define MISO_PIN  13
#define MOSI_PIN  11
#define GDO0_PIN  14

// SPI Settings
#define CC2500_SPI_SPEED 6500000
#define CC2500_SPI_MODE  SPI_MODE3

// Packet Structure

#define CC2500_DATA_BYTES 10
#define CC2500_PACKET_SIZE 12  // 10 + 2 (status)


#define CC2500_START_BYTE        0x88
#define CC2500_RX_STATUS_SIZE    2
#define RSSI_OFFSET_250K         72

class CC2500Receiver {
public:
    CC2500Receiver(uint8_t csPin, uint8_t sckPin, uint8_t misoPin, uint8_t mosiPin);
    void begin();
    bool receivePacket();
    bool getLatestControlData(int8_t& yaw, int8_t& pitch, int8_t& roll,
                              uint8_t& throttle, uint8_t& mode, uint8_t& takeoff,
                              uint8_t& failsafe, uint8_t& photo, uint8_t& video);
    bool isTimedOut(uint32_t timeoutMs = 200) const;


private:
    uint8_t _cs, _sck, _miso, _mosi;
    uint8_t _packet[CC2500_PACKET_SIZE];
    uint32_t _lastPacketTime;
    bool _hasValidPacket;

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
    uint8_t _readRegister(uint8_t addr);
    void _strobeCommand(uint8_t cmd);
    void _configureRadio();
    void _loadPATable();
    bool _readRXFIFO(uint8_t* buffer, uint8_t& len, bool& crcOk_out);
    bool _verifyPacket(const uint8_t* data, uint8_t len);
    bool _verifyPacket(const uint8_t* data, uint8_t len, bool crcOk);
    void _processStatusBytes(uint8_t rssiByte, uint8_t lqiByte, int8_t& rssi_dBm, uint8_t& lqi, bool& crcOk);
    uint8_t _calculateChecksum(const uint8_t* data, size_t length);
    void _debugPrintPacket(const uint8_t* data, uint8_t len);
    uint8_t _readMARCState();
    void _printConfigSummary();
    // SPI helpers
    void _select();
    void _deselect();
};

#endif
