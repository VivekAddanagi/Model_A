#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===============================
// Pin Definitions
// ===============================

// --- SPI pins (shared bus) ---
#define SPI_SCK_PIN   12
#define SPI_MISO_PIN  13
#define SPI_MOSI_PIN  11

// --- BMI323 IMU ---
#define BMI323_CS_PIN 8
#define BMI323_SPI_MODE SPI_MODE0
#define BMI323_SPI_SPEED 4000000  // Match CC2500 speed for bus sharing if needed
#define BMI323_INT_PIN 9

// --- CC2500 RF Module ---
#define CC2500_CS_PIN     10
#define CC2500_GDO0_PIN   14
#define CC2500_SPI_SPEED  6500000
#define CC2500_SPI_MODE   SPI_MODE0


// ===============================
// FIFO
// ===============================
//#ifndef FIFO_BUFFER_SIZE
//#define FIFO_BUFFER_SIZE 512
//#endif

// ===============================
// CC2500 Packet Structure
// ===============================
#define CC2500_DATA_BYTES     11
#define CC2500_PACKET_SIZE    13
#define CC2500_START_BYTE     0x88
#define CC2500_RX_STATUS_SIZE 2
#define RSSI_OFFSET_250K      72

// ===============================
// Debug
// ===============================
#ifndef DEBUG_SERIAL_BAUD
#define DEBUG_SERIAL_BAUD 115200
#endif

// ===============================
// BMP390 specific overrides & defaults
// (these may override defaults in bmp390.h)
// ===============================
#ifndef I2C_MASTER_SDA_IO
#define I2C_MASTER_SDA_IO             17
#endif

#ifndef I2C_MASTER_SCL_IO
#define I2C_MASTER_SCL_IO             18
#endif

#ifndef I2C_MASTER_FREQ_HZ
#define I2C_MASTER_FREQ_HZ            400000
#endif

// Interrupt pin used for BMP390 FIFO watermark/full (change if needed)
#ifndef BMP390_INT_PIN
#define BMP390_INT_PIN                21
#endif

// EEPROM storage slot (bytes) for BMP390 calibration
#ifndef CALIB_EEPROM_ADDR
#define CALIB_EEPROM_ADDR 32  // leave room for other data in EEPROM
#endif

// EEPROM storage size to reserve when using EEPROM.begin()
#ifndef BMP390_EEPROM_SIZE
#define BMP390_EEPROM_SIZE 128
#endif

// How long calibration remains valid (seconds). Default 24h
#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION 86400UL
#endif

#endif // CONFIG_H
