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

// --- CC2500 RF Module ---
#define CC2500_CS_PIN     10
#define CC2500_GDO0_PIN   14
#define CC2500_SPI_SPEED  4000000
#define CC2500_SPI_MODE   SPI_MODE0


// ===============================
// FIFO
// ===============================
#define FIFO_BUFFER_SIZE 512

// ===============================
// CC2500 Packet Structure
// ===============================
#define CC2500_DATA_BYTES     10
#define CC2500_PACKET_SIZE    12
#define CC2500_START_BYTE     0x88
#define CC2500_RX_STATUS_SIZE 2
#define RSSI_OFFSET_250K      72

// ===============================
// Debug
// ===============================
#define DEBUG_SERIAL_BAUD 115200

#endif // CONFIG_H
