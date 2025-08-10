#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =================== Pin Definitions ===================
#define CC2500_CS_PIN     10
#define CC2500_GDO0_PIN   14
#define SPI_SCK_PIN       12
#define SPI_MISO_PIN      13
#define SPI_MOSI_PIN      11

// =================== CC2500 SPI Settings ===================
#define CC2500_SPI_SPEED  6500000
#define CC2500_SPI_MODE   SPI_MODE3

// =================== Packet Structure ===================
#define CC2500_DATA_BYTES     10
#define CC2500_PACKET_SIZE    12
#define CC2500_START_BYTE     0x88
#define CC2500_RX_STATUS_SIZE 2
#define RSSI_OFFSET_250K      72

#endif // CONFIG_H
