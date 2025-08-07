#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//
// ────────────────────────────────────────────────────────────────
//   PLATFORM & I/O CONFIGURATION
// ────────────────────────────────────────────────────────────────
//

// === I2C (Barometer BMP390) ===
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_MASTER_FREQ_HZ    400000  // 400 kHz

// === SPI Shared Bus (BMI323 & CC2500) ===
#define SPI_SCK_PIN           12
#define SPI_MISO_PIN          13
#define SPI_MOSI_PIN          11

// === BMI323 IMU ===
#define BMI323_CS_PIN         8
#define BMI323_SPI_SPEED      6500000

// === CC2500 Receiver ===
#define CC2500_CS_PIN         10
#define CC2500_GDO0_PIN       14
#define CC2500_SPI_SPEED      6500000
#define CC2500_SPI_MODE       SPI_MODE3

// === LED Indicators ===
#define FRONT_LED_PIN         5
#define REAR_LED_PIN          43

// === LED PWM Configuration ===
#define FRONT_LED_PWM_CHANNEL 0
#define FRONT_LED_PWM_FREQ    5000
#define FRONT_LED_PWM_RES     8

// === Motor Pins (PWM) ===
#define MOTOR_FL_PIN          25
#define MOTOR_FR_PIN          26
#define MOTOR_BL_PIN          27
#define MOTOR_BR_PIN          32

// === IR Obstacle Avoidance ===
#define IR_RECEIVER_PIN       1
#define IR_EMITTER_FRONT      40
#define IR_EMITTER_RIGHT      41
#define IR_EMITTER_BACK       38
#define IR_EMITTER_LEFT       39

//
// ────────────────────────────────────────────────────────────────
//   FLIGHT SYSTEM SETTINGS
// ────────────────────────────────────────────────────────────────
//

// === Flight Loop Timing ===
#define LOOP_UPDATE_INTERVAL_MS   10    // 100Hz main update

// === Calibration Durations ===
#define CALIB_VALID_DURATION      86400UL  // 24 hours in seconds

// === IR Sensor Logic ===
#define IR_SETTLE_TIME_US         200
#define IR_READ_DELAY_US          300
#define IR_POLL_INTERVAL_MS       25
#define IR_DANGER_THRESHOLD       2000

// === CC2500 Packet Protocol ===
#define CC2500_START_BYTE         0x88
#define CC2500_PACKET_SIZE        12
#define CC2500_DATA_BYTES         10
#define RSSI_OFFSET_250K          72

//
// ────────────────────────────────────────────────────────────────
//   EEPROM STORAGE CONFIG
// ────────────────────────────────────────────────────────────────
//

#define BMP390_EEPROM_SIZE        64
#define CALIB_EEPROM_ADDR         0   // Start at address 0

//
// ────────────────────────────────────────────────────────────────
//   BMP390 Configuration Registers (used in bmp390.h too)
// ────────────────────────────────────────────────────────────────
//

#define BMP390_INT_PIN            15  // INT pin for FIFO interrupt (assign if used)

// FIFO configuration
#define BMP390_FIFO_MAX_SIZE      512

//
// ────────────────────────────────────────────────────────────────
//   ENUMERATIONS & FLIGHT MODES
// ────────────────────────────────────────────────────────────────
//

enum DroneState {
    STATE_INIT,
    STATE_ARMED,
    STATE_TAKEOFF,
    STATE_IN_FLIGHT,
    STATE_FAILSAFE
};

enum FlightMode {
    MODE_STABLE,
    MODE_HOVER,
    MODE_CRUISE
};

#endif // CONFIG_H
