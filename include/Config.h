#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*
  Centralized project configuration.
  Keep driver libraries' defines guarded (they use #ifndef checks)
  so including library headers after Config.h won't clash.
*/

// Update these to match your board wiring
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_MASTER_FREQ_HZ    400000

#define SPI_SCK_PIN           12
#define SPI_MISO_PIN          13
#define SPI_MOSI_PIN          11

// Per-device chip selects (no global CS_PIN)
#define BMI323_CS_PIN         8
#define CC2500_CS_PIN         10
#define CC2500_GDO0_PIN      14

// LED pins
#define FRONT_LED_PIN         5
#define REAR_LED_PIN          43

// Motor pins (PWM)
#define MOTOR_FL_PIN          25
#define MOTOR_FR_PIN          26
#define MOTOR_BL_PIN          27
#define MOTOR_BR_PIN          32

// IR sensors (analog receiver + 4 emitters)
#define IR_RECEIVER_PIN       1   // ADC1 channel (adjust to your board)
#define IR_EMITTER_FRONT      40
#define IR_EMITTER_RIGHT      41
#define IR_EMITTER_BACK       38
#define IR_EMITTER_LEFT       39

// Timing (millis)
#define IMU_UPDATE_INTERVAL_MS   10
#define BARO_UPDATE_INTERVAL_MS  50
#define IR_UPDATE_INTERVAL_MS    25
#define LOOP_UPDATE_INTERVAL_MS  10   // general loop cadence

// IR thresholds (tune experimentally)
#define IR_SETTLE_TIME_US         200
#define IR_READ_DELAY_US          300
#define IR_DANGER_THRESHOLD       2000

// CC2500 protocol constants
#define CC2500_START_BYTE         0x88
#define CC2500_PACKET_SIZE        12
#define CC2500_DATA_BYTES         10
#define RSSI_OFFSET_250K         72

// EEPROM & calibration
#ifndef CALIB_EEPROM_ADDR
#define CALIB_EEPROM_ADDR        0
#endif

#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION     86400UL   // 24 hours by default (override if you want longer)
#endif

// BMP390 cache/eeprom size (driver may define own fallback)
#ifndef BMP390_EEPROM_SIZE
#define BMP390_EEPROM_SIZE       64
#endif

// FIFO buffer size (guarded so drivers can pick their own if needed)
#ifndef FIFO_BUFFER_SIZE
#define FIFO_BUFFER_SIZE        512
#endif

// Flight states
enum DroneState {
    STATE_INIT,
    STATE_ARMED,
    STATE_TAKEOFF,
    STATE_IN_FLIGHT,
    STATE_FAILSAFE
};

// Single canonical FlightMode definition guard
#ifndef FLIGHT_MODE_DEFINED
#define FLIGHT_MODE_DEFINED
enum FlightMode {
    MODE_STABLE,
    MODE_HOVER,
    MODE_CRUISE
};
#endif

#endif // CONFIG_H
