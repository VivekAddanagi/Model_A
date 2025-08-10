#ifndef BMI323_H
#define BMI323_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <stdbool.h>
#include "Config.h"  // Central project config (optional)

// ---------------------------------------------------
// Chip Select and SPI pins (can be overridden in Config.h)
// ---------------------------------------------------
#ifndef BMI323_CS_PIN
#define BMI323_CS_PIN 8
#endif

#ifndef SPI_SCK_PIN
#define SPI_SCK_PIN  12
#endif
#ifndef SPI_MISO_PIN
#define SPI_MISO_PIN 13
#endif
#ifndef SPI_MOSI_PIN
#define SPI_MOSI_PIN 11
#endif

// ---------------------------------------------------
// BMI323 Register Addresses
// ---------------------------------------------------
#define CHIP_ID_REG            0x00
#define ERR_REG                0x01
#define STATUS_REG             0x02
#define ACC_X_REG              0x03
#define TEMP_REG               0x09
#define SAT_FLAGS_REG          0x0C
#define EXT_ST_RESULT_REG      0x24
#define CMD_REG                0x7E
#define ACC_CONF_REG           0x20
#define GYR_CONF_REG           0x21
#define FEATURE_CTRL_REG       0x40
#define FEATURE_IO_STATUS_REG  0x14
#define FEATURE_DATA_ADDR      0x41
#define FEATURE_DATA_TX        0x42
#define FEATURE_IO1_REG        0x11

// ---------------------------------------------------
// Constants
// ---------------------------------------------------
#define CHIP_ID_EXPECTED 0x43
#define RESET_CMD        0xDEAF

// FIFO dummy patterns (used for invalid data detection)
#define DUMMY_ACCEL 0x7F01
#define DUMMY_GYRO  0x7F02
#define DUMMY_TEMP  0x8000

// Calibration sample counts
#define GYRO_SAMPLES  200
#define ACCEL_SAMPLES 100

// FIFO settings
#define FIFO_FRAME_SIZE 16
#ifndef FIFO_BUFFER_SIZE
#define FIFO_BUFFER_SIZE 512
#endif

// ---------------------------------------------------
// Data Structures
// ---------------------------------------------------

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} bmi323_data_t;

typedef struct {
    float bias_x;
    float bias_y;
    float bias_z;
} GyroCalibration;

typedef struct {
    float z_offset;
} AccelCalibration;

// Flight mode enumeration and configuration struct
#ifndef FLIGHT_MODE_DEFINED
#define FLIGHT_MODE_DEFINED
typedef enum {
    MODE_STABLE,
    MODE_HOVER,
    MODE_CRUISE
} FlightMode;
#endif

typedef struct {
    bool stabilize_pitch;
    bool stabilize_roll;
    bool stabilize_yaw;
    bool hold_altitude;

    float pitch_gain;
    float roll_gain;
    float yaw_gain;
    float altitude_gain;
} FlightModeConfig;

// Predefined flight mode configs (extern, define in .cpp)
extern const FlightModeConfig stable_config;
extern const FlightModeConfig hover_config;
extern const FlightModeConfig cruise_config;

// ---------------------------------------------------
// Global State Variables (extern, define in .cpp)
// ---------------------------------------------------

extern bmi323_data_t sensor_data;

extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;

extern FlightMode current_mode;
extern const FlightModeConfig* current_config;

extern float estimated_pitch;
extern float estimated_roll;
extern float estimated_yaw;

extern unsigned long last_update_time;

// ---------------------------------------------------
// C-Compatible API Prototypes
// ---------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// Core BMI323 Functions
bool bmi323_init(void);
bool bmi323_read(bmi323_data_t* data);
bool bmi323_run_selftest(void);
bool bmi323_set_axis_remap(uint8_t map_config);

// Register access
uint16_t bmi323_readRegister(uint8_t reg);
void bmi323_writeRegister(uint8_t reg, uint16_t value);
bool bmi323_writeExtendedRegister(uint8_t extReg, uint16_t value);
void bmi323_burstRead(uint8_t reg, uint8_t* buffer, uint16_t length);

// Calibration
bool bmi323_quick_gyro_calibrate(GyroCalibration* cal);
bool bmi323_z_accel_calibrate(AccelCalibration* cal);
void apply_gyro_calibration(const GyroCalibration* cal);
void apply_accel_calibration(const AccelCalibration* cal);
bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal);
void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal);
void clear_calibration_flash(void);
void wait_for_user_confirmation(void);
bool user_requested_recalibration(void);
void perform_calibration_sequence(void);
void print_calibration_info(void);

// FIFO management
void bmi323_setup_fifo(void);
void bmi323_read_fifo(void);

// Orientation updates
void update_orientation(float ax, float ay, float az, float gx, float gy, float gz);

// Flight mode selection
FlightMode select_flight_mode(void);

#ifdef __cplusplus
}
#endif

// ---------------------------------------------------
// Optional: C++ Class API
// ---------------------------------------------------

#ifdef __cplusplus
class BMI323 {
public:
    explicit BMI323(uint8_t csPin = BMI323_CS_PIN);
    bool begin();
    bool readSensorData();
    void calibrate();
    void saveCalibration();
    void loadCalibration();

private:
    uint8_t _csPin;
};
#endif

#endif // BMI323_H
