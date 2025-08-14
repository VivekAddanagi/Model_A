#ifndef BMI323_H
#define BMI323_H

#include <stdint.h>
#include <stdbool.h>

// ----------------------------
// SPI Pin Configuration
// ----------------------------
#define BMI323_CS_PIN   8
#define SPI_SCK_PIN  12
#define SPI_MISO_PIN 13
#define SPI_MOSI_PIN 11

// ----------------------------
// BMI323 Register Addresses
// ----------------------------
#define CHIP_ID_REG        0x00
#define ERR_REG            0x01
#define STATUS_REG         0x02
#define ACC_X_REG          0x03
#define TEMP_REG           0x09
#define SAT_FLAGS_REG      0x0C
#define CMD_REG            0x7E
#define ACC_CONF_REG       0x20
#define GYR_CONF_REG       0x21
#define FEATURE_CTRL_REG       0x40
#define FEATURE_IO_STATUS_REG  0x14
#define FEATURE_DATA_ADDR      0x41
#define FEATURE_DATA_TX        0x42
#define EXT_ST_RESULT_REG      0x24
#define FEATURE_IO1_REG        0x11

// ----------------------------
// Constants
// ----------------------------
#define CHIP_ID_EXPECTED   0x43
#define RESET_CMD          0xDEAF

// ----------------------------
// BMI323 Data Structure
// ----------------------------
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} bmi323_data_t;

// ----------------------------
// Calibration Structures
// ----------------------------
typedef struct {
    float bias_x;
    float bias_y;
    float bias_z;
} GyroCalibration;

typedef struct {
    float bias_x;
    float bias_y;
    float bias_z;
} AccelCalibration;

// Global calibration state (defined in main.cpp)
extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;


// ------------------------
// Flight Mode Definitions
// ------------------------

enum FlightMode {
    MODE_STABLE,
    MODE_HOVER,
    MODE_CRUISE
};

struct FlightModeConfig {
    bool stabilize_pitch;
    bool stabilize_roll;
    bool stabilize_yaw;
    bool hold_altitude;

    float pitch_gain;
    float roll_gain;
    float yaw_gain;
    float altitude_gain;
};

const FlightModeConfig stable_config = {
    .stabilize_pitch = true,
    .stabilize_roll  = true,
    .stabilize_yaw   = true,
    .hold_altitude   = false,
    .pitch_gain = 1.2f,
    .roll_gain  = 1.2f,
    .yaw_gain   = 1.0f,
    .altitude_gain = 0.0f
};

const FlightModeConfig hover_config = {
    .stabilize_pitch = true,
    .stabilize_roll  = true,
    .stabilize_yaw   = true,
    .hold_altitude   = true,
    .pitch_gain = 1.0f,
    .roll_gain  = 1.0f,
    .yaw_gain   = 1.0f,
    .altitude_gain = 1.5f
};

const FlightModeConfig cruise_config = {
    .stabilize_pitch = false,
    .stabilize_roll  = false,
    .stabilize_yaw   = true,
    .hold_altitude   = false,
    .pitch_gain = 0.5f,
    .roll_gain  = 0.5f,
    .yaw_gain   = 0.8f,
    .altitude_gain = 0.0f
};
// Global flight mode state (defined in main.cpp)


// Flight mode & config
extern FlightMode current_mode;
extern const FlightModeConfig* current_config;

// Orientation state
extern float estimated_pitch;
extern float estimated_roll;
extern float estimated_yaw ;  

extern unsigned long last_update_time;

// IMU state
extern bmi323_data_t sensor_data;
extern GyroCalibration gyro_cal;
extern AccelCalibration accel_cal;



// ----------------------------
// C-Compatible API Prototypes
// ----------------------------
#ifdef __cplusplus
extern "C" {
#endif

// Core BMI323 Functions
bool bmi323_init(void);
bool bmi323_read(bmi323_data_t* data);
bool bmi323_run_selftest(void);
bool bmi323_set_axis_remap(uint8_t map_config);
uint16_t bmi323_readRegister(uint8_t reg);
void bmi323_writeRegister(uint8_t reg, uint16_t value);
bool bmi323_writeExtendedRegister(uint8_t extReg, uint16_t value);
void configure_sensor_for_calibration();
void configure_sensor_for_flight();
// Public functions
bool bmi323_quick_gyro_calibrate(GyroCalibration* cal);
bool bmi323_accel_calibrate_all(AccelCalibration* cal) ;
void apply_gyro_calibration(const GyroCalibration* cal);
void apply_accel_calibration(const AccelCalibration* cal);
bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal);
void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal);
void wait_for_user_confirmation();
bool user_requested_recalibration();
void clear_calibration_flash();
void perform_calibration_sequence();
void print_calibration_info();
FlightMode select_flight_mode();
void bmi323_setup_fifo(void);
void bmi323_read_fifo(void);
void update_orientation(float ax, float ay, float az, float gx, float gy ,float gz);
void bmi323_burstRead(uint8_t reg, uint8_t* buffer, uint16_t length);
bool bmi323_read_accel(float* ax, float* ay, float* az);
uint16_t bmi323_readRegister16(uint8_t reg);
#ifdef __cplusplus
}
#endif

#endif  // BMI323_H