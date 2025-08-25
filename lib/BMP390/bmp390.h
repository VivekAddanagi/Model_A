#ifndef BMP390_H
#define BMP390_H

#include <Arduino.h>
#include <Wire.h>

#include <stdint.h>
#include "Config.h" // Central project config

#ifdef __cplusplus
extern "C" {
#endif

// ========================= I2C & Hardware Configuration =========================
#ifndef BMP390_I2C_ADDR_PRIMARY
#define BMP390_I2C_ADDR_PRIMARY       0x77
#endif
#ifndef BMP390_I2C_ADDR_SECONDARY
#define BMP390_I2C_ADDR_SECONDARY     0x76
#endif

#ifndef I2C_MASTER_SDA_IO
#define I2C_MASTER_SDA_IO             17
#endif
#ifndef I2C_MASTER_SCL_IO
#define I2C_MASTER_SCL_IO             18
#endif
#ifndef I2C_MASTER_FREQ_HZ
#define I2C_MASTER_FREQ_HZ            400000
#endif
#ifndef I2C_MASTER_TIMEOUT_MS
#define I2C_MASTER_TIMEOUT_MS         1000
#endif

#ifndef BMP390_INT_PIN
#define BMP390_INT_PIN                21
#endif

// ============================== Register Definitions ==============================
#define BMP390_REG_CHIP_ID            0x00
#define BMP390_REG_STATUS             0x03
#define BMP390_REG_DATA               0x04
#define BMP390_REG_INT_STATUS         0x11
#define BMP390_REG_CONFIG             0x1F
#define BMP390_REG_PWR_CTRL           0x1B
#define BMP390_REG_OSR                0x1C
#define BMP390_REG_ODR                0x1D
#define BMP390_REG_CMD                0x7E

// FIFO Registers
#define BMP390_REG_FIFO_LENGTH_0      0x12
#define BMP390_REG_FIFO_LENGTH_1      0x13
#define BMP390_REG_FIFO_DATA          0x14
#define BMP390_REG_FIFO_WTM_0         0x15
#define BMP390_REG_FIFO_WTM_1         0x16
#define BMP390_FIFO_CONFIG1           0x17
#define BMP390_FIFO_CONFIG2           0x18
#define BMP390_REG_INT_CTRL           0x19

// Commands
#define BMP390_CMD_SOFT_RESET         0xB6
#define BMP390_CMD_FIFO_FLUSH         0xB0

// Identifiers & Modes
#define BMP390_CHIP_ID                0x60
#define BMP390_MODE_SLEEP             0x00
#define BMP390_MODE_FORCED            0x01
#define BMP390_MODE_NORMAL            0x03

// Calibration Memory
#define BMP390_CALIB_DATA_START_ADDR  0x31
#define BMP390_CALIB_DATA_LEN         21


// Calibration data validity duration (seconds)
#ifndef CALIB_VALID_DURATION
#define CALIB_VALID_DURATION 86400UL // 24h default
#endif

// FIFO
#define BMP390_FIFO_MAX_SIZE          512
#ifndef BMP390_FIFO_BUFFER_SIZE
#define BMP390_FIFO_BUFFER_SIZE 512
#endif


// ---------------------------------------------------
// BMP390 FIFO Frame Header Flags (per datasheet)
// ---------------------------------------------------

// Base frame types (fh_mode[1:0] bits)
#define BMP390_FRAME_HEADER_SENSOR       0x80  // 10xxxxxx
#define BMP390_FRAME_HEADER_CONTROL      0x40  // 01xxxxxx

// Specific sensor frame variants (fh_mode = 10)
#define BMP390_FRAME_HEADER_SENSORTIME   0xA0  // 10100000: Sensor + SensorTime
#define BMP390_FRAME_HEADER_TEMP         0x90  // 10010000: Sensor + Temp only
#define BMP390_FRAME_HEADER_PRESS        0x84  // 10000100: Sensor + Pressure only

// Specific control frame variants
#define BMP390_FRAME_HEADER_CONFIG_ERROR 0x44  // 01000100: Control + Config Error


// =============================== Data Structures ===============================

// Calibration Parameters
typedef struct {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t   par_t3;
    int16_t  par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int8_t   par_p10;
    int8_t   par_p11;

    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;

    float t_lin;
} BMP390_calib_data_t;

typedef struct {
    float pressure;
    float temperature;
} BMP390_calib_values_t;

typedef struct {
    float pressure;
    float temperature;
    uint32_t sensor_time;
    bool pressure_valid;
    bool temperature_valid;
    bool sensor_time_valid;
} bmp390_fifo_data_t;

typedef struct {
    bool pressure_enabled;
    bool temperature_enabled;
    bool sensortime_enabled;
    bool filter_data;
    uint8_t subsampling;
    bool stop_on_full;
    uint16_t watermark_level;
} bmp390_fifo_config_t;

typedef struct {
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t iir_coeff;
} FilterConfig;

typedef enum {
    BMP390_MODE_STABLE = 0,
    BMP390_MODE_HOVER  = 1,
    BMP390_MODE_CRUISE = 2
} bmp390_mode_t;

typedef struct {
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t iir_coeff;
    uint8_t odr_sel;
    uint8_t power_mode;
} bmp390_profile_t;

typedef struct {
    float pressure_offset;
    float calibration_temperature;
    uint32_t timestamp;
} BMP390_CalibrationData;

// ========================== Global Variables ==========================
extern BMP390_calib_data_t bmp390_calib;
extern volatile bool fifo_data_ready;


bool bmp390_begin();
void bmp390_update();
float bmp390_get_latest_altitude();

// ========================== C API Function Prototypes ==========================

// Init & Power Modes
bool bmp390_init_all();
int bmp390_set_power_mode(uint8_t mode_bits, bool enable_press, bool enable_temp);
int bmp390_set_sleep_mode(bool enable_press, bool enable_temp);
int bmp390_set_normal_mode(bool enable_press, bool enable_temp);

// Data Reading & Compensation
int bmp390_read(uint8_t reg, uint8_t *data, size_t len);
int bmp390_write(uint8_t reg, uint8_t data);
bool BMP390_read_raw_temp_and_press_from_config(bmp390_mode_t mode, int32_t* raw_press, int32_t* raw_temp);
float bmp390_compensate_temperature(int32_t uncomp_temp);
float bmp390_compensate_pressure(int32_t uncomp_press, float t_lin);
float bmp390_calculate_altitude(float pressure_pa);
float bmp390_altitude_from_ground(float pressure, float ground_pressure);
float bmp390_get_ground_pressure(void);

// Forced Measurement
int bmp390_perform_single_forced_measurement(float *pressure_pa, float *temperature_c, bool enable_press, bool enable_temp);
int bmp390_force_measurement_both(float *pressure_pa, float *temperature_c);

// FIFO Operations
int bmp390_fifo_init(void);
int bmp390_read_fifo_data(bmp390_fifo_data_t *data_array, uint16_t max_frames, uint16_t *frames_read);
int bmp390_check_fifo_overflow(void);
int bmp390_start_fifo_continuous_mode(bool pressure_enabled, bool temp_enabled);
void IRAM_ATTR bmp390_fifo_isr();
void BMP390_print_raw_before_fifo(void);

// Mode & Filtering Control
void BMP390_apply_mode_config(bmp390_mode_t mode);
void BMP390_init_filtering(bmp390_mode_t mode);
void BMP390_set_flight_mode(bmp390_mode_t mode);
void BMP390_discard_samples(uint8_t count, bmp390_mode_t mode);
bmp390_mode_t get_user_selected_mode();
void bmp390_print_configuration();

// Calibration & EEPROM
int bmp390_calibrate_offset(float sea_level_pressure);

int bmp390_apply_calibration(void);

float bmp390_get_relative_altitude(float pressure_now);
float bmp390_get_absolute_altitude(float pressure_now);

void updateAltitude(float acc_x, float acc_y, float acc_z,
                    float roll, float pitch,
                    float alt_baro, float dt);







#ifdef __cplusplus
} // extern "C"
#endif

// ========================== Arduino-Friendly C++ Class ==========================
#ifdef __cplusplus
class BMP390 {
public:
    BMP390();
    bool begin();
    float readPressure();
    float readAltitude(float seaLevelhPa = 1013.25);
    void calibrate();
    void saveCalibration();
    void loadCalibration();

private:
    // Calibration storage and internals
};
#endif

#endif // BMP390_H
