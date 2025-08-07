#ifndef BMP390_H
#define BMP390_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <driver/gpio.h>

//
// ========================= I2C & Hardware Configuration =========================
//
#define BMP390_I2C_ADDR_PRIMARY       0x77
#define BMP390_I2C_ADDR_SECONDARY     0x76

#define I2C_MASTER_SDA_IO             17
#define I2C_MASTER_SCL_IO             18
#define I2C_MASTER_FREQ_HZ            400000
#define I2C_MASTER_TIMEOUT_MS         1000

#define BMP390_INT_PIN                21

//
// ============================== Register Definitions ==============================
//

// --- General Registers ---
#define BMP390_REG_CHIP_ID            0x00
#define BMP390_REG_STATUS             0x03
#define BMP390_REG_DATA               0x04
#define BMP390_REG_INT_STATUS         0x11
#define BMP390_REG_CONFIG             0x1F
#define BMP390_REG_PWR_CTRL           0x1B
#define BMP390_REG_OSR                0x1C
#define BMP390_REG_ODR                0x1D
#define BMP390_REG_CMD                0x7E

// --- FIFO Registers ---
#define BMP390_REG_FIFO_LENGTH_0      0x12
#define BMP390_REG_FIFO_LENGTH_1      0x13
#define BMP390_REG_FIFO_DATA          0x14
#define BMP390_REG_FIFO_WTM_0         0x15
#define BMP390_REG_FIFO_WTM_1         0x16
#define BMP390_FIFO_CONFIG1           0x17
#define BMP390_FIFO_CONFIG2           0x18
#define BMP390_REG_INT_CTRL           0x19

// --- Commands ---
#define BMP390_CMD_SOFT_RESET         0xB6
#define BMP390_CMD_FIFO_FLUSH         0xB0

// --- Identifiers & Modes ---
#define BMP390_CHIP_ID                0x60
#define BMP390_MODE_SLEEP             0x00  // mode[5:4] = 00
#define BMP390_MODE_FORCED            0x01  // mode[5:4] = 01
#define BMP390_MODE_NORMAL            0x03  // mode[5:4] = 11

//
// =========================== Calibration Memory Configuration ===========================
//
#define BMP390_CALIB_DATA_START_ADDR  0x31
#define BMP390_CALIB_DATA_LEN         21
#define BMP390_EEPROM_SIZE            512
#define CALIB_EEPROM_ADDR             0
#define CALIB_VALID_DURATION          2592000UL  // 30 days in seconds

//
// ============================= FIFO Frame Header Flags =============================
//
#define BMP390_FRAME_HEADER_SENSOR         0x80
#define BMP390_FRAME_HEADER_CONTROL        0x40
#define BMP390_FRAME_HEADER_EMPTY          0x80
#define BMP390_FRAME_HEADER_SENSORTIME     0x20
#define BMP390_FRAME_HEADER_TEMP           0x10
#define BMP390_FRAME_HEADER_PRESS          0x04
#define BMP390_FRAME_HEADER_CONFIG_ERROR   0x44

//
// ============================== FIFO Configuration ==============================
//
#define BMP390_FIFO_MAX_SIZE          512
#define FIFO_BUFFER_SIZE              512

//
// =============================== Data Structures ===============================
//

// --- Calibration Parameters & Coefficients ---
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

// --- Processed Sensor Values ---
typedef struct {
    float pressure;
    float temperature;
} BMP390_calib_values_t;

// --- FIFO Frame Data ---
typedef struct {
    float pressure;
    float temperature;
    uint32_t sensor_time;
    bool pressure_valid;
    bool temperature_valid;
    bool sensor_time_valid;
} bmp390_fifo_data_t;

// --- FIFO Configuration ---
typedef struct {
    bool pressure_enabled;
    bool temperature_enabled;
    bool sensortime_enabled;
    bool filter_data;
    uint8_t subsampling;
    bool stop_on_full;
    uint16_t watermark_level;
} bmp390_fifo_config_t;

// --- Filtering Parameters ---
typedef struct {
    uint8_t osr_p;     // Pressure oversampling (0–7)
    uint8_t osr_t;     // Temperature oversampling (0–7)
    uint8_t iir_coeff; // IIR filter coefficient index (0–7)
} FilterConfig;

// --- Flight Modes ---
typedef enum {
    BMP390_MODE_STABLE = 0,
    BMP390_MODE_HOVER  = 1,
    BMP390_MODE_CRUISE = 2
} bmp390_mode_t;

// ===== Profile Configuration =====
typedef struct {
    uint8_t osr_p;
    uint8_t osr_t;
    uint8_t iir_coeff;
    uint8_t odr_sel;
    uint8_t power_mode;
} bmp390_profile_t;


// --- Stored Calibration Snapshot ---
typedef struct {
    float pressure_offset;
    float calibration_temperature;
    uint32_t timestamp;
} BMP390_CalibrationData;

//
// ========================== Global Variables ==========================
extern BMP390_calib_data_t bmp390_calib;
extern volatile bool fifo_data_ready;



//
// ========================== Function Prototypes ==========================
//

// --- Initialization & Power Modes ---
int bmp390_init_all(void);
int bmp390_set_power_mode(uint8_t mode_bits, bool enable_press, bool enable_temp);
int bmp390_set_sleep_mode(bool enable_press, bool enable_temp);
int bmp390_set_normal_mode(bool enable_press, bool enable_temp);

// --- Data Reading & Compensation ---
int bmp390_read(uint8_t reg, uint8_t *data, size_t len);
int bmp390_write(uint8_t reg, uint8_t data);
bool BMP390_read_raw_temp_and_press_from_config(bmp390_mode_t mode, int32_t* raw_press, int32_t* raw_temp);
float bmp390_compensate_temperature(int32_t uncomp_temp);
float bmp390_compensate_pressure(int32_t uncomp_press, float t_lin);
float bmp390_calculate_altitude(float pressure_pa);
float bmp390_altitude_from_ground(float pressure, float ground_pressure);
float bmp390_get_ground_pressure(void);

// --- Forced Measurement ---
int bmp390_perform_single_forced_measurement(float *pressure_pa, float *temperature_c, bool enable_press, bool enable_temp);
int bmp390_force_measurement_both(float *pressure_pa, float *temperature_c);

// --- FIFO Operations ---
int bmp390_fifo_init(void);
int bmp390_read_fifo_data(bmp390_fifo_data_t *data_array, uint16_t max_frames, uint16_t *frames_read);
int bmp390_check_fifo_overflow(void);
int bmp390_start_fifo_continuous_mode(bool pressure_enabled, bool temp_enabled);
void IRAM_ATTR bmp390_isr_handler(void);
void BMP390_print_raw_before_fifo(void);

// --- Mode & Filtering Control ---
void BMP390_apply_mode_config(bmp390_mode_t mode);
void BMP390_init_filtering(bmp390_mode_t mode);
void BMP390_set_flight_mode(bmp390_mode_t mode);
void BMP390_discard_samples(uint8_t count, bmp390_mode_t mode);
bmp390_mode_t get_user_selected_mode();
void bmp390_print_configuration();

// --- Calibration & EEPROM ---
int bmp390_calibrate_offset(void);         // Run calibration and save to EEPROM
int bmp390_apply_calibration(void);        // Load calibration from EEPROM

#ifdef __cplusplus
}
#endif

#endif // BMP390_H
