#include "bmp390.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// === BMP390 I2C address (selected dynamically) ===
static uint8_t bmp390_address = BMP390_I2C_ADDR_PRIMARY;

// === Global calibration data (used for compensation) ===
BMP390_calib_data_t bmp390_calib;

// =========================== I2C Helpers ===========================

/**
 * Write a single byte to a BMP390 register.
 * 
 * @param reg  Register address
 * @param data Byte to write
 * @return 0 on success, -1 on error
 */
int bmp390_write(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(bmp390_address);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission() == 0 ? 0 : -1;
}

/**
 * Read multiple bytes from BMP390 starting at a register.
 *
 * @param reg  Register to start reading from
 * @param data Pointer to buffer to store read data
 * @param len  Number of bytes to read
 * @return 0 on success, -1 on error
 */
int bmp390_read(uint8_t reg, uint8_t *data, size_t len) {
    if (!data || len == 0) return -1;

    Wire.beginTransmission(bmp390_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;

    Wire.requestFrom((int)bmp390_address, (int)len);
    for (size_t i = 0; i < len && Wire.available(); i++) {
        data[i] = Wire.read();
    }

    return 0;
}

/**
 * Check if a BMP390 sensor is connected on the current address.
 */
static bool bmp390_is_connected() {
    Wire.beginTransmission(bmp390_address);
    return Wire.endTransmission() == 0;
}

// ====================== Calibration Conversion ======================

/**
 * Convert raw register calibration data into floating-point values.
 *
 * @param calib Pointer to calibration structure to convert
 */
static void bmp390_convert_calib_data(BMP390_calib_data_t *calib) {
    calib->t1 = (float)calib->par_t1 / powf(2, -8);
    calib->t2 = (float)calib->par_t2 / powf(2, 30);
    calib->t3 = (float)calib->par_t3 / powf(2, 48);

    calib->p1 = ((float)calib->par_p1 - powf(2, 14)) / powf(2, 20);
    calib->p2 = ((float)calib->par_p2 - powf(2, 14)) / powf(2, 29);
    calib->p3 = (float)calib->par_p3 / powf(2, 32);
    calib->p4 = (float)calib->par_p4 / powf(2, 37);
    calib->p5 = (float)calib->par_p5 / powf(2, -3);
    calib->p6 = (float)calib->par_p6 / powf(2, 6);
    calib->p7 = (float)calib->par_p7 / powf(2, 8);
    calib->p8 = (float)calib->par_p8 / powf(2, 15);
    calib->p9 = (float)calib->par_p9 / powf(2, 48);
    calib->p10 = (float)calib->par_p10 / powf(2, 48);
    calib->p11 = (float)calib->par_p11 / powf(2, 65);
}

// =========================== Initialization ===========================

/**
 * Initialize BMP390 sensor: I2C, chip ID, calibration, sleep mode.
 *
 * @return 0 on success, -1 on failure
 */
int bmp390_init_all() {
    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    // Try primary address
    bmp390_address = BMP390_I2C_ADDR_PRIMARY;
    if (!bmp390_is_connected()) {
        bmp390_address = BMP390_I2C_ADDR_SECONDARY;
        if (!bmp390_is_connected()) {
            Serial.println("[ERROR] BMP390 not found.");
            return -1;
        }
    }

    // Soft reset
    bmp390_write(BMP390_REG_CMD, BMP390_CMD_SOFT_RESET);
    delay(50);

    // Check CHIP ID
    uint8_t chip_id = 0;
    if (bmp390_read(BMP390_REG_CHIP_ID, &chip_id, 1) != 0 || chip_id != BMP390_CHIP_ID) {
        Serial.println("[ERROR] Invalid CHIP ID.");
        return -1;
    }

    // Read and parse calibration registers
    uint8_t calib[BMP390_CALIB_DATA_LEN];
    if (bmp390_read(BMP390_CALIB_DATA_START_ADDR, calib, BMP390_CALIB_DATA_LEN) != 0)
        return -1;

    // Parse raw calibration bytes
    bmp390_calib.par_t1  = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp390_calib.par_t2  = (uint16_t)(calib[3] << 8 | calib[2]);
    bmp390_calib.par_t3  = (int8_t)calib[4];
    bmp390_calib.par_p1  = (int16_t)(calib[6] << 8 | calib[5]);
    bmp390_calib.par_p2  = (int16_t)(calib[8] << 8 | calib[7]);
    bmp390_calib.par_p3  = (int8_t)calib[9];
    bmp390_calib.par_p4  = (int8_t)calib[10];
    bmp390_calib.par_p5  = (uint16_t)(calib[12] << 8 | calib[11]);
    bmp390_calib.par_p6  = (uint16_t)(calib[14] << 8 | calib[13]);
    bmp390_calib.par_p7  = (int8_t)calib[15];
    bmp390_calib.par_p8  = (int8_t)calib[16];
    bmp390_calib.par_p9  = (int16_t)(calib[18] << 8 | calib[17]);
    bmp390_calib.par_p10 = (int8_t)calib[19];
    bmp390_calib.par_p11 = (int8_t)calib[20];

    // Convert to float
    bmp390_convert_calib_data(&bmp390_calib);

    // Start in sleep mode
    return bmp390_write(BMP390_REG_PWR_CTRL, BMP390_MODE_SLEEP);
}

// ========================= Compensation =========================

/**
 * Compensate raw temperature using calibration data.
 *
 * @param uncomp_temp Raw temperature from sensor
 * @return Compensated temperature in Celsius
 */
float bmp390_compensate_temperature(int32_t uncomp_temp) {
    float dt = (float)uncomp_temp - bmp390_calib.t1;
    float t2 = dt * bmp390_calib.t2;
    bmp390_calib.t_lin = t2 + dt * dt * bmp390_calib.t3;
    return bmp390_calib.t_lin;
}

/**
 * Compensate raw pressure using calibration data and linear temperature.
 *
 * @param uncomp_press Raw pressure from sensor
 * @param t_lin Linearized temperature from bmp390_compensate_temperature()
 * @return Compensated pressure in Pascals
 */
float bmp390_compensate_pressure(int32_t uncomp_press, float t_lin) {
    float t = t_lin;
    float t2 = t * t;
    float t3 = t2 * t;
    float up = (float)uncomp_press;
    float up2 = up * up;
    float up3 = up2 * up;

    float out1 = bmp390_calib.p5
               + bmp390_calib.p6 * t
               + bmp390_calib.p7 * t2
               + bmp390_calib.p8 * t3;

    float out2 = up * (bmp390_calib.p1
                     + bmp390_calib.p2 * t
                     + bmp390_calib.p3 * t2
                     + bmp390_calib.p4 * t3);

    float out3 = up2 * (bmp390_calib.p9 + bmp390_calib.p10 * t);
    float out4 = up3 * bmp390_calib.p11;

    return out1 + out2 + out3 + out4;
}
