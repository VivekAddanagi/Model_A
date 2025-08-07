/*
#include "bmi323.h"     // Your existing BMI323 low-level driver
#include <Arduino.h>    // Only if you're using PlatformIO with Arduino

#define FIFO_FRAME_SIZE 16  // 6 (accel) + 6 (gyro) + 2 (temp) + 2 (sensor time)
#define FIFO_BUFFER_SIZE 256
static uint8_t fifo_buffer[FIFO_BUFFER_SIZE];

// Dummy value signatures
#define DUMMY_ACCEL 0x7F01
#define DUMMY_GYRO  0x7F02
#define DUMMY_TEMP  0x8000

// FIFO flush with verification
void bmi323_flush_fifo() {
    bmi323_writeRegister(0x37, 0x0001);  // Trigger FIFO flush
    delay(2);

    // Poll until FIFO is empty
    uint16_t fifo_level;
    do {
        fifo_level = bmi323_readRegister(0x15) & 0x07FF; // bits 0-10
        delay(1);
    } while (fifo_level != 0);

    Serial.println("[FIFO] Flush successful.");
}

void bmi323_setup_fifo() {
    // 1. Set PMU to Normal Mode
    bmi323_writeRegister(0x7D, 0x0000);
    delay(10);

    // 2. Configure FIFO_CONF (0x36) with read-modify-write
    uint16_t fifo_conf = bmi323_readRegister(0x36);
    fifo_conf |= (1 << 8);   // Enable time
    fifo_conf |= (1 << 9);   // Enable accel
    fifo_conf |= (1 << 10);  // Enable gyro
    fifo_conf |= (1 << 11);  // Enable temp
    bmi323_writeRegister(0x36, fifo_conf);

    // 3. Set FIFO watermark to 48 words (~3 frames)
    bmi323_writeRegister(0x35, 48);

    // 4. Map FIFO watermark interrupt to INT1
    uint16_t int_map2 = bmi323_readRegister(0x3B);
    int_map2 &= ~(0b11 << 12);     // Clear fifo_wm_int bits
    int_map2 |=  (0b01 << 12);     // Map to INT1
    bmi323_writeRegister(0x3B, int_map2);

    // 5. Latch interrupt config
    uint16_t int_conf = bmi323_readRegister(0x39);
    int_conf |= (1 << 0);          // INT_LATCH = 1
    bmi323_writeRegister(0x39, int_conf);

    // 6. Flush FIFO before starting
    bmi323_flush_fifo();

    // 7. Debug readback
    Serial.printf("FIFO_CONFIG_0: 0x%04X\n", bmi323_readRegister(0x36));
    Serial.printf("FIFO_CONFIG_1: 0x%04X\n", bmi323_readRegister(0x37));
    Serial.printf("FIFO_CTRL:     0x%04X\n", bmi323_readRegister(0x3A));
    Serial.printf("FIFO_INT_0:    0x%04X\n", bmi323_readRegister(0x3B));
}

void bmi323_read_fifo() {
    uint16_t fifo_fill_words = bmi323_readRegister(0x15) & 0x07FF;
    if (fifo_fill_words == 0) return;

    int bytes_to_read = fifo_fill_words * 2;
    if (bytes_to_read + 1 > FIFO_BUFFER_SIZE) bytes_to_read = FIFO_BUFFER_SIZE - 1;

    // SPI dummy byte handling
    uint8_t raw[FIFO_BUFFER_SIZE + 1] = {0};
    bmi323_burstRead(0x16, raw, bytes_to_read + 1);  // SPI: first byte is dummy
    memcpy(fifo_buffer, raw + 1, bytes_to_read);     // skip dummy byte

    int index = 0;
    while (index + FIFO_FRAME_SIZE <= bytes_to_read) {
        // Read all components
        int16_t ax = (fifo_buffer[index + 1] << 8) | fifo_buffer[index + 0];
        int16_t ay = (fifo_buffer[index + 3] << 8) | fifo_buffer[index + 2];
        int16_t az = (fifo_buffer[index + 5] << 8) | fifo_buffer[index + 4];

        int16_t gx = (fifo_buffer[index + 7] << 8) | fifo_buffer[index + 6];
        int16_t gy = (fifo_buffer[index + 9] << 8) | fifo_buffer[index + 8];
        int16_t gz = (fifo_buffer[index +11] << 8) | fifo_buffer[index +10];

        int16_t temp_raw = (fifo_buffer[index +13] << 8) | fifo_buffer[index +12];

        index += FIFO_FRAME_SIZE;

        // Reject dummy frames
        if (ax == DUMMY_ACCEL || gx == DUMMY_GYRO || temp_raw == DUMMY_TEMP)
            continue;

        // Convert and apply calibration
        float ax_g = ax / 4096.0f;
        float ay_g = ay / 4096.0f;
        float az_g = az / 4096.0f - accel_cal.z_offset;

        float gx_dps = gx / 16.384f - gyro_cal.bias_x;
        float gy_dps = gy / 16.384f - gyro_cal.bias_y;
        float gz_dps = gz / 16.384f - gyro_cal.bias_z;

        // ✅ Orientation update
        update_orientation(ax_g, ay_g, az_g, gx_dps, gy_dps , gz_dps);

        // ✅ Corrections (mode-based)
        if (current_config->stabilize_pitch) {
            float pitch_error = 0.0f - estimated_pitch;
            float pitch_correction = pitch_error * current_config->pitch_gain;
            Serial.printf("Pitch correction: %.2f\n", pitch_correction);
        }

        if (current_config->stabilize_roll) {
            float roll_error = 0.0f - estimated_roll;
            float roll_correction = roll_error * current_config->roll_gain;
            Serial.printf("Roll correction: %.2f\n", roll_correction);
        }

        if (current_config->stabilize_yaw) {
            float yaw_error = 0.0f - estimated_yaw;
            float yaw_correction = yaw_error * current_config->yaw_gain;
            Serial.printf("Yaw correction: %.2f\n", yaw_correction);
        }

        // ✅ FIFO-style debug output
float temp_c = temp_raw / 512.0f + 23.0f;

// Optional: log raw value
Serial.printf("TEMP_RAW: %d | ", temp_raw);

// Clamp to valid range instead of marking as nan
temp_c = constrain(temp_c, -40.0f, 85.0f);

Serial.printf("ACC[g]: X=%.2f Y=%.2f Z=%.2f | ", ax_g, ay_g, az_g);
Serial.printf("GYRO[dps]: X=%.2f Y=%.2f Z=%.2f | ", gx_dps, gy_dps, gz_dps);
Serial.printf("TEMP: %.2f\n", temp_c);

    }
}
*/