#include "bmp390.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile bool fifo_data_ready = false;

static const uint8_t IIR_COEFFICIENTS[8] = {0, 1, 3, 7, 15, 31, 63, 127};



int bmp390_set_power_mode(uint8_t mode_bits, bool enable_press, bool enable_temp) {
    uint8_t reg = 0;

    if (enable_press) reg |= 0x01;     // Bit 0: press_en
    if (enable_temp)  reg |= 0x02;     // Bit 1: temp_en

    reg |= (mode_bits << 4);// Bits 5:4 = mode

    return bmp390_write(BMP390_REG_PWR_CTRL, reg);
}

int bmp390_set_sleep_mode(bool enable_press, bool enable_temp) {
    return bmp390_set_power_mode(BMP390_MODE_SLEEP, enable_press, enable_temp);
}

int bmp390_set_normal_mode(bool enable_press, bool enable_temp) {
    return bmp390_set_power_mode(BMP390_MODE_NORMAL, enable_press, enable_temp);
}

int bmp390_perform_single_forced_measurement(float *pressure_pa, float *temperature_c, bool enable_press, bool enable_temp) {
    // === 1. Set Power Mode to FORCED ===
    if (bmp390_set_power_mode(BMP390_MODE_FORCED, enable_press, enable_temp) != 0)
        return -1;

    // === 2. Wait for data ready ===
    uint8_t status = 0;
    int timeout = 50;  // ~500ms max
    while (--timeout > 0) {
        bmp390_read(BMP390_REG_STATUS, &status, 1);

        bool press_ready = enable_press && (status & (1 << 5));  // Bit 5 = drdy_press
        bool temp_ready  = enable_temp  && (status & (1 << 6));  // Bit 6 = drdy_temp

        if ((enable_press && enable_temp && press_ready && temp_ready) ||
            (!enable_press && temp_ready) ||
            (!enable_temp && press_ready)) {
            break;
        }

        delay(10);
    }

    if (timeout == 0) {
        Serial.println("[WARN] Data not ready (timeout)");
        return -1;
    }

    // === 3. Read raw pressure and temperature ===
    uint8_t buf[6];
    if (bmp390_read(BMP390_REG_DATA, buf, 6) != 0)
        return -1;

    uint32_t raw_press_u = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    uint32_t raw_temp_u  = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];

    // === 4. Compensate and update shared calibration state ===
    if (temperature_c) {
        float temp = bmp390_compensate_temperature(raw_temp_u);  // Ensure this updates bmp390_calib.t_lin
        *temperature_c = temp;
    }

    if (pressure_pa) {
        float press = bmp390_compensate_pressure(raw_press_u, bmp390_calib.t_lin);
        *pressure_pa = press;
    }

    return 0;
}

int bmp390_force_measurement_both(float *pressure_pa, float *temperature_c) {
    return bmp390_perform_single_forced_measurement(pressure_pa, temperature_c, true, true);
}



// === User Mode Selection ===
bmp390_mode_t get_user_selected_mode() {
    Serial.println("\n=== Select Flight Mode ===");
    Serial.println("1. STABLE");
    Serial.println("2. HOVER");
    Serial.println("3. CRUISE");
    Serial.print("Enter mode (1-3): ");

    while (Serial.available() == 0) delay(10);
    char input = Serial.read();
    Serial.println(input);

    switch (input) {
        case '1': return BMP390_MODE_STABLE;
        case '2': return BMP390_MODE_HOVER;
        case '3': return BMP390_MODE_CRUISE;
        default:
            Serial.println("[WARN] Invalid input, defaulting to STABLE");
            return BMP390_MODE_STABLE;
    }
}

bool BMP390_validate_profile(const bmp390_profile_t* profile) {
    // Extended ODR period table to support odr_sel up to 0x11
    static const float odr_periods_ms[] = {
        1000.0,   // 0x00: 1 Hz
         500.0,   // 0x01: 2 Hz
         250.0,   // 0x02: 4 Hz
         160.0,   // 0x03: 6.25 Hz
          80.0,   // 0x04: 12.5 Hz
          40.0,   // 0x05: 25 Hz
          20.0,   // 0x06: 50 Hz
          10.0,   // 0x07: 100 Hz
           5.0,   // 0x08: 200 Hz
           2.5,   // 0x09: 400 Hz
           1.25,  // 0x0A: 800 Hz
           0.625, // 0x0B: 1600 Hz
           0.3125,// 0x0C: 3200 Hz
           0.15625,//0x0D: 6400 Hz
           0.078125,//0x0E: 12800 Hz
           0.0390625,//0x0F: 25600 Hz
           0.01953125, // 0x10: 51200 Hz
           0.009765625 // 0x11: 102400 Hz
    };

    if (profile->odr_sel >= sizeof(odr_periods_ms) / sizeof(odr_periods_ms[0])) {
        return false;  // Invalid odr_sel
    }

    // Lookup table for oversampling repetition count (2^osr)
    static const uint8_t osr_lut[] = {1, 2, 4, 8, 16, 32};

    if (profile->osr_p > 5 || profile->osr_t > 5) {
        return false; // Invalid oversampling setting
    }

    uint8_t osr_p_reps = osr_lut[profile->osr_p];
    uint8_t osr_t_reps = osr_lut[profile->osr_t];

    // Tconv = 234µs + press_en * (392 + 2^osr_p * 2020) + temp_en * (163 + 2^osr_t * 2020)
    float t_meas_us = 234.0f
        + (392.0f + osr_p_reps * 2020.0f)  // pressure enabled
        + (163.0f + osr_t_reps * 2020.0f); // temperature enabled

    float t_meas_ms = t_meas_us / 1000.0f;
    float odr_period_ms = odr_periods_ms[profile->odr_sel];

    return t_meas_ms <= odr_period_ms;
}

static bmp390_profile_t bmp390_profiles[] = {
    { // STABLE (low-noise, low-power)
        .osr_p = 0x03,   // Pressure oversampling x8
        .osr_t = 0x01,   // Temperature oversampling x2
        .iir_coeff = 0x07, // IIR coefficient 3
        .odr_sel = 0x04  // 12.5 Hz (80 ms)
    },
    { // HOVER (stable tracking)
        .osr_p = 0x04,   // Pressure oversampling x16
        .osr_t = 0x01,   // Temperature oversampling x2
        .iir_coeff = 0x07, // IIR coefficient 15
        .odr_sel = 0x04  // 12.5 Hz
    },
    { // CRUISE (high-accuracy)
        .osr_p = 0x05,   // Pressure oversampling x32
        .osr_t = 0x01,   // Temperature oversampling x2
        .iir_coeff = 0x07,
        .odr_sel = 0x04
    }
};

void IRAM_ATTR bmp390_isr_handler() {
    fifo_data_ready = true;
}


// ===== FIFO Interrupt Setup =====
int bmp390_fifo_init() {
    pinMode(BMP390_INT_PIN, INPUT); // ✅ Use INPUT (not INPUT_PULLUP) for push-pull output
    attachInterrupt(digitalPinToInterrupt(BMP390_INT_PIN), bmp390_isr_handler, FALLING); // ✅ Active-low INT requires FALLING

    // INT_CTRL = 0b00011100 = 0x1C
    // - Bit 0: int_od = 0 (push-pull)
    // - Bit 1: int_level = 0 (active low)
    // - Bit 2: int_latch = 1 (latched)
    // - Bit 3: fwtm_en = 1 (FIFO watermark interrupt)
    // - Bit 4: ffull_en = 1 (FIFO full interrupt)
    bmp390_write(BMP390_REG_INT_CTRL, 0x1C); // ✅ Enables both FIFO watermark and full interrupts
    return 0;
}


// ===== Mode & Sensor Setup =====
void BMP390_apply_mode_config(bmp390_mode_t mode) {
    bmp390_profile_t profile = bmp390_profiles[mode];

    uint8_t osr = (profile.osr_t << 3) | profile.osr_p;      // OSR: bits [5:3]=osr_t, [2:0]=osr_p
    uint8_t iir = (profile.iir_coeff << 1);                  // CONFIG: bits [3:1]=iir_coeff
    uint8_t odr = profile.odr_sel;                           // ODR: bits [4:0]=odr_sel

    //Serial.println("[DEBUG] Writing OSR...");
    bmp390_write(BMP390_REG_OSR, osr);

    //Serial.println("[DEBUG] Writing CONFIG...");
    bmp390_write(BMP390_REG_CONFIG, iir);

    //Serial.println("[DEBUG] Writing ODR...");
    bmp390_write(BMP390_REG_ODR, odr);

    if (!BMP390_validate_profile(&profile)) {
        Serial.println("[ERROR] Invalid OSR/ODR configuration: Tmeas > ODR period");
    }

    // ⚠️ Power mode and FIFO setup must happen in bmp390_start_fifo_continuous_mode()
}


void BMP390_set_flight_mode(bmp390_mode_t mode) {
    BMP390_apply_mode_config(mode);  // Applies OSR, IIR, ODR settings

    // Recommended: Discard initial samples to let IIR filter stabilize
    BMP390_discard_samples(3, mode);
}



// ===== Start FIFO Mode =====
int bmp390_start_fifo_continuous_mode(bool pressure_enabled, bool temp_enabled) {
    uint8_t status;
    int timeout = 100;

    // Step 1: Flush FIFO before starting
    bmp390_write(BMP390_REG_CMD, BMP390_CMD_FIFO_FLUSH);
    delay(10); // Small delay to ensure flush completes

    // Step 2: Configure FIFO settings
    // FIFO_CONFIG_1: Enable FIFO mode, pressure, temperature, and sensor time
    bmp390_write(BMP390_FIFO_CONFIG1, 0b00011101);  // fifo_mode=1, time_en=1, press_en=1, temp_en=1

    uint8_t check_fifo1;
    bmp390_read(BMP390_FIFO_CONFIG1, &check_fifo1, 1);
    //Serial.printf("[CHECK] FIFO_CONFIG1 readback: 0x%02X\n", check_fifo1);

    // Step 3: Configure FIFO_CONFIG2 for filtered data, no subsampling
    
    bmp390_write(BMP390_FIFO_CONFIG2, 0b01000001); // data_select=01 (filtered), subsample=1

    // Step 4: Set FIFO watermark level to x bytes
    bmp390_write(BMP390_REG_FIFO_WTM_0, 0x90);  // Low byte
    bmp390_write(BMP390_REG_FIFO_WTM_1, 0x00);  // High byte

    uint8_t wtm0, wtm1;
    bmp390_read(BMP390_REG_FIFO_WTM_0, &wtm0, 1);
    bmp390_read(BMP390_REG_FIFO_WTM_1, &wtm1, 1);
    uint16_t watermark = (wtm1 << 8) | wtm0;
    //Serial.printf("[CHECK] FIFO_WTM = 0x%04X (%d bytes)\n", watermark, watermark);

    // Step 5: Wait for command decoder to be ready (check cmd_rdy in STATUS)
    do {
        bmp390_read(BMP390_REG_STATUS, &status, 1);
        delay(2);
    } while (!(status & (1 << 4)) && --timeout > 0); // Bit 4: cmd_rdy

    if (timeout <= 0) {
        Serial.println("[ERROR] Timeout waiting for command ready.");
        return -1;
    }

    // Step 6: Set Normal mode to start measurement
    if (bmp390_set_normal_mode(pressure_enabled, temp_enabled) != 0) {
        Serial.println("[ERROR] Failed to set NORMAL mode");
        return -2;
    }

    uint8_t pwr_ctrl;
    bmp390_read(BMP390_REG_PWR_CTRL, &pwr_ctrl, 1);
    //Serial.printf("[DEBUG] PWR_CTRL after setting NORMAL: 0x%02X\n", pwr_ctrl);

    Serial.println("[INFO] FIFO mode active.");
    return 0;
}


// ===== FIFO Reader =====
int bmp390_check_fifo_overflow() {
    uint8_t status = 0;
    if (bmp390_read(BMP390_REG_INT_STATUS, &status, 1) != 0) return -1;
    return (status & (1 << 1)) ? 1 : 0;  // Bit 1 = FIFO Full Interrupt
}


int bmp390_read_fifo_data(bmp390_fifo_data_t *data_array, uint16_t max_frames, uint16_t *frames_read) {
    //Serial.println("[DEBUG] Entered bmp390_read_fifo_data()");

    *frames_read = 0;
    int valid_count = 0;

    uint8_t len_bytes[2];
    if (bmp390_read(BMP390_REG_FIFO_LENGTH_0, len_bytes, 2) != 0) {
        Serial.println("[ERROR] Failed to read FIFO length");
        return -1;
    }

    uint16_t fifo_len = (len_bytes[1] << 8) | len_bytes[0];
    if (fifo_len == 0 || fifo_len > BMP390_FIFO_MAX_SIZE) {
        Serial.printf("[ERROR] Invalid FIFO length: %d bytes\n", fifo_len);
        return 0;
    }

    Serial.printf("[DEBUG] FIFO reported length: %d bytes\n", fifo_len);

    uint8_t buffer[BMP390_FIFO_MAX_SIZE];
    if (bmp390_read(BMP390_REG_FIFO_DATA, buffer, fifo_len) != 0) {
        Serial.println("[ERROR] Failed to read FIFO data");
        return -1;
    }

    /* Print bytes of the FIFO 
    Serial.println("[DUMP] FIFO Bytes:");
    for (uint16_t i = 0; i < fifo_len; ++i) {
        Serial.printf("0x%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) Serial.println();
    }
    Serial.println();
*/
    uint16_t index = 0;
    uint16_t frame_index = 0;

    while (index < fifo_len && *frames_read < max_frames) {
        uint8_t header = buffer[index++];
        uint8_t fh_mode = header >> 6;

       // Serial.printf("[DEBUG] Frame %d: Header=0x%02X | Mode=0x%02X\n", frame_index, header, fh_mode);

        switch (fh_mode) {
            case 0x02: {  // Sensor Frame
                bmp390_fifo_data_t *frame = &data_array[*frames_read];
                memset(frame, 0, sizeof(bmp390_fifo_data_t));

                bool has_temp = header & BMP390_FRAME_HEADER_TEMP;
                bool has_press = header & BMP390_FRAME_HEADER_PRESS;
                bool has_time = header & BMP390_FRAME_HEADER_SENSORTIME;

                if (has_temp) {
                    if (index + 3 > fifo_len) return -1;
                    int32_t raw_t = (buffer[index + 2] << 16) | (buffer[index + 1] << 8) | buffer[index];
                    index += 3;
                    frame->temperature = bmp390_compensate_temperature(raw_t);
                    frame->temperature_valid = true;
                   // Serial.printf("[DEBUG] Parsed Temp: %ld\n", raw_t);
                }

                if (has_press) {
                    if (index + 3 > fifo_len) return -1;
                    int32_t raw_p = (buffer[index + 2] << 16) | (buffer[index + 1] << 8) | buffer[index];
                    index += 3;
                    frame->pressure = bmp390_compensate_pressure(raw_p, bmp390_calib.t_lin);
                    frame->pressure_valid = true;
                   // Serial.printf("[DEBUG] Parsed Pressure: %ld\n", raw_p);
                }
                delay(1);  // gives time for USB to transmit

                if (has_time && (index + 3 <= fifo_len)) {
                    frame->sensor_time = (buffer[index + 2] << 16) | (buffer[index + 1] << 8) | buffer[index];
                    frame->sensor_time_valid = true;
                    index += 3;
                   // Serial.printf("[DEBUG] Parsed SensorTime: %lu\n", frame->sensor_time);
                }

                if (frame->pressure_valid || frame->temperature_valid) {
                    (*frames_read)++;
                    valid_count++;
                }

                break;
            }

            case 0x01: {  // Control Frame
                if (index < fifo_len) {
                    index++;  // skip opcode
                    Serial.println("[INFO] Control frame skipped.");
                } else {
                    Serial.println("[WARN] Truncated control frame.");
                    return -1;
                }
                break;
            }

            case 0x00: {  // Reserved/Padding/Empty Frame
                if ((header & 0x3F) == 0x00) {  // bits s,t,p = 0
                    if (index < fifo_len) {
                        uint8_t empty = buffer[index++];
                        if (empty != 0x00)
                            Serial.printf("[INFO] Nonzero byte in empty frame: 0x%02X\n", empty);
                    }
                } else {
                    Serial.println("[INFO] Reserved frame skipped.");
                }
                break;
            }

            case 0x03: {
                Serial.printf("[WARN] Unknown frame mode/header: 0x%02X\n", header);
                break;
            }
        }

        frame_index++;
    }

    // Clear INT flags by reading INT_STATUS
    uint8_t dummy;
    bmp390_read(BMP390_REG_INT_STATUS, &dummy, 1);

    //Serial.printf("[SUMMARY] Valid frames: %d / %d | Ready for altitude processing.\n", valid_count, *frames_read);
    return 0;
}


// ===== Utilities =====
 
void BMP390_discard_samples(uint8_t count, bmp390_mode_t mode) {
    Serial.printf("[INFO] Discarding %d sample(s)...\n", count);

    for (uint8_t i = 0; i < count; ++i) {
        float p, t;
        if (bmp390_perform_single_forced_measurement(&p, &t, true, true) == 0) {
            //Serial.printf("[INFO] Discarded Sample %d: P=%.2f Pa, T=%.2f °C\n", i + 1, p, t);
        } else {
            Serial.printf("[WARN] Failed to discard sample %d\n", i + 1);
        }
        delay(20);
    }

    Serial.println("[INFO] Sample discard complete.");
}

/* was used earlier but not any more ,just for referrence 
void BMP390_read_continuous(uint8_t sample_count, bmp390_mode_t mode) {
    for (uint8_t i = 0; i < sample_count; ++i) {
        int32_t raw_p, raw_t;
        if (BMP390_read_raw_temp_and_press_from_config(mode, &raw_p, &raw_t)) {
            float temp = bmp390_compensate_temperature(raw_t);
            float press = bmp390_compensate_pressure(raw_p, temp);
            float alt = bmp390_calculate_altitude(press);

            Serial.printf("Sample %d: P=%.2f Pa, T=%.2f °C, Alt=%.2f m\n",
                          i + 1, press, temp, alt);
        } else {
            Serial.printf("[WARN] Sample %d read failed.\n", i + 1);
        }

        delay(200);  // Delay ~1 ODR cycle for continuous reads
    }
}
*/

void bmp390_print_configuration() {
    uint8_t osr, odr, config, pwr, fifo1, fifo2;
    bmp390_read(BMP390_REG_OSR, &osr, 1);
    bmp390_read(BMP390_REG_ODR, &odr, 1);
    bmp390_read(BMP390_REG_CONFIG, &config, 1);
    bmp390_read(BMP390_REG_PWR_CTRL, &pwr, 1);
    bmp390_read(BMP390_FIFO_CONFIG1, &fifo1, 1);
    bmp390_read(BMP390_FIFO_CONFIG2, &fifo2, 1);

    Serial.println("\n=== BMP390 Configuration ===");
    Serial.printf("OSR   : 0x%02X | P_oversampling = x%d | T_oversampling = x%d\n", osr, 1 << (osr & 0x07), 1 << ((osr >> 3) & 0x07));
    Serial.printf("ODR   : 0x%02X\n", odr);
    Serial.printf("CONFIG: 0x%02X | IIR filter coeff = %d\n", config, ((config >> 1) & 0x07));
    Serial.printf("PWR   : 0x%02X | Mode = %s\n", pwr,
    (pwr & 0x03) == BMP390_MODE_NORMAL ? "NORMAL" :
    (pwr & 0x03) == BMP390_MODE_FORCED ? "FORCED" : "SLEEP");

    Serial.printf("FIFO_CONFIG1: 0x%02X\n", fifo1);
    Serial.printf("FIFO_CONFIG2: 0x%02X\n", fifo2);
    Serial.println("============================\n");
}

void BMP390_print_raw_before_fifo() {
    uint8_t data[6];
    if (bmp390_read(0x04, data, 6) != 0) {
        Serial.println("[ERROR] Failed to read raw temp/press");
        return;
    }

    int32_t raw_press = ((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | data[0];
    int32_t raw_temp  = ((int32_t)data[5] << 16) | ((int32_t)data[4] << 8) | data[3];

    //Serial.printf("[RAW-BEFORE FIFO] Raw Pressure: %ld | Raw Temp: %ld\n", raw_press, raw_temp);
}

bool BMP390_read_raw_temp_and_press_from_config(bmp390_mode_t mode, int32_t* raw_press, int32_t* raw_temp) {
    if (!raw_press || !raw_temp) return false;

    BMP390_apply_mode_config(mode);  // Sets forced/normal/sleep mode

    // Wait for measurement to complete (time depends on ODR/config)
    delay(50);  // adjust as per sensor timing

    uint8_t data[6];
    if (bmp390_read(BMP390_REG_DATA, data, 6) != 0) return false;

    *raw_press = ((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | data[0];
    *raw_temp  = ((int32_t)data[5] << 16) | ((int32_t)data[4] << 8) | data[3];

    return true;
}

float bmp390_altitude_from_ground(float pressure, float ground_pressure) {
    return 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));
}

float bmp390_calculate_altitude(float pressure_pa) {
    float reference_pressure = 101325.0f;
    return 44330.0f * (1.0f - powf(pressure_pa / reference_pressure, 1.0f / 5.255f));
}