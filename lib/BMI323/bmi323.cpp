#include <Arduino.h>
#include <SPI.h>
#include "bmi323.h"
#include <math.h>
#include <Preferences.h>

// === Define global variables here below===
FlightMode current_mode;
const FlightModeConfig* current_config = nullptr;

float estimated_pitch = 0.0f;
float estimated_roll = 0.0f;
float estimated_yaw = 0.0f;  

unsigned long last_update_time = 0;

bmi323_data_t sensor_data;
GyroCalibration gyro_cal;
AccelCalibration accel_cal;

// --- Internal SPI Configuration ---
static SPISettings bmi323_spi_settings(6500000, MSBFIRST, SPI_MODE0);

// --- Internal Helper Functions ---
 void bmi323_writeRegister(uint8_t reg, uint16_t value) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);              // Write
    SPI.transfer(value & 0xFF);            // LSB
    SPI.transfer((value >> 8) & 0xFF);     // MSB
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction();
}

 uint16_t bmi323_readRegister(uint8_t reg) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);              // Read
    SPI.transfer(0x00);                    // Dummy
    uint8_t lsb = SPI.transfer(0x00);
    uint8_t msb = SPI.transfer(0x00);
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction();
    return (msb << 8) | lsb;
}

// --- Extended Register Helper Functions ---
static bool waitForFeatureDataReady(uint8_t max_retries = 50) {
    for (uint8_t i = 0; i < max_retries; ++i) {
        uint16_t status = bmi323_readRegister(0x43); // FEATURE_DATA_STATUS
        if ((status >> 1) & 0x01) return true;  // Bit 1 = data_tx_ready
        delay(1);
    }
    return false;
}

 bool bmi323_writeExtendedRegister(uint8_t extReg, uint16_t value) {
    bmi323_writeRegister(0x41, extReg); // FEATURE_DATA_ADDR

    if (!waitForFeatureDataReady()) {
        Serial.println("[ERROR] Timeout waiting for FEATURE_DATA_STATUS before write.");
        return false;
    }

    bmi323_writeRegister(0x42, value); // FEATURE_DATA_TX
    return true;
}

static bool bmi323_readExtendedRegister(uint8_t extReg, uint16_t* out_value) {
    bmi323_writeRegister(0x41, extReg); // FEATURE_DATA_ADDR

    if (!waitForFeatureDataReady()) {
        Serial.println("[ERROR] Timeout waiting for FEATURE_DATA_STATUS before read.");
        return false;
    }

    *out_value = bmi323_readRegister(0x42); // FEATURE_DATA_TX
    return true;
}

// --- BMI323 Core Functions ---
bool bmi323_init(void) {
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, BMI323_CS_PIN);
    pinMode(BMI323_CS_PIN, OUTPUT);
    digitalWrite(BMI323_CS_PIN, HIGH);
    delay(10);

    bmi323_readRegister(CHIP_ID_REG);  // Dummy read to activate SPI
    delay(5);

    uint16_t chip_id = bmi323_readRegister(CHIP_ID_REG);
    if ((chip_id & 0xFF) != CHIP_ID_EXPECTED) {
        Serial.printf("[BMI323] Unexpected CHIP ID: 0x%02X\n", chip_id & 0xFF);
        return false;
    }
    Serial.println("[BMI323] CHIP ID OK");

    // --- Reset Sensor ---
    bmi323_writeRegister(CMD_REG, RESET_CMD);
    delay(200);
    bmi323_set_axis_remap(0x00);
    delay(200);

    // --- Feature Engine Initialization (Required for Self-Test) ---
    bmi323_writeRegister(0x12, 0x012C);             // FEATURE_IO2: startup_config_0
    bmi323_writeRegister(0x14, 0x0001);             // FEATURE_IO_STATUS: confirm config
    delay(10);
    bmi323_writeRegister(FEATURE_CTRL_REG, 0x0001); // Enable feature engine
    delay(5);

    bool feature_ready = false;
    for (int i = 0; i < 50; ++i) {
        uint16_t err_status = bmi323_readRegister(0x11); // FEATURE_IO1
        if ((err_status & 0x0F) == 0x01) {
            Serial.println("[BMI323] Feature engine initialized.");
            feature_ready = true;
            break;
        }
        delay(10);
    }
    if (!feature_ready) {
        Serial.println("[ERROR] Feature engine did not initialize.");
        return false;
    }

    
    uint16_t err = bmi323_readRegister(ERR_REG);
    if (err & 0x01) {
        Serial.println("[BMI323] Fatal error in ERR_REG.");
        return false;
    }

    for (int i = 0; i < 20; ++i) {
        if (bmi323_readRegister(STATUS_REG) & (1 << 7)) {
            
            break;
        }
        delay(10);
    }

      bmi323_writeRegister(ACC_CONF_REG, 0x70A9);  // ±8g, 200Hz, high perf
      bmi323_writeRegister(GYR_CONF_REG, 0x70C9);  // ±2000 dps, 200Hz, high perf
      Serial.println("[BMI323] Sensor configured successfully.");

        return true;
}

bool bmi323_read(bmi323_data_t* data) {
    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        if (bmi323_readRegister(STATUS_REG) & (1 << 7)) break;
        delay(10);
    }

    uint8_t buffer[13];
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(ACC_X_REG | 0x80);
    buffer[0] = SPI.transfer(0x00); // Dummy
    for (int i = 1; i < 13; ++i) buffer[i] = SPI.transfer(0x00);
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction();

    data->ax = (buffer[2] << 8) | buffer[1];
    data->ay = (buffer[4] << 8) | buffer[3];
    data->az = (buffer[6] << 8) | buffer[5];
    data->gx = (buffer[8] << 8) | buffer[7];
    data->gy = (buffer[10] << 8) | buffer[9];
    data->gz = (buffer[12] << 8) | buffer[11];
    data->temp = (int16_t)bmi323_readRegister(TEMP_REG);

    return true;
}


bool bmi323_run_selftest(void) {
    Serial.println("[BMI323] Preparing for self-test...");

    // Step 1: Disable alternate sensor configurations
    bmi323_writeRegister(0x28, 0x0000); // ALT_ACC_CONF
    bmi323_writeRegister(0x29, 0x0000); // ALT_GYR_CONF
    delay(100);

    // Step 2: Wait until system is in feature mode (FEATURE_IO1.state == 0b00)
    bool system_ready = false;
    uint16_t feature_io1 = 0;
    for (int i = 0; i < 20; ++i) {
        feature_io1 = bmi323_readRegister(0x11); // FEATURE_IO1
        uint8_t state = (feature_io1 >> 11) & 0x03;
        if (state == 0x00) {
            system_ready = true;
            break;
        }
        delay(10);
    }
    if (!system_ready) {
        Serial.println("[ERROR] BMI323 not in feature mode for self-test.");
        return false;
    }

    uint16_t io0 = bmi323_readRegister(0x10);
    if ((io0 & 0x01) != 0) {
        bmi323_writeRegister(0x10, io0 & ~0x01); // Disable i3c_sync_en
    }

    // Step 3: Start self-test
     bmi323_writeRegister(CMD_REG, 0x0100);
    delay(350); // Full duration for both acc + gyro test

    // Step 4: Check if self-test completed
    feature_io1 = bmi323_readRegister(0x11);
    bool completed = (feature_io1 >> 4) & 0x01;
    if (!completed) {
        Serial.println("[ERROR] Self-test did not complete within timeout.");
        return false;
    }
    Serial.println("[BMI323] Self-test completed.");

    // Step 5: Check overall result
    bool result = (feature_io1 >> 6) & 0x01;
    Serial.printf("[BMI323] Overall self-test result: 0x%01X (1=OK, 0=Not OK)\n", result);

    // Step 6: Final result check
    uint16_t feature_io1_reg_value = bmi323_readRegister(0x11);
    uint8_t overall_st_result = (feature_io1_reg_value >> 6) & 0x01;
    uint8_t error_status_final = (feature_io1_reg_value >> 0) & 0x0F;

    if (overall_st_result == 1 && error_status_final == 0x05) {
        Serial.println("✅ Self-test PASSED.");
        return true;
    }

    Serial.printf("[WARN] Self-test failed. error_status: 0x%02X\n", error_status_final);
    return false;
}

bool bmi323_set_axis_remap(uint8_t config) {
    bmi323_writeRegister(0x03, config);         // EXT.AXIS_MAP_1
    bmi323_writeRegister(CMD_REG, 0x0300);      // Trigger remap
    delay(10);
    return true;
}

void bmi323_burstRead(uint8_t reg, uint8_t* buffer, uint16_t length) {
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg | 0x80); // Set read bit
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(BMI323_CS_PIN, HIGH);
}

bool bmi323_read_accel(float* ax, float* ay, float* az) {
    bmi323_data_t data;
    if (!bmi323_read(&data)) {
        return false;
    }
    // Convert raw values to g
    *ax = data.ax / 4096.0f; // adjust divisor per BMI323 config
    *ay = data.ay / 4096.0f;
    *az = data.az / 4096.0f;
    return true;
}

// Global calibration biases (from your calibration step)

// corrected update_orientation_dt - integrate gyro axes to matching Euler angles
void update_orientation_dt(float ax, float ay, float az,
                           float gx, float gy, float gz, float dt,
                           bool gyro_in_rad_per_s = false) {

    const float alpha = 0.98f;
    if (!(dt > 0.0f && dt <= 0.1f)) return;

    // Optionally convert rad/s -> deg/s if needed
    if (gyro_in_rad_per_s) {
        const float RAD2DEG = 180.0f / M_PI;
        gx *= RAD2DEG;
        gy *= RAD2DEG;
        gz *= RAD2DEG;
    }

    // Accelerometer-based angles (deg)
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float acc_roll  = atan2f( ay, az ) * 180.0f / M_PI;

    float use_alpha = alpha;
    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (fabsf(acc_mag - 1.0f) > 0.4f) use_alpha = 1.0f; // ignore accel when magnitude off

    // NOTE THE FIX: gx -> roll, gy -> pitch
    estimated_roll  = use_alpha * (estimated_roll  + gx * dt) + (1.0f - use_alpha) * acc_roll;
    estimated_pitch = use_alpha * (estimated_pitch + gy * dt) + (1.0f - use_alpha) * acc_pitch;
    estimated_yaw  += gz * dt; // yaw remains integrated

    // normalize yaw
    if (estimated_yaw > 180.0f)  estimated_yaw -= 360.0f;
    if (estimated_yaw < -180.0f) estimated_yaw += 360.0f;

    // Throttle debug prints to avoid disturbing timing
    static unsigned long last_dbg = 0;
    if (millis() - last_dbg > 200) {
        Serial.printf("ROLL: %.2f | PITCH: %.2f | YAW: %.2f\n",
                      estimated_roll, estimated_pitch, estimated_yaw);
        last_dbg = millis();
    }
}


#define FIFO_FRAME_SIZE 16  // 6 (accel) + 6 (gyro) + 2 (temp) + 2 (sensor time)
#define FIFO_BUFFER_SIZE 512

static uint8_t raw_buffer[FIFO_BUFFER_SIZE + 1];
static uint8_t fifo_buffer[FIFO_BUFFER_SIZE];


// Dummy value signatures
#define DUMMY_ACCEL 0x7F01
#define DUMMY_GYRO  0x7F02
#define DUMMY_TEMP  0x8000

// ----------------- Public: flush FIFO -----------------
void bmi323_flush_fifo() {
    bmi323_writeRegister(REG_FIFO_CTRL, 0x0001);  // flush bit
    delay(2);
    // wait until empty
    while ((bmi323_readRegister(REG_FIFO_LENGTH) & 0x07FF) != 0) {
        delay(1);
    }
    Serial.println("[BMI323 FIFO] Flush successful.");
}

volatile bool bmi323_fifo_ready = false;

// ----------------- ISR -----------------
void IRAM_ATTR bmi323_isr_handler() {
    // Keep it short; do not do SPI here
#if USE_INT_LATCH
    // In latched mode, we’ll clear the latch in service function after draining FIFO
#endif
    bmi323_fifo_ready = true;
}

void bmi323_init_isr() {
    pinMode(BMI323_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BMI323_INT_PIN), bmi323_isr_handler, RISING);
}

// ----------------- Public: configure ODR/mode, FIFO, INT -----------------
void bmi323_setup_fifo() {
    // 1) Ensure sensors are ACTIVE (High Performance)
    // Example: your 200 Hz configs (keep if you like)
    // ACC: 0x70A9 => HP mode, ODR=200 Hz, ±8g, BW=ODR/4
    // GYR: 0x70C9 => HP mode, ODR=200 Hz, ±2000 dps, BW=ODR/4
    //bmi323_writeRegister(REG_ACC_CONF, 0x70A9);
    //bmi323_writeRegister(REG_GYR_CONF, 0x70C9);
    //delay(2);

    // 2) Enable FIFO content (time, acc, gyr, temp)
    uint16_t fifo_conf = bmi323_readRegister(REG_FIFO_CONF);
    fifo_conf |= (1 << 8);   // fifo_time_en
    fifo_conf |= (1 << 9);   // fifo_acc_en
    fifo_conf |= (1 << 10);  // fifo_gyr_en
    fifo_conf |= (1 << 11);  // fifo_temp_en
    bmi323_writeRegister(REG_FIFO_CONF, fifo_conf);

    // 3) Watermark (in WORDS). If your frame is 16B (8 words), WTM = frames * 8
    uint16_t wtm_words = (uint16_t)(WATERMARK_FRAMES * WORDS_PER_FRAME); // eg 4*8=32
    bmi323_writeRegister(REG_FIFO_WTM, wtm_words);

    // 4) Map FIFO watermark → INT1
    uint16_t int_map2 = bmi323_readRegister(REG_INT_MAP2);
    int_map2 &= ~(0b11 << 12); // clear fifo_wm map
    int_map2 |=  (0b01 << 12); // map fifo_wm to INT1
    bmi323_writeRegister(REG_INT_MAP2, int_map2);

    // 5) Configure INT1 output: active-high, push-pull, enable
    uint16_t io = bmi323_readRegister(REG_IO_INT_CTRL);
    io |= (1 << 0);  // int1_lvl = 1 (active high)
    io &= ~(1 << 1); // int1_od  = 0 (push-pull)
    io |= (1 << 2);  // int1_output_en = 1
    bmi323_writeRegister(REG_IO_INT_CTRL, io);

    // 6) Latch mode (recommend pulse = 0 for ESP32 attachInterrupt on RISING)
    uint16_t icf = bmi323_readRegister(REG_INT_CONF);
#if USE_INT_LATCH
    icf |=  (1 << 0); // latched
#else
    icf &= ~(1 << 0); // pulse
#endif
    bmi323_writeRegister(REG_INT_CONF, icf);

    // 7) Flush FIFO and attach ISR
    bmi323_flush_fifo();
    bmi323_init_isr();

    // 8) Debug/health checks
    uint16_t err = bmi323_readRegister(REG_ERR_REG);
    if (err) Serial.printf("[BMI323 ERROR] ERR_REG=0x%04X\n", err);
    bmi323_debug_readback();
    imu_filters_init(200.0f);

}

// Allocate memory here (only once in project)
float latest_ax = 0.0f;
float latest_ay = 0.0f;
float latest_az = 0.0f;
float latest_gx = 0.0f;
float latest_gy = 0.0f;
float latest_gz = 0.0f;

// Robust FIFO read

// helper: compute current FIFO frame size (bytes) based on FIFO_CONF
static uint16_t bmi323_frame_size_bytes_from_conf() {
    uint16_t fifo_conf = bmi323_readRegister(REG_FIFO_CONF); // same reg you use elsewhere
    uint16_t words = 0;
    // bit layout used in your code: bit8 = time, bit9 = acc, bit10 = gyr, bit11 = temp
    if (fifo_conf & (1 << 9))  words += 3; // accel (3 words)
    if (fifo_conf & (1 << 10)) words += 3; // gyro  (3 words)
    if (fifo_conf & (1 << 11)) words += 1; // temp  (1 word)
    if (fifo_conf & (1 << 8))  words += 1; // sensor time (1 word)
    return words * 2; // convert words -> bytes
}

// low-pass filter for gyro/accel data
struct Lpf1 {
    float y = 0, a = 0;
    void init(float cutoff_hz, float sample_hz) {
        float rc = 1.0f / (2.0f * PI * cutoff_hz);
        a = 1.0f / (1.0f + rc * sample_hz);
    }
    float step(float x) {
        y += a * (x - y);
        return y;
    }
};

// Instances for IMU axes
Lpf1 lpf_gx, lpf_gy, lpf_gz;
Lpf1 lpf_ax, lpf_ay, lpf_az;

// Call once after IMU ODR is known
void imu_filters_init(float imu_hz) {
    lpf_gx.init(60.0f, imu_hz);
    lpf_gy.init(60.0f, imu_hz);
    lpf_gz.init(60.0f, imu_hz);

    lpf_ax.init(30.0f, imu_hz);
    lpf_ay.init(30.0f, imu_hz);
    lpf_az.init(30.0f, imu_hz);
}

    // Globals
static bool have_bmi_time = false;
static uint16_t last_bmi_st = 0;
static float    bmi_time_lsb_sec = 0.0000390625f; // set from datasheet/config

void bmi323_read_fifo() {
  
    if (!bmi323_fifo_ready) return;
    bmi323_fifo_ready = false;

#if USE_INT_LATCH
    // Clear latched INT1 if using latched mode
    (void)bmi323_readRegister(REG_INT_STATUS_INT1);
#endif

    // get current fifo config & derive whether time/temp are present
    uint16_t fifo_conf = bmi323_readRegister(REG_FIFO_CONF);
    const bool fifo_time_en = (fifo_conf & (1 << 8)) != 0;
    const bool fifo_acc_en  = (fifo_conf & (1 << 9)) != 0;
    const bool fifo_gyr_en  = (fifo_conf & (1 << 10)) != 0;
    const bool fifo_temp_en = (fifo_conf & (1 << 11)) != 0;

    // compute expected frame size from FIFO_CONF (safe fallback to your FRAME_BYTES)
    uint16_t frame_bytes = bmi323_frame_size_bytes_from_conf();
    if (frame_bytes == 0) frame_bytes = FRAME_BYTES; // fallback (likely 14)

    // Read FIFO length (words)
    uint16_t fifo_fill_words = bmi323_readRegister(REG_FIFO_LENGTH) & 0x07FF;
    if (fifo_fill_words == 0) return;

    int bytes_to_read = fifo_fill_words * 2;
    if (bytes_to_read + 1 > FIFO_BUFFER_SIZE) bytes_to_read = FIFO_BUFFER_SIZE - 1;

    // SPI burst read: many drivers expect a dummy byte first
    uint8_t raw[FIFO_BUFFER_SIZE + 1] = {0};
    bmi323_burstRead(REG_FIFO_DATA, raw, bytes_to_read + 1);
    memcpy(fifo_buffer, raw + 1, bytes_to_read);  // skip dummy

    int index = 0;
    int parsed = 0, skipped = 0;

    while (index + (int)frame_bytes <= bytes_to_read) {
        int cur = index;
        int16_t ax_raw=0, ay_raw=0, az_raw=0;
        int16_t gx_raw=0, gy_raw=0, gz_raw=0;
        int16_t temp_raw = (int16_t)DUMMY_TEMP;
        uint16_t sensor_time_raw = 0;

        if (fifo_acc_en) {
            ax_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
            ay_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
            az_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
        }
        if (fifo_gyr_en) {
            gx_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
            gy_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
            gz_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
        }
        if (fifo_temp_en) {
            temp_raw = (int16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
        }
        if (fifo_time_en) {
            sensor_time_raw = (uint16_t)((fifo_buffer[cur + 1] << 8) | fifo_buffer[cur + 0]); cur += 2;
        }

        // Sanity check
        bool sane = true;
        if (fifo_acc_en) {
            if (abs(ax_raw) > 32767 || abs(ay_raw) > 32767 || abs(az_raw) > 32767) sane = false;
        }
        if (fifo_gyr_en) {
            if (abs(gx_raw) > 32767 || abs(gy_raw) > 32767 || abs(gz_raw) > 32767) sane = false;
        }

        if (!sane) {
            index++;
            skipped++;
            continue;
        }

        // Convert to physical units + calibration
        float ax_g = (fifo_acc_en ? (float)ax_raw / 4096.0f : 0.0f) - accel_cal.bias_x;
        float ay_g = (fifo_acc_en ? (float)ay_raw / 4096.0f : 0.0f) - accel_cal.bias_y;
        float az_g = (fifo_acc_en ? (float)az_raw / 4096.0f : 0.0f) - accel_cal.bias_z;

        float gx_dps = (fifo_gyr_en ? (float)gx_raw / 16.384f : 0.0f) - gyro_cal.bias_x;
        float gy_dps = (fifo_gyr_en ? (float)gy_raw / 16.384f : 0.0f) - gyro_cal.bias_y;
        float gz_dps = (fifo_gyr_en ? (float)gz_raw / 16.384f : 0.0f) - gyro_cal.bias_z;

        // 🔽 Apply low-pass filters here 🔽
        gx_dps = lpf_gx.step(gx_dps);
        gy_dps = lpf_gy.step(gy_dps);
        gz_dps = lpf_gz.step(gz_dps);

        ax_g   = lpf_ax.step(ax_g);
        ay_g   = lpf_ay.step(ay_g);
        az_g   = lpf_az.step(az_g);

        // Save filtered values
        latest_ax = ax_g; latest_ay = ay_g; latest_az = az_g;
        latest_gx = gx_dps; latest_gy = gy_dps; latest_gz = gz_dps;

        // Temp handling
        float temp_c = NAN;
        if (fifo_temp_en && temp_raw != (int16_t)0x8000) {
            temp_c = (float)temp_raw / 512.0f + 23.0f;
            temp_c = constrain(temp_c, -40.0f, 85.0f);
        }

        // --- dt calculation ---
        float dt = 0.0f;
        if (fifo_time_en) {
            if (have_bmi_time) {
                uint16_t d = (uint16_t)(sensor_time_raw - last_bmi_st); // handles wrap
                dt = d * bmi_time_lsb_sec;
            }
            last_bmi_st = sensor_time_raw;
            have_bmi_time = true;
        }
        if (dt <= 0.0f || dt > 0.02f) { // constrain to reasonable IMU ODR
            dt = (millis() - last_update_time) * 0.001f; // fallback
        }
        last_update_time = millis();

        // Use filtered values + dt in orientation update
        update_orientation_dt(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, dt);

       /* // Debug print
        Serial.print("S_T:0x");
        Serial.print(sensor_time_raw, HEX);
        Serial.printf(" | ms:%lu | ", millis());
        if (!isnan(temp_c)) Serial.printf("TEMP: %.2f°C | ", temp_c);
        else Serial.print("TEMP: [skip] | ");
        Serial.printf("ACC[g]: %.2f %.2f %.2f | GYRO[dps]: %.2f %.2f %.2f\n",
                      ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        */

        parsed++;
        index += frame_bytes;
    }

    if (index != bytes_to_read) {
        Serial.printf("[BMI323 FIFO] Misalign: read=%u, used=%d (frame_bytes=%u)\n",
                      bytes_to_read, index, (unsigned)frame_bytes);
    }
    if (parsed || skipped) {
       // Serial.printf("[BMI323 FIFO] Parsed=%d, Skipped=%d\n", parsed, skipped);
    }
}

// ----------------- Helper: dump critical regs -----------------
static void bmi323_debug_readback() {
    uint16_t err  = bmi323_readRegister(REG_ERR_REG);
    uint16_t ic   = bmi323_readRegister(REG_IO_INT_CTRL);
    uint16_t icf  = bmi323_readRegister(REG_INT_CONF);
    uint16_t m2   = bmi323_readRegister(REG_INT_MAP2);
    uint16_t fcf  = bmi323_readRegister(REG_FIFO_CONF);
    uint16_t wtm  = bmi323_readRegister(REG_FIFO_WTM);

    Serial.printf("[BMI323 DBG] ERR=0x%04X IO_INT=0x%04X INT_CONF=0x%04X INT_MAP2=0x%04X FIFO_CONF=0x%04X WTM=%u words\n",
                  err, ic, icf, m2, fcf, (unsigned)wtm);
}


//   Calibration Structures

static Preferences prefs;
#define GYRO_SAMPLES 500
#define ACCEL_SAMPLES 500

void bmi323_writeOffset(uint8_t reg, int16_t value, uint8_t bitWidth) {
    // 1. Read current register value
    uint16_t current = bmi323_readRegister(reg);

    // 2. Mask to only modify intended bits
    uint16_t mask = (1 << bitWidth) - 1;
    current &= ~mask;

    // 3. Mask new value to bitWidth
    uint16_t newVal = value & mask;

    // 4. Combine
    current |= newVal;

    // 5. Write back
    bmi323_writeRegister(reg, current);
}

int16_t bmi323_readOffset(uint8_t reg, uint8_t bitWidth) {
    // Read full register
    uint16_t raw = bmi323_readRegister(reg);

    // Mask the relevant bits
    int16_t val = raw & ((1 << bitWidth) - 1);

    // Sign-extend
    if (val & (1 << (bitWidth - 1))) {
        val |= ~((1 << bitWidth) - 1);
    }

    return val;
}

void wait_for_user_confirmation() {
    Serial.println("Place the drone level and press any key in Serial Monitor...");
    while (!Serial.available()) {
        delay(10);
    }
    while (Serial.available()) Serial.read();  // Clear buffer
}

bool bmi323_quick_gyro_calibrate(GyroCalibration* cal) {
    // 1. Ensure sensor is stable
    float temp_x = 0, temp_y = 0, temp_z = 0;

       // --- Ignore first 5 readings ---
    for (int i = 0; i < 5; i++) {
        bmi323_data_t dummy;
        bmi323_read(&dummy);
        delay(10);
    }
    
    // 2. Collect samples
    for(int i=0; i<GYRO_SAMPLES; i++) {
        bmi323_data_t data;
        if(!bmi323_read(&data)) return false;
        
        // inside bmi323_quick_gyro_calibrate
        temp_x += (float)data.gx / 16.384f;
        temp_y += (float)data.gy / 16.384f;
        temp_z += (float)data.gz / 16.384f;

        delay(10); // 100Hz sampling
    }

    // 3. Calculate biases
    cal->bias_x = temp_x / GYRO_SAMPLES;
    cal->bias_y = temp_y / GYRO_SAMPLES;
    cal->bias_z = temp_z / GYRO_SAMPLES;

    return true;
}


bool bmi323_accel_calibrate_all(AccelCalibration* cal) {
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < ACCEL_SAMPLES; i++) {
        float ax, ay, az;
        if (!bmi323_read_accel(&ax, &ay, &az)) return false;
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        delay(5);
    }

    float avg_x = sum_x / ACCEL_SAMPLES;
    float avg_y = sum_y / ACCEL_SAMPLES;
    float avg_z = sum_z / ACCEL_SAMPLES;

    // Correct handling:
    cal->bias_x = avg_x;            // should be ≈ 0
    cal->bias_y = avg_y;            // should be ≈ 0
    cal->bias_z = avg_z - 1.0f;     // remove gravity, keep ~+1g at rest

    return true;
}



void apply_gyro_calibration(const GyroCalibration* cal) {
    // Convert from dps to 10-bit register units
    int16_t ox = (int16_t)(cal->bias_x / 0.061f);
    int16_t oy = (int16_t)(cal->bias_y / 0.061f);
    int16_t oz = (int16_t)(cal->bias_z / 0.061f);

    // Constrain to 10-bit signed range
    ox = constrain(ox, -512, 511);
    oy = constrain(oy, -512, 511);
    oz = constrain(oz, -512, 511);

    // Write offsets safely using RMW approach
    bmi323_writeOffset(0x66, ox, 10); // GYR_DP_OFF_X
    bmi323_writeOffset(0x68, oy, 10); // GYR_DP_OFF_Y
    bmi323_writeOffset(0x6A, oz, 10); // GYR_DP_OFF_Z

    // Read back for verification
    int16_t rx = bmi323_readOffset(0x66, 10);
    int16_t ry = bmi323_readOffset(0x68, 10);
    int16_t rz = bmi323_readOffset(0x6A, 10);

   // Serial.printf("[GYRO CAL] Written offsets: X=%d | Y=%d | Z=%d\n", ox, oy, oz);
   // Serial.printf("[GYRO CAL] Read back offsets: X=%d | Y=%d | Z=%d\n", rx, ry, rz);
}

void apply_accel_calibration(const AccelCalibration* cal) {
    // Convert from m/s² to 14-bit register units
    int16_t ox = (int16_t)(cal->bias_x / 0.00003052f);
    int16_t oy = (int16_t)(cal->bias_y / 0.00003052f);
    int16_t oz = (int16_t)(cal->bias_z / 0.00003052f);

    // Constrain to 14-bit signed range
    ox = constrain(ox, -8192, 8191);
    oy = constrain(oy, -8192, 8191);
    oz = constrain(oz, -8192, 8191);

    // Write offsets safely using RMW approach
    bmi323_writeOffset(0x60, ox, 14); // ACC_DP_OFF_X
    bmi323_writeOffset(0x62, oy, 14); // ACC_DP_OFF_Y
    bmi323_writeOffset(0x64, oz, 14); // ACC_DP_OFF_Z

    // Read back for verification
    int16_t rx = bmi323_readOffset(0x60, 14);
    int16_t ry = bmi323_readOffset(0x62, 14);
    int16_t rz = bmi323_readOffset(0x64, 14);

   // Serial.printf("[ACCEL CAL] Written offsets: X=%d | Y=%d | Z=%d\n", ox, oy, oz);
   // Serial.printf("[ACCEL CAL] Read back offsets: X=%d | Y=%d | Z=%d\n", rx, ry, rz);
}