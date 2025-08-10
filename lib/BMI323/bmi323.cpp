#include "bmi323.h"
#include <SPI.h>
#include <Preferences.h>
#include <math.h>

static SPISettings bmi323_spi_settings(6500000, MSBFIRST, SPI_MODE0);
static Preferences prefs;
static uint8_t fifo_buffer[FIFO_BUFFER_SIZE];

// ----------------------------
// Global Variables
// ----------------------------

bmi323_data_t sensor_data = {};
GyroCalibration gyro_cal = {};
AccelCalibration accel_cal = {};
FlightMode current_mode = MODE_STABLE;
const FlightModeConfig* current_config = nullptr;

float estimated_pitch = 0.0f;
float estimated_roll  = 0.0f;
float estimated_yaw   = 0.0f;
unsigned long last_update_time = 0;

// ----------------------------
// SPI Communication
// ----------------------------

void bmi323_writeRegister(uint8_t reg, uint16_t value) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction();
}

uint16_t bmi323_readRegister(uint8_t reg) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(0x00);
    uint8_t lsb = SPI.transfer(0x00);
    uint8_t msb = SPI.transfer(0x00);
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction();
    return (msb << 8) | lsb;
}

void bmi323_burstRead(uint8_t reg, uint8_t* buffer, uint16_t length) {
    SPI.beginTransaction(bmi323_spi_settings); // Added to ensure SPI mode 0
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(reg | 0x80);
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(BMI323_CS_PIN, HIGH);
    SPI.endTransaction(); // Added to ensure bus release and mode restore
}


// ----------------------------
// Feature Engine Handling
// ----------------------------

static bool waitForFeatureDataReady(uint8_t max_retries = 50) {
    for (uint8_t i = 0; i < max_retries; ++i) {
        if ((bmi323_readRegister(0x43) >> 1) & 0x01) return true;
        delay(1);
    }
    return false;
}

bool bmi323_writeExtendedRegister(uint8_t extReg, uint16_t value) {
    bmi323_writeRegister(0x41, extReg);
    if (!waitForFeatureDataReady()) return false;
    bmi323_writeRegister(0x42, value);
    return true;
}

// ----------------------------
// Sensor Initialization
// ----------------------------

bool bmi323_init(void) {
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, BMI323_CS_PIN);
    pinMode(BMI323_CS_PIN, OUTPUT);
    digitalWrite(BMI323_CS_PIN, HIGH);
    delay(10);

    if ((bmi323_readRegister(CHIP_ID_REG) & 0xFF) != CHIP_ID_EXPECTED)
        return false;

    bmi323_writeRegister(CMD_REG, RESET_CMD);
    delay(200);

    bmi323_writeRegister(0x12, 0x012C);  // Startup config
    bmi323_writeRegister(0x14, 0x0001);
    delay(10);
    bmi323_writeRegister(FEATURE_CTRL_REG, 0x0001);
    delay(5);

    for (int i = 0; i < 50; ++i) {
        if ((bmi323_readRegister(FEATURE_IO1_REG) & 0x0F) == 0x01) break;
        delay(10);
    }

    bmi323_writeRegister(ACC_CONF_REG, 0x70A9);
    bmi323_writeRegister(GYR_CONF_REG, 0x70C9);

    return true;
}

bool bmi323_read(bmi323_data_t* data) {
    for (uint8_t i = 0; i < 10; ++i) {
        if (bmi323_readRegister(STATUS_REG) & (1 << 7)) break;
        delay(10);
    }

    uint8_t buffer[13];
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(BMI323_CS_PIN, LOW);
    SPI.transfer(ACC_X_REG | 0x80);
    buffer[0] = SPI.transfer(0x00);
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

// ----------------------------
// Orientation Estimation
// ----------------------------

void update_orientation(float ax, float ay, float az, float gx, float gy, float gz) {
    const float alpha = 0.98f;
    unsigned long now = millis();
    float dt = (now - last_update_time) / 1000.0f;
    last_update_time = now;

    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    float acc_roll  = atan2(ay, az) * 180.0f / PI;

    estimated_pitch = alpha * (estimated_pitch + gx * dt) + (1 - alpha) * acc_pitch;
    estimated_roll  = alpha * (estimated_roll + gy * dt) + (1 - alpha) * acc_roll;
    estimated_yaw  += gz * dt;
}

// ----------------------------
// FIFO Interface
// ----------------------------

void bmi323_flush_fifo() {
    bmi323_writeRegister(0x37, 0x0001);
    delay(2);
    while ((bmi323_readRegister(0x15) & 0x07FF) != 0) delay(1);
}

void bmi323_setup_fifo() {
    bmi323_writeRegister(0x7D, 0x0000);
    delay(10);

    uint16_t fifo_conf = bmi323_readRegister(0x36);
    fifo_conf |= (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11);
    bmi323_writeRegister(0x36, fifo_conf);

    bmi323_writeRegister(0x35, 48);
    uint16_t int_map2 = bmi323_readRegister(0x3B);
    int_map2 = (int_map2 & ~(0b11 << 12)) | (0b01 << 12);
    bmi323_writeRegister(0x3B, int_map2);

    uint16_t int_conf = bmi323_readRegister(0x39);
    int_conf |= (1 << 0);
    bmi323_writeRegister(0x39, int_conf);

    bmi323_flush_fifo();
}

void bmi323_read_fifo() {
    uint16_t fifo_fill_words = bmi323_readRegister(0x15) & 0x07FF;
    if (fifo_fill_words == 0) return;

    int bytes_to_read = min(fifo_fill_words * 2, FIFO_BUFFER_SIZE - 1);
    uint8_t raw[FIFO_BUFFER_SIZE + 1] = {0};
    bmi323_burstRead(0x16, raw, bytes_to_read + 1);
    memcpy(fifo_buffer, raw + 1, bytes_to_read);

    int index = 0;
    while (index + FIFO_FRAME_SIZE <= bytes_to_read) {
        int16_t ax = (fifo_buffer[index + 1] << 8) | fifo_buffer[index + 0];
        int16_t ay = (fifo_buffer[index + 3] << 8) | fifo_buffer[index + 2];
        int16_t az = (fifo_buffer[index + 5] << 8) | fifo_buffer[index + 4];
        int16_t gx = (fifo_buffer[index + 7] << 8) | fifo_buffer[index + 6];
        int16_t gy = (fifo_buffer[index + 9] << 8) | fifo_buffer[index + 8];
        int16_t gz = (fifo_buffer[index +11] << 8) | fifo_buffer[index +10];
        index += FIFO_FRAME_SIZE;

        if (ax == DUMMY_ACCEL || gx == DUMMY_GYRO) continue;

        float ax_g = ax / 4096.0f;
        float ay_g = ay / 4096.0f;
        float az_g = az / 4096.0f - accel_cal.z_offset;
        float gx_dps = gx / 16.384f - gyro_cal.bias_x;
        float gy_dps = gy / 16.384f - gyro_cal.bias_y;
        float gz_dps = gz / 16.384f - gyro_cal.bias_z;

        update_orientation(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
    }
}


// ----------------------------// Calibration Handling
// ----------------------------     


#define GYRO_SAMPLES 200
#define ACCEL_SAMPLES 100

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
        
        temp_x += data.gx;
        temp_y += data.gy;
        temp_z += data.gz;
        delay(10); // 100Hz sampling
    }

    // 3. Calculate biases
    cal->bias_x = temp_x / GYRO_SAMPLES;
    cal->bias_y = temp_y / GYRO_SAMPLES;
    cal->bias_z = temp_z / GYRO_SAMPLES;

    return true;
}

bool bmi323_z_accel_calibrate(AccelCalibration* cal) {
    // 1. Assume level surface (Z-axis should be +1g)
    float temp_z = 0;

    // --- Ignore first 5 readings ---
    for (int i = 0; i < 5; i++) {
        bmi323_data_t dummy;
        bmi323_read(&dummy);
        delay(20);
    }
    
    // 2. Collect samples
    for(int i=0; i<ACCEL_SAMPLES; i++) {
        bmi323_data_t data;
        if(!bmi323_read(&data)) return false;
        
        temp_z += data.az / 4096.0f; // Convert to g
        delay(20);
    }

    // 3. Calculate offset from ideal 1g
    cal->z_offset = (temp_z / ACCEL_SAMPLES) - 1.0f;
    return true;
}

void apply_gyro_calibration(const GyroCalibration* cal) {
    int16_t ox = (int16_t)(cal->bias_x * 16.384f);
    int16_t oy = (int16_t)(cal->bias_y * 16.384f);
    int16_t oz = (int16_t)(cal->bias_z * 16.384f);

    bmi323_writeRegister(0x66, ox); // GYR_DP_OFF_X
    bmi323_writeRegister(0x68, oy); // GYR_DP_OFF_Y
    bmi323_writeRegister(0x6A, oz); // GYR_DP_OFF_Z
}

void apply_accel_calibration(const AccelCalibration* cal) {
    int16_t oz = (int16_t)(cal->z_offset / 0.00003052f); // offset in LSBs
    bmi323_writeRegister(0x64, oz); // ACC_DP_OFF_Z
}

bool load_calibration_from_flash(GyroCalibration& gyro_cal, AccelCalibration& accel_cal) {
    prefs.begin("bmi323", true);  // Read-only
    if (!prefs.isKey("gyro_x")) {
        prefs.end();
        Serial.println("[FLASH] No calibration found.");
        return false;
    }

    gyro_cal.bias_x = prefs.getFloat("gyro_x");
    gyro_cal.bias_y = prefs.getFloat("gyro_y");
    gyro_cal.bias_z = prefs.getFloat("gyro_z");
    accel_cal.z_offset = prefs.getFloat("accel_z");

    prefs.end();

    Serial.println("[FLASH] Calibration loaded from NVS:");
    Serial.printf("  Gyro Bias: X=%.2f Y=%.2f Z=%.2f\n", gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    Serial.printf("  Accel Z Offset: %.3f\n", accel_cal.z_offset);
    return true;
}

void save_calibration_to_flash(const GyroCalibration& gyro_cal, const AccelCalibration& accel_cal) {
    prefs.begin("bmi323", false);  // Read-write
    prefs.putFloat("gyro_x", gyro_cal.bias_x);
    prefs.putFloat("gyro_y", gyro_cal.bias_y);
    prefs.putFloat("gyro_z", gyro_cal.bias_z);
    prefs.putFloat("accel_z", accel_cal.z_offset);
    prefs.end();
    Serial.println("[FLASH] Calibration saved to NVS.");
}

void clear_calibration_flash() {
    prefs.begin("bmi323", false);
    prefs.clear();
    prefs.end();
    Serial.println("[FLASH] Calibration erased from NVS.");
}

bool user_requested_recalibration() {
    Serial.println("Press 'c' to force calibration (within 5 seconds)...");

    unsigned long start = millis();
    while (millis() - start < 5000) {
        if (Serial.available()) {
            char ch = Serial.read();
            if (ch == 'c' || ch == 'C') return true;
        }
        delay(10);
    }
    return false;
}

void perform_calibration_sequence() {
    wait_for_user_confirmation();  // Ask user to place flat

    if (!bmi323_quick_gyro_calibrate(&gyro_cal)) {
        Serial.println("[ERROR] Gyro calibration failed.");
        while (1);
    }

    if (!bmi323_z_accel_calibrate(&accel_cal)) {
        Serial.println("[ERROR] Accel Z calibration failed.");
        while (1);
    }

    save_calibration_to_flash(gyro_cal, accel_cal);
    apply_gyro_calibration(&gyro_cal);
    apply_accel_calibration(&accel_cal);

    Serial.println("[CAL] Calibration completed and saved.");
}

void print_calibration_info() {
    Serial.println("Calibration Results:");
    Serial.printf("  Gyro Bias: X=%.2f Y=%.2f Z=%.2f\n",
                  gyro_cal.bias_x, gyro_cal.bias_y, gyro_cal.bias_z);
    Serial.printf("  Accel Z Offset: %.3f\n", accel_cal.z_offset);
}
