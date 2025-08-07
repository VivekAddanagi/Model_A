/*
#include <Arduino.h>
#include <SPI.h>
#include "bmi323.h"

// --- Internal SPI Configuration ---
static SPISettings bmi323_spi_settings(6500000, MSBFIRST, SPI_MODE0);

// --- Internal Helper Functions ---
 void bmi323_writeRegister(uint8_t reg, uint16_t value) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);              // Write
    SPI.transfer(value & 0xFF);            // LSB
    SPI.transfer((value >> 8) & 0xFF);     // MSB
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
}

 uint16_t bmi323_readRegister(uint8_t reg) {
    SPI.beginTransaction(bmi323_spi_settings);
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(reg | 0x80);              // Read
    SPI.transfer(0x00);                    // Dummy
    uint8_t lsb = SPI.transfer(0x00);
    uint8_t msb = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
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
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
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
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(ACC_X_REG | 0x80);
    buffer[0] = SPI.transfer(0x00); // Dummy
    for (int i = 1; i < 13; ++i) buffer[i] = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
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
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(reg | 0x80); // Set read bit
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(CS_PIN, HIGH);
}



// ------------------------
// Mode Selection Prompt
// ------------------------
FlightMode select_flight_mode() {
    Serial.println("Select Flight Mode:");
    Serial.println("  1 - Stable (Leveling)");
    Serial.println("  2 - Hover (Altitude Hold)");
    Serial.println("  3 - Cruise (Forward Flight)");
    Serial.print("Enter choice (1-3): ");

    while (true) {
        if (Serial.available()) {
            char ch = Serial.read();
            switch (ch) {
                case '1': Serial.println("Selected: Stable Mode"); return MODE_STABLE;
                case '2': Serial.println("Selected: Hover Mode");  return MODE_HOVER;
                case '3': Serial.println("Selected: Cruise Mode"); return MODE_CRUISE;
                default:
                    Serial.println("Invalid input. Enter 1, 2 or 3:");
            }
        }
        delay(10);
    }
}

// ------------------------
// Complementary Filter
// ------------------------
void update_orientation(float ax, float ay, float az, float gx, float gy, float gz) {
    const float alpha = 0.98f;
    unsigned long now = millis();
    float dt = (now - last_update_time) / 1000.0f;
    last_update_time = now;

    float gyro_pitch_delta = gx * dt;
    float gyro_roll_delta  = gy * dt;
    float gyro_yaw_delta   = gz * dt;

    float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    float acc_roll  = atan2(ay, az) * 180.0f / PI;

    estimated_pitch = alpha * (estimated_pitch + gyro_pitch_delta) + (1 - alpha) * acc_pitch;
    estimated_roll  = alpha * (estimated_roll + gyro_roll_delta) + (1 - alpha) * acc_roll;
    estimated_yaw   += gyro_yaw_delta;  // Simple integration for yaw (no accelerometer support)
}

*/