#include "lsm6ds3_driver.h"
#include "../config.h"

LSM6DS3Driver::LSM6DS3Driver(uint8_t address)
    : i2c_address(address)
    , wire(nullptr)
    , initialized(false)
    , last_error(nullptr)
    , accel_sensitivity(0.000488f)  // ±16g = 0.488 mg/LSB
    , gyro_sensitivity(0.070f)       // ±2000 dps = 70 mdps/LSB
{
}

bool LSM6DS3Driver::begin(TwoWire* wire_instance) {
    wire = wire_instance;
    
    DEBUG_UART.println(F("[LSM6DS3] Initializing IMU sensor..."));
    
    // Check WHO_AM_I register
    uint8_t who_am_i = readRegister(LSM6DS3_WHO_AM_I);
    if (who_am_i != 0x69) {
        last_error = "LSM6DS3 not detected (WHO_AM_I failed)";
        DEBUG_UART.print(F("[LSM6DS3] ERROR: WHO_AM_I returned 0x"));
        DEBUG_UART.println(who_am_i, HEX);
        DEBUG_UART.println(F("[LSM6DS3] Check I2C wiring (SDA=PB7, SCL=PB6)"));
        return false;
    }
    
    // Configure gyroscope: ±2000 dps, 104 Hz
    // CTRL2_G (0x11): FS_G[1:0] = 11 (±2000 dps), ODR_G[3:0] = 1000 (104 Hz)
    writeRegister(LSM6DS3_CTRL2_G, 0x8C);  // 0x8C = 1000 1100
    
    // Configure accelerometer: ±16g, 104 Hz
    // CTRL1_XL (0x10): FS_XL[1:0] = 11 (±16g), ODR_XL[3:0] = 1000 (104 Hz)
    writeRegister(LSM6DS3_CTRL1_XL, 0x8C);  // 0x8C = 1000 1100
    
    // Enable Block Data Update (BDU) for consistent reads
    // CTRL3_C (0x12): BDU = 1 (bit 6)
    writeRegister(LSM6DS3_CTRL3_C, 0x44);  // 0x44 = 0100 0100
    
    initialized = true;
    DEBUG_UART.println(F("[LSM6DS3] Initialization successful"));
    DEBUG_UART.println(F("[LSM6DS3] Accel: ±16g, Gyro: ±2000dps @ 104Hz"));
    
    return true;
}

bool LSM6DS3Driver::readData(IMUData& data) {
    if (!initialized || wire == nullptr) {
        last_error = "Sensor not initialized";
        return false;
    }
    
    // Read gyroscope data
    int16_t gyro_x_raw = readWord(LSM6DS3_OUTX_L_G);
    int16_t gyro_y_raw = readWord(LSM6DS3_OUTY_L_G);
    int16_t gyro_z_raw = readWord(LSM6DS3_OUTZ_L_G);
    
    // Read accelerometer data
    int16_t accel_x_raw = readWord(LSM6DS3_OUTX_L_XL);
    int16_t accel_y_raw = readWord(LSM6DS3_OUTY_L_XL);
    int16_t accel_z_raw = readWord(LSM6DS3_OUTZ_L_XL);
    
    // Read temperature data
    int16_t temp_raw = readWord(LSM6DS3_OUT_TEMP_L);
    
    // Convert to physical units
    data.gyro_x = gyro_x_raw * gyro_sensitivity;
    data.gyro_y = gyro_y_raw * gyro_sensitivity;
    data.gyro_z = gyro_z_raw * gyro_sensitivity;
    
    data.accel_x = accel_x_raw * accel_sensitivity;
    data.accel_y = accel_y_raw * accel_sensitivity;
    data.accel_z = accel_z_raw * accel_sensitivity;
    
    // Temperature formula from datasheet: T(°C) = (TEMP_OUT / 256) + 25
    data.temperature = (temp_raw / 256.0f) + 25.0f;
    
    return true;
}

void LSM6DS3Driver::writeRegister(uint8_t reg, uint8_t value) {
    wire->beginTransmission(i2c_address);
    wire->write(reg);
    wire->write(value);
    wire->endTransmission();
}

uint8_t LSM6DS3Driver::readRegister(uint8_t reg) {
    wire->beginTransmission(i2c_address);
    wire->write(reg);
    wire->endTransmission(false);
    
    wire->requestFrom((int)i2c_address, 1);
    if (wire->available()) {
        return wire->read();
    }
    return 0;
}

int16_t LSM6DS3Driver::readWord(uint8_t reg_low) {
    wire->beginTransmission(i2c_address);
    wire->write(reg_low);
    wire->endTransmission(false);
    
    wire->requestFrom((int)i2c_address, 2);
    if (wire->available() >= 2) {
        uint8_t lsb = wire->read();
        uint8_t msb = wire->read();
        return (int16_t)((msb << 8) | lsb);
    }
    return 0;
}
