#pragma once

#include <Arduino.h>
#include <Wire.h>

// LSM6DS3 I2C Address
#define LSM6DS3_ADDRESS_LOW   0x6A  // SA0 = LOW
#define LSM6DS3_ADDRESS_HIGH  0x6B  // SA0 = HIGH

// LSM6DS3 Registers
#define LSM6DS3_WHO_AM_I      0x0F  // Who am I register (should return 0x69)
#define LSM6DS3_CTRL1_XL      0x10  // Accelerometer control register
#define LSM6DS3_CTRL2_G       0x11  // Gyroscope control register
#define LSM6DS3_CTRL3_C       0x12  // Control register 3
#define LSM6DS3_STATUS_REG    0x1E  // Status register
#define LSM6DS3_OUTX_L_G      0x22  // Gyroscope X-axis (LSB)
#define LSM6DS3_OUTX_H_G      0x23  // Gyroscope X-axis (MSB)
#define LSM6DS3_OUTY_L_G      0x24  // Gyroscope Y-axis (LSB)
#define LSM6DS3_OUTY_H_G      0x25  // Gyroscope Y-axis (MSB)
#define LSM6DS3_OUTZ_L_G      0x26  // Gyroscope Z-axis (LSB)
#define LSM6DS3_OUTZ_H_G      0x27  // Gyroscope Z-axis (MSB)
#define LSM6DS3_OUTX_L_XL     0x28  // Accelerometer X-axis (LSB)
#define LSM6DS3_OUTX_H_XL     0x29  // Accelerometer X-axis (MSB)
#define LSM6DS3_OUTY_L_XL     0x2A  // Accelerometer Y-axis (LSB)
#define LSM6DS3_OUTY_H_XL     0x2B  // Accelerometer Y-axis (MSB)
#define LSM6DS3_OUTZ_L_XL     0x2C  // Accelerometer Z-axis (LSB)
#define LSM6DS3_OUTZ_H_XL     0x2D  // Accelerometer Z-axis (MSB)
#define LSM6DS3_OUT_TEMP_L    0x20  // Temperature (LSB)
#define LSM6DS3_OUT_TEMP_H    0x21  // Temperature (MSB)

/**
 * IMU data structure (accel + gyro)
 * All values are floating-point in physical units
 */
struct IMUData {
    float accel_x;   // Acceleration in g (-16 to +16)
    float accel_y;
    float accel_z;
    float gyro_x;    // Angular velocity in dps (-2000 to +2000)
    float gyro_y;
    float gyro_z;
    float temperature; // Temperature in Celsius
    
    // Default constructor
    IMUData() : accel_x(0), accel_y(0), accel_z(0), 
                gyro_x(0), gyro_y(0), gyro_z(0), temperature(0) {}
};

/**
 * LSM6DS3 Driver class
 * Lightweight I2C driver without external dependencies
 */
class LSM6DS3Driver {
public:
    /**
     * Constructor for LSM6DS3 driver
     */
    LSM6DS3Driver(uint8_t address = LSM6DS3_ADDRESS_LOW);
    
    /**
     * Initialize the LSM6DS3 sensor
     * @param wire TwoWire instance (default: &Wire)
     * @return true if initialization successful
     */
    bool begin(TwoWire* wire = &Wire);
    
    /**
     * Check if sensor is ready
     * @return true if sensor is initialized and responding
     */
    bool isReady() const { return initialized; }
    
    /**
     * Read all sensor data (accel, gyro, temperature)
     * @param data Reference to IMUData struct to fill
     * @return true if read successful
     */
    bool readData(IMUData& data);
    
    /**
     * Get last error message
     * @return Error description string
     */
    const char* getLastError() const { return last_error; }
    
private:
    // I2C helper functions
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    int16_t readWord(uint8_t reg_low);
    
    uint8_t i2c_address;
    TwoWire* wire;
    bool initialized;
    const char* last_error;
    
    // Sensitivity values (depends on configured range)
    float accel_sensitivity;   // in g/LSB
    float gyro_sensitivity;    // in dps/LSB
};
