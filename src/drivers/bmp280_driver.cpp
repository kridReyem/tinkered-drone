/**
 * BMP-280 Barometric Pressure Sensor Driver Implementation
 */

#include "bmp280_driver.h"

BMP280Driver::BMP280Driver(uint8_t address)
    : _address(address), _initialized(false), _sea_level_pressure(1013.25) {
}

bool BMP280Driver::init() {
    // Try to initialize with the specified I2C address
    // Adafruit_BMP280 uses Wire by default (already initialized in main)
    
    // Try address 0x76 first if that was specified
    if (_bmp.begin(_address)) {
        _initialized = true;
        
        // Configure sensor for weather monitoring (default settings)
        // Ultra-low power, minimal noise
    // Tipp für eine spätere Optimierung in deiner bmp280_driver.cpp:
    _bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,     // Fortlaufende Messung
        Adafruit_BMP280::SAMPLING_X2,     // Temperatur-Oversampling
        Adafruit_BMP280::SAMPLING_X16,    // Hohes Druck-Oversampling für präzise Höhe
        Adafruit_BMP280::FILTER_X16,      // Filter gegen Luftschübe/Rauschen
        Adafruit_BMP280::STANDBY_MS_63    // Schnelle Aktualisierung (~15Hz)
    );
        
        return true;
    }
    
    // Try alternative address (0x77) if first failed
    uint8_t alt_address = (_address == 0x76) ? 0x77 : 0x76;
    if (_bmp.begin(alt_address)) {
        _address = alt_address;  // Remember successful address
        _initialized = true;
        
        _bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X1,
            Adafruit_BMP280::SAMPLING_X1,
            Adafruit_BMP280::FILTER_OFF,
            Adafruit_BMP280::STANDBY_MS_1000
        );
        
        return true;
    }
    
    _initialized = false;
    return false;
}

void BMP280Driver::readData(float& pressure, float& temperature, float& altitude) {
    if (!_initialized) {
        pressure = 0;
        temperature = 0;
        altitude = 0;
        return;
    }
    
    pressure = _bmp.readPressure();      // Pa
    temperature = _bmp.readTemperature(); // °C
    altitude = _bmp.readAltitude(_sea_level_pressure);  // Convert hPa to Pa
}

float BMP280Driver::getPressureHPa() {
    if (!_initialized) return 0;
    return _bmp.readPressure() / 100.0;  // Convert Pa to hPa
}

float BMP280Driver::getTemperature() {
    if (!_initialized) return 0;
    return _bmp.readTemperature();
}

float BMP280Driver::getAltitude(float seaLevelPressure) {
    if (!_initialized) return 0;
    // readAltitude expects sea-level pressure in Pa
    return _bmp.readAltitude(seaLevelPressure * 100);
}

void BMP280Driver::setSeaLevelPressure(float pressure) {
    _sea_level_pressure = pressure;
}

bool BMP280Driver::isInitialized() {
    return _initialized;
}
