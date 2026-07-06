#pragma once
#include <Arduino.h>

// ===========================================================================
//  STM32F411CEU6 CONFIGURATION — CONTROLLER (Sender / transmitter.cpp)
// ===========================================================================

#define DEBUG_UART          Serial1
#define SEA_LEVEL_PRESSURE  1013.25f

// --- UART Debug (Serial1) ---
#define UART_TX_PIN         PA9
#define UART_RX_PIN         PA10

// --- Dashboard LEDs ---
#define LED_GREEN_PIN       PB1
#define LED_RED_PIN         PB10
#define LED_YELLOW_PIN      PB2
#define LED_BLUE_PIN        PB9

// --- I2C Bus (OLED SSD1306 + lokales BMP280) ---
#define I2C_SDA_PIN         PB7
#define I2C_SCL_PIN         PB6

// --- Sensor I2C Adressen ---
#define MY_BMP280_ADDRESS   0x76
#define OLED_I2C_ADDR       0x3C

// --- NRF24L01 SPI ---
#define NRF24_SPI_MOSI      PA7
#define NRF24_SPI_MISO      PA6
#define NRF24_SPI_SCK       PA5
#define NRF24_CE            PA8
#define NRF24_CSN           PB12

#define NRF24_CHANNEL       76
#define NRF24_DATA_RATE     RF24_2MBPS
#define NRF24_PA_LEVEL      RF24_PA_LOW

// --- NRF24 Adresse ---
static const uint8_t CONTROLLER_ADDRESS[5] = {'C', 'T', 'R', '0', '1'};

// --- Thumbsticks ---
#define LEFT_X_PIN          PA0
#define LEFT_Y_PIN          PA1
#define LEFT_SW_PIN         PA4
#define RIGHT_X_PIN         PA2
#define RIGHT_Y_PIN         PA3
#define RIGHT_SW_PIN        PB3

// --- Piezo Buzzer ---
#define BUZZER_PIN          PA15

// --- Controller Batterie (Spannungsteiler an PB0) ---
#define CTRL_BAT_ADC_PIN        PB0
#define CTRL_BAT_DIVIDER_FACTOR 1.5f
#define CTRL_BAT_VREF_MV        3300
#define CTRL_BAT_MIN_MV         3000
#define CTRL_BAT_MAX_MV         4500

// --- Drohnen Batterie (via Telemetrie, nur Grenzwerte) ---
#define DRONE_BAT_MIN_MV        3200
#define DRONE_BAT_MAX_MV        4200

// --- ADC Kalibrierung (Thumbsticks) ---
#define ADC_CENTER_VALUE    2047
#define ADC_DEADBAND        50
#define ADC_MIN_VALUE       0
#define ADC_MAX_VALUE       4095

// --- Display & Timing ---
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_UPDATE_MS      100
#define SEND_INTERVAL_MS    20
#define BUTTON_COOLDOWN_MS  200

// ===========================================================================
//  DATA STRUCTURES (müssen 1:1 mit config_drohne.h übereinstimmen!)
// ===========================================================================

struct __attribute__((packed)) ControllerData {
    uint16_t left_x;
    uint16_t left_y;
    uint16_t right_x;
    uint16_t right_y;
    uint8_t  left_click;
    uint8_t  right_click;
    uint8_t  battery_level;
    uint8_t  checksum;
};

struct __attribute__((packed)) SensorData {
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  altitude;
    uint16_t battery_mv;
    uint8_t  status;
    uint8_t  checksum;
};