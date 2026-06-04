#pragma once
#include <Arduino.h>

// ===========================================================================
//  STM32F411CEU6 GLOBAL CONFIGURATION (Drone & Controller)
// ===========================================================================

#define DEBUG_UART          Serial
#define SEA_LEVEL_PRESSURE  1013.25f

// --- Status LEDs Controller (Neu) ---
#define LED_GREEN_PIN       PB1   // Grüne Status-LED (z.B. Verbindung OK)
#define LED_RED_PIN         PB10  // Rote Status-LED (z.B. Fehler / Akkuwarnung)

// --- I2C Bus Pins (Sensors & OLED) ---
// HINWEIS: An diesem Bus hängen parallel das OLED-Display (0x3C)
// und das lokale Barometer BMP280 (0x76). Keine Adresskonflikte!
#define I2C_SDA_PIN         PB7
#define I2C_SCL_PIN         PB6

// --- Sensor I2C Addresses ---
#define LSM6DS3_ADDRESS     0x6A
#define MY_BMP280_ADDRESS   0x76
#define OLED_I2C_ADDR       0x3C

// --- NRF24L01 SPI Pins & Config ---
#define NRF24_SPI_MOSI      PA7
#define NRF24_SPI_MISO      PA6
#define NRF24_SPI_SCK       PA5
#define NRF24_CE            PA8
#define NRF24_CSN           PB12

#define NRF24_CHANNEL       76
#define NRF24_DATA_RATE     RF24_2MBPS
#define NRF24_PA_LEVEL      RF24_PA_LOW

static const uint8_t CONTROLLER_ADDRESS[5] = {'C', 'T', 'R', '0', '1'};

// ===========================================================================
//  DATA STRUCTURES
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

// ===========================================================================
//  CONTROLLER CONFIG
// ===========================================================================

// ===========================================================================
//  CONTROLLER CONFIG
// ===========================================================================
#define LEFT_X_PIN          PA0
#define LEFT_Y_PIN          PA1
#define LEFT_SW_PIN         PA4
#define RIGHT_X_PIN         PA2
#define RIGHT_Y_PIN         PA3
#define RIGHT_SW_PIN        PB3

// --- Controller Akku-Messung ---
// ===========================================================================
//  CONTROLLER CONFIG
// ===========================================================================
#define LEFT_X_PIN          PA0
#define LEFT_Y_PIN          PA1
#define LEFT_SW_PIN         PA4
#define RIGHT_X_PIN         PA2
#define RIGHT_Y_PIN         PA3
#define RIGHT_SW_PIN        PB3

// --- Angepasst für 3x 1.5V Batterien am Controller ---
#define CTRL_BAT_ADC_PIN     PB0
#define CTRL_BAT_DIVIDER_FACTOR 1.5f  
#define CTRL_BAT_VREF_MV     3300
#define CTRL_BAT_MIN_MV      3000     // 3x 1.0V (Batterien fast leer)
#define CTRL_BAT_MAX_MV      4500     // 3x 1.5V (Frische Batterien)

// --- Grenzen für die Drohne (1S LiPo) ---
#define DRONE_BAT_MIN_MV     3200
#define DRONE_BAT_MAX_MV     4200
// ---------------------------------------

#define BUZZER_PIN          PA15
#define ADC_CENTER_VALUE    2047
#define ADC_DEADBAND        50
#define ADC_MIN_VALUE       0
#define ADC_MAX_VALUE       4095

#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_UPDATE_MS      100
#define SEND_INTERVAL_MS    20
#define BUTTON_COOLDOWN_MS  200

// ===========================================================================
//  DRONE CONFIG
// ===========================================================================

#define MOTOR_FL_PIN        PA0
#define MOTOR_FR_PIN        PA1
#define MOTOR_RL_PIN        PA2
#define MOTOR_RR_PIN        PA3

#define MOTOR_SPEED_MIN     0
#define MOTOR_SPEED_MAX     1000
#define MOTOR_IDLE_SPEED    50
#define MOTOR_PWM_FREQ_HZ   400

#define BAT_ADC_PIN         PC_0
#define BAT_DIVIDER_FACTOR  1.5f
#define BAT_ADC_VREF_MV     3300
#define BAT_ADC_BITS        12

#define FLIGHT_LOOP_HZ      250
#define FLIGHT_LOOP_US      (1000000UL / FLIGHT_LOOP_HZ)
#define FAILSAFE_TIMEOUT_MS 500

#define PID_ROLL_P          2.5f
#define PID_ROLL_I          0.05f
#define PID_ROLL_D          0.5f
#define PID_PITCH_P         2.5f
#define PID_PITCH_I         0.05f
#define PID_PITCH_D         0.5f
#define PID_YAW_P           3.0f
#define PID_YAW_I           0.10f
#define PID_YAW_D           0.0f
#define PID_INTEGRAL_LIMIT  200.0f

#define MAX_ANGLE_DEG       25.0f
#define MAX_YAW_RATE        90.0f