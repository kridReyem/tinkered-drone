# STM32F411 Quadcopter Flight Controller & TX

A custom STM32F411-based flight controller system featuring a 250 Hz PID loop, NRF24L01+ control link, and a 3D visualizer transmitter.

## What it does
This project implements a complete drone flight stack on the STM32F411CEU6 "Black Pill". The **Receiver** runs a 250 Hz control loop using Kalman-filtered IMU data to stabilize a quadcopter via PID. The **Transmitter** sends stick commands over NRF24 and visualizes the drone's orientation in real-time on a 128x64 OLED.

## Hardware
*   **MCU**: STM32F411CEU6 Black Pill (100 MHz, 512KB Flash)
*   **Radio**: NRF24L01+ (2.4 GHz, SPI)
*   **IMU**: LSM6DS3 (Accel/Gyro, I2C)
*   **Barometer**: BMP280 (I2C)
*   **Display**: SSD1306 128x64 OLED (I2C)
*   **Input**: 2x KY-023 Thumbsticks (ADC + Button)
*   **Feedback**: KY-006 Piezo Buzzer
*   **Motors**: 4x Brushed (Coreless) with Logic-Level MOSFETs

## Wiring
**Safety Notes**
*   **MOSFETs**: Use logic-level MOSFETs (e.g., IRLZ44N). STM32 GPIO is 3.3V; standard FETs may not fully turn on.
*   **Gate Resistors**: Always use a ~10Ω resistor between the MCU pin and the MOSFET Gate to prevent ringing/oscillation.
*   **Decoupling**: Add a 100nF ceramic capacitor directly across the motor terminals to suppress electrical noise.
*   **Battery**: The voltage divider (R1=10k, R2=20k) is required for the 1S LiPo ADC input to prevent exceeding 3.3V on PC0.

**Receiver Pinout**
*   **NRF24L01+** (SPI1): MOSI=PA7, MISO=PA6, SCK=PA5, CE=PA8, CSN=PB12
*   **I2C Bus** (LSM6DS3 & BMP280): SDA=PB7, SCL=PB6
*   **Motors** (TIM5 PWM): FL=PA0, FR=PA1, RL=PA2, RR=PA3
*   **Buzzer**: PB8
*   **Battery ADC**: PC0 (via 10k/20k divider to GND)

**Transmitter Pinout**
*   **NRF24L01+**: Same as Receiver (PA7/PA6/PA5/PA8/PB12)
*   **I2C Bus** (OLED): SDA=PB7, SCL=PB6
*   **Left Stick**: X=PA0, Y=PA1, SW=PA4
*   **Right Stick**: X=PA2, Y=PA3, SW=PB3

## How to use
1.  **Flash**: Upload the firmware via ST-Link or USB (DFU).
2.  **Arm**: Power on the receiver. Hold the **Right Switch** on the transmitter for 2 seconds to arm the motors. The buzzer will confirm arming.
3.  **Disarm**: Hold the **Right Switch** again (throttle must be low) to disarm.
4.  **Failsafe**: If the receiver loses the radio signal for >500ms, it enters a controlled descent (landing) mode.

### PID Tuning Guide
The default PID values in `config.h` are a conservative starting point. Tuning is required for stable flight.

1.  **P (Proportional)**: Increase until the quadcopter oscillates slightly, then reduce slightly.
    *   *Symptom*: Wobbly or loose.
    *   *Fix*: Increase P.
2.  **I (Integral)**: Increase to correct steady-state drift (e.g., tilting forward constantly).
    *   *Symptom*: Slow oscillation or "wandering".
    *   *Fix*: Reduce I.
3.  **D (Derivative)**: Increase to dampen oscillations caused by high P.
    *   *Symptom*: Jittery or high-frequency vibration.
    *   *Fix*: Reduce D.

**Recommended Sequence**: Tune Roll/Pitch P first, then D, then I. Tune Yaw last.

## Files
*   `src/main.cpp` - Main flight controller loop, sensor fusion, and state machine.
*   `src/transmitter.cpp` - Transmitter logic, 3D cube visualization, and OLED rendering.
*   `src/config.h` - Pin definitions, PID constants, and hardware configuration.
*   `src/drivers/flight_controller.cpp` - PID logic, motor mixing, and failsafe handling.
*   `src/drivers/motor_driver.cpp` - HardwareTimer TIM5 setup for PWM generation.
*   `src/drivers/lsm6ds3_driver.cpp` - I2C driver for the IMU.
*   `src/drivers/bmp280_driver.cpp` - I2C driver for the barometer.
*   `src/drivers/nrf24_driver.cpp` - Radio driver with ACK payload handling.
*   `src/drivers/ssd1306_driver.cpp` - Custom framebuffer OLED driver.
*   `src/drivers/thumbstick.cpp` - ADC reading and deadband filtering.
*   `src/drivers/kalman_filter.h` - 2-state Kalman filter for angle estimation.
*   `src/drivers/pid_controller.h` - PID implementation with anti-windup.
*   `src/compat.h` - C++ compatibility shim for ARM newlib.