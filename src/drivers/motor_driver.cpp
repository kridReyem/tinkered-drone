#include "motor_driver.h"

void MotorDriver::init(uint32_t freq_hz) {
    // TIM5 on STM32F411 runs at APB1 clock * 2 = 100 MHz (with standard PLL config)
    // At 400 Hz: ARR = 100 MHz / (prescaler * 400 Hz)
    // We choose prescaler=10 → ARR = 100 MHz / (10 * 400) = 25 000 ticks
    // → 25 000 discrete duty-cycle steps (ample for motor control)

    _tim = new HardwareTimer(TIM5);

    // Set frequency first — STM32duino calculates prescaler + ARR automatically
    _tim->setOverflow(freq_hz, HERTZ_FORMAT);

    // Read back the actual ARR so we can scale duty cycle correctly
    _overflow = _tim->getOverflow(TICK_FORMAT);

    // Configure all four channels as PWM output
    _tim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_FL_PIN);
    _tim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_FR_PIN);
    _tim->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_RL_PIN);
    _tim->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, MOTOR_RR_PIN);

    // Start with duty = 0 (all motors off)
    _tim->setCaptureCompare(1, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(2, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(3, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(4, 0, TICK_COMPARE_FORMAT);

    _tim->resume();
}

void MotorDriver::write(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    if (!_tim) return;
    _tim->setCaptureCompare(1, toTicks(fl), TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(2, toTicks(fr), TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(3, toTicks(rl), TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(4, toTicks(rr), TICK_COMPARE_FORMAT);
}

void MotorDriver::stopAll() {
    if (!_tim) return;
    _tim->setCaptureCompare(1, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(2, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(3, 0, TICK_COMPARE_FORMAT);
    _tim->setCaptureCompare(4, 0, TICK_COMPARE_FORMAT);
}
