/**
 * Motor Driver — 4x Brushed Motors via HardwareTimer TIM5
 *
 * Uses STM32F411 TIM5 with 4 synchronized PWM channels:
 *   CH1 = PA0 → Front-Left  (FL, CCW)
 *   CH2 = PA1 → Front-Right (FR, CW)
 *   CH3 = PA2 → Rear-Left   (RL, CW)
 *   CH4 = PA3 → Rear-Right  (RR, CCW)
 *
 * HARDWARE NOTES:
 *   - STM32F411 GPIO drives 3.3 V — use logic-level MOSFETs only
 *     (e.g. IRLZ44N Vgs_th=1.5 V, 2N7002 Vgs_th=1-2.5 V)
 *   - Add 10 Ω gate resistor per MOSFET to suppress ringing
 *   - Add 100 nF ceramic cap across each motor terminal pair
 *   - 1S LiPo connects directly to MOSFET drain; source to motor –
 *
 * Speed scale: 0 (off) to MOTOR_SPEED_MAX (full throttle)
 */

#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>
#include "../config.h"

class MotorDriver {
public:
    MotorDriver() : _tim(nullptr), _overflow(0) {}

    /**
     * Initialise TIM5 PWM.  Call once in setup().
     * @param freq_hz  PWM carrier frequency in Hz (default MOTOR_PWM_FREQ_HZ)
     */
    void init(uint32_t freq_hz = MOTOR_PWM_FREQ_HZ);

    /**
     * Set individual motor speed.
     * @param fl  Front-Left  speed  0-MOTOR_SPEED_MAX
     * @param fr  Front-Right speed  0-MOTOR_SPEED_MAX
     * @param rl  Rear-Left   speed  0-MOTOR_SPEED_MAX
     * @param rr  Rear-Right  speed  0-MOTOR_SPEED_MAX
     */
    void write(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr);

    /** Stop all motors immediately (duty = 0) */
    void stopAll();

    /** True after init() succeeds */
    bool isReady() const { return _tim != nullptr; }

private:
    HardwareTimer* _tim;
    uint32_t       _overflow;   // timer auto-reload value (full-scale ticks)

    inline uint16_t clamp(uint16_t v) const {
        return v > MOTOR_SPEED_MAX ? MOTOR_SPEED_MAX : v;
    }

    /** Convert 0-MOTOR_SPEED_MAX to timer compare ticks */
    inline uint32_t toTicks(uint16_t speed) const {
        return ((uint32_t)clamp(speed) * _overflow) / MOTOR_SPEED_MAX;
    }
};
