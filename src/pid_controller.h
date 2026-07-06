/**
 * PID Controller — header-only
 *
 * Features:
 *   - Derivative on measurement (not error) — prevents derivative kick
 *     when setpoint changes suddenly
 *   - Anti-windup clamp on the integral term
 *   - Output clamped to [out_min, out_max]
 *   - Reset method to clear state on arm/disarm
 *
 * Usage:
 *   PIDController pid;
 *   pid.init(2.5f, 0.05f, 0.5f, -500.f, 500.f);
 *   float output = pid.compute(setpoint, measured, dt_s);
 *
 * Tuning guide (quadcopter angle PID, starting values in config.h):
 *   P — corrects position error.  Too high → oscillation.
 *   I — removes steady-state error.  Too high → slow oscillation / windup.
 *   D — damps oscillation.  Too high → high-frequency noise amplification.
 *   Typical first-flight values: P=2–4, I=0.02–0.1, D=0.1–1.0
 */

#pragma once
#include <math.h>

class PIDController {
public:
    PIDController()
        : kp(0), ki(0), kd(0)
        , _integral(0), _prev_meas(0)
        , _out_min(-500.f), _out_max(500.f)
        , _int_limit(200.f)
        , _first(true)
    {}

    /**
     * Initialise / re-tune the controller.
     * @param p        Proportional gain
     * @param i        Integral gain
     * @param d        Derivative gain
     * @param out_min  Minimum output value
     * @param out_max  Maximum output value
     * @param int_lim  Anti-windup integral clamp (symmetric ±)
     */
    void init(float p, float i, float d,
              float out_min = -500.f, float out_max = 500.f,
              float int_lim = 200.f)
    {
        kp = p; ki = i; kd = d;
        _out_min  = out_min;
        _out_max  = out_max;
        _int_limit = int_lim;
        reset();
    }

    /** Clear integral and derivative state (call on arm / disarm) */
    void reset() {
        _integral  = 0.0f;
        _prev_meas = 0.0f;
        _first     = true;
    }

    /**
     * Compute PID output.
     *
     * Derivative is on measurement (not error) to avoid derivative kick.
     *
     * @param setpoint  Desired value
     * @param measured  Actual value
     * @param dt        Time step in seconds
     * @return PID output, clamped to [out_min, out_max]
     */
    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return 0.0f;

        float error = setpoint - measured;

        // Integral with anti-windup clamp
        _integral += error * dt;
        if      (_integral >  _int_limit) _integral =  _int_limit;
        else if (_integral < -_int_limit) _integral = -_int_limit;

        // Derivative on measurement (avoids setpoint-change kick)
        float derivative = 0.0f;
        if (!_first) {
            derivative = -(measured - _prev_meas) / dt;
        }
        _prev_meas = measured;
        _first     = false;

        float output = kp * error + ki * _integral + kd * derivative;

        // Output clamp
        if      (output >  _out_max) output =  _out_max;
        else if (output <  _out_min) output =  _out_min;

        return output;
    }

    // Gains — public for live tuning via Serial if needed
    float kp, ki, kd;

private:
    float _integral;
    float _prev_meas;
    float _out_min, _out_max;
    float _int_limit;
    bool  _first;
};
