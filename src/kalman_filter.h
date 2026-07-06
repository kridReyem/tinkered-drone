/**
 * 2-State Kalman Filter for IMU Angle Estimation
 *
 * State vector:  x = [angle, gyro_bias]^T
 *
 * Process model (dt = time step in seconds):
 *   angle_new = angle + (gyro_rate - bias) * dt
 *   bias_new  = bias                              (assumed constant)
 *
 * Measurement:
 *   z = atan2(accel_components)  — accelerometer-derived angle
 *
 * This filter fuses the gyroscope (fast, precise, but drifts) with the
 * accelerometer (slow, noisy, but gives absolute reference).
 * It simultaneously estimates and removes the gyro bias.
 *
 * Tuning parameters:
 *   Q_angle   — process noise for angle    (higher = trust gyro more)
 *   Q_bias    — process noise for bias     (higher = bias changes faster)
 *   R_measure — measurement noise          (higher = trust accel less)
 *
 * Reference: "Kalman filter for Arduino" — Kristian Lauszus
 *
 * Usage:
 *   KalmanAngle kf;
 *   kf.init(initial_accel_angle);
 *   float angle = kf.update(gyro_rate_dps, accel_angle_deg, dt_s);
 *   float rate  = kf.getRate();   // bias-corrected rate
 */

#pragma once
#include <math.h>

class KalmanAngle {
public:
    KalmanAngle()
        : _angle(0.0f), _bias(0.0f), _rate(0.0f)
        , Q_angle(0.001f), Q_bias(0.003f), R_measure(0.03f)
    {
        covariance[0][0] = 0.0f; covariance[0][1] = 0.0f;
        covariance[1][0] = 0.0f; covariance[1][1] = 0.0f;
    }

    /**
     * Seed the filter with the accelerometer angle.
     * Call once in setup() after the first valid accel reading.
     */
    void init(float initial_angle_deg) {
        _angle = initial_angle_deg;
        _bias  = 0.0f;
        covariance[0][0] = 0.0f; covariance[0][1] = 0.0f;
        covariance[1][0] = 0.0f; covariance[1][1] = 0.0f;
    }

    /**
     * Run one Kalman step.
     *
     * @param gyro_rate_dps   raw gyroscope rate in degrees/second
     * @param accel_angle_deg accelerometer-derived angle in degrees
     * @param dt              time step in seconds
     * @return estimated angle in degrees
     */
    float update(float gyro_rate_dps, float accel_angle_deg, float dt) {
        // ---------- Predict ----------
        _rate   = gyro_rate_dps - _bias;
        _angle += _rate * dt;

        // Update error covariance
        covariance[0][0] += dt * (dt * covariance[1][1] - covariance[0][1] - covariance[1][0] + Q_angle);
        covariance[0][1] -= dt * covariance[1][1];
        covariance[1][0] -= dt * covariance[1][1];
        covariance[1][1] += Q_bias * dt;

        // ---------- Update (correct with accelerometer) ----------
        float innovation = accel_angle_deg - _angle;
        float S          = covariance[0][0] + R_measure;     // innovation variance

        float K0 = covariance[0][0] / S;   // Kalman gain for angle
        float K1 = covariance[1][0] / S;   // Kalman gain for bias

        _angle += K0 * innovation;
        _bias  += K1 * innovation;

        float P00_tmp = covariance[0][0];
        float P01_tmp = covariance[0][1];
        covariance[0][0] -= K0 * P00_tmp;
        covariance[0][1] -= K0 * P01_tmp;
        covariance[1][0] -= K1 * P00_tmp;
        covariance[1][1] -= K1 * P01_tmp;

        return _angle;
    }

    /** Bias-corrected angular rate from the last update() call */
    float getRate()  const { return _rate;  }

    /** Current estimated angle (degrees) */
    float getAngle() const { return _angle; }

    /** Current estimated gyro bias (degrees/second) */
    float getBias()  const { return _bias;  }

    // ---- Tuning parameters (public for runtime adjustment) ----
    float Q_angle;    ///< process noise: angle    (default 0.001)
    float Q_bias;     ///< process noise: bias     (default 0.003)
    float R_measure;  ///< measurement noise: accel (default 0.03)

private:
    float _angle;
    float _bias;
    float _rate;
    float covariance[2][2];   // 2x2 error covariance matrix
};
