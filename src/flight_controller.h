/**
 * Flight Controller
 *
 * Responsibilities:
 *   - State machine: DISARMED → ARMED → FAILSAFE → LANDING
 *   - Arming/disarming via RIGHT_SW button (hold ARM_HOLD_MS)
 *   - Failsafe: cut motors + controlled landing after packet timeout
 *   - Motor mixing: X-config quadcopter
 *   - Battery voltage measurement + 3-level classification
 *
 * X-config motor layout (top view):
 *
 *        FRONT
 *   FL(CCW) ← → FR(CW)
 *      ↖           ↗
 *      ↙           ↘
 *   RL(CW)  → ← RR(CCW)
 *        BACK
 *
 * Motor mixing (all values in MOTOR_SPEED_MAX units):
 *   FL = Throttle - Pitch + Roll + Yaw   (CCW → +Yaw increases FL)
 *   FR = Throttle - Pitch - Roll - Yaw
 *   RL = Throttle + Pitch + Roll - Yaw
 *   RR = Throttle + Pitch - Roll + Yaw
 *
 *   Pitch+  = nose forward (front motors slow, rear motors fast)
 *   Roll+   = roll right   (left motors fast, right motors slow)
 *   Yaw+    = yaw right    (CCW motors fast, CW motors slow)
 */

#pragma once

#include <Arduino.h>
#include "config_drohne.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "kalman_filter.h"

// ---------------------------------------------------------------------------
// Flight state machine
// ---------------------------------------------------------------------------
enum FlightState {
    STATE_DISARMED,   // motors off, all PIDs reset
    STATE_ARMING,     // button held, counting down
    STATE_ARMED,      // normal flight
    STATE_FAILSAFE,   // packet lost, descending
    STATE_LANDING,    // controlled descent (soft landing)
    STATE_DISARMING,  // button held while armed
};

// Battery level (3 steps + critical)
enum BatLevel {
    BAT_CRIT   = 0,   // < 3.2 V — force-land NOW
    BAT_LOW    = 1,   // < 3.3 V — land soon
    BAT_MEDIUM = 2,   // < 4.0 V — nominal
    BAT_FULL   = 3,   // >= 4.0 V — fully charged
};

// ---------------------------------------------------------------------------
class FlightController {
public:
    explicit FlightController(MotorDriver& motors);

    /** Call once in setup() */
    void init();

    /**
     * Main flight update — call at FLIGHT_LOOP_HZ.
     *
     * @param throttle_in  Stick throttle 0-1000 from receiver
     * @param pitch_in     Stick pitch    -500 to +500 (+ = nose forward)
     * @param roll_in      Stick roll     -500 to +500 (+ = right)
     * @param yaw_in       Stick yaw      -500 to +500 (+ = right CW)
     * @param pitch_angle  Kalman estimated pitch (degrees)
     * @param roll_angle   Kalman estimated roll  (degrees)
     * @param yaw_rate     Bias-corrected yaw rate (degrees/s)
     * @param dt           Loop dt in seconds
     */
    void update(float throttle_in, float pitch_in, float roll_in, float yaw_in,
                float pitch_angle, float roll_angle, float yaw_rate,
                float dt);

    /** Notify the controller that a valid packet was received */
    void packetReceived();

    /**
     * Notify arm/disarm button state.
     * Call every loop with the raw button state (true = pressed).
     */
    void updateArmButton(bool pressed);

    // ---- Battery ----

    /** Read and classify battery voltage from ADC (call at 1 Hz) */
    void updateBattery();

    float     getBatteryMv()    const { return _bat_mv;    }
    BatLevel  getBatteryLevel() const { return _bat_level; }

    // ---- Status queries ----

    FlightState getState()    const { return _state; }
    bool        isArmed()     const { return _state == STATE_ARMED; }
    bool        isFailsafe()  const { return _state == STATE_FAILSAFE ||
                                             _state == STATE_LANDING;  }

    // ---- PID gain access (for live tuning via Serial) ----
    PIDController pid_pitch;
    PIDController pid_roll;
    PIDController pid_yaw;

private:
    MotorDriver& _motors;
    FlightState  _state;

    // Arm/disarm button timer
    uint32_t _arm_btn_since;    // millis() when button press started
    bool     _arm_btn_was;      // last button state

    // Failsafe
    uint32_t _last_packet_ms;
    uint16_t _failsafe_throttle; // current throttle during descent

    // Battery
    float    _bat_mv;
    BatLevel _bat_level;

    // Buzzer feedback
    //void beepArmed();
    //void beepDisarmed();
    //void beepFailsafe();

    // Motor mixing (X-config)
    void mixAndApply(float throttle, float pitch_out, float roll_out, float yaw_out);

    // Constrain helper
    static inline float fclamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};
