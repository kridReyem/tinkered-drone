/**
 * KY-006 Passive Piezo Buzzer Driver
 *
 * Hardware: KY-006 passive piezo buzzer module
 * Interface: Digital/PWM pin (uses Arduino tone() function)
 * Pin (Receiver): PB8
 * Pin (Transmitter): PA15
 *
 * Three melodies are provided:
 *   playStartupMelody()  — played once on boot (Beepbox theme x2)
 *   playClickMelody()    — played on left thumbstick click (C6-E6 sting)
 *   playBeepboxMelody()  — raw single-pass Beepbox theme (190 BPM)
 */

#pragma once

#include <Arduino.h>

class PiezoBuzzer {
public:
    /**
     * Constructor
     * @param pin PWM-capable pin connected to buzzer Signal (S) pin
     */
    explicit PiezoBuzzer(uint8_t pin);

    /** Initialize pin direction */
    void init();

    /**
     * Play a single tone (blocking)
     * @param frequency  Hz
     * @param duration   ms
     */
    void playTone(unsigned int frequency, unsigned long duration);

    /** Short 1 kHz beep (100 ms, blocking) */
    void playBeep();

    /**
     * Single pass of the Beepbox theme at 190 BPM (blocking ~2.5 s).
     * Sequence: G3 G3 G3 E3 E3 A3 D3 G3 D3 G3 E3 A3 G3 A3 D4 E4 G4 A4
     */
    void playBeepboxMelody();

    /**
     * Startup melody — plays Beepbox theme twice (blocking ~5 s).
     * Call once in setup().
     */
    void playStartupMelody();

    /**
     * Click sting — C6 → E6 (blocking ~370 ms).
     * Call when left thumbstick click is received.
     */
    void playClickMelody();

    /** Stop any playing tone immediately */
    void stop();

private:
    uint8_t _pin;
};
