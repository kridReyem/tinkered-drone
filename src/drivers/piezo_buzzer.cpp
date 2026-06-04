/**
 * KY-006 Passive Piezo Buzzer Driver Implementation
 *
 * All melodies ported verbatim from the validated main.cpp implementation.
 * Tempo reference: 190 BPM → 79 ms / tick (quarter note = 2 ticks).
 */

#include "piezo_buzzer.h"

PiezoBuzzer::PiezoBuzzer(uint8_t pin) : _pin(pin) {}

void PiezoBuzzer::init() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void PiezoBuzzer::playTone(unsigned int frequency, unsigned long duration) {
    tone(_pin, frequency, duration);
    delay(duration + 10);
}

void PiezoBuzzer::playBeep() {
    tone(_pin, 1000, 100);
    delay(110);
}

// ---------------------------------------------------------------------------
// Beepbox theme — 190 BPM, single pass (~2.5 s total, blocking)
// Note mapping: G3=196 Hz, E3=165 Hz, A3=220 Hz, D3=147 Hz,
//               D4=294 Hz, E4=330 Hz, G4=392 Hz, A4=440 Hz
// ---------------------------------------------------------------------------
void PiezoBuzzer::playBeepboxMelody() {
    const int t = 79; // ms per tick at 190 BPM

    // Tick 0-1: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);
    // Tick 2-3: pause
    noTone(_pin); delay(2*t);

    // Tick 4-5: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);
    // Tick 6-7: pause
    noTone(_pin); delay(2*t);

    // Tick 8-9: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);

    // Tick 10: E3
    tone(_pin, 165, t); delay(t + 10);

    // Tick 11: E3
    tone(_pin, 165, t); delay(t + 10);

    // Tick 12: A3
    tone(_pin, 220, t); delay(t + 10);

    // Tick 13: pause
    noTone(_pin); delay(t);

    // Tick 14-15: D3
    tone(_pin, 147, 2*t); delay(2*t + 10);

    // Tick 16-17: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);

    // Tick 18-19: D3
    tone(_pin, 147, 2*t); delay(2*t + 10);

    // Tick 20-21: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);

    // Tick 22: E3
    tone(_pin, 165, t); delay(t + 10);

    // Tick 23: A3
    tone(_pin, 220, t); delay(t + 10);

    // Tick 24-25: G3
    tone(_pin, 196, 2*t); delay(2*t + 10);

    // Tick 26-27: A3
    tone(_pin, 220, 2*t); delay(2*t + 10);

    // Tick 28: D4
    tone(_pin, 294, t); delay(t + 10);

    // Tick 29: E4
    tone(_pin, 330, t); delay(t + 10);

    // Tick 30: G4
    tone(_pin, 392, t); delay(t + 10);

    // Tick 31: A4 — final ascent
    tone(_pin, 440, t); delay(t + 10);

    noTone(_pin);
}

void PiezoBuzzer::playStartupMelody() {
    playBeepboxMelody();
    playBeepboxMelody();
    noTone(_pin);
}

void PiezoBuzzer::playClickMelody() {
    tone(_pin, 1047, 100); // C6
    delay(120);
    tone(_pin, 1319, 200); // E6
    delay(250);
    noTone(_pin);
}

void PiezoBuzzer::stop() {
    noTone(_pin);
}
