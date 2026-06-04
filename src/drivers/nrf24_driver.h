#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "../config.h"

// ControllerData und SensorData sind in config.h definiert
// und werden von beiden Seiten (TX + RX) geteilt.

// ---------------------------------------------------------------------------
// NRF24Driver — Transmitter side (Ping-Pong protocol)
//
// Bidirectional protocol:
//   1. TX sendet ControllerData      (stopListening → write)
//   2. RX empfängt, antwortet sofort mit SensorData
//   3. TX wartet max 8ms auf Antwort (startListening → read)
// ---------------------------------------------------------------------------
class NRF24Driver {
public:
    NRF24Driver(uint8_t ce_pin, uint8_t csn_pin);

    bool begin();
    bool isReady() const { return _init; }

    bool sendAndReceive(const ControllerData& ctrl, SensorData* sensor = nullptr);
    bool sendMessage(const ControllerData& ctrl);

    void powerDown();
    void powerUp();
    void printDetails();

private:
    RF24    _radio;
    bool    _init;

    uint8_t checksumCtrl(const ControllerData& d);
};