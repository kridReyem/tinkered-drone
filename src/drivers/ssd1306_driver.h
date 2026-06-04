#pragma once

#include <Arduino.h>
#include <Wire.h>

class SSD1306Driver {
public:
    SSD1306Driver(TwoWire* wire = &Wire) : _wire(wire), _addr(0x3C) {}

    void init(uint8_t i2c_addr = 0x3C);
    void clearBuffer();
    void sendBuffer();
    void drawPixel(int16_t x, int16_t y, bool set = true);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h);
    void drawVLine(int16_t x, int16_t y, int16_t h);
    void drawHLine(int16_t x, int16_t y, int16_t w);
    void drawCircle(int16_t cx, int16_t cy, int16_t r);
    void fillCircle(int16_t cx, int16_t cy, int16_t r);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, bool set = true);
    void printAt(uint8_t px, uint8_t py, const char* str);
    void printAt(uint8_t px, uint8_t py, long value);
    void printAt(uint8_t px, uint8_t py, int value) { printAt(px, py, (long)value); }
    void printAt(uint8_t px, uint8_t py, unsigned long value);

private:
    TwoWire* _wire;
    uint8_t  _addr;
    uint8_t  _fb[1024];

    void sendCmd(uint8_t cmd);
    void sendCmd2(uint8_t cmd, uint8_t arg);

    static const uint8_t FONT[][5];
};