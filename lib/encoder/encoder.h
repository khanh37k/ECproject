#pragma once
#include <Arduino.h>

class Encoder {
public:
    Encoder(int pin);

    void begin();
    void tick();              // gọi trong ISR
    float getSpeed();         // xung / ms

private:
    int _pin;

    volatile long pulse = 0;

    long lastPulse = 0;
    unsigned long lastTime = 0;
};