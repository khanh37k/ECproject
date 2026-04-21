#include <encoder.h>

Encoder::Encoder(int pin){
    _pin = pin;
}

void Encoder::begin(){
    pinMode(_pin, INPUT);
}

void Encoder::tick(){
    pulse++;
}

float Encoder::getSpeed(){
    unsigned long now = millis();
    unsigned long dt = now - lastTime;

    if (dt == 0) return 0;
    long delta = pulse -lastPulse;
    float speed = (float)delta / dt;

    lastPulse = pulse;
    lastTime = now;

    return speed;

}