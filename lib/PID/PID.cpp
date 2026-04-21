#include <PID.h>

PID::PID(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;

    integral = 0;
    lastError = 0;
    lastDerivative = 0;

    outMin = -255;
    outMax = 255;

    intMin = -100;
    intMax = 100;

    lastTime = millis();
}

float PID::compute(float error) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;

    // tránh chia 0
    if (dt <= 0) dt = 1e-3;

    // ===== Integral (anti-windup) =====
    integral += error * dt;
    integral = constrain(integral, intMin, intMax);

    // ===== Derivative (lọc nhiễu) =====
    float derivative = (error - lastError) / dt;
    derivative = 0.7 * derivative + 0.3 * lastDerivative;

    // ===== PID output =====
    float output = Kp * error + Ki * integral + Kd * derivative;
    output = constrain(output, outMin, outMax);

    // ===== Update =====
    lastError = error;
    lastDerivative = derivative;
    lastTime = now;

    return output;
}

void PID::reset() {
    integral = 0;
    lastError = 0;
    lastDerivative = 0;
    lastTime = millis();
}

void PID::setOutputLimit(float minVal, float maxVal) {
    outMin = minVal;
    outMax = maxVal;
}

void PID::setIntegralLimit(float minVal, float maxVal) {
    intMin = minVal;
    intMax = maxVal;
}

void PID::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}