#pragma once
#include <Arduino.h>

class PID {
public:
    // Constructor
    PID(float kp, float ki, float kd);

    // Tính output từ error
    float compute(float error);

    // Reset trạng thái (khi đổi mode / start lại)
    void reset();

    // Set giới hạn
    void setOutputLimit(float minVal, float maxVal);
    void setIntegralLimit(float minVal, float maxVal);

    // Set hệ số
    void setTunings(float kp, float ki, float kd);

private:
    float Kp, Ki, Kd;

    float integral;
    float lastError;
    float lastDerivative;

    float outMin, outMax;
    float intMin, intMax;

    unsigned long lastTime;
};