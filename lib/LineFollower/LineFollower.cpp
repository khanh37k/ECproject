#include "LineFollower.h"

#include <Arduino.h>

void LineFollower::setTunings(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void LineFollower::reset() {
  _lastError = 0;
  _integral = 0;
  _lastCorrection = 0;
}

int LineFollower::compute(long position) {
  const long error = position;
  _integral += error;
  const long derivative = error - _lastError;

  const float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
  _lastCorrection = static_cast<int>(output);
  _lastError = error;
  return _lastCorrection;
}

int LineFollower::lastCorrection() const {
  return _lastCorrection;
}
