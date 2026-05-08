#include "MotorDriver.h"

void MotorDriver::begin(uint8_t ldir1, uint8_t ldir2, uint8_t lpwm, uint8_t lpwmCh,
                        uint8_t rdir1, uint8_t rdir2, uint8_t rpwm, uint8_t rpwmCh,
                        uint32_t pwmFreq, uint8_t pwmResolution) {
  _ldir1 = ldir1;
  _ldir2 = ldir2;
  _lpwm = lpwm;
  _lpwmCh = lpwmCh;
  _rdir1 = rdir1;
  _rdir2 = rdir2;
  _rpwm = rpwm;
  _rpwmCh = rpwmCh;
  _resolution = pwmResolution;

  pinMode(_ldir1, OUTPUT);
  pinMode(_ldir2, OUTPUT);
  pinMode(_rdir1, OUTPUT);
  pinMode(_rdir2, OUTPUT);

  ledcSetup(_lpwmCh, pwmFreq, _resolution);
  ledcSetup(_rpwmCh, pwmFreq, _resolution);
  ledcAttachPin(_lpwm, _lpwmCh);
  ledcAttachPin(_rpwm, _rpwmCh);

  stop();
}

void MotorDriver::setSpeed(int left, int right) {
  writeMotor(_ldir1, _ldir2, _lpwmCh, left);
  writeMotor(_rdir1, _rdir2, _rpwmCh, right);
}

void MotorDriver::stop() {
  setSpeed(0, 0);
}

void MotorDriver::forward(int speed) {
  setSpeed(speed, speed);
}

void MotorDriver::turnLeftInPlace(int speed) {
  setSpeed(-speed, speed);
}

void MotorDriver::turnRightInPlace(int speed) {
  setSpeed(speed, -speed);
}

void MotorDriver::writeMotor(uint8_t dir1, uint8_t dir2, uint8_t pwmCh, int speed) {
  const int maxPwm = (1 << _resolution) - 1;
  speed = constrain(speed, -maxPwm, maxPwm);

  if (speed > 0) {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    ledcWrite(pwmCh, speed);
  } else if (speed < 0) {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    ledcWrite(pwmCh, -speed);
  } else {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, LOW);
    ledcWrite(pwmCh, 0);
  }
}
