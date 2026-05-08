#pragma once

#include <Arduino.h>

class MotorDriver {
public:
  // begin:
  // Cấu hình chân DIR/PWM và khởi tạo 2 kênh LEDC cho driver motor.
  void begin(uint8_t ldir1, uint8_t ldir2, uint8_t lpwm, uint8_t lpwmCh,
             uint8_t rdir1, uint8_t rdir2, uint8_t rpwm, uint8_t rpwmCh,
             uint32_t pwmFreq = 2000, uint8_t pwmResolution = 12);

  // setSpeed:
  // Gửi tốc độ cho 2 bánh, số dương là tiến, âm là lùi.
  void setSpeed(int left, int right);

  // stop:
  // Dừng cả 2 động cơ ngay lập tức.
  void stop();

  // forward:
  // Cho xe chạy thẳng với cùng tốc độ ở 2 bánh.
  void forward(int speed);

  // turnLeftInPlace:
  // Quay trái tại chỗ, phù hợp để tạo state rẽ 90 độ.
  void turnLeftInPlace(int speed);

  // turnRightInPlace:
  // Quay phải tại chỗ, phù hợp để tạo state rẽ 90 độ.
  void turnRightInPlace(int speed);

private:
  void writeMotor(uint8_t dir1, uint8_t dir2, uint8_t pwmCh, int speed);

  uint8_t _ldir1 = 0;
  uint8_t _ldir2 = 0;
  uint8_t _lpwm = 0;
  uint8_t _lpwmCh = 0;
  uint8_t _rdir1 = 0;
  uint8_t _rdir2 = 0;
  uint8_t _rpwm = 0;
  uint8_t _rpwmCh = 0;
  uint8_t _resolution = 12;
};
