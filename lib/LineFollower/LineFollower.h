#pragma once

class LineFollower {
public:
  // setTunings:
  // Gán hệ số điều khiển cho bộ bám line.
  void setTunings(float kp, float ki, float kd);

  // reset:
  // Xóa lỗi tích lũy và lỗi trước đó khi bắt đầu lại một chặng.
  void reset();

  // compute:
  // Nhận vị trí line hiện tại và trả về correction cho 2 bánh.
  int compute(long position);

  // lastCorrection:
  // Trả về correction lần tính gần nhất để debug.
  int lastCorrection() const;

private:
  float _kp = 0.0f;
  float _ki = 0.0f;
  float _kd = 0.0f;
  long _lastError = 0;
  long _integral = 0;
  int _lastCorrection = 0;
};
