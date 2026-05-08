#pragma once

#include <Arduino.h>

class LineSensor8 {
public:
  static const uint8_t kSensorCount = 8;

  // begin:
  // Cấu hình 8 chân cảm biến line và nạp pin map cho thư viện.
  void begin(const uint8_t pins[kSensorCount]);

  // setCalibration:
  // Gán ngưỡng min/max cho từng mắt sau khi đã calibrate.
  void setCalibration(const uint16_t black[kSensorCount],
                      const uint16_t white[kSensorCount]);

  // readRaw:
  // Đọc trực tiếp giá trị ADC hiện tại của 8 mắt cảm biến.
  void readRaw();

  // updateBinaryMask:
  // Tạo mask 8 bit từ dữ liệu raw dựa trên ngưỡng trung bình đen/trắng.
  void updateBinaryMask();

  // getWeightedPosition:
  // Tính vị trí line theo trọng số để dùng cho PD/PID.
  long getWeightedPosition() const;

  // getMask:
  // Trả về pattern nhị phân 8 bit của line hiện tại.
  uint8_t getMask() const;

  // isNodeCandidate:
  // Nhận diện nhanh giao điểm/ngã rẽ dựa trên mask hiện tại.
  bool isNodeCandidate() const;

  // raw:
  // Cho phép truy cập giá trị raw của từng mắt để debug/log.
  uint16_t raw(uint8_t index) const;

private:
  uint8_t _pins[kSensorCount] = {0};
  uint16_t _raw[kSensorCount] = {0};
  uint16_t _black[kSensorCount] = {0};
  uint16_t _white[kSensorCount] = {0};
  uint8_t _mask = 0;
};
