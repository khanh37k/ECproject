#pragma once

#include <stdint.h>

class Navigator {
public:
  enum Action : uint8_t {
    Straight,
    Left90,
    Right90,
    UTurn,
    Stop
  };

  // setRoute:
  // Nạp mảng lệnh điều hướng dạng thẳng/trái/phải/quay đầu/dừng.
  void setRoute(const Action* route, int routeLen);

  // reset:
  // Đặt lại tiến trình route trước khi chạy chặng mới.
  void reset();

  // currentAction:
  // Trả về lệnh hiện tại đang được thực thi.
  Action currentAction() const;

  // routeIndex:
  // Trả về chỉ số lệnh hiện tại trong route.
  int routeIndex() const;

  // isFinished:
  // Báo route đã chạy xong hay chưa.
  bool isFinished() const;

private:
  const Action* _route = nullptr;
  int _routeLen = 0;
  int _routeIndex = 0;
};
