#pragma once

#include <stdint.h>

class TaskManager {
public:
  enum Task1State : uint8_t {
    Idle,
    RouteToSource,
    ScanSource,
    PickObject,
    RouteToTarget,
    DropObject,
    RouteToCheckpoint,
    Finished
  };

  // beginTask1:
  // Đặt state ban đầu cho nhiệm vụ 1.
  void beginTask1();

  // advanceTask1:
  // Tăng state khi một bước của nhiệm vụ 1 đã hoàn tất.
  void advanceTask1();

  // currentTask1State:
  // Trả về state hiện tại để main.cpp hoặc logger biết xe đang làm gì.
  Task1State currentTask1State() const;

private:
  Task1State _task1State = Idle;
};
