#pragma once

class Gripper {
public:
  // begin:
  // Khởi tạo phần cứng gắp/thả khi bạn chốt servo hoặc cơ cấu thật.
  void begin();

  // pick:
  // Hook cho thao tác gắp vật. Hiện để trống để bạn tự hoàn thiện.
  void pick();

  // drop:
  // Hook cho thao tác thả vật. Hiện để trống để bạn tự hoàn thiện.
  void drop();
};
