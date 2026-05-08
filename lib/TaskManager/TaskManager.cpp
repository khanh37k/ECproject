#include "TaskManager.h"

void TaskManager::beginTask1() {
  _task1State = RouteToSource;
}

void TaskManager::advanceTask1() {
  if (_task1State == Finished) return;
  _task1State = static_cast<Task1State>(_task1State + 1);
}

TaskManager::Task1State TaskManager::currentTask1State() const {
  return _task1State;
}
