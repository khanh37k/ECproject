#include "Navigator.h"

void Navigator::setRoute(const Action* route, int routeLen) {
  _route = route;
  _routeLen = routeLen;
  _routeIndex = 0;
}

void Navigator::reset() {
  _routeIndex = 0;
}

Navigator::Action Navigator::currentAction() const {
  if (_route == nullptr || _routeIndex >= _routeLen) return Stop;
  return _route[_routeIndex];
}

int Navigator::routeIndex() const {
  return _routeIndex;
}

bool Navigator::isFinished() const {
  return (_route == nullptr) || (_routeIndex >= _routeLen);
}
