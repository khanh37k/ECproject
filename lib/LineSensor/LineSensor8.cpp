#include "LineSensor8.h"

namespace {
uint8_t countBits(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += (value & 0x01);
    value >>= 1;
  }
  return count;
}
}

void LineSensor8::begin(const uint8_t pins[kSensorCount]) {
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    _pins[i] = pins[i];
    pinMode(_pins[i], INPUT);
  }
}

void LineSensor8::setCalibration(const uint16_t black[kSensorCount],
                                 const uint16_t white[kSensorCount]) {
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    _black[i] = black[i];
    _white[i] = white[i];
  }
}

void LineSensor8::readRaw() {
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    _raw[i] = 4095 - analogRead(_pins[i]);
  }
}

void LineSensor8::updateBinaryMask() {
  uint8_t nextMask = 0;
  for (uint8_t i = 0; i < kSensorCount; ++i) {
    const uint16_t threshold = (_black[i] + _white[i]) / 2;
    nextMask <<= 1;
    if (_raw[i] > threshold) {
      nextMask |= 0x01;
    }
  }
  _mask = nextMask;
}

long LineSensor8::getWeightedPosition() const {
  long weightedSum = 0;
  long sum = 0;

  for (uint8_t i = 0; i < kSensorCount; ++i) {
    uint16_t low = _black[i];
    uint16_t high = _white[i];
    uint16_t sample = _raw[i];

    if (sample < low) sample = low;
    if (sample > high) sample = high;

    const long normalized = map(sample, low, high, 1, 3000);
    weightedSum += normalized * (i * 3000L);
    sum += normalized;
  }

  if (sum == 0) return 0;
  return (weightedSum / sum) - 10500L;
}

uint8_t LineSensor8::getMask() const {
  return _mask;
}

bool LineSensor8::isNodeCandidate() const {
  const bool leftBranch = (_mask & 0xE0) != 0;
  const bool rightBranch = (_mask & 0x07) != 0;
  const bool centerSeen = (_mask & 0x18) != 0 || (_mask & 0x3C) == 0x3C;
  return centerSeen && countBits(_mask) >= 4 && (leftBranch || rightBranch);
}

uint16_t LineSensor8::raw(uint8_t index) const {
  if (index >= kSensorCount) return 0;
  return _raw[index];
}
