#pragma once
#include <deque>
#include <vector>

#include "Arduino.h"
struct TwoWire {
  std::deque<uint8_t> input;
  std::vector<std::vector<uint8_t>> writes;
  size_t capacity = 32;
  int failWrite = -1;
  bool failRead = false;
  int reportedCount = -1;
};
extern TwoWire Wire;
class Adafruit_I2CDevice {
 public:
  Adafruit_I2CDevice(uint8_t, TwoWire* wire) : wire(wire) {}
  bool begin() {
    return true;
  }
  size_t maxBufferSize() {
    return wire->capacity;
  }
  bool write(const uint8_t* data, size_t size) {
    if (size < 2 || size > wire->capacity)
      return false;
    if (wire->failWrite == (int)wire->writes.size())
      return false;
    wire->writes.emplace_back(data, data + size);
    return true;
  }
  bool readRegister(uint8_t address, uint8_t* data, size_t size) {
    if (wire->failRead)
      return false;
    if (address == 0xFD && size == 2) {
      uint16_t count = wire->input.size();
      if (wire->reportedCount >= 0)
        count = wire->reportedCount;
      data[0] = count / 256;
      data[1] = count;
      return true;
    }
    if (address != 0xFF || size > wire->capacity || wire->input.size() < size)
      return false;
    for (size_t i = 0; i < size; ++i) {
      data[i] = wire->input.front();
      wire->input.pop_front();
    }
    return true;
  }

 private:
  TwoWire* wire;
};
