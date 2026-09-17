// Minimal deterministic host shim; no emulation of receiver hardware.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <algorithm>
using std::min;
class __FlashStringHelper;
#define F(text) reinterpret_cast<const __FlashStringHelper*>(text)
#define HEX 16
inline uint32_t& testClock() {
  static uint32_t clock = 0;
  return clock;
}
inline unsigned long millis() {
  return testClock()++;
}
inline void delay(unsigned long ms) {
  testClock() += ms;
}
class TestSerial {
 public:
  template <typename T>
  void print(T) {}
  template <typename T>
  void print(T, int) {}
  template <typename T>
  void println(T) {}
  void println() {}
};
extern TestSerial Serial;
