#pragma once
#include <assert.h>
#include <limits.h>

#include <functional>
#include <string>
#include <vector>

#include "Adafruit_UBX.h"
#include "Adafruit_UBloxDDC.h"
TestSerial Serial;
TwoWire Wire;
using Bytes = std::vector<uint8_t>;
inline Bytes packet(uint8_t cls, uint8_t id, const Bytes& payload = {}) {
  Bytes result = {0xB5,
                  0x62,
                  cls,
                  id,
                  (uint8_t)payload.size(),
                  (uint8_t)(payload.size() / 256)};
  result.insert(result.end(), payload.begin(), payload.end());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < result.size(); ++i) {
    a += result[i];
    b += a;
  }
  result.push_back(a);
  result.push_back(b);
  return result;
}
inline Bytes sentence(const char* body) {
  char text[256];
  size_t count =
      Adafruit_NMEA::buildCommand(text, sizeof(text), body, strlen(body));
  assert(count);
  return Bytes(text, text + count);
}
class FakeStream : public Stream {
 public:
  std::deque<uint8_t> input;
  Bytes output;
  std::vector<size_t> chunks;
  std::function<void()> reply;
  bool endless = false;
  bool readFailure = false;
  int shortWrite = -1;
  size_t readCount = 0;
  void flush() override {}
  int available() override {
    return endless ? 1 : input.size();
  }
  int read() override {
    ++readCount;
    if (readFailure)
      return -1;
    if (endless)
      return 0;
    if (input.empty())
      return -1;
    int byte = input.front();
    input.pop_front();
    return byte;
  }
  size_t write(const uint8_t* data, size_t size) override {
    if (shortWrite == (int)chunks.size())
      return size - 1;
    chunks.push_back(size);
    output.insert(output.end(), data, data + size);
    if (reply)
      reply();
    return size;
  }
  void push(const Bytes& bytes) {
    input.insert(input.end(), bytes.begin(), bytes.end());
  }
};
inline void drain(Adafruit_UBX& ubx, FakeStream& port) {
  while (port.available())
    ubx.checkMessages();
}
