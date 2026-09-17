#pragma once
#include "Arduino.h"
class Stream {
 public:
  virtual ~Stream() {}
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() {
    return -1;
  }
  virtual void flush() = 0;
  virtual size_t write(uint8_t) {
    return 0;
  }
  virtual size_t write(const uint8_t* data, size_t size) = 0;
};
