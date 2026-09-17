#pragma once
#include "Adafruit_I2CDevice.h"
class Adafruit_BusIO_Register {
 public:
  Adafruit_BusIO_Register(Adafruit_I2CDevice* device, uint8_t address, uint8_t)
      : device(device), address(address) {}
  bool read(uint8_t* data, size_t size) {
    return device->readRegister(address, data, size);
  }

 private:
  Adafruit_I2CDevice* device;
  uint8_t address;
};
