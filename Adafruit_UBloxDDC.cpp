/*!
 * @file Adafruit_UBloxDDC.cpp
 *
 * @section ddc_intro_sec Introduction
 *
 * This is a library for the u-blox GPS/RTK modules using I2C interface (DDC)
 *
 * Designed specifically to work with u-blox GPS/RTK modules
 * like NEO-M8P, ZED-F9P, etc.
 *
 * @section ddc_dependencies Dependencies
 *
 * This library depends on:
 * <a href="https://github.com/adafruit/Adafruit_BusIO">Adafruit_BusIO</a>
 *
 * @section ddc_author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section ddc_license License
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBloxDDC.h"

#include <limits.h>

/*!
 *  @brief  Constructor
 *  @param  address
 *          i2c address (default 0x42)
 *  @param  wire
 *          TwoWire instance (default &Wire)
 */
Adafruit_UBloxDDC::Adafruit_UBloxDDC(uint8_t address, TwoWire* wire) {
  _i2cDevice = new Adafruit_I2CDevice(address, wire);
}

/*!
 *  @brief  Destructor - frees allocated resources
 */
Adafruit_UBloxDDC::~Adafruit_UBloxDDC() {
  if (_i2cDevice) {
    delete _i2cDevice;
  }
}

/*!
 *  @brief  Initializes the GPS module and I2C interface
 *  @return True if GPS module responds, false on any failure
 */
bool Adafruit_UBloxDDC::begin() {
  _hasPeeked = false;
  _available = 0;
  return _i2cDevice->begin();
}

/*! @brief Complete pending writes (no-op: DDC writes are synchronous).
 * Does
 * not discard receiver data or a cached peeked byte.
 */
void Adafruit_UBloxDDC::flush() {}

/*!
 *  @brief  Gets the number of bytes available for reading
 *  @return Known unread count including a peeked byte, saturated at INT_MAX.
 *
 * @details Refreshes the receiver count after consuming the previous count.
 *
 * On a bus error, reports a cached peeked byte or zero.
 */
int Adafruit_UBloxDDC::available() {
  // Only this Stream consumes DDC output. Reuse the unread count instead of
  // spending an I2C transaction on the count before every single data byte.
  if (_available) {
    uint32_t count = (uint32_t)_available + (_hasPeeked ? 1 : 0);
    if (count > INT_MAX)
      return INT_MAX;
    return (int)count;
  }
  uint8_t buffer[2];

  // Create a register for reading bytes available
  Adafruit_BusIO_Register bytesAvailableReg =
      Adafruit_BusIO_Register(_i2cDevice, REG_BYTES_AVAILABLE_MSB, 2);

  if (!bytesAvailableReg.read(buffer, 2)) {
    _available = 0;
    return _hasPeeked ? 1 : 0;
  }

  _available = (uint16_t)buffer[0] * 256 + buffer[1];
  uint32_t count = (uint32_t)_available + (_hasPeeked ? 1 : 0);
  if (count > INT_MAX)
    return INT_MAX;
  return (int)count;
}

/*!
 *  @brief  Reads a single byte from the data stream
 *  @return -1 if no data available or error, otherwise the byte read (0-255)
 */
int Adafruit_UBloxDDC::read() {
  // If we have a peeked byte, return it
  if (_hasPeeked) {
    _hasPeeked = false;
    return _lastByte;
  }

  // Query first so an empty DDC stream's 0xFF idle byte is never peek-cached.
  // A 0xFF byte inside an available UBX payload is still ordinary data.
  if (!_available && !available())
    return -1;

  uint8_t value;

  // Create a register for the data stream
  Adafruit_BusIO_Register dataStreamReg =
      Adafruit_BusIO_Register(_i2cDevice, REG_DATA_STREAM, 1);

  if (!dataStreamReg.read(&value, 1)) {
    _available = 0;
    return -1;
  }
  --_available;
  return value;
}

/*!
 *  @brief  Peek at the next available byte without removing it from the stream
 *  @return -1 if no data available or error, otherwise the byte (0-255)
 */
int Adafruit_UBloxDDC::peek() {
  // If we've already peeked, return the last byte
  if (_hasPeeked) {
    return _lastByte;
  }

  // Otherwise, read a byte and store it
  _lastByte = read();
  if (_lastByte != -1) {
    _hasPeeked = true;
  }

  return _lastByte;
}

/*!
 *  @brief  Write a single byte (required by Stream but not suitable for I2C)
 *  @param  val  Byte to write
 *  @return Always returns 0 as single-byte writes aren't supported on I2C
 */
size_t Adafruit_UBloxDDC::write(uint8_t val) {
  (void)val;
  // Single-byte writes aren't suitable for I2C/DDC
  // This shouldn't be called if properly using the multi-byte version
  return 0;
}

/*!
 *  @brief  Write multiple bytes at once (required for I2C/DDC)
 *  @param  buffer  Pointer to data buffer
 *  @param  size    Number of bytes to write
 *  @return Number of bytes written
 */
size_t Adafruit_UBloxDDC::write(const uint8_t* buffer, size_t size) {
  // For I2C/DDC, we need at least 2 bytes for a write
  if (!buffer || size < 2 || _i2cDevice->maxBufferSize() < 2) {
    // Single-byte writes aren't supported
    return 0;
  }

  // DDC writes need at least two bytes (M8 protocol section 11.5.2). Split
  // at the actual Wire capacity without leaving a one-byte final transaction.
  size_t written = 0;
  while (written < size) {
    size_t count = size - written;
    if (count > _i2cDevice->maxBufferSize())
      count = _i2cDevice->maxBufferSize();
    if (size - written - count == 1)
      --count;
    if (count < 2 || !_i2cDevice->write(buffer + written, count))
      break;
    written += count;
  }
  return written;
}

/*!
 *  @brief  Read multiple bytes from the data stream
 *  @param  buffer  Pointer to buffer to store data
 *  @param  length  Maximum number of bytes to read
 *  @return Number of bytes actually read, which may be less than requested
 */
uint16_t Adafruit_UBloxDDC::readBytes(uint8_t* buffer, uint16_t length) {
  if (buffer == nullptr || length == 0) {
    return 0;
  }

  uint16_t bytesRead = 0;
  uint16_t bytesAvailable = available();

  // Don't try to read more bytes than are available
  length = min(length, bytesAvailable);

  // Handle any peeked byte first
  if (_hasPeeked && length > 0) {
    buffer[0] = _lastByte;
    _hasPeeked = false;
    bytesRead = 1;
  }

  if (bytesRead >= length) {
    return bytesRead;
  }

  // Create a register for the data stream
  Adafruit_BusIO_Register dataStreamReg =
      Adafruit_BusIO_Register(_i2cDevice, REG_DATA_STREAM, 1);

  while (bytesRead < length) {
    // Calculate chunk size (I2C has a limit on bytes per transfer)
    uint16_t chunkSize = min((uint16_t)(length - bytesRead),
                             (uint16_t)_i2cDevice->maxBufferSize());

    if (!dataStreamReg.read(&buffer[bytesRead], chunkSize)) {
      _available = 0;
      break;
    }

    bytesRead += chunkSize;
    _available -= chunkSize;
  }

  return bytesRead;
}

/*!
 *  @brief  Read currently available stream bytes; not protocol framing
 *
 * @param  buffer     Pointer to buffer to store message data
 *  @param  maxLength  Maximum length of buffer
 *  @return Number of bytes read; may contain partial or multiple messages.
 */
uint16_t Adafruit_UBloxDDC::readMessage(uint8_t* buffer, uint16_t maxLength) {
  uint16_t bytesAvailable = available();

  if (bytesAvailable == 0) {
    return 0;
  }

  // Limit to buffer size
  uint16_t bytesToRead = min(bytesAvailable, maxLength);
  return readBytes(buffer, bytesToRead);
}

/*!
 *  @brief  Read available stream bytes into the internal buffer
 *  @param
 * messageLength  Pointer to variable to store message length
 *  @return Borrowed buffer of up to 128 bytes, or NULL for a NULL length
 * pointer.
 *  This does not find UBX/NMEA boundaries; use Adafruit_UBX for
 * framed messages.
 */
uint8_t* Adafruit_UBloxDDC::readMessage(uint16_t* messageLength) {
  if (!messageLength)
    return NULL;
  *messageLength = readMessage(_buffer, MAX_BUFFER_SIZE);
  return _buffer;
}
