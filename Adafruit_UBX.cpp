/*!
 * @file Adafruit_UBX.cpp
 *
 * @mainpage Arduino library for UBX protocol from u-blox GPS/RTK modules
 *
 * @section intro_sec Introduction
 *
 * This is a library for parsing UBX protocol messages from u-blox GPS/RTK
 * modules. It works with any Stream-based interface including UART and DDC
 * (I2C).
 *
 * Designed specifically to work with u-blox GPS/RTK modules
 * like NEO-M8P, ZED-F9P, etc.
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution
 */

#include "Adafruit_UBX.h"

/*! @brief Create a receiver with 96 bytes of UBX payload storage.
 * @param stream Transport owned by the caller; this parser is its only reader.
 */
Adafruit_UBX::Adafruit_UBX(Stream& stream)
    : _stream(&stream), _payload(_buffer), _capacity(sizeof(_buffer)) {
  onUBXMessage = NULL;
  resetParser();
}

/*! @brief Release the receiver; borrowed transport/storage are not freed. */
Adafruit_UBX::~Adafruit_UBX() {}

/*! @brief Clear receive state without changing receiver hardware configuration.
 * @return False during a callback or command; otherwise true.
 */
bool Adafruit_UBX::begin() {
  if (_checking || _transaction)
    return false;
  resetParser();
  _nmeaActive = false;
  _rtcmHeader = 0;
  _rtcmRemaining = 0;
  if (_nmea)
    _nmea->reset();
  return true;
}

/*! @brief Select storage for larger UBX packets, without allocation.
 * @param buffer Caller-owned storage kept alive until detached; NULL restores
 * the internal 96-byte buffer. Do not share it with NMEA receive storage.
 * @param capacity Payload capacity, or zero when restoring internal storage.
 * @return False for invalid arguments or during receive/command dispatch.
 * Oversized packets are consumed but never dispatched. Partial UBX input
 * resets.
 */
bool Adafruit_UBX::setPayloadBuffer(uint8_t* buffer, uint16_t capacity) {
  if (_checking || _transaction || (buffer && !capacity) ||
      (!buffer && capacity))
    return false;
  _payload = buffer ? buffer : _buffer;
  _capacity = buffer ? capacity : sizeof(_buffer);
  resetParser();
  return true;
}

/*! @brief Attach the shared GNSS receiver to this transport's receive path.
 * @param parser Caller-owned parser and its buffers; NULL detaches it.
 * @param callback Optional callback for complete lines, including invalid ones.
 * Callbacks must consume views immediately and must not read the transport,
 * change buffers, or run blocking transactions. Ignored during dispatch.
 * No NMEA storage is allocated here. RTCM3 frames are skipped, not decoded.
 */
void Adafruit_UBX::setNMEAParser(Adafruit_GNSS* parser,
                                 UBXNMEACallback callback) {
  if (_checking || _transaction)
    return;
  _nmea = parser;
  _nmeaCallback = callback;
  _nmeaActive = false;
  if (_nmea)
    _nmea->reset();
}

/*! @brief Set the UBX callback; payload views expire when reception continues.
 * @param callback Callback, or NULL to disable it. Do not read this transport
 * or issue blocking transactions from callbacks.
 */
void Adafruit_UBX::setMessageCallback(UBXMessageCallback callback) {
  onUBXMessage = callback;
}

/*! @brief Reset only the binary UBX frame under construction. */
void Adafruit_UBX::resetParser() {
  _parserState = WAIT_SYNC_1;
  _payloadCounter = _payloadLength = 0;
  _checksumA = _checksumB = 0;
  _checksumMatches = false;
  _replyEligible = false;
}

/*! @brief Add one header/payload byte to the UBX Fletcher checksum.
 * @param byte Next checksum-covered byte.
 */
void Adafruit_UBX::addChecksum(uint8_t byte) {
  _checksumA += byte;
  _checksumB += _checksumA;
}

/*! @brief Consume bounded input while dispatching UBX and optional NMEA.
 * @param maxBytes Maximum bytes consumed on this call, including skipped data.
 * @return True if at least one valid, in-capacity UBX packet was dispatched.
 * Keep calling from loop(); false does not imply that the transport is empty.
 * Frames interrupted by a one-second receive gap are abandoned. Transport I/O
 * and user callbacks must themselves return promptly for deadlines to hold.
 */
bool Adafruit_UBX::checkMessages(uint16_t maxBytes) {
  if (_checking)
    return false;
  _checking = true;
  bool received = false;
  while (maxBytes-- && _stream->available() > 0) {
    int byte = _stream->read();
    if (byte < 0)
      break;
    if (processByte((uint8_t)byte, millis()))
      received = true;
  }
  _checking = false;
  return received;
}

/*! @brief Route a byte exclusively to the active frame type.
 * @param byte Received byte.
 * @param now Receive time in milliseconds.
 * @return True for a complete validated UBX packet within payload capacity.
 */
bool Adafruit_UBX::processByte(uint8_t byte, uint32_t now) {
  if ((uint32_t)(now - _byteTime) >= RECEIVE_GAP_MS) {
    resetParser();
    _nmeaActive = false;
    _rtcmHeader = 0;
    _rtcmRemaining = 0;
  }
  _byteTime = now;

  // RTCM3 has a 10-bit payload length followed by a three-byte CRC. Consume
  // the entire frame, including embedded '$' and UBX sync bytes. This driver
  // does not validate or publish RTCM; it only keeps it out of the other
  // parsers.
  if (_rtcmHeader) {
    if (_rtcmHeader == 1) {
      if (byte > 3) {
        _rtcmHeader = 0;
      } else {
        _rtcmRemaining = (uint16_t)byte * 256;
        _rtcmHeader = 2;
        return false;
      }
    } else if (_rtcmHeader == 2) {
      _rtcmRemaining += (uint16_t)byte + 3;
      _rtcmHeader = 3;
      return false;
    } else {
      if (--_rtcmRemaining == 0)
        _rtcmHeader = 0;
      return false;
    }
  }

  if (_parserState == WAIT_SYNC_1) {
    if (byte == UBX_SYNC_CHAR_1 || byte == RTCM3_PREAMBLE) {
      _nmeaActive = false;
      if (byte == RTCM3_PREAMBLE)
        _rtcmHeader = 1;
      else {
        _parserState = WAIT_SYNC_2;
        _replyEligible = _waitingAck || _portReply != NULL;
      }
      return false;
    }
    if (byte == '$' || byte == '!')
      _nmeaActive = true;
    if (_nmeaActive) {
      nmea_frame_status_t status = NMEA_FRAME_INCOMPLETE;
      if (_nmea)
        status = _nmea->feed(byte, now);
      if (byte == '\n') {
        _nmeaActive = false;
        if (_nmea && _nmeaCallback && status != NMEA_FRAME_INCOMPLETE &&
            status != NMEA_FRAME_OVERFLOW) {
          nmea_sentence_t sentence = _nmea->lastSentence();
          if (sentence.status != NMEA_FRAME_INCOMPLETE)
            _nmeaCallback(sentence);
        }
      }
    }
    return false;
  }

  switch (_parserState) {
    case WAIT_SYNC_1:
      break;
    case WAIT_SYNC_2:
      if (byte == UBX_SYNC_CHAR_2) {
        _checksumA = _checksumB = 0;
        _parserState = GET_CLASS;
      } else if (byte == UBX_SYNC_CHAR_1) {
        _replyEligible = _waitingAck || _portReply != NULL;
      } else {
        resetParser();
        return processByte(byte, now);
      }
      break;
    case GET_CLASS:
      _msgClass = byte;
      addChecksum(byte);
      _parserState = GET_ID;
      break;
    case GET_ID:
      _msgId = byte;
      addChecksum(byte);
      _parserState = GET_LENGTH_1;
      break;
    case GET_LENGTH_1:
      _payloadLength = byte;
      addChecksum(byte);
      _parserState = GET_LENGTH_2;
      break;
    case GET_LENGTH_2:
      _payloadLength += (uint16_t)byte * 256;
      addChecksum(byte);
      _payloadCounter = 0;
      _parserState = _payloadLength ? GET_PAYLOAD : GET_CHECKSUM_A;
      break;
    case GET_PAYLOAD:
      if (_payloadLength <= _capacity)
        _payload[_payloadCounter] = byte;
      addChecksum(byte);
      if (++_payloadCounter == _payloadLength)
        _parserState = GET_CHECKSUM_A;
      break;
    case GET_CHECKSUM_A:
      _checksumMatches = byte == _checksumA;
      _parserState = GET_CHECKSUM_B;
      break;
    case GET_CHECKSUM_B: {
      bool valid =
          _checksumMatches && byte == _checksumB && _payloadLength <= _capacity;
      if (valid) {
        // Latch before invoking callbacks or consuming any later packet.
        if (_replyEligible && _waitingAck && _ackStatus == UBX_SEND_TIMEOUT &&
            _msgClass == UBX_CLASS_ACK && _payloadLength == 2 &&
            _payload[0] == _ackClass && _payload[1] == _ackId) {
          if (_msgId == UBX_ACK_ACK)
            _ackStatus = UBX_SEND_SUCCESS;
          if (_msgId == UBX_ACK_NAK)
            _ackStatus = UBX_SEND_NAK;
        }
        if (_replyEligible && _portReply && !_portReceived &&
            _msgClass == UBX_CLASS_CFG && _msgId == UBX_CFG_PRT &&
            _payloadLength == sizeof(UBX_CFG_PRT_t) &&
            _payload[0] == _portReply->fields.portID) {
          memcpy(_portReply->raw, _payload, sizeof(UBX_CFG_PRT_t));
          _portReceived = true;
        }
        if (verbose_debug) {
          Serial.print(F("UBX RX: "));
          printHex(_msgClass);
          Serial.print(' ');
          printHex(_msgId);
          if (verbose_debug > 1)
            printHexBuffer(F(" PL"), _payload, _payloadLength);
          Serial.println();
        }
        if (onUBXMessage)
          onUBXMessage(_msgClass, _msgId, _payloadLength, _payload);
      }
      resetParser();
      return valid;
    }
  }
  return false;
}

/*! @brief Drain queued input before arming a new command transaction.
 * @param timeout_ms Elapsed-time bound for draining old packets.
 * @return True once the input queue is empty; false on time/byte exhaustion.
 * No more than 4096 bytes are drained. Navigation callbacks still run. Partial

 * * frames retain ownership of their bytes but cannot acknowledge the new
 * command.
 * UBX ACK has no sequence number, so a
 * delayed ACK for an
 * identical earlier command cannot be distinguished.
 */
bool Adafruit_UBX::prepareTransaction(uint16_t timeout_ms) {
  _replyEligible = false;
  uint32_t start = millis();
  uint16_t remaining = 4096;
  while (_stream->available() > 0) {
    if (!remaining || (uint32_t)(millis() - start) >= timeout_ms)
      return false;
    checkMessages(1);
    --remaining;
  }
  return true;
}

/*! @brief Send a UBX command and retain the first matching ACK/NAK.
 * @param msgClass Command class.
 * @param msgId Command ID.
 * @param payload Payload, or NULL for a zero-length poll.
 * @param length Payload length in bytes.
 * @param timeout_ms Overall deadline for queue draining, sending and receiving.
 * @return Send failure, timeout, ACK success or NAK rejection.
 * NMEA and UBX callbacks continue during the wait. Reentrant calls fail.
 */
UBXSendStatus Adafruit_UBX::sendMessageWithAck(uint8_t msgClass, uint8_t msgId,
                                               const uint8_t* payload,
                                               uint16_t length,
                                               uint16_t timeout_ms) {
  if (_checking || _transaction || (length && !payload))
    return UBX_SEND_FAIL;
  _transaction = true;
  uint32_t start = millis();
  if (!prepareTransaction(timeout_ms)) {
    _transaction = false;
    return UBX_SEND_TIMEOUT;
  }
  _ackClass = msgClass;
  _ackId = msgId;
  _ackStatus = UBX_SEND_TIMEOUT;
  _waitingAck = true;
  if (!sendMessage(msgClass, msgId, payload, length)) {
    _waitingAck = false;
    _transaction = false;
    return UBX_SEND_FAIL;
  }
  while (_ackStatus == UBX_SEND_TIMEOUT &&
         (uint32_t)(millis() - start) < timeout_ms) {
    checkMessages(1);
    if (_ackStatus == UBX_SEND_TIMEOUT && _stream->available() <= 0)
      delay(1);
  }
  _waitingAck = false;
  _transaction = false;
  return _ackStatus;
}

/*! @brief Change protocol masks without replacing baud/address/framing
 * settings.
 * @param portID Port to query and update.
 * @param checkAck Whether to wait for the configuration ACK.
 * @param timeout_ms Deadline per query or ACK phase, in milliseconds.
 * @return Failure/timeout, or the configuration ACK when requested.
 * Polls CFG-PRT first, then changes only its input/output protocol masks. It
 * does not enable individual navigation messages or save settings to flash.
 */
UBXSendStatus Adafruit_UBX::setUBXOnly(UBXPortId portID, bool checkAck,
                                       uint16_t timeout_ms) {
  if (_checking || _transaction)
    return UBX_SEND_FAIL;
  UBX_CFG_PRT_t port = {};
  port.fields.portID = portID;
  _transaction = true;
  uint32_t start = millis();
  if (!prepareTransaction(timeout_ms)) {
    _transaction = false;
    return UBX_SEND_TIMEOUT;
  }
  _portReply = &port;
  _portReceived = false;
  if (!sendMessage(UBX_CLASS_CFG, UBX_CFG_PRT, &port.fields.portID, 1)) {
    _portReply = NULL;
    _transaction = false;
    return UBX_SEND_FAIL;
  }
  while (!_portReceived && (uint32_t)(millis() - start) < timeout_ms) {
    checkMessages(1);
    if (!_portReceived && _stream->available() <= 0)
      delay(1);
  }
  _portReply = NULL;
  _transaction = false;
  if (!_portReceived)
    return UBX_SEND_TIMEOUT;
  port.fields.inProtoMask = UBX_PROTOCOL_UBX;
  port.fields.outProtoMask = UBX_PROTOCOL_UBX;
  if (checkAck)
    return sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_PRT, port.raw,
                              sizeof(port), timeout_ms);
  return sendMessage(UBX_CLASS_CFG, UBX_CFG_PRT, port.raw, sizeof(port))
             ? UBX_SEND_SUCCESS
             : UBX_SEND_FAIL;
}

/*! @brief Send a UBX packet with constant, 32-byte stack storage.
 * @param msgClass Message class.
 * @param msgId Message ID.
 * @param payload Payload, or NULL only when length is zero.
 * @param length Payload length; all uint16_t lengths are supported.
 * @return True only if every chunk was accepted. A short write fails
 * immediately. Chunks contain at least two bytes so a DDC device never
 * interprets a final lone byte as a register address. No heap allocation or
 * payload-sized VLA.
 */
bool Adafruit_UBX::sendMessage(uint8_t msgClass, uint8_t msgId,
                               const uint8_t* payload, uint16_t length) {
  if (_checking || (length && !payload))
    return false;
  uint8_t chunk[TX_CHUNK_SIZE];
  uint8_t header[6] = {UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2,        msgClass, msgId,
                       (uint8_t)length, (uint8_t)(length / 256)};
  uint8_t a = 0, b = 0;
  for (uint8_t i = 2; i < sizeof(header); ++i) {
    a += header[i];
    b += a;
  }
  for (uint16_t i = 0; i < length; ++i) {
    a += payload[i];
    b += a;
  }
  uint32_t total = (uint32_t)length + 8;
  uint32_t offset = 0;
  while (offset < total) {
    uint8_t count = TX_CHUNK_SIZE;
    if (total - offset < count)
      count = total - offset;
    if (total - offset - count == 1)
      --count;
    for (uint8_t i = 0; i < count; ++i) {
      uint32_t index = offset + i;
      if (index < 6)
        chunk[i] = header[index];
      else if (index < (uint32_t)length + 6)
        chunk[i] = payload[index - 6];
      else if (index == (uint32_t)length + 6)
        chunk[i] = a;
      else
        chunk[i] = b;
    }
    if (verbose_debug > 1)
      printHexBuffer(F("UBX TX "), chunk, count);
    if (_stream->write(chunk, count) != count)
      return false;
    offset += count;
  }
  return true;
}

/*! @brief Print one debug byte as two hexadecimal digits.
 * @param val Byte to print.
 */
void Adafruit_UBX::printHex(uint8_t val) {
  if (val < 0x10)
    Serial.print('0');
  Serial.print(val, HEX);
}

/*! @brief Print a labeled debug byte buffer.
 * @param label Flash-resident label.
 * @param buf Byte buffer.
 * @param len Number of bytes.
 */
void Adafruit_UBX::printHexBuffer(const __FlashStringHelper* label,
                                  const uint8_t* buf, uint16_t len) {
  Serial.print(label);
  Serial.print('[');
  for (uint16_t i = 0; i < len; ++i) {
    printHex(buf[i]);
    Serial.print(' ');
  }
  Serial.print(F("] "));
}
