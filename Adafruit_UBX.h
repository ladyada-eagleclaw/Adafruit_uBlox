/*!
 * @file Adafruit_UBX.h
 *
 * Arduino library for parsing UBX protocol from u-blox GPS/RTK modules.
 *
 * This library can use any Stream object as input (UART, DDC, or other).
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#ifndef ADAFRUIT_UBX_H
#define ADAFRUIT_UBX_H

#include <Adafruit_GNSS.h>
#include <Arduino.h>
#include <Stream.h>

#include "Adafruit_uBlox_typedef.h"

// UBX protocol constants
#define UBX_SYNC_CHAR_1 0xB5 ///< First UBX protocol sync char (ï¿½)
#define UBX_SYNC_CHAR_2 0x62 ///< Second UBX protocol sync char (b)
// UBX ACK Message IDs
#define UBX_ACK_NAK 0x00 ///< Message Not Acknowledged
#define UBX_ACK_ACK 0x01 ///< Message Acknowledged

/*!
 *  @brief  Callback function type for UBX messages - defined at global scope so
 * other classes can use it
 *  @param  msgClass Message class
 *  @param  msgId Message ID
 *  @param  payloadLen Length of payload data
 *  @param  payload Pointer to payload data
 */
typedef void (*UBXMessageCallback)(uint8_t msgClass, uint8_t msgId,
                                   uint16_t payloadLen, uint8_t* payload);

/** Callback for a complete NMEA line, including checksum/format failures.
 *
 * @param sentence Borrowed view; consume before the next line is received.
 */
typedef void (*UBXNMEACallback)(const nmea_sentence_t& sentence);

/*!
 * @brief Class for parsing UBX protocol messages from u-blox GPS/RTK modules
 */
class Adafruit_UBX {
 public:
  Adafruit_UBX(Stream& stream);
  ~Adafruit_UBX();
  uint8_t verbose_debug = 0; ///<  0=off, 1=basic, 2=verbose
  // Basic methods
  bool begin();
  bool checkMessages(uint16_t maxBytes = 256);
  bool setPayloadBuffer(uint8_t* buffer, uint16_t capacity);
  void setNMEAParser(Adafruit_GNSS* parser, UBXNMEACallback callback = NULL);
  bool sendMessage(uint8_t msgClass, uint8_t msgId, const uint8_t* payload,
                   uint16_t length); // Send a UBX message
  UBXSendStatus sendMessageWithAck(uint8_t msgClass, uint8_t msgId,
                                   const uint8_t* payload, uint16_t length,
                                   uint16_t timeout_ms = 500);

  // Configure port to use UBX protocol only (disable NMEA)
  UBXSendStatus setUBXOnly(UBXPortId portID, bool checkAck = true,
                           uint16_t timeout_ms = 500);

  void setMessageCallback(UBXMessageCallback callback); // Set callback function
  UBXMessageCallback onUBXMessage; ///< Callback for message received

  /// @brief Copies would share transport and receive storage.
  /// @param other Receiver that cannot be copied.
  Adafruit_UBX(const Adafruit_UBX& other) = delete;
  /// @brief Assignment is disabled to preserve receive ownership.
  /// @param other Receiver that cannot be assigned.
  /// @return No value; this operation is deleted.
  Adafruit_UBX& operator=(const Adafruit_UBX& other) = delete;

 private:
  Stream* _stream; // Stream interface for reading data

  // Buffer for reading messages
  static const uint16_t MAX_PAYLOAD_SIZE = 96; ///< Includes 92-byte NAV-PVT.
  static const uint16_t RECEIVE_GAP_MS = 1000; ///< Abandon interrupted frames.
  static const uint16_t TX_CHUNK_SIZE = 32; ///< Bounded transmit stack storage.
  static const uint8_t RTCM3_PREAMBLE = 0xD3; ///< RTCM3 transport marker.
  uint8_t _buffer[MAX_PAYLOAD_SIZE];          ///< Default payload storage only.
  uint8_t* _payload;           ///< Default or caller-owned UBX payload storage.
  uint16_t _capacity;          ///< Available payload storage.
  Adafruit_GNSS* _nmea = NULL; ///< Optional caller-owned NMEA receiver.
  UBXNMEACallback _nmeaCallback = NULL; ///< Optional complete-line callback.
  bool _nmeaActive = false;             ///< NMEA owns incoming bytes until LF.
  bool _checking = false;    ///< Prevent receive reentry from callbacks.
  bool _transaction = false; ///< Prevent nested command transactions.
  bool _waitingAck = false;  ///< Match ACK only after the command is sent.
  uint8_t _ackClass = 0;     ///< Class of command awaiting acknowledgment.
  uint8_t _ackId = 0;        ///< ID of command awaiting acknowledgment.
  UBXSendStatus _ackStatus = UBX_SEND_TIMEOUT; ///< First matching ACK/NAK.
  uint32_t _byteTime = 0;      ///< Timestamp of most recently consumed byte.
  uint16_t _rtcmRemaining = 0; ///< Bytes left in an ignored RTCM3 frame.
  uint8_t _rtcmHeader = 0;     ///< RTCM3 header bytes read, 0 when inactive.
  UBX_CFG_PRT_t* _portReply = NULL; ///< Temporary port-query destination.
  bool _portReceived = false;       ///< Matching port configuration arrived.

  // Parser state machine
  enum ParserState {
    WAIT_SYNC_1,    // Waiting for first sync char (0xB5)
    WAIT_SYNC_2,    // Waiting for second sync char (0x62)
    GET_CLASS,      // Reading message class
    GET_ID,         // Reading message ID
    GET_LENGTH_1,   // Reading length LSB
    GET_LENGTH_2,   // Reading length MSB
    GET_PAYLOAD,    // Reading payload
    GET_CHECKSUM_A, // Reading checksum A
    GET_CHECKSUM_B  // Reading checksum B
  };

  ParserState _parserState = WAIT_SYNC_1; // Current state of the parser
  uint8_t _msgClass;                      // Message class of current message
  uint8_t _msgId;                         // Message ID of current message
  uint16_t _payloadLength;                // Length of current message payload
  uint16_t _payloadCounter;               // Counter for payload bytes received
  uint8_t _checksumA;                     // Running checksum A
  uint8_t _checksumB;                     // Running checksum B
  bool _checksumMatches;                  ///< Received checksum A matched.
  bool _replyEligible; ///< Packet started after this command was armed.

  // Frame dispatch and command transaction helpers
  bool processByte(uint8_t byte, uint32_t now);
  void addChecksum(uint8_t byte);
  bool prepareTransaction(uint16_t timeout_ms);

  // Reset parser state
  void resetParser();

  void printHex(uint8_t val);
  void printHexBuffer(const __FlashStringHelper* label, const uint8_t* buf,
                      uint16_t len);
};

#endif // ADAFRUIT_UBX_H
