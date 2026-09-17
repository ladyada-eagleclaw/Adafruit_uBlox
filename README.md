# Adafruit uBlox Library [![Build Status](https://github.com/adafruit/Adafruit_uBlox/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_uBlox/actions) [![Documentation](https://img.shields.io/badge/Documentation-doxygen-blue.svg)](https://adafruit.github.io/Adafruit_uBlox/html/index.html)

This is a driver library to abstract away the details of communicating with u-blox GPS and RTK modules. It provides a simple interface for sending and receiving data over a generic Stream interface.

This library provides two main classes:
- `Adafruit_UBX`: Interfaces with u-blox GPS/RTK modules using any stream object as an input (UART, DDC, or other) and parses UBX protocol messages.
- `Adafruit_UBloxDDC`: Interface for communicating with u-blox modules over DDC (I2C).

Install **Adafruit uBlox** from Arduino Library Manager, including its dependencies:
Adafruit BusIO and **Adafruit GPS Library 1.9.0 or later**. The GPS package supplies
the shared `Adafruit_NMEA` and `Adafruit_GNSS` classes; no separate NMEA or GNSS
library installation is needed.

For mixed NMEA and UBX traffic, attach a caller-owned `Adafruit_GNSS` parser with
`ubx.setNMEAParser(&gnss, callback)` and call `ubx.checkMessages()` from `loop()`.
This is the connection's only reader. NMEA callbacks continue during command
ACK waits, and `Adafruit_GNSS::parsePosition()` returns validated per-sentence
position data. `formatCoordinate()` preserves the exact coordinate components
without converting through `float`. See **ublox_ddc_parse_nmea** for a complete
DDC example with two caller-owned sentence buffers.

`checkMessages()` processes at most 256 bytes per call; call it frequently.
Its return value reports a complete UBX packet, while NMEA arrives through the
attached parser/callback. UBX callbacks borrow the payload only for the duration
of the call. Consume NMEA views in their callback before reception continues.
Callbacks must not read the transport or start another blocking transaction.

UBX receive storage defaults to 96 payload bytes, enough for M8 NAV-PVT. Use
`setPayloadBuffer(buffer, capacity)` for larger replies such as extended MON-VER
or NAV-SAT. The caller keeps that storage alive; `setPayloadBuffer(NULL, 0)`
restores internal storage. Oversized packets are consumed without dispatching
their contents as other messages. Interrupted frames recover after a one-second
receive gap. RTCM3 frames are skipped by length; RTCM decoding and forwarding
are not provided.

`sendMessageWithAck()` returns distinct success, NAK, send-failure and timeout
results. Queued replies are drained before sending, and the first matching
ACK/NAK is retained. UBX has no transaction sequence number, so a delayed ACK
for an identical earlier command remains ambiguous. Transport operations and
callbacks must return promptly for elapsed-time deadlines to be effective.

`setUBXOnly()` polls the selected port first and preserves its baud rate,
address, framing and other settings while changing protocol masks. Selecting
UBX alone does not enable periodic navigation messages: **ublox_ubxtest** polls
NAV-PVT explicitly. **ublox_ddc** shows raw DDC streaming. Examples do not save
configuration to receiver flash.

Protocol details follow the [u-blox 8/M8 receiver protocol specification](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf),
including DDC's minimum two-byte writes and CFG-PRT read/modify/write behavior.

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

MIT license, all text above must be included in any redistribution
