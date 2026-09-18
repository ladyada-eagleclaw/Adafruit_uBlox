#include "UBloxTestHelpers.h"
static std::vector<Bytes> messages;
static std::vector<nmea_frame_status_t> lines;
static gnss_position_t position;
static void ubxMessage(uint8_t cls, uint8_t id, uint16_t size, uint8_t* data) {
  Bytes msg = {cls, id};
  msg.insert(msg.end(), data, data + size);
  messages.push_back(msg);
}
static void nmeaMessage(const nmea_sentence_t& text) {
  lines.push_back(text.status);
  position = Adafruit_GNSS::parsePosition(text);
}
int main() {
  FakeStream port;
  Adafruit_UBX ubx(port);
  char first[160], second[160];
  Adafruit_GNSS gnss(first, second, sizeof(first));
  ubx.setNMEAParser(&gnss, nmeaMessage);
  ubx.setMessageCallback(ubxMessage);
  assert(ubx.begin());
  port.push(packet(1, 7));
  port.push(packet(1, 7, Bytes(92, 0xFF)));
  drain(ubx, port);
  assert(messages.size() == 2 && messages[0].size() == 2 &&
         messages[1].size() == 94);

  const auto fix = sentence(
      "GNGGA,123519,4807.038123456,N,01131.000987654,E,4,08,0.9,545.4,M,46.9,M,"
      ",");
  port.push(fix);
  drain(ubx, port);
  assert(lines.size() == 1 && position.fix && position.fixQuality == 4);
  assert(position.latitude.fractionalMinutes == 38123456);
  char coordinate[GNSS_COORDINATE_TEXT_SIZE];
  Adafruit_GNSS::formatCoordinate(coordinate, sizeof(coordinate),
                                  position.latitude);
  assert(std::string(coordinate) == "48.11730205760");

  // A binary payload must not leak a valid NMEA sentence into the GNSS parser.
  uint8_t larger[256];
  assert(ubx.setPayloadBuffer(larger, sizeof(larger)));
  port.push(packet(1, 2, fix));
  drain(ubx, port);
  assert(lines.size() == 1 && messages.size() == 3);
  assert(ubx.setPayloadBuffer(NULL, 0));
  assert(!ubx.setPayloadBuffer(NULL, 3));

  // Oversized frames, including the maximum UBX length, consume their payload
  // and checksums without delivering embedded UBX/NMEA frames.
  for (size_t size : {size_t(97), size_t(65535)}) {
    Bytes payload(size, 0);
    auto nested = packet(5, 1, {6, 8});
    std::copy(nested.begin(), nested.end(), payload.begin());
    port.push(packet(1, 2, payload));
    port.push(packet(1, 3));
    drain(ubx, port);
  }
  assert(messages.size() == 5 && lines.size() == 1);

  // RTCM3 length framing must isolate binary content too, including its CRC.
  auto nested = packet(5, 1, {6, 8});
  Bytes rtcm = {0xD3, 0, (uint8_t)nested.size()};
  rtcm.insert(rtcm.end(), nested.begin(), nested.end());
  rtcm.insert(rtcm.end(), {0xB5, 0x62, '$'});
  port.push(rtcm);
  port.push(fix);
  drain(ubx, port);
  assert(messages.size() == 5 && lines.size() == 2);

  for (int checksum : {0, 1}) {
    auto bad = packet(1, 7, Bytes(92, 3));
    bad[bad.size() - 2 + checksum]++;
    port.push(bad);
    port.push(packet(1, 4));
    drain(ubx, port);
  }
  assert(messages.size() == 7);
  port.push({0xB5});
  port.push(packet(1, 5));
  drain(ubx, port);
  assert(messages.size() == 8);

  // A truncated frame must recover after a receive gap, including clock wrap.
  port.push({0xB5, 0x62, 1, 7, 92, 0, 42});
  drain(ubx, port);
  testClock() += 1001;
  port.push(packet(1, 6));
  drain(ubx, port);
  assert(messages.size() == 9);
  testClock() = UINT32_MAX - 10;
  port.push({0xB5, 0x62, 1, 7});
  drain(ubx, port);
  testClock() += 1001;
  port.push(packet(1, 6));
  drain(ubx, port);
  assert(messages.size() == 10);

  Bytes bad = fix;
  bad[bad.size() - 4] = 'Z';
  port.push(bad);
  port.push(Bytes{'$', 'G', 'P'});
  drain(ubx, port);
  assert(lines.back() != NMEA_FRAME_VALID);
  port.push(Bytes(400, 'X'));
  port.push({'\n'});
  port.push(fix);
  drain(ubx, port);
  assert(lines.back() == NMEA_FRAME_VALID && position.fix);

  // Separate receivers do not share state/storage, and work stays bounded.
  FakeStream secondPort;
  Adafruit_UBX secondUbx(secondPort);
  secondPort.push(packet(2, 1));
  secondUbx.setMessageCallback(ubxMessage);
  drain(secondUbx, secondPort);
  port.endless = true;
  size_t before = port.readCount;
  assert(!ubx.checkMessages(7) && port.readCount == before + 7);
  port.readFailure = true;
  before = port.readCount;
  assert(!ubx.checkMessages() && port.readCount == before + 1);
}
