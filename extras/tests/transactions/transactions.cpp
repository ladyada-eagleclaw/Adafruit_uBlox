#include "TestHarness.h"
static int callbacks;
static int nmeaCallbacks;
static Adafruit_UBX* receiver;
static void nmeaCallback(const nmea_sentence_t& sentence) {
  assert(sentence.status == NMEA_FRAME_VALID);
  ++nmeaCallbacks;
}
static void callback(uint8_t, uint8_t, uint16_t, uint8_t*) {
  ++callbacks;
  assert(!receiver->checkMessages());
  assert(!receiver->begin());
  assert(receiver->sendMessageWithAck(6, 8, NULL, 0) == UBX_SEND_FAIL);
}
int main() {
  FakeStream port;
  Adafruit_UBX ubx(port);
  receiver = &ubx;
  char first[100], second[100];
  Adafruit_GNSS gnss(first, second, sizeof(first));
  ubx.setNMEAParser(&gnss, nmeaCallback);
  ubx.setMessageCallback(callback);
  auto ack = packet(5, 1, {6, 8});
  auto nak = packet(5, 0, {6, 8});
  for (bool accept : {true, false}) {
    port.reply = [&]() {
      port.push(sentence("GNGLL,,,,,123456,V,N"));
      port.push(packet(5, 1, {6, 9}));    // Wrong command.
      port.push(packet(5, 1, {6, 8, 0})); // Invalid ACK length.
      port.push(accept ? ack : nak);
      port.push(packet(1, 7, Bytes(92)));
    };
    assert(ubx.sendMessageWithAck(6, 8, NULL, 0) ==
           (accept ? UBX_SEND_SUCCESS : UBX_SEND_NAK));
    drain(ubx, port);
  }
  assert(callbacks == 8);
  assert(nmeaCallbacks == 2);
  port.push(ack); // Stale queued ACK is dispatched before arming the new wait.
  port.reply = [&]() {
    port.push(nak);
    port.push(ack);
  };
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0) == UBX_SEND_NAK);
  drain(ubx, port);
  // A queued partial ACK must be consumed after the write without becoming the
  // new command's reply. Partial NMEA is retained across the same boundary.
  port.push(Bytes(ack.begin(), ack.begin() + 4));
  port.reply = [&]() {
    port.push(Bytes(ack.begin() + 4, ack.end()));
    port.push(nak);
  };
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0) == UBX_SEND_NAK);
  auto line = sentence("GNGLL,,,,,123456,V,N");
  port.push(Bytes(line.begin(), line.begin() + 8));
  port.reply = [&]() {
    port.push(Bytes(line.begin() + 8, line.end()));
    port.push(ack);
  };
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0) == UBX_SEND_SUCCESS);
  assert(nmeaCallbacks == 3);
  // Nor may a partial RTCM frame expose its embedded ACK after arming.
  port.push({0xD3, 0, (uint8_t)ack.size()});
  port.reply = [&]() {
    port.push(ack);
    port.push({0, 0, 0});
    port.push(nak);
  };
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0) == UBX_SEND_NAK);
  port.reply = [&]() { port.push(packet(5, 1, {6, 9})); };
  testClock() = UINT32_MAX - 20;
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0, 30) == UBX_SEND_TIMEOUT);
  drain(ubx, port);
  port.endless = true;
  size_t sent = port.output.size();
  uint32_t start = testClock();
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0, 30) == UBX_SEND_TIMEOUT);
  assert((uint32_t)(testClock() - start) < 40 && port.output.size() == sent);
  port.endless = false;
  port.reply = [&]() { port.endless = true; };
  start = testClock();
  assert(ubx.sendMessageWithAck(6, 8, NULL, 0, 30) == UBX_SEND_TIMEOUT);
  assert((uint32_t)(testClock() - start) < 40);
  port.endless = false;
  port.reply = nullptr;
  assert(!ubx.sendMessage(6, 8, NULL, 1));

  // Streaming transmission reproduces the exact wire packet at every boundary.
  for (size_t count : {size_t(0), size_t(1), size_t(24), size_t(25), size_t(57),
                       size_t(92), size_t(65535)}) {
    port.output.clear();
    port.chunks.clear();
    Bytes payload(count, 0xA5);
    assert(ubx.sendMessage(6, 8, payload.data(), count));
    assert(port.output == packet(6, 8, payload));
    for (size_t size : port.chunks)
      assert(size >= 2 && size <= 32);
  }
  port.output.clear();
  port.chunks.clear();
  port.shortWrite = 1;
  Bytes payload(100);
  assert(!ubx.sendMessage(6, 8, payload.data(), payload.size()));
  assert(port.output.size() == 32);
  port.shortWrite = -1;

  // Port configuration preserves the live baud rate, mode, address and flags.
  port.output.clear();
  port.chunks.clear();
  UBX_CFG_PRT_t config = {};
  config.fields.portID = UBX_PORT_UART1;
  config.fields.mode = UBX_UART_MODE_8N1;
  config.fields.baudRate = 115200;
  config.fields.inProtoMask = 3;
  config.fields.outProtoMask = 3;
  config.fields.flags = 2;
  port.reply = [&]() {
    if (port.output.size() == 9)
      port.push(packet(6, 0, Bytes(config.raw, config.raw + 20)));
    else
      port.push(packet(5, 1, {6, 0}));
  };
  assert(ubx.setUBXOnly(UBX_PORT_UART1) == UBX_SEND_SUCCESS);
  config.fields.inProtoMask = 1;
  config.fields.outProtoMask = 1;
  auto expected = packet(6, 0, Bytes(config.raw, config.raw + 20));
  assert(Bytes(port.output.begin() + 9, port.output.end()) == expected);
}
