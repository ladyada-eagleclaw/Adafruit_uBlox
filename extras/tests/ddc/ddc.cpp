#include <type_traits>

#include "TestHarness.h"
int main() {
  static_assert(!std::is_copy_constructible<Adafruit_UBloxDDC>::value,
                "Owned device");
  static_assert(!std::is_copy_constructible<Adafruit_UBX>::value,
                "Owned buffers");
  TwoWire wire;
  Adafruit_UBloxDDC ddc(0x42, &wire);
  assert(ddc.begin());
  assert(ddc.read() == -1 && ddc.peek() == -1 && ddc.available() == 0);
  wire.input.push_back(0xFF);
  assert(ddc.peek() == 0xFF && ddc.peek() == 0xFF && ddc.available() == 1);
  ddc.flush();
  assert(ddc.peek() == 0xFF);
  uint8_t buffer[128];
  assert(ddc.readBytes(buffer, 1) == 1 && buffer[0] == 0xFF &&
         !ddc.available());
  wire.input.push_back(42);
  assert(ddc.peek() == 42);
  wire.failRead = true;
  assert(ddc.available() == 1 && ddc.readMessage(buffer, 1) == 1 &&
         buffer[0] == 42);
  wire.failRead = false;
  for (int i = 0; i < 90; ++i)
    wire.input.push_back(i);
  assert(ddc.peek() == 0 && ddc.available() == 90);
  assert(ddc.readBytes(buffer, 128) == 90);
  for (int i = 0; i < 90; ++i)
    assert(buffer[i] == i);
  assert(!ddc.readMessage(NULL));
  assert(!ddc.readBytes(NULL, 3));
  wire.reportedCount = 65535;
  assert(ddc.available() > 0);
  wire.reportedCount = -1;
  for (size_t limit : {size_t(16), size_t(32), size_t(64)}) {
    wire.capacity = limit;
    for (size_t count :
         {size_t(2), size_t(32), size_t(33), size_t(65), size_t(97)}) {
      wire.writes.clear();
      Bytes payload(count, 43), joined;
      assert(ddc.write(payload.data(), count) == count);
      for (const auto& part : wire.writes) {
        assert(part.size() >= 2 && part.size() <= limit);
        joined.insert(joined.end(), part.begin(), part.end());
      }
      assert(joined == payload);
    }
  }
  wire.capacity = 32;
  wire.writes.clear();
  wire.failWrite = 1;
  assert(ddc.write(buffer, 90) == 32);
  assert(ddc.write(NULL, 2) == 0 && ddc.write(buffer, 1) == 0);
}
