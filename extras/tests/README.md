# Host regressions

Run all tests using Linux g++, Adafruit GPS 1.9.0 or later, and a checkout of
[Adafruit_TestHarness](https://github.com/adafruit/Adafruit_TestHarness):

```sh
python3 extras/tests/run_tests.py --gps-dir ../Adafruit_GPS \
  --harness-dir ../Adafruit_TestHarness
```

CI discovers every `extras/tests/*/*.cpp` and runs it with AddressSanitizer,
UndefinedBehaviorSanitizer, and compiler warnings as errors. Arduino compatibility,
the fake clock, fake serial streams, and the runner come from TestHarness. The
local support files contain only u-blox packet helpers and DDC-specific I2C/BusIO
fakes. These are deterministic transport fakes, not receiver hardware emulation.
TestHarness is a development dependency, not an Arduino sketch dependency.

Coverage includes zero-length/92-byte/oversized UBX packets, checksum and gap
recovery, overlapping sync bytes, NMEA/UBX/RTCM isolation, exact coordinates,
ACK/NAK ordering, queued and partial stale replies, clock rollover, reentrant callbacks,
bounded continuous input, constant-memory transmit boundaries, port-setting
preservation, ACK/port replies behind queued navigation without per-byte sleeps,
DDC peek/read semantics, count-query reuse/error recovery, and Wire-sized writes.

RTCM3 input is skipped by its length; no RTCM CRC validation or forwarding is
claimed. UBX ACKs have no transaction sequence number: an identical delayed
reply that arrives after a new command cannot be distinguished from its reply.

Hardware sketches are separate under `extras/hw_tests` and are run explicitly.
