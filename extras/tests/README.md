# Host regressions

Run all tests using Linux g++ and Adafruit GPS 1.9.0 or later:

```sh
python3 extras/tests/run_tests.py --gps-dir ../Adafruit_GPS
```

CI discovers every `extras/tests/*/*.cpp` and runs it with AddressSanitizer,
UndefinedBehaviorSanitizer, and compiler warnings as errors. The support classes
are deterministic transport fakes, not receiver hardware emulation.

Coverage includes zero-length/92-byte/oversized UBX packets, checksum and gap
recovery, overlapping sync bytes, NMEA/UBX/RTCM isolation, exact coordinates,
ACK/NAK ordering, queued and partial stale replies, clock rollover, reentrant callbacks,
bounded continuous input, constant-memory transmit boundaries, port-setting
preservation, DDC peek/read semantics, and Wire-sized writes.

RTCM3 input is skipped by its length; no RTCM CRC validation or forwarding is
claimed. UBX ACKs have no transaction sequence number: an identical delayed
reply that arrives after a new command cannot be distinguished from its reply.

Hardware sketches are separate under `extras/hw_tests` and are run explicitly.
