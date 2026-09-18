# SAM-M8Q breakout on Jumperless and Nano V3

Use the classic ATmega328P Nano **in the Jumperless socket and connected through
its own USB cable**. Upload and monitor through that cable, not the Jumperless
UART passthrough. Keep Jumperless UART_TX/D0 and UART_RX/D1 disconnected.
The verified build target is `arduino:avr:nano:cpu=atmega328` (new bootloader).

The established SAM-M8Q prototype straddles the breadboard in this orientation:

| Row | Breakout pin | Test connection |
| --- | --- | --- |
| 13 | GND | Existing ground jumper |
| 14 | PPS | Nano D6, input only |
| 15 | VIN | Existing power arrangement; do not add a second supply |
| 16 | RX | Unconnected for these tests |
| 17 | TX | Nano D8, 9600 baud |
| 18 | INT / wake | Unconnected |
| 43 | VIN | Existing 5 V power jumper |
| 44 | 3.3 V output | Existing Jumperless ADC probe; do not drive |
| 45 | GND | Common breakout ground |
| 46 | SCL | Nano A5, breakout's level-shifted host side |
| 47 | SDA | Nano A4, breakout's level-shifted host side |
| 48 | RESET_N | Unconnected |

This pin order was checked against `SAM-MxQ rev A.sch`. Do not infer it for a
rotated board or a different revision. Retain the physical power/ground jumpers.
The 3.3 V receiver outputs must not be pulled toward 5 V. UART and PPS sketches
disable input pull-ups; UART TX D7 stays physically disconnected. The DDC bus
uses the breakout's level shifter, not bare receiver pins.

Run `jumperless_setup.py` on the Jumperless MicroPython interface. It adds only
the four signal routes above, checks for conflicting Nano routes, and leaves
power settings and unrelated wiring alone. PA1010D at 0x10 can retain its
existing row 24/A4 and row 25/A5 routes on the shared host bus; M8Q uses 0x42.
Do not join PA6H TX row 5 and M8Q TX row 17 to the same input.

On the development fixture, Nano USB is COM17 and Jumperless MicroPython is
COM11. Re-enumerate ports before use; port numbers are not permanent.

## Sketches

- `00_ddc`: initializes DDC, polls CFG-RATE with a zero-length payload, reapplies
  the exact current settings and checks the ACK, polls a full 92-byte NAV-PVT,
  and checks ten seconds of NMEA/GNSS traffic. It does not save to flash or
  require a satellite fix. Any failure prints its step and halts safely.
- `01_uart`: receives M8Q's existing 9600-baud NMEA for fifteen seconds through
  D8, checks framing/decoding and SoftwareSerial overflow, then continues
  reporting counts. Sends no receiver commands. Other configured baud rates
  or disabled UART NMEA must be restored before this test.
- `02_pps`: input-only observation of D6 for fifteen seconds. Reports PASS only
  after at least three 1 Hz periods. Missing/configuration-dependent PPS is
  explicitly NOT VERIFIED. It does not establish timing accuracy.
- `03_port`: captures the complete original DDC port configuration, selects
  UBX-only output, verifies that NMEA stops while UBX navigation/poll replies
  continue, checks that all other port settings are preserved, then restores
  the original configuration and verifies that NMEA resumes. Failure paths
  attempt restoration and explicitly report a failed restore. No NVM save.

From this checkout, compile each sketch explicitly, for example:

```sh
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 --library . extras/hw_tests/00_ddc
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328 --port COM17 extras/hw_tests/00_ddc
```

Monitor at 115200 baud. Keep captures containing locations private. These
engineering tests are separate from host CI and are not production acceptance
of antenna sensitivity, position accuracy, backup retention, reset/wake,
temperature range or the final PCB revision.

## Bench evidence, 2026-09-17

On the existing SAM-M8Q prototype with Nano USB and the routes above, DDC polling,
ACK handling, the 92-byte NAV-PVT response, and shared NMEA parsing passed.
The DDC capture reached 144 valid NMEA lines with zero invalid frames. UART
reception reached 160 valid lines and 60 decoded position sentences with zero
invalid frames. No satellite fix was available during the DDC run; position
accuracy is untested. The initial indoor PPS run did not observe stable 1 Hz
pulses and reported NOT VERIFIED.

The port-configuration test also passed: UBX-only operation suppressed NMEA,
UBX replies remained available, all other DDC port settings matched their
original values, and restoration resumed NMEA output. No settings were saved
to nonvolatile memory.

After moving the fixture outdoors, the receiver acquired a valid 2D fix. All
four hardware sketches were compiled again for the Nano and passed live:

- DDC decoded 39 position fixes, received the complete NAV-PVT reply, and
  reached 162 valid NMEA lines with zero invalid frames.
- UART received 179 valid NMEA lines and decoded 60 position sentences with
  zero invalid frames and no SoftwareSerial overflow.
- PPS passed the standard 1 Hz test. A separate input-only diagnostic counted
  16 rising edges in 15 seconds, with intervals of 997-999 ms on the Nano's
  clock. This verifies the signal path, not absolute timing accuracy.
- The protocol-switching test passed again and restored the original DDC
  settings, with NMEA output resuming. No settings were saved to NVM.

Read-only UBX polls also returned the 32-byte CFG-TP5 configuration and a
92-byte NAV-PVT reporting a valid 2D fix using three satellites. CFG-TP5 had
timepulse enabled, a one-second period, and a 100 ms pulse when time-locked.
The brief initial outdoor PPS failure cleared without changing these settings.
In the final 20-second UART capture, eight GPS satellites were reported in view,
but no valid fix was available: reception remains intermittent. Sustained 3D
positioning, position accuracy and the final PCB revision remain untested.
The Nano was left running a receive-only UART monitor through its own USB
connection.
