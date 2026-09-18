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
| 16 | RX | D7, open drain, only for `04_uart_commands` |
| 17 | TX | Nano D8, 9600 baud |
| 18 | INT / wake | Jumperless GPIO_1 and Nano A1, only for `06_wake` |
| 43 | VIN | Existing 5 V power jumper |
| 44 | 3.3 V output | Existing Jumperless ADC probe; do not drive |
| 45 | GND | Common breakout ground |
| 46 | SCL | Nano A5, breakout's level-shifted host side |
| 47 | SDA | Nano A4, breakout's level-shifted host side |
| 48 | RESET_N | D9 open drain and A0 sense, only for `05_reset` |

This pin order was checked against `SAM-MxQ rev A.sch`. Do not infer it for a
rotated board or a different revision. Retain the physical power/ground jumpers.
The 3.3 V receiver outputs must not be pulled toward 5 V. UART and PPS sketches
disable input pull-ups; UART TX D7 stays disconnected except during test 04.
The DDC bus
uses the breakout's level shifter, not bare receiver pins.

Run `jumperless_setup.py` on the Jumperless MicroPython interface. It adds only
the four signal routes above, checks for conflicting Nano routes, and leaves
power settings and unrelated wiring alone. PA1010D at 0x10 can retain its
existing row 24/A4 and row 25/A5 routes on the shared host bus; M8Q uses 0x42.
Do not join PA6H TX row 5 and M8Q TX row 17 to the same input.

For tests 04-06, first upload the selected sketch so its outputs start released.
Then execute `jumperless_controls.py` with `MODE` set to `uart`, `reset`, or
`wake_low`. The sketch waits for `s` over Nano USB before starting. For wake,
run the helper again with `MODE = "wake_high"` when the sketch prints
`WAKE NOW`. Finally run `MODE = "release"` to remove these extra control
routes before another sketch or the base setup script. D12 is the unused
SoftwareSerial TX pin in tests 04-06 and must remain disconnected.

The helper runs on Jumperless MicroPython, for example by executing the text
`MODE = "uart"` followed by the file contents in the same global context.
Use named `j.INPUT` / `j.OUTPUT` constants: this firmware interprets integer
direction arguments differently from Python booleans.

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
- `04_uart_commands`: sends through the breakout's diode-protected RX input
  using a Nano open-drain 9600-baud transmitter. Verifies CFG-RATE polling and
  ACK, a MON-VER response larger than the default 96-byte buffer, and NAV-PVT
  after restoring the internal buffer. Limited to a 16 MHz ATmega328P; software
  UART transmission is half duplex, so this is not a full-duplex stress test.
- `05_reset`: senses RESET_N on A0, asserts it with D9 pulling low, verifies
  navigation stops, releases it, then verifies voltage and navigation recover.
  No 5 V output or pull-up is applied to RESET_N. Analog readings use the
  nominal Nano 5 V reference and are signal-state checks, not precision metrology.
- `06_wake`: for protocol 18+, sends RXM-PMREQ over DDC with a 30-second
  automatic recovery timer and only EXTINT0 enabled as a pin wake source.
  Observes UART silence, then verifies navigation resumes after the 3.3 V
  Jumperless edge before the timer expires. A1 senses both pin states. It does
  not establish backup-mode current consumption or battery retention.
- `07_backup`: retains a nondefault navigation-rate marker across a verified
  main-power interruption, then restores the original runtime rate and saves
  that navigation configuration back to battery-backed RAM. Requires the
  manual procedure below. No flash or EEPROM writes; this deliberately saves
  the current navigation profile to BBR, replacing any older stored profile.

### Battery retention procedure

1. Fit a CR1220. Leave Nano/Jumperless USB and module ground connected. Keep
   the Nano serial session open throughout: restarting it loses the saved rate.
2. Release optional controls, restore the base signal routes, and upload
   `07_backup`. At `READY`, send `s` with no line ending. It snapshots the rate,
   changes it to a test marker, and saves navigation settings to BBR only.
3. At `ARMED`, execute `jumperless_power.py` with `MODE = "isolate"`. It
   disconnects the four signal routes without changing power rails or ADC
   probes. Remove only the SAM-M8Q VIN jumper(s), not the shared rail supply.
4. Use `MODE = "status"` to verify **both VIN and 3.3 V are below 0.2 V**
   relative to module ground. Leave power off for at least 20 seconds. A
   retained marker with the main rail still powered does not prove backup.
5. Reconnect VIN; verify the 3.3 V rail is within 3.0-3.6 V, then execute
   `MODE = "restore"` to restore exactly the isolated signal routes. Send `v`
   only after confirming the power interruption. The sketch checks retention
   and restores the original rate in runtime and BBR.
6. To cancel, restore power and routes and send `a`. The sketch attempts rate
   restoration on failure or after five minutes. Never leave a reported
   `RESTORE FAILED` unresolved. Restoring BBR does not measure coin-cell
   capacity or long-term retention.

The power helper measures existing ADC0/ADC2/ADC3 contacts on VIN/3.3 V/GND and
subtracts the ground ADC reading. It does not switch the shared power rail.
Run isolation only once per interruption so its saved route list is retained.

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

## Additional pin tests, 2026-09-18

On the same Rev A fixture, UART CFG-RATE polling and ACK passed through RX/TX.
A 160-byte MON-VER reply passed using the caller-owned buffer (SPG 3.01,
protocol 18.00), followed by NAV-PVT with the default buffer restored.
RESET_N measured 3.543 V released and 0.048 V asserted using the Nano's nominal
reference; navigation stopped and recovered as expected. EXTINT0 measured
0.029 V low and 3.523 V high, and woke navigation before the 30-second timer.
These Nano voltage readings include USB-supply/reference error.

Battery retention **failed** with a coin cell installed. With all signal routes
isolated, the measured main rails fell to VIN 0.005 V and VCC 0.009 V for over
20 seconds; after power returned, the nondefault 1500 ms rate marker was lost.
The original 1000 ms rate was restored in runtime and BBR. A separate BBR
save/load diagnostic retained the marker while main power stayed present,
confirming the command sequence works. Check battery voltage, holder contacts
and the V_BCKP path before considering backup retention verified.
With main power restored, VIN measured about 4.50 V and VCC about 3.30 V.
After cleanup, a 25-second receive-only capture contained 255 valid NMEA
messages, zero invalid frames and 23 fix reports. The last report was a 3D fix
using six satellites with HDOP 2.31; this is a short acquisition observation,
not an accuracy or long-duration positioning test.

Still outside this fixture's coverage: individual QT/header contact testing,
LED visibility and jumpers, supply-range/current/temperature sweeps, absolute
PPS accuracy, and sustained positioning accuracy. Rev B circuitry has not
been tested. SAFEBOOT is reserved in the SAM-M8Q datasheet and is left alone.

Pin voltage limits and battery behavior follow the
[SAM-M8Q datasheet](https://content.u-blox.com/sites/default/files/documents/SAM-M8Q_DataSheet_UBX-16012619.pdf).
The command layouts follow sections 32.10.3, 32.10.27 and 32.18.3 of the
[u-blox 8/M8 protocol manual](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf).
