// Verify actual navigation epochs at 1 Hz, 2 Hz, then 1 Hz over DDC.
// Requires GGA output with valid UTC time. No fix or GPIO outputs are required.
// Restores all six original CFG-RATE bytes; never saves to battery RAM or flash.
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

union Rate {
  struct {
    uint16_t measurementMilliseconds;
    uint16_t navigationCycles;
    uint16_t timeReference;
  } fields;
  uint8_t raw[6];
};
static_assert(sizeof(Rate) == 6, "CFG-RATE must be six bytes");

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
Rate receivedRate, originalRate;
bool gotRate = false, needsRestore = false, measuring = false, haveEpoch = false;
uint16_t expectedMilliseconds = 1000;
uint32_t previousEpoch = 0;
uint16_t goodIntervals = 0, badIntervals = 0, invalidLines = 0, fixReports = 0;

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload);
void receiveNMEA(const nmea_sentence_t& sentence);
void receiveFor(uint32_t duration);
bool pollRate();
bool writeRate(const Rate& rate);
bool restoreRate();
void testRate(uint16_t milliseconds);
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q navigation rate hardware test"));

  if (!ddc.begin()) fail(F("DDC begin failed"));
  if (!ubx.begin()) fail(F("Parser begin failed"));
  ubx.setMessageCallback(receiveUBX);
  ubx.setNMEAParser(&gnss, receiveNMEA);
  if (!pollRate()) fail(F("Could not read original rate"));
  originalRate = receivedRate;
  Serial.print(F("PASS: original measurement interval ms: "));
  Serial.println(originalRate.fields.measurementMilliseconds);

  testRate(1000);
  testRate(500);
  testRate(1000);

  if (!restoreRate()) fail(F("Could not restore original rate"));
  Serial.println(F("PASS: original CFG-RATE restored and verified; no NVM save"));
  Serial.println(F("PASS: navigation rate test complete"));
}

void loop() {
  ubx.checkMessages();
}

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload) {
  if (cls == UBX_CLASS_CFG && id == UBX_CFG_RATE && length == sizeof(Rate)) {
    memcpy(receivedRate.raw, payload, sizeof(Rate));
    gotRate = true;
  }
}

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (!measuring) return;
  if (sentence.status != NMEA_FRAME_VALID) {
    ++invalidLines;
    return;
  }
  // Use only GGA so RMC/GLL from the same epoch do not count as duplicates.
  if (sentence.address.length != 5 ||
      memcmp(sentence.address.data + 2, "GGA", 3)) return;
  gnss_position_t position = Adafruit_GNSS::parsePosition(sentence);
  if (position.validation.status != GNSS_SENTENCE_VALID ||
      position.time.status != NMEA_NUMBER_VALID) {
    ++invalidLines;
    return;
  }
  if (position.fix) ++fixReports;
  uint32_t epoch = ((uint32_t)position.time.hour * 3600 +
                    (uint32_t)position.time.minute * 60 + position.time.second) *
                   1000 + position.time.millisecond;
  if (haveEpoch) {
    // UTC wraps at midnight; this short cadence test excludes leap seconds.
    uint32_t interval = (epoch + 86400000UL - previousEpoch) % 86400000UL;
    if (interval == expectedMilliseconds) ++goodIntervals;
    else ++badIntervals;
  }
  previousEpoch = epoch;
  haveEpoch = true;
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) ubx.checkMessages();
}

bool pollRate() {
  gotRate = false;
  if (!ubx.sendMessage(UBX_CLASS_CFG, UBX_CFG_RATE, NULL, 0)) return false;
  uint32_t start = millis();
  while (!gotRate && millis() - start < 2000) ubx.checkMessages();
  return gotRate;
}

bool writeRate(const Rate& rate) {
  return ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_RATE, rate.raw,
                                sizeof(Rate), 2000) == UBX_SEND_SUCCESS;
}

bool restoreRate() {
  if (!needsRestore) return true;
  if (!writeRate(originalRate)) return false;
  if (!pollRate() || memcmp(receivedRate.raw, originalRate.raw, sizeof(Rate)))
    return false;
  needsRestore = false;
  return true;
}

void testRate(uint16_t milliseconds) {
  Rate requested = originalRate;
  requested.fields.measurementMilliseconds = milliseconds;
  requested.fields.navigationCycles = 1;
  needsRestore = true;
  if (!writeRate(requested)) fail(F("Rate command ACK failed"));
  if (!pollRate() || memcmp(receivedRate.raw, requested.raw, sizeof(Rate)))
    fail(F("Rate readback mismatch"));
  Serial.print(F("PASS: configured measurement interval ms: "));
  Serial.println(milliseconds);

  // Drain output from the previous configuration before counting new epochs.
  receiveFor(2500);
  expectedMilliseconds = milliseconds;
  haveEpoch = false;
  goodIntervals = badIntervals = invalidLines = fixReports = 0;
  measuring = true;
  receiveFor(15000);
  measuring = false;
  Serial.print(F("Good intervals: ")); Serial.print(goodIntervals);
  Serial.print(F(", bad intervals: ")); Serial.print(badIntervals);
  Serial.print(F(", invalid NMEA: ")); Serial.print(invalidLines);
  Serial.print(F(", fix reports: ")); Serial.println(fixReports);
  if (goodIntervals < 15000 / milliseconds - 2 || badIntervals || invalidLines)
    fail(F("Observed GGA epochs do not match the requested rate"));
  Serial.println(F("PASS: navigation timestamps match the configured rate"));
}

void fail(const __FlashStringHelper* message) {
  measuring = false;
  Serial.print(F("FAIL: "));
  Serial.println(message);
  if (needsRestore) {
    if (restoreRate()) Serial.println(F("Original rate restored and verified"));
    else Serial.println(F("RESTORE FAILED: restore original CFG-RATE before continuing"));
  }
  while (true) delay(10);
}
