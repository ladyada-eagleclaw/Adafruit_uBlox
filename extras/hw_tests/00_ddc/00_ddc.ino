// SAM-M8Q on Jumperless + classic Nano V3, uploaded over the Nano's own USB.
// Uses the breakout's level-shifted SDA/SCL. No reset, power, or UART pin writes.
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
uint8_t savedRate[6];
bool gotRate = false;
bool gotPVT = false;
uint32_t validLines = 0;
uint32_t invalidLines = 0;
uint32_t positions = 0;
uint32_t fixes = 0;
bool testPassed = false;

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload);
void receiveNMEA(const nmea_sentence_t& sentence);
void receiveFor(uint32_t duration);
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q DDC hardware test"));

  if (!ddc.begin()) fail(F("No DDC response at 0x42"));
  Serial.println(F("PASS: DDC begin"));
  if (!ubx.begin()) fail(F("Parser begin failed"));
  ubx.setMessageCallback(receiveUBX);
  ubx.setNMEAParser(&gnss, receiveNMEA);
  Serial.println(F("PASS: shared parser begin"));

  // A zero-payload UBX poll must return a real, checksummed receiver reply.
  if (!ubx.sendMessage(UBX_CLASS_CFG, UBX_CFG_RATE, NULL, 0))
    fail(F("Rate poll write failed"));
  receiveFor(1500);
  if (!gotRate) fail(F("No six-byte CFG-RATE reply"));
  Serial.println(F("PASS: zero-length poll and CFG-RATE response"));

  // Reapply exactly the read-back rate, without changing user settings or NVM.
  // This exercises ACK dispatch while navigation continues on the same DDC bus.
  if (ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_RATE, savedRate,
                             sizeof(savedRate), 1500) != UBX_SEND_SUCCESS)
    fail(F("No matching configuration ACK"));
  Serial.println(F("PASS: configuration ACK amid navigation traffic"));

  if (!ubx.sendMessage(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0))
    fail(F("NAV-PVT poll write failed"));
  receiveFor(1500);
  if (!gotPVT) fail(F("No 92-byte NAV-PVT response"));
  Serial.println(F("PASS: complete 92-byte NAV-PVT response"));

  uint32_t before = validLines;
  receiveFor(10000);
  if (validLines - before < 5 || positions < 2)
    fail(F("Insufficient valid NMEA navigation traffic"));
  if (invalidLines) fail(F("NMEA checksum or format errors"));
  Serial.println(F("PASS: sustained shared NMEA/GNSS reception"));
  Serial.print(F("Position fixes observed: "));
  Serial.println(fixes);
  Serial.println(F("A satellite fix is not required for this communication test."));
  Serial.println(F("PASS: SAM-M8Q DDC test complete; settings retained"));
  testPassed = true;
}

void loop() {
  ubx.checkMessages();
  static uint32_t lastReport = 0;
  if (testPassed && millis() - lastReport >= 5000) {
    lastReport = millis();
    Serial.print(F("Valid NMEA: "));
    Serial.print(validLines);
    Serial.print(F(", invalid: "));
    Serial.println(invalidLines);
  }
}

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload) {
  if (cls == UBX_CLASS_CFG && id == UBX_CFG_RATE && length == sizeof(savedRate)) {
    memcpy(savedRate, payload, sizeof(savedRate));
    gotRate = true;
  }
  if (cls == UBX_CLASS_NAV && id == UBX_NAV_PVT && length == 92) gotPVT = true;
}

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status != NMEA_FRAME_VALID) {
    ++invalidLines;
    return;
  }
  ++validLines;
  gnss_position_t position = Adafruit_GNSS::parsePosition(sentence);
  if (position.validation.status == GNSS_SENTENCE_VALID) {
    ++positions;
    if (position.fix) ++fixes;
  }
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) ubx.checkMessages();
}

void fail(const __FlashStringHelper* message) {
  // No test GPIO outputs or temporary configuration need restoring.
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) delay(10);
}
