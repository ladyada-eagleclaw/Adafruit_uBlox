// Verify CFG-PRT protocol changes and restore the complete original DDC settings.
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
UBX_CFG_PRT_t receivedPort;
UBX_CFG_PRT_t savedPort;
bool gotPort = false, haveSavedPort = false, gotPVT = false;
uint32_t nmeaCount = 0;
void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload);
void receiveNMEA(const nmea_sentence_t& sentence);
void receiveFor(uint32_t duration);
void fail(const __FlashStringHelper* message);
bool restorePort();

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q DDC port configuration test"));

  if (!ddc.begin()) fail(F("No DDC response"));
  ubx.begin();
  ubx.setMessageCallback(receiveUBX);
  ubx.setNMEAParser(&gnss, receiveNMEA);
  Serial.println(F("PASS: DDC and shared parsers initialized"));

  uint8_t portID = UBX_PORT_DDC;
  if (!ubx.sendMessage(UBX_CLASS_CFG, UBX_CFG_PRT, &portID, 1))
    fail(F("Port poll write failed"));
  receiveFor(1500);
  if (!gotPort) fail(F("No 20-byte port configuration reply"));
  memcpy(savedPort.raw, receivedPort.raw, sizeof(savedPort));
  haveSavedPort = true;
  if (!nmeaCount) fail(F("Enable DDC NMEA before running this test"));
  Serial.println(F("PASS: original port configuration and NMEA captured"));

  if (ubx.setUBXOnly(UBX_PORT_DDC, true, 1500) != UBX_SEND_SUCCESS)
    fail(F("UBX-only configuration failed"));
  Serial.println(F("PASS: UBX-only configuration acknowledged"));

  // Allow navigation queued before the configuration change to finish.
  receiveFor(1500);
  nmeaCount = 0;
  gotPort = false;
  if (!ubx.sendMessage(UBX_CLASS_CFG, UBX_CFG_PRT, &portID, 1))
    fail(F("Port readback write failed"));
  if (!ubx.sendMessage(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0))
    fail(F("NAV-PVT poll write failed"));
  receiveFor(3000);
  if (!gotPort || !gotPVT) fail(F("UBX replies stopped after port change"));
  if (nmeaCount) fail(F("NMEA continued after UBX-only configuration"));
  UBX_CFG_PRT_t expected = savedPort;
  expected.fields.inProtoMask = UBX_PROTOCOL_UBX;
  expected.fields.outProtoMask = UBX_PROTOCOL_UBX;
  if (memcmp(expected.raw, receivedPort.raw, sizeof(expected)))
    fail(F("A port setting other than protocol masks changed"));
  Serial.println(F("PASS: UBX replies continue, NMEA stops, other settings retained"));

  if (!restorePort()) fail(F("Could not restore original DDC configuration"));
  nmeaCount = 0;
  receiveFor(3000);
  if (!nmeaCount) fail(F("NMEA did not resume after restoration"));
  Serial.println(F("PASS: original configuration restored and NMEA resumed"));
  Serial.println(F("PASS: port configuration test complete; no NVM save"));
}

void loop() {
  ubx.checkMessages();
}

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload) {
  if (cls == UBX_CLASS_CFG && id == UBX_CFG_PRT && length == sizeof(receivedPort) &&
      payload[0] == UBX_PORT_DDC) {
    memcpy(receivedPort.raw, payload, sizeof(receivedPort));
    gotPort = true;
  }
  if (cls == UBX_CLASS_NAV && id == UBX_NAV_PVT && length == 92) gotPVT = true;
}

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status == NMEA_FRAME_VALID) ++nmeaCount;
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) ubx.checkMessages();
}

bool restorePort() {
  if (!haveSavedPort) return true;
  if (ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_PRT, savedPort.raw,
                             sizeof(savedPort), 1500) != UBX_SEND_SUCCESS)
    return false;
  haveSavedPort = false;
  return true;
}

void fail(const __FlashStringHelper* message) {
  Serial.print(F("FAIL: "));
  Serial.println(message);
  if (haveSavedPort) {
    if (restorePort()) Serial.println(F("Original port configuration restored"));
    else Serial.println(F("RESTORE FAILED: restore DDC settings before continuing"));
  }
  while (true) delay(10);
}
