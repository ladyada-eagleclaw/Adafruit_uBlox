// Battery retention test with a real main-power interruption; Nano stays on USB.
// Requires a CR1220 and external confirmation that VIN AND 3.3 V fall near zero.
// The host must isolate all signal routes before removing breakout VIN.
// This saves navigation settings to battery-backed RAM, never flash/EEPROM.
// Keep this serial session open: restarting the Nano loses the saved rate.
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
Rate receivedRate, originalRate, markerRate;
bool gotRate = false, needsRestore = false;
// UBX-CFG-CFG: saveMask navConf bit 3; optional deviceMask devBBR bit 0.
// R28 protocol specification section 32.10.3. Clear/load masks stay zero.
const uint8_t saveNavigationBBR[13] = {
  0, 0, 0, 0, 0x08, 0, 0, 0, 0, 0, 0, 0, 0x01
};
void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload);
bool pollRate();
bool writeRate(const Rate& rate);
bool saveBBR();
bool restore();
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q battery retention test"));
  Serial.println(F("WARNING: navigation settings will be saved to battery RAM only."));
  Serial.println(F("Keep Nano USB and this session connected until restoration."));

  if (!ddc.begin()) fail(F("DDC begin failed"));
  ubx.begin();
  ubx.setMessageCallback(receiveUBX);
  if (!pollRate()) fail(F("Could not read original rate"));
  originalRate = receivedRate;
  markerRate = originalRate;
  markerRate.fields.measurementMilliseconds = 1500;
  if (originalRate.fields.measurementMilliseconds == 1500)
    markerRate.fields.measurementMilliseconds = 1000;
  Serial.print(F("Original measurement interval ms: "));
  Serial.println(originalRate.fields.measurementMilliseconds);
  Serial.print(F("Original navigation cycles: "));
  Serial.println(originalRate.fields.navigationCycles);
  Serial.print(F("Original time reference: "));
  Serial.println(originalRate.fields.timeReference);
  Serial.println(F("READY: battery and power-isolation setup checked; send s"));
  while (!Serial.available()) delay(1);
  if (Serial.read() != 's') fail(F("Expected s"));

  needsRestore = true;
  if (!writeRate(markerRate)) fail(F("Marker rate write failed"));
  if (!saveBBR()) fail(F("Marker BBR save failed"));
  if (!pollRate() || memcmp(receivedRate.raw, markerRate.raw, sizeof(Rate)))
    fail(F("Marker readback failed"));
  Serial.println(F("ARMED: marker saved; isolate signals, remove VIN for 20 seconds."));
  Serial.println(F("Verify VIN and 3.3 V are near zero, then restore power/routes."));
  Serial.println(F("Send v only after that verification, or a to abort and restore."));

  uint32_t start = millis();
  while (!Serial.available() && millis() - start < 300000) delay(10);
  if (!Serial.available()) fail(F("Power-cycle verification timed out"));
  char result = Serial.read();
  if (result != 'v') fail(F("Power-cycle test aborted"));

  if (!ddc.begin()) fail(F("DDC did not return after power cycle"));
  if (!pollRate()) fail(F("No rate response after power cycle"));
  if (memcmp(receivedRate.raw, markerRate.raw, sizeof(Rate)))
    fail(F("Battery-backed marker was not retained"));
  Serial.println(F("PASS: nondefault rate survived confirmed main-power removal"));
  if (!restore()) fail(F("Could not restore original rate and BBR"));
  if (!pollRate() || memcmp(receivedRate.raw, originalRate.raw, sizeof(Rate)))
    fail(F("Restored rate readback failed"));
  Serial.println(F("PASS: original rate restored in runtime and battery RAM"));
}

void loop() { ubx.checkMessages(); }

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload) {
  if (cls == UBX_CLASS_CFG && id == UBX_CFG_RATE && length == sizeof(Rate)) {
    memcpy(receivedRate.raw, payload, sizeof(Rate));
    gotRate = true;
  }
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

bool saveBBR() {
  return ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_CFG, saveNavigationBBR,
                                sizeof(saveNavigationBBR), 2000) == UBX_SEND_SUCCESS;
}

bool restore() {
  if (!needsRestore) return true;
  if (!writeRate(originalRate)) return false;
  if (!saveBBR()) return false;
  needsRestore = false;
  return true;
}

void fail(const __FlashStringHelper* message) {
  Serial.print(F("FAIL: "));
  Serial.println(message);
  if (needsRestore) {
    if (restore()) Serial.println(F("Original rate restored in runtime and BBR"));
    else Serial.println(F("RESTORE FAILED: reconnect power/DDC and restore original rate"));
  }
  while (true) delay(10);
}
