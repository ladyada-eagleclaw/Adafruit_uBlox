// SAM-M8Q protocol 18+: request temporary backup, then wake through EXTINT0.
// GPIO_1 (3.3 V Jumperless output) -> row 18 -> Nano A1, input-only sense.
// DDC A4/A5 sends the command. GPS TX -> D8 observes sleep/wake without polling.
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>
#include <SoftwareSerial.h>

const uint16_t WAKE_SENSE = A1;
const uint8_t RXM_PMREQ = 0x41;
// UBX-13003221 R28 section 32.18.3.2, little-endian AVR wire layout.
union {
  struct {
    uint8_t version;
    uint8_t reserved[3];
    uint32_t duration;
    uint32_t flags;
    uint32_t wakeupSources;
  } fields;
  uint8_t raw[16];
} request = {};
static_assert(sizeof(request) == 16, "PMREQ must be 16 bytes");

Adafruit_UBloxDDC ddc;
Adafruit_UBX commands(ddc);
SoftwareSerial gpsPort(8, 12);
Adafruit_UBX receiver(gpsPort);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
uint32_t sentences = 0;
void receiveNMEA(const nmea_sentence_t& sentence);
void receiveFor(uint32_t duration);
float wakeVolts();
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q EXTINT0 wake test"));

  pinMode(WAKE_SENSE, INPUT);
  digitalWrite(WAKE_SENSE, LOW);
  gpsPort.begin(9600);
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  pinMode(12, INPUT);
  digitalWrite(12, LOW);
  receiver.begin();
  receiver.setNMEAParser(&gnss, receiveNMEA);
  Serial.println(F("READY: connect 3.3 V GPIO_1 LOW and A1 to row 18; send s"));
  while (!Serial.available()) delay(1);
  if (Serial.read() != 's') fail(F("Expected s"));
  Serial.print(F("EXTINT0 low volts: "));
  Serial.println(wakeVolts(), 3);
  if (wakeVolts() > 0.3) fail(F("EXTINT0 is not low"));
  if (!ddc.begin()) fail(F("DDC begin failed"));
  commands.begin();
  receiveFor(2500);
  if (sentences < 3) fail(F("No baseline navigation"));
  Serial.println(F("PASS: EXTINT0 low and navigation active"));

  // Backup flag bit 1 and EXTINT0 wake bit 5. No other pin wake sources.
  // A finite duration guarantees recovery even if the test host disappears.
  request.fields.duration = 30000;
  request.fields.flags = 0x02;
  request.fields.wakeupSources = 0x20;
  if (!commands.sendMessage(UBX_CLASS_RXM, RXM_PMREQ, request.raw, sizeof(request)))
    fail(F("Backup request write failed"));
  uint32_t sleepStart = millis();
  receiveFor(1500); // Drain navigation sent before the backup request.
  sentences = 0;
  receiveFor(3000);
  if (sentences) fail(F("Navigation did not stop during backup"));
  Serial.println(F("PASS: navigation stopped during temporary backup"));

  sentences = 0;
  Serial.println(F("WAKE NOW: set Jumperless GPIO_1 HIGH (3.3 V)"));
  uint32_t wakeStart = millis();
  while (millis() - wakeStart < 10000 && sentences < 3) receiver.checkMessages();
  Serial.print(F("EXTINT0 high volts: "));
  Serial.println(wakeVolts(), 3);
  if (abs(wakeVolts() - 3.3) > 0.5) fail(F("No 3.3 V wake edge observed"));
  if (sentences < 3) fail(F("Navigation did not resume after EXTINT0 edge"));
  if (millis() - sleepStart >= 25000) fail(F("Cannot distinguish wake from timer"));
  Serial.println(F("PASS: EXTINT0 woke navigation before the 30-second timer"));
  Serial.println(F("Release the Jumperless control route; no NVM save"));
}

void loop() { receiver.checkMessages(); }

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status == NMEA_FRAME_VALID) ++sentences;
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) receiver.checkMessages();
}

float wakeVolts() {
  long millivolts = map(analogRead(WAKE_SENSE), 0, 1023, 0, 5000);
  return millivolts / 1000.0;
}

void fail(const __FlashStringHelper* message) {
  Serial.print(F("FAIL: "));
  Serial.println(message);
  Serial.println(F("Set GPIO_1 to input; backup expires within 30 seconds"));
  while (true) delay(10);
}
