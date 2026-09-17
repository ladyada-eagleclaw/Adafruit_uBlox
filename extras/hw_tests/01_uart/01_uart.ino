// Receive-only UART test. GPS TX row 17 -> D8; D7 stays physically disconnected.
#include <Adafruit_UBX.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsPort(8, 7);
Adafruit_UBX ubx(gpsPort);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
uint32_t validLines = 0, invalidLines = 0, positions = 0;
void receiveNMEA(const nmea_sentence_t& sentence);
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q UART reception test"));

  gpsPort.begin(9600);
  // SoftwareSerial enables an input pull-up: disable it for the 3.3 V GPS TX.
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  ubx.begin();
  ubx.setNMEAParser(&gnss, receiveNMEA);
  Serial.println(F("PASS: receive-only UART initialized"));

  uint32_t start = millis();
  while (millis() - start < 15000) ubx.checkMessages();
  if (validLines < 10 || positions < 3) fail(F("Insufficient navigation traffic"));
  if (invalidLines) fail(F("Invalid NMEA frames received"));
  if (gpsPort.overflow()) fail(F("SoftwareSerial receive overflow"));
  Serial.println(F("PASS: SAM-M8Q UART framing and GNSS decoding"));
}

void loop() {
  ubx.checkMessages();
  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 5000) {
    lastReport = millis();
    Serial.print(F("Valid NMEA: "));
    Serial.print(validLines);
    Serial.print(F(", decoded positions: "));
    Serial.print(positions);
    Serial.print(F(", invalid: "));
    Serial.println(invalidLines);
  }
}

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status != NMEA_FRAME_VALID) {
    ++invalidLines;
    return;
  }
  ++validLines;
  if (Adafruit_GNSS::parsePosition(sentence).validation.status == GNSS_SENTENCE_VALID)
    ++positions;
}

void fail(const __FlashStringHelper* message) {
  gpsPort.end();
  pinMode(7, INPUT);
  digitalWrite(7, LOW);
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) delay(10);
}
