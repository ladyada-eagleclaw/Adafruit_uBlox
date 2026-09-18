// Rev A RESET_N row 48 -> Nano D9 and A0. GPS TX row 17 -> D8.
// D9 is open drain: pull low or release, never enable a 5 V pull-up.
#include <Adafruit_UBX.h>
#include <SoftwareSerial.h>

const uint16_t RESET_PIN = 9;
const uint16_t RESET_SENSE = A0;
SoftwareSerial gpsPort(8, 12);
Adafruit_UBX ubx(gpsPort);
char first[128], second[128];
Adafruit_GNSS gnss(first, second, sizeof(first));
uint32_t sentences = 0;
void receiveNMEA(const nmea_sentence_t& sentence);
void receiveFor(uint32_t duration);
float resetVolts();
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q reset pin test"));

  pinMode(RESET_PIN, INPUT);
  digitalWrite(RESET_PIN, LOW);
  pinMode(RESET_SENSE, INPUT);
  digitalWrite(RESET_SENSE, LOW);
  gpsPort.begin(9600);
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  pinMode(12, INPUT);
  digitalWrite(12, LOW);
  ubx.begin();
  ubx.setNMEAParser(&gnss, receiveNMEA);
  Serial.println(F("READY: connect reset and sense routes, then send s"));
  while (!Serial.available()) delay(1);
  if (Serial.read() != 's') fail(F("Expected s"));

  float volts = resetVolts();
  Serial.print(F("Released RESET_N volts: "));
  Serial.println(volts, 3);
  if (abs(volts - 3.3) > 0.5) fail(F("Reset is not released near 3.3 V"));
  receiveFor(2500);
  if (sentences < 3) fail(F("No navigation before reset"));
  Serial.println(F("PASS: released reset and baseline navigation"));

  pinMode(RESET_PIN, OUTPUT);
  delay(10);
  volts = resetVolts();
  Serial.print(F("Asserted RESET_N volts: "));
  Serial.println(volts, 3);
  if (volts > 0.3) fail(F("Reset did not pull low"));
  receiveFor(500); // Discard bytes queued before assertion.
  sentences = 0;
  receiveFor(1500);
  if (sentences) fail(F("Navigation continued while reset was held low"));
  Serial.println(F("PASS: reset voltage low and navigation stopped"));

  pinMode(RESET_PIN, INPUT);
  sentences = 0;
  receiveFor(7000);
  if (abs(resetVolts() - 3.3) > 0.5) fail(F("Reset did not return high"));
  if (sentences < 3) fail(F("Navigation did not recover after reset"));
  Serial.println(F("PASS: reset released and navigation recovered"));
}

void loop() { ubx.checkMessages(); }

void receiveNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status == NMEA_FRAME_VALID) ++sentences;
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) ubx.checkMessages();
}

float resetVolts() {
  // Nominal Nano 5 V reference: broad limits check the signal state only.
  long millivolts = map(analogRead(RESET_SENSE), 0, 1023, 0, 5000);
  return millivolts / 1000.0;
}

void fail(const __FlashStringHelper* message) {
  pinMode(RESET_PIN, INPUT);
  digitalWrite(RESET_PIN, LOW);
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) delay(10);
}
