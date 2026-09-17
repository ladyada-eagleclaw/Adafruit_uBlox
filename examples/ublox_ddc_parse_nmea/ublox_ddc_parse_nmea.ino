/*!
 * @file ublox_ddc_parse_nmea.ino
 * Shared NMEA/GNSS decoding over u-blox DDC (I2C).
 * Written by Brent Rubell and Limor Fried for Adafruit Industries.
 * MIT license; retain this notice in redistributions.
 */
#include <Adafruit_UBX.h>
#include <Adafruit_UBloxDDC.h>

Adafruit_UBloxDDC ddc;
Adafruit_UBX ubx(ddc);
char firstSentence[128];
char secondSentence[128];
Adafruit_GNSS gnss(firstSentence, secondSentence, sizeof(firstSentence));
uint32_t lastPrint = 0;

void onNMEA(const nmea_sentence_t& sentence);

void setup() {
  Serial.begin(115200);
  // Native USB boards wait for the monitor; remove this for standalone use.
  while (!Serial) delay(10);
  delay(250);

  Serial.println(F("Adafruit uBlox shared GNSS example"));

  if (!ddc.begin()) {
    Serial.println(F("Could not find u-blox DDC at 0x42"));
    while (true) delay(10);
  }
  ubx.begin();
  ubx.setNMEAParser(&gnss, onNMEA);
  Serial.println(F("Waiting for the receiver's existing NMEA output"));
}

void loop() {
  // One reader handles UBX and NMEA, also during sendMessageWithAck().
  ubx.checkMessages();
}

void onNMEA(const nmea_sentence_t& sentence) {
  if (sentence.status != NMEA_FRAME_VALID) return;
  // A complete sentence is decoded independently; old and new fixes are not mixed.
  gnss_position_t position = Adafruit_GNSS::parsePosition(sentence);
  if (position.validation.status != GNSS_SENTENCE_VALID) return;
  if (millis() - lastPrint < 2000) return;
  lastPrint = millis();

  if (!position.fix) {
    Serial.println(F("Receiving GNSS data; waiting for a position fix"));
    return;
  }
  if (position.latitude.status != NMEA_NUMBER_VALID ||
      position.longitude.status != NMEA_NUMBER_VALID) return;

  // Format the exact coordinate components without an intermediate float.
  char coordinate[GNSS_COORDINATE_TEXT_SIZE];
  Adafruit_GNSS::formatCoordinate(coordinate, sizeof(coordinate), position.latitude);
  Serial.print(F("Latitude: "));
  Serial.print(coordinate);
  Adafruit_GNSS::formatCoordinate(coordinate, sizeof(coordinate), position.longitude);
  Serial.print(F(", Longitude: "));
  Serial.println(coordinate);
}
