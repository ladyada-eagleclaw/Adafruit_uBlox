// Observe the breakout PPS output without driving it or changing timepulse settings.
// Jumperless row 14 -> Nano D6. No pull-up to the Nano's 5 V rail.
const uint16_t PPS_PIN = 6;

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q PPS input test"));

  pinMode(PPS_PIN, INPUT);
  digitalWrite(PPS_PIN, LOW);
  Serial.println(F("PASS: PPS configured as input without a pull-up"));

  bool previous = digitalRead(PPS_PIN);
  uint32_t start = millis();
  uint32_t lastRise = 0;
  uint8_t goodPeriods = 0;
  uint8_t badPeriods = 0;
  bool haveRise = false;
  while (millis() - start < 15000) {
    bool current = digitalRead(PPS_PIN);
    if (current && !previous) {
      uint32_t now = millis();
      if (haveRise) {
        uint32_t period = now - lastRise;
        if (period >= 900 && period <= 1100) ++goodPeriods;
        else ++badPeriods;
      }
      haveRise = true;
      lastRise = now;
    }
    previous = current;
  }
  if (goodPeriods >= 3 && badPeriods == 0) {
    Serial.println(F("PASS: at least three consecutive 1 Hz PPS periods"));
  } else {
    Serial.println(F("NOT VERIFIED: no stable 1 Hz PPS observed"));
    Serial.println(F("Check wiring, satellite visibility and timepulse configuration."));
  }
  Serial.println(F("This GPIO test does not measure absolute timing accuracy."));
}

void loop() {
  delay(10);
}
