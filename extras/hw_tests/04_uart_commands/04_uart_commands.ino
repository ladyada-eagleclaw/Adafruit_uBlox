// Nano V3: GPS TX row 17 -> D8, diode-protected RX row 16 -> D7.
// D12 is unused. D7 only pulls low or releases; it never outputs 5 V.
#include <Adafruit_UBX.h>
#include <SoftwareSerial.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error This open-drain UART fixture requires a 16 MHz ATmega328P Nano.
#endif

class OpenDrainUART : public Stream {
 public:
  SoftwareSerial receiver = SoftwareSerial(8, 12);
  void begin();
  void release();
  int available();
  int read();
  int peek();
  void flush();
  size_t write(uint8_t value);
};

OpenDrainUART gpsPort;
Adafruit_UBX ubx(gpsPort);
uint8_t extendedPayload[384];
uint8_t rate[6];
bool gotRate = false, gotVersion = false, gotPVT = false;
const uint8_t MON_VER = 0x04;
void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload);
void receiveFor(uint32_t duration);
void fail(const __FlashStringHelper* message);

void setup() {
  Serial.begin(115200);
  delay(250);

  Serial.println(F("Adafruit SAM-M8Q UART command test"));

  gpsPort.begin();
  ubx.begin();
  ubx.setMessageCallback(receiveUBX);
  Serial.println(F("READY: connect the UART routes, then send s"));
  while (!Serial.available()) delay(1);
  if (Serial.read() != 's') fail(F("Expected s"));

  if (!ubx.sendMessage(UBX_CLASS_CFG, UBX_CFG_RATE, NULL, 0))
    fail(F("Rate poll write failed"));
  receiveFor(2000);
  if (!gotRate) fail(F("No UART CFG-RATE response"));
  Serial.println(F("PASS: UART command reaches RX and reply returns through TX"));

  if (ubx.sendMessageWithAck(UBX_CLASS_CFG, UBX_CFG_RATE, rate, sizeof(rate),
                             2000) != UBX_SEND_SUCCESS)
    fail(F("No matching UART configuration ACK"));
  Serial.println(F("PASS: UART ACK with unchanged rate"));

  if (!ubx.setPayloadBuffer(extendedPayload, sizeof(extendedPayload)))
    fail(F("Could not attach larger payload buffer"));
  if (!ubx.sendMessage(UBX_CLASS_MON, MON_VER, NULL, 0))
    fail(F("Version poll failed"));
  receiveFor(2000);
  if (!gotVersion) fail(F("No extended MON-VER response"));
  Serial.println(F("PASS: caller-owned buffer receives MON-VER beyond 96 bytes"));

  if (!ubx.setPayloadBuffer(NULL, 0)) fail(F("Could not restore internal buffer"));
  if (!ubx.sendMessage(UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0))
    fail(F("NAV-PVT poll failed"));
  receiveFor(2000);
  if (!gotPVT) fail(F("No NAV-PVT after restoring internal buffer"));
  gpsPort.release();
  Serial.println(F("PASS: UART command test complete; RX released, no NVM save"));
}

void loop() { ubx.checkMessages(); }

void OpenDrainUART::begin() {
  receiver.begin(9600);
  pinMode(8, INPUT);
  digitalWrite(8, LOW);
  pinMode(12, INPUT);
  digitalWrite(12, LOW);
  release();
}

void OpenDrainUART::release() {
  pinMode(7, INPUT);
  digitalWrite(7, LOW);
}

int OpenDrainUART::available() { return receiver.available(); }
int OpenDrainUART::read() { return receiver.read(); }
int OpenDrainUART::peek() { return receiver.peek(); }
void OpenDrainUART::flush() {}

size_t OpenDrainUART::write(uint8_t value) {
  // AVR 16 MHz / 9600 baud fixture only. Direction switching gives an
  // open-drain start/data/stop waveform. SoftwareSerial is half duplex.
  uint8_t savedInterrupts = SREG;
  noInterrupts();
  pinMode(7, OUTPUT);
  delayMicroseconds(100);
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (value % 2) pinMode(7, INPUT);
    else pinMode(7, OUTPUT);
    value /= 2;
    delayMicroseconds(100);
  }
  pinMode(7, INPUT);
  SREG = savedInterrupts;
  delayMicroseconds(104);
  return 1;
}

void receiveUBX(uint8_t cls, uint8_t id, uint16_t length, uint8_t* payload) {
  if (cls == UBX_CLASS_CFG && id == UBX_CFG_RATE && length == sizeof(rate)) {
    memcpy(rate, payload, sizeof(rate));
    gotRate = true;
  }
  if (cls == UBX_CLASS_MON && id == MON_VER && length > 96 &&
      (length - 40) % 30 == 0) {
    gotVersion = true;
    Serial.print(F("MON-VER payload bytes: "));
    Serial.println(length);
    // Each extension is a bounded 30-byte string, not necessarily terminated.
    for (uint16_t offset = 40; offset < length; offset += 30) {
      for (uint8_t i = 0; i < 30 && payload[offset + i]; ++i)
        Serial.write(payload[offset + i]);
      Serial.println();
    }
  }
  if (cls == UBX_CLASS_NAV && id == UBX_NAV_PVT && length == 92) gotPVT = true;
}

void receiveFor(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) ubx.checkMessages();
}

void fail(const __FlashStringHelper* message) {
  gpsPort.release();
  Serial.print(F("FAIL: "));
  Serial.println(message);
  while (true) delay(10);
}
