/**
 * @file slave_diagnostics.ino
 * @brief UWB モジュール配線・ボーレート自動診断ツール
 * @details 
 * Slave 1, 2, 3 の各ポートに対して、複数のボーレート (115200, 9600, 57600) で
 * "AT\r\n" を送信し、モジュールから応答 (生バイト) が返ってくるかを徹底検査します。
 */

#include <SoftwareSerial.h>

const uint8_t PINS[3][2] = {
  {2, 3}, // Slave 1: RX=D2, TX=D3
  {4, 5}, // Slave 2: RX=D4, TX=D5
  {6, 7}  // Slave 3: RX=D6, TX=D7
};

const unsigned long TEST_BAUDS[] = {9600, 115200, 57600};
const uint8_t NUM_BAUDS = sizeof(TEST_BAUDS) / sizeof(TEST_BAUDS[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(1000);

  Serial.println(F("=================================================="));
  Serial.println(F("  RYUW122 Hardware & Wiring Diagnostic Tool       "));
  Serial.println(F("=================================================="));
  Serial.println(F("Testing each Slave port across multiple baudrates..."));
  Serial.println(F("--------------------------------------------------"));

  for (uint8_t s = 0; s < 3; s++) {
    uint8_t rxPin = PINS[s][0];
    uint8_t txPin = PINS[s][1];
    Serial.print(F("\n>>> Testing SLAVE "));
    Serial.print(s + 1);
    Serial.print(F(" (Arduino RX=D"));
    Serial.print(rxPin);
    Serial.print(F(", TX=D"));
    Serial.print(txPin);
    Serial.println(F(") <<<"));

    bool responded = false;

    for (uint8_t b = 0; b < NUM_BAUDS; b++) {
      unsigned long baud = TEST_BAUDS[b];
      Serial.print(F("  Testing Baudrate: "));
      Serial.print(baud);
      Serial.print(F(" bps ... "));

      SoftwareSerial testSerial(rxPin, txPin);
      testSerial.begin(baud);
      testSerial.listen();
      delay(50);

      // バッファクリア
      while (testSerial.available() > 0) { testSerial.read(); }

      // "AT\r\n" 送信
      testSerial.print(F("AT\r\n"));

      unsigned long start = millis();
      String reply = "";
      while (millis() - start < 300) {
        if (testSerial.available()) {
          char c = testSerial.read();
          reply += c;
        }
      }

      if (reply.length() > 0) {
        Serial.print(F("[RESPONSE DETECTED!] Length="));
        Serial.print(reply.length());
        Serial.print(F(", RAW bytes: "));
        for (unsigned int i = 0; i < reply.length(); i++) {
          char c = reply[i];
          if (c >= 32 && c <= 126) {
            Serial.print(c);
          } else {
            Serial.print(F("<0x"));
            Serial.print((uint8_t)c, HEX);
            Serial.print(F(">"));
          }
        }
        Serial.println();
        responded = true;
        break; // このスレーブは検出成功
      } else {
        Serial.println(F("[NO RESPONSE]"));
      }
    }

    if (!responded) {
      Serial.print(F("  [!] SLAVE "));
      Serial.print(s + 1);
      Serial.println(F(" FAILED: No data received at any baudrate."));
      Serial.println(F("      Check: 1. Power (VDD 3.3V, GND)"));
      Serial.println(F("             2. Reverse TX/RX wiring"));
      Serial.println(F("             3. Breadboard connection"));
    }
  }

  Serial.println(F("\n=================================================="));
  Serial.println(F("Diagnostic complete."));
  Serial.println(F("=================================================="));
}

void loop() {
  // 終了後は何もしない
  delay(1000);
}

