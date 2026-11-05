/**
 * @file slave_config_tool.ino
 * @brief RYUW122 モジュール初期設定・ATコマンド送受信用ツール
 * @details 
 * PCのシリアルモニタから入力したATコマンドをそのまま指定ピンのRYUW122へ転送し、
 * モジュールからの応答をPCへ返します。
 * モジュールのボーレート変更 (AT+IPR=9600)、モード設定 (AT+MODE=1)、
 * アドレス設定 (AT+ADDRESS=SLAVE1 等) に使用します。
 */

#include <SoftwareSerial.h>

// テスト・設定したいモジュールの接続ピンを指定
const uint8_t TARGET_RX_PIN = 2; // Arduino RX (RYUW122 TXD に接続)
const uint8_t TARGET_TX_PIN = 3; // Arduino TX (RYUW122 RXD に分圧接続)

// PC通信速度
const unsigned long PC_BAUDRATE = 115200;

// RYUW122の現在の通信速度
// ※ 出荷時初期状態は 115200 です。9600に変更済みの場合は 9600 に設定してください。
const unsigned long UWB_CURRENT_BAUDRATE = 115200;

SoftwareSerial uwbSerial(TARGET_RX_PIN, TARGET_TX_PIN);

void setup() {
  Serial.begin(PC_BAUDRATE);
  while (!Serial) {
    ;
  }
  
  uwbSerial.begin(UWB_CURRENT_BAUDRATE);

  Serial.println(F("========================================"));
  Serial.println(F("RYUW122 AT Command Config Tool"));
  Serial.println(F("========================================"));
  Serial.println(F("Send AT commands via Serial Monitor (CR+LF required)."));
  Serial.println(F("Typical setup commands for Anchor:"));
  Serial.println(F("  1. Check connection: AT"));
  Serial.println(F("  2. Set Anchor mode:  AT+MODE=1"));
  Serial.println(F("  3. Set Network ID:   AT+NETWORKID=LOCALGPS"));
  Serial.println(F("  4. Set Address:      AT+ADDRESS=SLAVE1"));
  Serial.println(F("  5. Change Baudrate:  AT+IPR=9600"));
  Serial.println(F("========================================"));
}

void loop() {
  // PCからの入力をUWBモジュールへ
  if (Serial.available()) {
    char c = Serial.read();
    uwbSerial.write(c);
  }

  // UWBモジュールからの応答をPCへ
  if (uwbSerial.available()) {
    char c = uwbSerial.read();
    Serial.write(c);
  }
}

