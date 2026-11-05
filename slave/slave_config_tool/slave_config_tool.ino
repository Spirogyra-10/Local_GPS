/**
 * @file slave_config_tool.ino
 * @brief RYUW122 モジュール初期設定・ATコマンド送受信用高機能ツール
 * @details 
 * Arduino Uno R3 に接続された 3 基の RYUW122_Lite を配線を差し替えることなく、
 * シリアルモニタからのコマンドで切り替えて設定・疎通確認できます。
 * 
 * 【主な機能】
 * - 対象ポートの動的切替 (コマンド: #1, #2, #3)
 * - ワンタッチ 9600bps 変更機能 (コマンド: #SET9600)
 * - AT コマンドの直接パススルー
 */

#include <SoftwareSerial.h>

// 各スレーブのピン配置
const uint8_t SLAVE_PINS[3][2] = {
  {2, 3}, // Slave 1: RX=D2, TX=D3
  {4, 5}, // Slave 2: RX=D4, TX=D5
  {6, 7}  // Slave 3: RX=D6, TX=D7
};

SoftwareSerial uwb1(SLAVE_PINS[0][0], SLAVE_PINS[0][1]);
SoftwareSerial uwb2(SLAVE_PINS[1][0], SLAVE_PINS[1][1]);
SoftwareSerial uwb3(SLAVE_PINS[2][0], SLAVE_PINS[2][1]);

SoftwareSerial *activeSerial = &uwb1;
uint8_t activePort = 1;
unsigned long currentBaud = 9600; // 初期ボーレート (未設定モジュールは115200)

void printHelp();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // デフォルト 9600bps で初期化
  uwb1.begin(currentBaud);
  uwb2.begin(currentBaud);
  uwb3.begin(currentBaud);

  activeSerial->listen();

  printHelp();
}

void loop() {
  // PCからの入力処理
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("#1")) {
      activePort = 1;
      activeSerial = &uwb1;
      activeSerial->listen();
      Serial.println(F("[Tool] Switched target to SLAVE 1 (Pins: RX=D2, TX=D3)"));
    } else if (input.startsWith("#2")) {
      activePort = 2;
      activeSerial = &uwb2;
      activeSerial->listen();
      Serial.println(F("[Tool] Switched target to SLAVE 2 (Pins: RX=D4, TX=D5)"));
    } else if (input.startsWith("#3")) {
      activePort = 3;
      activeSerial = &uwb3;
      activeSerial->listen();
      Serial.println(F("[Tool] Switched target to SLAVE 3 (Pins: RX=D6, TX=D7)"));
    } else if (input.startsWith("#BAUD=")) {
      currentBaud = input.substring(6).toInt();
      activeSerial->begin(currentBaud);
      activeSerial->listen();
      Serial.print(F("[Tool] Arduino SoftwareSerial baudrate set to: "));
      Serial.println(currentBaud);
    } else if (input.equalsIgnoreCase("#SET9600")) {
      // 新品モジュール(115200)を 9600 に強制変更するマクロ
      Serial.println(F("[Tool] Attempting to set module baudrate to 9600..."));
      activeSerial->begin(115200);
      delay(50);
      activeSerial->print(F("AT+IPR=9600\r\n"));
      delay(100);
      activeSerial->begin(9600);
      activeSerial->listen();
      delay(50);
      activeSerial->print(F("AT\r\n"));
      Serial.println(F("[Tool] Sent 'AT+IPR=9600' and re-initialized Arduino port to 9600."));
    } else if (input.equalsIgnoreCase("#HELP")) {
      printHelp();
    } else if (input.length() > 0) {
      // 通常のATコマンドをモジュールへ送信
      activeSerial->print(input);
      activeSerial->print(F("\r\n"));
    }
  }

  // UWBモジュールからの応答をPCへ
  if (activeSerial->available()) {
    char c = activeSerial->read();
    Serial.write(c);
  }
}

void printHelp() {
  Serial.println(F("=================================================="));
  Serial.println(F("  RYUW122 Multi-Anchor Config Tool v1.1           "));
  Serial.println(F("=================================================="));
  Serial.println(F("Special Commands:"));
  Serial.println(F("  #1 / #2 / #3   : Switch active Slave (Default: #1)"));
  Serial.println(F("  #SET9600       : Auto-configure module from 115200 to 9600"));
  Serial.println(F("  #BAUD=<rate>   : Change SoftwareSerial baudrate"));
  Serial.println(F("  #HELP          : Show this help"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("Setup Sequence Example for Anchor:"));
  Serial.println(F("  AT                 (Check connection)"));
  Serial.println(F("  AT+MODE=1          (Set Anchor mode)"));
  Serial.println(F("  AT+NETWORKID=LGPS  (Set Network ID)"));
  Serial.println(F("  AT+ADDRESS=SLAVE1  (Set Device Address)"));
  Serial.println(F("=================================================="));
}
