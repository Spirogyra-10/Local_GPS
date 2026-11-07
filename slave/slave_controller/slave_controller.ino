/**
 * @file slave_controller.ino
 * @brief UWB Local GPS - 3連Anchor (Slave 1, 2, 3) 統合制御プログラム
 * @details 
 * Arduino Uno R3 に接続された3基の RYUW122_Lite (Anchorモード) を
 * SoftwareSerial を用いて時分割順次ポーリングし、
 * ロボット側 Tag (Master) との距離を取得して PC へシリアル出力します。
 * 
 * 【自動リセット機能】
 * - 起動時に Arduino D8 から全モジュールの Pin 2 (NRST) へリセットパルスを送信し自動起動
 * 
 * 【デバッグ機能】
 * - PCから 'd' または 'D' を送信すると、詳細デバッグログ ⇔ CSV出力 をリアルタイム切替可能
 * - 各Slaveの送受信ログ、生レスポンス、タイムアウト状態を可視化
 */

#include <SoftwareSerial.h>

// ==========================================
// デバッグ設定
// ==========================================
// 起動時のデバッグモード (false: 本番CSV出力 'DIST,d1,d2,d3', true: 詳細ログ出力)
// ※ 実行中にシリアルモニタから 'd' を送るといつでも切替可能です
bool isDebugMode = false;

// ==========================================
// ピン配置設定 (Arduino Uno R3)
// ==========================================
// 全モジュール共通ハードウェアリセットピン
const uint8_t UWB_RESET_PIN = 8; // to All Slaves Pin 2 (NRST)

// Slave 1: 原点 (0, 0)
const uint8_t SLAVE1_RX_PIN = 2; // to Slave1 Pin 4 (TXD)
const uint8_t SLAVE1_TX_PIN = 3; // to Slave1 Pin 3 (RXD)

// Slave 2: Y軸 (0, 1.0)
const uint8_t SLAVE2_RX_PIN = 4; // to Slave2 Pin 4 (TXD)
const uint8_t SLAVE2_TX_PIN = 5; // to Slave2 Pin 3 (RXD)

// Slave 3: X軸 (1.0, 0)
const uint8_t SLAVE3_RX_PIN = 6; // to Slave3 Pin 4 (TXD)
const uint8_t SLAVE3_TX_PIN = 7; // to Slave3 Pin 3 (RXD)

const uint8_t HEARTBEAT_LED = 13; // 動作確認用内蔵LED

// ==========================================
// 通信設定
// ==========================================
// PCとの通信速度 (ハードウェアシリアル)
const unsigned long PC_BAUDRATE = 115200;

// RYUW122との通信速度 (SoftwareSerialのため 9600bps)
const unsigned long UWB_BAUDRATE = 9600;

// 測距対象の Tag (Master) アドレス
const char TARGET_TAG_ADDR[] = "TAG1";

// 測距コマンドのタイムアウト時間 [ms] (標準応答時間は約60-100ms)
const unsigned long RANGING_TIMEOUT_MS = 200;

// ポート切り替え後の安定化ウェイト [ms]
const unsigned long SWITCH_DELAY_MS = 15;

// 測定値の上限 [cm]
const long MAX_VALID_DISTANCE_CM = 3000;

// ==========================================
// SoftwareSerial インスタンス定義
// ==========================================
SoftwareSerial uwbSerial1(SLAVE1_RX_PIN, SLAVE1_TX_PIN);
SoftwareSerial uwbSerial2(SLAVE2_RX_PIN, SLAVE2_TX_PIN);
SoftwareSerial uwbSerial3(SLAVE3_RX_PIN, SLAVE3_TX_PIN);

// ==========================================
// プロトタイプ宣言
// ==========================================
long measureDistance(SoftwareSerial &serial, const char *anchorTag);
void clearSerialBuffer(SoftwareSerial &serial);
void checkDebugCommand();
void resetAllUwbModules();

void setup() {
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  // PCシリアル初期化
  Serial.begin(PC_BAUDRATE);
  while (!Serial) {
    ; // シリアル接続待機
  }

  Serial.println(F("# =================================================="));
  Serial.println(F("# SYSTEM_START: Local GPS Slave Controller v2.0    "));
  Serial.println(F("# Hardware Auto-Reset enabled on Pin D8             "));
  Serial.println(F("# Send 'd' or 'D' via Serial to TOGGLE DEBUG LOGS  "));
  Serial.println(F("# =================================================="));

  // 1. 全UWBモジュールのハードウェア一括リセット
  resetAllUwbModules();

  // 2. 各UWB SoftwareSerial初期化
  uwbSerial1.begin(UWB_BAUDRATE);
  uwbSerial2.begin(UWB_BAUDRATE);
  uwbSerial3.begin(UWB_BAUDRATE);

  clearSerialBuffer(uwbSerial1);
  clearSerialBuffer(uwbSerial2);
  clearSerialBuffer(uwbSerial3);

  Serial.println(F("# [READY] All 3 Anchors Initialized & Active."));
  delay(100);
}

void loop() {
  // PCからのデバッグ切替コマンド確認
  checkDebugCommand();

  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(HEARTBEAT_LED, ledState ? HIGH : LOW);

  // 1. Slave 1 (原点: 0, 0)
  long d1 = measureDistance(uwbSerial1, "S1");

  // 2. Slave 2 (Y軸: 0, 1.0)
  long d2 = measureDistance(uwbSerial2, "S2");

  // 3. Slave 3 (X軸: 1.0, 0)
  long d3 = measureDistance(uwbSerial3, "S3");

  // データ出力
  if (isDebugMode) {
    Serial.print(F("[SUMMARY] S1: "));
    Serial.print(d1);
    Serial.print(F(" cm | S2: "));
    Serial.print(d2);
    Serial.print(F(" cm | S3: "));
    Serial.print(d3);
    Serial.println(F(" cm"));
    Serial.println(F("--------------------------------------------------"));
  } else {
    // 本番CSV出力フォーマット: DIST,d1,d2,d3
    Serial.print(F("DIST,"));
    Serial.print(d1);
    Serial.print(F(","));
    Serial.print(d2);
    Serial.print(F(","));
    Serial.println(d3);
  }
}

/**
 * @brief 全UWBモジュールにハードウェアリセットパルスを印加して初期化
 */
void resetAllUwbModules() {
  Serial.println(F("# [INIT] Resetting UWB Anchors via D8..."));
  pinMode(UWB_RESET_PIN, OUTPUT);
  digitalWrite(UWB_RESET_PIN, LOW);   // Active-LOW reset
  delay(30);                           // 30msパルス
  digitalWrite(UWB_RESET_PIN, HIGH);
  pinMode(UWB_RESET_PIN, INPUT_PULLUP);// オープンドレイン/プルアップ解放
  delay(600);                          // モジュール起動・+READY出力待ち
  Serial.println(F("# [INIT] UWB Anchors boot sequence complete."));
}

/**
 * @brief PCからのデバッグ切り替えキー ('d' / 'D') をチェック
 */
void checkDebugCommand() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      isDebugMode = !isDebugMode;
      Serial.print(F("# [MODE CHANGED] Debug Mode is now: "));
      Serial.println(isDebugMode ? F("ON (Verbose Logs)") : F("OFF (CSV Output)"));
    }
  }
}

/**
 * @brief 指定したSoftwareSerialポートに対して測距コマンドを送信し、距離を取得する
 * 
 * @param serial 対象の SoftwareSerial インスタンス
 * @param anchorTag アンカーの識別名 (例: "S1")
 * @return long 測定された距離 [cm] (失敗・タイムアウト時は -1)
 */
long measureDistance(SoftwareSerial &serial, const char *anchorTag) {
  // SoftwareSerial で複数ポートを扱う場合、受信対象を明示的に切り替える
  serial.listen();
  delay(SWITCH_DELAY_MS);
  clearSerialBuffer(serial);

  if (isDebugMode) {
    Serial.print(F("["));
    Serial.print(anchorTag);
    Serial.print(F("] TX: AT+ANCHOR_SEND="));
    Serial.print(TARGET_TAG_ADDR);
    Serial.println(F(",4,PING"));
  }

  // 測距要求コマンド送信
  serial.print(F("AT+ANCHOR_SEND="));
  serial.print(TARGET_TAG_ADDR);
  serial.print(F(",4,PING\r\n"));

  unsigned long startTime = millis();
  char rxBuf[64];
  uint8_t bufIdx = 0;

  while (millis() - startTime < RANGING_TIMEOUT_MS) {
    while (serial.available() > 0) {
      char c = serial.read();
      if (c == '\n') {
        rxBuf[bufIdx] = '\0';

        if (isDebugMode) {
          Serial.print(F("["));
          Serial.print(anchorTag);
          Serial.print(F("] RX: "));
          Serial.println(rxBuf);
        }

        // 成功レスポンス: +ANCHOR_RCV=<Tag_Address>,<Length>,<Data>,<Distance_cm> cm
        if (strncmp(rxBuf, "+ANCHOR_RCV=", 12) == 0) {
          char *lastComma = strrchr(rxBuf, ',');
          if (lastComma != NULL) {
            long dist = atol(lastComma + 1);
            if (dist > 0 && dist <= MAX_VALID_DISTANCE_CM) {
              return dist;
            }
          }
        } else if (strncmp(rxBuf, "+ERR", 4) == 0) {
          if (isDebugMode) {
            Serial.print(F("["));
            Serial.print(anchorTag);
            Serial.println(F("] -> Received ERROR response"));
          }
          return -1;
        }

        bufIdx = 0;
      } else if (c != '\r') {
        if (bufIdx < sizeof(rxBuf) - 1) {
          rxBuf[bufIdx++] = c;
        }
      }
    }
  }

  if (isDebugMode) {
    Serial.print(F("["));
    Serial.print(anchorTag);
    Serial.println(F("] -> TIMEOUT (No response from module or tag)"));
  }

  return -1;
}

/**
 * @brief シリアルバッファに残っているゴミデータを破棄
 */
void clearSerialBuffer(SoftwareSerial &serial) {
  while (serial.available() > 0) {
    serial.read();
  }
}
