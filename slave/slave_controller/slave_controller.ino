/**
 * @file slave_controller.ino
 * @brief UWB Local GPS - 3連Anchor (Slave 1, 2, 3) 統合制御プログラム
 * @details 
 * Arduino Uno R3 に接続された3基の RYUW122_Lite (Anchorモード) を
 * SoftwareSerial を用いて時分割順次ポーリングし、
 * ロボット側 Tag (Master) との距離を取得して PC へシリアル出力します。
 */

#include <SoftwareSerial.h>

// ==========================================
// ピン配置設定 (Arduino Uno R3)
// ==========================================
// ※ Arduino RX は RYUW122 TXD に直結可能 (3.3V -> 5V入力OK)
// ※ Arduino TX は RYUW122 RXD に接続時、必ず分圧(1kΩ/2kΩ等)で3.3Vに降圧してください。
const uint8_t SLAVE1_RX_PIN = 2; // to Slave1 TXD
const uint8_t SLAVE1_TX_PIN = 3; // to Slave1 RXD (分圧後)

const uint8_t SLAVE2_RX_PIN = 4; // to Slave2 TXD
const uint8_t SLAVE2_TX_PIN = 5; // to Slave2 RXD (分圧後)

const uint8_t SLAVE3_RX_PIN = 6; // to Slave3 TXD
const uint8_t SLAVE3_TX_PIN = 7; // to Slave3 RXD (分圧後)

// ==========================================
// 通信設定
// ==========================================
// PCとの通信速度 (ハードウェアシリアル)
const unsigned long PC_BAUDRATE = 115200;

// RYUW122との通信速度 (SoftwareSerialのため 9600bps 推奨)
const unsigned long UWB_BAUDRATE = 9600;

// 測距対象の Tag (Master) アドレス
const char TARGET_TAG_ADDR[] = "TAG1";

// 測距コマンドのタイムアウト時間 [ms]
const unsigned long RANGING_TIMEOUT_MS = 250;

// ポーリング間隔 [ms]
const unsigned long CYCLE_DELAY_MS = 20;

// ==========================================
// SoftwareSerial インスタンス定義
// ==========================================
SoftwareSerial uwbSerial1(SLAVE1_RX_PIN, SLAVE1_TX_PIN);
SoftwareSerial uwbSerial2(SLAVE2_RX_PIN, SLAVE2_TX_PIN);
SoftwareSerial uwbSerial3(SLAVE3_RX_PIN, SLAVE3_TX_PIN);

// ==========================================
// プロトタイプ宣言
// ==========================================
long measureDistance(SoftwareSerial &serial, const char *anchorName);
void clearSerialBuffer(SoftwareSerial &serial);

void setup() {
  // PCシリアル初期化
  Serial.begin(PC_BAUDRATE);
  while (!Serial) {
    ; // シリアル接続待機 (Leonardo等の互換用)
  }

  // 各UWB通信初期化
  uwbSerial1.begin(UWB_BAUDRATE);
  uwbSerial2.begin(UWB_BAUDRATE);
  uwbSerial3.begin(UWB_BAUDRATE);

  Serial.println(F("# SYSTEM_START: Local GPS Slave Controller Initialized"));
  Serial.println(F("# Output format: DIST,d1_cm,d2_cm,d3_cm (d=-1 on error/timeout)"));
  delay(500);
}

void loop() {
  // 1. Slave 1 (原点: 0, 0)
  long d1 = measureDistance(uwbSerial1, "SLAVE1");
  delay(CYCLE_DELAY_MS);

  // 2. Slave 2 (Y軸: 0, 1.0)
  long d2 = measureDistance(uwbSerial2, "SLAVE2");
  delay(CYCLE_DELAY_MS);

  // 3. Slave 3 (X軸: 1.0, 0)
  long d3 = measureDistance(uwbSerial3, "SLAVE3");
  delay(CYCLE_DELAY_MS);

  // PCへデータ出力 (フォーマット: DIST,d1,d2,d3)
  // 例: DIST,120,85,150  (単位: cm, 失敗時は -1)
  Serial.print(F("DIST,"));
  Serial.print(d1);
  Serial.print(F(","));
  Serial.print(d2);
  Serial.print(F(","));
  Serial.println(d3);
}

/**
 * @brief 指定したSoftwareSerialポートに対して測距コマンドを送信し、距離を取得する
 * 
 * @param serial 対象の SoftwareSerial インスタンス
 * @param anchorName デバッグ用アンカー名
 * @return long 測定された距離 [cm] (失敗・タイムアウト時は -1)
 */
long measureDistance(SoftwareSerial &serial, const char *anchorName) {
  // SoftwareSerial で複数ポートを扱う場合、受信対象を明示的に切り替える必要がある
  serial.listen();
  clearSerialBuffer(serial);

  // 測距要求コマンド: AT+ANCHOR_SEND=<Address>,<Length>,<Data>\r\n
  // ダミーデータ "PING" (4文字) を送信してTWR測距を行う
  serial.print(F("AT+ANCHOR_SEND="));
  serial.print(TARGET_TAG_ADDR);
  serial.print(F(",4,PING\r\n"));

  unsigned long startTime = millis();
  String responseLine = "";
  long distanceCm = -1;

  while (millis() - startTime < RANGING_TIMEOUT_MS) {
    if (serial.available()) {
      char c = serial.read();
      if (c == '\n') {
        responseLine.trim();
        // 成功レスポンス: +ANCHOR_RCV=<Tag_Address>,<Length>,<Data>,<Distance_cm>
        if (responseLine.startsWith("+ANCHOR_RCV=")) {
          int lastCommaIdx = responseLine.lastIndexOf(',');
          if (lastCommaIdx != -1) {
            String distStr = responseLine.substring(lastCommaIdx + 1);
            distanceCm = distStr.toInt();
            return distanceCm;
          }
        } else if (responseLine.startsWith("+ERR")) {
          // エラー応答が返った場合
          return -1;
        }
        responseLine = "";
      } else if (c != '\r') {
        responseLine += c;
      }
    }
  }

  // タイムアウト
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

