/**
 * @file slave_controller.ino
 * @brief UWB Local GPS - 3連Anchor (Slave 1, 2, 3) 統合制御プログラム
 * @details 
 * Arduino Uno R3 に接続された3基の RYUW122_Lite (Anchorモード) を
 * SoftwareSerial を用いて時分割順次ポーリングし、
 * ロボット側 Tag (Master) との距離を取得して PC へシリアル出力します。
 * 
 * 【改善点・見直しポイント】
 * 1. String クラスを全廃し、固定長 char バッファを使用（RAM 2KB の断片化・フリーズ防止）
 * 2. SoftwareSerial 切替 (listen()) 後の安定化ディレイ (10ms) を追加
 * 3. タイムアウト値を 250ms -> 120ms に短縮し、サンプリングレートを高速化
 * 4. 内蔵 LED (D13) を利用したハートビート点滅インジケータ
 * 5. 測定値の妥当性検証（0cm以下または最大距離オーバーの除外）
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

const uint8_t HEARTBEAT_LED = 13; // 動作確認用内蔵LED

// ==========================================
// 通信設定
// ==========================================
// PCとの通信速度 (ハードウェアシリアル)
const unsigned long PC_BAUDRATE = 115200;

// RYUW122との通信速度 (SoftwareSerialのため 9600bps 推奨)
const unsigned long UWB_BAUDRATE = 9600;

// 測距対象の Tag (Master) アドレス
const char TARGET_TAG_ADDR[] = "TAG1";

// 測距コマンドのタイムアウト時間 [ms] (RYUW122の標準応答時間は約50ms)
const unsigned long RANGING_TIMEOUT_MS = 120;

// ポート切り替え後の安定化ウェイト [ms]
const unsigned long SWITCH_DELAY_MS = 10;

// 測定値の上限 [cm] (定格100mだが屋内測位として30m以上はノイズとみなす)
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
long measureDistance(SoftwareSerial &serial);
void clearSerialBuffer(SoftwareSerial &serial);

void setup() {
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  // PCシリアル初期化
  Serial.begin(PC_BAUDRATE);
  while (!Serial) {
    ; // シリアル接続待機
  }

  // 各UWB通信初期化
  uwbSerial1.begin(UWB_BAUDRATE);
  uwbSerial2.begin(UWB_BAUDRATE);
  uwbSerial3.begin(UWB_BAUDRATE);

  Serial.println(F("# SYSTEM_START: Local GPS Slave Controller v1.1"));
  Serial.println(F("# Output format: DIST,d1_cm,d2_cm,d3_cm (d=-1 on error/timeout)"));
  delay(200);
}

void loop() {
  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(HEARTBEAT_LED, ledState ? HIGH : LOW);

  // 1. Slave 1 (原点: 0, 0)
  long d1 = measureDistance(uwbSerial1);

  // 2. Slave 2 (Y軸: 0, 1.0)
  long d2 = measureDistance(uwbSerial2);

  // 3. Slave 3 (X軸: 1.0, 0)
  long d3 = measureDistance(uwbSerial3);

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
 * @return long 測定された距離 [cm] (失敗・タイムアウト時は -1)
 */
long measureDistance(SoftwareSerial &serial) {
  // SoftwareSerial で複数ポートを扱う場合、受信対象を明示的に切り替える
  serial.listen();
  delay(SWITCH_DELAY_MS); // 切替後のピン割り込み安定化
  clearSerialBuffer(serial);

  // 測距要求コマンド: AT+ANCHOR_SEND=<Address>,<Length>,<Data>\r\n
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
        // 行終端
        rxBuf[bufIdx] = '\0';

        // 成功レスポンス: +ANCHOR_RCV=<Tag_Address>,<Length>,<Data>,<Distance_cm>
        if (strncmp(rxBuf, "+ANCHOR_RCV=", 12) == 0) {
          // 最後のカンマを探して距離文字列を抽出
          char *lastComma = strrchr(rxBuf, ',');
          if (lastComma != NULL) {
            long dist = atol(lastComma + 1);
            if (dist > 0 && dist <= MAX_VALID_DISTANCE_CM) {
              return dist;
            }
          }
        } else if (strncmp(rxBuf, "+ERR", 4) == 0) {
          return -1;
        }

        // 次の行に備えてバッファクリア
        bufIdx = 0;
      } else if (c != '\r') {
        if (bufIdx < sizeof(rxBuf) - 1) {
          rxBuf[bufIdx++] = c;
        }
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
