/**
 * @file slave_controller.ino
 * @brief UWB Local GPS - 3連Anchor統合制御プログラム (自動キャリブレーション機能付き)
 */

#include <SoftwareSerial.h>

// ==========================================
// デバッグ設定
// ==========================================
bool isDebugMode = false;

// ==========================================
// ピン配置設定 (Arduino Uno R3)
// ==========================================
const uint8_t UWB_RESET_PIN = 8; // to All Slaves Pin 2 (NRST)

const uint8_t SLAVE1_RX_PIN = 2; // to Slave1 Pin 4 (TXD)
const uint8_t SLAVE1_TX_PIN = 3; // to Slave1 Pin 3 (RXD)

const uint8_t SLAVE2_RX_PIN = 4; // to Slave2 Pin 4 (TXD)
const uint8_t SLAVE2_TX_PIN = 5; // to Slave2 Pin 3 (RXD)

const uint8_t SLAVE3_RX_PIN = 6; // to Slave3 Pin 4 (TXD)
const uint8_t SLAVE3_TX_PIN = 7; // to Slave3 Pin 3 (RXD)

const uint8_t HEARTBEAT_LED = 13; // 動作確認用内蔵LED

// ==========================================
// 通信設定
// ==========================================
const unsigned long PC_BAUDRATE = 115200;
const unsigned long UWB_BAUDRATE = 9600;

const char TARGET_TAG_ADDR[] = "TAG1";

const unsigned long DEFAULT_RANGING_TIMEOUT_MS = 200;
const unsigned long SWITCH_DELAY_MS = 15;
const unsigned long CALIBRATION_INTERVAL_MS = 5000;
const long MAX_VALID_DISTANCE_CM = 3000;

// ==========================================
// SoftwareSerial インスタンス定義
// ==========================================
SoftwareSerial uwbSerial1(SLAVE1_RX_PIN, SLAVE1_TX_PIN);
SoftwareSerial uwbSerial2(SLAVE2_RX_PIN, SLAVE2_TX_PIN);
SoftwareSerial uwbSerial3(SLAVE3_RX_PIN, SLAVE3_TX_PIN);

long calib_d12 = 100; // デフォルト 1.0m (cm)
long calib_d13 = 100; // デフォルト 1.0m (cm)
unsigned long lastCalibTime = 0;

// ==========================================
// プロトタイプ宣言
// ==========================================
long measureDistance(SoftwareSerial &serial, const char *targetAddr, const char *anchorTag, unsigned long timeoutMs = DEFAULT_RANGING_TIMEOUT_MS);
void clearSerialBuffer(SoftwareSerial &serial);
void checkDebugCommand();
void resetAllUwbModules();
void runAnchorCalibration();

void setup() {
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  Serial.begin(PC_BAUDRATE);
  while (!Serial);

  Serial.println(F("# =================================================="));
  Serial.println(F("# SYSTEM_START: Local GPS Slave Controller v2.2    "));
  Serial.println(F("# Auto-Calibration: 5s Interval (S1->S2, S1->S3)    "));
  Serial.println(F("# Hardware Auto-Reset on Pin D8                     "));
  Serial.println(F("# =================================================="));

  resetAllUwbModules();

  uwbSerial1.begin(UWB_BAUDRATE);
  uwbSerial2.begin(UWB_BAUDRATE);
  uwbSerial3.begin(UWB_BAUDRATE);

  clearSerialBuffer(uwbSerial1);
  clearSerialBuffer(uwbSerial2);
  clearSerialBuffer(uwbSerial3);

  // 初回キャリブレーション
  runAnchorCalibration();
  lastCalibTime = millis();

  Serial.println(F("# [READY] Controller Active."));
  delay(100);
}

void loop() {
  checkDebugCommand();

  static bool ledState = false;
  ledState = !ledState;
  digitalWrite(HEARTBEAT_LED, ledState ? HIGH : LOW);

  // 1. 定期アンカー自動キャリブレーション (5秒に1回)
  if (millis() - lastCalibTime >= CALIBRATION_INTERVAL_MS) {
    lastCalibTime = millis();
    runAnchorCalibration();
  }

  // 2. ロボット側 Tag (Master) の測距
  long d1 = measureDistance(uwbSerial1, TARGET_TAG_ADDR, "S1");
  long d2 = measureDistance(uwbSerial2, TARGET_TAG_ADDR, "S2");
  long d3 = measureDistance(uwbSerial3, TARGET_TAG_ADDR, "S3");

  // 3. データ出力
  if (isDebugMode) {
    Serial.print(F("[SUMMARY] S1: "));
    Serial.print(d1);
    Serial.print(F(" cm | S2: "));
    Serial.print(d2);
    Serial.print(F(" cm | S3: "));
    Serial.print(d3);
    Serial.print(F(" cm | Calib: d12="));
    Serial.print(calib_d12);
    Serial.print(F(", d13="));
    Serial.print(calib_d13);
    Serial.println(F(" cm"));
    Serial.println(F("--------------------------------------------------"));
  } else {
    Serial.print(F("DIST,"));
    Serial.print(d1);
    Serial.print(F(","));
    Serial.print(d2);
    Serial.print(F(","));
    Serial.println(d3);
  }
}

/**
 * @brief アンカー間自動キャリブレーション処理
 */
void runAnchorCalibration() {
  if (isDebugMode) {
    Serial.println(F("# [CALIB] Starting Auto-Calibration Sequence..."));
  }

  // --- Step 1: Slave 2 を一時的に TAG モードに切り替え ---
  uwbSerial2.listen();
  delay(SWITCH_DELAY_MS);
  uwbSerial2.print(F("AT+MODE=0\r\n"));
  delay(200);
  clearSerialBuffer(uwbSerial2);

  // Slave 1 から Slave 2 を測距 (タイムアウト 400ms)
  long d12_meas = measureDistance(uwbSerial1, "SLAVE2", "CALIB_S1->S2", 400);

  // Slave 2 を ANCHOR モードに復旧
  uwbSerial2.listen();
  delay(SWITCH_DELAY_MS);
  uwbSerial2.print(F("AT+MODE=1\r\n"));
  delay(200);
  clearSerialBuffer(uwbSerial2);

  // --- Step 2: Slave 3 を一時的に TAG モードに切り替え ---
  uwbSerial3.listen();
  delay(SWITCH_DELAY_MS);
  uwbSerial3.print(F("AT+MODE=0\r\n"));
  delay(200);
  clearSerialBuffer(uwbSerial3);

  // Slave 1 から Slave 3 を測距 (タイムアウト 400ms)
  long d13_meas = measureDistance(uwbSerial1, "SLAVE3", "CALIB_S1->S3", 400);

  // Slave 3 を ANCHOR モードに復旧
  uwbSerial3.listen();
  delay(SWITCH_DELAY_MS);
  uwbSerial3.print(F("AT+MODE=1\r\n"));
  delay(200);
  clearSerialBuffer(uwbSerial3);

  if (d12_meas > 0 && d12_meas <= MAX_VALID_DISTANCE_CM) {
    calib_d12 = d12_meas;
  }
  if (d13_meas > 0 && d13_meas <= MAX_VALID_DISTANCE_CM) {
    calib_d13 = d13_meas;
  }

  // PCへキャリブレーション結果を出力: CALIB,d12,d13
  Serial.print(F("CALIB,"));
  Serial.print(calib_d12);
  Serial.print(F(","));
  Serial.println(calib_d13);

  if (isDebugMode) {
    Serial.print(F("# [CALIB] Updated baselines: S1-S2="));
    Serial.print(calib_d12);
    Serial.print(F(" cm, S1-S3="));
    Serial.print(calib_d13);
    Serial.println(F(" cm"));
  }
}

void resetAllUwbModules() {
  Serial.println(F("# [INIT] Resetting UWB Anchors via D8..."));
  pinMode(UWB_RESET_PIN, OUTPUT);
  digitalWrite(UWB_RESET_PIN, LOW);
  delay(30);
  digitalWrite(UWB_RESET_PIN, HIGH);
  pinMode(UWB_RESET_PIN, INPUT_PULLUP);
  delay(600);
  Serial.println(F("# [INIT] UWB Anchors boot sequence complete."));
}

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

long measureDistance(SoftwareSerial &serial, const char *targetAddr, const char *anchorTag, unsigned long timeoutMs) {
  serial.listen();
  delay(SWITCH_DELAY_MS);
  clearSerialBuffer(serial);

  if (isDebugMode) {
    Serial.print(F("["));
    Serial.print(anchorTag);
    Serial.print(F("] TX: AT+ANCHOR_SEND="));
    Serial.print(targetAddr);
    Serial.println(F(",4,PING"));
  }

  serial.print(F("AT+ANCHOR_SEND="));
  serial.print(targetAddr);
  serial.print(F(",4,PING\r\n"));

  unsigned long startTime = millis();
  char rxBuf[64];
  uint8_t bufIdx = 0;

  while (millis() - startTime < timeoutMs) {
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
    Serial.println(F("] -> TIMEOUT"));
  }

  return -1;
}

void clearSerialBuffer(SoftwareSerial &serial) {
  while (serial.available() > 0) {
    serial.read();
  }
}
