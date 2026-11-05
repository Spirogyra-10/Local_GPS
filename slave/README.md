# Slave (Anchor 1, 2, 3) 統合制御システム

Arduino Uno R3 を用いて、地上側の3台の固定基準点（Anchor: RYUW122_Lite）を一括制御し、時分割順次ポーリングによって移動機 Tag（Master）との距離を高速測定・PCへ送信するシステムです。

---

## 1. ハードウェア接続・ピン配置

### Arduino Uno R3 ピンアサイン
| 接続先 | Arduino ピン | RYUW122_Lite ピン | 備考 |
| :--- | :--- | :--- | :--- |
| **PC (USB)** | D0 (RX), D1 (TX) | - | USBシリアル通信 (115200 bps) |
| **Slave 1 (0, 0)** | **D2 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D3 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須 (1kΩ/2kΩ)** |
| **Slave 2 (0, 1.0)** | **D4 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D5 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須 (1kΩ/2kΩ)** |
| **Slave 3 (1.0, 0)** | **D6 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D7 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須 (1kΩ/2kΩ)** |
| **インジケータ** | **D13 (LED)** | 内蔵LED | 測距ループ実行時にトグル点滅 (ハートビート) |
| **電源共通** | 3.3V / GND | Pin 1: VDD / Pin 6: GND | ※外部3.3V電源またはArduinoの3.3V出力 |

> [!WARNING]
> **ロジックレベル保護**  
> ArduinoのTXピン（5V出力）を RYUW122 の RXD（3.3V耐圧）に直接繋ぐと故障の原因になります。  
> 抵抗分圧（Arduino TX → 1kΩ → RYUW122 RXD → 2kΩ → GND）を必ず介してください。

---

## 2. モジュールの事前設定ツール (`slave_config_tool/`)

配線を Arduino に差したまま、シリアルモニタ（115200 bps, 改行設定: なし または CR+LF）から 3 台のモジュールを切り替えて初期設定できます。

### 特殊コマンド
- `#1` / `#2` / `#3` : 操作対象のモジュールを Slave 1, 2, 3 に切り替え
- `#SET9600` : 出荷時 115200 bps のモジュールへ `AT+IPR=9600` をブラインド送信し、Arduino 側も自動で 9600 bps に再初期化

### 3台の設定手順例
1. `#1` を送信 $\to$ `#SET9600` を送信 $\to$ `AT+MODE=1` $\to$ `AT+NETWORKID=LOCALGPS` $\to$ `AT+ADDRESS=SLAVE1`
2. `#2` を送信 $\to$ `#SET9600` を送信 $\to$ `AT+MODE=1` $\to$ `AT+NETWORKID=LOCALGPS` $\to$ `AT+ADDRESS=SLAVE2`
3. `#3` を送信 $\to$ `#SET9600` を送信 $\to$ `AT+MODE=1` $\to$ `AT+NETWORKID=LOCALGPS` $\to$ `AT+ADDRESS=SLAVE3`

---

## 3. 本番制御スケッチ (`slave_controller/`)

`slave_controller.ino` を書き込むと、自動的に順次ポーリング測定が開始されます。

### 信頼性・性能向上のための設計仕様
- **String型全廃によるヒープ断片化防止**: 固定長 char バッファと標準 C ライブラリ関数 (`strrchr`, `atol`) により、Arduino Uno の 2KB SRAM 環境下で長時間安定稼働。
- **サンプリング高速化**: タイムアウトを 120ms に短縮し、ポート切替ディレイ (10ms) を付与することで、毎秒 3〜5 回の測位更新レートを実現。
- **ハートビートLED**: D13 内蔵 LED が測距サイクルごとにトグル点滅し、ハードウェア単体で動作確認可能。

### PCシリアル出力フォーマット
```text
DIST,<d1_cm>,<d2_cm>,<d3_cm>
```
- 例: `DIST,120,95,150` （120cm, 95cm, 150cm）
- タイムアウト・未検出時: `-1`（例: `DIST,120,-1,150`）

---

## 4. PC側リアルタイム可視化 (`PC/`)

- **3点測量 (線形最小二乗解)** に加え、1点欠損時の **2点幾何推定フォールバック** を搭載。一時的な電波途切れでも自己位置ロストを防ぎます。
- 自動再接続 (Auto-reconnect) により、USB ケーブルの抜き差しにも対応。
- 実機なしでテスト可能なシミュレーションモード (`python main.py --sim`) を完備。
