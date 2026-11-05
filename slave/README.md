# Slave (Anchor 1, 2, 3) 制御プログラム

Arduino Uno R3 を用いて、地上側の3台の固定基準点（Anchor: RYUW122_Lite）を一括制御し、時分割順次ポーリングによってTag（Master）との距離を測定・PCへ送信するシステムです。

---

## 1. ハードウェア接続・ピン配置

### Arduino Uno R3 ピンアサイン
| モジュール | Arduino ピン | RYUW122_Lite ピン | 備考 |
| :--- | :--- | :--- | :--- |
| **PC (USB)** | D0 (RX), D1 (TX) | - | USBシリアル通信 (115200 bps) |
| **Slave 1 (0, 0)** | **D2 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D3 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須** |
| **Slave 2 (0, 1.0)** | **D4 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D5 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須** |
| **Slave 3 (1.0, 0)** | **D6 (RX)** | Pin 4: TXD | 3.3V出力 → 5V入力（直結OK） |
| | **D7 (TX)** | Pin 3: RXD | **5V → 3.3V 分圧必須** |
| **電源共通** | 3.3V / GND | Pin 1: VDD / Pin 6: GND | ※3台合計の消費電流に注意（外部3.3V電源推奨） |

> [!WARNING]
> **ロジックレベル変換について**  
> ArduinoのTXピン（5V出力）を RYUW122 の RXD（3.3V耐圧）に直接繋ぐと破損の原因になります。  
> 抵抗分圧（Arduino TX → 1kΩ → RYUW122 RXD → 2kΩ → GND）またはロジックレベル変換モジュールを必ず挟んでください。

---

## 2. モジュールの事前設定（初期セットアップ）

新品の RYUW122 は初期ボーレートが `115200` bps です。Arduino の `SoftwareSerial` で安定して通信を行うため、`slave_config_tool/` を使って各モジュールを `9600` bps に設定し、アドレスや動作モードを設定してください。

### 初期設定コマンド手順
シリアルモニタの改行設定を **CR+LF**、ボーレートを **115200** に設定して以下を順次送信します：

1. **導通確認**
   ```text
   AT
   -> +OK
   ```
2. **Anchorモードへ変更**
   ```text
   AT+MODE=1
   -> +OK
   ```
3. **共通ネットワークIDの設定**
   ```text
   AT+NETWORKID=LOCALGPS
   -> +OK
   ```
4. **モジュール個別アドレスの設定**
   - Slave 1: `AT+ADDRESS=SLAVE1`
   - Slave 2: `AT+ADDRESS=SLAVE2`
   - Slave 3: `AT+ADDRESS=SLAVE3`
   - (Tag側): `AT+ADDRESS=TAG1` （※Tag側は `AT+MODE=0`）
5. **ボーレートを 9600 bps に変更**
   ```text
   AT+IPR=9600
   -> +OK
   ```
   *(※ボーレート変更後はシリアルモニタ側・スケッチ側も 9600 に切り替えてください)*

---

## 3. 動作仕様・出力フォーマット

`slave_controller/slave_controller.ino` を書き込むと、以下の手順で自動的に測距ループを実行します。

### ポーリングシーケンス
1. `uwbSerial1.listen()` → Slave 1 から Tag(`TAG1`) へ測距要求 (`AT+ANCHOR_SEND=TAG1,4,PING`)
2. 距離 $d_1$ [cm] を受信（タイムアウト時は `-1`）
3. `uwbSerial2.listen()` → Slave 2 から Tag へ測距要求 → $d_2$ 受信
4. `uwbSerial3.listen()` → Slave 3 から Tag へ測距要求 → $d_3$ 受信
5. PC（USBシリアル）へデータ出力

### PC向け出力フォーマット (CSV)
```text
DIST,<d1_cm>,<d2_cm>,<d3_cm>
```
- 例: `DIST,120,95,150` （Slave1から120cm、Slave2から95cm、Slave3から150cm）
- 測定失敗・タイムアウト時: `DIST,120,-1,150` （Slave2のみ失敗）

