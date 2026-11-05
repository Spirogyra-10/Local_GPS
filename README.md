# Local_GPS

RYUW122搭載 UWB評価用ブレークアウトボード（秋月電子通商: 132079 / REYAX RYUW122_Lite）と Arduino Uno R3 を用いた、地上でのGPSのような2次元自己位置推定（ローカル測位）システムです。

![Python](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Uno%20R3-00979D?logo=arduino&logoColor=white)
![Hardware](https://img.shields.io/badge/Hardware-RYUW122__Lite-orange)

---

## 概要 (Overview)

地上に設置した3つのUWBモジュールを固定基準点（Anchor）とし、移動するロボットに搭載したUWBモジュール（Tag）との距離を双方向測距（Two-Way Ranging: TWR）により測定します。
地上側の基準点3台は1台のArduino Unoに集約制御され、取得された距離データをPC（Pythonプログラム等）にシリアル転送して三辺測量を行うことで、ロボットのリアルタイム2D自己位置推定および画面へのリアルタイム表示を行います。

```text
    Y [m]
     ^
 1.0 +--------- [Slave 2: Anchor (0, 1)]
     |        \
     |         \  d2
     |          \
     |           ● Master: Tag (ロボット: x, y)
     |          / \
     |      d1 /   \ d3
 0.0 +--------+-----+---------> X [m]
    [Slave 1: Anchor (0, 0)]   [Slave 3: Anchor (1, 0)]
```

---

## システム構成 (System Architecture)

### 座標配置
* **Slave 1 (Anchor 1)**: 原点 $(0, 0)$ [m]
* **Slave 2 (Anchor 2)**: Y軸上 $(0, 1.0)$ [m]
* **Slave 3 (Anchor 3)**: X軸上 $(1.0, 0)$ [m]
* **Master (Tag)**: 移動ロボット搭載機 $(x, y)$

### ハードウェア・通信構成
* **地上側（固定基準点）**:
  * **PC**: Python（データ受信、三辺測量座標計算、リアルタイムプロット描画）
  * **Arduino Uno R3 (1台に集約)**: SoftwareSerial を用いて Slave 1〜3 を時分割順次ポーリングし、測距データをPCへUSBシリアル送信
  * **Slave 1, 2, 3**: RYUW122_Lite（Anchorモード: `AT+MODE=1`）
* **ロボット側（移動機）**:
  * **Master**: RYUW122_Lite（Tagモード: `AT+MODE=0`）
  * 電源（3.3V）供給のみでAnchorからの測距要求に自律応答

---

## ディレクトリ構成 (Directory Structure)

```text
Local_GPS/
├── Datasheet/
│   └── RYUW122_Lite.pdf       # UWBモジュールデータシート
├── master/                    # ロボット側（Master / Tag）関連プログラム・設定
├── slave/                     # 地上基準点側（Slave 1〜3 / Anchor）Arduinoスケッチ・PC側受信/描画スクリプト
├── .gitignore                 # バージョン管理除外設定
└── README.md                  # 本ドキュメント
```

---

## Commit Rules
コミットメッセージの初めには必ず prefix をつける様にしましょう.  
[参考：僕が考える最強のコミットメッセージの書き方(Qiita)](https://qiita.com/konatsu_p/items/dfe199ebe3a7d2010b3e)

| Prefix   | 意味                                                    |
| -------- | ------------------------------------------------------- |
| add      | ちょっとしたファイルやコードの追加(画像など)            |
| change   | ちょっとしたファイルやコードの変更(画像差し替え)        |
| feat     | ユーザが利用する機能の追加(add/change を内包しても良い) |
| style    | 機能部分を変更しない、コードの見た目の変化(CSS)         |
| refactor | リファクタリング                                        |
| fix      | バグ修正                                                |
| remove   | ファイルなどの削除                                      |
| test     | テスト関連                                              |
| move     | フォルダの移動                                          |
| chore    | ビルド、補助ツール、ライブラリ関連                      |

---

## 環境セットアップ (Setup)

### 必要な環境
- Git / GitHub が利用可能な環境
- Python 3.10以上
- Arduino IDE (Arduino Uno R3 へのスケッチ書き込み用)
- プログラミングエディタ (推奨 VSCode)

### 仮想環境の構築 (Windows 推奨)
```bash
python -m venv venv
.\venv\Scripts\activate
```

### 必要なライブラリのインストール
```bash
pip install pyserial numpy matplotlib
```

---

## ハードウェア接続時の注意 (Hardware Notice)
- **ロジックレベル**: RYUW122_Lite の動作電圧は **3.3V** です。Arduino Uno R3 のデジタルピン（5V系）からモジュールの RXD へ入力する際は、**必ずロジックレベル変換器または抵抗分圧回路（例: 1kΩ と 2kΩ）** を使用してください。
- **通信速度**: SoftwareSerial 使用時は、モジュール側のボーレートを `9600 bps` 等に設定して安定動作させることを推奨します。

