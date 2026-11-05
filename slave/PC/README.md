# Local GPS - PC側 測位・リアルタイム可視化プログラム

Arduino Uno R3 から USB シリアル経由で送られてくる 3 点の測距データ (`DIST,d1,d2,d3`) を受信し、三辺測量（Trilateration）によって移動ロボット（Tag: Master）の 2D 自己位置 $(x, y)$ をリアルタイムに推定・画面プロット・ロギングする Python プログラム群です。

---

## 1. ファイル構成

```text
slave/PC/
├── main.py            # リアルタイム可視化・メインアプリケーション
├── trilateration.py   # 三辺測量アルゴリズム・移動平均フィルタ
├── serial_reader.py   # シリアル受信スレッド (およびテスト用シミュレータ)
├── requirements.txt   # 必要ライブラリ一覧
└── README.md          # 本マニュアル
```

---

## 2. セットアップ

### 仮想環境の有効化と依存ライブラリのインストール
```bash
# プロジェクトルートで仮想環境を有効化している場合
pip install -r requirements.txt
```

---

## 3. 実行方法

### 通常実行（Arduino 接続時）
```bash
python main.py
```
- 利用可能な COM ポートを自動検出し、通信を開始します。
- 特定のポートを指定する場合:
  ```bash
  python main.py --port COM3
  ```

### シミュレーションモード（Arduino 実機なしでテスト）
Arduino や UWB モジュールが手元にない場合でも、円軌道を描くダミーデータを用いて画面描画や三辺測量アルゴリズムの動作を確認できます。
```bash
python main.py --sim
```

---

## 4. 主なコマンドラインオプション

| オプション | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `--port <PORT>` | 自動検出 | 接続する COM ポート（例: `COM3`） |
| `--baud <BAUD>` | `115200` | Arduino とのシリアル通信速度 |
| `--sim` | 無効 | シミュレーションモードで起動 |
| `--no-filter` | フィルタ有効 | 移動平均フィルタ（スムージング）を無効化 |
| `--history <N>` | `50` | 描画する過去の軌跡点数 |
| `--log <FILE>` | 自動生成 | 測距ログ CSV の出力先パス |

---

## 5. ロギング仕様
実行中は自動的に `logs/ranging_log_YYYYMMDD_HHMMSS.csv` にデータが保存されます。
保存されるカラム:
- `timestamp`: 計測日時 (`YYYY-MM-DD HH:MM:SS.mmm`)
- `d1_m`, `d2_m`, `d3_m`: 各アンカーからの距離 [m]
- `est_x_m`, `est_y_m`: 推定された自己位置座標 [m]

