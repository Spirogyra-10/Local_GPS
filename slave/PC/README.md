# Local GPS - PC側 測位・リアルタイム可視化プログラム

Arduino Uno R3 から USB シリアル経由で送られてくる 3 点の測距データ (`DIST,d1,d2,d3`) を受信し、三辺測量（Trilateration）によって移動ロボット（Tag: Master）の 2D 自己位置 $(x, y)$ をリアルタイムに推定・画面プロット・ロギングする Python プログラム群です。

---

## 1. ファイル構成

```text
slave/PC/
├── main.py            # リアルタイム可視化・メインアプリケーション
├── web_visualizer.py  # ブラウザ Web GUI サーバー & SSE 配信エンジン
├── web/               # ブラウザ UI (オフライン完結 HTML / CSS / Canvas JS)
│   ├── index.html
│   ├── style.css
│   └── app.js
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

### 通常実行（Arduino 接続時・ブラウザ Web GUI）
```bash
python3 main.py --port /dev/ttyACM0
```
- 起動するとローカル Web サーバー（デフォルト: `http://127.0.0.1:8080`）が立ち上がり、ブラウザが自動的に開きます。
- ポート指定を省略すると、利用可能なシリアルポートを自動検出します。
- 主な特徴:
  - **距離追従自動スケール**: 測距値やロボット位置に応じて 2D マップ表示範囲と距離推移グラフの Y 軸が滑らかに自動拡縮します（マウスホイール・ドラッグ操作、ダブルクリックで自動スケール復帰）。
  - **高視認性テレメトリ**: 大型座標表示、色分け距離バー、ダーク / ライトテーマ切替。
  - **ライブ操作**: 画面ボタンまたはショートカットキー（`C`: 再キャリブレーション、`R`: ポート入替、`X`: 軌跡クリア、`A`: 自動スケール復帰）。
- 従来の Matplotlib ウィンドウを使う場合は `--gui matplotlib` を指定してください。

### シミュレーションモード（Arduino 実機なしでテスト）
Arduino や UWB モジュールが手元にない場合でも、ダミーデータを用いてブラウザ画面描画や三辺測量アルゴリズムの動作を確認できます。
```bash
python3 main.py --sim
```

---

## 4. 主なコマンドラインオプション

| オプション | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `--port <PORT>` | 自動検出 | 接続するシリアルポート（例: `/dev/ttyACM0`, `COM3`） |
| `--baud <BAUD>` | `115200` | Arduino とのシリアル通信速度 |
| `--gui <TYPE>` | `web` | GUI バックエンド (`web`: ブラウザ, `matplotlib`: 従来ウィンドウ) |
| `--host <HOST>` | `127.0.0.1` | Web GUI サーバーの待受ホスト |
| `--web-port <PORT>` | `8080` | Web GUI サーバーの HTTP ポート（使用中なら連番で自動選択） |
| `--no-browser` | 無効 | Web GUI 起動時にブラウザを自動で開かない |
| `--spacing <M>` | `1.0` | アンカー間の標準配置距離 [m] |
| `--spacing-x <M>` | `--spacing` | X軸 (Slave 3) の想定配置距離 [m] |
| `--spacing-y <M>` | `--spacing` | Y軸 (Slave 2) の想定配置距離 [m] |
| `--relabel <MODE>` | `auto` | リラベルモード (`auto` / `normal` / `swap`) |
| `--range <M>` | `4.0` | 初期表示範囲 [m]（Web GUI では距離に応じて自動拡縮） |
| `--sim` | 無効 | シミュレーションモードで起動 |
| `--no-filter` | フィルタ有効 | スムージングフィルタを無効化 |
| `--history <N>` | `60` | 描画する過去の軌跡点数 |
| `--log <FILE>` | 自動生成 | 測距ログ CSV の出力先パス |

---

## 5. ロギング仕様
実行中は自動的に `logs/ranging_log_YYYYMMDD_HHMMSS.csv` にデータが保存されます。
保存されるカラム:
- `timestamp`: 計測日時 (`YYYY-MM-DD HH:MM:SS.mmm`)
- `d1_m`, `d2_m`, `d3_m`: 各アンカーからの距離 [m]
- `est_x_m`, `est_y_m`: 推定された自己位置座標 [m]

