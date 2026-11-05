"""
serial_reader.py
Arduino からの USB シリアル通信を受信・パースするモジュール
"""

import math
import random
import re
import threading
import time
from typing import Callable, Optional, Tuple
import serial
import serial.tools.list_ports


def list_available_ports():
    """利用可能なCOMポートの一覧を返す"""
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]


class SerialReader:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        on_data_received: Optional[
            Callable[[float, float, float], None]
        ] = None,
    ):
        """シリアル受信スレッドの初期化

        Parameters
        ----------
        port : Optional[str]
            接続先COMポート (例: 'COM3')。Noneの場合は自動検出を試みる
        baudrate : int
            Arduino との通信速度 (デフォルト: 115200)
        on_data_received : Optional[Callable[[float, float, float], None]]
            距離データ (d1_m, d2_m, d3_m) 受信時のコールバック関数
        """
        self.port = port
        self.baudrate = baudrate
        self.on_data_received = on_data_received

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._serial: Optional[serial.Serial] = None

        # 最新の距離データ [m] (スレッドセーフ保持用)
        self.latest_distances: Tuple[float, float, float] = (-1.0, -1.0, -1.0)
        self._lock = threading.Lock()

    def start(self):
        """受信スレッドの開始"""
        if self._running:
            return

        if not self.port:
            ports = list_available_ports()
            if not ports:
                raise RuntimeError("No serial COM ports found!")
            self.port = ports[0]
            print(f"[SerialReader] Auto-selected port: {self.port}")

        self._serial = serial.Serial(
            self.port, self.baudrate, timeout=1.0
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[SerialReader] Started listening on {self.port} at {self.baudrate} bps.")

    def stop(self):
        """受信スレッドの停止"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("[SerialReader] Stopped.")

    def get_latest_distances(self) -> Tuple[float, float, float]:
        """最新の距離データ [m] を取得"""
        with self._lock:
            return self.latest_distances

    def _read_loop(self):
        """バックグラウンド受信ループ"""
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    break

                line_bytes = self._serial.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # フォーマット: DIST,d1,d2,d3 (単位: cm)
                if line.startswith("DIST,"):
                    parts = line.split(",")
                    if len(parts) == 4:
                        try:
                            # cm から m に変換 (-1 の場合はそのまま -1.0)
                            d1_cm = float(parts[1])
                            d2_cm = float(parts[2])
                            d3_cm = float(parts[3])

                            d1_m = d1_cm / 100.0 if d1_cm >= 0 else -1.0
                            d2_m = d2_cm / 100.0 if d2_cm >= 0 else -1.0
                            d3_m = d3_cm / 100.0 if d3_cm >= 0 else -1.0

                            with self._lock:
                                self.latest_distances = (d1_m, d2_m, d3_m)

                            if self.on_data_received:
                                self.on_data_received(d1_m, d2_m, d3_m)

                        except ValueError:
                            pass
                elif line.startswith("#"):
                    print(f"[Arduino LOG] {line}")

            except Exception as e:
                if self._running:
                    print(f"[SerialReader Error] {e}")
                time.sleep(0.1)


class DummySerialReader:
    """実機が無い場合のシミュレーション・テスト用リーダー (円軌道を生成)"""

    def __init__(
        self,
        anchor1=(0.0, 0.0),
        anchor2=(0.0, 1.0),
        anchor3=(1.0, 0.0),
        on_data_received=None,
    ):
        self.anchors = [anchor1, anchor2, anchor3]
        self.on_data_received = on_data_received
        self._running = False
        self._thread = None
        self.latest_distances = (-1.0, -1.0, -1.0)
        self._lock = threading.Lock()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        print("[DummySerialReader] Simulation mode started (generating circular trajectory).")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)

    def get_latest_distances(self):
        with self._lock:
            return self.latest_distances

    def _sim_loop(self):
        # 中心 (0.5, 0.5), 半径 0.25 の円運動
        t = 0.0
        while self._running:
            x = 0.5 + 0.25 * math.cos(t)
            y = 0.5 + 0.25 * math.sin(t)

            # ノイズを少し加えた理想距離
            d1 = math.hypot(x - self.anchors[0][0], y - self.anchors[0][1]) + random.gauss(0, 0.01)
            d2 = math.hypot(x - self.anchors[1][0], y - self.anchors[1][1]) + random.gauss(0, 0.01)
            d3 = math.hypot(x - self.anchors[2][0], y - self.anchors[2][1]) + random.gauss(0, 0.01)

            with self._lock:
                self.latest_distances = (d1, d2, d3)

            if self.on_data_received:
                self.on_data_received(d1, d2, d3)

            t += 0.1
            time.sleep(0.05)
