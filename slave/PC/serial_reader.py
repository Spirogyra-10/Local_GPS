"""
serial_reader.py
Arduino からの USB シリアル通信を受信・パースするモジュール
- 自動再接続機能 (Auto-reconnect)
- スレッドセーフなデータ保持
- テスト用シミュレータ (DummySerialReader)
"""

import math
import random
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
            接続先COMポート (例: 'COM3')。Noneの場合は自動検出
        baudrate : int
            Arduino との通信速度 (デフォルト: 115200)
        on_data_received : Optional[Callable[[float, float, float], None]]
            距離データ受信時のコールバック
        """
        self.port = port
        self.baudrate = baudrate
        self.on_data_received = on_data_received

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._serial: Optional[serial.Serial] = None

        self.latest_distances: Tuple[float, float, float] = (-1.0, -1.0, -1.0)
        self._lock = threading.Lock()
        self.is_connected = False

    def start(self):
        """受信スレッドの開始"""
        if self._running:
            return

        self._running = True
        self._thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """受信スレッドの停止"""
        self._running = False
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.is_connected = False
        print("[SerialReader] Stopped.")

    def get_latest_distances(self) -> Tuple[float, float, float]:
        """最新の距離データ [m] を取得"""
        with self._lock:
            return self.latest_distances

    def _connection_loop(self):
        """接続維持・自動再接続ループ"""
        while self._running:
            try:
                target_port = self.port
                if not target_port:
                    ports = list_available_ports()
                    if ports:
                        target_port = ports[0]
                    else:
                        time.sleep(1.0)
                        continue

                self._serial = serial.Serial(target_port, self.baudrate, timeout=0.5)
                self._serial.reset_input_buffer()
                self.is_connected = True
                print(f"[SerialReader] Connected to {target_port} at {self.baudrate} bps.")

                # 受信ループ
                while self._running and self._serial.is_open:
                    line_bytes = self._serial.readline()
                    if not line_bytes:
                        continue

                    line = line_bytes.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    # フォーマット: DIST,d1,d2,d3 (単位: cm)
                    if line.startswith("DIST,"):
                        parts = line.split(",")
                        if len(parts) >= 4:
                            try:
                                d1_cm = float(parts[1])
                                d2_cm = float(parts[2])
                                d3_cm = float(parts[3])

                                d1_m = d1_cm / 100.0 if d1_cm > 0 else -1.0
                                d2_m = d2_cm / 100.0 if d2_cm > 0 else -1.0
                                d3_m = d3_cm / 100.0 if d3_cm > 0 else -1.0

                                with self._lock:
                                    self.latest_distances = (d1_m, d2_m, d3_m)

                                if self.on_data_received:
                                    self.on_data_received(d1_m, d2_m, d3_m)

                            except ValueError:
                                pass
                    elif line.startswith("#"):
                        print(f"[Arduino] {line}")

            except (serial.SerialException, OSError) as e:
                self.is_connected = False
                if self._running:
                    print(f"[SerialReader] Disconnected / Retrying ({e})...")
                    time.sleep(1.5)
            except Exception as e:
                if self._running:
                    print(f"[SerialReader Error] {e}")
                    time.sleep(1.0)


class DummySerialReader:
    """実機が無い場合のテスト用シミュレータリーダー"""

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
        self.is_connected = True

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()
        print("[DummySerialReader] Simulation mode started (generating 8-figure trajectory).")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)

    def get_latest_distances(self):
        with self._lock:
            return self.latest_distances

    def _sim_loop(self):
        # 8の字ループ軌道
        t = 0.0
        while self._running:
            x = 0.5 + 0.3 * math.cos(t)
            y = 0.5 + 0.3 * math.sin(2.0 * t) / 2.0

            d1 = math.hypot(x - self.anchors[0][0], y - self.anchors[0][1]) + random.gauss(0, 0.015)
            d2 = math.hypot(x - self.anchors[1][0], y - self.anchors[1][1]) + random.gauss(0, 0.015)
            d3 = math.hypot(x - self.anchors[2][0], y - self.anchors[2][1]) + random.gauss(0, 0.015)

            with self._lock:
                self.latest_distances = (d1, d2, d3)

            if self.on_data_received:
                self.on_data_received(d1, d2, d3)

            t += 0.05
            time.sleep(0.05)
