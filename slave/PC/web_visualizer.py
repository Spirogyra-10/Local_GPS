"""
web_visualizer.py
Local GPS - ブラウザベースのリアルタイム可視化サーバー & 位置推定エンジン
- Python標準ライブラリのみで動作 (追加Webライブラリ不要・完全オフライン対応)
- Server-Sent Events (SSE) による低遅延リアルタイム配信 (/api/stream)
- JSON API によるブラウザからの操作 (/api/action)
"""

import csv
import datetime
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlsplit
import webbrowser

import numpy as np

from trilateration import DistanceFilter, MovingAverageFilter, Trilateration

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


def _clean_float(value: float, digits: int = 4) -> float:
    """JSONシリアライズ安全な有限 float 値へ変換する"""
    val = float(value)
    if not math.isfinite(val):
        return -1.0
    return round(val, digits)


class LocalGPSEngine:
    """シリアル入力から測位計算・フィルタリング・CSVロギング・状態配信を担うスレッドセーフなエンジン"""

    def __init__(
        self,
        reader: Any,
        anchor1: Tuple[float, float] = (0.0, 0.0),
        anchor2: Tuple[float, float] = (0.0, 1.0),
        anchor3: Tuple[float, float] = (1.0, 0.0),
        nominal_d12: float = 1.0,
        nominal_d13: float = 1.0,
        relabel_mode: str = "auto",
        use_filter: bool = True,
        history_len: int = 60,
        log_csv: Optional[str] = None,
        initial_range: float = 4.0,
        port_label: str = "",
        is_sim: bool = False,
        update_interval: float = 0.05,
    ):
        self.reader = reader
        self.tri = Trilateration(
            anchor1=anchor1,
            anchor2=anchor2,
            anchor3=anchor3,
            nominal_d12=nominal_d12,
            nominal_d13=nominal_d13,
            relabel_mode=relabel_mode,
        )
        self.dist_filter = DistanceFilter(alpha=0.35)
        self.pos_filter = MovingAverageFilter(window_size=5)
        self.use_filter = bool(use_filter)
        self.history_len = max(10, min(500, int(history_len)))
        self.initial_range = float(initial_range)
        self.port_label = port_label
        self.is_sim = bool(is_sim)
        self.update_interval = float(update_interval)
        self.latest_calib: Tuple[float, float] = (float(anchor2[1]), float(anchor3[0]))

        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self.trajectory = []
        self._seq = 0
        self._last_event = ""
        self._frame_count = 0
        self._last_tick_time: Optional[float] = None
        self._hz = 0.0

        self._last_raw = (-1.0, -1.0, -1.0)
        self._last_remapped = (-1.0, -1.0, -1.0)
        self._last_filtered = (-1.0, -1.0, -1.0)
        self._last_pos: Optional[Tuple[float, float]] = None
        self._last_mode = "ERROR"

        self.log_csv = log_csv
        self.csv_file = None
        self.csv_writer = None
        if self.log_csv:
            log_dir = os.path.dirname(self.log_csv)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            self.csv_file = open(self.log_csv, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "timestamp",
                    "raw_d1",
                    "raw_d2",
                    "raw_d3",
                    "d1_m",
                    "d2_m",
                    "d3_m",
                    "est_x_m",
                    "est_y_m",
                    "mode",
                    "calib_d12",
                    "calib_d13",
                    "relabel",
                ]
            )

        if hasattr(self.reader, "on_calib_received"):
            self.reader.on_calib_received = self.on_calib_update

        self._latest_state_dict: Dict[str, Any] = {}
        self._latest_state_json = "{}"
        with self._lock:
            self._rebuild_state_locked(datetime.datetime.now().isoformat())

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    def _rebuild_state_locked(self, timestamp: Optional[str] = None) -> None:
        """最新の状態辞書と JSON 文字列を再構築する (要 _lock 保持)"""
        ts = timestamp or datetime.datetime.now().isoformat()
        pos = self._last_pos
        state: Dict[str, Any] = {
            "seq": self._seq,
            "timestamp": ts,
            "connected": bool(getattr(self.reader, "is_connected", True)),
            "is_sim": self.is_sim,
            "port": self.port_label,
            "hz": round(self._hz, 1),
            "raw_distances": [_clean_float(v) for v in self._last_raw],
            "distances": [_clean_float(v) for v in self._last_remapped],
            "filtered_distances": [_clean_float(v) for v in self._last_filtered],
            "pos": [_clean_float(pos[0]), _clean_float(pos[1])] if pos is not None else None,
            "mode": self._last_mode,
            "anchors": [[_clean_float(a[0]), _clean_float(a[1])] for a in self.tri.anchors],
            "calib": {
                "d12": _clean_float(self.tri.d12),
                "d13": _clean_float(self.tri.d13),
                "nominal_d12": _clean_float(self.tri.nominal_d12),
                "nominal_d13": _clean_float(self.tri.nominal_d13),
            },
            "is_swapped": bool(self.tri.is_swapped),
            "relabel_mode": self.tri.relabel_mode,
            "use_filter": bool(self.use_filter),
            "history_len": int(self.history_len),
            "initial_range": _clean_float(self.initial_range, 2),
            "trajectory": list(self.trajectory),
            "log_file": self.log_csv or "",
            "last_event": self._last_event,
        }
        self._latest_state_dict = state
        self._latest_state_json = json.dumps(state, ensure_ascii=False, separators=(",", ":"))

    def _publish_locked(self, timestamp: Optional[str] = None) -> Dict[str, Any]:
        """状態を更新・インクリメントして SSE 待機スレッドへ通知する (要 _lock 保持)"""
        self._seq += 1
        self._rebuild_state_locked(timestamp)
        self._cond.notify_all()
        return dict(self._latest_state_dict)

    def _reset_filters_locked(self) -> None:
        self.dist_filter.reset()
        self.pos_filter.history_x.clear()
        self.pos_filter.history_y.clear()

    def on_calib_update(self, d12_m: float, d13_m: float) -> None:
        """Arduino から CALIB,d12,d13 が届いた時の更新処理 (SerialReader スレッドから呼ばれる)"""
        with self._lock:
            self.latest_calib = (float(d12_m), float(d13_m))
            swapped_changed = self.tri.update_anchors(d12_m, d13_m)

            if swapped_changed:
                self._reset_filters_locked()
                state_str = (
                    "SWAPPED (Relabeled: Port 3->S2, Port 2->S3)"
                    if self.tri.is_swapped
                    else "NORMAL (Port 2->S2, Port 3->S3)"
                )
                print(f"\n[Auto-Relabel] Mapping updated -> {state_str}")
                self._last_event = f"Auto-Relabel: {state_str}"
            else:
                self._last_event = f"Calibration updated: S2={self.tri.d12:.2f}m, S3={self.tri.d13:.2f}m"

            self._publish_locked()

    def step(self) -> None:
        """1フレーム分の測距読込・三辺測量・ロギング・配信更新を実行"""
        if hasattr(self.reader, "get_latest_distances"):
            raw_d1, raw_d2, raw_d3 = self.reader.get_latest_distances()
        else:
            raw_d1, raw_d2, raw_d3 = self.reader.latest_distances
        c12, c13 = getattr(self.reader, "latest_calib", self.latest_calib)

        now_mono = time.monotonic()
        ts = datetime.datetime.now().isoformat()

        with self._lock:
            if self._last_tick_time is not None:
                dt = now_mono - self._last_tick_time
                if dt > 1e-4:
                    inst_hz = min(200.0, 1.0 / dt)
                    self._hz = inst_hz if self._hz <= 0.0 else (0.2 * inst_hz + 0.8 * self._hz)
            self._last_tick_time = now_mono

            d1, d2, d3 = self.tri.remap_distances(raw_d1, raw_d2, raw_d3)
            if self.use_filter:
                fd1, fd2, fd3 = self.dist_filter.update(d1, d2, d3)
            else:
                fd1, fd2, fd3 = d1, d2, d3

            pos, mode = self.tri.calculate_position(fd1, fd2, fd3)
            if pos is not None and self.use_filter:
                pos = self.pos_filter.update(pos)

            self._last_raw = (raw_d1, raw_d2, raw_d3)
            self._last_remapped = (d1, d2, d3)
            self._last_filtered = (fd1, fd2, fd3)
            self._last_pos = pos
            self._last_mode = mode

            if pos is not None:
                self.trajectory.append([_clean_float(pos[0]), _clean_float(pos[1])])
                if len(self.trajectory) > self.history_len:
                    del self.trajectory[: len(self.trajectory) - self.history_len]

            if self.csv_writer:
                px = f"{pos[0]:.4f}" if pos is not None else ""
                py = f"{pos[1]:.4f}" if pos is not None else ""
                relabel_flag = "SWAPPED" if self.tri.is_swapped else "NORMAL"
                self.csv_writer.writerow(
                    [ts, raw_d1, raw_d2, raw_d3, d1, d2, d3, px, py, mode, c12, c13, relabel_flag]
                )
                self._frame_count += 1
                if self.csv_file and (self._frame_count % 20 == 0):
                    self.csv_file.flush()

            self._publish_locked(ts)

    def trigger_recalib(self) -> Dict[str, Any]:
        """Arduino へ再キャリブレーション要求 ('c\\n') を送信する"""
        if hasattr(self.reader, "write"):
            self.reader.write(b"c\n")
            print("\n[WebGUI] Re-calibration button/key -> Sent 'c' to Arduino.")
        with self._lock:
            self._last_event = "再キャリブレーション要求を送信しました (c)"
            return self._publish_locked()

    def toggle_swap(self) -> Dict[str, Any]:
        """Port 2 / Port 3 の物理アンカー割り当てを手動で切り替える"""
        with self._lock:
            self.tri.is_swapped = not self.tri.is_swapped
            self.tri.relabel_mode = "swap" if self.tri.is_swapped else "normal"

            c12, c13 = self.latest_calib
            if c12 >= 0.10 and c13 >= 0.10:
                self.tri.update_anchors(c12, c13)
            else:
                self.tri.d12, self.tri.d13 = self.tri.d13, self.tri.d12
                self.tri.anchors = np.array(
                    [
                        (0.0, 0.0),
                        (0.0, self.tri.d12),
                        (self.tri.d13, 0.0),
                    ],
                    dtype=float,
                )

            self._reset_filters_locked()
            state_str = (
                "SWAPPED (Port 3->S2, Port 2->S3)"
                if self.tri.is_swapped
                else "NORMAL (Port 2->S2, Port 3->S3)"
            )
            print(f"\n[WebGUI] Manual mapping toggle -> {state_str}")
            self._last_event = f"ポート入替: {state_str}"
            return self._publish_locked()

    def clear_trajectory(self) -> Dict[str, Any]:
        """描画中の移動軌跡バッファをクリアする"""
        with self._lock:
            self.trajectory.clear()
            self._last_event = "軌跡をクリアしました"
            return self._publish_locked()

    def set_filter(self, enabled: bool) -> Dict[str, Any]:
        """測距・座標平滑化フィルタの有効/無効を切り替える"""
        with self._lock:
            self.use_filter = bool(enabled)
            self._reset_filters_locked()
            self._last_event = f"平滑化フィルタ: {'ON' if self.use_filter else 'OFF'}"
            return self._publish_locked()

    def set_history_len(self, length: int) -> Dict[str, Any]:
        """軌跡履歴の最大保持点数を変更する"""
        clamped = max(10, min(500, int(length)))
        with self._lock:
            self.history_len = clamped
            if len(self.trajectory) > self.history_len:
                del self.trajectory[: len(self.trajectory) - self.history_len]
            self._last_event = f"軌跡表示点数: {self.history_len} 点"
            return self._publish_locked()

    def get_state_dict(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._latest_state_dict)

    def get_state_json(self) -> Tuple[int, str]:
        with self._lock:
            return self._seq, self._latest_state_json

    def wait_for_next_state(self, last_seq: int, timeout: float = 1.0) -> Tuple[int, str]:
        """新しい状態シーケンスが生成されるかタイムアウトまで待機する"""
        with self._cond:
            if self._running and self._seq == last_seq:
                self._cond.wait(timeout=timeout)
            return self._seq, self._latest_state_json

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
        self.reader.start()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self.is_running:
            t0 = time.monotonic()
            try:
                self.step()
            except Exception as e:
                print(f"[WebGUI Engine Error] {e}")
            elapsed = time.monotonic() - t0
            sleep_time = self.update_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self) -> None:
        with self._lock:
            already_stopped = not self._running and self._thread is None
            self._running = False
            self._cond.notify_all()

        if not already_stopped:
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=1.5)
            self._thread = None
            try:
                self.reader.stop()
            except Exception as e:
                print(f"[WebGUI] Reader stop error: {e}")

        if self.csv_file:
            try:
                self.csv_file.flush()
                self.csv_file.close()
                print(f"[Visualizer] Log saved to {self.log_csv}")
            finally:
                self.csv_file = None
                self.csv_writer = None


class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class _GPSRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    engine: LocalGPSEngine  # server 属性またはクラスから参照

    def log_message(self, format: str, *args: Any) -> None:
        # ターミナルの Arduino ログを見やすく保つため HTTP アクセスログを抑制
        return

    def _get_engine(self) -> LocalGPSEngine:
        return getattr(self.server, "engine")

    def _send_bytes(self, status: int, content_type: str, body: bytes, cache_control: str = "no-cache") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache_control)
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body, "no-store")

    def _serve_static(self, filename: str, content_type: str) -> None:
        file_path = os.path.join(WEB_DIR, filename)
        if not os.path.isfile(file_path):
            self._send_bytes(404, "text/plain; charset=utf-8", b"Not Found")
            return
        with open(file_path, "rb") as f:
            data = f.read()
        self._send_bytes(200, content_type, data)

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        engine = self._get_engine()

        if path in ("/", "/index.html"):
            self._serve_static("index.html", "text/html; charset=utf-8")
        elif path == "/style.css":
            self._serve_static("style.css", "text/css; charset=utf-8")
        elif path == "/app.js":
            self._serve_static("app.js", "application/javascript; charset=utf-8")
        elif path == "/api/state":
            _, state_json = engine.get_state_json()
            self._send_bytes(200, "application/json; charset=utf-8", state_json.encode("utf-8"), "no-store")
        elif path == "/api/stream":
            self._handle_sse(engine)
        else:
            self._send_bytes(404, "text/plain; charset=utf-8", b"Not Found")

    def _handle_sse(self, engine: LocalGPSEngine) -> None:
        try:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()

            last_seq, state_json = engine.get_state_json()
            self.wfile.write(f"data: {state_json}\n\n".encode("utf-8"))
            self.wfile.flush()

            while engine.is_running:
                seq, payload = engine.wait_for_next_state(last_seq, timeout=1.0)
                if not engine.is_running:
                    break
                if seq != last_seq:
                    last_seq = seq
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                else:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            return

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        if path != "/api/action":
            self._send_json(404, {"ok": False, "error": "Not Found"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self._send_json(400, {"ok": False, "error": "Invalid Content-Length"})
            return

        if length <= 0 or length > 16384:
            self._send_json(400, {"ok": False, "error": "Invalid request body size"})
            return

        raw_body = self.rfile.read(length)
        try:
            data = json.loads(raw_body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"ok": False, "error": "Invalid JSON"})
            return

        if not isinstance(data, dict):
            self._send_json(400, {"ok": False, "error": "JSON object required"})
            return

        action = data.get("action")
        engine = self._get_engine()

        if action == "recalib":
            state = engine.trigger_recalib()
        elif action == "toggle_swap":
            state = engine.toggle_swap()
        elif action == "clear_trajectory":
            state = engine.clear_trajectory()
        elif action == "set_filter":
            if "enabled" not in data:
                self._send_json(400, {"ok": False, "error": "'enabled' field required"})
                return
            state = engine.set_filter(bool(data["enabled"]))
        elif action == "set_history":
            try:
                hist_len = int(data.get("history_len", 60))
            except (TypeError, ValueError):
                self._send_json(400, {"ok": False, "error": "Invalid 'history_len'"})
                return
            state = engine.set_history_len(hist_len)
        else:
            self._send_json(400, {"ok": False, "error": f"Unknown action: {action}"})
            return

        self._send_json(200, {"ok": True, "state": state})


class WebGPSVisualizer:
    """ブラウザ UI を提供する Local GPS サーバーラッパー"""

    def __init__(
        self,
        reader: Any,
        anchor1: Tuple[float, float] = (0.0, 0.0),
        anchor2: Tuple[float, float] = (0.0, 1.0),
        anchor3: Tuple[float, float] = (1.0, 0.0),
        nominal_d12: float = 1.0,
        nominal_d13: float = 1.0,
        relabel_mode: str = "auto",
        use_filter: bool = True,
        history_len: int = 60,
        log_csv: Optional[str] = None,
        plot_range: Tuple[float, float] = (-0.5, 4.0),
        host: str = "127.0.0.1",
        web_port: int = 8080,
        open_browser: bool = True,
        port_label: str = "",
        is_sim: bool = False,
    ):
        self.host = host
        self.requested_port = int(web_port)
        self.open_browser = bool(open_browser)
        self.bound_port = self.requested_port
        self.server: Optional[_ReusableThreadingHTTPServer] = None

        self.engine = LocalGPSEngine(
            reader=reader,
            anchor1=anchor1,
            anchor2=anchor2,
            anchor3=anchor3,
            nominal_d12=nominal_d12,
            nominal_d13=nominal_d13,
            relabel_mode=relabel_mode,
            use_filter=use_filter,
            history_len=history_len,
            log_csv=log_csv,
            initial_range=float(plot_range[1]),
            port_label=port_label,
            is_sim=is_sim,
        )

    def _bind_server(self) -> _ReusableThreadingHTTPServer:
        last_err: Optional[OSError] = None
        for port in range(self.requested_port, self.requested_port + 20):
            try:
                srv = _ReusableThreadingHTTPServer((self.host, port), _GPSRequestHandler)
                srv.engine = self.engine  # type: ignore[attr-defined]
                self.bound_port = port
                return srv
            except OSError as e:
                last_err = e
                if e.errno != errno.EADDRINUSE:
                    raise
        raise OSError(
            f"Could not bind WebGUI port in range {self.requested_port}-{self.requested_port + 19}: {last_err}"
        )

    def run(self) -> None:
        self.server = self._bind_server()
        display_host = "127.0.0.1" if self.host in ("0.0.0.0", "") else self.host
        url = f"http://{display_host}:{self.bound_port}"

        print(f"[WebGUI] Server running at: {url}")
        print("[WebGUI] Press Ctrl+C to stop.")

        self.engine.start()

        browser_timer: Optional[threading.Timer] = None
        if self.open_browser:
            browser_timer = threading.Timer(0.25, lambda: webbrowser.open(url))
            browser_timer.daemon = True
            browser_timer.start()

        try:
            self.server.serve_forever(poll_interval=0.25)
        except KeyboardInterrupt:
            print("\n[WebGUI] Shutting down...")
        finally:
            if browser_timer is not None:
                browser_timer.cancel()
            self.cleanup()

    def cleanup(self) -> None:
        self.engine.stop()
        if self.server is not None:
            try:
                self.server.server_close()
            except Exception:
                pass
            self.server = None
