"""
main.py
Local GPS - UWB 自己位置推定 リアルタイム可視化 & ロギングアプリケーション
(自動キャリブレーションフィードバック対応)
"""

import argparse
import csv
import datetime
import os
import sys

# 自ディレクトリをモジュール検索パスに追加 (どこから起動しても安全にインポート可能)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

from trilateration import DistanceFilter, MovingAverageFilter, Trilateration
from serial_reader import DummySerialReader, SerialReader, list_available_ports


class LocalGPSVisualizer:
    def __init__(
        self,
        reader,
        anchor1=(0.0, 0.0),
        anchor2=(0.0, 1.0),
        anchor3=(1.0, 0.0),
        use_filter=True,
        history_len=60,
        log_csv=None,
        plot_range=(-0.5, 4.0),
    ):
        self.reader = reader
        self.tri = Trilateration(anchor1, anchor2, anchor3)
        self.dist_filter = DistanceFilter(alpha=0.35) if use_filter else None
        self.filter = MovingAverageFilter(window_size=5) if use_filter else None
        self.history_len = history_len
        self.log_csv = log_csv
        self.csv_file = None
        self.csv_writer = None
        self.plot_range = plot_range
        self.latest_calib = (1.0, 1.0)

        # キャリブレーション受信コールバック登録
        if hasattr(self.reader, "on_calib_received"):
            self.reader.on_calib_received = self.on_calib_update

        if self.log_csv:
            self.csv_file = open(self.log_csv, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["timestamp", "d1_m", "d2_m", "d3_m", "est_x_m", "est_y_m", "mode", "calib_d12", "calib_d13"]
            )

        # 軌跡バッファ
        self.trajectory_x = []
        self.trajectory_y = []

        # Matplotlib 設定
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.canvas.manager.set_window_title("Local GPS - UWB Positioning Monitor (Auto-Calibrated)")

        # プロット要素の初期化
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_title("Realtime 2D Robot Position (UWB Trilateration)", fontsize=14, fontweight="bold")
        self.ax.set_xlabel("X [m]", fontsize=12)
        self.ax.set_ylabel("Y [m]", fontsize=12)
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_aspect("equal", adjustable="box")

        # 表示範囲
        rmin, rmax = self.plot_range
        self.ax.set_xlim(rmin, rmax)
        self.ax.set_ylim(rmin, rmax)

        # アンカープロット
        anchors = self.tri.anchors
        self.anchor_scatter = self.ax.scatter(
            anchors[:, 0],
            anchors[:, 1],
            c="royalblue",
            s=140,
            marker="s",
            zorder=5,
            edgecolors="navy",
            linewidths=1.5,
            label="Anchors (Slaves)",
        )
        self.anchor_texts = []
        for i, (ax_x, ax_y) in enumerate(anchors, start=1):
            t = self.ax.text(
                ax_x + 0.05,
                ax_y + 0.05,
                f"Slave {i} ({ax_x:.2f}, {ax_y:.2f})",
                fontsize=10,
                color="navy",
                fontweight="bold",
            )
            self.anchor_texts.append(t)

        # 軌跡ライン
        (self.line_traj,) = self.ax.plot(
            [], [], color="deepskyblue", linestyle="-", linewidth=1.8, alpha=0.65, label="Trajectory"
        )

        # 現在位置プロット (Tag / Robot)
        (self.scatter_robot,) = self.ax.plot(
            [], [], marker="o", color="limegreen", markersize=14, markeredgecolor="darkgreen",
            markeredgewidth=2.0, zorder=6, label="Robot (Tag)"
        )

        # 測距円 (各アンカーからの円)
        self.circle1 = Circle(anchors[0], 0, fill=False, color="crimson", linestyle=":", linewidth=1.2, alpha=0.5)
        self.circle2 = Circle(anchors[1], 0, fill=False, color="darkorange", linestyle=":", linewidth=1.2, alpha=0.5)
        self.circle3 = Circle(anchors[2], 0, fill=False, color="purple", linestyle=":", linewidth=1.2, alpha=0.5)
        self.ax.add_patch(self.circle1)
        self.ax.add_patch(self.circle2)
        self.ax.add_patch(self.circle3)

        # 情報表示テキスト (右上固定)
        self.info_text = self.ax.text(
            0.02,
            0.98,
            "Connecting...",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.88, edgecolor="gray"),
            zorder=10,
        )

        self.ax.legend(loc="lower right")

    def on_calib_update(self, d12_m, d13_m):
        """Arduino から CALIB,d12,d13 が届いた時の更新処理"""
        self.latest_calib = (d12_m, d13_m)
        self.tri.update_anchors(d12_m, d13_m)

        anchors = self.tri.anchors
        # プロット上のアンカー位置を更新
        self.anchor_scatter.set_offsets(anchors)
        self.circle1.center = tuple(anchors[0])
        self.circle2.center = tuple(anchors[1])
        self.circle3.center = tuple(anchors[2])

        for i, (ax_x, ax_y) in enumerate(anchors):
            self.anchor_texts[i].set_position((ax_x + 0.05, ax_y + 0.05))
            self.anchor_texts[i].set_text(f"Slave {i+1} ({ax_x:.2f}, {ax_y:.2f})")

    def update_frame(self):
        d1, d2, d3 = self.reader.latest_distances
        c12, c13 = getattr(self.reader, "latest_calib", self.latest_calib)

        # 1. 測距値ドメインでの平滑化 (UWBジッター除去 & 外れ値リジェクション)
        if self.dist_filter:
            fd1, fd2, fd3 = self.dist_filter.update(d1, d2, d3)
        else:
            fd1, fd2, fd3 = d1, d2, d3

        # 2. 三辺測量で位置推定 (ロバスト非線形最小二乗解)
        pos, mode = self.tri.calculate_position(fd1, fd2, fd3)

        # 3. 座標ドメインでの平滑化 (移動平均)
        if pos is not None and self.filter:
            pos = self.filter.update(pos)

        # CSV ロギング
        if self.csv_writer:
            ts = datetime.datetime.now().isoformat()
            px = f"{pos[0]:.4f}" if pos else ""
            py = f"{pos[1]:.4f}" if pos else ""
            self.csv_writer.writerow([ts, d1, d2, d3, px, py, mode, c12, c13])

        # プロット更新
        if pos is not None:
            self.trajectory_x.append(pos[0])
            self.trajectory_y.append(pos[1])
            if len(self.trajectory_x) > self.history_len:
                self.trajectory_x.pop(0)
                self.trajectory_y.pop(0)

            self.line_traj.set_data(self.trajectory_x, self.trajectory_y)
            self.scatter_robot.set_data([pos[0]], [pos[1]])
        else:
            self.scatter_robot.set_data([], [])

        # 測距円の更新
        if d1 > 0:
            self.circle1.set_radius(d1)
            self.circle1.set_visible(True)
        else:
            self.circle1.set_visible(False)

        if d2 > 0:
            self.circle2.set_radius(d2)
            self.circle2.set_visible(True)
        else:
            self.circle2.set_visible(False)

        if d3 > 0:
            self.circle3.set_radius(d3)
            self.circle3.set_visible(True)
        else:
            self.circle3.set_visible(False)

        # ステータステキスト更新
        conn_str = "CONNECTED" if getattr(self.reader, "is_connected", True) else "DISCONNECTED"
        d1_str = f"{d1:5.2f} m" if d1 > 0 else "  LOST  "
        d2_str = f"{d2:5.2f} m" if d2 > 0 else "  LOST  "
        d3_str = f"{d3:5.2f} m" if d3 > 0 else "  LOST  "

        status_str = (
            f"[LINK] {conn_str}\n"
            f"Slave 1 (d1): {d1_str}\n"
            f"Slave 2 (d2): {d2_str}\n"
            f"Slave 3 (d3): {d3_str}\n"
            f"───────────────────────────────\n"
            f"Calib S1-S2(Y): {c12:.2f} m\n"
            f"Calib S1-S3(X): {c13:.2f} m\n"
            f"───────────────────────────────\n"
        )
        if pos is not None:
            status_str += f"Robot: X={pos[0]:.3f} m, Y={pos[1]:.3f} m\nMode:  {mode}"
        else:
            status_str += f"Robot: Searching...\nMode:  {mode}"

        self.info_text.set_text(status_str)
        self.fig.canvas.draw_idle()

    def run(self):
        self.reader.start()

        timer = self.fig.canvas.new_timer(interval=50)
        timer.add_callback(self.update_frame)
        timer.start()

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        self.reader.stop()
        if self.csv_file:
            self.csv_file.close()
            print(f"[Visualizer] Log saved to {self.log_csv}")


def main():
    parser = argparse.ArgumentParser(description="Local GPS PC Visualizer with Auto-Calibration")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--spacing", type=float, default=1.0, help="Initial anchor spacing in meters (default: 1.0m)")
    parser.add_argument("--range", type=float, default=4.0, help="Plot range in meters (default: 4.0m)")
    parser.add_argument("--sim", action="store_true", help="Run simulation mode without Arduino hardware")
    parser.add_argument("--no-filter", action="store_true", help="Disable moving average filter")
    parser.add_argument("--log", type=str, default=None, help="Path to save log CSV (e.g. log.csv)")
    parser.add_argument("--history", type=int, default=60, help="Trajectory history length (default: 60)")

    args = parser.parse_args()

    s = args.spacing
    anchor1 = (0.0, 0.0)
    anchor2 = (0.0, s)
    anchor3 = (s, 0.0)

    if args.sim:
        print("[Mode] Running in SIMULATION mode.")
        reader = DummySerialReader(anchor1, anchor2, anchor3)
    else:
        port = args.port
        if port is None:
            available = list_available_ports()
            if not available:
                print("[Warning] No COM ports detected. Falling back to --sim mode.")
                reader = DummySerialReader(anchor1, anchor2, anchor3)
            else:
                port = available[0]
                print(f"[Serial] Using detected port: {port}")
                reader = SerialReader(port=port, baudrate=args.baud)
        else:
            reader = SerialReader(port=port, baudrate=args.baud)

    log_path = args.log
    if log_path is None:
        os.makedirs("logs", exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join("logs", f"ranging_log_{timestamp}.csv")

    rmin = -0.5
    rmax = max(args.range, s * 1.5)

    app = LocalGPSVisualizer(
        reader=reader,
        anchor1=anchor1,
        anchor2=anchor2,
        anchor3=anchor3,
        use_filter=not args.no_filter,
        history_len=args.history,
        log_csv=log_path,
        plot_range=(rmin, rmax),
    )
    app.run()


if __name__ == "__main__":
    main()
