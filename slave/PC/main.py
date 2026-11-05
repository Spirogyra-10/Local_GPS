"""
main.py
Local GPS - UWB 自己位置推定 リアルタイム可視化 & ロギングアプリケーション
"""

import argparse
import csv
import datetime
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

from trilateration import MovingAverageFilter, Trilateration
from serial_reader import DummySerialReader, SerialReader, list_available_ports


class LocalGPSVisualizer:
    def __init__(
        self,
        reader,
        anchor1=(0.0, 0.0),
        anchor2=(0.0, 1.0),
        anchor3=(1.0, 0.0),
        use_filter=True,
        history_len=50,
        log_csv=None,
    ):
        self.reader = reader
        self.tri = Trilateration(anchor1, anchor2, anchor3)
        self.filter = MovingAverageFilter(window_size=5) if use_filter else None
        self.history_len = history_len
        self.log_csv = log_csv
        self.csv_file = None
        self.csv_writer = None

        if self.log_csv:
            self.csv_file = open(self.log_csv, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["timestamp", "d1_m", "d2_m", "d3_m", "est_x_m", "est_y_m"]
            )

        # 軌跡バッファ
        self.trajectory_x = []
        self.trajectory_y = []

        # Matplotlib 設定
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Local GPS - UWB Positioning Monitor")

        # プロット要素の初期化
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_title("Realtime 2D Robot Position (UWB Trilateration)", fontsize=14, fontweight="bold")
        self.ax.set_xlabel("X [m]", fontsize=12)
        self.ax.set_ylabel("Y [m]", fontsize=12)
        self.ax.grid(True, linestyle="--", alpha=0.6)
        self.ax.set_aspect("equal", adjustable="box")

        # 表示範囲 (0〜1.5m 程度にマージンを持たせる)
        self.ax.set_xlim(-0.5, 1.8)
        self.ax.set_ylim(-0.5, 1.8)

        # アンカープロット
        anchors = self.tri.anchors
        self.ax.scatter(
            anchors[:, 0],
            anchors[:, 1],
            c="blue",
            s=120,
            marker="s",
            zorder=5,
            label="Anchors (Slaves)",
        )
        for i, (ax_x, ax_y) in enumerate(anchors, start=1):
            self.ax.text(
                ax_x + 0.04,
                ax_y + 0.04,
                f"Slave {i} ({ax_x:.1f}, {ax_y:.1f})",
                fontsize=10,
                color="navy",
                fontweight="bold",
            )

        # 軌跡ライン
        (self.line_traj,) = self.ax.plot(
            [], [], color="deepskyblue", linestyle="-", linewidth=1.5, alpha=0.7, label="Trajectory"
        )

        # 現在位置プロット (Tag / Robot)
        (self.scatter_robot,) = self.ax.plot(
            [], [], marker="o", color="red", markersize=12, zorder=6, label="Robot (Tag)"
        )

        # 測距円 (各アンカーからの円)
        self.circle1 = Circle(anchors[0], 0, fill=False, color="gray", linestyle=":", alpha=0.5)
        self.circle2 = Circle(anchors[1], 0, fill=False, color="gray", linestyle=":", alpha=0.5)
        self.circle3 = Circle(anchors[2], 0, fill=False, color="gray", linestyle=":", alpha=0.5)
        self.ax.add_patch(self.circle1)
        self.ax.add_patch(self.circle2)
        self.ax.add_patch(self.circle3)

        # 情報表示テキスト
        self.info_text = self.ax.text(
            0.03,
            0.95,
            "",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.85, edgecolor="lightgray"),
        )

        self.ax.legend(loc="upper right", framealpha=0.9)

    def update_frame(self):
        d1, d2, d3 = self.reader.get_latest_distances()
        raw_pos = self.tri.calculate_position(d1, d2, d3)

        pos = self.filter.update(raw_pos) if self.filter else raw_pos

        # ロギング
        if self.csv_writer:
            now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            x_val = pos[0] if pos else ""
            y_val = pos[1] if pos else ""
            self.csv_writer.writerow([now_str, d1, d2, d3, x_val, y_val])

        # 描画更新
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
        status_str = (
            f"Slave 1 (d1): {d1:.2f} m\n"
            f"Slave 2 (d2): {d2:.2f} m\n"
            f"Slave 3 (d3): {d3:.2f} m\n"
            f"────────────────\n"
        )
        if pos is not None:
            status_str += f"Robot Pos: X={pos[0]:.3f} m, Y={pos[1]:.3f} m"
        else:
            status_str += "Robot Pos: Estimating..."

        self.info_text.set_text(status_str)
        self.fig.canvas.draw_idle()

    def run(self):
        self.reader.start()

        # タイマーによる描画更新 (30fps: ~33ms)
        timer = self.fig.canvas.new_timer(interval=33)
        timer.add_callback(self.update_frame)
        timer.start()

        try:
            plt.show()
        finally:
            self.close()

    def close(self):
        self.reader.stop()
        if self.csv_file:
            self.csv_file.close()
            print(f"[Visualizer] Log saved to {self.log_csv}")


def main():
    parser = argparse.ArgumentParser(description="Local GPS PC Visualizer & Position Estimator")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--sim", action="store_true", help="Run simulation mode without Arduino hardware")
    parser.add_argument("--no-filter", action="store_true", help="Disable moving average filter")
    parser.add_argument("--log", type=str, default=None, help="Path to save log CSV (e.g. log.csv)")
    parser.add_argument("--history", type=int, default=50, help="Trajectory history length (default: 50)")

    args = parser.parse_args()

    # シミュレーションモードか実機シリアルかの判定
    if args.sim:
        print("[Mode] Running in SIMULATION mode.")
        reader = DummySerialReader()
    else:
        if args.port is None:
            available = list_available_ports()
            if not available:
                print("[Warning] No COM ports detected. Falling back to --sim mode.")
                reader = DummySerialReader()
            else:
                print(f"[Serial] Available ports: {available}")
                port = available[0]
                print(f"[Serial] Using port: {port}")
                reader = SerialReader(port=port, baudrate=args.baud)
        else:
            reader = SerialReader(port=args.port, baudrate=args.baud)

    # ログファイル名自動生成 (指定されていない場合でもログフォルダに保存可能)
    log_path = args.log
    if log_path is None:
        os.makedirs("logs", exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join("logs", f"ranging_log_{timestamp}.csv")

    app = LocalGPSVisualizer(
        reader=reader,
        use_filter=not args.no_filter,
        history_len=args.history,
        log_csv=log_path,
    )
    app.run()


if __name__ == "__main__":
    main()

