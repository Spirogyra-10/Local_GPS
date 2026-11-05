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
        history_len=60,
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
                ["timestamp", "d1_m", "d2_m", "d3_m", "est_x_m", "est_y_m", "mode"]
            )

        # 軌跡バッファ
        self.trajectory_x = []
        self.trajectory_y = []

        # Matplotlib 設定
        self.fig, self.ax = plt.subplots(figsize=(8.5, 8.5))
        self.fig.canvas.manager.set_window_title("Local GPS - UWB Positioning Monitor")

        # プロット要素の初期化
        self.setup_plot()

    def setup_plot(self):
        self.ax.set_title("Realtime 2D Robot Position (UWB Trilateration)", fontsize=14, fontweight="bold")
        self.ax.set_xlabel("X [m]", fontsize=12)
        self.ax.set_ylabel("Y [m]", fontsize=12)
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_aspect("equal", adjustable="box")

        # 表示範囲
        self.ax.set_xlim(-0.4, 1.6)
        self.ax.set_ylim(-0.4, 1.6)

        # アンカープロット
        anchors = self.tri.anchors
        self.ax.scatter(
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
            [], [], color="deepskyblue", linestyle="-", linewidth=1.8, alpha=0.65, label="Trajectory"
        )

        # 現在位置プロット (Tag / Robot)
        (self.scatter_robot,) = self.ax.plot(
            [], [], marker="o", color="limegreen", markersize=12, markeredgecolor="darkgreen",
            markeredgewidth=1.5, zorder=6, label="Robot (Tag)"
        )

        # 測距円 (各アンカーからの円)
        self.circle1 = Circle(anchors[0], 0, fill=False, color="crimson", linestyle=":", linewidth=1.2, alpha=0.5)
        self.circle2 = Circle(anchors[1], 0, fill=False, color="darkorange", linestyle=":", linewidth=1.2, alpha=0.5)
        self.circle3 = Circle(anchors[2], 0, fill=False, color="purple", linestyle=":", linewidth=1.2, alpha=0.5)
        self.ax.add_patch(self.circle1)
        self.ax.add_patch(self.circle2)
        self.ax.add_patch(self.circle3)

        # 情報表示テキスト
        self.info_text = self.ax.text(
            0.03,
            0.96,
            "",
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment="top",
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.6", facecolor="white", alpha=0.9, edgecolor="lightgray"),
        )

        self.ax.legend(loc="upper right", framealpha=0.9)

    def update_frame(self):
        d1, d2, d3 = self.reader.get_latest_distances()
        raw_pos, mode = self.tri.calculate_position(d1, d2, d3)

        pos = self.filter.update(raw_pos) if self.filter else raw_pos

        # ロギング
        if self.csv_writer:
            now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            x_val = f"{pos[0]:.4f}" if pos else ""
            y_val = f"{pos[1]:.4f}" if pos else ""
            self.csv_writer.writerow([now_str, f"{d1:.3f}", f"{d2:.3f}", f"{d3:.3f}", x_val, y_val, mode])

        # 描画更新
        if pos is not None:
            self.trajectory_x.append(pos[0])
            self.trajectory_y.append(pos[1])
            if len(self.trajectory_x) > self.history_len:
                self.trajectory_x.pop(0)
                self.trajectory_y.pop(0)

            self.line_traj.set_data(self.trajectory_x, self.trajectory_y)
            self.scatter_robot.set_data([pos[0]], [pos[1]])

            # モードによるマーカー色の変更
            if mode == "3_ANCHORS":
                self.scatter_robot.set_color("limegreen")
                self.scatter_robot.set_markeredgecolor("darkgreen")
            else:
                self.scatter_robot.set_color("gold")
                self.scatter_robot.set_markeredgecolor("darkorange")
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
            f"────────────────────────\n"
        )
        if pos is not None:
            status_str += f"Robot: X={pos[0]:.3f} m, Y={pos[1]:.3f} m\nMode:  {mode}"
        else:
            status_str += f"Robot: Searching...\nMode:  {mode}"

        self.info_text.set_text(status_str)
        self.fig.canvas.draw_idle()

    def run(self):
        self.reader.start()

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
    parser.add_argument("--history", type=int, default=60, help="Trajectory history length (default: 60)")

    args = parser.parse_args()

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
