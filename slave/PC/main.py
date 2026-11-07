"""
main.py
Local GPS - UWB 自己位置推定 リアルタイム可視化 & ロギングアプリケーション
(統合スイート版: 自動キャリブレーション + 動的リラベル + EKF / 物理運動フィルタ + GDOP 95% 信頼楕円)
"""

import argparse
import csv
import datetime
import math
import os
import sys

# 自ディレクトリをモジュール検索パスに追加 (どこから起動しても安全にインポート可能)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
import numpy as np

# Matplotlib 既定ショートカットの衝突をグローバルに無効化
# f: 全画面, k/l: X/Y軸対数スケール化, c: 戻る, r: ホーム, g: グリッド
for _key in [
    "keymap.fullscreen",
    "keymap.xscale",
    "keymap.yscale",
    "keymap.home",
    "keymap.back",
    "keymap.grid",
    "keymap.grid_minor",
]:
    plt.rcParams[_key] = []

from trilateration import (
    DistanceFilter,
    ExtendedKalmanFilter2D,
    KinematicPositionFilter,
    MovingAverageFilter,
    Trilateration,
)
from serial_reader import DummySerialReader, SerialReader, list_available_ports


class LocalGPSVisualizer:
    def __init__(
        self,
        reader,
        anchor1=(0.0, 0.0),
        anchor2=(0.0, 1.0),
        anchor3=(1.0, 0.0),
        nominal_d12=1.0,
        nominal_d13=1.0,
        relabel_mode="auto",
        filter_mode="ekf",
        history_len=60,
        log_csv=None,
        plot_range=(-0.5, 4.0),
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
        self.filter_mode = filter_mode.lower()  # "ekf", "kinematic", "moving_avg", or "raw"
        self.dist_filter = DistanceFilter(alpha=0.35) if self.filter_mode != "raw" else None
        self.filter = MovingAverageFilter(window_size=5)
        self.kinematic = KinematicPositionFilter(max_speed_mps=1.5)
        self.ekf = ExtendedKalmanFilter2D(sigma_a=1.0, sigma_r=0.05, sigma_pos=0.08)
        self.history_len = history_len
        self.log_csv = log_csv
        self.csv_file = None
        self.csv_writer = None
        self.plot_range = plot_range
        self.latest_calib = (anchor2[1], anchor3[0])
        self.show_gdop_ellipse = True
        self.latest_gdop = 1.0
        self.latest_gdop_rating = "EXCELLENT"

        # キャリブレーション受信コールバック登録
        if hasattr(self.reader, "on_calib_received"):
            self.reader.on_calib_received = self.on_calib_update

        if self.log_csv:
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
                    "raw_x_m",
                    "raw_y_m",
                    "est_x_m",
                    "est_y_m",
                    "vel_x_mps",
                    "vel_y_mps",
                    "speed_mps",
                    "heading_deg",
                    "hdop",
                    "gdop_rating",
                    "ellipse_major_m",
                    "ellipse_minor_m",
                    "mode",
                    "filter_mode",
                    "calib_d12",
                    "calib_d13",
                    "relabel",
                ]
            )

        # 軌跡バッファ
        self.trajectory_x = []
        self.trajectory_y = []

        # Matplotlib 既定ショートカットの衝突を無効化 (f:全画面, k:X軸対数スケール化, c:戻る, r:ホーム, g:グリッド)
        for conflict_key in [
            "keymap.fullscreen",
            "keymap.xscale",
            "keymap.yscale",
            "keymap.home",
            "keymap.back",
            "keymap.grid",
            "keymap.grid_minor",
        ]:
            plt.rcParams[conflict_key] = []

        # Matplotlib 設定
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.canvas.manager.set_window_title("Local GPS - UWB Positioning Monitor (Integrated Suite)")

        # プロット要素の初期化
        self.setup_plot()
        self.fig.canvas.mpl_connect("key_press_event", self.on_key_press)

    def on_key_press(self, event):
        if event.key in ("c", "C"):
            if hasattr(self.reader, "write"):
                self.reader.write(b"c\n")
                print("\n[GUI] 'C' key pressed -> Sent re-calibration request to Arduino.")
        elif event.key in ("r", "R"):
            # 手動リラベル切り替え (Toggle Swapped)
            self.tri.is_swapped = not self.tri.is_swapped
            self.tri.relabel_mode = "swap" if self.tri.is_swapped else "normal"
            if hasattr(self, "latest_calib"):
                self.tri.update_anchors(*self.latest_calib)
            else:
                self.tri.d12, self.tri.d13 = self.tri.d13, self.tri.d12
                self.tri.anchors = np.array([
                    (0.0, 0.0),
                    (0.0, self.tri.d12),
                    (self.tri.d13, 0.0),
                ], dtype=float)
            if self.dist_filter:
                self.dist_filter.reset()
            self.ekf.reset()
            self.kinematic.reset()
            self.on_calib_update(*getattr(self, "latest_calib", (self.tri.d12, self.tri.d13)))
            state_str = "SWAPPED (Port 3->S2, Port 2->S3)" if self.tri.is_swapped else "NORMAL (Port 2->S2, Port 3->S3)"
            print(f"\n[GUI] 'R' key pressed -> Manual mapping toggle: {state_str}")
        elif event.key in ("g", "G"):
            # GDOP 信頼楕円表示トグル
            self.show_gdop_ellipse = not self.show_gdop_ellipse
            print(f"\n[GUI] 'G' key pressed -> GDOP Confidence Ellipse: {'ON' if self.show_gdop_ellipse else 'OFF'}")
        elif event.key in ("f", "F"):
            # フィルタモード切り替え (EKF -> KINEMATIC -> MOVING_AVG -> RAW -> EKF)
            modes = ["ekf", "kinematic", "moving_avg", "raw"]
            idx = (modes.index(self.filter_mode) + 1) % len(modes) if self.filter_mode in modes else 0
            self.filter_mode = modes[idx]
            if self.filter_mode == "ekf":
                self.ekf.reset()
            elif self.filter_mode == "kinematic":
                self.kinematic.reset()
            elif self.filter_mode == "moving_avg":
                self.filter = MovingAverageFilter(window_size=5)
            print(f"\n[GUI] 'F' key pressed -> Filter mode switched to: {self.filter_mode.upper()}")
        elif event.key in ("k", "K"):
            # フィルタ状態リセット & 軸スケール正常化 (linear)
            self.ekf.reset()
            self.kinematic.reset()
            self.ax.set_xscale("linear")
            self.ax.set_yscale("linear")
            rmin, rmax = self.plot_range
            self.ax.set_xlim(rmin, rmax)
            self.ax.set_ylim(rmin, rmax)
            print("\n[GUI] 'K' key pressed -> Filters reset (linear scale confirmed).")

    def setup_plot(self):
        self.ax.set_title("Realtime 2D Robot Position (Integrated: EKF + GDOP + Kinematics)", fontsize=13, fontweight="bold")
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
            zorder=6,
            edgecolors="navy",
            linewidths=1.5,
            label="Anchors (Slaves)",
        )
        self.anchor_texts = []
        p2_port = "P3" if self.tri.is_swapped else "P2"
        p3_port = "P2" if self.tri.is_swapped else "P3"
        labels = [
            f"Slave 1 (0.00, 0.00)",
            f"Slave 2 [{p2_port}] ({anchors[1][0]:.2f}, {anchors[1][1]:.2f})",
            f"Slave 3 [{p3_port}] ({anchors[2][0]:.2f}, {anchors[2][1]:.2f})",
        ]
        for i, (ax_x, ax_y) in enumerate(anchors):
            t = self.ax.text(
                ax_x + 0.05,
                ax_y + 0.05,
                labels[i],
                fontsize=10,
                color="navy",
                fontweight="bold",
            )
            self.anchor_texts.append(t)

        # 軌跡ライン
        (self.line_traj,) = self.ax.plot(
            [], [], color="deepskyblue", linestyle="-", linewidth=1.8, alpha=0.65, label="Trajectory"
        )

        # 95% 信頼楕円 (Error Confidence Ellipse: GDOP 可視化)
        self.ellipse_confidence = Ellipse(
            (0, 0),
            width=0.2,
            height=0.2,
            angle=0,
            fill=True,
            facecolor="limegreen",
            edgecolor="darkgreen",
            alpha=0.35,
            linestyle="--",
            linewidth=1.6,
            zorder=4,
            label="Confidence (95%)",
        )
        self.ax.add_patch(self.ellipse_confidence)

        # 現在位置プロット (Tag / Robot)
        (self.scatter_robot,) = self.ax.plot(
            [], [], marker="o", color="limegreen", markersize=14, markeredgecolor="darkgreen",
            markeredgewidth=2.0, zorder=7, label="Robot (Tag)"
        )

        # 速度・進行方位ベクトル矢印 (Velocity / Heading Vector)
        (self.line_velocity,) = self.ax.plot(
            [], [], color="crimson", linewidth=2.5, zorder=8, label="Velocity Vector"
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
        """Arduino から CALIB,d12,d13 が届いた時の更新処理 (自動リラベル照合付き)"""
        self.latest_calib = (d12_m, d13_m)
        swapped_changed = self.tri.update_anchors(d12_m, d13_m)

        if swapped_changed:
            if self.dist_filter:
                self.dist_filter.reset()
            state_str = "SWAPPED (Relabeled: Port 3->S2, Port 2->S3)" if self.tri.is_swapped else "NORMAL (Port 2->S2, Port 3->S3)"
            print(f"\n[Auto-Relabel] Mapping updated -> {state_str}")

        anchors = self.tri.anchors
        # プロット上のアンカー位置を更新
        self.anchor_scatter.set_offsets(anchors)
        self.circle1.center = tuple(anchors[0])
        self.circle2.center = tuple(anchors[1])
        self.circle3.center = tuple(anchors[2])

        p2_port = "P3" if self.tri.is_swapped else "P2"
        p3_port = "P2" if self.tri.is_swapped else "P3"
        self.anchor_texts[0].set_text(f"Slave 1 ({anchors[0][0]:.2f}, {anchors[0][1]:.2f})")
        self.anchor_texts[1].set_position((anchors[1][0] + 0.05, anchors[1][1] + 0.05))
        self.anchor_texts[1].set_text(f"Slave 2 [{p2_port}] ({anchors[1][0]:.2f}, {anchors[1][1]:.2f})")
        self.anchor_texts[2].set_position((anchors[2][0] + 0.05, anchors[2][1] + 0.05))
        self.anchor_texts[2].set_text(f"Slave 3 [{p3_port}] ({anchors[2][0]:.2f}, {anchors[2][1]:.2f})")

    def update_frame(self):
        raw_d1, raw_d2, raw_d3 = self.reader.latest_distances
        c12, c13 = getattr(self.reader, "latest_calib", self.latest_calib)

        # 0. 物理ポート -> 論理アンカーのリラベル適用 (入れ替え設置の自動補正)
        d1, d2, d3 = self.tri.remap_distances(raw_d1, raw_d2, raw_d3)

        # 1. 測距値ドメインでの平滑化 (UWBジッター除去 & 外れ値リジェクション)
        if self.dist_filter:
            fd1, fd2, fd3 = self.dist_filter.update(d1, d2, d3)
        else:
            fd1, fd2, fd3 = d1, d2, d3

        # 2. 三辺測量で位置推定 (ロバスト非線形最小二乗解)
        raw_pos, mode = self.tri.calculate_position(fd1, fd2, fd3)

        pos = raw_pos
        vx, vy, speed, heading = 0.0, 0.0, 0.0, 0.0

        if self.filter_mode == "ekf":
            self.ekf.predict()
            if raw_pos is not None:
                if not self.ekf.is_initialized:
                    self.ekf.reset(initial_pos=raw_pos)
                pos = self.ekf.update_position(raw_pos)
            elif self.ekf.is_initialized:
                # 欠損時も残りの有効アンカー測距値で EKF 更新
                pos = self.ekf.update_ranges(fd1, fd2, fd3, self.tri.anchors)
                mode = "EKF_DR_FALLBACK"

            if self.ekf.is_initialized:
                px, py, vx, vy, speed, heading = self.ekf.get_state()
                pos = (px, py)
        elif self.filter_mode == "kinematic":
            if raw_pos is not None:
                pos, (vx, vy), speed, heading = self.kinematic.update(raw_pos)
            else:
                pos, (vx, vy), speed, heading = self.kinematic.update(None)
        elif self.filter_mode == "moving_avg":
            if pos is not None and self.filter:
                pos = self.filter.update(pos)

        # 4. GDOP (幾何学的精度低下率) & 95% 信頼楕円の算出
        hdop = 1.0
        semi_major = 0.1
        semi_minor = 0.1
        angle_deg = 0.0
        gdop_rating = "EXCELLENT"

        if pos is not None:
            hdop, semi_major, semi_minor, angle_deg, gdop_rating = self.tri.calculate_gdop(pos)
            self.latest_gdop = hdop
            self.latest_gdop_rating = gdop_rating

            if self.show_gdop_ellipse:
                self.ellipse_confidence.center = pos
                # Ellipse の width, height は全幅・全高 (2 * 半径)
                self.ellipse_confidence.width = 2.0 * semi_major
                self.ellipse_confidence.height = 2.0 * semi_minor
                self.ellipse_confidence.angle = angle_deg

                # 精度評価に応じた色分け (良: 緑, 中: 黄, 警戒: 橙, 危険: 赤)
                if gdop_rating == "EXCELLENT":
                    self.ellipse_confidence.set_facecolor("limegreen")
                    self.ellipse_confidence.set_edgecolor("darkgreen")
                elif gdop_rating == "GOOD":
                    self.ellipse_confidence.set_facecolor("gold")
                    self.ellipse_confidence.set_edgecolor("darkgoldenrod")
                elif gdop_rating == "MODERATE":
                    self.ellipse_confidence.set_facecolor("darkorange")
                    self.ellipse_confidence.set_edgecolor("orangered")
                else:
                    self.ellipse_confidence.set_facecolor("crimson")
                    self.ellipse_confidence.set_edgecolor("darkred")

                self.ellipse_confidence.set_visible(True)
            else:
                self.ellipse_confidence.set_visible(False)
        else:
            self.ellipse_confidence.set_visible(False)

        # CSV ロギング
        if self.csv_writer:
            ts = datetime.datetime.now().isoformat()
            r_px = f"{raw_pos[0]:.4f}" if raw_pos else ""
            r_py = f"{raw_pos[1]:.4f}" if raw_pos else ""
            px_str = f"{pos[0]:.4f}" if pos else ""
            py_str = f"{pos[1]:.4f}" if pos else ""
            relabel_flag = "SWAPPED" if self.tri.is_swapped else "NORMAL"
            self.csv_writer.writerow(
                [
                    ts,
                    raw_d1,
                    raw_d2,
                    raw_d3,
                    d1,
                    d2,
                    d3,
                    r_px,
                    r_py,
                    px_str,
                    py_str,
                    f"{vx:.3f}",
                    f"{vy:.3f}",
                    f"{speed:.3f}",
                    f"{heading:.1f}",
                    f"{hdop:.2f}",
                    gdop_rating,
                    f"{semi_major:.3f}",
                    f"{semi_minor:.3f}",
                    mode,
                    self.filter_mode.upper(),
                    c12,
                    c13,
                    relabel_flag,
                ]
            )

        # プロット更新
        if pos is not None:
            self.trajectory_x.append(pos[0])
            self.trajectory_y.append(pos[1])
            if len(self.trajectory_x) > self.history_len:
                self.trajectory_x.pop(0)
                self.trajectory_y.pop(0)

            self.line_traj.set_data(self.trajectory_x, self.trajectory_y)
            self.scatter_robot.set_data([pos[0]], [pos[1]])

            # 速度・進行方位ベクトル矢印の更新
            if self.filter_mode in ("ekf", "kinematic") and speed > 0.05:
                arrow_len = min(0.8, max(0.15, speed * 0.6))
                rad = math.radians(heading)
                dx = arrow_len * math.cos(rad)
                dy = arrow_len * math.sin(rad)
                self.line_velocity.set_data([pos[0], pos[0] + dx], [pos[1], pos[1] + dy])
            else:
                self.line_velocity.set_data([], [])
        else:
            self.scatter_robot.set_data([], [])
            self.line_velocity.set_data([], [])

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

        p2_port = "Port 3" if self.tri.is_swapped else "Port 2"
        p3_port = "Port 2" if self.tri.is_swapped else "Port 3"
        relabel_desc = "RELABELED (P3->S2, P2->S3)" if self.tri.is_swapped else "NORMAL (P2->S2, P3->S3)"

        status_str = (
            f"[LINK] {conn_str}\n"
            f"Slave 1 (d1): {d1_str}\n"
            f"Slave 2 [{p2_port}]: {d2_str}\n"
            f"Slave 3 [{p3_port}]: {d3_str}\n"
            f"───────────────────────────────\n"
            f"Mapping: {relabel_desc}\n"
            f"Calib S2(Y): {self.tri.d12:.2f} m (Nom: {self.tri.nominal_d12:.2f}m)\n"
            f"Calib S3(X): {self.tri.d13:.2f} m (Nom: {self.tri.nominal_d13:.2f}m)\n"
            f"[C] Calib | [R] Swap | [G] GDOP | [F] {self.filter_mode.upper()}\n"
            f"───────────────────────────────\n"
        )
        if pos is not None:
            gdop_badge = f"GDOP:  {hdop:.1f} [{gdop_rating}] (±{semi_major:.2f}m)"
            vel_str = f"Vel:   {speed:.2f} m/s ({heading:.1f}°)" if speed > 0.05 else "Vel:   0.00 m/s (Stationary)"
            status_str += f"Robot: X={pos[0]:.3f} m, Y={pos[1]:.3f} m\n{gdop_badge}\n{vel_str}\nMode:  {mode}"
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
    parser = argparse.ArgumentParser(description="Local GPS PC Visualizer with Auto-Calibration & Dynamic Relabeling")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--spacing", type=float, default=1.0, help="Default anchor spacing in meters (default: 1.0m)")
    parser.add_argument("--spacing-y", type=float, default=None, help="Nominal Y-axis (Slave 2) spacing in meters")
    parser.add_argument("--spacing-x", type=float, default=None, help="Nominal X-axis (Slave 3) spacing in meters")
    parser.add_argument(
        "--relabel",
        choices=["auto", "normal", "swap"],
        default="auto",
        help="Relabeling mode: auto (match with nominal), normal (Port2=S2, Port3=S3), swap (Port3=S2, Port2=S3)",
    )
    parser.add_argument("--range", type=float, default=4.0, help="Plot range in meters (default: 4.0m)")
    parser.add_argument("--sim", action="store_true", help="Run simulation mode without Arduino hardware")
    parser.add_argument(
        "--filter",
        choices=["ekf", "kinematic", "moving_avg", "raw"],
        default="ekf",
        help="Filtering mode: ekf (Extended Kalman Filter), kinematic (Physical velocity clamping), moving_avg, raw",
    )
    parser.add_argument("--log", type=str, default=None, help="Path to save log CSV (e.g. log.csv)")
    parser.add_argument("--history", type=int, default=60, help="Trajectory history length (default: 60)")

    args = parser.parse_args()

    sy = args.spacing_y if args.spacing_y is not None else args.spacing
    sx = args.spacing_x if args.spacing_x is not None else args.spacing
    anchor1 = (0.0, 0.0)
    anchor2 = (0.0, sy)
    anchor3 = (sx, 0.0)

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
    rmax = max(args.range, max(sy, sx) * 1.5)

    app = LocalGPSVisualizer(
        reader=reader,
        anchor1=anchor1,
        anchor2=anchor2,
        anchor3=anchor3,
        nominal_d12=sy,
        nominal_d13=sx,
        relabel_mode=args.relabel,
        filter_mode=args.filter,
        history_len=args.history,
        log_csv=log_path,
        plot_range=(rmin, rmax),
    )
    app.run()


if __name__ == "__main__":
    main()
