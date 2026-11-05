"""
trilateration.py
UWB 測距データに基づく2次元自己位置推定 (三辺測量) モジュール
- 3点アンカーによる線形最小二乗解
- 1点欠損時 (2点有効時) の幾何学的フォールバック推定
- 座標平滑化用移動平均フィルタ
"""

import math
from typing import Optional, Tuple
import numpy as np


class Trilateration:
    def __init__(
        self,
        anchor1: Tuple[float, float] = (0.0, 0.0),
        anchor2: Tuple[float, float] = (0.0, 1.0),
        anchor3: Tuple[float, float] = (1.0, 0.0),
    ):
        """三辺測量計算クラスの初期化

        Parameters
        ----------
        anchor1 : Tuple[float, float]
            Slave 1 の (x, y) 座標 [m] (デフォルト: 原点 (0.0, 0.0))
        anchor2 : Tuple[float, float]
            Slave 2 の (x, y) 座標 [m] (デフォルト: Y軸上 (0.0, 1.0))
        anchor3 : Tuple[float, float]
            Slave 3 の (x, y) 座標 [m] (デフォルト: X軸上 (1.0, 0.0))
        """
        self.anchors = np.array([anchor1, anchor2, anchor3], dtype=float)

    def calculate_position(
        self, d1: float, d2: float, d3: float, allow_fallback: bool = True
    ) -> Tuple[Optional[Tuple[float, float]], str]:
        """測距データから Tag の 2D 座標 (x, y) [m] を算出する。

        Parameters
        ----------
        d1, d2, d3 : float
            Slave 1, 2, 3 からの測定距離 [m] (エラー・タイムアウト時は <= 0)
        allow_fallback : bool
            1点欠損時に2点から推定するフォールバックを許可するか

        Returns
        -------
        pos : Optional[Tuple[float, float]]
            推定された (x, y) [m]。推定不可能な場合は None
        mode : str
            推定モード ("3_ANCHORS", "FALLBACK_12", "FALLBACK_13", "FALLBACK_23", "ERROR")
        """
        v1 = d1 > 0.0
        v2 = d2 > 0.0
        v3 = d3 > 0.0

        # 1. 3点すべて有効な場合 (標準: 線形最小二乗解)
        if v1 and v2 and v3:
            pos = self._solve_3_anchors(d1, d2, d3)
            if pos is not None:
                return pos, "3_ANCHORS"

        if not allow_fallback:
            return None, "ERROR"

        # 2. 1点欠損フォールバック (2点のみ有効な場合)
        if v1 and v2:  # Slave 1 (0,0) と Slave 2 (0, y2)
            pos = self._solve_2_anchors_12(d1, d2)
            if pos is not None:
                return pos, "FALLBACK_12"

        if v1 and v3:  # Slave 1 (0,0) と Slave 3 (x3, 0)
            pos = self._solve_2_anchors_13(d1, d3)
            if pos is not None:
                return pos, "FALLBACK_13"

        if v2 and v3:  # Slave 2 (0, y2) と Slave 3 (x3, 0)
            pos = self._solve_2_anchors_23(d2, d3)
            if pos is not None:
                return pos, "FALLBACK_23"

        return None, "ERROR"

    def _solve_3_anchors(self, d1: float, d2: float, d3: float) -> Optional[Tuple[float, float]]:
        """3点アンカーによる連立方程式の線形最小二乗解"""
        x1, y1 = self.anchors[0]
        x2, y2 = self.anchors[1]
        x3, y3 = self.anchors[2]

        A = np.array(
            [
                [2.0 * (x2 - x1), 2.0 * (y2 - y1)],
                [2.0 * (x3 - x1), 2.0 * (y3 - y1)],
            ],
            dtype=float,
        )

        b = np.array(
            [
                d1**2 - d2**2 + (x2**2 + y2**2) - (x1**2 + y1**2),
                d1**2 - d3**2 + (x3**2 + y3**2) - (x1**2 + y1**2),
            ],
            dtype=float,
        )

        try:
            pos, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            return float(pos[0]), float(pos[1])
        except np.linalg.LinAlgError:
            return None

    def _solve_2_anchors_12(self, d1: float, d2: float) -> Optional[Tuple[float, float]]:
        """Slave 1 (0, 0) と Slave 2 (0, y2) からの2点幾何推定"""
        y2 = self.anchors[1][1]
        if abs(y2) < 1e-6:
            return None

        # y 座標は一意に決まる: y = (d1^2 - d2^2 + y2^2) / (2 * y2)
        y = (d1**2 - d2**2 + y2**2) / (2.0 * y2)

        # x^2 = d1^2 - y^2 (ロボットは通常 x >= 0 領域に存在する前提)
        radicand = d1**2 - y**2
        if radicand < -0.1:  # 測距誤差で許容できる範囲外
            return None
        x = math.sqrt(max(0.0, radicand))
        return x, y

    def _solve_2_anchors_13(self, d1: float, d3: float) -> Optional[Tuple[float, float]]:
        """Slave 1 (0, 0) と Slave 3 (x3, 0) からの2点幾何推定"""
        x3 = self.anchors[2][0]
        if abs(x3) < 1e-6:
            return None

        # x 座標は一意に決まる: x = (d1^2 - d3^2 + x3^2) / (2 * x3)
        x = (d1**2 - d3**2 + x3**2) / (2.0 * x3)

        # y^2 = d1^2 - x^2 (ロボットは通常 y >= 0 領域に存在する前提)
        radicand = d1**2 - x**2
        if radicand < -0.1:
            return None
        y = math.sqrt(max(0.0, radicand))
        return x, y

    def _solve_2_anchors_23(self, d2: float, d3: float) -> Optional[Tuple[float, float]]:
        """Slave 2 (0, y2) と Slave 3 (x3, 0) からの2点幾何交点推定"""
        x2, y2 = self.anchors[1]
        x3, y3 = self.anchors[2]

        # 2点間の距離
        D = math.hypot(x3 - x2, y3 - y2)
        if D < 1e-6 or (d2 + d3) < D * 0.8:
            return None

        # 円の交点算出
        a = (d2**2 - d3**2 + D**2) / (2.0 * D)
        h_sq = d2**2 - a**2
        h = math.sqrt(max(0.0, h_sq))

        # 中間点
        mx = x2 + a * (x3 - x2) / D
        my = y2 + a * (y3 - y2) / D

        # 2つの交点候補 (法線方向)
        rx1 = mx + h * (y3 - y2) / D
        ry1 = my - h * (x3 - x2) / D
        rx2 = mx - h * (y3 - y2) / D
        ry2 = my + h * (x3 - x2) / D

        # 原点に近い方（通常第1象限内部）を採用
        dist_sq1 = rx1**2 + ry1**2
        dist_sq2 = rx2**2 + ry2**2
        return (rx1, ry1) if dist_sq1 < dist_sq2 else (rx2, ry2)


class MovingAverageFilter:
    """座標平滑化用の単純移動平均フィルタ"""

    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.history_x = []
        self.history_y = []

    def update(
        self, pos: Optional[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        if pos is None:
            return None
        self.history_x.append(pos[0])
        self.history_y.append(pos[1])

        if len(self.history_x) > self.window_size:
            self.history_x.pop(0)
            self.history_y.pop(0)

        avg_x = sum(self.history_x) / len(self.history_x)
        avg_y = sum(self.history_y) / len(self.history_y)
        return avg_x, avg_y


if __name__ == "__main__":
    # 単体テスト
    tri = Trilateration()
    d = math.sqrt(0.5)

    # 1. 3点正常
    p3, mode3 = tri.calculate_position(d, d, d)
    print(f"3-Anchor Test: pos={p3}, mode={mode3}")
    assert p3 is not None and mode3 == "3_ANCHORS"
    assert abs(p3[0] - 0.5) < 1e-3 and abs(p3[1] - 0.5) < 1e-3

    # 2. Slave 3 欠損 (Slave 1 & 2 のみ)
    p2_12, mode2_12 = tri.calculate_position(d, d, -1.0)
    print(f"Fallback 1-2 Test: pos={p2_12}, mode={mode2_12}")
    assert p2_12 is not None and mode2_12 == "FALLBACK_12"
    assert abs(p2_12[0] - 0.5) < 1e-3 and abs(p2_12[1] - 0.5) < 1e-3

    # 3. Slave 2 欠損 (Slave 1 & 3 のみ)
    p2_13, mode2_13 = tri.calculate_position(d, -1.0, d)
    print(f"Fallback 1-3 Test: pos={p2_13}, mode={mode2_13}")
    assert p2_13 is not None and mode2_13 == "FALLBACK_13"
    assert abs(p2_13[0] - 0.5) < 1e-3 and abs(p2_13[1] - 0.5) < 1e-3

    print("All Trilateration tests passed!")
