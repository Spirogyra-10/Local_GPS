"""
trilateration.py
UWB 測距データに基づく2次元自己位置推定 (三辺測量) モジュール
- 3点アンカーによるロバスト非線形最小二乗解 (Levenberg-Marquardt / SciPy NLS)
- 短基線 (High GDOP) 環境に対応した幾何学的制約と解の安定化
- 1点欠損時 (2点有効時) の幾何学的フォールバック推定 (幾何学的クランプ付き)
- 測距値・座標平滑化フィルタ (DistanceFilter, MovingAverageFilter)
"""

import math
from typing import Optional, Tuple
import numpy as np

try:
    from scipy.optimize import least_squares
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


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

    def update_anchors(self, d12: float, d13: float):
        """自動キャリブレーションで得られた実測基準線長からアンカー座標を動的に更新する

        Parameters
        ----------
        d12 : float
            Slave 1 から Slave 2 までの実測距離 [m] (Y軸アンカー)
        d13 : float
            Slave 1 から Slave 3 までの実測距離 [m] (X軸アンカー)
        """
        # 極端な近接飽和ノイズ (10cm未満) や異常値を排除
        if d12 >= 0.10 and d13 >= 0.10:
            self.anchors = np.array([
                (0.0, 0.0),
                (0.0, float(d12)),
                (float(d13), 0.0),
            ], dtype=float)

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

        # 1. 3点すべて有効な場合 (ロバスト非線形最小二乗法)
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
        """3点アンカーによるロバスト非線形最小二乗法 (NLS) による自己位置推定"""
        dists = np.array([d1, d2, d3], dtype=float)
        anchors = self.anchors  # shape: (3, 2)

        # 初期値推定 (Initial Guess):
        # 原点からの距離 d1 を基準に、アンカー間距離差から方向角を近似
        y2 = max(0.05, float(anchors[1, 1]))
        x3 = max(0.05, float(anchors[2, 0]))

        # 方向余弦の初期推定: cos(theta) ~= (d1 - d3) / x3, sin(theta) ~= (d1 - d2) / y2
        cos_est = np.clip((d1 - d3) / x3, -1.0, 1.0)
        sin_est = np.clip((d1 - d2) / y2, -1.0, 1.0)

        # 第一象限（正の領域）へのプロジェクション
        if cos_est <= 0 and sin_est <= 0:
            init_x = d1 / math.sqrt(2.0)
            init_y = d1 / math.sqrt(2.0)
        else:
            norm = math.hypot(max(0.0, cos_est), max(0.0, sin_est))
            if norm > 1e-4:
                init_x = d1 * max(0.0, cos_est) / norm
                init_y = d1 * max(0.0, sin_est) / norm
            else:
                init_x = d1 / math.sqrt(2.0)
                init_y = d1 / math.sqrt(2.0)

        p0 = [init_x, init_y]

        if HAS_SCIPY:
            def residuals(p):
                diff = anchors - p
                calc_dists = np.hypot(diff[:, 0], diff[:, 1])
                return calc_dists - dists

            try:
                res = least_squares(
                    residuals,
                    p0,
                    bounds=([0.0, 0.0], [50.0, 50.0]),
                    method="trf",
                    ftol=1e-4,
                    xtol=1e-4,
                    max_nfev=35,
                )
                if res.success or res.cost < 5.0:
                    return float(res.x[0]), float(res.x[1])
            except Exception:
                pass

        # SciPy が利用できない場合の純 NumPy Levenberg-Marquardt フォールバック
        p = np.array(p0, dtype=float)
        for _ in range(15):
            diff = p - anchors
            d_est = np.hypot(diff[:, 0], diff[:, 1])
            d_est = np.maximum(d_est, 1e-6)
            res = d_est - dists
            J = diff / d_est[:, np.newaxis]
            try:
                dp = np.linalg.lstsq(J.T @ J + 1e-3 * np.eye(2), -J.T @ res, rcond=None)[0]
                p += dp
                p[0] = max(0.0, p[0])
                p[1] = max(0.0, p[1])
                if np.linalg.norm(dp) < 1e-4:
                    break
            except np.linalg.LinAlgError:
                break

        return float(p[0]), float(p[1])

    def _solve_2_anchors_12(self, d1: float, d2: float) -> Optional[Tuple[float, float]]:
        """Slave 1 (0, 0) と Slave 2 (0, y2) からの2点幾何推定 (クランプ付き)"""
        y2 = self.anchors[1][1]
        if abs(y2) < 1e-6:
            return None

        # y 座標算出: y = (d1^2 - d2^2 + y2^2) / (2 * y2)
        y = (d1**2 - d2**2 + y2**2) / (2.0 * y2)
        # 測定誤差で y > d1 となる場合は幾何学的上限 d1 にクランプ
        y = max(-d1, min(d1, y))

        radicand = max(0.0, d1**2 - y**2)
        x = math.sqrt(radicand)
        return x, y

    def _solve_2_anchors_13(self, d1: float, d3: float) -> Optional[Tuple[float, float]]:
        """Slave 1 (0, 0) と Slave 3 (x3, 0) からの2点幾何推定 (クランプ付き)"""
        x3 = self.anchors[2][0]
        if abs(x3) < 1e-6:
            return None

        # x 座標算出: x = (d1^2 - d3^2 + x3^2) / (2 * x3)
        x = (d1**2 - d3**2 + x3**2) / (2.0 * x3)
        # 測定誤差で x > d1 となる場合は幾何学的上限 d1 にクランプ
        x = max(-d1, min(d1, x))

        radicand = max(0.0, d1**2 - x**2)
        y = math.sqrt(radicand)
        return x, y

    def _solve_2_anchors_23(self, d2: float, d3: float) -> Optional[Tuple[float, float]]:
        """Slave 2 (0, y2) と Slave 3 (x3, 0) からの2点幾何交点推定"""
        x2, y2 = self.anchors[1]
        x3, y3 = self.anchors[2]

        D = math.hypot(x3 - x2, y3 - y2)
        if D < 1e-6:
            return None

        # 円の交点算出 (三角不等式外れ値クランプ)
        a = (d2**2 - d3**2 + D**2) / (2.0 * D)
        a = max(-d2, min(d2, a))
        h_sq = max(0.0, d2**2 - a**2)
        h = math.sqrt(h_sq)

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


class DistanceFilter:
    """測距データのジッターを低減する距離ドメイン指数平滑化 (EMA) フィルタ"""

    def __init__(self, alpha: float = 0.35, max_step: float = 0.8):
        """
        Parameters
        ----------
        alpha : float
            平滑化係数 (0 < alpha <= 1.0)。小さいほど滑らか
        max_step : float
            1フレームあたりの最大許容距離変動幅 [m] (外れ値リジェクション用)
        """
        self.alpha = alpha
        self.max_step = max_step
        self.prev_d = [None, None, None]

    def update(self, d1: float, d2: float, d3: float) -> Tuple[float, float, float]:
        raw = [d1, d2, d3]
        filtered = []

        for i, val in enumerate(raw):
            if val <= 0:
                self.prev_d[i] = None
                filtered.append(val)
                continue

            if self.prev_d[i] is None:
                self.prev_d[i] = val
            else:
                # 急激なスパイク外れ値をクランプ
                diff = val - self.prev_d[i]
                if abs(diff) > self.max_step:
                    val = self.prev_d[i] + math.copysign(self.max_step, diff)

                self.prev_d[i] = self.alpha * val + (1.0 - self.alpha) * self.prev_d[i]

            filtered.append(float(self.prev_d[i]))

        return filtered[0], filtered[1], filtered[2]

    def reset(self):
        self.prev_d = [None, None, None]


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
    assert abs(p3[0] - 0.5) < 1e-2 and abs(p3[1] - 0.5) < 1e-2

    # 2. Slave 3 欠損 (Slave 1 & 2 のみ)
    p2_12, mode2_12 = tri.calculate_position(d, d, -1.0)
    print(f"Fallback 1-2 Test: pos={p2_12}, mode={mode2_12}")
    assert p2_12 is not None and mode2_12 == "FALLBACK_12"
    assert abs(p2_12[0] - 0.5) < 1e-2 and abs(p2_12[1] - 0.5) < 1e-2

    # 3. Slave 2 欠損 (Slave 1 & 3 のみ)
    p2_13, mode2_13 = tri.calculate_position(d, -1.0, d)
    print(f"Fallback 1-3 Test: pos={p2_13}, mode={mode2_13}")
    assert p2_13 is not None and mode2_13 == "FALLBACK_13"
    assert abs(p2_13[0] - 0.5) < 1e-2 and abs(p2_13[1] - 0.5) < 1e-2

    print("All Trilateration tests passed!")
