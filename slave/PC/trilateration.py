"""
trilateration.py
UWB 測距データに基づく2次元自己位置推定 (三辺測量) モジュール
- 3点アンカーによるロバスト非線形最小二乗解 (Levenberg-Marquardt / SciPy NLS)
- 短基線 (High GDOP) 環境に対応した幾何学的制約と解の安定化
- 1点欠損時 (2点有効時) の幾何学的フォールバック推定 (幾何学的クランプ付き)
- 測距値・座標平滑化フィルタ (DistanceFilter, MovingAverageFilter)
"""

import math
import time
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
        nominal_d12: Optional[float] = None,
        nominal_d13: Optional[float] = None,
        relabel_mode: str = "auto",
    ):
        """三辺測量計算クラスの初期化

        Parameters
        ----------
        anchor1 : Tuple[float, float]
            Slave 1 の (x, y) 座標 [m] (原点 (0.0, 0.0))
        anchor2 : Tuple[float, float]
            Slave 2 の初期 (x, y) 座標 [m] (Y軸上 (0.0, d12))
        anchor3 : Tuple[float, float]
            Slave 3 の初期 (x, y) 座標 [m] (X軸上 (d13, 0.0))
        nominal_d12 : Optional[float]
            Slave 2 (Y軸) の想定配置距離 [m] (None の場合は anchor2[1] を使用)
        nominal_d13 : Optional[float]
            Slave 3 (X軸) の想定配置距離 [m] (None の場合は anchor3[0] を使用)
        relabel_mode : str
            リラベルモード: "auto" (想定距離との照合で自動判定), "normal" (固定通常), "swap" (強制入れ替え)
        """
        self.nominal_d12 = float(nominal_d12 if nominal_d12 is not None else anchor2[1])
        self.nominal_d13 = float(nominal_d13 if nominal_d13 is not None else anchor3[0])
        self.relabel_mode = relabel_mode
        self.is_swapped = (relabel_mode == "swap")

        self.d12 = float(anchor2[1]) if anchor2[1] > 0 else self.nominal_d12
        self.d13 = float(anchor3[0]) if anchor3[0] > 0 else self.nominal_d13

        self.anchors = np.array([anchor1, (0.0, self.d12), (self.d13, 0.0)], dtype=float)

    def update_anchors(self, raw_d12: float, raw_d13: float) -> bool:
        """自動キャリブレーションで得られた実測基準線長からアンカー座標を動的に更新する。
        事前設定された想定距離 (nominal_d12, nominal_d13) との残差を評価し、
        物理モジュールが入れ替わって設置されていた場合は自動的にリラベルする。

        Parameters
        ----------
        raw_d12 : float
            Arduino Port 2 (D4/D5) モジュールへの実測距離 [m]
        raw_d13 : float
            Arduino Port 3 (D6/D7) モジュールへの実測距離 [m]

        Returns
        -------
        bool
            リラベル状態 (is_swapped) が切り替わった場合 True
        """
        if raw_d12 < 0.10 or raw_d13 < 0.10:
            return False

        old_swapped = self.is_swapped

        if self.relabel_mode == "auto":
            # 想定配置 (P2->Y軸, P3->X軸) と 入れ替え配置 (P3->Y軸, P2->X軸) の誤差比較
            cost_normal = abs(raw_d12 - self.nominal_d12) + abs(raw_d13 - self.nominal_d13)
            cost_swapped = abs(raw_d13 - self.nominal_d12) + abs(raw_d12 - self.nominal_d13)

            # 入れ替え配置の方が想定に明らかに適合している場合 (しきい値マージン 1cm)
            if cost_swapped < cost_normal - 0.01:
                self.is_swapped = True
            elif cost_normal < cost_swapped - 0.01:
                self.is_swapped = False
        elif self.relabel_mode == "swap":
            self.is_swapped = True
        else:
            self.is_swapped = False

        if self.is_swapped:
            self.d12 = float(raw_d13)  # Port 3 の距離を Y軸 (Slave 2) に割り当て
            self.d13 = float(raw_d12)  # Port 2 の距離を X軸 (Slave 3) に割り当て
        else:
            self.d12 = float(raw_d12)  # Port 2 の距離を Y軸 (Slave 2) に割り当て
            self.d13 = float(raw_d13)  # Port 3 の距離を X軸 (Slave 3) に割り当て

        self.anchors = np.array([
            (0.0, 0.0),
            (0.0, self.d12),
            (self.d13, 0.0),
        ], dtype=float)

        return self.is_swapped != old_swapped

    def remap_distances(self, raw_d1: float, raw_d2: float, raw_d3: float) -> Tuple[float, float, float]:
        """リラベル状態に応じて、測定距離を論理アンカー (S1, S2, S3) の順序に並べ替える"""
        if self.is_swapped:
            return raw_d1, raw_d3, raw_d2
        return raw_d1, raw_d2, raw_d3

    def calculate_gdop(
        self, pos: Tuple[float, float], sigma_r: float = 0.05
    ) -> Tuple[float, float, float, float, str]:
        """指定された座標における幾何学的精度低下率 (HDOP) および 95% 信頼楕円パラメータを算出する

        Parameters
        ----------
        pos : Tuple[float, float]
            評価対象の (x, y) 座標 [m]
        sigma_r : float
            UWB 測距誤差の標準偏差 [m] (デフォルト: 0.05m = 5cm)

        Returns
        -------
        hdop : float
            幾何学的精度低下率 (HDOP: 水平精度低下率)
        semi_major_m : float
            95% 信頼楕円の長半径 [m] (2 * sigma)
        semi_minor_m : float
            95% 信頼楕円の短半径 [m] (2 * sigma)
        orientation_deg : float
            信頼楕円の長軸方向角 [度] (X軸基準反時計回り)
        rating : str
            配置精度評価 ("EXCELLENT", "GOOD", "MODERATE", "POOR (SPREAD ANCHORS!)")
        """
        p = np.array(pos, dtype=float)
        diff = p - self.anchors  # shape: (3, 2)
        dists = np.hypot(diff[:, 0], diff[:, 1])[:, np.newaxis]
        dists = np.maximum(dists, 1e-4)

        # 視線方向の単位ベクトル行列 (設計行列 H)
        H = diff / dists  # shape: (3, 2)
        A = H.T @ H       # shape: (2, 2)

        det = float(np.linalg.det(A))
        if det < 1e-6:
            return 99.9, 5.0, 5.0, 0.0, "POOR (SPREAD ANCHORS!)"

        try:
            Q = np.linalg.inv(A)
            hdop = float(np.sqrt(max(0.0, np.trace(Q))))

            # 信頼楕円パラメータ (共分散幾何行列 Q の固有値分解)
            evals, evecs = np.linalg.eigh(Q)
            order = np.argsort(evals)[::-1]
            evals = evals[order]
            evecs = evecs[:, order]

            semi_major = float(2.0 * sigma_r * np.sqrt(max(0.0, evals[0])))
            semi_minor = float(2.0 * sigma_r * np.sqrt(max(0.0, evals[1])))
            angle_deg = float(np.degrees(np.arctan2(evecs[1, 0], evecs[0, 0])) % 360)

            if hdop < 2.5:
                rating = "EXCELLENT"
            elif hdop < 5.0:
                rating = "GOOD"
            elif hdop < 10.0:
                rating = "MODERATE"
            else:
                rating = "POOR (SPREAD ANCHORS!)"

            return hdop, semi_major, semi_minor, angle_deg, rating
        except Exception:
            return 99.9, 5.0, 5.0, 0.0, "POOR (SPREAD ANCHORS!)"

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


class KinematicPositionFilter:
    """ロボットの物理運動モデル (等速度運動 + 速度上限制約) に基づく自己位置フィルタ

    - 測定ノイズによる物理的にあり得ない瞬間移動 (ワープ) を動的に検出・制限
    - 速度ベクトル (vx, vy)、速さ [m/s]、進行方位角 [deg] のリアルタイム推定
    - Alpha-Beta 追従フィルタによる滑らかな運動軌跡生成
    """

    def __init__(self, max_speed_mps: float = 1.5, alpha: float = 0.35, beta: float = 0.12):
        """
        Parameters
        ----------
        max_speed_mps : float
            ロボットの想定最高速度 [m/s] (デフォルト: 1.5 m/s)。この速度を超えるジャンプは物理的にカット
        alpha : float
            位置追従ゲイン (0 < alpha <= 1.0)
        beta : float
            速度追従ゲイン (0 < beta <= 1.0)
        """
        self.max_speed_mps = float(max_speed_mps)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.pos: Optional[np.ndarray] = None
        self.vel: np.ndarray = np.array([0.0, 0.0], dtype=float)
        self.last_time: Optional[float] = None
        self.warp_count: int = 0
        self.is_clamped_last: bool = False

    def update(
        self, raw_pos: Optional[Tuple[float, float]], timestamp: Optional[float] = None
    ) -> Tuple[Optional[Tuple[float, float]], Tuple[float, float], float, float]:
        """新しい推定座標を入力し、物理制約フィルタを適用した (pos, vel, speed, heading_deg) を返す

        Parameters
        ----------
        raw_pos : Optional[Tuple[float, float]]
            三辺測量等で得られた生 (x, y) 座標 [m]
        timestamp : Optional[float]
            現在時刻 [秒] (None の場合は time.time() を使用)

        Returns
        -------
        filtered_pos : Optional[Tuple[float, float]]
            平滑化・速度制限後の (x, y) 座標 [m]
        vel : Tuple[float, float]
            現在の速度ベクトル (vx, vy) [m/s]
        speed : float
            現在の移動速さ [m/s]
        heading_deg : float
            進行方位角 [度] (X軸基準反時計回り: 0〜360度)
        """
        if raw_pos is None:
            self.is_clamped_last = False
            speed = float(np.hypot(self.vel[0], self.vel[1])) if self.pos is not None else 0.0
            heading = float(math.degrees(math.atan2(self.vel[1], self.vel[0])) % 360) if speed > 0.05 else 0.0
            cur_p = (float(self.pos[0]), float(self.pos[1])) if self.pos is not None else None
            return cur_p, (float(self.vel[0]), float(self.vel[1])), speed, heading

        now = timestamp if timestamp is not None else time.time()
        raw_p = np.array(raw_pos, dtype=float)

        if self.pos is None or self.last_time is None:
            self.pos = raw_p.copy()
            self.vel = np.array([0.0, 0.0], dtype=float)
            self.last_time = now
            self.is_clamped_last = False
            return (float(self.pos[0]), float(self.pos[1])), (0.0, 0.0), 0.0, 0.0

        dt = now - self.last_time
        self.last_time = now

        # dt が極端に小さい、または長時間途切れた場合の安全策
        if dt < 1e-4:
            dt = 1e-4
        elif dt > 1.0:
            # 1秒以上途切れた場合は運動モデルをリセットして現在位置を再採用
            self.pos = raw_p.copy()
            self.vel = np.array([0.0, 0.0], dtype=float)
            self.is_clamped_last = False
            return (float(self.pos[0]), float(self.pos[1])), (0.0, 0.0), 0.0, 0.0

        # 1. 等速度運動モデルによる位置予測 (Prediction)
        pred_pos = self.pos + self.vel * dt

        # 2. 最高速度制限チェック (Physical Velocity Clamping)
        diff = raw_p - self.pos
        dist_jump = float(np.hypot(diff[0], diff[1]))
        apparent_speed = dist_jump / dt

        if apparent_speed > self.max_speed_mps and dist_jump > 1e-6:
            self.warp_count += 1
            self.is_clamped_last = True
            # 移動量を最大許容速度分に強制クランプ
            clamped_dist = self.max_speed_mps * dt
            meas_pos = self.pos + diff * (clamped_dist / dist_jump)
        else:
            self.is_clamped_last = False
            meas_pos = raw_p

        # 3. Alpha-Beta 追従更新 (Correction)
        residual = meas_pos - pred_pos
        self.pos = pred_pos + self.alpha * residual
        self.vel = self.vel + (self.beta / dt) * residual

        # 速度ベクトルのノルムも最大速度にクリップ
        speed = float(np.hypot(self.vel[0], self.vel[1]))
        if speed > self.max_speed_mps and speed > 1e-6:
            self.vel = self.vel * (self.max_speed_mps / speed)
            speed = self.max_speed_mps

        heading_deg = float(math.degrees(math.atan2(self.vel[1], self.vel[0])) % 360) if speed > 0.05 else 0.0
        return (float(self.pos[0]), float(self.pos[1])), (float(self.vel[0]), float(self.vel[1])), speed, heading_deg

    def reset(self):
        self.pos = None
        self.vel = np.array([0.0, 0.0], dtype=float)
        self.last_time = None
        self.is_clamped_last = False


class ExtendedKalmanFilter2D:
    """2次元等速運動モデル (Constant Velocity) に基づく拡張カルマンフィルタ (EKF)

    状態ベクトル: x = [px, py, vx, vy]^T
    観測モデル:
      1) 非線形距離観測 (Direct Range EKF): z = [d1, d2, d3]^T
         h_i(x) = sqrt((px - a_ix)^2 + (py - a_iy)^2)
      2) 座標観測 (Linear KF Update): z = [px, py]^T

    特徴:
      - 欠損アンカー (2点以下) 時の動的適応更新
      - 3-sigma 残差ゲーティングによるマルチパス・外れ値リジェクション
      - 速度ベクトル (vx, vy)、速さ、進行方位角のリアルタイム推定
      - 状態共分散行列 P からの自己位置不確かさ推定
    """

    def __init__(
        self,
        sigma_a: float = 1.0,
        sigma_r: float = 0.05,
        sigma_pos: float = 0.08,
        p_init_pos: float = 0.5,
        p_init_vel: float = 1.0,
    ):
        """
        Parameters
        ----------
        sigma_a : float
            プロセスノイズ: 加速度変動の標準偏差 [m/s^2]
        sigma_r : float
            観測ノイズ: UWB測距の標準偏差 [m] (デフォルト: 0.05m)
        sigma_pos : float
            観測ノイズ: 三辺測量座標の標準偏差 [m] (デフォルト: 0.08m)
        p_init_pos : float
            初期位置共分散の標準偏差 [m]
        p_init_vel : float
            初期速度共分散の標準偏差 [m/s]
        """
        self.sigma_a = float(sigma_a)
        self.sigma_r = float(sigma_r)
        self.sigma_pos = float(sigma_pos)

        # 状態ベクトル [x, y, vx, vy]^T
        self.x = np.zeros(4, dtype=float)

        # 状態共分散行列 P (4x4)
        self.P = np.diag([
            p_init_pos ** 2,
            p_init_pos ** 2,
            p_init_vel ** 2,
            p_init_vel ** 2,
        ]).astype(float)

        self.last_time: Optional[float] = None
        self.is_initialized = False
        self.last_valid_heading: float = 0.0

    def reset(self, initial_pos: Optional[Tuple[float, float]] = None):
        """フィルタをリセットする"""
        if initial_pos is not None:
            self.x = np.array([initial_pos[0], initial_pos[1], 0.0, 0.0], dtype=float)
            self.P = np.diag([0.1 ** 2, 0.1 ** 2, 1.0 ** 2, 1.0 ** 2]).astype(float)
            self.is_initialized = True
        else:
            self.x = np.zeros(4, dtype=float)
            self.P = np.diag([1.0 ** 2, 1.0 ** 2, 2.0 ** 2, 2.0 ** 2]).astype(float)
            self.is_initialized = False
        self.last_time = None
        self.last_valid_heading = 0.0

    def predict(self, dt: Optional[float] = None) -> np.ndarray:
        """状態予測ステップ (時間発展)

        Parameters
        ----------
        dt : Optional[float]
            前回ステップからの経過時間 [秒]。省略時は内部タイマーで自動計算。

        Returns
        -------
        np.ndarray
            予測状態ベクトル [x, y, vx, vy]
        """
        now = time.perf_counter()
        if dt is None:
            if self.last_time is not None:
                dt = now - self.last_time
                # 異常に長い/短いステップ幅を安全にクランプ
                dt = max(0.005, min(0.5, dt))
            else:
                dt = 0.05
        self.last_time = now

        # 状態遷移行列 F
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=float)

        # 離散白色加速度ノイズモデル Q
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        q_pos = 0.25 * dt4 * (self.sigma_a ** 2)
        q_cross = 0.5 * dt3 * (self.sigma_a ** 2)
        q_vel = dt2 * (self.sigma_a ** 2)

        Q = np.array([
            [q_pos,   0.0,     q_cross, 0.0],
            [0.0,     q_pos,   0.0,     q_cross],
            [q_cross, 0.0,     q_vel,   0.0],
            [0.0,     q_cross, 0.0,     q_vel],
        ], dtype=float)

        # 予測
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        # 対称性維持
        self.P = 0.5 * (self.P + self.P.T)
        return self.x

    def update_position(
        self, pos: Tuple[float, float], gate_m: float = 1.5
    ) -> Tuple[float, float]:
        """三辺測量で得られた (x, y) 座標による線形カルマン更新

        Parameters
        ----------
        pos : Tuple[float, float]
            観測されたロボット座標 (x, y) [m]
        gate_m : float
            外れ値リジェクションの残差ゲート [m] (これを超える跳躍は除外)

        Returns
        -------
        Tuple[float, float]
            更新後の平滑化座標 (x, y)
        """
        if not self.is_initialized:
            self.x[0] = pos[0]
            self.x[1] = pos[1]
            self.x[2] = 0.0
            self.x[3] = 0.0
            self.is_initialized = True
            return pos[0], pos[1]

        z = np.array([pos[0], pos[1]], dtype=float)
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=float)

        y = z - H @ self.x  # イノベーション
        residual_norm = float(np.hypot(y[0], y[1]))

        # 外れ値ゲーティング (物理的にありえない大ジャンプを抑制)
        if residual_norm > gate_m:
            y = (gate_m / residual_norm) * y

        R = (self.sigma_pos ** 2) * np.eye(2, dtype=float)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I_KH = np.eye(4, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return float(self.x[0]), float(self.x[1])

    def update_ranges(
        self,
        d1: float,
        d2: float,
        d3: float,
        anchors: np.ndarray,
        gate_m: float = 0.8,
    ) -> Tuple[float, float]:
        """UWB生測距値による拡張カルマンフィルタ (EKF) 更新
        非線形観測方程式 h_i(x) = ||p - a_i|| のヤコビアンを用いて直接更新。
        一部アンカー欠損時でも有効アンカーのみで動的適応更新可能。

        Parameters
        ----------
        d1, d2, d3 : float
            各アンカーへの測距値 [m] (無効値は <= 0)
        anchors : np.ndarray
            アンカー座標配列 shape: (3, 2)
        gate_m : float
            測距残差の許容ゲート幅 [m] (マルチパス跳躍の外れ値除外)

        Returns
        -------
        Tuple[float, float]
            更新後の平滑化座標 (x, y)
        """
        raw_dists = [d1, d2, d3]
        valid_indices = [i for i, d in enumerate(raw_dists) if d > 0.05]

        if not self.is_initialized:
            return float(self.x[0]), float(self.x[1])

        if len(valid_indices) == 0:
            return float(self.x[0]), float(self.x[1])

        px, py = self.x[0], self.x[1]
        active_H = []
        active_y = []
        active_R = []

        for idx in valid_indices:
            an = anchors[idx]
            dx = px - an[0]
            dy = py - an[1]
            dist_est = math.hypot(dx, dy)
            dist_est = max(dist_est, 1e-4)

            residual = raw_dists[idx] - dist_est

            # 外れ値ゲーティング (マルチパス急変のクランプ)
            if abs(residual) > gate_m:
                residual = math.copysign(gate_m, residual)

            h_row = [dx / dist_est, dy / dist_est, 0.0, 0.0]
            active_H.append(h_row)
            active_y.append(residual)
            active_R.append(self.sigma_r ** 2)

        H = np.array(active_H, dtype=float)
        y = np.array(active_y, dtype=float)
        R = np.diag(active_R).astype(float)

        S = H @ self.P @ H.T + R
        det_S = float(np.linalg.det(S))
        if det_S < 1e-9:
            return float(self.x[0]), float(self.x[1])

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I_KH = np.eye(4, dtype=float) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return float(self.x[0]), float(self.x[1])

    def get_state(self) -> Tuple[float, float, float, float, float, float]:
        """推定された状態と運動指標を取得する

        Returns
        -------
        px : float
            X座標 [m]
        py : float
            Y座標 [m]
        vx : float
            X方向速度 [m/s]
        vy : float
            Y方向速度 [m/s]
        speed : float
            速さ [m/s]
        heading_deg : float
            進行方位角 [度] (0~360度, X軸基準反時計回り)
        """
        px = float(self.x[0])
        py = float(self.x[1])
        vx = float(self.x[2])
        vy = float(self.x[3])
        speed = float(math.hypot(vx, vy))

        # 静止時(速度 < 0.05m/s)の方位角ハンチング・暴れを抑制
        if speed > 0.05:
            self.last_valid_heading = float(math.degrees(math.atan2(vy, vx)) % 360)

        return px, py, vx, vy, speed, self.last_valid_heading

    def get_position_uncertainty(self) -> Tuple[float, float]:
        """自己位置の 95% (2-sigma) 不確かさ半径 [m] を取得する

        Returns
        -------
        sigma_x : float
            X方向 2-sigma 不確かさ [m]
        sigma_y : float
            Y方向 2-sigma 不確かさ [m]
        """
        sx = 2.0 * math.sqrt(max(0.0, float(self.P[0, 0])))
        sy = 2.0 * math.sqrt(max(0.0, float(self.P[1, 1])))
        return sx, sy


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

    # 4. 動的リラベルテスト (Slave2とSlave3が物理的に入れ替わって置かれた場合)
    tri_relabel = Trilateration(nominal_d12=0.8, nominal_d13=1.5)
    # 物理ポート3がY軸位置(0.8m)、物理ポート2がX軸位置(1.5m)に置かれた場合のCALIB測定
    swapped = tri_relabel.update_anchors(raw_d12=1.5, raw_d13=0.8)
    print(f"Auto-Relabel Test: is_swapped={tri_relabel.is_swapped}, changed={swapped}")
    assert tri_relabel.is_swapped is True
    assert abs(tri_relabel.d12 - 0.8) < 1e-3 and abs(tri_relabel.d13 - 1.5) < 1e-3

    # ロボットが (0.5, 0.5) にある時:
    # d1 = sqrt(0.5^2 + 0.5^2) ~= 0.7071
    # Y軸(S2)からの距離 = sqrt(0.5^2 + (0.5-0.8)^2) = sqrt(0.25 + 0.09) = sqrt(0.34) ~= 0.5831
    # X軸(S3)からの距離 = sqrt((0.5-1.5)^2 + 0.5^2) = sqrt(1.0 + 0.25) = sqrt(1.25) ~= 1.1180
    # 物理ポート2がX軸にあり、物理ポート3がY軸にあるため、Arduinoから届く生データは:
    raw_d1 = math.hypot(0.5, 0.5)
    raw_d2 = math.hypot(0.5 - 1.5, 0.5)  # 物理ポート2 (X軸位置にある)
    raw_d3 = math.hypot(0.5, 0.5 - 0.8)  # 物理ポート3 (Y軸位置にある)

    # リラベルを適用して距離を再配置
    rem_d1, rem_d2, rem_d3 = tri_relabel.remap_distances(raw_d1, raw_d2, raw_d3)
    pos_rel, mode_rel = tri_relabel.calculate_position(rem_d1, rem_d2, rem_d3)
    print(f"Relabeled Position Test: pos={pos_rel}, mode={mode_rel}")
    assert pos_rel is not None
    assert abs(pos_rel[0] - 0.5) < 1e-2 and abs(pos_rel[1] - 0.5) < 1e-2

    # 5. GDOP & 信頼楕円計算テスト
    tri_wide = Trilateration(anchor1=(0.0, 0.0), anchor2=(0.0, 2.0), anchor3=(2.0, 0.0))
    hdop_w, a_w, b_w, ang_w, rating_w = tri_wide.calculate_gdop((1.0, 1.0))
    print(f"Wide GDOP Test: hdop={hdop_w:.2f}, rating={rating_w}, ellipse=({a_w:.3f}m, {b_w:.3f}m)")
    assert hdop_w < 2.5 and rating_w == "EXCELLENT"

    tri_narrow = Trilateration(anchor1=(0.0, 0.0), anchor2=(0.0, 0.2), anchor3=(0.2, 0.0))
    hdop_n, a_n, b_n, ang_n, rating_n = tri_narrow.calculate_gdop((3.0, 3.0))
    print(f"Narrow GDOP Test: hdop={hdop_n:.2f}, rating={rating_n}, ellipse=({a_n:.3f}m, {b_n:.3f}m)")
    assert hdop_n > 10.0 and rating_n == "POOR (SPREAD ANCHORS!)"

    # 6. 物理運動モデル (ワープ抑制 & 速度・方位推定) テスト
    kf = KinematicPositionFilter(max_speed_mps=1.0)
    p0, v0, s0, h0 = kf.update((0.0, 0.0), timestamp=0.0)
    assert p0 == (0.0, 0.0)
    p1, v1, s1, h1 = kf.update((0.05, 0.0), timestamp=0.1)
    assert kf.is_clamped_last is False
    assert s1 > 0.04
    p2, v2, s2, h2 = kf.update((2.0, 0.0), timestamp=0.2)
    assert kf.is_clamped_last is True
    assert kf.warp_count == 1
    assert p2[0] < 0.35
    print(f"KinematicFilter Test: warp_suppressed={kf.warp_count}, clamped_x={p2[0]:.3f}, speed={s2:.2f}m/s, heading={h2:.1f}deg")

    # 7. EKF トラッキングテスト (等速直線運動 + 測距ノイズ + 1点欠損耐性)
    ekf = ExtendedKalmanFilter2D(sigma_a=0.5, sigma_r=0.03, sigma_pos=0.05)
    ekf.reset(initial_pos=(0.0, 0.0))
    anchors = np.array([[0.0, 0.0], [0.0, 2.0], [2.0, 0.0]])
    dt = 0.1
    true_x, true_y = 0.0, 0.0
    vx_true, vy_true = 0.4, 0.3  # speed = 0.5 m/s, heading = ~36.87 deg

    for step in range(30):
        true_x += vx_true * dt
        true_y += vy_true * dt
        ekf.predict(dt=dt)
        d1 = math.hypot(true_x, true_y) + (0.01 if step % 2 == 0 else -0.01)
        d2 = math.hypot(true_x, true_y - 2.0)
        d3 = -1.0 if 20 <= step <= 25 else math.hypot(true_x - 2.0, true_y)
        ekf.update_ranges(d1, d2, d3, anchors)

    px, py, vx, vy, spd, hdg = ekf.get_state()
    print(f"EKF Tracking Result: pos=({px:.3f}, {py:.3f}), true=({true_x:.3f}, {true_y:.3f}), speed={spd:.3f} m/s, heading={hdg:.1f}°")
    assert abs(px - true_x) < 0.15 and abs(py - true_y) < 0.15
    assert abs(spd - 0.5) < 0.15
    assert abs(hdg - 36.87) < 10.0

    print("All Trilateration, GDOP, Kinematic, and EKF tests passed!")
