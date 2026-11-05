"""
trilateration.py
UWB 3点測距データに基づく2次元自己位置推定 (三辺測量) モジュール
"""

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
        self, d1: float, d2: float, d3: float
    ) -> Optional[Tuple[float, float]]:
        """3つの距離 [m] から Tag の 2D 座標 (x, y) [m] を算出する。

        解法:
          (x - x_i)^2 + (y - y_i)^2 = d_i^2
          基準点 (x1, y1) を引くことで線形化:
          2(x2 - x1)x + 2(y2 - y1)y = d1^2 - d2^2 + (x2^2 + y2^2) - (x1^2 + y1^2)
          2(x3 - x1)x + 2(y3 - y1)y = d1^2 - d3^2 + (x3^2 + y3^2) - (x1^2 + y1^2)
          A @ [x, y]^T = b

        Parameters
        ----------
        d1 : float
            Slave 1 からの距離 [m] (エラー時は < 0)
        d2 : float
            Slave 2 からの距離 [m] (エラー時は < 0)
        d3 : float
            Slave 3 からの距離 [m] (エラー時は < 0)

        Returns
        -------
        Optional[Tuple[float, float]]
            推定された (x, y) [m]。いずれかの距離が無効または特異点の場合は None
        """
        # 有効性チェック
        if d1 <= 0.0 or d2 <= 0.0 or d3 <= 0.0:
            return None

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
            # 最小二乗法で解く
            pos, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            return float(pos[0]), float(pos[1])
        except np.linalg.LinAlgError:
            return None


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
    # 単体テスト: (0.5, 0.5) にTagがある場合の理想距離
    tri = Trilateration()
    # A1(0,0) -> sqrt(0.5^2 + 0.5^2) = sqrt(0.5) ~= 0.7071
    # A2(0,1) -> sqrt(0.5^2 + (-0.5)^2) = sqrt(0.5) ~= 0.7071
    # A3(1,0) -> sqrt((-0.5)^2 + 0.5^2) = sqrt(0.5) ~= 0.7071
    d = np.sqrt(0.5)
    est = tri.calculate_position(d, d, d)
    print(f"Test Tag (0.5, 0.5): Estimated = {est}")
    assert est is not None
    assert abs(est[0] - 0.5) < 1e-4 and abs(est[1] - 0.5) < 1e-4
    print("Trilateration test passed!")

