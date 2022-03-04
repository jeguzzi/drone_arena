from typing import Tuple, Optional

import numpy as np


def rotate(alpha: float) -> np.ndarray:
    return np.array([[np.cos(alpha), -np.sin(alpha), 0],
                     [np.sin(alpha), np.cos(alpha), 0],
                     [0, 0, 1]])


class KalmanFilter:

    H = np.matrix(
        [[1, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0],
         [0, 0, 1, 0, 0, 0]])

    def __init__(self, r_x: float, r_y: float, r_phi: float, q_v: float, q_omega: float,
                 p_xy: float = 100, p_phi: float = 10, p_v: float = 10, p_omega: float = 10):
        # Should compute the error in robot frame

        self.q_omega = q_omega
        self.q_v = q_v
        self._R = np.diag([r_x, r_y, r_phi])
        self.t: Optional[float] = None
        self._x = np.array([[0, 0, 0, 0, 0, 0]]).T
        self._p = np.diag([p_xy, p_xy, p_phi, p_v, p_v, p_omega])

    def R(self, yaw: float) -> np.ndarray:
        r = rotate(yaw)
        return r @ self._R @ r.T

    def dynamic(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        F = np.matrix(
            [[1, 0, 0, dt, 0, 0],
             [0, 1, 0, 0, dt, 0],
             [0, 0, 1, 0, 0, dt],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 1]])
        q_v = self.q_v * (dt**2)
        q_omega = self.q_omega * (dt**2)
        Q = np.diag([0, 0, 0, q_v, q_v, q_omega])
        return F, Q

    def update(self, t: float, yaw: float, z: np.ndarray) -> None:
        if self.t is None:
            dt = 0.0
        else:
            dt = t - self.t
        self.t = t
        F, Q = self.dynamic(dt)
        x = self._x
        p = self._p
        H = self.H
        R = self.R(yaw)
        x_ = F @ x
        p_ = F @ p @ F.T + Q
        y = z - H @ x_
        if y[2] > np.pi:
            y[2] -= 2 * np.pi
        elif y[2] < -np.pi:
            y[2] += 2 * np.pi
        S = H @ p_ @ H.T + R
        K = p_ @ H.T @ np.linalg.inv(S)
        self._x = x_ + K @ y
        self._p = p_ - K @ H @ p_
