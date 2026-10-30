"""Motion and measurement models. State x = [px, py, pz, vx, vy, vz] (site ENU metres)."""
from dataclasses import dataclass

import numpy as np


def cv_model(dt: float, q: float = 3.0):
    """Constant-velocity transition F and process noise Q (white-noise acceleration, q in m/s^2)."""
    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt
    G = np.vstack([np.eye(3) * dt ** 2 / 2, np.eye(3) * dt])
    return F, G @ G.T * q ** 2


@dataclass
class Measurement:
    t: float
    sensor_id: str
    kind: str               # "position" (radar/EO with range) | "bearing" (acoustic, RF DF)
    z: np.ndarray           # position: [x,y,z]; bearing: [azimuth_rad]
    R: np.ndarray           # measurement covariance
    origin: np.ndarray      # sensor location [x,y,z]
    classification: str = "unknown"
    class_conf: float = 0.0


def h_and_H(x: np.ndarray, m: Measurement):
    """Predicted measurement and Jacobian (EKF linearisation for bearing-only sensors)."""
    if m.kind == "position":
        H = np.zeros((3, 6)); H[:, 0:3] = np.eye(3)
        return x[0:3].copy(), H
    dx, dy = x[0] - m.origin[0], x[1] - m.origin[1]
    r2 = max(dx * dx + dy * dy, 1e-6)
    H = np.zeros((1, 6))
    H[0, 0], H[0, 1] = -dy / r2, dx / r2
    return np.array([np.arctan2(dy, dx)]), H


def innovation(z, zp, kind):
    y = z - zp
    if kind == "bearing":
        y = (y + np.pi) % (2 * np.pi) - np.pi
    return y
