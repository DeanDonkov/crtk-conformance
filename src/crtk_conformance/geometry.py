"""Minimal SE(3) utilities (no ROS dependency).

A pose is represented as a 4x4 homogeneous numpy array. Quaternions follow the ROS convention
(x, y, z, w). Everything here is plain linear algebra so the probes and the error model can be
unit-tested without a ROS master.
"""
from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Rotation matrix -> (x, y, z, w), Shepperd's method."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def make_pose(R: np.ndarray | None = None, t: Iterable[float] | None = None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(list(t), dtype=float)
    return T


def axis_angle(axis: Iterable[float], theta: float) -> np.ndarray:
    k = np.asarray(list(axis), dtype=float)
    n = np.linalg.norm(k)
    if n == 0.0:
        return np.eye(3)
    k = k / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def rotation_angle(R: np.ndarray) -> float:
    """Angle of rotation in radians, in [0, pi]."""
    c = (np.trace(R) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def compose(*Ts: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    for T in Ts:
        out = out @ T
    return out


def translation(T: np.ndarray) -> np.ndarray:
    return T[:3, 3].copy()


def rotation(T: np.ndarray) -> np.ndarray:
    return T[:3, :3].copy()


def pose_distance(Ta: np.ndarray, Tb: np.ndarray) -> Tuple[float, float]:
    """(translation error [m], rotation error [rad]) between two poses."""
    d = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    ang = rotation_angle(Ta[:3, :3].T @ Tb[:3, :3])
    return d, ang


def average_rotation(Rs: Iterable[np.ndarray]) -> np.ndarray:
    """Chordal mean of rotations via SVD projection."""
    M = sum(np.asarray(R, dtype=float) for R in Rs)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def average_pose(Ts: Iterable[np.ndarray]) -> np.ndarray:
    Ts = list(Ts)
    R = average_rotation(T[:3, :3] for T in Ts)
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    return make_pose(R, t)


# ---------------------------------------------------------------- error model (paper, Section 5)


def m1_positional_error(R: np.ndarray, t: np.ndarray, p_c: np.ndarray) -> float:
    """Eq. (1): e_p = ||(R - I) p_c + t||."""
    return float(np.linalg.norm((R - np.eye(3)) @ p_c + t))


def m1_upper_bound(R: np.ndarray, t: np.ndarray, p_norm: float) -> float:
    """Eq. (3): e_p <= ||t|| + 2 sin(theta/2) ||p_c||."""
    theta = rotation_angle(R)
    return float(np.linalg.norm(t) + 2.0 * math.sin(theta / 2.0) * p_norm)


def m1_lower_bound_exact(R: np.ndarray, t: np.ndarray) -> float:
    """Eq. (3'): e_p >= ||t_parallel|| where t_parallel is the component of t along the rotation axis
    (for theta = 0 the whole of t counts)."""
    theta = rotation_angle(R)
    if theta < 1e-9:
        return float(np.linalg.norm(t))
    # rotation axis from the skew part of R
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(k)
    if n < 1e-9:  # theta ~ pi: axis from symmetric part
        w, v = np.linalg.eigh(R)
        k = v[:, np.argmax(w)]
    else:
        k = k / n
    return float(abs(np.dot(t, k)))


def m1_incremental_bound(R: np.ndarray, step_norm: float) -> float:
    """Eq. (3b): e_delta <= 2 sin(theta/2) ||delta p||."""
    return float(2.0 * math.sin(rotation_angle(R) / 2.0) * step_norm)


def m2_error(s: float, p_norm: float) -> float:
    """Eq. (M2.1): e = |1 - s| ||p|| (single step or distance from origin)."""
    return abs(1.0 - s) * p_norm


def m3_zoh_bound(speed: float, rate_hz: float) -> float:
    """Eq. (6): e_ZOH <= v / f."""
    return speed / rate_hz


def m3_required_rate(speed: float, tolerance: float) -> float:
    """Eq. (6) inverted: f >= v / epsilon."""
    return speed / tolerance


def m3_bounded_jitter_rate(tau_w: float, j_max: float) -> float:
    """Eq. (4): f > 1 / (tau_w - J_max)."""
    if tau_w <= j_max:
        return math.inf
    return 1.0 / (tau_w - j_max)
