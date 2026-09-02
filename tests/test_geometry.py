import math
import numpy as np
import pytest
from crtk_conformance import geometry as G

rng = np.random.default_rng(1)


def rand_rot():
    return G.axis_angle(rng.normal(size=3), rng.uniform(0, math.pi))


def test_quat_roundtrip():
    for _ in range(200):
        R = rand_rot()
        q = G.rot_to_quat(R)
        assert np.allclose(G.quat_to_rot(*q), R, atol=1e-9)


def test_rotation_angle_axis_angle():
    for th in [0.0, 0.3, 1.0, 2.5, math.pi - 1e-6]:
        R = G.axis_angle([0.3, -0.2, 0.9], th)
        assert abs(G.rotation_angle(R) - th) < 1e-6


def test_invert_compose():
    T = G.make_pose(rand_rot(), rng.normal(size=3))
    assert np.allclose(G.compose(T, G.invert(T)), np.eye(4), atol=1e-12)


def test_average_pose_recovers_constant():
    T = G.make_pose(rand_rot(), [0.2, 0, 0])
    Ts = [T + np.pad(rng.normal(0, 1e-9, (3, 4)), ((0, 1), (0, 0))) for _ in range(20)]
    Tm = G.average_pose(Ts)
    assert np.allclose(Tm, T, atol=1e-6)


def test_m1_bounds_never_violated():
    """Eq. (2) two-sided bound and eq. (3) upper bound hold on random inputs (paper Sec. 5.1)."""
    for _ in range(5000):
        R = rand_rot(); t = rng.normal(size=3) * 0.2; p = rng.normal(size=3) * 0.2
        e = G.m1_positional_error(R, t, p)
        a = np.linalg.norm((R - np.eye(3)) @ p)
        assert abs(np.linalg.norm(t) - a) - 1e-12 <= e <= np.linalg.norm(t) + a + 1e-12
        assert e <= G.m1_upper_bound(R, t, np.linalg.norm(p)) + 1e-12
        assert e >= G.m1_lower_bound_exact(R, t) - 1e-9


def test_m1_jhu_configuration_values():
    """Numbers in the paper (DERIVATIONS.md): theta = 150 deg, e_p(0) = 200 mm, e_p(0.1 m, perpendicular) = 278.1 mm."""
    R = np.array([[1, 0, 0], [0, -0.866025404, 0.5], [0, -0.5, -0.866025404]]); t = np.array([0.20, 0, 0])
    assert abs(math.degrees(G.rotation_angle(R)) - 150.0) < 1e-3
    assert abs(G.m1_positional_error(R, t, np.zeros(3)) - 0.200) < 1e-9
    assert abs(G.m1_positional_error(R, t, np.array([0, 0, 0.10])) - 0.2781) < 5e-4
    assert abs(G.m1_upper_bound(R, t, 0.10) - 0.3932) < 5e-4
    assert abs(G.m1_incremental_bound(R, 0.001) - 0.00193) < 1e-5
    assert abs(G.m1_lower_bound_exact(R, t) - 0.200) < 1e-9


def test_m2_m3():
    assert abs(G.m2_error(0.1, 0.010) - 0.009) < 1e-12
    assert abs(G.m3_zoh_bound(0.05, 100) - 0.0005) < 1e-12
    assert abs(G.m3_required_rate(0.05, 0.0005) - 100) < 1e-9
    assert abs(G.m3_bounded_jitter_rate(0.030, 0.005) - 40.0) < 1e-9
    assert G.m3_bounded_jitter_rate(0.030, 0.030) == math.inf
