"""Spatial decision statistics (0.1.2): exact maximum error, propagated confidence region, coverage.

Regressions for the RC3 adversarial review (rc4/RC3_ADVERSARIAL_REVIEW.md): finding 1 (an upper bound was used
as proof of violation) and finding 4 (the re-centred Student-t interval had ~45 % coverage at zero residual).
"""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as Rot

from crtk_conformance import geometry as G
from crtk_conformance.probes.base import Outcome, decide
from crtk_conformance.spatial import exact_max_error, exact_min_error, hotelling_radius, spatial_decision
from crtk_conformance.thresholds import Tolerance


def test_reviewer_counterexample_parallel_translation_small_rotation_is_conformant():
    # RC3 review, finding 1: t = 0.7 mm along the rotation axis, theta = 0.3 deg, r_ws = 0.1 m, epsilon = 1 mm
    tol = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1)
    R = G.axis_angle([0, 0, 1], math.radians(0.3))
    t = np.array([0.0, 0.0, 0.0007])
    e_exact = tol.spatial_error_exact(R, t)
    e_bound = tol.spatial_error(float(np.linalg.norm(t)), math.radians(0.3))
    assert abs(e_exact - 0.00087416) < 1e-7 and e_bound > 0.001
    # noiseless residual repeated over trials: the region collapses and the verdict is conformant
    E = G.make_pose(R, t)
    sd = spatial_decision([E.copy() for _ in range(10)], 0.1)
    assert sd.ci_high_m < 0.001 and decide(sd.ci_low_m, sd.ci_high_m, 0.001) == Outcome.CONFORMANT


def test_exact_max_matches_brute_force():
    rng = np.random.default_rng(3)
    for _ in range(200):
        ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
        R = G.axis_angle(ax, rng.uniform(0, 3.0)); t = rng.normal(size=3) * 0.05
        P = rng.normal(size=(3000, 3)); P /= np.linalg.norm(P, axis=1)[:, None]; P *= 0.1 * rng.uniform(0, 1, 3000)[:, None] ** (1 / 3)
        P = np.vstack([P, 0.1 * P[:1500] / np.linalg.norm(P[:1500], axis=1)[:, None]])
        e = np.linalg.norm(P @ (R - np.eye(3)).T + t, axis=1)
        assert e.max() <= exact_max_error(R, t, 0.1) + 1e-12
        assert e.max() >= exact_max_error(R, t, 0.1) - 3e-3
        assert e.min() >= exact_min_error(R, t, 0.1) - 1e-12


def test_exact_max_reduces_to_bound_when_translation_is_perpendicular():
    R = G.axis_angle([1, 0, 0], math.radians(-150.0))
    t = np.array([0.0, 0.2, 0.0])  # perpendicular to the axis: the bound is attained
    tol = Tolerance(workspace_radius_m=0.1)
    assert abs(tol.spatial_error_exact(R, t) - tol.spatial_error(0.2, math.radians(150.0))) < 1e-12
    t2 = np.array([0.2, 0.0, 0.0])  # along the axis (the JHU case): strictly below the bound
    assert tol.spatial_error_exact(R, t2) < tol.spatial_error(0.2, math.radians(150.0))
    assert abs(tol.spatial_error_exact(R, t2) - math.sqrt(0.2 ** 2 + (2 * math.sin(math.radians(75)) * 0.1) ** 2)) < 1e-12


def _coverage(true_t, true_R, sigma_t, sigma_rho_rad, n, reps, seed, r_ws=0.1):
    rng = np.random.default_rng(seed)
    e_true = exact_max_error(true_R, true_t, r_ws)
    hits = 0
    for _ in range(reps):
        Es = []
        for _ in range(n):
            dR = Rot.from_rotvec(rng.normal(size=3) * sigma_rho_rad).as_matrix()
            Es.append(G.make_pose(dR @ true_R, true_t + rng.normal(size=3) * sigma_t))
        sd = spatial_decision(Es, r_ws)
        hits += sd.ci_low_m <= e_true <= sd.ci_high_m
    return hits / reps


@pytest.mark.parametrize("case", ["zero", "boundary", "rotation"])
def test_interval_coverage_at_least_nominal(case):
    # the reviewer's Gaussian special case (n = 10, 0.1 mm per-axis noise) and two harder ones; the region is
    # conservative by construction, so the empirical coverage must be >= 0.95 (it is typically ~0.99)
    if case == "zero":
        cov = _coverage(np.zeros(3), np.eye(3), 1e-4, 0.0, 10, 400, 1)
    elif case == "boundary":
        cov = _coverage(np.array([0.0, 0.0, 0.001]), np.eye(3), 1e-4, 0.0, 10, 400, 2)
    else:
        cov = _coverage(np.array([0.0007, 0.0, 0.0]), G.axis_angle([1, 0, 0], math.radians(0.3)), 5e-5, math.radians(0.02), 10, 400, 3)
    assert cov >= 0.95, cov


def test_false_divergence_rate_at_tiny_tolerance_is_controlled():
    # reviewer's second statistic: at a 1 um tolerance and zero true error, RC3 declared divergence in 53 % of
    # replicates; a calibrated region must not exceed the nominal 5 % (it is far below: the region contains 0)
    rng = np.random.default_rng(5)
    false_div = 0
    reps = 400
    for _ in range(reps):
        Es = [G.make_pose(np.eye(3), rng.normal(size=3) * 1e-4) for _ in range(10)]
        sd = spatial_decision(Es, 0.1)
        false_div += decide(sd.ci_low_m, sd.ci_high_m, 1e-6) == Outcome.DIVERGENT
    assert false_div / reps <= 0.05


def test_too_few_trials_gives_undetermined():
    Es = [G.make_pose(np.eye(3), [0.0, 0.0, 0.0002 * k]) for k in range(3)]
    sd = spatial_decision(Es, 0.1)
    assert math.isinf(sd.ci_high_m) and decide(sd.ci_low_m, sd.ci_high_m, 0.001) == Outcome.UNDETERMINED


def test_hotelling_radius_zero_noise_is_zero():
    X = np.tile(np.array([[1.0, 2.0, 3.0]]), (10, 1))
    assert hotelling_radius(X, 0.025) == 0.0
