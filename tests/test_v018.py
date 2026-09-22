"""0.1.8 (RC13 external review, points 1 and 3).

1. Frame-probe correlation guard (spatial.correlation_plan / correlation_guard): the correlation time of the paired
   residual, measured on a resting window, sets the trial spacing; the verdict is withheld when it is not resolved, when
   the spaced trials exceed the budget, or when fewer than four effectively independent trials remain.
2. Joint-space geometry anchor (dimensional.poe_fit_trial / geometry_anchor_poe_decision): the twists of every joint are
   fitted to the measured joint changes (product of exponentials), so coupled or incomplete joint motion does not bias
   the common normal of the wrist axes.
"""
import math

import numpy as np
import pytest

from crtk_conformance.dimensional import geometry_anchor_poe_decision, poe_fit_trial, se3_exp
from crtk_conformance.spatial import (block_mean_correlation, correlation_guard, correlation_plan, effective_trials,
                                      integrated_autocorr_time)
from crtk_conformance.thresholds import Tolerance

L = 0.0091
TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10)
# PSM-like chain at the reference configuration: outer yaw, outer pitch (about the RCM), insertion, roll, wrist pitch,
# wrist yaw (L distal of the pitch axis, perpendicular to it), published point 10.6 mm beyond the yaw axis
CHAIN = [((0, 0, 1), (0, 0, 0)), ((1, 0, 0), (0, 0, 0)), (None, (0, 0, -1)), ((0, 0, -1), (0, 0, 0)), ((1, 0, 0), (0, 0, -0.1)), ((0, 1, 0), (0, 0, -0.1 - L))]
T_REF = np.eye(4)
T_REF[:3, 3] = [0.0, 0.0, -0.1 - L - 0.0106]


def _fk(d, unit=1.0):
    T = np.eye(4)
    for (w, p), th in zip(CHAIN, d):
        if w is None:
            T = T @ se3_exp(np.zeros(3), np.array(p, float), th)
        else:
            w = np.array(w, float)
            T = T @ se3_exp(w, -np.cross(w, np.array(p, float)), th)
    T = T @ T_REF
    T[:3, 3] /= unit
    return T


def _trial(rng, unit=1.0, coupling=0.0, reach=(1, 1, 1, 1, 1, 1), noise=0.0):
    steps = [0.1, 0.1, 0.005, 0.25, 0.25, 0.25]
    qs, Ts = [], []
    for j in range(6):
        for s in (1, -1):
            d = np.zeros(6); d[j] = s * steps[j] * reach[j]
            d += coupling * rng.normal(0, 1, 6) * np.array([1, 1, 0.05, 1, 1, 1])
            T = _fk(d, unit)
            if noise:
                T[:3, 3] += rng.normal(0, noise, 3)
            q = d.copy(); q[2] /= unit  # the prismatic joint is reported in interface units
            qs.append(q); Ts.append(T)
    return poe_fit_trial("RRPRRR", np.zeros(6), _fk(np.zeros(6), unit), qs, Ts, 4, 5)


def test_poe_fit_recovers_the_wrist_common_normal_exactly():
    rng = np.random.default_rng(1)
    for unit in (1.0, 0.1):
        f = _trial(rng, unit)
        assert f.ok and f.d_int_if == pytest.approx(L / unit, rel=1e-9)
        assert f.axes_angle_deg == pytest.approx(90.0, abs=1e-6) and f.max_trans_rel < 1e-9


def test_poe_fit_is_unbiased_under_coupled_and_incomplete_joint_motion():
    # the SRC v1.0.0 failure mode: a wrist step also moves the other joints, and the yaw step reaches 44 % of the command
    rng = np.random.default_rng(2)
    fits = [_trial(rng, 0.1, coupling=0.02, reach=(1, 1, 1, 1, 1, 0.44)) for _ in range(5)]
    assert all(f.ok and f.d_int_if == pytest.approx(L / 0.1, rel=1e-9) for f in fits)
    g = geometry_anchor_poe_decision(fits, L, 0.015, 1.0, TOL)
    assert g.gates_passed and g.outcome == "divergent" and g.lambda_hat_m == pytest.approx(0.1, rel=1e-9)
    g5 = geometry_anchor_poe_decision([_trial(rng, 1.0) for _ in range(5)], L, 0.015, 1.0, Tolerance(epsilon_m=0.005, workspace_radius_m=0.10))
    assert g5.outcome == "conformant"
    g1 = geometry_anchor_poe_decision([_trial(rng, 1.0) for _ in range(5)], L, 0.015, 1.0, TOL)
    assert g1.outcome == "undetermined" and g1.gates_passed  # u_L r_ws = 1.5 mm > 1 mm


def test_poe_gates_withhold_a_poor_fit_and_a_wrist_that_did_not_move():
    rng = np.random.default_rng(3)
    noisy = [_trial(rng, 1.0, noise=1e-3) for _ in range(5)]  # 1 mm noise against a 9.1 mm common normal
    g = geometry_anchor_poe_decision(noisy, L, 0.015, 1.0, TOL)
    assert not g.gates_passed and g.outcome == "undetermined" and any("residual" in s for s in g.gate_failures)
    stuck = [_trial(rng, 1.0, reach=(1, 1, 1, 1, 1, 0.1)) for _ in range(5)]  # yaw realized 0.025 rad
    g = geometry_anchor_poe_decision(stuck, L, 0.015, 1.0, TOL)
    assert not g.gates_passed and any("moved only" in s for s in g.gate_failures)
    g = geometry_anchor_poe_decision(noisy[:2], L, 0.015, 1.0, TOL)
    assert not g.gates_passed


def _ar1(rng, phi, n):
    e = np.zeros(n); e[0] = rng.normal()
    for t in range(1, n):
        e[t] = phi * e[t - 1] + math.sqrt(1 - phi * phi) * rng.normal()
    return e


def test_integrated_autocorrelation_time():
    rng = np.random.default_rng(4)
    assert integrated_autocorr_time(rng.normal(size=2000))[0] == pytest.approx(1.0, abs=0.3)
    assert integrated_autocorr_time(_ar1(rng, 0.9, 20000))[0] == pytest.approx(19.0, rel=0.25)
    assert integrated_autocorr_time(np.zeros(100))[0] == 1.0


def test_correlation_plan_spaces_trials_and_guards():
    rng = np.random.default_rng(5)
    det = np.zeros((300, 6)); det[:, 0] = 0.2
    p = correlation_plan(det, 5, 10)
    assert p.deterministic and p.spacing_samples == 5 and correlation_guard(p, 10, 100.0)[0]
    iid = rng.normal(size=(300, 6)) * 1e-4
    p = correlation_plan(iid, 5, 10)
    assert p.resolved and p.spacing_samples <= 6 and p.effective_trials > 5 and correlation_guard(p, 10, 100.0)[0]
    ar = np.column_stack([_ar1(rng, 0.9, 3000) * 1e-4 for _ in range(3)] + [np.zeros(3000)] * 3)
    p = correlation_plan(ar, 5, 10)
    assert p.resolved and p.spacing_samples >= 2 * 15 and abs(p.between_trial_correlation) < 0.3
    ok, why = correlation_guard(p, 10, 100.0, max_sampling_s=1.0)
    assert not ok and "budget" in why
    short = correlation_plan(ar[:100], 5, 10)
    assert not short.resolved and not correlation_guard(short, 10, 100.0)[0]


def test_block_mean_correlation_and_effective_trials():
    acf = 0.9 ** np.arange(200)
    assert block_mean_correlation(acf, 5, 5) > 0.7
    assert block_mean_correlation(acf, 5, 60) < 0.05
    assert effective_trials(10, 0.0) == 10 and effective_trials(10, 0.9) < 1.0
