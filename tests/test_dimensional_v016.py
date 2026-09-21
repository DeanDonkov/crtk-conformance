"""0.1.6: dimensional.scale_decision reproduces every archived anchored-scale outcome; the instrument-geometry anchor
recovers the unit from DH-generated poses whatever the published point, and its decisions follow the declared u_rel."""
import glob
import json
import math
import os

import numpy as np
import pytest

from crtk_conformance.dimensional import common_normal, geometry_anchor_decision, scale_decision, screw_axis, rotation_vector_angle
from crtk_conformance.stats import Estimate
from crtk_conformance.thresholds import Tolerance

ARCH = os.path.join(os.path.dirname(__file__), "..", "validation", "v0.1.3", "mock")


def test_scale_decision_replays_archive():
    files = sorted(glob.glob(os.path.join(ARCH, "S_*.json")))
    assert files
    checked = 0
    meta = json.load(open(os.path.join(ARCH, "meta.json")))
    tol = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10)  # the campaign's task parameters (meta.json, RC8 Sec. VI-A)
    for f in files:
        rec = json.load(open(f))
        for p in [rec["result"]] if isinstance(rec.get("result"), dict) else []:
            if p["probe"] != "ScalingUnitsProbe" or "expected_unit_m" not in p["estimates"]:
                continue
            e = p["estimates"]["scale_anchored"]
            est = Estimate(e["n"], e["mean"], e["std"] if e["std"] is not None else float("nan"), e["ci_low"], e["ci_high"], e["alpha"])
            out, e_pred = scale_decision(est, p["estimates"]["expected_unit_m"], tol)
            assert out.value == p["outcome"], f
            pe = p["estimates"]["predicted_error_at_workspace_edge_m"]
            assert math.isclose(e_pred.ci_low, pe["ci_low"], rel_tol=1e-12, abs_tol=1e-15)
            assert math.isclose(e_pred.ci_high, pe["ci_high"], rel_tol=1e-12, abs_tol=1e-15)
            checked += 1
    assert checked >= 27


# ---- modified-DH wrist of the Large Needle Driver (dVRK tool file): roll (D = L_tool), pitch (alpha -pi/2, offset -pi/2),
# yaw (alpha -pi/2, A = L_p2y, offset -pi/2), then an optional control-point row (SRC v1: D = 0.106 units, offset pi/2)
def _mdh(alpha, a, theta, d):
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0, a], [st * ca, ct * ca, -sa, -sa * d], [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1.0]])


def _wrist_pose(q_pitch, q_yaw, unit_m, L_p2y_m=0.0091, L_tool_m=0.4162, tip_m=0.0, base=None):
    s = 1.0 / unit_m  # interface units per metre
    T = np.eye(4) if base is None else base.copy()
    T = T @ _mdh(0.0, 0.0, 0.3, L_tool_m * s)
    T = T @ _mdh(-math.pi / 2, 0.0, q_pitch - math.pi / 2, 0.0)
    T = T @ _mdh(-math.pi / 2, L_p2y_m * s, q_yaw - math.pi / 2, 0.0)
    if tip_m:
        T = T @ _mdh(-math.pi / 2, 0.0, math.pi / 2, tip_m * s)
    return T


def _axes(unit_m, tip_m=0.0, dq=0.5, L=0.0091, noise=0.0, rng=None, n=5):
    base = np.eye(4)
    base[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    base[:3, 3] = [0.05 / unit_m, -0.02 / unit_m, 0.1 / unit_m]
    P, Y = [], []
    for _ in range(n):
        def pose(qp, qy):
            T = _wrist_pose(qp, qy, unit_m, L_p2y_m=L, tip_m=tip_m, base=base)
            if noise and rng is not None:
                T = T.copy(); T[:3, 3] += rng.normal(0, noise / unit_m, 3)
            return T
        P.append(screw_axis(pose(0.1, -0.2), pose(0.1 + dq, -0.2)))
        Y.append(screw_axis(pose(0.1, -0.2), pose(0.1, -0.2 + dq)))
    return P, Y


@pytest.mark.parametrize("unit_m,tip_m", [(1.0, 0.0), (0.1, 0.0106), (1.0, 0.0102), (0.001, 0.0)])
def test_common_normal_is_the_pitch_to_yaw_length_whatever_the_point(unit_m, tip_m):
    P, Y = _axes(unit_m, tip_m=tip_m)
    for p, y in zip(P, Y):
        d, s = common_normal(p, y)
        assert math.isclose(d * unit_m, 0.0091, rel_tol=1e-9)
        assert math.isclose(s, 1.0, rel_tol=1e-9)
        assert math.isclose(p.angle_rad, 0.5, rel_tol=1e-9) and p.slide_if < 1e-9


def test_rotation_vector_angle_has_no_acos_floor():
    th = 3e-10
    R = np.array([[1, 0, 0], [0, math.cos(th), -math.sin(th)], [0, math.sin(th), math.cos(th)]])
    assert math.isclose(rotation_vector_angle(R), th, rel_tol=1e-6)


def _decide(unit_true, L_phys, u_rel, expected, eps_mm, L_model=0.0091, tip_m=0.0):
    P, Y = _axes(unit_true, tip_m=tip_m, L=L_model)
    return geometry_anchor_decision(P, Y, 0.5, L_phys, u_rel, expected, Tolerance(epsilon_m=eps_mm / 1000.0, workspace_radius_m=0.10))


def test_geometry_anchor_decisions_match_the_preregistered_expectations():
    # dVRK-like (SI, model 9.1 mm): SI client U at 1 mm (u_rel 1.5 % -> up to 1.5 mm), C at 5 mm; mm client D
    assert _decide(1.0, 0.0091, 0.015, 1.0, 1.0).outcome == "undetermined"
    assert _decide(1.0, 0.0091, 0.015, 1.0, 5.0).outcome == "conformant"
    assert _decide(1.0, 0.0091, 0.015, 0.001, 1.0).outcome == "divergent"
    # SRC v2-like (SI, model 9.0 mm) against L = 9.1 mm: lambda_hat = 1.0111 -> 1.11 mm; U at 1 mm, C at 5 mm
    d = _decide(1.0, 0.0091, 0.015, 1.0, 1.0, L_model=0.009)
    assert math.isclose(d.lambda_hat_m, 0.0091 / 0.009, rel_tol=1e-9) and d.outcome == "undetermined"
    assert _decide(1.0, 0.0091, 0.015, 1.0, 5.0, L_model=0.009).outcome == "conformant"
    # without the declared uncertainty the 1.1 % model disagreement would be a (false) divergence at 1 mm
    assert _decide(1.0, 0.0091, 0.0, 1.0, 1.0, L_model=0.009).outcome == "divergent"
    # SRC v1-like (0.1 m units, model 9.0 mm, control point 10.6 mm beyond the yaw axis): D at 1 and 5 mm
    d1 = _decide(0.1, 0.0091, 0.015, 1.0, 1.0, L_model=0.009, tip_m=0.0106)
    assert math.isclose(d1.lambda_hat_m, 0.0091 / 0.09, rel_tol=1e-9) and d1.outcome == "divergent"
    assert _decide(0.1, 0.0091, 0.015, 1.0, 5.0, L_model=0.009, tip_m=0.0106).outcome == "divergent"


def test_gates_withhold_a_verdict():
    P, Y = _axes(1.0, dq=0.5)
    d = geometry_anchor_decision(P, Y, 0.6, 0.0091, 0.015, 1.0, Tolerance(epsilon_m=0.005))  # commanded 0.6, executed 0.5
    assert d.outcome == "undetermined" and not d.gates_passed and d.gate_failures
    d = geometry_anchor_decision(P[:2], Y[:2], 0.5, 0.0091, 0.015, 1.0, Tolerance(epsilon_m=0.005))
    assert d.outcome == "undetermined"


def test_noisy_anchor_interval_covers_truth():
    rng = np.random.default_rng(3)
    P, Y = _axes(1.0, noise=2e-6, rng=rng, n=8)
    d = geometry_anchor_decision(P, Y, 0.5, 0.0091, 0.0, None, Tolerance())
    assert d.lambda_ci_m[0] <= 1.0 <= d.lambda_ci_m[1]
