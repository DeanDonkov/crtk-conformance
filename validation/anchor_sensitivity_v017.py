#!/usr/bin/env python3
"""Offline sensitivity of the instrument-geometry unit anchor (external review of RC12, point 6).  No ROS.

The released functions are used unchanged: dimensional.screw_axis, dimensional.geometry_anchor_decision.  Only the
wrist kinematics are simulated: a pitch axis along z through the origin and a yaw axis at common-normal distance L_impl
(along y), at an angle theta to it; the published pose is rigidly attached to the yaw link, 10.6 mm beyond the yaw axis
(as SRC v1.0.0).  pose(q_p, q_y) = Rot_pitch(q_p) Rot_yaw(q_y) T_home.  A trial steps the pitch joint by dq from the
reference configuration (the yaw joint moving by c dq when coupled) and then the yaw joint by dq (pitch moving c dq);
each measured pose carries Gaussian translation noise sigma_t per axis and rotation noise sigma_r = sigma_t / 10 mm.
Interface unit: 1 m (SI), so the true lambda is 1 when L_impl = L_decl.

One factor at a time around a baseline (sigma_t 0.01 mm, dq 0.5 rad, k 5 trials, c 0, theta 90 deg, joint offsets 0,
u_L 1.5 %, L_impl = L_decl = 9.1 mm); N replicates per condition.  Per condition: median bias of lambda_hat, coverage of
lambda = 1 by the widened interval, gate pass rate, and the verdict of an SI client at 1 and 5 mm (r_ws = 0.1 m; truth
conformant unless L_impl != L_decl makes lambda_hat disagree with the declared L, which the anchor cannot see).

Usage: python3 validation/anchor_sensitivity_v017.py [--n 500] [--out validation/v0.1.7/studies]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from crtk_conformance.dimensional import geometry_anchor_decision, screw_axis  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402

L_DECL = 0.0091
BASE = {"sigma_t_mm": 0.01, "dq": 0.5, "k": 5, "c": 0.0, "theta_deg": 90.0, "offset_rad": 0.0, "u_L": 0.015, "L_impl_ratio": 1.0}
FACTORS = {
    "sigma_t_mm": [0.0, 0.01, 0.05, 0.1, 0.2],
    "dq": [0.1, 0.25, 0.5, 1.0],
    "k": [3, 5, 10],
    "c": [0.0, 0.005, 0.01, 0.02, 0.05, 0.1],
    "theta_deg": [90.0, 89.0, 88.0, 85.0],
    "offset_rad": [0.0, 0.1, 0.3],
    "u_L": [0.005, 0.015, 0.03],
    "L_impl_ratio": [1.0, 9.0 / 9.1, 0.97],
}


def rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def about_line(point, axis, ang):
    M = np.eye(4); R = rot(axis, ang)
    M[:3, :3] = R; M[:3, 3] = np.asarray(point) - R @ np.asarray(point)
    return M


def make_kinematics(L_impl, theta_deg):
    th = math.radians(theta_deg)
    k_p, o_p = np.array([0.0, 0.0, 1.0]), np.zeros(3)
    k_y, o_y = np.array([math.sin(th), 0.0, math.cos(th)]), np.array([0.0, L_impl, 0.0])
    T_home = np.eye(4); T_home[:3, 3] = [0.0, L_impl + 0.0106, 0.0]

    def pose(qp, qy):
        return about_line(o_p, k_p, qp) @ about_line(o_y, k_y, qy) @ T_home
    return pose


def noisy(T, rng, s_t, s_r):
    N = T.copy()
    if s_t > 0:
        N[:3, 3] = T[:3, 3] + rng.normal(0, s_t, 3)
        rv = rng.normal(0, s_r, 3)
        a = float(np.linalg.norm(rv))
        if a > 0:
            N[:3, :3] = rot(rv / a, a) @ T[:3, :3]
    return N


def replicate(rng, p):
    pose = make_kinematics(L_DECL * p["L_impl_ratio"], p["theta_deg"])
    s_t = p["sigma_t_mm"] * 1e-3; s_r = s_t / 0.010
    q0p, q0y = p["offset_rad"], -p["offset_rad"]
    P, Y = [], []
    for _ in range(int(p["k"])):
        T0 = pose(q0p, q0y)
        P.append(screw_axis(noisy(T0, rng, s_t, s_r), noisy(pose(q0p + p["dq"], q0y + p["c"] * p["dq"]), rng, s_t, s_r)))
        Y.append(screw_axis(noisy(T0, rng, s_t, s_r), noisy(pose(q0p + p["c"] * p["dq"], q0y + p["dq"]), rng, s_t, s_r)))
    out = {}
    for eps_mm in (1.0, 5.0):
        tol = Tolerance(epsilon_m=eps_mm / 1000.0, workspace_radius_m=0.10)
        g = geometry_anchor_decision(P, Y, p["dq"], L_DECL, p["u_L"], 1.0, tol)
        out[eps_mm] = g
    return out


def run(n, seed):
    rng = np.random.default_rng(seed)
    rows = []
    for fac, levels in FACTORS.items():
        for lv in levels:
            p = dict(BASE, **{fac: lv})
            lam, cov, gates, ver = [], 0, 0, {1.0: {"conformant": 0, "divergent": 0, "undetermined": 0}, 5.0: {"conformant": 0, "divergent": 0, "undetermined": 0}}
            for _ in range(n):
                g = replicate(rng, p)
                g5 = g[5.0]
                lam.append(g5.lambda_hat_m)
                cov += int(g5.lambda_ci_widened_m[0] <= 1.0 <= g5.lambda_ci_widened_m[1])
                gates += int(g5.gates_passed)
                for e in (1.0, 5.0):
                    ver[e][g[e].outcome] += 1
            lam = np.array(lam)
            rows.append({"factor": fac, "level": lv, "n": n, "lambda_bias_pct_median": float(np.median(lam) - 1.0) * 100, "lambda_sd_pct": float(np.std(lam)) * 100,
                         "coverage_widened": cov / n, "gate_pass": gates / n,
                         **{f"eps{int(e)}mm_{k}": v / n for e in (1.0, 5.0) for k, v in ver[e].items()}})
            print(json.dumps(rows[-1]), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=77000)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.7", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n, a.seed)
    json.dump({"n": a.n, "seed": a.seed, "baseline": BASE, "rows": rows}, open(os.path.join(a.out, "anchor_sensitivity.json"), "w"), indent=1)
    with open(os.path.join(a.out, "anchor_sensitivity.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n"); w.writeheader(); w.writerows(rows)
