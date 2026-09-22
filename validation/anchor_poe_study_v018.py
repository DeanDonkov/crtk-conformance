#!/usr/bin/env python3
"""Offline sensitivity of the 0.1.8 joint-space (product-of-exponentials) geometry anchor (RC13 external review, points
3 and 4).  No ROS.  The released functions are used unchanged: dimensional.poe_fit_trial, dimensional.geometry_anchor_poe_decision.

Simulated arm: a PSM-like serial chain at its reference configuration (outer yaw and pitch about the remote centre,
insertion, roll, wrist pitch 0.1 m down the tool, wrist yaw L_impl distal of it at the given angle, published point
10.6 mm beyond the yaw axis), interface in SI.  A trial measures the reference pose and then steps every joint by +- its
step (outer joints 0.1 rad, insertion 5 %, roll and wrist step_rad) as the probe does.  Imperfect execution: every joint
also moves by coupling x step x N(0, 1) in each step, and the wrist-yaw step reaches `reach` of its command; both are
REPORTED in measured_js, as on SRC v1.0.0.  Outside the model: `kappa` scales the reported wrist-joint changes (an
encoder-scale error), so measured_js no longer produced measured_cp.  Every averaged pose carries Gaussian translation
noise sigma_t per axis and rotation noise sigma_t / 10 mm.

One factor at a time around the baseline (sigma_t 0.01 mm, step 0.25 rad, k 5, coupling 0, reach 1, axes 90 deg, u_L
1.5 %, L_impl = L = 9.1 mm, kappa 1), N replicates per level; verdicts of an SI client at 1 and 5 mm (r_ws = 0.1 m).

Usage: python3 validation/anchor_poe_study_v018.py [--n 100] [--out validation/v0.1.8/studies]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.dimensional import geometry_anchor_poe_decision, poe_fit_trial, se3_exp  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402

L_DECL = 0.0091
BASE = {"sigma_t_mm": 0.01, "step_rad": 0.25, "k": 5, "coupling": 0.0, "reach": 1.0, "theta_deg": 90.0, "u_L": 0.015, "L_impl_ratio": 1.0, "kappa": 1.0}
FACTORS = {
    "sigma_t_mm": [0.0, 0.01, 0.02, 0.05, 0.1],
    "step_rad": [0.1, 0.25, 0.5],
    "k": [3, 5, 10],
    "coupling": [0.0, 0.02, 0.05, 0.1],
    "reach": [1.0, 0.44],
    "theta_deg": [90.0, 89.0, 88.0, 85.0],
    "u_L": [0.005, 0.015, 0.03],
    "L_impl_ratio": [1.0, 9.0 / 9.1, 0.97],
    "kappa": [1.0, 0.98, 0.95],
}


def chain(L_impl, theta_deg):
    th = math.radians(theta_deg)
    return [((0, 0, 1), (0, 0, 0)), ((1, 0, 0), (0, 0, 0)), (None, (0, 0, -1)), ((0, 0, -1), (0, 0, 0)),
            ((1, 0, 0), (0, 0, -0.1)), ((math.cos(th), math.sin(th), 0), (0, 0, -0.1 - L_impl))]


def fk(ch, T_ref, d):
    T = np.eye(4)
    for (w, p), a in zip(ch, d):
        if w is None:
            T = T @ se3_exp(np.zeros(3), np.array(p, float), a)
        else:
            w = np.array(w, float)
            T = T @ se3_exp(w, -np.cross(w, np.array(p, float)), a)
    return T @ T_ref


def rot(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def noisy(T, rng, s_t):
    if s_t <= 0:
        return T
    N = T.copy()
    N[:3, 3] += rng.normal(0, s_t, 3)
    rv = rng.normal(0, s_t / 0.010, 3)
    a = float(np.linalg.norm(rv))
    if a > 0:
        N[:3, :3] = rot(rv / a, a) @ T[:3, :3]
    return N


def trial(rng, p):
    L_impl = L_DECL * p["L_impl_ratio"]
    ch = chain(L_impl, p["theta_deg"])
    T_ref = np.eye(4); T_ref[:3, 3] = [0.0, 0.0, -0.1 - L_impl - 0.0106]
    steps = [0.1, 0.1, 0.05 * 0.1, p["step_rad"], p["step_rad"], p["step_rad"]]
    s_t = p["sigma_t_mm"] * 1e-3
    qs, Ts = [], []
    for j in range(6):
        for sg in (1.0, -1.0):
            d = np.zeros(6); d[j] = sg * steps[j] * (p["reach"] if j == 5 else 1.0)
            if p["coupling"] > 0:
                d += p["coupling"] * steps[j] * rng.normal(0, 1, 6) * np.array([1, 1, 0.05, 1, 1, 1])
            T = noisy(fk(ch, T_ref, d), rng, s_t)
            q = d.copy(); q[4] *= p["kappa"]; q[5] *= p["kappa"]  # reported joint changes
            qs.append(q); Ts.append(T)
    return poe_fit_trial("RRPRRR", np.zeros(6), noisy(fk(ch, T_ref, np.zeros(6)), rng, s_t), qs, Ts, 4, 5)


def run(n, seed):
    rng = np.random.default_rng(seed)
    rows = []
    for fac, levels in FACTORS.items():
        for lv in levels:
            p = dict(BASE, **{fac: lv})
            lam, cov, gates = [], 0, 0
            ver = {e: {"conformant": 0, "divergent": 0, "undetermined": 0} for e in (1.0, 5.0)}
            fails = {}
            for _ in range(n):
                fits = [trial(rng, p) for _ in range(int(p["k"]))]
                for e in (1.0, 5.0):
                    g = geometry_anchor_poe_decision(fits, L_DECL, p["u_L"], 1.0, Tolerance(epsilon_m=e / 1000.0, workspace_radius_m=0.10))
                    ver[e][g.outcome] += 1
                lam.append(g.lambda_hat_m)
                cov += int(g.lambda_ci_widened_m[0] <= 1.0 <= g.lambda_ci_widened_m[1])
                gates += int(g.gates_passed)
                for fmsg in g.gate_failures:
                    key = "residual" if "residual" in fmsg else ("excitation" if "moved only" in fmsg else ("axes" if "axes" in fmsg else "other"))
                    fails[key] = fails.get(key, 0) + 1
            lam = np.array(lam)
            rows.append({"factor": fac, "level": lv, "n": n, "lambda_bias_pct_median": float(np.median(lam) - 1.0) * 100, "lambda_sd_pct": float(np.std(lam)) * 100,
                         "coverage_widened": cov / n, "gate_pass": gates / n, "gate_failure_kinds": fails,
                         **{f"eps{int(e)}mm_{k}": v / n for e in (1.0, 5.0) for k, v in ver[e].items()}})
            print(json.dumps(rows[-1]), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--seed", type=int, default=79000)
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n, a.seed)
    json.dump({"n": a.n, "seed": a.seed, "baseline": BASE, "rows": rows}, open(os.path.join(a.out, "anchor_poe.json"), "w"), indent=1)
    with open(os.path.join(a.out, "anchor_poe.csv"), "w", newline="") as f:
        cols = [c for c in rows[0] if c != "gate_failure_kinds"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", lineterminator="\n"); w.writeheader(); w.writerows(rows)
