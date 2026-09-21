#!/usr/bin/env python3
"""WP5 (RC9 / external review, point 3): decision reliability of the released decision rules near the tolerance
boundary, offline.  No ROS.  The decision functions are the released ones, unchanged:
  frame: spatial.spatial_decision (Hotelling T^2 regions propagated to the exact maximum eq. (3')) + probes.base.decide
  scale: dimensional.scale_decision (eq. (6), moved unchanged from ScalingUnitsProbe in 0.1.6) on stats.estimate
Only the trial-level error model is simulated:
  frame  per-trial residual E_i = (I, t_true + eps_i); Gaussian eps_i ~ N(0, (2/5) sigma^2 I) -- the probe averages five
         measured/local pairs whose positions carry independent N(0, sigma^2) noise -- or the RC8 mixture per trial
         (x: -20 um w.p. 0.95, +380 um w.p. 0.05, plus N(0, (1 um)^2) on every axis; zero mean)
  scale  per-trial anchored ratio s_i = s ||delta + e0_i|| / ||delta||, delta = 5 mm along the trial's axis (x, y, z,
         alternating sign, as the probe), e0_i ~ N(0, (sigma / s)^2 / 30 I): the goal is the measured start pose (mean
         of the 0.3-s pre-window at 100 Hz) plus delta, and the SI-pose anchor is exact
Truth is labelled EXACTLY (fractions): a case is conformant iff E_true <= epsilon (closed boundary), where E_true =
ratio x epsilon with ratio in {0.90, 0.95, 0.98, 1.00, 1.02, 1.05, 1.10}.  Outcomes: correct C, correct D, false C
(truth divergent, verdict conformant), false D (truth conformant, verdict divergent), U; Clopper-Pearson 95 % intervals.

Usage: python3 validation/boundary_montecarlo_v016.py [--n-rep 2000] [--out validation/v0.1.6/boundary]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from fractions import Fraction

import numpy as np
from scipy.stats import beta

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from crtk_conformance.spatial import spatial_decision  # noqa: E402
from crtk_conformance.probes.base import decide  # noqa: E402
from crtk_conformance.dimensional import scale_decision  # noqa: E402
from crtk_conformance.stats import estimate  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402

RATIOS = ["0.90", "0.95", "0.98", "1.00", "1.02", "1.05", "1.10"]
EPS = Fraction(1, 1000)  # 1 mm
R_WS = Fraction(1, 10)
TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10)


def cp(k, n, conf=0.95):
    a = (1 - conf) / 2
    lo = 0.0 if k == 0 else float(beta.ppf(a, k, n - k + 1))
    hi = 1.0 if k == n else float(beta.ppf(1 - a, k + 1, n - k))
    return lo, hi


def frame_trial_residuals(rng, t_true_m: float, model: str, sigma_mm: float, n: int):
    Es = []
    for _ in range(n):
        if model == "gaussian":
            e = rng.normal(0.0, math.sqrt(2.0 / 5.0) * sigma_mm * 1e-3, 3)
        else:  # RC8 mixture per trial
            e = rng.normal(0.0, 1e-6, 3)
            e[0] += 380e-6 if rng.random() < 0.05 else -20e-6
        E = np.eye(4)
        E[:3, 3] = np.array([t_true_m, 0.0, 0.0]) + e
        Es.append(E)
    return Es


def scale_trials(rng, s_true: float, sigma_mm: float, n: int):
    out = []
    step = 0.005
    for k in range(n):
        ax = k % 3
        sign = 1 if (k // 3) % 2 == 0 else -1
        d = np.zeros(3); d[ax] = sign * step
        e0 = rng.normal(0.0, (sigma_mm * 1e-3 / s_true) / math.sqrt(30.0), 3)
        out.append(s_true * float(np.linalg.norm(d + e0)) / step)
    return out


def classify(outcome: str, truth_conformant: bool) -> str:
    if outcome == "undetermined":
        return "U"
    if outcome == "conformant":
        return "correct_C" if truth_conformant else "false_C"
    return "false_D" if truth_conformant else "correct_D"


def run(n_rep: int, seed: int):
    rng = np.random.default_rng(seed)
    rows = []
    cells = [("frame", "gaussian", 0.02), ("frame", "gaussian", 0.1), ("frame", "mixture", 0.0),
             ("scale", "gaussian", 0.02), ("scale", "gaussian", 0.1)]
    for probe, model, sigma in cells:
        for rs in RATIOS:
            ratio = Fraction(rs)
            e_true = ratio * EPS  # exact
            truth_c = e_true <= EPS
            counts = {"correct_C": 0, "correct_D": 0, "false_C": 0, "false_D": 0, "U": 0}
            covered = 0
            for _ in range(n_rep):
                if probe == "frame":
                    Es = frame_trial_residuals(rng, float(e_true), model, sigma, 10)
                    sd = spatial_decision(Es, TOL.workspace_radius_m)
                    out = decide(sd.ci_low_m, sd.ci_high_m, TOL.epsilon_m).value
                    covered += int(sd.ci_low_m <= float(e_true) <= sd.ci_high_m)
                else:
                    s_true = float(1 + e_true / R_WS)  # |1 - s| r_ws = e_true, s > 1
                    e_s = estimate(scale_trials(rng, s_true, sigma, 9))
                    o, e_pred = scale_decision(e_s, 1.0, TOL)
                    out = o.value
                    covered += int(e_pred.ci_low <= float(e_true) <= e_pred.ci_high)
                counts[classify(out, truth_c)] += 1
            row = {"probe": probe, "noise_model": model, "sigma_mm": sigma, "ratio": rs, "E_true_mm": float(e_true) * 1e3,
                   "truth": "conformant" if truth_c else "divergent", "n_rep": n_rep, **counts, "interval_coverage": covered / n_rep}
            for k in ("false_C", "false_D", "U"):
                lo, hi = cp(counts[k], n_rep)
                row[f"{k}_rate"] = counts[k] / n_rep
                row[f"{k}_ci95"] = [lo, hi]
            rows.append(row)
            print(json.dumps({k: row[k] for k in ("probe", "noise_model", "sigma_mm", "ratio", "correct_C", "correct_D", "false_C", "false_D", "U", "interval_coverage")}), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-rep", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=31000)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "boundary"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n_rep, a.seed)
    json.dump({"n_rep": a.n_rep, "seed": a.seed, "epsilon_m": 0.001, "r_ws_m": 0.10, "rows": rows}, open(os.path.join(a.out, "boundary_montecarlo.json"), "w"), indent=1)
    with open(os.path.join(a.out, "boundary_montecarlo.csv"), "w", newline="") as f:
        cols = ["probe", "noise_model", "sigma_mm", "ratio", "E_true_mm", "truth", "n_rep", "correct_C", "correct_D", "false_C", "false_D", "U",
                "false_C_rate", "false_D_rate", "U_rate", "interval_coverage"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
