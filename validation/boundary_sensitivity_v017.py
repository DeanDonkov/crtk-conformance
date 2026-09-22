#!/usr/bin/env python3
"""Offline sensitivity of the released boundary decisions to departures from their error model (external review of
RC12, point 5).  No ROS.  The decision functions and the trial models are those of validation/boundary_montecarlo_v016.py
(frame: Hotelling T^2 regions -> exact E_max interval -> decide; scale: Student-t interval -> eq. (6)); only the
per-trial error process is changed, with the same marginal standard deviation (sigma = 0.1 mm):
  gauss   iid Gaussian (the model the intervals assume; as v0.1.6)
  ar1-0.5 / ar1-0.9   AR(1) correlation across the trials of a run (rho 0.5, 0.9)
  bias    a constant offset of 0.5 sigma on every trial of a run (unknown to the probe)
  drift   a linear drift across the trials of a run, total span 2 sigma, zero mean
  t3      Student-t (3 dof) heavy tails, rescaled to sigma
at E / epsilon in {0.95, 1.00, 1.05} (closed boundary; exact truth), N replicates per cell.

Usage: python3 validation/boundary_sensitivity_v017.py [--n-rep 2000] [--out validation/v0.1.7/studies]
"""
import argparse
import csv
import json
import math
import os
import sys
from fractions import Fraction

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from boundary_montecarlo_v016 import EPS, R_WS, TOL, classify, cp  # noqa: E402
from crtk_conformance.spatial import spatial_decision  # noqa: E402
from crtk_conformance.probes.base import decide  # noqa: E402
from crtk_conformance.dimensional import scale_decision  # noqa: E402
from crtk_conformance.stats import estimate  # noqa: E402

MODELS = ["gauss", "ar1-0.5", "ar1-0.9", "bias", "drift", "t3"]
RATIOS = ["0.95", "1.00", "1.05"]


def errors(rng, model, n, sd):
    """n x 3 trial errors with marginal standard deviation sd per axis."""
    if model == "gauss":
        return rng.normal(0, sd, (n, 3))
    if model.startswith("ar1"):
        rho = float(model.split("-")[1])
        e = np.zeros((n, 3)); e[0] = rng.normal(0, sd, 3)
        for i in range(1, n):
            e[i] = rho * e[i - 1] + rng.normal(0, sd * math.sqrt(1 - rho * rho), 3)
        return e
    if model == "bias":
        return rng.normal(0, sd, (n, 3)) + np.array([0.5 * sd, 0.0, 0.0])
    if model == "drift":
        ramp = np.linspace(-sd, sd, n)[:, None] * np.array([1.0, 0.0, 0.0])
        return rng.normal(0, sd, (n, 3)) + ramp
    if model == "t3":
        return rng.standard_t(3, (n, 3)) * sd / math.sqrt(3.0)
    raise ValueError(model)


def run(n_rep, seed, sigma_mm=0.1):
    rng = np.random.default_rng(seed)
    rows = []
    for probe in ("frame", "scale"):
        for model in MODELS:
            for rs in RATIOS:
                ratio = Fraction(rs); e_true = ratio * EPS; truth_c = e_true <= EPS
                counts = {"correct_C": 0, "correct_D": 0, "false_C": 0, "false_D": 0, "U": 0}; covered = 0
                for _ in range(n_rep):
                    if probe == "frame":
                        e = errors(rng, model, 10, math.sqrt(2.0 / 5.0) * sigma_mm * 1e-3)
                        Es = []
                        for i in range(10):
                            E = np.eye(4); E[:3, 3] = np.array([float(e_true), 0.0, 0.0]) + e[i]; Es.append(E)
                        sd = spatial_decision(Es, TOL.workspace_radius_m)
                        out = decide(sd.ci_low_m, sd.ci_high_m, TOL.epsilon_m).value
                        covered += int(sd.ci_low_m <= float(e_true) <= sd.ci_high_m)
                    else:
                        s_true = float(1 + e_true / R_WS)
                        e0 = errors(rng, model, 9, (sigma_mm * 1e-3 / s_true) / math.sqrt(30.0))
                        ratios = []
                        for k in range(9):
                            ax = k % 3; sign = 1 if (k // 3) % 2 == 0 else -1
                            d = np.zeros(3); d[ax] = sign * 0.005
                            ratios.append(s_true * float(np.linalg.norm(d + e0[k])) / 0.005)
                        o, e_pred = scale_decision(estimate(ratios), 1.0, TOL)
                        out = o.value
                        covered += int(e_pred.ci_low <= float(e_true) <= e_pred.ci_high)
                    counts[classify(out, truth_c)] += 1
                row = {"probe": probe, "model": model, "ratio": rs, "n_rep": n_rep, **counts, "coverage": covered / n_rep}
                for k in ("false_C", "false_D", "U"):
                    row[f"{k}_rate"] = counts[k] / n_rep; row[f"{k}_ci95"] = list(cp(counts[k], n_rep))
                rows.append(row)
                print(json.dumps({k: row[k] for k in ("probe", "model", "ratio", "correct_C", "correct_D", "false_C", "false_D", "U", "coverage")}), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-rep", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=76000)
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.7", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n_rep, a.seed)
    json.dump({"n_rep": a.n_rep, "seed": a.seed, "sigma_mm": 0.1, "rows": rows}, open(os.path.join(a.out, "boundary_sensitivity.json"), "w"), indent=1)
    with open(os.path.join(a.out, "boundary_sensitivity.csv"), "w", newline="") as f:
        cols = ["probe", "model", "ratio", "n_rep", "correct_C", "correct_D", "false_C", "false_D", "U", "false_C_rate", "false_D_rate", "U_rate", "coverage"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", lineterminator="\n"); w.writeheader(); w.writerows(rows)
