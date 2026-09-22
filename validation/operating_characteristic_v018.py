#!/usr/bin/env python3
"""Operating characteristic of the released frame and scale decisions (RC13 external review, point 2).  No ROS.

For a three-valued rule the unconditional error rate is not enough: near the boundary most runs abstain, and the few
determinate verdicts can be wrong much more often than the unconditional rate suggests.  This study reports, per cell,
the decision rate P(determinate), the unconditional false rate P(false), and the error among determinate verdicts
P(false | determinate), as functions of the distance from the boundary (E / epsilon), the position noise sigma and the
number of trials n.  Trial models, decision functions, exact truth and the closed boundary are those of
validation/boundary_montecarlo_v016.py (iid Gaussian trials; the frame and scale functions unchanged).

Usage: python3 validation/operating_characteristic_v018.py [--n-rep 1000] [--out validation/v0.1.8/studies]
"""
import argparse
import csv
import json
import os
import sys
from fractions import Fraction

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from boundary_montecarlo_v016 import EPS, R_WS, TOL, classify, cp, frame_trial_residuals, scale_trials  # noqa: E402
from crtk_conformance.spatial import spatial_decision  # noqa: E402
from crtk_conformance.probes.base import decide  # noqa: E402
from crtk_conformance.dimensional import scale_decision  # noqa: E402
from crtk_conformance.stats import estimate  # noqa: E402

RATIOS = ["0.50", "0.80", "0.90", "0.95", "0.98", "1.00", "1.02", "1.05", "1.10", "1.20", "1.50", "2.00"]
SIGMAS = [0.02, 0.1, 0.2]
TRIALS = {"frame": [5, 10, 20, 40], "scale": [5, 9, 20, 40]}


def run(n_rep, seed):
    rng = np.random.default_rng(seed)
    rows = []
    for probe in ("frame", "scale"):
        for sigma in SIGMAS:
            for n in TRIALS[probe]:
                for rs in RATIOS:
                    ratio = Fraction(rs); e_true = ratio * EPS; truth_c = e_true <= EPS
                    c = {"correct_C": 0, "correct_D": 0, "false_C": 0, "false_D": 0, "U": 0}
                    for _ in range(n_rep):
                        if probe == "frame":
                            sd = spatial_decision(frame_trial_residuals(rng, float(e_true), "gaussian", sigma, n), TOL.workspace_radius_m)
                            out = decide(sd.ci_low_m, sd.ci_high_m, TOL.epsilon_m).value
                        else:
                            s_true = float(1 + e_true / R_WS)
                            out = scale_decision(estimate(scale_trials(rng, s_true, sigma, n)), 1.0, TOL)[0].value
                        c[classify(out, truth_c)] += 1
                    det = n_rep - c["U"]
                    fk = "false_C" if not truth_c else "false_D"
                    lo, hi = cp(c[fk], det) if det else (float("nan"), float("nan"))
                    rows.append({"probe": probe, "sigma_mm": sigma, "trials": n, "ratio": rs, "n_rep": n_rep, **c,
                                 "decided": det / n_rep, "false": c[fk] / n_rep, "false_given_decided": (c[fk] / det) if det else None,
                                 "false_given_decided_ci95": [lo, hi]})
                print(json.dumps({k: rows[-1][k] for k in ("probe", "sigma_mm", "trials")}), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-rep", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=80000)
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n_rep, a.seed)
    json.dump({"n_rep": a.n_rep, "seed": a.seed, "rows": rows}, open(os.path.join(a.out, "operating_characteristic.json"), "w"), indent=1)
    with open(os.path.join(a.out, "operating_characteristic.csv"), "w", newline="") as f:
        cols = ["probe", "sigma_mm", "trials", "ratio", "n_rep", "correct_C", "correct_D", "false_C", "false_D", "U", "decided", "false", "false_given_decided"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore", lineterminator="\n"); w.writeheader(); w.writerows(rows)
