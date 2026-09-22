#!/usr/bin/env python3
"""Offline study of the 0.1.8 frame-probe correlation guard (RC13 external review, point 1).  No ROS.

The released functions are used unchanged: spatial.correlation_plan, spatial.correlation_guard, spatial.residual_series,
spatial.spatial_decision and probes.base.decide.  Only the paired-residual stream is simulated, sample by sample at
100 Hz: translation error e_k (3 axes) = phi e_{k-1} + sqrt(1 - phi^2) N(0, sigma_s^2), with sigma_s = sqrt(2) x sigma
(two independent channels of sigma = 0.02 or 0.1 mm), plus the true residual t_true = (E/epsilon) x epsilon along x; no rotation.
Two procedures on the same stream:
  0.1.7  ten trials of m = 5 consecutive samples, back to back (no resting window);
  0.1.8  a resting window of 300 samples (3 s), extended in steps of 100 samples (1 s) up to 3000 (30 s) until it spans 20 x
         tau_int; trials spaced by max(5, ceil(2 tau_int)) samples; verdict withheld if unresolved, if the spaced trials
         exceed 60 s, or if fewer than 4 effectively independent trials remain.
Truth is labelled exactly (closed boundary).  phi in {0, 0.5, 0.8, 0.9, 0.95, 0.98, 0.99} per sample; the implied
correlation between back-to-back trial means is reported (0.99 per sample ~ 0.96 between trials).

Usage: python3 validation/frame_guard_study_v018.py [--n-rep 1000] [--out validation/v0.1.8/studies]
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
from boundary_montecarlo_v016 import EPS, TOL, classify, cp  # noqa: E402
from crtk_conformance.spatial import (block_mean_correlation, correlation_guard, correlation_plan, residual_series,  # noqa: E402
                                      spatial_decision)
from crtk_conformance.probes.base import decide  # noqa: E402

PHIS = [0.0, 0.5, 0.8, 0.9, 0.95, 0.98, 0.99]
RATIOS = ["0.95", "1.00", "1.05"]
M, N, RATE = 5, 10, 100.0
W0, WSTEP, WMAX = 300, 100, 3000


def stream(rng, phi, n, sd):
    from scipy.signal import lfilter
    z = rng.normal(0, 1, (n, 3)) * math.sqrt(max(0.0, 1 - phi * phi)) * sd
    z[0] = rng.normal(0, sd, 3)  # stationary start
    return lfilter([1.0], [1.0, -phi], z, axis=0)


def poses(block, t_true):
    out = []
    for x in block:
        T = np.eye(4); T[:3, 3] = x + np.array([t_true, 0.0, 0.0]); out.append(T)
    return out


def decide_trials(trials):
    sd = spatial_decision(trials, TOL.workspace_radius_m)
    return decide(sd.ci_low_m, sd.ci_high_m, TOL.epsilon_m).value, sd


def one(rng, phi, t_true, sd):
    n_total = WMAX + N * 800 + 10
    e = stream(rng, phi, n_total, sd)
    # 0.1.7: back to back from the start
    tr = [np.mean(poses(e[i * M:(i + 1) * M], t_true), axis=0) for i in range(N)]
    for T in tr:
        T[3, :] = [0, 0, 0, 1]
    o17, _ = decide_trials(tr)
    # 0.1.8: resting window, spacing, guard
    w = W0
    # the rotation components of residual_series are noise-free here (skipped as deterministic), so the plan of the
    # translation series equals the plan of residual_series(poses(...)); checked on the first replicate of each cell
    plan = correlation_plan(e[:w], M, N)
    while not plan.resolved and w < WMAX:
        w = min(WMAX, w + WSTEP)
        plan = correlation_plan(e[:w], M, N)
    ok, why = correlation_guard(plan, N, RATE)
    sp = plan.spacing_samples if ok else M
    start = w
    tr = []
    for i in range(N):
        blk = e[start + i * sp: start + i * sp + M]
        T = np.eye(4); T[:3, 3] = blk.mean(axis=0) + np.array([t_true, 0.0, 0.0]); tr.append(T)
    o18u, _ = decide_trials(tr)
    o18 = o18u if ok or o18u == "undetermined" else "undetermined"
    reason = "" if ok else ("unresolved" if not plan.resolved else ("budget" if "budget" in why else "neff"))
    return o17, o18, reason, plan, w


def run(n_rep, seed, sigmas=(0.02, 0.1)):
    rng = np.random.default_rng(seed)
    rows = []
    for sigma_mm in sigmas:
      sd = math.sqrt(2.0) * sigma_mm * 1e-3
      for phi in PHIS:
          acf = phi ** np.arange(4000) if phi > 0 else np.array([1.0])
          rho_b2b = block_mean_correlation(acf, M, M)
          for rs in RATIOS:
              ratio = Fraction(rs); e_true = ratio * EPS; truth_c = e_true <= EPS
              c17 = {"correct_C": 0, "correct_D": 0, "false_C": 0, "false_D": 0, "U": 0}
              c18 = dict(c17)
              reasons = {"unresolved": 0, "budget": 0, "neff": 0}
              taus, spacings, windows = [], [], []
              for rep in range(n_rep):
                  o17, o18, reason, plan, w = one(rng, phi, float(e_true), sd)
                  c17[classify(o17, truth_c)] += 1
                  c18[classify(o18, truth_c)] += 1
                  if reason:
                      reasons[reason] += 1
                  taus.append(plan.tau_int_samples); spacings.append(plan.spacing_samples); windows.append(w)
              row = {"sigma_mm": sigma_mm, "phi": phi, "rho_trials_back_to_back": rho_b2b, "tau_true": (1 + phi) / (1 - phi), "ratio": rs, "n_rep": n_rep,
                     "v017": c17, "v018": c18, "withheld": reasons, "tau_median": float(np.median(taus)), "spacing_median": float(np.median(spacings)),
                     "window_median": float(np.median(windows))}
              for tag, c in (("v017", c17), ("v018", c18)):
                  fk = "false_C" if float(rs) > 1 else "false_D"
                  det = n_rep - c["U"]
                  row[f"{tag}_false_rate"] = c[fk] / n_rep
                  row[f"{tag}_false_ci95"] = list(cp(c[fk], n_rep))
                  row[f"{tag}_decided"] = det / n_rep
                  row[f"{tag}_false_given_decided"] = (c[fk] / det) if det else None
              rows.append(row)
              print(json.dumps({k: row[k] for k in ("sigma_mm", "phi", "ratio", "v017", "v018", "withheld", "tau_median", "spacing_median")}), flush=True)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-rep", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=78000)
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = run(a.n_rep, a.seed)
    json.dump({"n_rep": a.n_rep, "seed": a.seed, "rows": rows}, open(os.path.join(a.out, "frame_guard.json"), "w"), indent=1)
    with open(os.path.join(a.out, "frame_guard.csv"), "w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["sigma_mm", "phi", "rho_trials_back_to_back", "ratio", "v017_false_rate", "v017_decided", "v018_false_rate", "v018_decided", "withheld_unresolved", "withheld_budget", "withheld_neff", "tau_median", "spacing_median"])
        for r in rows:
            w.writerow([r["sigma_mm"], r["phi"], round(r["rho_trials_back_to_back"], 4), r["ratio"], r["v017_false_rate"], r["v017_decided"], r["v018_false_rate"], r["v018_decided"],
                        r["withheld"]["unresolved"], r["withheld"]["budget"], r["withheld"]["neff"], r["tau_median"], r["spacing_median"]])
