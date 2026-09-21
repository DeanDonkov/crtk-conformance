#!/usr/bin/env python3
"""RC9 (external review, point 3): Fig. 1(d) with the exact 1-mm point, and the decisions of the mixture-noise stress
case next to its coverage.  Offline, reads the frozen v0.1.3 archive (unchanged) and writes to v0.1.6/rescoring.

(1) Tolerance sweep with exact truth.  analyze_v013.py labels a run positive (divergent truth) when the ARCHIVED
    float truth exceeds epsilon; the archived truth of S_si at s = 1.01 is 1.0000000000000009 mm (floating-point
    |1 - 1.01| x 0.1 m), so at epsilon = 1 mm that run was labelled divergent and its divergent verdict scored as
    correct.  Here the truth is computed exactly from the decimal injected value (Decimal(repr(s)): |1 - 1.01| x 100 mm
    = 1 mm exactly for the scale runs; for the frame runs with theta = 0 the exact maximum error is |t|, Decimal(repr)
    of the injected translation; for theta != 0 the archived float truth is used -- none lies within 0.7 mm of 1 mm)
    under the closed boundary (truth conformant iff E <= epsilon).  The grid is the archived 61-point grid plus
    epsilon = 1 mm exactly.  Verdicts are recomputed with the released decide() (1e-9 relative guard band) from the
    archived intervals, so they equal the archived verdicts wherever epsilon = 1 mm was the run tolerance.
(2) The RC8 mixture stress case (tests/test_spatial.py, seed 20260909, same draw order), 2000 replicates: coverage of
    the true zero error and the C/D/U decisions at epsilon = 1 um (the reviewer's tolerance), 10 um, 0.1 mm, 1 mm.

Usage: python3 validation/rescore_v016.py [validation/v0.1.3/mock] [--out validation/v0.1.6/rescoring]
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
from decimal import Decimal

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance import geometry as G  # noqa: E402
from crtk_conformance.probes.base import decide  # noqa: E402
from crtk_conformance.spatial import spatial_decision  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("archive", nargs="?", default=os.path.join(HERE, "v0.1.3", "mock"))
ap.add_argument("--out", default=os.path.join(HERE, "v0.1.6", "rescoring"))
ap.add_argument("--mixture-reps", type=int, default=2000)
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)
R_WS = Decimal("0.1")


def frame_records():
    recs = []
    for p in sorted(glob.glob(os.path.join(args.archive, "F_id_*.json"))):
        d = json.load(open(p)); tr = d["truth"]; sd = d["result"]["estimates"].get("spatial_decision") or {}
        if tr["theta_deg"] == 0:
            comps = [Decimal(repr(float(v))) for v in tr["t_m"]]
            nz = [c for c in comps if c != 0]
            if len(nz) <= 1:
                truth, exact = (abs(nz[0]) if nz else Decimal(0)), True
            else:
                truth, exact = Decimal(repr(tr["exact_max_error_m"])), False
        else:
            truth, exact = Decimal(repr(tr["exact_max_error_m"])), False
        recs.append({"run": os.path.basename(p)[:-5], "truth_m": truth, "truth_exact": exact, "noise_mm": tr["noise_m"] * 1e3,
                     "lo": sd.get("ci_low_m"), "hi": sd.get("ci_high_m"), "archived_outcome": d["result"]["outcome"], "archived_truth_m": tr["exact_max_error_m"]})
    return recs


def scale_records():
    recs = []
    for p in sorted(glob.glob(os.path.join(args.archive, "S_si_*.json"))):
        d = json.load(open(p)); tr = d["truth"]; pe = d["result"]["estimates"].get("predicted_error_at_workspace_edge_m", {})
        truth = abs(Decimal(1) - Decimal(repr(float(tr["s"])))) * R_WS
        recs.append({"run": os.path.basename(p)[:-5], "truth_m": truth, "truth_exact": True, "noise_mm": tr["noise_m"] * 1e3, "s": tr["s"],
                     "lo": pe.get("ci_low"), "hi": pe.get("ci_high"), "archived_outcome": d["result"]["outcome"], "archived_truth_m": tr["model_error_m"]})
    return recs


def verdict(lo, hi, eps):
    if lo is None or hi is None:
        return "undetermined"
    return decide(float(lo), float(hi), float(eps)).value


def sweep(recs, grid):
    rows = []
    for eps in grid:
        c = {"FP": 0, "FN": 0, "U": 0, "pos": 0, "neg": 0, "correct_C": 0, "correct_D": 0}
        fp_runs = []
        for r in recs:
            divergent_truth = r["truth_m"] > eps           # closed boundary: E <= eps is conformant
            v = verdict(r["lo"], r["hi"], eps)
            if divergent_truth:
                c["pos"] += 1; c["FN"] += v == "conformant"; c["correct_D"] += v == "divergent"
            else:
                c["neg"] += 1; c["FP"] += v == "divergent"; c["correct_C"] += v == "conformant"
                if v == "divergent":
                    fp_runs.append(r["run"])
            c["U"] += v == "undetermined"
        rows.append({"eps_mm": float(eps * 1000), "eps_exact": str(eps), "positives": c["pos"], "negatives": c["neg"], "FP": c["FP"], "FN": c["FN"], "undetermined": c["U"],
                     "FPR": c["FP"] / c["neg"] if c["neg"] else float("nan"), "FNR": c["FN"] / c["pos"] if c["pos"] else float("nan"), "undetermined_rate": c["U"] / len(recs),
                     "false_divergent_runs": ";".join(fp_runs)})
    return rows


grid = [Decimal(repr(float(e))) for e in np.logspace(math.log10(0.05e-3), math.log10(50e-3), 61)]
grid = sorted(set(grid + [Decimal("0.001")]))
summary = {}
for name, recs in (("F_id", frame_records()), ("S_si", scale_records())):
    rows = sweep(recs, grid)
    with open(os.path.join(args.out, f"{name}_threshold_sweep_exact.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n"); w.writeheader(); w.writerows(rows)
    at1 = next(r for r in rows if r["eps_exact"] == "0.001")
    at1_runs = [{"run": r["run"], "truth_mm": float(r["truth_m"] * 1000), "truth_exact": r["truth_exact"], "noise_mm": r["noise_mm"],
                 "interval_mm": [None if r["lo"] is None else r["lo"] * 1e3, None if r["hi"] is None else r["hi"] * 1e3],
                 "verdict_at_1mm": verdict(r["lo"], r["hi"], Decimal("0.001")), "archived_outcome": r["archived_outcome"],
                 "archived_float_truth_mm": r["archived_truth_m"] * 1e3}
                for r in recs if abs(r["truth_m"] - Decimal("0.001")) <= Decimal("1e-9")]
    # the same point scored with the archived float truth (analyze_v013 convention: positive iff float truth > eps)
    fp_float = sum(1 for r in recs if not (r["archived_truth_m"] > 0.001) and verdict(r["lo"], r["hi"], Decimal("0.001")) == "divergent")
    summary[name] = {"runs": len(recs), "at_eps_1mm_exact_truth": {k: at1[k] for k in ("positives", "negatives", "FP", "FN", "undetermined", "false_divergent_runs")},
                     "at_eps_1mm_float_truth_FP": fp_float, "runs_with_truth_exactly_1mm": at1_runs,
                     "verdicts_equal_archived_at_1mm": all(verdict(r["lo"], r["hi"], Decimal("0.001")) == r["archived_outcome"] for r in recs)}

# (2) mixture stress
rng = np.random.default_rng(20260909)
eps_list = [1e-6, 1e-5, 1e-4, 1e-3]
cnt = {e: {"conformant": 0, "divergent": 0, "undetermined": 0} for e in eps_list}
hits = 0
for _ in range(args.mixture_reps):
    Es = []
    for _ in range(10):
        x = -20e-6 if rng.random() < 0.95 else 380e-6
        Es.append(G.make_pose(np.eye(3), np.array([x, 0.0, 0.0]) + rng.normal(size=3) * 1e-6))
    sd = spatial_decision(Es, 0.1)
    hits += sd.ci_low_m <= 0.0 <= sd.ci_high_m
    for e in eps_list:
        cnt[e][decide(sd.ci_low_m, sd.ci_high_m, e).value] += 1
summary["mixture_stress"] = {"replicates": args.mixture_reps, "seed": 20260909, "true_error_m": 0.0, "coverage": hits / args.mixture_reps,
                             "decisions": {f"{e*1e3:g} mm": cnt[e] for e in eps_list},
                             "note": "truth is conformant at every tolerance (true error 0): every divergent verdict is a false divergent"}
json.dump(summary, open(os.path.join(args.out, "rescoring_summary.json"), "w"), indent=1, default=str)
print(json.dumps(summary, indent=1, default=str))
