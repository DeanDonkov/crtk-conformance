#!/usr/bin/env python3
"""EXPLORATORY (found by the final test run of RC14; not pre-registered): does the joint-space fit depend on the SciPy
version?  No ROS, no new run.

dimensional.poe_fit_trial calls scipy.optimize.least_squares(method="lm", max_nfev=200).  In SciPy 1.10 (the campaign
image) max_nfev for "lm" also counts the residual evaluations of the finite-difference Jacobian, so with 33 parameters
the fit stops after about six iterations (status 0); in newer SciPy (1.17 here) it counts residual evaluations only and
the fit converges (status > 0).  This script, with the released functions unchanged:
  1. reports the SciPy version and the budget semantics it observes;
  2. refits every trial of campaigns D and S from its recorded (q, T) and re-decides every run
     (geometry_anchor_poe_decision, the run's own tolerance and declaration), and compares with the archived outcome;
  3. repeats the coupling and yaw-reach levels of anchor_poe_study_v018.py (same seed, 100 replicates per level).
Run it in both environments and compare the outputs (validation/v0.1.8/analysis/poe_scipy_budget_<tag>.json).

Usage: python3 validation/poe_scipy_budget_v018.py <tag> [--study-n 100]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
import scipy  # noqa: E402
import scipy.optimize as so  # noqa: E402
from crtk_conformance.dimensional import geometry_anchor_poe_decision, poe_fit_trial  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("tag")
ap.add_argument("--study-n", type=int, default=100)
ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "analysis"))
a = ap.parse_args()

status = []
_orig = so.least_squares


def _wrap(*args, **kw):
    r = _orig(*args, **kw)
    status.append((int(r.nfev), int(r.status)))
    return r


so.least_squares = _wrap
out = {"tag": a.tag, "scipy": scipy.__version__, "numpy": np.__version__, "exploratory": True}

# 2. campaign trials
V18 = os.path.join(HERE, "v0.1.8")
runs = []
for p in sorted(glob.glob(os.path.join(V18, "dvrk_sim", "D", "runs", "*", "U_*.json")) + glob.glob(os.path.join(V18, "src_live", "live-src-*", "launch*", "*_geometry_*.json"))):
    if p.endswith(".bring_up.json"):
        continue
    d = json.load(open(p))
    tol_m = d.get("tolerance", {}).get("epsilon_m")
    for r in d.get("probes") or []:
        if r.get("probe") != "GeometryAnchorProbe":
            continue
        ob = r["observations"]; e = ob["expectation"]; g0 = r["estimates"]["geometry_anchor"]
        n0 = len(status)
        fits = [poe_fit_trial(e.get("joint_types", "RRPRRR"), t["q_ref_measured"], np.array(t["T_ref_measured"]), [np.array(q) for q in t["q_steps_measured"]],
                              [np.array(T) for T in t["T_steps_measured"]], ob["pitch_joint_index"], ob["yaw_joint_index"]) for t in ob["trials"]]
        st = status[n0:]
        tol = Tolerance(epsilon_m=float(tol_m if tol_m else (0.005 if p.endswith("5mm.json") else 0.001)), workspace_radius_m=0.10)
        g = geometry_anchor_poe_decision(fits, e["L_m"], e["u_rel"], e.get("expected_unit_m"), tol)
        runs.append({"run": os.path.relpath(p, V18), "archived_outcome": r["outcome"], "refit_outcome": g.outcome, "archived_gates": g0["gates_passed"],
                     "refit_gates": g.gates_passed, "archived_lambda": g0["lambda_hat_m"], "refit_lambda": g.lambda_hat_m,
                     "fit_status": [s for _, s in st], "fit_nfev": [n for n, _ in st]})
out["campaign_refits"] = runs
out["campaign_outcomes_equal"] = all(x["archived_outcome"] == x["refit_outcome"] and x["archived_gates"] == x["refit_gates"] for x in runs)
out["campaign_fits_stopped_at_budget"] = sum(s == 0 for x in runs for s in x["fit_status"])
out["campaign_fits"] = sum(len(x["fit_status"]) for x in runs)
print(json.dumps({k: out[k] for k in ("tag", "scipy", "campaign_outcomes_equal", "campaign_fits_stopped_at_budget", "campaign_fits")}))
for x in runs:
    if x["archived_outcome"] != x["refit_outcome"] or x["archived_gates"] != x["refit_gates"]:
        print("DIFFERS", x["run"], x["archived_outcome"], "->", x["refit_outcome"], x["archived_gates"], "->", x["refit_gates"])

# 3. study levels
import anchor_poe_study_v018 as S  # noqa: E402
S.FACTORS = {"coupling": [0.02, 0.05, 0.1], "reach": [0.44]}
n0 = len(status)
rows = S.run(a.study_n, 79000)
out["study_rows"] = rows
out["study_fits_stopped_at_budget"] = sum(s == 0 for _, s in status[n0:])
out["study_fits"] = len(status) - n0
json.dump(out, open(os.path.join(a.out, f"poe_scipy_budget_{a.tag}.json"), "w"), indent=1)
print(json.dumps({"study_fits_stopped_at_budget": out["study_fits_stopped_at_budget"], "study_fits": out["study_fits"]}))
