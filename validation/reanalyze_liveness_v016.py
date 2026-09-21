#!/usr/bin/env python3
"""RC9 (external review, point 4): the archived v0.1.3 liveness campaign re-derived offline under the 0.1.6
confirmation rule, next to the 0.1.5 rule, and every formed interval's width relative to the injected timeout.
No ROS, no new measurement; the archive is read only.

The replay is the one of reanalyze_liveness_v015.py (same archival proxy for the departure guard of r_i); the interval
is then formed twice with liveness.timeout_interval_from_trials, rule="0.1.5" and rule="0.1.6".  Under 0.1.6 a
`rejected` trial (no response, no state change) is attributable only when its non-response was confirmed by repeats
at the same gap; the archived records carry no repeats, so every archived rejection is unattributable under 0.1.6.
`faulted` (seen in the operating state) and `drifted` (seen in the pose) trials are unaffected.  The 0.1.5 intervals
are checked against validation/v0.1.5/reanalysis/L_liveness_v015.csv (must agree to 1e-12 s).

Usage: python3 validation/reanalyze_liveness_v016.py [validation/v0.1.3/mock] [--out validation/v0.1.6/reanalysis]
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.liveness import timeout_interval_from_trials, loss_upper_bound  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("archive", nargs="?", default=os.path.join(HERE, "v0.1.3", "mock"))
ap.add_argument("--v015", default=os.path.join(HERE, "v0.1.5", "reanalysis", "L_liveness_v015.csv"))
ap.add_argument("--out", default=os.path.join(HERE, "v0.1.6", "reanalysis"))
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)
v015 = {r["run"]: r for r in csv.DictReader(open(args.v015))}

rows = []
for path in sorted(glob.glob(os.path.join(args.archive, "L_*.json"))):
    d = json.load(open(path)); name = os.path.basename(path)[:-5]
    obs = d["result"]["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s")
    fp = float(R.get("feedback_period_s") or 0.01)
    n_resp = int(R.get("latency_probes_responded") or 0)
    tt = d["truth"].get("tau_w_s")
    row = {"run": name, "tau_true_s": tt, "mode": d["truth"].get("mode"), "stop_class": Lv.get("stop_class"), "calibration_responded": n_resp,
           "loss_upper_bound_95": loss_upper_bound(12, 12 - n_resp), "trial_classes": ";".join(sorted({t["class"] for t in Lv.get("trials", [])})),
           "rejected_trials": sum(1 for t in Lv.get("trials", []) if t["class"] == "rejected")}
    if isinstance(tau, dict) and tau.get("status") == "ok":
        trials = Lv["trials"]
        resp = [t["time_to_respond_s"] for t in trials if t.get("responded") and isinstance(t.get("time_to_respond_s"), float) and not math.isnan(t["time_to_respond_s"])]
        med_resp = float(np.median(resp)) if resp else float("nan")
        floor = med_resp - fp if resp else -math.inf
        replay = []
        for t in trials:
            r = t.get("last_stream_latency_s")
            if isinstance(r, float) and not math.isnan(r) and r < floor:
                t = dict(t, last_stream_latency_s=float("nan"))
            replay.append(t)
        for rule in ("0.1.5", "0.1.6"):
            est = timeout_interval_from_trials(replay, L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=fp, hold_tol=Lv["hold_tolerance_m"],
                                               stop_class=Lv["stop_class"], baseline_loss=12 - n_resp, rule=rule)
            k = rule.replace(".", "")
            ok = est["status"] == "formed" and est["interval_low_s"] > R["resolution_floor_s"]
            row[f"v{k}_status"] = "ok" if ok else est["status"]
            row[f"v{k}_low_s"], row[f"v{k}_high_s"] = est["interval_low_s"], est["interval_high_s"]
            row[f"v{k}_contains"] = est["interval_low_s"] <= tt <= est["interval_high_s"]
            row[f"v{k}_width_ms"] = (est["interval_high_s"] - est["interval_low_s"]) * 1e3
            row[f"v{k}_width_over_tau"] = (est["interval_high_s"] - est["interval_low_s"]) / tt
            row[f"v{k}_unattributable_trials"] = est["unattributable_trials"]
        a = v015[name]
        assert abs(float(a["v015_low_s"]) - row["v015_low_s"]) < 1e-12 and abs(float(a["v015_high_s"]) - row["v015_high_s"]) < 1e-12, name
        row["changed_by_016"] = (row["v015_low_s"], row["v015_high_s"], row["v015_status"]) != (row["v016_low_s"], row["v016_high_s"], row["v016_status"])
    else:
        row["v015_status"] = v015[name]["v015_status"] or v015[name]["v013_status"]
        row["v016_status"] = row["v015_status"]
        row["changed_by_016"] = False
    rows.append(row)

# regression runs: R_drop_50 is the only archived run with calibration loss and a rejection-class stop
for name in ("R_drop_50",):
    d = json.load(open(os.path.join(args.archive, name + ".json"))); te = next(x for x in d["results"] if x["binding_class"] == "temporal"); obs = te["observations"]
    n_resp = int(obs["resolution"].get("latency_probes_responded") or 0)
    Lv = obs["liveness"]
    rows.append({"run": name, "mode": "regression", "stop_class": Lv.get("stop_class"), "calibration_responded": n_resp, "loss_upper_bound_95": loss_upper_bound(12, 12 - n_resp),
                 "trial_classes": ";".join(sorted({t["class"] for t in Lv.get("trials", [])})), "rejected_trials": sum(1 for t in Lv.get("trials", []) if t["class"] == "rejected"),
                 "v015_status": v015[name]["v015_status"], "v016_status": "undetermined (unconfirmed rejections: no repeats in the archive)", "changed_by_016": False})

keys = ["run", "tau_true_s", "mode", "stop_class", "trial_classes", "rejected_trials", "calibration_responded", "loss_upper_bound_95",
        "v015_status", "v015_low_s", "v015_high_s", "v015_contains", "v015_width_ms", "v015_width_over_tau",
        "v016_status", "v016_low_s", "v016_high_s", "v016_contains", "v016_width_ms", "v016_width_over_tau", "v016_unattributable_trials", "changed_by_016"]
with open(os.path.join(args.out, "L_liveness_v016.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore", lineterminator="\n"); w.writeheader(); w.writerows(rows)
formed = [r for r in rows if r.get("v015_status") == "ok"]
wr = np.array([r["v015_width_over_tau"] for r in formed]); wm = np.array([r["v015_width_ms"] for r in formed])
fault = [r for r in formed if r["mode"] == "fault"]; drift = [r for r in formed if r["mode"] == "release"]
summ = {"archived_runs": len([r for r in rows if r["mode"] != "regression"]), "formed_v015": len(formed), "formed_v016": sum(1 for r in rows if r.get("v016_status") == "ok"),
        "contain_v015": sum(bool(r["v015_contains"]) for r in formed), "contain_v016": sum(bool(r.get("v016_contains")) for r in formed if r.get("v016_status") == "ok"),
        "changed_by_016": [r["run"] for r in rows if r.get("changed_by_016")],
        "archived_runs_with_rejected_trials": [r["run"] for r in rows if r.get("rejected_trials")],
        "width_ms": {"median": float(np.median(wm)), "min": float(wm.min()), "max": float(wm.max())},
        "width_over_tau": {"median": float(np.median(wr)), "min": float(wr.min()), "max": float(wr.max()),
                           "fault_median": float(np.median([r["v015_width_over_tau"] for r in fault])), "drift_median": float(np.median([r["v015_width_over_tau"] for r in drift]))},
        "width_over_tau_by_tau": {str(t): [round(r["v015_width_over_tau"], 4) for r in formed if r["tau_true_s"] == t] for t in sorted({r["tau_true_s"] for r in formed})},
        "calibration_loss_bound_12_of_12": loss_upper_bound(12, 0), "calibration_loss_bound_30_of_30": loss_upper_bound(30, 0)}
json.dump(summ, open(os.path.join(args.out, "L_liveness_v016_summary.json"), "w"), indent=1)
print(json.dumps(summ, indent=1))
