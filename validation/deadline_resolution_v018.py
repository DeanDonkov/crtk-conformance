#!/usr/bin/env python3
"""Which fault deadlines can the sound (0.1.7) timeout bound decide?  (RC13 external review, point 5.)  No ROS, no runs.

For a formed interval [lo, hi] of tau_w, liveness.fault_horizon_decision decides a declared horizon H "fault within H"
satisfied only if hi <= H, and violated only if lo > H (the client's own margin, eq. (7), aside).  So H is decidable
satisfied from hi on and violated below lo; horizons in [lo, hi) are undetermined.  This script reports, per run, the
confirmation margin hi - tau_w and the refutation margin tau_w - lo, for the sound bound and for the conditional
(0.1.6-rule) estimate, from:
  * the 32 fault-policy runs of the confirmatory v0.1.7 campaign (validation/v0.1.7/mock/L, tau_w = 250 ms, 100-Hz
    feedback, four loss conditions);
  * the 23 archived v0.1.3 intervals re-derived under 0.1.7 (validation/v0.1.7/L_archived_v017.csv; tau_w from 50 ms
    to 1 s, a 20-Hz feedback case and a 300-ms application-delay case).
It then checks, by replaying liveness.fault_horizon_decision, that every confirmatory interval decides H = tau_w +
margin satisfied exactly when the margin is reached.

Usage: python3 validation/deadline_resolution_v018.py [--out validation/v0.1.8/studies]
"""
import argparse
import csv
import glob
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.liveness import fault_horizon_decision  # noqa: E402

NEED = 1.0 / 100.0 + 0.005


def confirmatory():
    rows = []
    for p in sorted(glob.glob(os.path.join(HERE, "v0.1.7", "mock", "L", "L17_017_fault250_*.json"))):
        d = json.load(open(p))
        L = d["result"]["observations"]["liveness"]; tau = L.get("tau_w_estimate_s") or {}
        ce = tau.get("conditional_estimate_s") or {}
        tw = d["truth"]["tau_w_s"]
        if tau.get("status") != "ok":
            continue
        lo, hi = tau["interval_low_s"], tau["interval_high_s"]
        # replay: the decision just below and at hi
        at = fault_horizon_decision(tau, hi, NEED)[0]
        below = fault_horizon_decision(tau, hi - 1e-3, NEED)[0]
        row = {"source": "confirmatory v0.1.7", "run": os.path.basename(p)[:-5], "tau_w_s": tw, "loss": d["truth"]["loss"], "feedback_hz": 100,
               "sound_confirm_margin_s": hi - tw, "sound_refute_margin_s": tw - lo, "replay_at_hi": at, "replay_below_hi": below,
               "resp_wait_s": L.get("response_timeout_s")}
        if ce.get("status") == "formed":
            row["conditional_confirm_margin_s"] = ce["interval_high_s"] - tw
            row["conditional_refute_margin_s"] = tw - ce["interval_low_s"]
        rows.append(row)
    return rows


def archived():
    rows = []
    for r in csv.DictReader(open(os.path.join(HERE, "v0.1.7", "L_archived_v017.csv"))):
        if r["v017_status"] != "ok":
            continue
        tw = float(r["tau_true_s"])
        row = {"source": "archived v0.1.3", "run": r["run"], "mode": r["mode"], "tau_w_s": tw, "resp_wait_s": float(r["response_wait_s"]),
               "sound_confirm_margin_s": float(r["v017_high_s"]) - tw, "sound_refute_margin_s": tw - float(r["v017_low_s"])}
        if r["v016_status"] == "ok":
            row["conditional_confirm_margin_s"] = float(r["v016_high_s"]) - tw
            row["conditional_refute_margin_s"] = tw - float(r["v016_low_s"])
        rows.append(row)
    return rows


def med(v):
    v = [x for x in v if x is not None]
    return statistics.median(v) if v else None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "studies"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    c, h = confirmatory(), archived()
    summ = {"confirmatory": {}, "archived_fault": {}, "archived_drift": {}}
    for lo in ("none", "iid2", "iid5", "ge5"):
        rs = [r for r in c if r["loss"] == lo]
        summ["confirmatory"][lo] = {"n": len(rs), "sound_confirm_margin_median_ms": 1e3 * med([r["sound_confirm_margin_s"] for r in rs]),
                                    "conditional_confirm_margin_median_ms": 1e3 * med([r.get("conditional_confirm_margin_s") for r in rs]),
                                    "sound_refute_margin_median_ms": 1e3 * med([r["sound_refute_margin_s"] for r in rs])}
    for mode in ("fault", "drift"):
        for r in [x for x in h if x["mode"] == mode]:
            key = f"{r['tau_w_s']*1e3:g} ms"
            summ[f"archived_{mode}"].setdefault(key, []).append({"run": r["run"], "sound_confirm_ms": 1e3 * r["sound_confirm_margin_s"],
                                                                 "conditional_confirm_ms": None if r.get("conditional_confirm_margin_s") is None else 1e3 * r["conditional_confirm_margin_s"],
                                                                 "refute_ms": 1e3 * r["sound_refute_margin_s"], "resp_wait_s": r["resp_wait_s"]})
    summ["replay_consistent"] = all(r["replay_at_hi"] == "satisfied" and r["replay_below_hi"] != "satisfied" for r in c)
    json.dump({"summary": summ, "confirmatory_rows": c, "archived_rows": h}, open(os.path.join(a.out, "deadline_resolution.json"), "w"), indent=1)
    print(json.dumps(summ["confirmatory"], indent=1)); print("replay consistent:", summ["replay_consistent"])
    for mode in ("fault", "drift"):
        for k, v in summ[f"archived_{mode}"].items():
            print(mode, k, [(x["run"], round(x["sound_confirm_ms"]), None if x["conditional_confirm_ms"] is None else round(x["conditional_confirm_ms"]), round(x["refute_ms"])) for x in v])
