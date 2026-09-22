#!/usr/bin/env python3
"""Analysis of the confirmatory v0.1.7 campaign (validation/v0.1.7/PREREGISTRATION.md, section 4).  Written and
committed with the pre-registration, before any run.  Reads validation/v0.1.7/mock/{K,L}; writes
validation/v0.1.7/analysis/confirmatory_v017.json and prints every pre-registered prediction with its outcome."""
import glob
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.liveness import fault_horizon_decision  # noqa: E402

V17 = os.path.join(HERE, "v0.1.7")
NEED = 1.0 / 100.0 + 0.005
preds = []


def pred(pid, text, ok, detail=""):
    preds.append({"id": pid, "prediction": text, "matched": bool(ok), "observed": detail})
    print(("MATCH    " if ok else "DEVIATES ") + f"{pid}: {text} -- {detail}")


# ---------------------------------------------------------------- K
K = {os.path.basename(p)[:-5]: json.load(open(p)) for p in sorted(glob.glob(os.path.join(V17, "mock", "K", "K17_*.json")))}
k1 = {n: r for n, r in K.items() if "_K1_" in n}
ctrl = {n: r for n, r in K.items() if "_ctrl_" in n}


def kview(r):
    sc = r["scale"]["result"]; fr = r["frame"]["result"]
    cc = sc["estimates"].get("command_feedback_consistency") or {}
    return {"frame": fr["outcome"], "unit": sc["outcome"], "unit_ungated": sc["estimates"].get("unit_outcome_without_consistency_gate"),
            "flag": cc.get("flag_non_shared_binding_or_tracking_deficit"), "residual_mm": (cc.get("goal_residual_if") or {}).get("mean", float("nan")) * 1e3,
            "command_semantic": r["verdict_kinds"]["spatial_command_semantic"], "dimensional_summary": r["report_summary"]["dimensional"],
            "assumption": r["verdict_kinds"]["shared_binding_assumption"]}


kv = {n: kview(r) for n, r in K.items()}
pred("K-1", "K1: consistency flag raised in 3/3", len(k1) == 3 and all(kv[n]["flag"] for n in k1), str({n: kv[n]["flag"] for n in k1}))
pred("K-2", "K1: unit verdict undetermined (withheld) in 3/3, ungated outcome divergent", len(k1) == 3 and all(kv[n]["unit"] == "undetermined" and kv[n]["unit_ungated"] == "divergent" for n in k1),
     str({n: (kv[n]["unit"], kv[n]["unit_ungated"]) for n in k1}))
pred("K-3", "K1: frame (feedback-binding) verdict conformant in 3/3", len(k1) == 3 and all(kv[n]["frame"] == "conformant" for n in k1), str({n: kv[n]["frame"] for n in k1}))
pred("K-4", "K1: report command-semantic reading undetermined, dimensional summary undetermined, assumption contradicted, in 3/3",
     len(k1) == 3 and all(kv[n]["command_semantic"] == "undetermined" and kv[n]["dimensional_summary"] == "undetermined" and kv[n]["assumption"].startswith("contradicted") for n in k1),
     str({n: (kv[n]["command_semantic"], kv[n]["dimensional_summary"]) for n in k1}))
pred("K-5", "controls: no flag; frame, unit and command-semantic reading conformant, in 6/6",
     len(ctrl) == 6 and all(not kv[n]["flag"] and kv[n]["frame"] == kv[n]["unit"] == kv[n]["command_semantic"] == "conformant" for n in ctrl),
     str({n: (kv[n]["flag"], kv[n]["frame"], kv[n]["unit"], kv[n]["command_semantic"]) for n in ctrl}))

# ---------------------------------------------------------------- L
rows = []
for p in sorted(glob.glob(os.path.join(V17, "mock", "L", "L17_*.json"))):
    d = json.load(open(p)); name = os.path.basename(p)[:-5]
    res = d["result"]; Lv = (res["observations"].get("liveness") or {}); tau = Lv.get("tau_w_estimate_s") or {}
    ce = tau.get("conditional_estimate_s") or {}
    tt = d["truth"]["tau_w_s"]
    row = {"run": name, "policy": d["truth"]["mode"], "loss": d["truth"]["loss"], "stop_class": Lv.get("stop_class"), "status": tau.get("status"),
           "low_s": tau.get("interval_low_s"), "high_s": tau.get("interval_high_s"), "sub_verdict": (res["estimates"].get("sub_verdicts") or {}).get("stop_behaviour"),
           "cond_low_s": ce.get("interval_low_s"), "cond_high_s": ce.get("interval_high_s"), "cond_status": ce.get("status"),
           "calibration_lost": (Lv.get("baseline_command_loss") or {}).get("lost"), "wall_s": d.get("wall_s")}
    if tt is not None and tau.get("status") == "ok":
        row["contains"] = tau["interval_low_s"] <= tt <= tau["interval_high_s"]
        row["width_ms"] = (tau["interval_high_s"] - tau["interval_low_s"]) * 1e3
        row["decision_at_0245"] = fault_horizon_decision(tau, 0.245, NEED)[0]
        if ce.get("status") == "formed":
            row["cond_contains"] = ce["interval_low_s"] <= tt <= ce["interval_high_s"]
            row["cond_decision_at_0245"] = fault_horizon_decision(dict(ce, status="ok"), 0.245, NEED)[0]
    rows.append(row)
hold = [r for r in rows if r["policy"] == "hold"]
fault = [r for r in rows if r["policy"] == "fault"]
formed = [r for r in fault if r["status"] == "ok"]
pred("L-1", "hold: 32 runs, held through the range in 32/32 (no false trip class)", len(hold) == 32 and all(r["stop_class"] == "held_through_range" for r in hold),
     str(sorted({r["stop_class"] for r in hold})))
pred("L-2", "hold: stop sub-verdict satisfied in 32/32", len(hold) == 32 and all(r["sub_verdict"] == "satisfied" for r in hold), str(sorted({str(r["sub_verdict"]) for r in hold})))
pred("L-3", "fault: 32 runs; no formed interval excludes 250 ms", len(fault) == 32 and all(r.get("contains") for r in formed),
     f"{sum(bool(r.get('contains')) for r in formed)}/{len(formed)} formed contain; statuses {sorted({str(r['status']) for r in fault})}")
pred("L-4", "fault: no contradictory (inconsistent) interval", all(r["status"] != "inconsistent" for r in fault), str([r["run"] for r in fault if r["status"] == "inconsistent"]))
pred("L-5", "fault: stop sub-verdict satisfied in every run with a formed interval; violated in none",
     all(r["sub_verdict"] == "satisfied" for r in formed) and not any(r["sub_verdict"] == "violated" for r in fault), str(sorted({str(r["sub_verdict"]) for r in fault})))
wn = [r["width_ms"] for r in formed if r["loss"] == "none"]
pred("L-6", "fault: median width without loss between 300 and 400 ms", bool(wn) and 300 <= statistics.median(wn) <= 400, f"{statistics.median(wn):.1f} ms (n={len(wn)})" if wn else "none formed")
pred("L-7", "fault: a conditional (0.1.6) estimate is reported with every formed interval", all(r["cond_status"] is not None for r in formed), str(sum(r["cond_status"] is not None for r in formed)))
pred("L-8", "secondary: at a declared horizon of 245 ms (false for 250 ms) no formed interval decides satisfied", not any(r.get("decision_at_0245") == "satisfied" for r in formed),
     str([r["run"] for r in formed if r.get("decision_at_0245") == "satisfied"]))

cells = {}
for r in rows:
    c = cells.setdefault(f"{r['policy']}/{r['loss']}", {"n": 0, "held": 0, "contains": 0, "excludes": 0, "contradictory": 0, "not_formed": 0, "satisfied": 0})
    c["n"] += 1
    if r["policy"] == "hold":
        c["held"] += r["stop_class"] == "held_through_range"
    elif r["status"] == "ok":
        c["contains" if r.get("contains") else "excludes"] += 1
    elif r["status"] == "inconsistent":
        c["contradictory"] += 1
    else:
        c["not_formed"] += 1
    c["satisfied"] += r["sub_verdict"] == "satisfied"
summary = {"predictions": preds, "all_matched": all(p["matched"] for p in preds), "cells": cells,
           "fault_width_ms_median_by_loss": {lo: statistics.median([r["width_ms"] for r in formed if r["loss"] == lo]) for lo in ("none", "iid2", "iid5", "ge5") if any(r["loss"] == lo for r in formed)},
           "conditional_estimates_excluding": [r["run"] for r in formed if r.get("cond_contains") is False],
           "conditional_estimates_deciding_0245_satisfied": [r["run"] for r in formed if r.get("cond_decision_at_0245") == "satisfied"],
           "calibration_saw_loss": sum(1 for r in rows if (r["calibration_lost"] or 0) > 0), "K": kv, "L_rows": rows}
os.makedirs(os.path.join(V17, "analysis"), exist_ok=True)
json.dump(summary, open(os.path.join(V17, "analysis", "confirmatory_v017.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in summary.items() if k not in ("K", "L_rows", "predictions")}, indent=1))
