#!/usr/bin/env python3
"""RC9, POST HOC (after the pre-registered WP6 runs; not a pre-registered analysis): the loss campaign's timeout intervals
re-derived offline with every `faulted` trial bounded by the time its FAULT was observed (tau_w <= t_state) instead of
gap + L.  Rationale: in the one excluding interval of the campaign (L16_016_fault250_iid5_03) the post-gap command of a
trial at gap 0.228 s was lost, the 0.25-s fault fired during the response wait, and with 30 of 30 calibration commands
answered the trial was bounded by gap + L = 0.242 s.  Both released rules use gap + L for a faulted trial unless the
calibration saw loss; t_state is sound whether or not the post-gap command arrived, at the cost of the response wait.
The replay uses liveness.timeout_interval_from_trials with baseline_loss forced to >= 1, which is exactly the
state-bounded rule for faulted trials (and makes an unconfirmed-under-0.1.5 rejection unattributable).  Reads
validation/v0.1.6/mock/L (read only); writes validation/v0.1.6/analysis/L_state_bounded_posthoc.json."""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.liveness import timeout_interval_from_trials  # noqa: E402

rows = []
for p in sorted(glob.glob(os.path.join(HERE, "v0.1.6", "mock", "L", "L16_*_fault250_*.json"))):
    d = json.load(open(p)); name = os.path.basename(p)[:-5]
    obs = d["result"]["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s") or {}
    if "latency_allowance_s" not in tau:
        continue
    rule = d["truth"]["rule"]
    n_sent = 30 if rule == "0.1.6" else 12
    lost = n_sent - int(R.get("latency_probes_responded") or 0)
    est = timeout_interval_from_trials(Lv["trials"], L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=float(R.get("feedback_period_s") or 0.01),
                                       hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"], baseline_loss=max(1, lost), rule=rule)
    lo, hi = est["interval_low_s"], est["interval_high_s"]
    ok = est["status"] == "formed" and lo > R["resolution_floor_s"] and lo <= hi
    rows.append({"run": name, "rule": rule, "loss": d["truth"]["loss"], "released": [tau.get("interval_low_s"), tau.get("interval_high_s"), tau.get("status")],
                 "state_bounded": [lo, hi, "ok" if ok else est["status"]], "contains": bool(ok and lo <= 0.25 <= hi),
                 "width_ms": (hi - lo) * 1e3 if ok else None})
summ = {}
for rule in ("0.1.5", "0.1.6"):
    for loss in ("none", "iid2", "iid5", "ge5"):
        rs = [r for r in rows if r["rule"] == rule and r["loss"] == loss]
        ok = [r for r in rs if r["state_bounded"][2] == "ok"]
        summ[f"{rule}/{loss}"] = {"n": len(rs), "formed": len(ok), "contains": sum(r["contains"] for r in ok), "excludes": sum(not r["contains"] for r in ok),
                                   "not_formed": len(rs) - len(ok), "median_width_ms": float(np.median([r["width_ms"] for r in ok])) if ok else None,
                                   "released_median_width_ms": float(np.median([(r["released"][1] - r["released"][0]) * 1e3 for r in rs if r["released"][2] == "ok"])) if any(r["released"][2] == "ok" for r in rs) else None}
json.dump({"note": "POST HOC; not pre-registered", "summary": summ, "rows": rows}, open(os.path.join(HERE, "v0.1.6", "analysis", "L_state_bounded_posthoc.json"), "w"), indent=1)
print(json.dumps(summ, indent=1))
