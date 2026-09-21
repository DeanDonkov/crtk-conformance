#!/usr/bin/env python3
"""Re-derive the archived v0.1.3 liveness intervals under the 0.1.5 rules, offline (no ROS, no new measurement).

Usage: python3 validation/reanalyze_liveness_v015.py [validation/v0.1.3/mock] [--out validation/v0.1.5/reanalysis]

Two 0.1.5 changes affect an archived interval (RC7 adversarial review, findings F1 and F8); a third affects the
stop sub-verdict of runs with baseline command loss (finding F10):

  F8  the onset-based drift lower bound carries the granularity term G (timeout_interval_from_trials);
  F1  the response latency r_i of the last streamed command is used only when it can be the response.  The archived
      trial records hold no raw gap samples, so the 0.1.5 departure guard cannot be replayed sample by sample; the
      archival proxy used here is: r_i is retained only if r_i >= (median response latency of the run's post-gap
      commands) - one feedback period, i.e. only if the last command could have been answered as fast as r_i says.  A
      value smaller than that was read from the still-executing in-band stream (L_delayed: 0.2-9.7 ms against
      ~310 ms responses) and is replaced by the latency allowance L, exactly as the 0.1.5 guard does when no
      departure is observed;
  (v)  the observability rules of 0.1.5 (a held trial counts as a passing trial for a drift policy only if its silence
      outlasted the reference window; a trial with the pose already at the post-gap goal is not observable) are checked
      against the archived records and reported in the summary line;
  F10 a run whose calibration probes (12 commands sent without a preceding silence) drew fewer than 12 responses has
      its rejection-class evidence marked as unattributable (confounded by baseline command loss); a fault seen in the
      operating state or a drift seen in the pose remains attributable (no archived run other than R_drop_50 has any
      loss, so this affects only R_drop_50).

The archive is read only.  Outputs: L_liveness_v015.csv and L_liveness_v015.md in --out.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from crtk_conformance.liveness import timeout_interval_from_trials  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("archive", nargs="?", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.3", "mock"))
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.5", "reanalysis"))
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

rows = []
for path in sorted(glob.glob(os.path.join(args.archive, "L_*.json"))):
    d = json.load(open(path)); name = os.path.basename(path)[:-5]
    obs = d["result"]["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s")
    fp = float(R.get("feedback_period_s") or 0.01)
    n_resp = int(R.get("latency_probes_responded") or 0)
    row = {"run": name, "tau_true_s": d["truth"].get("tau_w_s"), "mode": d["truth"].get("mode"), "expect": d["expectation"]["temporal"]["stop_behaviour"],
           "horizon_s": d["expectation"]["temporal"].get("horizon_s"), "stop_class": Lv.get("stop_class"),
           "calibration_responded": n_resp, "baseline_command_loss": 12 - n_resp,
           "v013_status": tau.get("status") if isinstance(tau, dict) and "status" in tau else ("upper_bound" if isinstance(tau, dict) and "upper_bound_s" in tau else "none"),
           "v013_low_s": tau.get("interval_low_s") if isinstance(tau, dict) else None, "v013_high_s": tau.get("interval_high_s") if isinstance(tau, dict) else None,
           "v013_contains": None, "v015_low_s": None, "v015_high_s": None, "v015_contains": None, "r_i_replaced_by_L": 0, "r_i_retained": 0,
           "low_change_ms": None, "v015_status": None, "note": ""}
    step = float(R.get("step_used_m") or 0.002)
    row["pose_at_goal_trials"] = sum(1 for t in Lv.get("trials", []) if t.get("initial_distance_m") is not None and t["initial_distance_m"] < 0.25 * step)
    row["held_without_drift_evaluation"] = (sum(1 for t in Lv.get("trials", []) if t["class"] == "held" and not (t["gap_s"] > (t.get("reference_window_s") or 0.0)))
                                            if Lv.get("stop_class") == "drifted" else 0)  # the rule applies to drift policies only
    row["held_without_drift_evaluation_max_gap_s"] = max([t["gap_s"] for t in Lv.get("trials", []) if t["class"] == "held" and not (t["gap_s"] > (t.get("reference_window_s") or 0.0))] or [0.0]) if Lv.get("stop_class") == "drifted" else 0.0
    if isinstance(tau, dict) and tau.get("status") == "ok":
        trials = Lv["trials"]
        resp = [t["time_to_respond_s"] for t in trials if t.get("responded") and isinstance(t.get("time_to_respond_s"), float) and not math.isnan(t["time_to_respond_s"])]
        med_resp = float(np.median(resp)) if resp else float("nan")
        floor = med_resp - fp if resp else -math.inf
        replay = []
        for t in trials:
            r = t.get("last_stream_latency_s")
            ok = isinstance(r, float) and not math.isnan(r)
            if ok and r < floor:
                t = dict(t, last_stream_latency_s=float("nan")); row["r_i_replaced_by_L"] += 1
            elif ok:
                row["r_i_retained"] += 1
            replay.append(t)
        est = timeout_interval_from_trials(replay, L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=fp, hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"],
                                           baseline_loss=12 - n_resp)
        tt = d["truth"].get("tau_w_s")
        row.update(v013_contains=(tau["interval_low_s"] <= tt <= tau["interval_high_s"]), v015_low_s=est["interval_low_s"], v015_high_s=est["interval_high_s"],
                   v015_contains=(est["interval_low_s"] <= tt <= est["interval_high_s"]), low_change_ms=(est["interval_low_s"] - tau["interval_low_s"]) * 1e3,
                   v015_status=("ok" if est["status"] == "formed" and est["interval_low_s"] > R["resolution_floor_s"] else est["status"]),
                   median_post_gap_response_ms=med_resp * 1e3, conditional=tau.get("conditional"))
        if row["r_i_replaced_by_L"]:
            row["note"] = f"r_i below the run's response floor ({med_resp*1e3:.1f} - {fp*1e3:.1f} ms) in {row['r_i_replaced_by_L']} trials: replaced by L = {tau['latency_allowance_s']*1e3:.1f} ms"
        if any(t["class"] == "drifted" for t in trials):
            row["note"] += ("; " if row["note"] else "") + "onset lower bound now includes G"
    if n_resp < 12 and Lv.get("stop_class") in ("rejected",):
        row["note"] += ("; " if row["note"] else "") + "rejection evidence confounded by baseline command loss (0.1.5: undetermined)"
    rows.append(row)

# the regression case with random drops
for name in ("R_drop_50", "R_delay_50ms_jitter_20ms", "R_noisy", "R_nolocal"):
    p = os.path.join(args.archive, name + ".json")
    if not os.path.exists(p):
        continue
    d = json.load(open(p)); te = next(x for x in d["results"] if x["binding_class"] == "temporal"); obs = te["observations"]
    n_resp = int(obs["resolution"].get("latency_probes_responded") or 0)
    Lv = obs["liveness"]
    rows.append({"run": name, "tau_true_s": None, "mode": "regression", "expect": d["expectation"]["temporal"]["stop_behaviour"], "horizon_s": d["expectation"]["temporal"].get("horizon_s"),
                 "stop_class": Lv.get("stop_class"), "calibration_responded": n_resp, "baseline_command_loss": 12 - n_resp,
                 "v013_status": (Lv.get("tau_w_estimate_s") or {}).get("status"), "v013_low_s": None, "v013_high_s": None, "v013_contains": None, "v015_low_s": None, "v015_high_s": None,
                 "v015_contains": None, "r_i_replaced_by_L": 0, "r_i_retained": 0, "low_change_ms": None,
                 "v015_status": ("undetermined (confounded)" if (n_resp < 12 and Lv.get("stop_class") == "rejected") else (Lv.get("tau_w_estimate_s") or {}).get("status")),
                 "note": ("0.1.3 stop sub-verdict %s; 0.1.5: undetermined -- %d of 12 calibration commands unanswered, no stop policy injected" % (te["estimates"]["sub_verdicts"].get("stop_behaviour"), 12 - n_resp))
                         if (n_resp < 12 and Lv.get("stop_class") == "rejected") else "unchanged"})

keys = ["run", "tau_true_s", "mode", "expect", "horizon_s", "stop_class", "calibration_responded", "baseline_command_loss", "v013_status", "v013_low_s", "v013_high_s", "v013_contains",
        "v015_low_s", "v015_high_s", "v015_contains", "low_change_ms", "r_i_retained", "r_i_replaced_by_L", "pose_at_goal_trials", "held_without_drift_evaluation", "v015_status", "note"]
with open(os.path.join(args.out, "L_liveness_v015.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore", lineterminator="\n"); w.writeheader(); w.writerows(rows)
formed = [r for r in rows if r["v013_status"] == "ok"]
widths013 = [(r["v013_high_s"] - r["v013_low_s"]) * 1e3 for r in formed]
widths015 = [(r["v015_high_s"] - r["v015_low_s"]) * 1e3 for r in formed]
md = ["# Archived v0.1.3 liveness campaign under the 0.1.5 rules (offline re-derivation; archive unchanged)", "",
      f"Runs: {len([r for r in rows if r['mode'] != 'regression'])}; intervals with status ok in 0.1.3: {len(formed)}; containing the injected timeout: 0.1.3 {sum(bool(r['v013_contains']) for r in formed)}/{len(formed)}, 0.1.5 {sum(bool(r['v015_contains']) for r in formed)}/{len(formed)}; "
      f"median width 0.1.3 {np.median(widths013):.1f} ms, 0.1.5 {np.median(widths015):.1f} ms; lower ends changed in {sum(1 for r in formed if abs(r['low_change_ms']) > 1e-6)} intervals; "
      f"r_i replaced by L in {sum(r['r_i_replaced_by_L'] for r in formed)} trials of {sum(1 for r in formed if r['r_i_replaced_by_L'])} runs; "
      f"trials with the pose already at the post-gap goal (not observable under 0.1.5): {sum(r.get('pose_at_goal_trials', 0) for r in rows)}; "
      f"held trials of drift-policy runs whose silence did not outlast the reference window (no passing bound under 0.1.5): {sum(r.get('held_without_drift_evaluation', 0) for r in rows)}, "
      f"all at the resolution floor (largest such gap {max(r.get('held_without_drift_evaluation_max_gap_s', 0.0) for r in rows)*1e3:.1f} ms, where the passing bound is negative).", "",
      "| run | tau_w (s) | class | 0.1.3 interval (ms) | 0.1.5 interval (ms) | lower-end change (ms) | contains (0.1.3 / 0.1.5) | note |", "|---|---|---|---|---|---|---|---|"]
for r in rows:
    if r["v013_status"] == "ok":
        md.append(f"| {r['run']} | {r['tau_true_s']} | {r['stop_class']} | [{r['v013_low_s']*1e3:.1f}, {r['v013_high_s']*1e3:.1f}] | [{r['v015_low_s']*1e3:.1f}, {r['v015_high_s']*1e3:.1f}] | {r['low_change_ms']:+.1f} | {r['v013_contains']} / {r['v015_contains']} | {r['note']} |")
    else:
        md.append(f"| {r['run']} | {r['tau_true_s']} | {r['stop_class']} | {r['v013_status']} | {r['v015_status']} | — | — | {r['note']} |")
open(os.path.join(args.out, "L_liveness_v015.md"), "w").write("\n".join(md) + "\n")
print("\n".join(md))
