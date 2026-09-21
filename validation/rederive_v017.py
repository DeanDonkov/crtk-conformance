#!/usr/bin/env python3
"""Offline re-derivation of the archived campaigns under the 0.1.7 decision rules (RC12 review, points 1 and 2).
POST HOC: the rules were changed after the pre-registered v0.1.6 campaigns, which are reported unchanged under their
own rules.  No run is repeated; every input is an archived probe report.  Outputs in validation/v0.1.7/.

1. Consistency gate (dimensional.consistency_gate, report.verdict_kinds): every archived run that carries the scale
   probe's command/feedback consistency diagnostic (K, B scale, dVRK-sim) -- unit verdict and command-semantic frame
   reading under 0.1.7 against the archived 0.1.6 outcome.
2. Sound fault bound (liveness.trip_upper_bound, rule "0.1.7"): the 64 fault-policy runs of the loss experiment and the
   archived v0.1.3 liveness runs (their faulted trials carry no observation time; they are bounded by gap + the run's
   response wait + the 0.2 s state query + G, liveness.fault_observation_window).  Stop sub-verdicts for fault
   expectations are re-decided with liveness.fault_horizon_decision on the 0.1.7 interval.
"""
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.dimensional import consistency_gate  # noqa: E402
from crtk_conformance.liveness import fault_horizon_decision, fault_observation_window, timeout_interval_from_trials  # noqa: E402
from crtk_conformance.probes.base import Outcome  # noqa: E402

V16 = os.path.join(HERE, "v0.1.6"); V13 = os.path.join(HERE, "v0.1.3", "mock"); OUT = os.path.join(HERE, "v0.1.7")
os.makedirs(OUT, exist_ok=True)
NEED = 1.0 / 100.0 + 0.005  # client 100 Hz + J_max 5 ms (every campaign)


def cons_of(res):
    c = (res.get("estimates") or {}).get("command_feedback_consistency")
    return c.get("flag_non_shared_binding_or_tracking_deficit") if isinstance(c, dict) else None


# ---------------------------------------------------------------- 1. consistency gate
rows = []
for p in sorted(glob.glob(os.path.join(V16, "mock", "K", "*.json"))):
    d = json.load(open(p)); sc = d["scale"]["result"]; fr = d["frame"]["result"]
    flag = cons_of(sc)
    u7, withheld = consistency_gate(Outcome(sc["outcome"]), bool(flag))
    rows.append({"set": "K", "run": os.path.basename(p)[:-5], "flag": flag, "frame_016": fr["outcome"], "unit_016": sc["outcome"], "unit_017": u7.value,
                 "frame_command_semantic_017": "undetermined" if flag else fr["outcome"], "withheld": withheld})
for p in sorted(glob.glob(os.path.join(V16, "mock", "B", "BS_*.json"))):
    d = json.load(open(p)); sc = d["result"]; flag = cons_of(sc)
    u7, withheld = consistency_gate(Outcome(sc["outcome"]), bool(flag))
    rows.append({"set": "B-scale", "run": os.path.basename(p)[:-5], "flag": flag, "frame_016": "", "unit_016": sc["outcome"], "unit_017": u7.value,
                 "frame_command_semantic_017": "", "withheld": withheld})
for p in sorted(glob.glob(os.path.join(V16, "dvrk_sim", "runs", "*", "*.json"))):
    d = json.load(open(p))
    for res in d.get("probes", []) if isinstance(d.get("probes"), list) else []:
        if res.get("probe") == "scale_units" or "command_feedback_consistency" in (res.get("estimates") or {}):
            flag = cons_of(res)
            if flag is None:
                continue
            u7, withheld = consistency_gate(Outcome(res["outcome"]), bool(flag))
            rows.append({"set": "dVRK-sim", "run": os.path.relpath(p, V16), "flag": flag, "frame_016": "", "unit_016": res["outcome"], "unit_017": u7.value,
                         "frame_command_semantic_017": "", "withheld": withheld})
with open(os.path.join(OUT, "consistency_gate_v017.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n"); w.writeheader(); w.writerows(rows)
gate = {"runs_with_diagnostic": len(rows), "by_set": {s: sum(1 for r in rows if r["set"] == s) for s in sorted({r["set"] for r in rows})},
        "flagged": [r["run"] for r in rows if r["flag"]], "changed": [(r["run"], r["unit_016"], r["unit_017"]) for r in rows if r["unit_016"] != r["unit_017"]]}

# ---------------------------------------------------------------- 2a. loss experiment, fault policy (64 runs)
loss = []
for p in sorted(glob.glob(os.path.join(V16, "mock", "L", "L16_*_fault250_*.json"))):
    d = json.load(open(p)); name = os.path.basename(p)[:-5]
    res = d["result"]; obs = res["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s") or {}
    row = {"run": name, "rule_run": d["truth"]["rule"], "loss": d["truth"]["loss"], "tau_true_s": d["truth"]["tau_w_s"],
           "released_status": tau.get("status"), "released_low_s": tau.get("interval_low_s"), "released_high_s": tau.get("interval_high_s"),
           "sub_verdict_016": (res["estimates"].get("sub_verdicts") or {}).get("stop_behaviour")}
    if "latency_allowance_s" in tau:
        lost = int((Lv.get("baseline_command_loss") or {}).get("lost", 0))
        fw = fault_observation_window(Lv["response_timeout_s"], tau["granularity_allowance_s"])
        iv = timeout_interval_from_trials(Lv["trials"], L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=float(R.get("feedback_period_s") or 0.01),
                                          hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"], baseline_loss=lost, rule="0.1.7", fault_window_s=fw)
        ok = iv["status"] == "formed" and iv["interval_low_s"] > R["resolution_floor_s"] and len(tau.get("brackets_s") or []) >= 3
        st = "ok" if ok else ("inconsistent" if iv["status"] == "inconsistent" else "undetermined")
        t7 = {"status": st, "interval_low_s": iv["interval_low_s"], "interval_high_s": iv["interval_high_s"]}
        row.update(v017_status=st, v017_low_s=iv["interval_low_s"], v017_high_s=iv["interval_high_s"],
                   v017_contains=bool(ok and iv["interval_low_s"] <= row["tau_true_s"] <= iv["interval_high_s"]),
                   v017_width_ms=(iv["interval_high_s"] - iv["interval_low_s"]) * 1e3 if ok else None,
                   sub_verdict_017=fault_horizon_decision(t7, float(res["estimates"].get("stop_horizon_s") or 2.0), NEED)[0],
                   sub_verdict_017_at_0245=fault_horizon_decision(t7, 0.245, NEED)[0],
                   sub_verdict_016_at_0245=fault_horizon_decision(tau, 0.245, NEED)[0])
    else:
        row.update(v017_status="not formed (as released)", sub_verdict_017=row["sub_verdict_016"])
    loss.append(row)
ok7 = [r for r in loss if r.get("v017_status") == "ok"]
ok6 = [r for r in loss if r["released_status"] == "ok"]
lsum = {"fault_runs": len(loss), "v017_formed": len(ok7), "v017_contain": sum(r["v017_contains"] for r in ok7),
        "v016_formed": len(ok6), "v016_contain": sum(1 for r in ok6 if r["released_low_s"] <= r["tau_true_s"] <= r["released_high_s"]),
        "v017_median_width_ms_no_loss": float(np.median([r["v017_width_ms"] for r in ok7 if r["loss"] == "none"])),
        "v016_median_width_ms_no_loss": float(np.median([(r["released_high_s"] - r["released_low_s"]) * 1e3 for r in ok6 if r["loss"] == "none"])),
        "sub_verdict_changes": [(r["run"], r["sub_verdict_016"], r["sub_verdict_017"]) for r in loss if r["sub_verdict_016"] != r["sub_verdict_017"]],
        "horizon_0245_016_satisfied_while_false": [r["run"] for r in loss if r.get("sub_verdict_016_at_0245") == "satisfied"],
        "horizon_0245_017_satisfied": [r["run"] for r in loss if r.get("sub_verdict_017_at_0245") == "satisfied"]}

# ---------------------------------------------------------------- 2b. archived v0.1.3 liveness runs
arch = []
for path in sorted(glob.glob(os.path.join(V13, "L_*.json"))):
    d = json.load(open(path)); name = os.path.basename(path)[:-5]
    res = d["result"]; obs = res["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s")
    if not (isinstance(tau, dict) and tau.get("status") == "ok"):
        continue
    fp = float(R.get("feedback_period_s") or 0.01); n_resp = int(R.get("latency_probes_responded") or 0); tt = d["truth"].get("tau_w_s")
    trials = Lv["trials"]
    # the replay of reanalyze_liveness_v015/v016: a last-stream latency below the run's response floor was not a response
    resp = [t["time_to_respond_s"] for t in trials if t.get("responded") and isinstance(t.get("time_to_respond_s"), float) and not math.isnan(t["time_to_respond_s"])]
    floor = float(np.median(resp)) - fp if resp else -math.inf
    replay = [dict(t, last_stream_latency_s=float("nan")) if (isinstance(t.get("last_stream_latency_s"), float) and not math.isnan(t["last_stream_latency_s"]) and t["last_stream_latency_s"] < floor) else t for t in trials]
    out = {"run": name, "mode": d["truth"].get("mode"), "tau_true_s": tt, "stop_class": Lv.get("stop_class"), "response_wait_s": Lv.get("response_timeout_s")}
    for rule in ("0.1.6", "0.1.7"):
        fw = fault_observation_window(Lv["response_timeout_s"], tau["granularity_allowance_s"]) if rule == "0.1.7" else None
        iv = timeout_interval_from_trials(replay, L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=fp, hold_tol=Lv["hold_tolerance_m"],
                                          stop_class=Lv["stop_class"], baseline_loss=12 - n_resp, rule=rule, fault_window_s=fw)
        ok = iv["status"] == "formed" and iv["interval_low_s"] > R["resolution_floor_s"]
        k = rule.replace(".", "")
        out.update({f"v{k}_status": "ok" if ok else iv["status"], f"v{k}_low_s": iv["interval_low_s"], f"v{k}_high_s": iv["interval_high_s"],
                    f"v{k}_contains": bool(ok and iv["interval_low_s"] <= tt <= iv["interval_high_s"]),
                    f"v{k}_width_over_tau": (iv["interval_high_s"] - iv["interval_low_s"]) / tt if ok else None})
        hz = (d["truth"] or {}).get("horizon_s")
        if hz is not None and d["truth"].get("mode") == "fault":
            t_ = {"status": "ok" if ok else iv["status"], "interval_low_s": iv["interval_low_s"], "interval_high_s": iv["interval_high_s"]}
            out[f"v{k}_fault_horizon_{hz}"] = fault_horizon_decision(t_, float(hz), float(d["truth"].get("eq7_need_s") or NEED))[0]
    out["archived_sub_verdict"] = (res["estimates"].get("sub_verdicts") or {}).get("stop_behaviour")
    arch.append(out)
keys = sorted({k for r in arch for k in r}, key=lambda k: (k not in ("run", "mode", "tau_true_s", "stop_class", "response_wait_s"), k))
with open(os.path.join(OUT, "L_archived_v017.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys, lineterminator="\n"); w.writeheader(); w.writerows(arch)
f7 = [r for r in arch if r["v017_status"] == "ok"]; f6 = [r for r in arch if r["v016_status"] == "ok"]
w7 = np.array([r["v017_width_over_tau"] for r in f7]); w6 = np.array([r["v016_width_over_tau"] for r in f6])
asum = {"formed_v016": len(f6), "contain_v016": sum(r["v016_contains"] for r in f6), "formed_v017": len(f7), "contain_v017": sum(r["v017_contains"] for r in f7),
        "not_formed_v017": [r["run"] for r in arch if r["v017_status"] != "ok"],
        "width_over_tau_v016": {"median": float(np.median(w6)), "min": float(w6.min()), "max": float(w6.max())},
        "width_over_tau_v017": {"median": float(np.median(w7)), "min": float(w7.min()), "max": float(w7.max()),
                                "fault_median": float(np.median([r["v017_width_over_tau"] for r in f7 if r["mode"] == "fault"])),
                                "drift_median": float(np.median([r["v017_width_over_tau"] for r in f7 if r["mode"] == "release"]))},
        "fault_horizon_claims": {r["run"]: {"archived": r["archived_sub_verdict"], **{k: v for k, v in r.items() if "fault_horizon" in k}} for r in arch if any("fault_horizon" in k for k in r)}}
summary = {"note": "POST HOC re-derivation under the 0.1.7 rules; no run repeated", "consistency_gate": gate, "loss_fault_runs": lsum, "archived_v013": asum}
json.dump({"summary": summary, "loss_rows": loss}, open(os.path.join(OUT, "rederivation_v017.json"), "w"), indent=1, default=str)
print(json.dumps(summary, indent=1, default=str))
