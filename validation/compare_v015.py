#!/usr/bin/env python3
"""Compare the v0.1.5 verification campaign (run_validation_v015.py) with the v0.1.3 archive, experiment by experiment,
from the run JSON files (no analyzer output needed).  Both archives are read only.

Usage: python3 validation/compare_v015.py [--old validation/v0.1.3/mock] [--new validation/v0.1.5/mock] [--out validation/v0.1.5/COMPARISON.md]
"""
import argparse
import glob
import json
import os

import numpy as np

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument("--old", default=os.path.join(here, "v0.1.3", "mock"))
ap.add_argument("--new", default=os.path.join(here, "v0.1.5", "mock"))
ap.add_argument("--out", default=os.path.join(here, "v0.1.5", "COMPARISON.md"))
args = ap.parse_args()


def runs(d, prefix):
    out = {}
    for p in sorted(glob.glob(os.path.join(d, prefix + "*.json"))):
        name = os.path.basename(p)[:-5]
        if name == "meta":
            continue
        out[name] = json.load(open(p))
    return out


def outcome(rec):
    if "result" in rec:
        return rec["result"]["outcome"]
    if "results" in rec:
        return "/".join(r["outcome"] for r in rec["results"])
    if "probes" in rec:
        return "/".join(r["outcome"] for r in rec["probes"])
    return None


md = ["# v0.1.5 verification campaign versus the v0.1.3 archive (reference node; same design, seeds +30000)", ""]
old_meta = json.load(open(os.path.join(args.old, "meta.json"))); new_meta = json.load(open(os.path.join(args.new, "meta.json")))
md.append(f"v0.1.3: {old_meta['crtk_conformance_version']} at {old_meta['repo_commit'][:7]}, {old_meta['started_at'][:10]}, wall {old_meta['wall_s']/60:.1f} min; "
          f"v0.1.5: {new_meta['crtk_conformance_version']} at {new_meta['repo_commit'][:7]}{' (working tree with the 0.1.5 patch)' if new_meta.get('repo_dirty') else ''}, {new_meta['started_at'][:10]}, wall {new_meta['wall_s']/60:.1f} min; image {new_meta.get('container_digest')}.")
md.append("")

# ---- outcomes of every run present in both archives.  The archived files carry the 0.1.3 outcomes; they are compared
# under the current rules: a declared rate requirement is undetermined since 0.1.4 (every T_ run, the authored presets and
# the regression cases), and a stop sub-verdict resting on rejection-class trials under baseline command loss is
# undetermined since 0.1.5 (R_drop_50).
def combine(sub):
    declared = [v for v in sub.values() if v is not None]
    if "violated" in declared:
        return "divergent"
    if declared and all(v == "satisfied" for v in declared):
        return "conformant"
    return "undetermined"


def current_rules(name, rec):
    out = outcome(rec)
    if name.startswith("T_"):
        return "undetermined"
    key = "results" if "results" in rec else ("probes" if "probes" in rec else None)
    if key is None:
        return out
    parts = []
    for r in rec[key]:
        if r["binding_class"] != "temporal":
            parts.append(r["outcome"]); continue
        sub = dict(r["estimates"].get("sub_verdicts", {}))
        if sub.get("rate") is not None:
            sub["rate"] = "undetermined"
        ob = r["observations"]
        if ob.get("resolution") and ob.get("liveness"):
            n_resp = int(ob["resolution"].get("latency_probes_responded") or 0)
            tripped = [t for t in ob["liveness"].get("trials", []) if t["class"] in ("rejected", "faulted", "drifted")]
            if 12 - n_resp > 0 and tripped and all(t["class"] == "rejected" for t in tripped) and sub.get("stop_behaviour") is not None:
                sub["stop_behaviour"] = "undetermined"
        parts.append(combine(sub) if any(v is not None for v in sub.values()) else r["outcome"])
    return "/".join(parts)


common = []
for prefix in ("F_", "C_", "S_", "T_", "L_", "P_", "R_", "PRESET_"):
    o, n = runs(args.old, prefix), runs(args.new, prefix)
    for name in sorted(set(o) & set(n)):
        if outcome(o[name]) is None:
            continue  # coverage replication records carry per-replicate outcomes, compared in their own section
        common.append((name, current_rules(name, o[name]), outcome(n[name])))
same = sum(1 for _, a, b in common if a == b)
md += [f"## Outcomes of the {len(common)} runs common to both campaigns (archive under the current rules)", "", f"Identical outcome in {same} of {len(common)} runs.", ""]
diff = [(nm, a, b) for nm, a, b in common if a != b]
if diff:
    md += ["| run | v0.1.3 outcome | v0.1.5 outcome |", "|---|---|---|"] + [f"| {nm} | {a} | {b} |" for nm, a, b in diff] + [""]

# ---- frame: containment and errors
def frame_stats(d):
    rs = runs(d, "F_id_")
    tol = 1e-9  # metres: floating-point comparison tolerance of the analyzer, not a task tolerance
    cont = sum(1 for r in rs.values() if r["result"]["predicted_error_ci"] and r["result"]["predicted_error_ci"][0] - tol <= r["truth"]["exact_max_error_m"] <= r["result"]["predicted_error_ci"][1] + tol)
    err = [np.linalg.norm(np.array(r["result"]["estimates"]["binding_translation_m"]) - np.array(r["truth"]["t_m"])) * 1e3 for r in rs.values() if r["truth"].get("noise_m") == 0.0001]
    return len(rs), cont, (np.mean(err) if err else float("nan")), (max(err) if err else float("nan"))


try:
    o = frame_stats(args.old); n = frame_stats(args.new)
    md += ["## Frame sweep (F_id)", "", f"v0.1.3: {o[1]}/{o[0]} intervals contain the exact maximum error; translation error at 0.1 mm noise mean/max {o[2]:.3f}/{o[3]:.3f} mm.",
           f"v0.1.5: {n[1]}/{n[0]}; mean/max {n[2]:.3f}/{n[3]:.3f} mm.", ""]
except Exception as e:  # noqa: BLE001
    md += ["## Frame sweep", "", f"(frame statistics not computed: {e})", ""]

# ---- coverage
def cov(d):
    out = {}
    for name, r in runs(d, "C_cov_").items():
        reps = r["replicates"]
        tt = r["truth"]["exact_max_error_m"]
        out[name] = (len(reps), sum(1 for x in reps if x["predicted_abs_error_at_workspace_edge_m"]["ci_low"] - 1e-9 <= tt <= x["predicted_abs_error_at_workspace_edge_m"]["ci_high"] + 1e-9),
                     {k: sum(1 for x in reps if x["outcome"] == k) for k in ("conformant", "divergent", "undetermined")})
    return out


try:
    o, n = cov(args.old), cov(args.new)
    md += ["## Spatial coverage replication (C_cov)", "", "| configuration | v0.1.3 contain | v0.1.3 C/D/U | v0.1.5 contain | v0.1.5 C/D/U |", "|---|---|---|---|---|"]
    for name in sorted(o):
        a, b = o[name], n.get(name)
        md.append(f"| {name} | {a[1]}/{a[0]} | {a[2]['conformant']}/{a[2]['divergent']}/{a[2]['undetermined']} | " + (f"{b[1]}/{b[0]} | {b[2]['conformant']}/{b[2]['divergent']}/{b[2]['undetermined']} |" if b else "— | — |"))
    md.append("")
except Exception as e:  # noqa: BLE001
    md += ["## Spatial coverage", "", f"(not computed: {e})", ""]

# ---- scale
def scale_stats(d):
    rs = runs(d, "S_si_")
    c = {k: sum(1 for r in rs.values() if r["result"]["outcome"] == k) for k in ("conformant", "divergent", "undetermined")}
    rel = [abs(r["result"]["estimates"]["scale_anchored"]["mean"] - r["truth"]["s"]) / r["truth"]["s"] for r in rs.values() if (r["result"]["estimates"].get("scale_anchored") or {}).get("mean") is not None]
    return len(rs), c, (max(rel) * 100 if rel else float("nan"))


try:
    o, n = scale_stats(args.old), scale_stats(args.new)
    md += ["## Anchored scale sweep (S_si)", "", f"v0.1.3: {o[0]} runs, C/D/U {o[1]['conformant']}/{o[1]['divergent']}/{o[1]['undetermined']}, max relative error of s_hat {o[2]:.2f} %.",
           f"v0.1.5: {n[0]} runs, C/D/U {n[1]['conformant']}/{n[1]['divergent']}/{n[1]['undetermined']}, max relative error {n[2]:.2f} %.", ""]
except Exception as e:  # noqa: BLE001
    md += ["## Scale sweep", "", f"(not computed: {e})", ""]

# ---- liveness
def live_stats(d):
    rows = []
    for name, r in runs(d, "L_").items():
        L = r["result"]["observations"]["liveness"]; tau = L.get("tau_w_estimate_s") or {}; tt = r["truth"].get("tau_w_s")
        st = tau.get("status") if isinstance(tau, dict) else None
        ok = st == "ok"
        rows.append(dict(run=name, status=st or ("upper_bound" if isinstance(tau, dict) and "upper_bound_s" in tau else "none"), contains=(tau["interval_low_s"] <= tt <= tau["interval_high_s"]) if ok else None,
                         width=(tau["interval_high_s"] - tau["interval_low_s"]) * 1e3 if ok else None, low=tau.get("interval_low_s") if ok else None, high=tau.get("interval_high_s") if ok else None,
                         conditional=tau.get("conditional") if ok else None, tau=tt, stop=r["result"]["estimates"]["sub_verdicts"].get("stop_behaviour"), outcome=r["result"]["outcome"],
                         departures=sum(1 for t in L.get("trials", []) if t.get("last_stream_departure_observed")), n_trials=len(L.get("trials", [])),
                         r_i=[t.get("last_stream_latency_s") for t in L.get("trials", []) if isinstance(t.get("last_stream_latency_s"), float)]))
    return rows


o, n = live_stats(args.old), live_stats(args.new)
fo = [r for r in o if r["status"] == "ok"]; fn = [r for r in n if r["status"] == "ok"]
md += ["## Liveness (L)", "", f"v0.1.3: {len(o)} runs, {len(fo)} formed, {sum(bool(r['contains']) for r in fo)}/{len(fo)} contain the injected timeout, {sum(bool(r['conditional']) for r in fo)} conditional, median width {np.median([r['width'] for r in fo]):.1f} ms.",
       f"v0.1.5: {len(n)} runs, {len(fn)} formed, {sum(bool(r['contains']) for r in fn)}/{len(fn)} contain, {sum(bool(r['conditional']) for r in fn)} conditional, median width {np.median([r['width'] for r in fn]) if fn else float('nan'):.1f} ms.", "",
       "| run | tau_w (s) | v0.1.3 interval (ms) / stop / outcome | v0.1.5 interval (ms) / stop / outcome | v0.1.5 departures observed | v0.1.5 r_i (ms) |", "|---|---|---|---|---|---|"]
on = {r["run"]: r for r in o}
for r in n:
    a = on.get(r["run"])
    fa = (f"[{a['low']*1e3:.1f}, {a['high']*1e3:.1f}]" if a and a["status"] == "ok" else (a["status"] if a else "—")) + (f" / {a['stop']} / {a['outcome']}" if a else "")
    fb = (f"[{r['low']*1e3:.1f}, {r['high']*1e3:.1f}]" if r["status"] == "ok" else r["status"]) + f" / {r['stop']} / {r['outcome']}"
    ri = (f"{min(r['r_i'])*1e3:.1f}–{max(r['r_i'])*1e3:.1f}" if r["r_i"] else "none (L used)")
    md.append(f"| {r['run']} | {r['tau']} | {fa} | {fb} | {r['departures']}/{r['n_trials']} | {ri} |")
md.append("")

# ---- experiment V
md += ["## Experiment V (0.1.5 verification cases; no v0.1.3 counterpart)", "", "| run | injected | expectation | outcome | stop sub-verdict | timeout estimate | departures | r_i (ms) | calibration lost | confounded | finding |", "|---|---|---|---|---|---|---|---|---|---|---|"]
for name, r in runs(args.new, "V_").items():
    res = r["result"]; L = res["observations"]["liveness"]; tau = L.get("tau_w_estimate_s") or {}
    est = (f"[{tau['interval_low_s']*1e3:.1f}, {tau['interval_high_s']*1e3:.1f}] ms" + ("" if r["truth"].get("tau_w_s") is None else (" contains" if tau["interval_low_s"] <= r["truth"]["tau_w_s"] <= tau["interval_high_s"] else " MISSES") + f" {r['truth']['tau_w_s']*1e3:.0f} ms")) if tau.get("status") == "ok" else str(tau.get("status")) + (f" (upper bound {tau['upper_bound_s']*1e3:.1f} ms)" if tau.get("upper_bound_s") else "")
    ri = [t.get("last_stream_latency_s") for t in L.get("trials", []) if isinstance(t.get("last_stream_latency_s"), float)]
    md.append(f"| {name} | {r['truth'].get('case', '')} | {r['expectation']['temporal'].get('stop_behaviour')} | {res['outcome']} | {res['estimates']['sub_verdicts'].get('stop_behaviour')} | {est} | "
              f"{sum(1 for t in L.get('trials', []) if t.get('last_stream_departure_observed'))}/{len(L.get('trials', []))} | {(f'{min(ri)*1e3:.1f}–{max(ri)*1e3:.1f}' if ri else 'none (L used)')} | "
              f"{(L.get('baseline_command_loss') or {}).get('lost', 0)} | {L.get('rejection_confounded_by_command_loss')} | {L.get('finding', '')[:160]} |")
md.append("")

# ---- rate diagnostic: containment against the event log is computed by analyze_v013.py; here only the sub-verdicts
def rate_undetermined(d):
    rs = runs(d, "T_")
    tot = 0
    for r in rs.values():
        for w in r["result"]["observations"].get("effective_rate", {}).get("per_rate", []):
            tot += 1
    return len(rs), tot


md += ["## Rate windows (T)", "", f"v0.1.3: {rate_undetermined(args.old)[0]} runs / {rate_undetermined(args.old)[1]} windows; v0.1.5: {rate_undetermined(args.new)[0]} runs / {rate_undetermined(args.new)[1]} windows. "
       "The rate sub-verdict is undetermined in every window since v0.1.4; containment of the source-age diagnostic against the event log is computed by analyze_v013.py on each archive (tables/T_rate_summary.json).", ""]
for d, lab in ((args.old, "v0.1.3"), (args.new, "v0.1.5")):
    p = os.path.join(d, "tables", "T_rate_summary.json")
    if os.path.exists(p):
        t = json.load(open(p)); md.append(f"{lab} analyzer summary: " + ", ".join(f"{k} = {v}" for k, v in t.items() if isinstance(v, (int, float))) + ".")
md.append("")
open(args.out, "w").write("\n".join(md) + "\n")
print("\n".join(md))
