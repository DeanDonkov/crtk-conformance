#!/usr/bin/env python3
"""Offline analysis of the v0.1.3 archives: tables (CSV + Markdown) and figures (PDF/PNG).

Usage: python validation/analyze_v013.py validation/v0.1.3/mock [--live validation/v0.1.3/live-src-v1 ...] [--figdir figures]
Nothing here talks to ROS; everything is computed from the archived JSON and the mock's event logs.  The decision
rules reproduced here are the released ones (probes/base.py decide() with the 1e-9 boundary guard for the tolerance
sweeps; rate_estimator.rate_subverdict for every rate window).

Rate truth (RC4 adversarial review, finding 5).  For every rate window the truth is NOT derived from the mock's
configuration but from the mock's own event log: the apply events are matched to the probe's sends by the client's
header stamp, and the true source age of the applied setpoint over the window is the largest of (i) the time from
the first send to the first application of a command of this window ("nothing applied yet"), (ii) for each
application, the age of the previously applied command at that moment, and (iii) the age of the last applied
command at the window end (the application of the last target, or the end of the observation).  A window's truth
class is `ok` (true age <= required period), `violated` (true age > period while the client's own stream sustained
the rate: mean rate >= f_req and longest send gap <= period) or `client_limited`.  Every window at every requested
rate is scored (the estimator's verdict is re-derived from the archived bracket with the released rule); a verdict
is SOUND when it is undetermined or agrees with the truth class; a false conformant is `satisfied` on a window whose
truth is not `ok`; a false divergent is `violated` on a window whose truth is not `violated`.  COMPLETENESS is the
fraction of windows with a determinate truth (`ok` or `violated`) that received a determinate verdict.  UNSCORED
windows (no event log, no sent command, no channel) are counted separately.  Bracket containment (age_lower <=
true age <= age_upper) is reported for every window with an `ok` estimate; an excess of age_lower over the true
age measures the channel's transport delay, which the (conditional) violated verdict assumes below its margin.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from crtk_conformance import geometry as G  # noqa: E402
from crtk_conformance.probes.base import decide as _decide, Outcome  # noqa: E402
from crtk_conformance.rate_estimator import AppliedAgeEstimate, rate_subverdict  # noqa: E402

C_BLUE, C_ORANGE, C_GREEN, C_PINK, C_GREY = "#0072B2", "#E69F00", "#009E73", "#CC79A7", "#6e6e6e"
TOL_M = 1e-9  # containment is judged with a 1 nm tolerance: at zero noise the interval collapses to a point and the truth is the same number up to floating-point rounding


def load(d, prefix):
    out = []
    for f in sorted(glob.glob(os.path.join(d, prefix + "*.json"))):
        with open(f) as fh:
            out.append((os.path.basename(f)[:-5], json.load(fh)))
    return out


def decide(lo, hi, eps):
    if lo is None or hi is None:
        return "undetermined"
    return _decide(float(lo), float(hi), eps).value


def sweep(records, eps_grid):
    rows = []
    for eps in eps_grid:
        pos = neg = fp = fn = und = 0
        for truth, lo, hi in records:
            label_pos = truth > eps
            d = decide(lo, hi, eps)
            if label_pos:
                pos += 1
                fn += d == "conformant"
            else:
                neg += 1
                fp += d == "divergent"
            und += d == "undetermined"
        rows.append({"eps_mm": eps * 1e3, "positives": pos, "negatives": neg, "FP": fp, "FN": fn, "undetermined": und,
                     "FPR": fp / neg if neg else float("nan"), "FNR": fn / pos if pos else float("nan"), "undetermined_rate": und / len(records)})
    return rows


def write_csv(path, rows):
    if not rows:
        return
    keys = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def load_events(d, rec):
    """The mock's event log of a run (list of dicts) or None."""
    name = rec.get("event_log")
    if not name:
        return None
    p = os.path.join(d, name)
    if not os.path.exists(p):
        return None
    out = []
    with open(p) as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def rate_truth(row, events, f_req):
    """Truth of one rate window from the event log (module docstring).  Returns a dict or None (unscorable)."""
    tr = row.get("trace") or {}
    send = tr.get("send_times_mono") or []
    stamps = tr.get("send_stamps_wall") or []
    if not send or events is None:
        return None
    n = len(send)
    by_stamp = {round(float(st), 6): k for k, st in enumerate(stamps)}
    applies = sorted((float(e["t"]), by_stamp[round(float(e["stamp"]), 6)]) for e in events if e.get("event") == "apply" and round(float(e["stamp"]), 6) in by_stamp)
    received = {by_stamp[round(float(e["stamp"]), 6)] for e in events if e.get("event") == "receive" and round(float(e["stamp"]), 6) in by_stamp}
    not_applied = {by_stamp[round(float(e["stamp"]), 6)]: e["event"] for e in events if e.get("event") in ("drop", "reject", "ignore", "supersede", "discard") and round(float(e["stamp"]), 6) in by_stamp}
    t_obs_end = float(tr.get("window_end_mono", send[-1]))
    w0 = float(send[0])
    period = 1.0 / f_req
    t_last = next((t for t, k in applies if k == n - 1), None)
    w1 = min(t_last, t_obs_end) if t_last is not None else t_obs_end
    w1 = max(w1, float(send[-1]))
    inwin = [(t, k) for t, k in applies if t <= w1 + 1e-9]
    if inwin:
        ages = [inwin[0][0] - w0] + [inwin[j][0] - float(send[inwin[j - 1][1]]) for j in range(1, len(inwin))] + [w1 - float(send[inwin[-1][1]])]
    else:
        ages = [w1 - w0]
    true_age = max(ages)
    ooo = sum(1 for j in range(1, len(inwin)) if inwin[j][1] < inwin[j - 1][1])
    diffs = np.diff(np.asarray(send, dtype=float)) if n > 1 else np.array([])
    achieved = (n - 1) / (send[-1] - send[0]) if n > 1 and send[-1] > send[0] else float("nan")
    max_gap = float(diffs.max()) if len(diffs) else float("nan")
    client_ok = (not math.isnan(achieved)) and achieved >= f_req * (1 - 1e-6) and not (not math.isnan(max_gap) and max_gap > period * (1 + 1e-6))
    if true_age <= period * (1 + 1e-9):
        cls = "ok"
    elif client_ok:
        cls = "violated"
    else:
        cls = "client_limited"
    return {"true_age_s": true_age, "class": cls, "applied": len(inwin), "applied_distinct": len({k for _, k in inwin}), "received": len(received & set(range(n))),
            "never_applied": n - len({k for _, k in applies}), "not_applied_events": len(not_applied), "out_of_order": ooo, "window_end_s": w1, "last_target_applied": t_last is not None,
            "client_achieved_hz": achieved, "client_max_send_gap_s": max_gap, "client_ok": client_ok}


def score_window(row, truth, f_req):
    """(verdict, category) of one window: verdict re-derived with the released rule from the archived bracket."""
    acc = row.get("acceptance") or {}
    est = AppliedAgeEstimate(**{k: v for k, v in acc.items() if k in AppliedAgeEstimate.__dataclass_fields__}) if acc else None
    if est is not None:
        for k in ("age_lower_s", "age_upper_s", "client_rate_achieved_hz", "client_max_send_gap_s", "first_application_delay_s", "max_update_gap_s", "channel_period_s"):
            if getattr(est, k) is None:
                setattr(est, k, float("nan"))
    verdict = rate_subverdict(est, f_req)
    if truth is None:
        return verdict, "unscored"
    if verdict == "undetermined":
        return verdict, "sound_undetermined"
    if verdict == "satisfied":
        return verdict, "sound" if truth["class"] == "ok" else "false_conformant"
    return verdict, "sound" if truth["class"] == "violated" else "false_divergent"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("archive")
    ap.add_argument("--live", nargs="*", default=[])
    ap.add_argument("--figdir", default=None)
    args = ap.parse_args()
    d = args.archive
    tdir = os.path.join(d, "tables")
    os.makedirs(tdir, exist_ok=True)
    figdir = args.figdir or tdir
    os.makedirs(figdir, exist_ok=True)
    md = ["# v0.1.3 mock validation — summary tables (generated by analyze_v013.py; every number from the archived JSON and event logs)", ""]
    meta = json.load(open(os.path.join(d, "meta.json")))
    md.append(f"Archive: commit {meta['repo_commit'][:12]}, {meta['started_at']} → {meta['finished_at']}, wall {meta['wall_s']/60:.1f} min, "
              f"Python {meta['python'].split()[0]}, numpy {meta['numpy']}, scipy {meta['scipy']}, container {meta.get('container_image')} ({str(meta.get('container_digest'))[:19]}).")
    md.append("")
    false_conformant = []
    false_divergent = []

    # ------------------------------------------------------------------ F identity grid
    frows, frec, frec_bound = [], [], []
    for name, r in load(d, "F_id_"):
        tr, res, est = r["truth"], r["result"], r["result"]["estimates"]
        t_hat = np.array(est.get("binding_translation_m", [np.nan] * 3))
        t_true = np.array(tr["t_m"])
        th_hat = est.get("binding_rotation_deg", {}).get("mean", float("nan"))
        sd = est.get("spatial_decision") or {}
        row = {"run": name, "t_true_mm": tr["t_norm_m"] * 1e3, "theta_true_deg": tr["theta_deg"], "noise_mm": tr["noise_m"] * 1e3,
               "t_hat_mm": float(np.linalg.norm(t_hat)) * 1e3, "t_err_mm": float(np.linalg.norm(t_hat - t_true)) * 1e3,
               "theta_hat_deg": th_hat, "theta_err_deg": abs(th_hat - tr["theta_deg"]) if not math.isnan(th_hat) else float("nan"),
               "exact_max_true_mm": tr["exact_max_error_m"] * 1e3, "bound_eq3_true_mm": tr["bound_eq3_m"] * 1e3,
               "e_max_hat_mm": sd.get("e_max_m", float("nan")) * 1e3 if sd else float("nan"),
               "ci_low_mm": sd.get("ci_low_m", float("nan")) * 1e3 if sd else float("nan"), "ci_high_mm": sd.get("ci_high_m", float("nan")) * 1e3 if sd else float("nan"),
               "delta_t_mm": sd.get("delta_t_m", float("nan")) * 1e3 if sd else float("nan"), "delta_rho_deg": math.degrees(sd.get("delta_rho_rad", float("nan"))) if sd else float("nan"),
               "contains_truth": (sd.get("ci_low_m") - TOL_M <= tr["exact_max_error_m"] <= sd.get("ci_high_m") + TOL_M) if sd else None,
               "outcome": res["outcome"], "unpaired": est.get("unpaired_samples")}
        frows.append(row)
        frec.append((tr["exact_max_error_m"], sd.get("ci_low_m"), sd.get("ci_high_m")))
        if res["outcome"] == "conformant" and tr["exact_max_error_m"] > 0.001:
            false_conformant.append(name)
        if res["outcome"] == "divergent" and tr["exact_max_error_m"] <= 0.001:
            false_divergent.append(name + f" (spatial, true e_max {tr['exact_max_error_m']*1e3:.3f} mm)")
    write_csv(os.path.join(tdir, "F_id_runs.csv"), frows)
    fsum = []
    for noise in sorted({r["noise_mm"] for r in frows}):
        sub = [r for r in frows if r["noise_mm"] == noise]
        zero = [r for r in sub if r["t_true_mm"] == 0 and r["theta_true_deg"] == 0]
        fsum.append({"noise_mm": noise, "runs": len(sub), "t_err_mean_mm": float(np.mean([r["t_err_mm"] for r in sub])), "t_err_max_mm": float(np.max([r["t_err_mm"] for r in sub])),
                     "theta_err_max_deg": float(np.max([r["theta_err_deg"] for r in sub])), "floor_t_hat_at_zero_mm": zero[0]["t_hat_mm"] if zero else float("nan"),
                     "intervals_containing_truth": sum(1 for r in sub if r["contains_truth"]), "median_half_width_mm": float(np.median([(r["ci_high_mm"] - r["ci_low_mm"]) / 2 for r in sub])),
                     "conformant": sum(r["outcome"] == "conformant" for r in sub), "divergent": sum(r["outcome"] == "divergent" for r in sub), "undetermined": sum(r["outcome"] == "undetermined" for r in sub)})
    write_csv(os.path.join(tdir, "F_id_summary.csv"), fsum)
    eps_grid = np.logspace(math.log10(0.05e-3), math.log10(50e-3), 61)
    fsweep = sweep(frec, eps_grid)
    write_csv(os.path.join(tdir, "F_id_threshold_sweep.csv"), fsweep)
    md += ["## Frame probe, identity expectation (F_id, %d runs; truth = exact maximum error eq. (3'))" % len(frows), "", "| noise (mm) | runs | mean ‖t̂−t‖ (mm) | max ‖t̂−t‖ (mm) | max |θ̂−θ| (deg) | floor ‖t̂‖ at t=0 (mm) | intervals containing the truth | median half-width (mm) | conformant / divergent / undetermined at ε = 1 mm |", "|---|---|---|---|---|---|---|---|---|"]
    for r in fsum:
        md.append(f"| {r['noise_mm']:g} | {r['runs']} | {r['t_err_mean_mm']:.4f} | {r['t_err_max_mm']:.4f} | {r['theta_err_max_deg']:.2e} | {r['floor_t_hat_at_zero_mm']:.4f} | {r['intervals_containing_truth']} / {r['runs']} | {r['median_half_width_mm']:.4f} | {r['conformant']} / {r['divergent']} / {r['undetermined']} |")
    fp_max = max(r["FP"] for r in fsweep); fn_max = max(r["FN"] for r in fsweep); und_max = max(r["undetermined"] for r in fsweep)
    md.append(f"\nTolerance sweep 0.05–50 mm (61 points): max false divergent (FP) {fp_max}, max false conformant (FN) {fn_max}, max undetermined {und_max} of {len(frec)} " +
              ("(" + "; ".join(f"ε={r['eps_mm']:.3g} mm: FP {r['FP']} FN {r['FN']} und {r['undetermined']}" for r in fsweep if r['FP'] or r['FN']) + ")" if (fp_max or fn_max) else "") + ".")
    md.append("")
    # coverage experiment
    crows = []
    for name, r in load(d, "C_cov_"):
        tr = r["truth"]
        reps = r["replicates"]
        sds = [x["spatial_decision"] for x in reps if x["spatial_decision"]]
        widths = [(x["ci_high_m"] - x["ci_low_m"]) * 1e3 for x in sds if math.isfinite(x["ci_high_m"])]
        hits = sum(1 for x in sds if x["ci_low_m"] - TOL_M <= tr["exact_max_error_m"] <= x["ci_high_m"] + TOL_M)
        crows.append({"config": name[6:], "true_e_max_mm": tr["exact_max_error_m"] * 1e3, "noise_mm": tr["noise_m"] * 1e3, "orientation_noise_deg": tr.get("orientation_noise_deg", 0.0), "replicates": len(reps), "trials_per_replicate": tr["trials"],
                      "coverage": hits / len(reps), "median_width_mm": float(np.median(widths)) if widths else float("nan"),
                      "conformant": sum(x["outcome"] == "conformant" for x in reps), "divergent": sum(x["outcome"] == "divergent" for x in reps), "undetermined": sum(x["outcome"] == "undetermined" for x in reps),
                      "false_divergent": sum(1 for x in reps if x["outcome"] == "divergent" and tr["exact_max_error_m"] <= 0.001), "false_conformant": sum(1 for x in reps if x["outcome"] == "conformant" and tr["exact_max_error_m"] > 0.001)})
    if crows:
        write_csv(os.path.join(tdir, "C_coverage.csv"), crows)
        md += ["## Spatial interval coverage (C_cov; replicated frame-probe runs, n = %d trials each)" % crows[0]["trials_per_replicate"], "", "| configuration | true e_max (mm) | noise (mm / deg) | replicates | coverage | median width (mm) | conformant / divergent / undetermined at ε = 1 mm | false divergent / false conformant |", "|---|---|---|---|---|---|---|---|"]
        for r in crows:
            md.append(f"| {r['config']} | {r['true_e_max_mm']:.3f} | {r['noise_mm']:g} / {r['orientation_noise_deg']:g} | {r['replicates']} | {r['coverage']:.3f} | {r['median_width_mm']:.4f} | {r['conformant']} / {r['divergent']} / {r['undetermined']} | {r['false_divergent']} / {r['false_conformant']} |")
        md.append("")
    # other frame cases
    md += ["| frame case | injected | expectation | outcome | ‖t̂‖ (mm) | θ̂ (deg) | residual (mm) | ê_max (mm) | interval (mm) | true e_max (mm) | note |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    fx = []
    for prefix in ("F_exp_", "F_expwrong_", "F_disc_", "F_nolocal", "F_orient_", "F_skew_", "F_rev_", "F_unit01_", "F_oritol_"):
        for name, r in load(d, prefix):
            tr, res, est = r["truth"], r["result"], r["result"]["estimates"]
            sd = est.get("spatial_decision") or {}
            truth_e = tr.get("exact_max_error_m")
            row = {"run": name, "truth": json.dumps(tr), "expectation": r["expectation"]["spatial"]["mode"], "outcome": res["outcome"],
                   "t_hat_mm": est.get("binding_translation_norm_m", {}).get("mean", float("nan")) * 1e3 if est.get("binding_translation_norm_m") else float("nan"),
                   "theta_hat_deg": est.get("binding_rotation_deg", {}).get("mean", float("nan")) if est.get("binding_rotation_deg") else float("nan"),
                   "residual_mm": est.get("residual_translation_norm_m", float("nan")) * 1e3 if "residual_translation_norm_m" in est else float("nan"),
                   "e_max_hat_mm": sd.get("e_max_m", float("nan")) * 1e3 if sd else float("nan"),
                   "ci_mm": f"{sd['ci_low_m']*1e3:.3f}..{sd['ci_high_m']*1e3:.3f}" if sd else "—", "true_e_max_mm": truth_e * 1e3 if truth_e is not None else float("nan"),
                   "contains_truth": (sd["ci_low_m"] - TOL_M <= truth_e <= sd["ci_high_m"] + TOL_M) if (sd and truth_e is not None) else None,
                   "orientation_outcome": est.get("orientation_outcome"), "unpaired": est.get("unpaired_samples"), "basis": res["decision_basis"][:80]}
            fx.append(row)
            md.append(f"| {name} | {row['truth'][:60]} | {row['expectation']} | {res['outcome']} | {fmt(row['t_hat_mm'])} | {fmt(row['theta_hat_deg'])} | {fmt(row['residual_mm'])} | {fmt(row['e_max_hat_mm'])} | {row['ci_mm']} | {fmt(row['true_e_max_mm'])} | {row['basis'][:60]} |")
            if res["outcome"] == "conformant" and (r["expectation"]["spatial"]["mode"] == "discover_only" or (truth_e or 0) > 0.001 or (tr.get("residual_rotation_deg", 0) > tr.get("orientation_tolerance_deg", float("inf")))):
                false_conformant.append(name)
            if res["outcome"] == "divergent" and truth_e is not None and truth_e <= 0.001 and not (tr.get("residual_rotation_deg", 0) > tr.get("orientation_tolerance_deg", float("inf"))):
                false_divergent.append(name + f" (spatial, true e_max {truth_e*1e3:.3f} mm)")
    write_csv(os.path.join(tdir, "F_cases.csv"), fx)
    md.append("")

    # ------------------------------------------------------------------ S
    srows, srec = [], []
    for name, r in load(d, "S_si_"):
        tr, res, est = r["truth"], r["result"], r["result"]["estimates"]
        sa = est.get("scale_anchored", {})
        pe = est.get("predicted_error_at_workspace_edge_m", {})
        row = {"run": name, "s_true": tr["s"], "noise_mm": tr["noise_m"] * 1e3, "r_int_mean": est["internal_ratio"]["mean"], "s_hat_mean": sa.get("mean", float("nan")),
               "s_hat_rel_err": abs(sa.get("mean", float("nan")) - tr["s"]) / tr["s"], "s_hat_ci_low": sa.get("ci_low"), "s_hat_ci_high": sa.get("ci_high"),
               "model_error_mm": tr["model_error_m"] * 1e3, "outcome": res["outcome"], "valid_trials": r["result"]["observations"].get("valid_trials"),
               "latency_ms": est.get("response_latency_s", {}).get("mean", float("nan")) * 1e3 if est.get("response_latency_s") else float("nan")}
        srows.append(row)
        srec.append((tr["model_error_m"], pe.get("ci_low"), pe.get("ci_high")))
        if res["outcome"] == "conformant" and tr["model_error_m"] > 0.001:
            false_conformant.append(name)
        if res["outcome"] == "divergent" and tr["model_error_m"] <= 0.001:
            false_divergent.append(name + f" (dimensional, true error {tr['model_error_m']*1e3:.3f} mm)")
    write_csv(os.path.join(tdir, "S_si_runs.csv"), srows)
    ssweep = sweep(srec, eps_grid)
    write_csv(os.path.join(tdir, "S_si_threshold_sweep.csv"), ssweep)
    md += ["## Scale probe, SI expectation with anchor (S_si, %d runs)" % len(srows), "", "| noise (mm) | runs | r_int mean (min–max) | max rel. error of ŝ | conformant / divergent / undetermined at ε = 1 mm |", "|---|---|---|---|---|"]
    for noise in sorted({r["noise_mm"] for r in srows}):
        sub = [r for r in srows if r["noise_mm"] == noise]
        md.append(f"| {noise:g} | {len(sub)} | {np.mean([r['r_int_mean'] for r in sub]):.4f} ({min(r['r_int_mean'] for r in sub):.4f}–{max(r['r_int_mean'] for r in sub):.4f}) | {100*max(r['s_hat_rel_err'] for r in sub):.2f} % | "
                  f"{sum(r['outcome']=='conformant' for r in sub)} / {sum(r['outcome']=='divergent' for r in sub)} / {sum(r['outcome']=='undetermined' for r in sub)} |")
    fp_max = max(r["FP"] for r in ssweep); fn_max = max(r["FN"] for r in ssweep); und_max = max(r["undetermined"] for r in ssweep)
    md.append(f"\nTolerance sweep: max false divergent (FP) {fp_max}, max false conformant (FN) {fn_max}, max undetermined {und_max} of {len(srec)}.")
    md += ["", "| scale case | truth | expectation | outcome | r_int | ŝ | valid / no-response trials | basis |", "|---|---|---|---|---|---|---|---|"]
    sx = []
    for prefix in ("S_noanchor", "S_disc", "S_unit01", "S_anchornoise_", "S_drop_"):
        for name, r in load(d, prefix):
            tr, res, est, obs = r["truth"], r["result"], r["result"]["estimates"], r["result"]["observations"]
            row = {"run": name, "truth": json.dumps(tr), "expectation": r["expectation"]["dimensional"]["mode"], "outcome": res["outcome"], "r_int": est.get("internal_ratio", {}).get("mean"),
                   "s_hat": est.get("scale_anchored", {}).get("mean"), "valid_trials": obs.get("valid_trials"), "no_response": obs.get("no_response_trials"), "basis": res["decision_basis"][:90]}
            sx.append(row)
            md.append(f"| {name} | {row['truth'][:50]} | {row['expectation']} | {res['outcome']} | {fmt(row['r_int'], 4)} | {fmt(row['s_hat'], 4)} | {row['valid_trials']} / {row['no_response']} | {row['basis'][:70]} |")
            if res["outcome"] == "conformant" and (r["expectation"]["dimensional"]["mode"] == "discover_only" or tr.get("model_error_m", 0) > 0.001):
                false_conformant.append(name)
    write_csv(os.path.join(tdir, "S_cases.csv"), sx)
    md.append("")

    # ------------------------------------------------------------------ T rate (source age of the applied setpoint; truth from the event log)
    trows = []
    viol = 0
    f_req = 50.0  # v / epsilon = 0.05 / 0.001 (the campaign tolerance; recorded per report as required_rate_hz_from_tolerance)
    cat_counts = {"sound": 0, "sound_undetermined": 0, "false_conformant": 0, "false_divergent": 0, "unscored": 0}
    truth_counts = {"ok": 0, "violated": 0, "client_limited": 0}
    contain = {"n": 0, "contained": 0, "excess_lower_ms_max": 0.0, "excess_lower_ms": []}
    t_wrong = []
    for prefix in ("T_",):
        for name, r in load(d, prefix):
            tr, res = r["truth"], r["result"]
            f_req = res["estimates"].get("required_rate_hz_from_tolerance", f_req)
            events = load_events(d, r)
            sub_at_client = res["estimates"].get("sub_verdicts", {}).get("rate")
            for row in res["observations"]["effective_rate"]["per_rate"]:
                viol += row["transitions"] > row["commands_sent"]
                acc = row.get("acceptance") or {}
                f = row["command_rate_requested_hz"]
                truth = rate_truth(row, events, f_req) if row["commands_sent"] > 0 else None
                verdict, cat = score_window(row, truth, f_req)
                if truth is None and acc.get("channel") == "none":
                    cat = "unscored"  # no channel: undetermined by construction (the reviewer's 'no channel' cases); reported, not scored
                cat_counts[cat] += 1
                if truth is not None:
                    truth_counts[truth["class"]] += 1
                lo, hi = acc.get("age_lower_s"), acc.get("age_upper_s")
                contained = None
                excess = float("nan")
                if truth is not None and acc.get("status") == "ok" and lo is not None and hi is not None and not (isinstance(hi, float) and math.isnan(hi)):
                    contained = (lo - 1e-6 <= truth["true_age_s"] <= hi + 1e-6)
                    excess = (lo - truth["true_age_s"]) * 1e3
                    contain["n"] += 1
                    contain["contained"] += bool(contained)
                    contain["excess_lower_ms"].append(excess)
                    contain["excess_lower_ms_max"] = max(contain["excess_lower_ms_max"], excess)
                trows.append({"run": name, "case": tr.get("case", ""), "requested_hz": f, "client_achieved_hz": row["client_rate_achieved_hz"],
                              "client_max_send_gap_ms": ((acc.get("client_max_send_gap_s") if acc.get("client_max_send_gap_s") is not None else float("nan")) * 1e3), "commands": row["commands_sent"],
                              "crossings": row["transitions"], "applied_seen": acc.get("accepted"), "age_lower_ms": (lo if lo is not None else float("nan")) * 1e3, "age_upper_ms": (hi if hi is not None else float("nan")) * 1e3,
                              "age_lower_at_receipt_ms": (acc.get("age_lower_at_receipt_s") if acc.get("age_lower_at_receipt_s") is not None else float("nan")) * 1e3,
                              "true_age_ms": truth["true_age_s"] * 1e3 if truth else float("nan"), "true_applied": truth["applied"] if truth else None, "true_never_applied": truth["never_applied"] if truth else None,
                              "true_class": truth["class"] if truth else "unscored", "contains_truth": contained, "excess_lower_ms": excess,
                              "channel_period_ms": (acc.get("channel_period_s") or float("nan")) * 1e3, "publish_hz": row["publish_rate_hz"], "unmatched_fraction": row["unmatched_fraction"],
                              "acc_status": acc.get("status"), "acc_reason": acc.get("reason", ""), "row_status": row["status"], "verdict": verdict, "category": cat,
                              "sub_verdict_at_client_rate": sub_at_client, "outcome": res["outcome"]})
                if cat == "false_conformant":
                    false_conformant.append(f"{name}@{f:.0f}Hz")
                    t_wrong.append(f"{name}@{f:.0f}Hz: satisfied, true age {truth['true_age_s']*1e3:.1f} ms ({truth['class']})")
                if cat == "false_divergent":
                    false_divergent.append(f"{name}@{f:.0f}Hz (rate: violated, true age {truth['true_age_s']*1e3:.1f} ms, {truth['class']})")
                    t_wrong.append(f"{name}@{f:.0f}Hz: violated, true age {truth['true_age_s']*1e3:.1f} ms ({truth['class']})")
    write_csv(os.path.join(tdir, "T_rate_rows.csv"), trows)
    det_truth = truth_counts["ok"] + truth_counts["violated"]
    det_verdict = sum(1 for r in trows if r["true_class"] in ("ok", "violated") and r["verdict"] != "undetermined")
    t_summary = {"windows": len(trows), "scored": len(trows) - cat_counts["unscored"], "unscored": cat_counts["unscored"], "truth": truth_counts,
                 "sound_determinate": cat_counts["sound"], "sound_undetermined": cat_counts["sound_undetermined"], "false_conformant": cat_counts["false_conformant"], "false_divergent": cat_counts["false_divergent"],
                 "completeness": (det_verdict / det_truth) if det_truth else float("nan"), "determinate_truth_windows": det_truth, "determinate_verdicts_on_them": det_verdict,
                 "undetermined_on_client_limited": sum(1 for r in trows if r["true_class"] == "client_limited" and r["verdict"] == "undetermined"),
                 "bracket_windows": contain["n"], "bracket_contains_truth": contain["contained"], "excess_lower_ms_max": contain["excess_lower_ms_max"],
                 "excess_lower_ms_p95": float(np.percentile(contain["excess_lower_ms"], 95)) if contain["excess_lower_ms"] else float("nan"),
                 "crossings_invariant_violations": viol, "required_rate_hz": f_req}
    with open(os.path.join(tdir, "T_rate_summary.json"), "w") as fh:
        json.dump(t_summary, fh, indent=1)
    md += ["## Rate sub-probe: source age of the applied setpoint versus the event-log truth (%d windows; invariant crossings ≤ commands violated in %d rows)" % (len(trows), viol), "",
           "| run | case | requested (Hz) | client achieved (Hz) / max send gap (ms) | applied seen / commands | bracket [lower, upper] (ms) | true age (ms) | truth class | contains | verdict | category |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in trows:
        md.append(f"| {r['run']} | {r['case'][:40]} | {r['requested_hz']:.0f} | {r['client_achieved_hz']:.1f} / {fmt(r['client_max_send_gap_ms'], 1)} | {r['applied_seen']} / {r['commands']} | [{fmt(r['age_lower_ms'], 1)}, {fmt(r['age_upper_ms'], 1)}] | {fmt(r['true_age_ms'], 1)} | {r['true_class']} | {r['contains_truth']} | {r['verdict']} | {r['category']} |")
    md.append("")
    md.append(f"Scored windows {t_summary['scored']} of {t_summary['windows']} ({t_summary['unscored']} unscored: no channel / no command sent). Truth classes: ok {truth_counts['ok']}, violated {truth_counts['violated']}, client-limited {truth_counts['client_limited']}. "
              f"Soundness: {cat_counts['sound']} determinate verdicts agree with the truth, {cat_counts['sound_undetermined']} undetermined, {cat_counts['false_conformant']} false conformant, {cat_counts['false_divergent']} false divergent. "
              f"Completeness: {det_verdict} of {det_truth} windows with a determinate truth received a determinate verdict ({100*t_summary['completeness']:.0f} %). "
              f"Bracket containment: {contain['contained']} of {contain['n']} brackets contain the true age; largest excess of the lower bound over the truth {contain['excess_lower_ms_max']:.2f} ms (the channel's transport delay, which a violated verdict assumes below its margin).")
    if t_wrong:
        md.append("Disagreements: " + "; ".join(t_wrong))
    md.append("")

    # ------------------------------------------------------------------ L liveness (interval containment; eq. (7) against the interval)
    lrows = []
    l_wrong = []
    for prefix in ("L_",):
        for name, r in load(d, prefix):
            tr, res = r["truth"], r["result"]
            L, R = res["observations"]["liveness"], res["observations"]["resolution"]
            tau = L.get("tau_w_estimate_s")
            sv = res["estimates"].get("sub_verdicts", {}).get("stop_behaviour")
            expect = r["expectation"]["temporal"]["stop_behaviour"]
            horizon = r["expectation"]["temporal"].get("horizon_s")
            tau_true = tr.get("tau_w_s")
            status = (tau.get("status") if isinstance(tau, dict) and "status" in tau else ("upper_bound" if isinstance(tau, dict) and "upper_bound_s" in tau else "none"))
            lo = tau.get("interval_low_s") if isinstance(tau, dict) else None
            hi = tau.get("interval_high_s") if isinstance(tau, dict) else None
            contains = (lo <= tau_true <= hi) if (status == "ok" and tau_true) else None
            row = {"run": name, "tau_true_s": tau_true, "mode": tr.get("mode"), "drift_m_s": tr.get("drift_m_s"), "expect": expect, "horizon_s": horizon, "floor_ms": R["resolution_floor_s"] * 1e3,
                   "latency_p95_ms": R["response_latency_p95_s"] * 1e3 if R.get("response_latency_p95_s") is not None else float("nan"),
                   "response_timeout_s": R["response_timeout_s"], "stop_class": L.get("stop_class"), "interval_low_s": lo, "interval_high_s": hi,
                   "width_ms": (hi - lo) * 1e3 if (lo is not None and hi is not None) else float("nan"), "contains_truth": contains,
                   "tau_status": status, "tau_reason": tau.get("reason", "") if isinstance(tau, dict) else "", "tau_upper_bound_s": tau.get("upper_bound_s") if isinstance(tau, dict) else None,
                   "n": tau.get("n") if isinstance(tau, dict) else None, "L_ms": (tau.get("latency_allowance_s") or 0) * 1e3 if isinstance(tau, dict) else float("nan"),
                   "G_ms": (tau.get("granularity_allowance_s") or 0) * 1e3 if isinstance(tau, dict) else float("nan"), "t_detect_ms": (tau.get("detection_delay_s") or 0) * 1e3 if isinstance(tau, dict) else float("nan"),
                   "trials_below_resolution": L.get("trials_below_resolution"), "conditional": tau.get("conditional") if isinstance(tau, dict) else None,
                   "assumptions": " | ".join(tau.get("assumptions", [])) if isinstance(tau, dict) else "", "stop_subverdict": sv, "outcome": res["outcome"], "finding": L.get("finding")}
            lrows.append(row)
            # expected sub-verdict from the injected policy (fault expectation: eq. (7) margin need = 1/f_c + J_max)
            need = tr.get("eq7_need_s", 0.015)
            mode = tr.get("mode")
            tested = 1.5
            h = horizon if horizon is not None else tested  # the claimed horizon defaults to the tested range
            if expect == "fault":
                if mode == "fault" and tau_true and tau_true <= tested:
                    # RC4 review, finding 4: the policy must fire within the claimed horizon AND the client's stream must survive it
                    exp_sv = "satisfied" if (tau_true <= h and need < tau_true) else "violated"
                elif mode == "fault" and tau_true and tau_true > tested:
                    exp_sv = "undetermined" if h > tested else "violated"  # nothing tripped within the tested silences: violated for a horizon inside them, undetermined beyond
                else:
                    exp_sv = "violated"
            elif expect == "hold":
                if horizon is not None and horizon > tested:
                    exp_sv = "undetermined"
                elif mode in ("none",) or (mode == "fault" and tau_true > tested) or (mode == "release" and not tr.get("drift_m_s")):
                    exp_sv = "satisfied"
                else:
                    exp_sv = "violated"
            elif expect == "drift":
                if mode == "release" and tr.get("drift_m_s"):
                    exp_sv = "satisfied" if (tau_true <= h and need < tau_true) else "violated"
                else:
                    exp_sv = "violated"
            elif expect == "release":
                exp_sv = "undetermined"
            else:
                exp_sv = None
            row["expected_subverdict"] = exp_sv
            # 'undetermined' is never wrong (it is the narrower result); the two errors are a wrong definite verdict
            if exp_sv is not None and sv != exp_sv and sv != "undetermined":
                l_wrong.append(f"{name}: {sv}, expected {exp_sv}")
            if sv == "satisfied" and exp_sv == "violated":
                false_conformant.append(name)
            if sv == "violated" and exp_sv == "satisfied":
                false_divergent.append(name + f" (temporal: {sv}, expected {exp_sv})")
    write_csv(os.path.join(tdir, "L_liveness.csv"), lrows)
    n_cond = sum(1 for r in lrows if r["tau_status"] == "ok" and r.get("conditional"))
    n_ok = sum(1 for r in lrows if r["tau_status"] == "ok")
    n_cont = sum(1 for r in lrows if r["contains_truth"])
    md += ["## Liveness / stop behaviour (%d runs; %d intervals with status ok, %d of them contain the injected timeout; %d conditional on a run-maximum latency allowance)" % (len(lrows), n_ok, n_cont, n_cond), "",
           "| run | injected τ_w (s) | mode / drift | expectation | floor (ms) | L (ms) | stop class | interval (s) | width (ms) | contains truth | status | n | sub-verdict (expected) | outcome |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in lrows:
        iv = f"{r['interval_low_s']:.4f}..{r['interval_high_s']:.4f}" if r["interval_low_s"] is not None and r["interval_high_s"] is not None else ("≤ %.4f" % r["tau_upper_bound_s"] if r["tau_upper_bound_s"] else "—")
        md.append(f"| {r['run']} | {r['tau_true_s']} | {r['mode']} / {r['drift_m_s']} | {r['expect']}{' (h=' + str(r['horizon_s']) + ')' if r['horizon_s'] else ''} | {r['floor_ms']:.1f} | {fmt(r['L_ms'], 1)} | {r['stop_class']} | {iv} | {fmt(r['width_ms'], 1)} | {r['contains_truth']} | {r['tau_status']} | {r['n']} | {r['stop_subverdict']} ({r['expected_subverdict']}) | {r['outcome']} |")
    md.append("")
    md.append(f"Stop sub-verdict versus the injected policy: {len(l_wrong)} wrong definite verdict(s)" + (": " + "; ".join(l_wrong) if l_wrong else "") + ".")
    md.append("")

    # ------------------------------------------------------------------ P state
    prows = []
    for name, r in load(d, "P_state_"):
        tr, res, A = r["truth"], r["result"], r["result"]["observations"]["state_precondition"]
        en = A.get("enable") if isinstance(A.get("enable"), dict) else {}
        prows.append({"run": name, "preset": tr.get("preset"), "expect": tr.get("expect"), "state_topic": A.get("operating_state_present"), "executed_when_disabled": A.get("executed_when_disabled"),
                      "executed_when_enabled": A.get("executed_when_enabled"), "executed_without_sm": A.get("executed_without_state_machine"), "enable_latency_ms": en.get("enable_latency_s", float("nan")) * 1e3 if en.get("enable_latency_s") else float("nan"), "outcome": res["outcome"]})
    write_csv(os.path.join(tdir, "P_state.csv"), prows)
    md += ["## State precondition", "", "| run | preset | expectation | state topic | executed disabled / enabled / without SM | enable latency (ms) | outcome |", "|---|---|---|---|---|---|---|"]
    for r in prows:
        md.append(f"| {r['run']} | {r['preset']} | {r['expect']} | {r['state_topic']} | {r['executed_when_disabled']} / {r['executed_when_enabled']} / {r['executed_without_sm']} | {fmt(r['enable_latency_ms'], 1)} | {r['outcome']} |")
        if r["outcome"] == "conformant" and r["expect"] == "discover":
            false_conformant.append(r["run"])
    md.append("")

    # ------------------------------------------------------------------ R + presets
    md += ["## Regression cases (all classes declared)", "", "| case | spatial | dimensional | temporal | notes |", "|---|---|---|---|---|"]
    rrows = []
    for name, r in load(d, "R_"):
        oc = {x["binding_class"]: x["outcome"] for x in r["results"]}
        notes = "; ".join(n[:70] for x in r["results"] for n in x["notes"][:1])
        rrows.append({"case": name, **oc, "notes": notes})
        md.append(f"| {name} | {oc.get('spatial')} | {oc.get('dimensional')} | {oc.get('temporal')} | {notes[:120]} |")
        for x in r["results"]:
            if x["outcome"] == "conformant" and name in ("R_noisy",) and x["binding_class"] == "spatial":
                false_conformant.append(name)
    write_csv(os.path.join(tdir, "R_regression.csv"), rrows)
    md += ["", "## Presets (full reports)", "", "| preset | expectation | spatial | dimensional | temporal | frame ‖t̂‖ / θ̂ | ŝ | stop class | temporal sub-verdicts |", "|---|---|---|---|---|---|---|---|---|"]
    xrows = []
    for name, rep in load(d, "PRESET_"):
        s = rep["summary"]
        fr = next((p for p in rep["probes"] if p["binding_class"] == "spatial"), {})
        sc = next((p for p in rep["probes"] if p["binding_class"] == "dimensional"), {})
        te = next((p for p in rep["probes"] if p["binding_class"] == "temporal"), {})
        fe = fr.get("estimates", {})
        row = {"preset": rep["parameters"]["preset"], "label": rep["parameters"]["label"], **s,
               "t_hat_mm": fe.get("binding_translation_norm_m", {}).get("mean", float("nan")) * 1e3 if fe.get("binding_translation_norm_m") else float("nan"),
               "theta_hat_deg": fe.get("binding_rotation_deg", {}).get("mean", float("nan")) if fe.get("binding_rotation_deg") else float("nan"),
               "s_hat": sc.get("estimates", {}).get("scale_anchored", {}).get("mean"), "stop_class": te.get("estimates", {}).get("stop_class"),
               "temporal_sub": json.dumps(te.get("estimates", {}).get("sub_verdicts"))}
        xrows.append(row)
        md.append(f"| {row['preset']} | {row['label']} | {s['spatial']} | {s['dimensional']} | {s['temporal']} | {fmt(row['t_hat_mm'], 1)} / {fmt(row['theta_hat_deg'], 1)} | {fmt(row['s_hat'], 4)} | {row['stop_class']} | {row['temporal_sub']} |")
        if row["label"] == "discover" and "conformant" in s.values():
            false_conformant.append(name)
        if row["label"] == "authored":
            # the authored expectation declares SI units and (for the reference/JHU presets) the preset's own binding and
            # stop policy: a dimensional 'divergent' on a preset whose unit is 1 m is a false divergent (second campaign,
            # emul-ambf-object-watchdog: s_hat 1.91 from release drift inside the silent settle window)
            from crtk_mock.presets import get as _get_preset
            cfg = _get_preset(rep["parameters"]["preset"])
            expect_dim = "divergent" if abs(cfg.unit_m - 1.0) > 1e-9 else "conformant"
            if s["dimensional"] not in (expect_dim, "undetermined"):
                false_divergent.append(name + f" (dimensional {s['dimensional']}, unit_m {cfg.unit_m})")
    write_csv(os.path.join(tdir, "PRESETS.csv"), xrows)
    md.append("")

    # ------------------------------------------------------------------ M
    mrows = []
    mm = load(d, "M_model_check")
    if mm:
        for c in mm[0][1]["cases"]:
            x = c.get("p_c_norm_m", c.get("step_m"))
            mrows.append({"kind": c["kind"], "x_mm": x * 1e3, "measured_mm": c["measured_error_m"] * 1e3, "predicted_mm": c["predicted_error_m"] * 1e3,
                          "bound_mm": c.get("upper_bound_m", float("nan")) * 1e3 if c.get("upper_bound_m") is not None else float("nan"),
                          "exact_max_ball_mm": c.get("exact_max_over_ball_m", float("nan")) * 1e3 if c.get("exact_max_over_ball_m") is not None else float("nan"),
                          "abs_diff_mm": abs(c["measured_error_m"] - c["predicted_error_m"]) * 1e3})
        write_csv(os.path.join(tdir, "M_model_check.csv"), mrows)
        md += ["## Model check (mock executed error vs model prediction with T = base_frame⁻¹; implementation verification)", "", "| case | ‖p_c‖ or step (mm) | executed error (mm) | predicted (mm) | bound (3) (mm) | exact max over the ball (3') (mm) | |diff| (mm) |", "|---|---|---|---|---|---|---|"]
        for r in mrows:
            md.append(f"| {r['kind']} | {r['x_mm']:.1f} | {r['measured_mm']:.2f} | {r['predicted_mm']:.2f} | {fmt(r['bound_mm'], 2)} | {fmt(r['exact_max_ball_mm'], 2)} | {r['abs_diff_mm']:.3f} |")
        md.append(f"\nMax |executed − predicted| = {max(r['abs_diff_mm'] for r in mrows):.3f} mm over {len(mrows)} points.")
        md.append("")

    md += [f"## False conformant (general definition): {len(false_conformant)} run(s)" + (": " + ", ".join(false_conformant) if false_conformant else ""), "",
           f"## False divergent (presets with authored expectations, unit known): {len(false_divergent)} run(s)" + (": " + ", ".join(false_divergent) if false_divergent else ""), ""]

    # ------------------------------------------------------------------ live
    for ld in args.live:
        tag = os.path.basename(ld.rstrip("/"))
        md += [f"## Live: {tag}", ""]
        aux_p = os.path.join(ld, "live_aux.json")
        if os.path.exists(aux_p):
            a = json.load(open(aux_p))
            pres = a["presence"]
            md.append("Topics: " + ", ".join(f"{k}={'yes' if v else 'no'}" for k, v in pres.items()) + f"; /tf {'yes' if a.get('tf_present') else 'no'}.")
            mc = a["measured_cp"]
            md.append(f"measured_cp: frame_id `{mc['frame_id']}`, publish rate {mc['rate_hz']:.1f} Hz (inter-arrival mean {mc['interarrival_s']['mean']*1e3:.2f} ms, max {mc['interarrival_s']['max']*1e3:.1f} ms).")
            if "T_b_w" in a:
                md.append(f"T_b_w: frame_id `{a['T_b_w']['frame_id']}`, translation {np.round(a['T_b_w']['pose'][:3], 4).tolist()} (interface units).")
            for k, v in a.get("joint_configs", {}).items():
                md.append(f"joint config {k}: commanded q {v['commanded_q']}; measured_js {np.round(v.get('measured_js_mean', []), 4).tolist()}; reported − simulated joints {np.round(v.get('reported_minus_simulated_joint', []), 4).tolist()}; ‖measured_cp‖ = {v.get('measured_cp_position_norm', float('nan')):.5f} interface units.")
            if "servo_cp_step" in a:
                s = a["servo_cp_step"]
                md.append(f"servo_cp step of {s['commanded_step_if_units']} (x): residual ‖measured − goal‖ = {s['residual_norm_if_units']:.4f} interface units after settling; measured std {np.round(s['measured_std'], 5).tolist()}.")
            md.append("")
        for rep_name in ("report_discover_only", "report_src_client", "report_dvrk_client"):
            p = os.path.join(ld, rep_name + ".json")
            if not os.path.exists(p):
                continue
            rep = json.load(open(p))
            md.append(f"**{rep_name}** (expectations: {rep['expectations']['source']}): summary {rep['summary']}")
            for pr in rep["probes"]:
                md.append(f"- {pr['binding_class']}: {pr['outcome']} — {pr['decision_basis'][:160]}")
                if pr["binding_class"] == "temporal":
                    o = pr["observations"]
                    R = o["resolution"]
                    md.append(f"  - resolution floor {R['resolution_floor_s']*1e3:.1f} ms, feedback period {R['feedback_period_s']*1e3:.2f} ms, resting noise σ̂ {R['resting_noise_sigma_m']:.2e} (interface units), response timeout {R['response_timeout_s']:.2f} s; state topic {o['state_precondition'].get('operating_state_present')}, executed without state machine {o['state_precondition'].get('executed_without_state_machine')} (attained {o['state_precondition'].get('attained_without_state_machine')})")
                    md.append(f"  - liveness: {o['liveness'].get('finding')}")
                    tau = o["liveness"].get("tau_w_estimate_s")
                    if isinstance(tau, dict):
                        md.append(f"  - tau_w estimate: {json.dumps({k: v for k, v in tau.items() if k not in ('interval_semantics', 'brackets_s')})}")
                    for row in o["effective_rate"]["per_rate"]:
                        acc = row.get("acceptance") or {}
                        md.append(f"  - rate {row['command_rate_requested_hz']:.0f} Hz: accepted-command channel {acc.get('channel')} ({acc.get('status')}{', ' + acc['reason'] if acc.get('reason') else ''}); diagnostic feedback crossings {row['transitions']}/{row['commands_sent']} ({row['observable_rate_hz']:.1f} Hz), client {row['client_rate_achieved_hz']:.1f} Hz, publish {row['publish_rate_hz']:.1f} Hz, unmatched {row['unmatched_fraction']:.2f}, δ {row['match_tolerance_m']:.4g}, step {(row.get('step_used_m') if row.get('step_used_m') is not None else float('nan')):.4g}, {row['status']} ({row.get('reason') or row['observation_bounded_by']})")
                if pr["binding_class"] == "dimensional":
                    o = pr["observations"]
                    e = pr["estimates"]
                    md.append(f"  - internal ratio {e.get('internal_ratio', {}).get('mean', float('nan')):.4f} (95 % CI {e.get('internal_ratio', {}).get('ci_low', float('nan')):.3f}..{e.get('internal_ratio', {}).get('ci_high', float('nan')):.3f}, n = {e.get('internal_ratio', {}).get('n')}), step used {o.get('step_used_if')}, resting σ̂ {o.get('resting_noise_sigma_if')}, no-response trials {o.get('no_response_trials')}, latency {e.get('response_latency_s', {}).get('mean', float('nan'))*1e3:.1f} ms")
                if pr["binding_class"] == "spatial":
                    md.append(f"  - frame_ids {pr['observations'].get('frame_ids')}; T_b_w {pr['estimates'].get('T_b_w_translation_m')} / {pr['estimates'].get('T_b_w_rotation_deg')}; assumptions {pr['observations'].get('assumptions')}")
            md.append("")

    with open(os.path.join(tdir, "SUMMARY.md"), "w") as f:
        f.write("\n".join(md))
    print("\n".join(md))

    # ------------------------------------------------------------------ figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.5, "legend.frameon": False})
    fig, axs = plt.subplots(2, 2, figsize=(6.8, 5.2))
    markers = {0.0: "o", 0.02: "s", 0.1: "^"}
    cols = {0.0: C_BLUE, 0.02: C_ORANGE, 0.1: C_GREEN}
    ax = axs[0, 0]
    for noise in sorted({r["noise_mm"] for r in frows}):
        sub = [r for r in frows if r["noise_mm"] == noise and r["theta_true_deg"] == 0]
        ax.plot([max(r["t_true_mm"], 0.03) for r in sub], [max(r["t_err_mm"], 1e-5) for r in sub], marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=1, ms=4, label=f"noise {noise:g} mm")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected ‖t‖ (mm)  [θ = 0]"); ax.set_ylabel("‖t̂ − t‖ (mm)  (floor 1e-5)"); ax.set_title("(a) frame: translation estimate", loc="left"); ax.legend(fontsize=7)
    ax = axs[0, 1]
    for noise in sorted({r["noise_mm"] for r in srows}):
        sub = sorted([r for r in srows if r["noise_mm"] == noise], key=lambda r: r["s_true"])
        ax.plot([r["s_true"] for r in sub], [r["s_hat_mean"] for r in sub], marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=1, ms=4, label=f"ŝ (anchored), noise {noise:g} mm")
        ax.plot([r["s_true"] for r in sub], [r["r_int_mean"] for r in sub], marker=markers.get(noise, "o"), mfc="none", color=cols.get(noise, C_GREY), lw=1, ls="--", ms=4)
    ax.plot([0.1, 10], [0.1, 10], color=C_GREY, lw=0.8, ls=":")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected scale s"); ax.set_ylabel("estimate"); ax.set_title("(b) scale: ŝ (solid), internal ratio (dashed)", loc="left", fontsize=7.5); ax.legend(fontsize=6.5, loc="upper left")
    ax = axs[1, 0]
    ok = [r for r in lrows if r["tau_status"] == "ok" and r["mode"] == "fault" and r["expect"] == "fault"]
    if ok:
        mid = [0.5 * (r["interval_low_s"] + r["interval_high_s"]) for r in ok]
        ax.errorbar([r["tau_true_s"] for r in ok], mid, yerr=[[m - r["interval_low_s"] for m, r in zip(mid, ok)], [r["interval_high_s"] - m for m, r in zip(mid, ok)]], fmt="o", color=C_BLUE, ms=3.5, capsize=2, lw=1, label="fault: τ_w interval (midpoint marked)")
    ub = [r for r in lrows if r["tau_status"] == "upper_bound" and r["mode"] == "fault"]
    if ub:
        ax.plot([r["tau_true_s"] for r in ub], [r["tau_upper_bound_s"] for r in ub], "v", color=C_PINK, ms=5, label="below resolution: upper bound")
    und = [r for r in lrows if r["tau_status"] in ("undetermined", "inconsistent") and r["mode"] == "fault" and r["interval_low_s"] is not None]
    if und:
        ax.plot([r["tau_true_s"] for r in und], [r["interval_high_s"] for r in und], "x", color=C_GREY, ms=5, label="undetermined (upper end shown)")
    rel = [r for r in lrows if r["tau_status"] == "ok" and r["mode"] == "release" and r["expect"] == "drift"]
    if rel:
        mid = [0.5 * (r["interval_low_s"] + r["interval_high_s"]) for r in rel]
        ax.errorbar([r["tau_true_s"] for r in rel], mid, yerr=[[m - r["interval_low_s"] for m, r in zip(mid, rel)], [r["interval_high_s"] - m for m, r in zip(mid, rel)]], fmt="s", color=C_ORANGE, ms=3.5, capsize=2, lw=1, label="release with drift: τ_w interval")
    if lrows:
        fl = float(np.median([r["floor_ms"] for r in lrows])) / 1e3
        ax.axhline(fl, color=C_GREY, lw=0.8, ls="--", label=f"measured resolution floor ≈ {fl*1e3:.0f} ms")
    ax.plot([0.01, 1.2], [0.01, 1.2], color=C_GREY, lw=0.8, ls=":")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected τ_w (s)"); ax.set_ylabel("τ_w interval (s)"); ax.set_title("(c) liveness timeout interval", loc="left"); ax.legend(fontsize=5.5, loc="upper left")
    ax = axs[1, 1]
    ax.plot([r["eps_mm"] for r in fsweep], [r["FNR"] for r in fsweep], color=C_BLUE, lw=1.5, label="frame false-conformant (FNR)")
    ax.plot([r["eps_mm"] for r in fsweep], [r["FPR"] for r in fsweep], color=C_BLUE, lw=1.5, ls="--", label="frame false-divergent (FPR)")
    ax.plot([r["eps_mm"] for r in fsweep], [r["undetermined_rate"] for r in fsweep], color=C_BLUE, lw=1, ls=":", label="frame undetermined")
    ax.plot([r["eps_mm"] for r in ssweep], [r["FNR"] for r in ssweep], color=C_ORANGE, lw=1.5, label="scale false-conformant (FNR)")
    ax.plot([r["eps_mm"] for r in ssweep], [r["FPR"] for r in ssweep], color=C_ORANGE, lw=1.5, ls="--", label="scale false-divergent (FPR)")
    ax.plot([r["eps_mm"] for r in ssweep], [r["undetermined_rate"] for r in ssweep], color=C_ORANGE, lw=1, ls=":", label="scale undetermined")
    ax.set_xscale("log"); ax.set_xlabel("tolerance ε (mm)"); ax.set_ylabel("rate"); ax.set_ylim(-0.02, 1.02); ax.set_title("(d) decision rates vs. tolerance", loc="left"); ax.legend(fontsize=5.5, ncol=2, loc="upper center")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "fig05_validation.pdf")); fig.savefig(os.path.join(figdir, "fig05_validation.png"), dpi=200)

    # rate: bracket versus event-log truth (RC4 review, finding 5)
    fig, ax = plt.subplots(figsize=(3.4, 2.9))
    sc = [r for r in trows if r["true_class"] != "unscored" and not math.isnan(r["age_upper_ms"])]
    cols_v = {"satisfied": C_GREEN, "violated": C_ORANGE, "undetermined": C_GREY}
    for v, c in cols_v.items():
        sub = [r for r in sc if r["verdict"] == v]
        if not sub:
            continue
        x = np.array([max(r["true_age_ms"], 1.0) for r in sub]); lo = np.array([max(r["age_lower_ms"], 1.0) for r in sub]); hi = np.array([max(r["age_upper_ms"], 1.0) for r in sub])
        ax.errorbar(x, np.sqrt(lo * hi), yerr=[np.sqrt(lo * hi) - lo, hi - np.sqrt(lo * hi)], fmt="o", color=c, ms=2.5, capsize=1.5, lw=0.7, elinewidth=0.7, label=f"{v} (n={len(sub)})")
    ax.plot([1, 3000], [1, 3000], color=C_GREY, lw=0.8, ls=":")
    ax.axvline(1e3 / f_req, color=C_GREY, lw=0.8, ls="--"); ax.axhline(1e3 / f_req, color=C_GREY, lw=0.8, ls="--")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("true source age of the applied setpoint (ms), event log"); ax.set_ylabel("estimated bracket [lower, upper] (ms)")
    ax.set_title("rate: bracket vs. truth (dashed: required period)", loc="left", fontsize=7.5); ax.legend(fontsize=6, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "fig06_rate_truth.pdf")); fig.savefig(os.path.join(figdir, "fig06_rate_truth.png"), dpi=200)

    # error-model figure: model with T = base_frame^-1 (JHU), mock points from M
    fig, axs = plt.subplots(1, 3, figsize=(6.8, 2.3))
    T_base = G.make_pose(G.axis_angle([1, 0, 0], math.radians(-150.0)), [0.20, 0, 0])
    Tm = G.invert(T_base); R, t = Tm[:3, :3], Tm[:3, 3]
    ps = np.linspace(0, 0.10, 101)
    ax = axs[0]
    ax.plot(ps * 1e3, [G.m1_positional_error(R, t, np.array([0, 0, p])) * 1e3 for p in ps], color=C_BLUE, lw=1.5, label="absolute, p⊥axis (eq. 1)")
    ax.plot(ps * 1e3, [G.m1_positional_error(R, t, np.array([p, 0, 0])) * 1e3 for p in ps], color=C_BLUE, lw=1.5, ls="--", label="absolute, p∥axis")
    ax.plot(ps * 1e3, [G.m1_upper_bound(R, t, p) * 1e3 for p in ps], color=C_GREY, lw=1, ls=":", label="bound (3)")
    from crtk_conformance.spatial import exact_max_error as _emax
    ax.plot(ps * 1e3, [_emax(R, t, p) * 1e3 for p in ps], color=C_GREEN, lw=1.2, ls="-.", label="exact max over ‖p‖ ≤ r (3')")
    ax.plot(ps * 1e3, [G.m1_incremental_bound(R, p) * 1e3 for p in ps], color=C_ORANGE, lw=1.5, label="incremental (5), step = ‖p‖")
    ax.plot(ps * 1e3, [0 for p in ps], color=C_ORANGE, lw=1.5, ls="--", label="incremental, R = I")
    for m in mrows:
        if m["kind"].startswith("absolute"):
            ax.plot(m["x_mm"], m["measured_mm"], "o", color=C_BLUE, ms=4, mfc="white")
        if m["kind"].startswith("incremental_perp"):
            ax.plot(m["x_mm"], m["measured_mm"], "o", color=C_ORANGE, ms=4, mfc="white")
    ax.set_xlabel("‖p_c‖ or step (mm)"); ax.set_ylabel("positional error (mm)"); ax.set_title("(a) M1, T = base_frame⁻¹ (JHU)", loc="left"); ax.set_ylim(-10, 470); ax.legend(fontsize=5, loc="upper left")
    ax = axs[1]
    for s, c in ((0.1, C_BLUE), (0.5, C_ORANGE), (1.1, C_GREEN)):
        ax.plot(ps * 1e3, [G.m2_error(s, p) * 1e3 for p in ps], color=c, lw=1.5, label=f"s = {s}")
    for m in mrows:
        if m["kind"] == "scale_absolute":
            ax.plot(m["x_mm"], m["measured_mm"], "o", color=C_BLUE, ms=4, mfc="white")
    ax.set_xlabel("distance from origin (mm)"); ax.set_ylabel("error (mm)"); ax.set_title("(b) M2, |1−s|‖p‖", loc="left"); ax.legend(fontsize=6)
    ax = axs[2]
    fs = np.logspace(1, 3, 100)
    for v, c in ((0.02, C_GREEN), (0.05, C_BLUE), (0.1, C_ORANGE)):
        ax.plot(fs, v / fs * 1e3, color=c, lw=1.5, label=f"v = {v*1e3:.0f} mm/s")
    ax.axhline(1.0, color=C_GREY, lw=0.8, ls=":"); ax.axhline(0.5, color=C_GREY, lw=0.8, ls=":")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("command rate f (Hz)"); ax.set_ylabel("ZOH lag bound v/f (mm)"); ax.set_title("(c) M3, eq. (9)", loc="left"); ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "fig03_error_model.pdf")); fig.savefig(os.path.join(figdir, "fig03_error_model.png"), dpi=200)
    print("figures written to", figdir)


if __name__ == "__main__":
    main()
