#!/usr/bin/env python3
"""Offline analysis of validation/results: tables (CSV + Markdown) and figures (PDF).

Usage: python validation/analyze.py validation/results [--figdir figures]
Nothing here talks to ROS; everything is computed from the archived JSON.
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

# Okabe-Ito subset (validated colour-blind-safe); identity also carried by marker shape / line style.
C_BLUE, C_ORANGE, C_GREEN, C_PINK, C_GREY = "#0072B2", "#E69F00", "#009E73", "#CC79A7", "#6e6e6e"


def load(d, prefix):
    out = []
    for f in sorted(glob.glob(os.path.join(d, prefix + "*.json"))):
        with open(f) as fh:
            out.append((os.path.basename(f)[:-5], json.load(fh)))
    return out


def decide(ci_low, ci_high, eps):
    if ci_low is None or ci_high is None:
        return "undetermined"
    if ci_high <= eps:
        return "conformant"
    if ci_low > eps:
        return "divergent"
    return "undetermined"


def sweep(records, eps_grid, ci_key):
    """records: list of (truth_model_error_m, ci_low, ci_high). Returns FPR, FNR, undetermined-rate per eps."""
    rows = []
    for eps in eps_grid:
        pos = neg = fp = fn = und = 0
        for truth, lo, hi in records:
            label_pos = truth > eps
            d = decide(lo, hi, eps)
            if label_pos:
                pos += 1
                if d == "conformant":
                    fn += 1
            else:
                neg += 1
                if d == "divergent":
                    fp += 1
            if d == "undetermined":
                und += 1
        rows.append({"eps_mm": eps * 1e3, "positives": pos, "negatives": neg, "FP": fp, "FN": fn, "undetermined": und,
                     "FPR": fp / neg if neg else float("nan"), "FNR": fn / pos if pos else float("nan"), "undetermined_rate": und / len(records)})
    return rows


def write_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("--figdir", default=None)
    args = ap.parse_args()
    d = args.results
    tabdir = os.path.join(d, "tables"); os.makedirs(tabdir, exist_ok=True)
    figdir = args.figdir or os.path.join(d, "figures"); os.makedirs(figdir, exist_ok=True)
    md = []

    # ------------------------------------------------------------------ F
    F = load(d, "F_")
    frows = []
    frec = []
    for name, r in F:
        tr = r["truth"]; est = r["result"]["estimates"]
        t_hat = np.array(est.get("binding_translation_m", [np.nan] * 3)); t_true = np.array(tr["t_m"])
        th_hat = est.get("binding_rotation_deg", {}).get("mean", np.nan)
        pe = est.get("predicted_abs_error_at_workspace_edge_m", {})
        frows.append({"run": name, "t_true_mm": tr["t_norm_m"] * 1e3, "theta_true_deg": tr["theta_deg"], "noise_mm": tr["noise_m"] * 1e3,
                      "t_err_mm": float(np.linalg.norm(t_hat - t_true)) * 1e3, "theta_err_deg": abs(th_hat - tr["theta_deg"]),
                      "t_hat_std_mm": est.get("binding_translation_norm_m", {}).get("std", np.nan) * 1e3,
                      "model_error_mm": tr["model_error_m"] * 1e3, "pred_mm": pe.get("mean", np.nan) * 1e3,
                      "pred_ci_low_mm": pe.get("ci_low", np.nan) * 1e3, "pred_ci_high_mm": pe.get("ci_high", np.nan) * 1e3,
                      "outcome": r["result"]["outcome"], "n": est.get("binding_translation_norm_m", {}).get("n", 0), "probe_s": r["result"]["duration_s"]})
        frec.append((tr["model_error_m"], pe.get("ci_low"), pe.get("ci_high")))
    write_csv(os.path.join(tabdir, "F_runs.csv"), frows)
    fsum = []
    for noise in sorted({r["noise_mm"] for r in frows}):
        sub = [r for r in frows if r["noise_mm"] == noise]
        fsum.append({"noise_mm": noise, "runs": len(sub), "t_err_mean_mm": np.mean([r["t_err_mm"] for r in sub]), "t_err_max_mm": np.max([r["t_err_mm"] for r in sub]),
                     "theta_err_mean_deg": np.mean([r["theta_err_deg"] for r in sub]), "theta_err_max_deg": np.max([r["theta_err_deg"] for r in sub]),
                     "t_err_mean_mm_at_t0": np.mean([r["t_err_mm"] for r in sub if r["t_true_mm"] == 0]) if any(r["t_true_mm"] == 0 for r in sub) else np.nan,
                     "FP_at_1mm": sum(1 for r in sub if r["model_error_mm"] <= 1.0 and r["outcome"] == "divergent"),
                     "FN_at_1mm": sum(1 for r in sub if r["model_error_mm"] > 1.0 and r["outcome"] == "conformant"),
                     "undetermined_at_1mm": sum(1 for r in sub if r["outcome"] == "undetermined"), "n_trials": sub[0]["n"]})
    write_csv(os.path.join(tabdir, "F_summary.csv"), fsum)
    eps_grid = np.logspace(math.log10(0.05e-3), math.log10(50e-3), 61)
    fsweep = sweep(frec, eps_grid, None)
    write_csv(os.path.join(tabdir, "F_threshold_sweep.csv"), fsweep)
    md.append("## F — frame probe\n")
    md.append("| noise (mm) | runs | mean ‖t̂−t‖ (mm) | max | mean |θ̂−θ| (deg) | max | mean ‖t̂‖ at t=0 (mm) | FP@1mm | FN@1mm | undet.@1mm | n |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in fsum:
        md.append(f"| {r['noise_mm']:.2f} | {r['runs']} | {r['t_err_mean_mm']:.4f} | {r['t_err_max_mm']:.4f} | {r['theta_err_mean_deg']:.4f} | {r['theta_err_max_deg']:.4f} | {r['t_err_mean_mm_at_t0']:.4f} | {r['FP_at_1mm']} | {r['FN_at_1mm']} | {r['undetermined_at_1mm']} | {r['n_trials']} |")

    # ------------------------------------------------------------------ S
    S = [x for x in load(d, "S_") if not x[0].endswith("noanchor")]
    srows = []
    srec = []
    for name, r in S:
        tr = r["truth"]; est = r["result"]["estimates"]
        ri = est.get("internal_ratio", {}); sa = est.get("scale_anchored", {}); pe = est.get("predicted_error_at_workspace_edge_m", {})
        srows.append({"run": name, "s_true": tr["s"], "noise_mm": tr["noise_m"] * 1e3, "r_int_mean": ri.get("mean"), "r_int_std": ri.get("std"),
                      "s_hat_mean": sa.get("mean"), "s_hat_std": sa.get("std"), "s_err": abs(sa.get("mean", np.nan) - tr["s"]),
                      "s_rel_err": abs(sa.get("mean", np.nan) - tr["s"]) / tr["s"], "model_error_mm": tr["model_error_m"] * 1e3,
                      "pred_ci_low_mm": pe.get("ci_low", np.nan) * 1e3, "pred_ci_high_mm": pe.get("ci_high", np.nan) * 1e3,
                      "outcome": r["result"]["outcome"], "n": ri.get("n"), "latency_ms": est.get("response_latency_s", {}).get("mean", np.nan) * 1e3, "probe_s": r["result"]["duration_s"]})
        srec.append((tr["model_error_m"], pe.get("ci_low"), pe.get("ci_high")))
    write_csv(os.path.join(tabdir, "S_runs.csv"), srows)
    ssweep = sweep(srec, eps_grid, None)
    write_csv(os.path.join(tabdir, "S_threshold_sweep.csv"), ssweep)
    noanchor = load(d, "S_noanchor")
    md.append("\n## S — scale probe\n")
    md.append("| s | noise (mm) | r_int (mean ± std) | ŝ (mean ± std) | |ŝ−s|/s | outcome@1mm | latency (ms) |")
    md.append("|---|---|---|---|---|---|---|")
    for r in srows:
        md.append(f"| {r['s_true']} | {r['noise_mm']:.2f} | {r['r_int_mean']:.4f} ± {r['r_int_std']:.4f} | {r['s_hat_mean']:.4f} ± {r['s_hat_std']:.4f} | {r['s_rel_err']:.2e} | {r['outcome']} | {r['latency_ms']:.1f} |")
    all_rint = [r["r_int_mean"] for r in srows]
    md.append(f"\nInternal ratio over all {len(srows)} runs (s from 0.1 to 10): mean {np.mean(all_rint):.4f}, min {np.min(all_rint):.4f}, max {np.max(all_rint):.4f} — feedback invisibility.")
    if noanchor:
        na = noanchor[0][1]["result"]
        md.append(f"No-anchor control (s = 0.1): outcome **{na['outcome']}**, r_int = {na['estimates']['internal_ratio']['mean']:.4f}.")

    # ------------------------------------------------------------------ T
    T = load(d, "T_live_")
    trows = []
    for name, r in T:
        live = r["result"]["observations"]["liveness"]; tau = live.get("tau_w_estimate_s")
        trows.append({"run": name, "tau_true_s": r["truth"]["tau_w_s"], "tau_hat_s": tau["mean"] if tau else None, "tau_hat_std_s": tau["std"] if tau else None,
                      "tau_ci_low": tau["ci_low"] if tau else None, "tau_ci_high": tau["ci_high"] if tau else None, "n": tau["n"] if tau else live["executed_after_gap_max"]["total"],
                      "finding": live["finding"], "outcome": r["result"]["outcome"], "probe_s": r["result"]["duration_s"], "publish_rate_hz": r["result"]["observations"]["effective_rate"]["publish_rate_hz"]})
    write_csv(os.path.join(tabdir, "T_liveness.csv"), trows)
    md.append("\n## T — temporal probe: liveness\n")
    md.append("| τ_w injected (s) | τ̂_w (s) | std | 95 % CI | n | finding |")
    md.append("|---|---|---|---|---|---|")
    for r in trows:
        if r["tau_hat_s"] is None:
            md.append(f"| {r['tau_true_s']} | — | — | — | {r['n']} | {r['finding']} |")
        else:
            md.append(f"| {r['tau_true_s']} | {r['tau_hat_s']:.3f} | {r['tau_hat_std_s']:.4f} | {r['tau_ci_low']:.3f}..{r['tau_ci_high']:.3f} | {r['n']} | liveness policy detected |")
    rel = load(d, "T_release")
    if rel:
        live = rel[0][1]["result"]["observations"]["liveness"]
        md.append(f"\nReleased-drift emulation (τ_w = 0.5 s, release, 20 mm/s drift): {live['finding']}; drift during 1.5 s gap = {live['drift_during_gap_max_m']['mean']*1e3:.1f} mm (n={live['drift_during_gap_max_m']['n']}).")
    R_ = load(d, "T_rate_")
    md.append("\n## T — effective command rate\n")
    md.append("| loop (Hz) | publish (Hz) | measured publish (Hz) | effective rate at client 50/100/200/500/1000 Hz |")
    md.append("|---|---|---|---|")
    rrows = []
    for name, r in R_:
        er = r["result"]["observations"]["effective_rate"]
        eff = {x["command_rate_hz"]: x["effective_rate_hz"] for x in er["per_rate"]}
        rrows.append({"loop_hz": r["truth"]["loop_rate_hz"], "publish_hz": r["truth"]["publish_rate_hz"], "publish_measured_hz": er["publish_rate_hz"], **{f"eff_{k}": v for k, v in eff.items()}})
        md.append(f"| {r['truth']['loop_rate_hz']} | {r['truth']['publish_rate_hz']} | {er['publish_rate_hz']:.1f} | " + " / ".join(f"{eff[k]:.0f}" for k in sorted(eff)) + " |")
    write_csv(os.path.join(tabdir, "T_effective_rate.csv"), rrows)
    St = load(d, "T_state_")
    md.append("\n## T — state precondition on presets\n")
    md.append("| preset | operating_state topic | executed when DISABLED | executed after enable+home | enable latency (ms) | executed without state machine |")
    md.append("|---|---|---|---|---|---|")
    for name, r in St:
        sp = r["result"]["observations"]["state_precondition"]
        lat = sp.get("enable", {}).get("enable_latency_s")
        md.append(f"| {r['truth']['preset']} | {sp['operating_state_present']} | {sp.get('executed_when_disabled', '—')} | {sp.get('executed_when_enabled', '—')} | {lat*1e3 if lat else '—'} | {sp.get('executed_without_state_machine', '—')} |")

    # ------------------------------------------------------------------ R
    Rb = load(d, "R_")
    md.append("\n## R — robustness\n")
    md.append("| case | spatial | dimensional | temporal | false 'conformant'? |")
    md.append("|---|---|---|---|---|")
    rob = []
    for name, r in Rb:
        o = {x["binding_class"]: x["outcome"] for x in r["results"]}
        case = r["case"]
        # a false conformant is 'conformant' where the injected condition makes the class unverifiable or divergent
        bad = (case == "missing_measured_cp" and any(v == "conformant" for v in o.values())) or (case == "no_local_topic" and o["spatial"] == "conformant") or (case == "divergent_but_noisy" and o["spatial"] == "conformant")
        rob.append({"case": case, **o, "false_conformant": bad})
        md.append(f"| {case} | {o['spatial']} | {o['dimensional']} | {o['temporal']} | {'YES' if bad else 'no'} |")
    write_csv(os.path.join(tabdir, "R_robustness.csv"), rob)

    # ------------------------------------------------------------------ P
    P = load(d, "P_")
    md.append("\n## P — presets (full run)\n")
    md.append("| preset | spatial | dimensional | temporal | key estimates |")
    md.append("|---|---|---|---|---|")
    for name, rep in P:
        s = rep["summary"]; ests = []
        for p in rep["probes"]:
            e = p["estimates"]
            if p["probe"] == "FrameSemanticsProbe" and "binding_translation_norm_m" in e:
                ests.append(f"‖t̂‖={e['binding_translation_norm_m']['mean']*1e3:.1f} mm, θ̂={e['binding_rotation_deg']['mean']:.1f}°")
            if p["probe"] == "ScalingUnitsProbe" and "scale_anchored" in e:
                ests.append(f"ŝ={e['scale_anchored']['mean']:.3f}")
            if p["probe"] == "RateSensitivityProbe":
                ests.append(f"eff@100Hz={e.get('effective_rate_at_client_rate_hz', float('nan')):.0f} Hz")
        md.append(f"| {name[2:]} | {s['spatial']} | {s['dimensional']} | {s['temporal']} | {'; '.join(ests)} |")

    # ------------------------------------------------------------------ M
    M = load(d, "M_model_check")
    mrows = []
    if M:
        md.append("\n## M — model check (mock execution vs. error model)\n")
        md.append("| case | ‖p_c‖ or step (mm) | executed error (mm) | model prediction (mm) | bound (mm) |")
        md.append("|---|---|---|---|---|")
        for c in M[0][1]["cases"]:
            x = c.get("p_c_norm_m", c.get("step_m")) * 1e3
            mrows.append({"kind": c["kind"], "x_mm": x, "measured_mm": c["measured_error_m"] * 1e3, "predicted_mm": c["predicted_error_m"] * 1e3, "bound_mm": c.get("upper_bound_m", float("nan")) * 1e3})
            md.append(f"| {c['kind']} | {x:.1f} | {c['measured_error_m']*1e3:.2f} | {c['predicted_error_m']*1e3:.2f} | {c.get('upper_bound_m', float('nan'))*1e3:.2f} |")
        write_csv(os.path.join(tabdir, "M_model_check.csv"), mrows)

    meta = json.load(open(os.path.join(d, "meta.json"))) if os.path.exists(os.path.join(d, "meta.json")) else {}
    md.insert(0, f"# Validation results — crtk-conformance {meta.get('crtk_conformance_version','?')} @ {meta.get('repo_commit','?')[:12]}, {meta.get('started_at','?')}, wall {meta.get('wall_s',0)/60:.1f} min, {meta.get('platform','?')}\n")
    with open(os.path.join(tabdir, "SUMMARY.md"), "w") as f:
        f.write("\n".join(md) + "\n")
    print("\n".join(md))

    # ------------------------------------------------------------------ figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.5, "legend.frameon": False})
    fig, axs = plt.subplots(2, 2, figsize=(6.8, 5.2))
    # (a) frame estimation error vs injected translation, by noise
    ax = axs[0, 0]
    markers = {0.0: "o", 0.02: "s", 0.1: "^"}
    cols = {0.0: C_BLUE, 0.02: C_ORANGE, 0.1: C_GREEN}
    for noise in sorted({r["noise_mm"] for r in frows}):
        sub = [r for r in frows if r["noise_mm"] == noise and r["theta_true_deg"] == 0]
        xs = [max(r["t_true_mm"], 0.03) for r in sub]; ys = [max(r["t_err_mm"], 1e-5) for r in sub]
        ax.plot(xs, ys, marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=1, ms=4, label=f"noise {noise:g} mm")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected ‖t‖ (mm)  [θ = 0]"); ax.set_ylabel("‖t̂ − t‖ (mm)  (floor 1e-5)"); ax.set_title("(a) frame: translation estimate", loc="left")
    ax.legend(fontsize=7)
    # (b) scale
    ax = axs[0, 1]
    for noise in sorted({r["noise_mm"] for r in srows}):
        sub = sorted([r for r in srows if r["noise_mm"] == noise], key=lambda r: r["s_true"])
        ax.plot([r["s_true"] for r in sub], [r["s_hat_mean"] for r in sub], marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=1, ms=4, label=f"ŝ (anchored), noise {noise:g} mm")
        ax.plot([r["s_true"] for r in sub], [r["r_int_mean"] for r in sub], marker=markers.get(noise, "o"), mfc="none", color=cols.get(noise, C_GREY), lw=1, ls="--", ms=4)
    ax.plot([0.1, 10], [0.1, 10], color=C_GREY, lw=0.8, ls=":")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected scale s"); ax.set_ylabel("estimate"); ax.set_title("(b) scale: anchored ŝ (solid), internal ratio (dashed)", loc="left", fontsize=7.5)
    ax.legend(fontsize=6.5)
    # (c) liveness
    ax = axs[1, 0]
    tt = [r for r in trows if r["tau_hat_s"] is not None]
    ax.errorbar([r["tau_true_s"] for r in tt], [r["tau_hat_s"] for r in tt], yerr=[[r["tau_hat_s"] - r["tau_ci_low"] for r in tt], [r["tau_ci_high"] - r["tau_hat_s"] for r in tt]], fmt="o", color=C_BLUE, ms=4, capsize=2, lw=1, label="τ̂_w (95 % CI)")
    ax.plot([0.03, 1.2], [0.03, 1.2], color=C_GREY, lw=0.8, ls=":")
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected τ_w (s)"); ax.set_ylabel("estimated τ̂_w (s)"); ax.set_title("(c) liveness timeout estimate", loc="left"); ax.legend(fontsize=7)
    # (d) threshold sweep
    ax = axs[1, 1]
    ax.plot([r["eps_mm"] for r in fsweep], [r["FPR"] for r in fsweep], color=C_BLUE, lw=1.5, label="frame FPR")
    ax.plot([r["eps_mm"] for r in fsweep], [r["FNR"] for r in fsweep], color=C_BLUE, lw=1.5, ls="--", label="frame FNR")
    ax.plot([r["eps_mm"] for r in fsweep], [r["undetermined_rate"] for r in fsweep], color=C_BLUE, lw=1, ls=":", label="frame undetermined")
    ax.plot([r["eps_mm"] for r in ssweep], [r["FPR"] for r in ssweep], color=C_ORANGE, lw=1.5, label="scale FPR")
    ax.plot([r["eps_mm"] for r in ssweep], [r["FNR"] for r in ssweep], color=C_ORANGE, lw=1.5, ls="--", label="scale FNR")
    ax.plot([r["eps_mm"] for r in ssweep], [r["undetermined_rate"] for r in ssweep], color=C_ORANGE, lw=1, ls=":", label="scale undetermined")
    ax.set_xscale("log"); ax.set_xlabel("tolerance ε (mm)"); ax.set_ylabel("rate"); ax.set_ylim(-0.02, 1.02); ax.set_title("(d) decision rates vs. tolerance", loc="left"); ax.legend(fontsize=6.5, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(figdir, "fig05_validation.pdf")); fig.savefig(os.path.join(figdir, "fig05_validation.png"), dpi=200)

    # error-model figure with measured points from M
    fig, axs = plt.subplots(1, 3, figsize=(6.8, 2.3))
    R = G.axis_angle([1, 0, 0], math.radians(150.0)); t = np.array([0.20, 0, 0])
    ps = np.linspace(0, 0.10, 101)
    ax = axs[0]
    ax.plot(ps * 1e3, [G.m1_positional_error(R, t, np.array([0, 0, p])) * 1e3 for p in ps], color=C_BLUE, lw=1.5, label="absolute, p⊥axis (eq. 1)")
    ax.plot(ps * 1e3, [G.m1_positional_error(R, t, np.array([p, 0, 0])) * 1e3 for p in ps], color=C_BLUE, lw=1.5, ls="--", label="absolute, p∥axis")
    ax.plot(ps * 1e3, [G.m1_upper_bound(R, t, p) * 1e3 for p in ps], color=C_GREY, lw=1, ls=":", label="bound (3)")
    ax.plot(ps * 1e3, [G.m1_incremental_bound(R, p) * 1e3 for p in ps], color=C_ORANGE, lw=1.5, label="incremental (5), step = ‖p‖")
    ax.plot(ps * 1e3, [0 for p in ps], color=C_ORANGE, lw=1.5, ls="--", label="incremental, R = I")
    for m in mrows:
        if m["kind"].startswith("absolute"):
            ax.plot(m["x_mm"], m["measured_mm"], "o", color=C_BLUE, ms=4, mfc="white")
        if m["kind"].startswith("incremental_perp"):
            ax.plot(m["x_mm"], m["measured_mm"], "o", color=C_ORANGE, ms=4, mfc="white")
    ax.set_xlabel("‖p_c‖ or step (mm)"); ax.set_ylabel("positional error (mm)"); ax.set_title("(a) M1, JHU base_frame", loc="left"); ax.legend(fontsize=5.5)
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
