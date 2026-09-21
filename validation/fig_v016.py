#!/usr/bin/env python3
"""fig05_validation for RC9 (0.1.6 rules): panels (a) and (b) as fig_v015.py / analyze_v013.py draw them from the archived
tables; panel (c) takes the timeout intervals from the offline re-derivation under the 0.1.6 rule
(reanalyze_liveness_v016.py; identical to 0.1.5 for all 23); panel (d) takes the tolerance sweeps re-scored with exact
decimal truth on the archived grid plus epsilon = 1 mm exactly (rescore_v016.py) and marks the 1-mm point.
Reads only; writes the figure to --figdir.

Usage: python3 validation/fig_v016.py [validation/v0.1.3/mock] [--figdir figures]
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("archive", nargs="?", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.3", "mock"))
ap.add_argument("--reanalysis", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "reanalysis", "L_liveness_v016.csv"))
ap.add_argument("--rescoring", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "rescoring"))
ap.add_argument("--figdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "figures"))
args = ap.parse_args()
os.makedirs(args.figdir, exist_ok=True)
tdir = os.path.join(args.archive, "tables")


def rows(name, path=None):
    with open(path or os.path.join(tdir, name)) as f:
        out = []
        for r in csv.DictReader(f):
            d = {}
            for k, v in r.items():
                try:
                    d[k] = float(v) if v not in ("", None) else None
                except ValueError:
                    d[k] = v
            out.append(d)
        return out


frows = rows("F_id_runs.csv"); srows = rows("S_si_runs.csv")
fsweep = rows(None, os.path.join(args.rescoring, "F_id_threshold_sweep_exact.csv")); ssweep = rows(None, os.path.join(args.rescoring, "S_si_threshold_sweep_exact.csv"))
lrows = rows("L_liveness.csv"); rean = {r["run"]: r for r in rows(None, args.reanalysis)}
for r in lrows:
    r["noise_mm"] = r.get("noise_mm")
for r in frows + srows:
    r["noise_mm"] = float(r["noise_mm"])
# 0.1.6 intervals replace the archived ones in panel (c); everything else (status, mode, expectation) is unchanged
for r in lrows:
    q = rean.get(r["run"])
    if q and q.get("v016_status") == "ok":
        r["interval_low_s"], r["interval_high_s"] = q["v016_low_s"], q["v016_high_s"]

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

C_BLUE, C_ORANGE, C_GREEN, C_PINK, C_GREY = "#0072B2", "#E69F00", "#009E73", "#CC79A7", "#6e6e6e"
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
    ax.errorbar([r["tau_true_s"] for r in ok], mid, yerr=[[m - r["interval_low_s"] for m, r in zip(mid, ok)], [r["interval_high_s"] - m for m, r in zip(mid, ok)]], fmt="o", color=C_BLUE, ms=3.5, capsize=2, lw=1, label="fault: τ_w interval, v0.1.6 rules (midpoint marked)")
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
fl = float(np.median([r["floor_ms"] for r in lrows])) / 1e3
ax.axhline(fl, color=C_GREY, lw=0.8, ls="--", label=f"median measured resolution floor {fl*1e3:.1f} ms")
ax.plot([0.01, 1.2], [0.01, 1.2], color=C_GREY, lw=0.8, ls=":")
ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected τ_w (s)"); ax.set_ylabel("τ_w interval (s)"); ax.set_title("(c) liveness timeout interval", loc="left"); ax.legend(fontsize=5.5, loc="upper left")
ax = axs[1, 1]
ax.plot([r["eps_mm"] for r in fsweep], [r["FNR"] for r in fsweep], color=C_BLUE, lw=1.5, label="frame false-conformant (FNR)")
ax.plot([r["eps_mm"] for r in fsweep], [r["FPR"] for r in fsweep], color=C_BLUE, lw=1.5, ls="--", label="frame false-divergent (FPR)")
ax.plot([r["eps_mm"] for r in fsweep], [r["undetermined_rate"] for r in fsweep], color=C_BLUE, lw=1, ls=":", label="frame undetermined")
ax.plot([r["eps_mm"] for r in ssweep], [r["FNR"] for r in ssweep], color=C_ORANGE, lw=1.5, label="scale false-conformant (FNR)")
ax.plot([r["eps_mm"] for r in ssweep], [r["FPR"] for r in ssweep], color=C_ORANGE, lw=1.5, ls="--", label="scale false-divergent (FPR)")
ax.plot([r["eps_mm"] for r in ssweep], [r["undetermined_rate"] for r in ssweep], color=C_ORANGE, lw=1, ls=":", label="scale undetermined")
e1 = [r for r in ssweep if abs(r["eps_mm"] - 1.0) < 1e-12]
if e1:
    ax.plot([1.0], [e1[0]["FPR"]], "o", mfc="none", mec=C_ORANGE, ms=6, mew=1.2)
    ax.annotate(f"ε = 1 mm exactly: {int(e1[0]['FP'])} false D (s = 1.01)", (1.0, e1[0]["FPR"]), xytext=(1.6, 0.30), fontsize=5.5, arrowprops=dict(arrowstyle="-", lw=0.5, color=C_GREY))
ax.set_xscale("log"); ax.set_xlabel("tolerance ε (mm)"); ax.set_ylabel("rate"); ax.set_ylim(-0.02, 1.02); ax.set_title("(d) decision rates vs. tolerance (exact truth)", loc="left"); ax.legend(fontsize=5.5, ncol=2, loc="upper center")
fig.tight_layout()
fig.savefig(os.path.join(args.figdir, "fig05_validation.pdf")); fig.savefig(os.path.join(args.figdir, "fig05_validation.png"), dpi=200)
print("written", os.path.join(args.figdir, "fig05_validation.pdf"), f"median floor {fl*1e3:.2f} ms; {len(ok)} fault + {len(rel)} drift intervals (v0.1.6)")
