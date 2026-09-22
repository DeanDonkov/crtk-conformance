#!/usr/bin/env python3
"""fig05_validation for RC14 (0.1.8): as fig_v016.py, except that panel (c) shows, for every archived fault and drift run,
both the sound interval of the evaluated rule (0.1.7 fault bound; validation/v0.1.7/L_archived_v017.csv) and the
conditional estimate (the 0.1.6 interval, which decides nothing), so that the width the sound bound costs is visible
(RC13 external review, minor comment).  Panels (a), (b), (d) are unchanged.  Original panel (c) note: the 0.1.6 intervals
came from the offline re-derivation (reanalyze_liveness_v016.py; identical to 0.1.5 for all 23); panel (d) takes the tolerance sweeps re-scored with exact
decimal truth on the archived grid plus epsilon = 1 mm exactly (rescore_v016.py) and marks the 1-mm point.
Reads only; writes the figure to --figdir.

Usage: python3 validation/fig_v018.py [validation/v0.1.3/mock] [--figdir figures]
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("archive", nargs="?", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.3", "mock"))
ap.add_argument("--reanalysis", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "reanalysis", "L_liveness_v016.csv"))
ap.add_argument("--rescoring", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.6", "rescoring"))
ap.add_argument("--figdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.8", "figures"))
ap.add_argument("--sound", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.7", "L_archived_v017.csv"))
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
sound = {r["run"]: r for r in rows(None, args.sound)}
for r in lrows:
    q = sound.get(r["run"])
    r["sound_low_s"], r["sound_high_s"] = (q["v017_low_s"], q["v017_high_s"]) if q and q.get("v017_status") == "ok" else (None, None)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

C_BLUE, C_ORANGE, C_GREEN, C_PINK, C_GREY = "#0072B2", "#E69F00", "#009E73", "#CC79A7", "#6e6e6e"
plt.rcParams.update({"font.size": 6.5, "axes.titlesize": 6.8, "axes.labelsize": 6.5, "xtick.labelsize": 6, "ytick.labelsize": 6, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.5, "legend.frameon": False,
                     "legend.handlelength": 1.6, "legend.borderaxespad": 0.2, "legend.labelspacing": 0.25})
fig, axs = plt.subplots(1, 4, figsize=(7.16, 1.95))  # RC14: one full-width strip (was 2 x 2 at 0.6 of the text width)
markers = {0.0: "o", 0.02: "s", 0.1: "^"}
cols = {0.0: C_BLUE, 0.02: C_ORANGE, 0.1: C_GREEN}
ax = axs[0]
for noise in sorted({r["noise_mm"] for r in frows}):
    sub = [r for r in frows if r["noise_mm"] == noise and r["theta_true_deg"] == 0]
    ax.plot([max(r["t_true_mm"], 0.03) for r in sub], [max(r["t_err_mm"], 1e-5) for r in sub], marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=0.9, ms=3, label=f"noise {noise:g} mm")
ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected ‖t‖ (mm), θ = 0"); ax.set_ylabel("‖t̂ − t‖ (mm), floor 1e-5"); ax.set_title("(a) frame: translation", loc="left"); ax.legend(fontsize=5.2, loc="center right")
ax = axs[1]
for noise in sorted({r["noise_mm"] for r in srows}):
    sub = sorted([r for r in srows if r["noise_mm"] == noise], key=lambda r: r["s_true"])
    ax.plot([r["s_true"] for r in sub], [r["s_hat_mean"] for r in sub], marker=markers.get(noise, "o"), color=cols.get(noise, C_GREY), lw=0.9, ms=3, label=f"ŝ, noise {noise:g} mm")
    ax.plot([r["s_true"] for r in sub], [r["r_int_mean"] for r in sub], marker=markers.get(noise, "o"), mfc="none", color=cols.get(noise, C_GREY), lw=0.9, ls="--", ms=3)
ax.plot([0.1, 10], [0.1, 10], color=C_GREY, lw=0.7, ls=":")
ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected scale s"); ax.set_ylabel("estimate"); ax.set_title("(b) scale: ŝ; internal ratio (dashed)", loc="left"); ax.legend(fontsize=5.2, loc="upper left")
ax = axs[2]
ok = [r for r in lrows if r["tau_status"] == "ok" and r["mode"] == "fault" and r["expect"] == "fault" and r["sound_low_s"] is not None]
if ok:
    x = [r["tau_true_s"] * 1.07 for r in ok]
    ax.vlines(x, [r["sound_low_s"] for r in ok], [r["sound_high_s"] for r in ok], color=C_BLUE, lw=1.5, label="fault: sound (decides)")
    x = [r["tau_true_s"] / 1.07 for r in ok]
    ax.vlines(x, [r["interval_low_s"] for r in ok], [r["interval_high_s"] for r in ok], color="#222222", lw=1.0, label="fault: conditional")
ub = [r for r in lrows if r["tau_status"] == "upper_bound" and r["mode"] == "fault"]
if ub:
    ax.plot([r["tau_true_s"] for r in ub], [r["tau_upper_bound_s"] for r in ub], "v", color=C_PINK, ms=4, label="below resolution: upper bound")
rel = [r for r in lrows if r["tau_status"] == "ok" and r["mode"] == "release" and r["expect"] == "drift"]
if rel:
    ax.vlines([r["tau_true_s"] for r in rel], [r["sound_low_s"] if r["sound_low_s"] is not None else r["interval_low_s"] for r in rel],
              [r["sound_high_s"] if r["sound_high_s"] is not None else r["interval_high_s"] for r in rel], color=C_ORANGE, lw=1.5, label="drift")
fl = float(np.median([r["floor_ms"] for r in lrows])) / 1e3
ax.axhline(fl, color=C_GREY, lw=0.7, ls="--", label=f"resolution floor {fl*1e3:.1f} ms")
ax.plot([0.01, 1.2], [0.01, 1.2], color="#bbbbbb", lw=0.6, label="y = x")
ax.set_xscale("log"); ax.set_yscale("log"); ax.set_xlabel("injected τ_w (s)"); ax.set_ylabel("τ_w interval (s)"); ax.set_title("(c) timeout intervals", loc="left"); ax.legend(fontsize=5.0, loc="upper left")
ax = axs[3]
ax.plot([r["eps_mm"] for r in fsweep], [r["FNR"] for r in fsweep], color=C_BLUE, lw=1.2, label="frame false C")
ax.plot([r["eps_mm"] for r in fsweep], [r["FPR"] for r in fsweep], color=C_BLUE, lw=1.2, ls="--", label="frame false D")
ax.plot([r["eps_mm"] for r in fsweep], [r["undetermined_rate"] for r in fsweep], color=C_BLUE, lw=0.9, ls=":", label="frame U")
ax.plot([r["eps_mm"] for r in ssweep], [r["FNR"] for r in ssweep], color=C_ORANGE, lw=1.2, label="scale false C")
ax.plot([r["eps_mm"] for r in ssweep], [r["FPR"] for r in ssweep], color=C_ORANGE, lw=1.2, ls="--", label="scale false D")
ax.plot([r["eps_mm"] for r in ssweep], [r["undetermined_rate"] for r in ssweep], color=C_ORANGE, lw=0.9, ls=":", label="scale U")
e1 = [r for r in ssweep if abs(r["eps_mm"] - 1.0) < 1e-12]
if e1:
    ax.plot([1.0], [e1[0]["FPR"]], "o", mfc="none", mec=C_ORANGE, ms=5, mew=1.0)
    ax.annotate(f"ε = 1 mm: {int(e1[0]['FP'])} false D (s = 1.01)", (1.0, e1[0]["FPR"]), xytext=(1.5, 0.42), fontsize=5.0, arrowprops=dict(arrowstyle="-", lw=0.5, color=C_GREY))
ax.set_xscale("log"); ax.set_xlabel("tolerance ε (mm)"); ax.set_ylabel("rate"); ax.set_ylim(-0.02, 1.02); ax.set_title("(d) rates vs. tolerance", loc="left"); ax.legend(fontsize=5.0, ncol=2, loc="upper center")
fig.tight_layout(pad=0.3, w_pad=0.6)
fig.savefig(os.path.join(args.figdir, "fig05_validation.pdf")); fig.savefig(os.path.join(args.figdir, "fig05_validation.png"), dpi=200)
print("written", os.path.join(args.figdir, "fig05_validation.pdf"), f"median floor {fl*1e3:.2f} ms; {len(ok)} fault + {len(rel)} drift intervals (sound and conditional)")
