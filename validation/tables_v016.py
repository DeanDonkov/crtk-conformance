#!/usr/bin/env python3
"""RC9: LaTeX tables of the v0.1.6 campaigns, generated from validation/v0.1.6/analysis (analyze_v016.py outputs).
Usage: python3 validation/tables_v016.py <outdir>   (writes dvrk_cases_v016.tex, src_geometry_v016.tex, ...)"""
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.join(HERE, "v0.1.6", "analysis")
out = sys.argv[1] if len(sys.argv) > 1 else A
os.makedirs(out, exist_ok=True)
NAMES = {"C_dvrk": "dVRK-authored + anchor", "C_src": "SRC-authored + anchor", "C_disc": "none (discover only)", "F2_identity": "identity",
         "F3_inverse": r"\code{base\_frame}$^{-1}$", "F4_0874": "residual 0.874 mm", "F5a_095": "residual 0.95 mm", "F5b_098": "residual 0.98 mm",
         "F5c_102": "residual 1.02 mm", "F5d_105": "residual 1.05 mm", "U_si_5mm": r"anchor, SI, $\varepsilon=5$ mm", "U_mm_1mm": r"anchor, mm client, 1 mm",
         "U_mm_5mm": r"anchor, mm client, 5 mm", "T_fault025": r"fault within 0.25 s", "F_exactJHU": "JHU transform"}


def dvrk():
    rows = list(csv.DictReader(open(os.path.join(A, "dvrk_sim_summary.csv"))))
    L = [r"\begin{tabular}{@{}llllll@{}}\toprule", r"Config & Declaration & Outcome & Predicted & $\hat E_{\max}$ / $\hat\lambda$ & Launches \\\midrule"]
    for r in rows:
        est = (f"{float(r['E_max_hat_mm']):.4f} mm" if r["E_max_hat_mm"] else "") + (("; " if r["E_max_hat_mm"] else "") + f"{float(r['lambda_hat']):.6f}" if r["lambda_hat"] else "")
        L.append(f"\\emph{{{r['config']}}} & {NAMES.get(r['case'], r['case'])} & {r['outcomes']} & {r['predicted']} & {est} & {r['launches']}{'' if r['all_match'] == 'True' else ' (!)'} \\\\")
    L += [r"\bottomrule\end{tabular}", r"\par\smallskip\footnotesize Outcome = spatial/dimensional/temporal in every launch; ``-'' = no prediction (class not probed). Truth $E_{\max}$: 278.0657, 412.3106, 0.8742, 0.9500, 0.9800, 1.0200, 1.0500 mm (derived from the configuration file). $\hat\lambda$ in m per interface unit."]
    open(os.path.join(out, "dvrk_cases_v016.tex"), "w").write("\n".join(L) + "\n")


def src():
    rows = json.load(open(os.path.join(A, "src_geometry.json")))
    L = [r"\begin{tabular}{@{}llllllp{3.2cm}@{}}\toprule", r"Release & Run & Outcome & Predicted & $d_{\mathrm{int}}$ (units) & $\hat\lambda$ [95\%] (m/unit) & Gates \\\midrule"]
    for r in rows:
        lo, hi = r["lambda_ci"]
        g = "passed" if r["gates_passed"] else "failed: " + ("axes " + "--".join(f"{x:.0f}" for x in _axes(r)) + r"$^\circ$" if _axes(r) else "")
        L.append(f"{r['release']} & {r['case'].replace('_geometry', '').replace('_', ' ')} & {r['spatial']}/{r['dimensional']}/{r['temporal']} & {r['pred']} & {r['d_int_if']:.7f} & {r['lambda_hat']:.4f} [{lo:.4f}, {hi:.4f}] & {g} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "src_geometry_v016.tex"), "w").write("\n".join(L) + "\n")


def _axes(r):
    import re
    vals = [float(m) for f in (r.get("gate_failures") or []) for m in re.findall(r"axes at ([0-9.]+) deg", f)]
    return (min(vals), max(vals)) if vals else None


def mc_full():
    """Supplement: the full offline Monte Carlo grid with Clopper-Pearson 95 % intervals."""
    d = json.load(open(os.path.join(HERE, "v0.1.6", "boundary", "boundary_montecarlo.json")))
    L = [r"\begin{tabular}{@{}lllrrrrrrl@{}}\toprule",
         r"Probe & Noise & $E/\varepsilon$ & C & D & false C & false D & U & coverage & false rate [95\% CI] (\%) \\\midrule"]
    prev = None
    for r in d["rows"]:
        key = (r["probe"], r["noise_model"], r["sigma_mm"])
        noise = f"Gauss.\\ {r['sigma_mm']:g} mm" if r["noise_model"] == "gaussian" else "mixture"
        head = f"{r['probe']} & {noise}" if key != prev else " & "
        if prev is not None and key != prev:
            L.append(r"\addlinespace")
        prev = key
        fk = "false_C" if r["truth"] == "divergent" else "false_D"
        lo, hi = r[f"{fk}_ci95"]
        L.append(f"{head} & {r['ratio']} & {r['correct_C']} & {r['correct_D']} & {r['false_C']} & {r['false_D']} & {r['U']} & {r['interval_coverage']:.3f} & "
                 f"{100 * r[f'{fk}_rate']:.2f} [{100 * lo:.2f}, {100 * hi:.2f}] \\\\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "mc_full_v016.tex"), "w").write("\n".join(L) + "\n")


def boundary_main():
    """Main text: false / undetermined rates (%) near the boundary, Monte Carlo (N = 2000) and live (N = 30)."""
    d = json.load(open(os.path.join(HERE, "v0.1.6", "boundary", "boundary_montecarlo.json")))
    mc = {(r["probe"], r["noise_model"], r["sigma_mm"], r["ratio"]): r for r in d["rows"]}
    live = {}
    lp = os.path.join(A, "B_live_boundary.csv")
    if os.path.exists(lp):
        for r in csv.DictReader(open(lp)):
            sig = {"0.02mm": 0.02, "0.1mm": 0.1, "0.001mm": 0.0}[r["sigma"]]
            live[(r["probe"], r["noise_model"], sig, r["ratio"])] = r
    ratios = ["0.95", "0.98", "1.00", "1.02", "1.05"]
    L = [r"\begin{tabular}{@{}llccccc@{}}\toprule", r" & & \multicolumn{5}{c}{$E/\varepsilon$} \\\cmidrule(l){3-7}",
         r"Probe & Noise & 0.95 & 0.98 & 1.00 & 1.02 & 1.05 \\\midrule"]
    for probe, model, sig, lab in (("frame", "gaussian", 0.02, "G 0.02"), ("frame", "gaussian", 0.1, "G 0.1"), ("frame", "mixture", 0.0, "mixture"),
                                   ("scale", "gaussian", 0.02, "G 0.02"), ("scale", "gaussian", 0.1, "G 0.1")):
        cells = []
        for rt in ratios:
            r = mc[(probe, model, sig, rt)]
            fk = "false_C" if r["truth"] == "divergent" else "false_D"
            u = 100 * r["U_rate"]
            us = f"{u:.1f}" if (99.5 <= u < 100 or 0 < u < 0.5) else f"{u:.0f}"
            cells.append(f"{100 * r[f'{fk}_rate']:.1f}/{us}")
        L.append(f"{probe.capitalize()} & {lab} & " + " & ".join(cells) + r" \\")
        if any((probe, model, sig, rt) in live for rt in ratios):
            cells = []
            for rt in ratios:
                q = live.get((probe, model, sig, rt))
                if q is None:
                    cells.append("")
                else:
                    n = int(q["n"]); fk = "false_C" if float(rt) > 1 else "false_D"
                    cells.append(f"{int(q[fk])}/{int(q['U'])}")
            L.append(r" & \quad live & " + " & ".join(cells) + r" \\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "boundary_main_v016.tex"), "w").write("\n".join(L) + "\n")


def loss_main():
    """Main text: stop evidence under command loss (N = 8 per cell)."""
    d = json.load(open(os.path.join(A, "L_loss.json")))
    cell = {(r["rule"], r["policy"], r["loss"]): r for r in d["rows"]}
    losses = ["none", "iid2", "iid5", "ge5"]
    L = [r"\begin{tabular}{@{}llcccc@{}}\toprule", r"Policy & Rule & none & iid 2\% & iid 5\% & GE 5\% \\\midrule"]
    for pol, lab in (("hold", r"hold$^a$"), ("fault", r"fault$^b$")):
        for rule in ("0.1.5", "0.1.6"):
            cells = []
            for lo in losses:
                r = cell[(rule, pol, lo)]
                c = r["classes"]
                if pol == "hold":
                    ft = c.get("false_trip", 0)
                    cells.append(f"{c.get('correct_hold', 0)}" + (f" ({ft})" if ft else ""))
                else:
                    cells.append(f"{c.get('contains', 0)}/{c.get('excludes', 0)}/{c.get('contradictory', 0) + sum(v for k, v in c.items() if k.startswith('not_formed'))}")
            L.append(f"{lab if rule == '0.1.5' else ''} & {rule} & " + " & ".join(cells) + r" \\")
            lab = lab
    L.append(r"\midrule")
    for rule in ("0.1.5", "0.1.6"):
        cells = [str(cell[(rule, "hold", lo)]["calibration_caught_loss"] + cell[(rule, "fault", lo)]["calibration_caught_loss"]) for lo in losses]
        L.append(f"{'loss seen$^c$' if rule == '0.1.5' else ''} & {rule} & " + " & ".join(c + "/16" for c in cells) + r" \\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "loss_main_v016.tex"), "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    dvrk()
    src()
    mc_full()
    boundary_main()
    if os.path.exists(os.path.join(A, "L_loss.json")):
        loss_main()
    print("written to", out)
