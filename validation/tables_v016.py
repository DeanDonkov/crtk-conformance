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


if __name__ == "__main__":
    dvrk()
    src()
    print("written to", out)
