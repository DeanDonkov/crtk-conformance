#!/usr/bin/env python3
"""RC13: LaTeX tables of the confirmatory v0.1.7 campaign and the v0.1.7 sensitivity studies.
Reads validation/v0.1.6/analysis/L_loss.json (rules 0.1.5 and 0.1.6), validation/v0.1.7/analysis/confirmatory_v017.json
(analyze_v017.py) and validation/v0.1.7/studies/*.json.
Usage: python3 validation/tables_v017.py <outdir>   (writes loss_main_v017.tex, confirm_v017.tex, boundary_sens_v017.tex,
anchor_sens_v017.tex)"""
import json
import os
import statistics
import sys
from decimal import Decimal, ROUND_HALF_UP

HERE = os.path.dirname(os.path.abspath(__file__))
A16 = os.path.join(HERE, "v0.1.6", "analysis")
V17 = os.path.join(HERE, "v0.1.7")
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(V17, "analysis")
os.makedirs(out, exist_ok=True)
LOSSES = ["none", "iid2", "iid5", "ge5"]


def pct(k, n, nd=1):
    q = Decimal(1).scaleb(-nd)
    return str((Decimal(100 * k) / Decimal(n)).quantize(q, rounding=ROUND_HALF_UP))


def r0(x):
    return str(Decimal(repr(x)).quantize(Decimal(1), rounding=ROUND_HALF_UP))


def v17_runs():
    c = json.load(open(os.path.join(V17, "analysis", "confirmatory_v017.json")))
    return c, c["L_rows"]


def v17_calibration_lost():
    """run -> calibration commands left unanswered, from the raw records (the analysis row reads it for fault runs only)."""
    import glob
    out = {}
    for p in glob.glob(os.path.join(V17, "mock", "L", "L17_*.json")):
        d = json.load(open(p))
        R = d["result"]["observations"]["resolution"]
        out[os.path.basename(p)[:-5]] = 30 - int(R["latency_probes_responded"])
    return out


def loss_main():
    d = json.load(open(os.path.join(A16, "L_loss.json")))
    cell = {(r["rule"], r["policy"], r["loss"]): r for r in d["rows"]}
    c17, rows17 = v17_runs()
    L = [r"\begin{tabular}{@{}llcccc@{}}\toprule", r"Policy & Rule & none & iid 2\% & iid 5\% & GE 5\% \\\midrule"]
    for pol, lab in (("hold", r"hold$^a$"), ("fault", r"fault$^b$")):
        for rule in ("0.1.5", "0.1.6", "0.1.7"):
            cells = []
            for lo in LOSSES:
                if rule == "0.1.7":
                    rs = [r for r in rows17 if r["policy"] == pol and r["loss"] == lo]
                    if pol == "hold":
                        ok = sum(r["stop_class"] == "held_through_range" for r in rs)
                        ft = sum(r["stop_class"] in ("rejected", "faulted", "drifted") for r in rs)
                        no = sum(r["stop_class"] == "not_observable" for r in rs)
                        cells.append(f"{ok}" + (f" ({ft})" if ft else "") + ("$^f$" if no else ""))
                    else:
                        con = sum(r["status"] == "ok" and bool(r.get("contains")) for r in rs)
                        exc = sum(r["status"] == "ok" and not r.get("contains") for r in rs)
                        cells.append(f"{con}/{exc}/{len(rs) - con - exc}")
                    continue
                r = cell[(rule, pol, lo)]
                c = r["classes"]
                if pol == "hold":
                    ft = c.get("false_trip", 0)
                    cells.append(f"{c.get('correct_hold', 0)}" + (f" ({ft})" if ft else ""))
                else:
                    cells.append(f"{c.get('contains', 0)}/{c.get('excludes', 0)}/{c.get('contradictory', 0) + sum(v for k, v in c.items() if k.startswith('not_formed'))}")
            L.append(f"{lab if rule == '0.1.5' else ''} & {rule}{'$^d$' if rule == '0.1.7' else ''} & " + " & ".join(cells) + r" \\")
    L.append(r"\midrule")
    for rule in ("0.1.5", "0.1.6", "0.1.7"):
        if rule == "0.1.7":
            cl = v17_calibration_lost()
            cells = [str(sum(1 for r in rows17 if r["loss"] == lo and cl[r["run"]] > 0)) for lo in LOSSES]
        else:
            cells = [str(cell[(rule, "hold", lo)]["calibration_caught_loss"] + cell[(rule, "fault", lo)]["calibration_caught_loss"]) for lo in LOSSES]
        L.append(f"{'loss seen$^c$' if rule == '0.1.5' else ''} & {rule} & " + " & ".join(x + "/16" for x in cells) + r" \\")
    L.append(r"\midrule")
    w16 = []
    for lo in LOSSES:
        ws = [x["interval_ms"][1] - x["interval_ms"][0] for x in cell[("0.1.6", "fault", lo)]["runs"] if x["class"] in ("contains", "excludes")]
        w16.append(r0(statistics.median(ws)))
    w17 = [r0(statistics.median([r["width_ms"] for r in rows17 if r["policy"] == "fault" and r["loss"] == lo and r["status"] == "ok"])) for lo in LOSSES]
    L.append(r"width$^e$ (ms) & 0.1.6 & " + " & ".join(w16) + r" \\")
    L.append(r" & 0.1.7 & " + " & ".join(w17) + r" \\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "loss_main_v017.tex"), "w").write("\n".join(L) + "\n")


def confirm():
    """Supplement: every pre-registered prediction with a concise observation recomputed from the analysis rows."""
    c, rows = v17_runs()
    K = c["K"]
    k1 = [v for n, v in K.items() if "_K1_" in n]
    ct = [v for n, v in K.items() if "_ctrl_" in n]
    hold = [r for r in rows if r["policy"] == "hold"]
    fault = [r for r in rows if r["policy"] == "fault"]
    formed = [r for r in fault if r["status"] == "ok"]
    odd = [r["run"].replace("L17_017_", "").replace("_", r"\_") for r in hold if r["stop_class"] != "held_through_range"]
    wn = [r["width_ms"] for r in formed if r["loss"] == "none"]
    obs = {
        "K-1": f"raised in {sum(bool(v['flag']) for v in k1)}/{len(k1)}",
        "K-2": f"undetermined in {sum(v['unit'] == 'undetermined' for v in k1)}/{len(k1)}; ungated divergent in {sum(v['unit_ungated'] == 'divergent' for v in k1)}/{len(k1)}",
        "K-3": f"conformant in {sum(v['frame'] == 'conformant' for v in k1)}/{len(k1)}",
        "K-4": f"all three as predicted in {sum(v['command_semantic'] == 'undetermined' and v['dimensional_summary'] == 'undetermined' and v['assumption'].startswith('contradicted') for v in k1)}/{len(k1)}",
        "K-5": f"no flag and all conformant in {sum((not v['flag']) and v['frame'] == v['unit'] == v['command_semantic'] == 'conformant' for v in ct)}/{len(ct)}",
        "L-1": f"{sum(r['stop_class'] == 'held_through_range' for r in hold)}/{len(hold)} held; " + ", ".join(f"not observable: \\code{{{o}}}" for o in odd) if odd else f"{len(hold)}/{len(hold)} held",
        "L-2": f"{sum(r['sub_verdict'] == 'satisfied' for r in hold)}/{len(hold)} satisfied; {sum(r['sub_verdict'] == 'undetermined' for r in hold)} undetermined",
        "L-3": f"{len(formed)}/{len(fault)} formed; {sum(bool(r.get('contains')) for r in formed)} contain",
        "L-4": f"{sum(r['status'] == 'inconsistent' for r in fault)} contradictory",
        "L-5": f"{sum(r['sub_verdict'] == 'satisfied' for r in formed)}/{len(formed)} satisfied; {sum(r['sub_verdict'] == 'violated' for r in fault)} violated",
        "L-6": f"{statistics.median(wn):.1f} ms ($n={len(wn)}$)",
        "L-7": f"{sum(r['cond_status'] is not None for r in formed)}/{len(formed)}",
        "L-8": f"{sum(r.get('decision_at_0245') == 'satisfied' for r in formed)} of {len(formed)} decide satisfied",
    }
    L = [r"\begin{tabular}{@{}lp{8.3cm}p{5.2cm}l@{}}\toprule", r"ID & Prediction & Observed & Outcome \\\midrule"]
    for p in c["predictions"]:
        outc = "matched" if p["matched"] else r"\textbf{deviates}"
        pr = p["prediction"].replace("%", "\\%")
        L.append(f"{p['id']} & {pr} & {obs[p['id']]} & {outc} " + "\\\\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "confirm_v017.tex"), "w").write("\n".join(L) + "\n")


MODEL_LAB = {"gauss": "iid Gaussian", "ar1-0.5": r"AR(1), $\rho=0.5$", "ar1-0.9": r"AR(1), $\rho=0.9$", "bias": r"bias $0.5\sigma$",
             "drift": r"drift, span $2\sigma$", "t3": r"Student-$t_3$"}


def boundary_sens():
    d = json.load(open(os.path.join(V17, "studies", "boundary_sensitivity.json")))
    idx = {(r["probe"], r["model"], r["ratio"]): r for r in d["rows"]}
    L = [r"\begin{tabular}{@{}llccccc@{}}\toprule", r" & & \multicolumn{3}{c}{false/undetermined (\%) at $E/\varepsilon$} & \multicolumn{2}{c}{coverage} \\\cmidrule(lr){3-5}\cmidrule(l){6-7}",
         r"Probe & Trial errors & 0.95 & 1.00 & 1.05 & at 1.00 & max false [95\% CI] (\%) \\\midrule"]
    for probe in ("frame", "scale"):
        for m in ("gauss", "ar1-0.5", "ar1-0.9", "bias", "drift", "t3"):
            cells = []
            worst = None
            for rt in ("0.95", "1.00", "1.05"):
                r = idx[(probe, m, rt)]
                fk = "false_C" if float(rt) > 1 else "false_D"
                cells.append(f"{pct(r[fk], r['n_rep'])}/{pct(r['U'], r['n_rep'], 0)}")
                if worst is None or r[fk] > worst[0]:
                    worst = (r[fk], r[f"{fk}_ci95"], r["n_rep"])
            cov = idx[(probe, m, "1.00")]["coverage"]
            L.append(f"{probe.capitalize() if m == 'gauss' else ''} & {MODEL_LAB[m]} & " + " & ".join(cells) + f" & {cov:.3f} & {pct(worst[0], worst[2])} [{100 * worst[1][0]:.1f}, {100 * worst[1][1]:.1f}] \\\\")
        if probe == "frame":
            L.append(r"\addlinespace")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "boundary_sens_v017.tex"), "w").write("\n".join(L) + "\n")


FAC_LAB = {"sigma_t_mm": r"pose noise $\sigma_t$ (mm)", "dq": r"step $\Delta q$ (rad)", "k": "trials $k$", "c": "coupling $c$", "theta_deg": r"axes angle ($^\circ$)",
           "offset_rad": "joint offset (rad)", "u_L": "$u_L$", "L_impl_ratio": r"$L_{\mathrm{impl}}/L$"}


def anchor_sens():
    d = json.load(open(os.path.join(V17, "studies", "anchor_sensitivity.json")))
    L = [r"\begin{tabular}{@{}llrrrrll@{}}\toprule",
         r"Factor & Level & bias (\%) & s.d. (\%) & coverage & gates & C/D/U at 1 mm (\%) & C/D/U at 5 mm (\%) \\\midrule"]
    prev = None
    for r in d["rows"]:
        if prev is not None and r["factor"] != prev:
            L.append(r"\addlinespace")
        lab = FAC_LAB[r["factor"]] if r["factor"] != prev else ""
        prev = r["factor"]
        lv = r["level"]
        lvs = f"{lv:.4f}" if r["factor"] == "L_impl_ratio" else (f"{lv:g}")
        n = r["n"]
        f1 = "/".join(pct(round(r[f"eps1mm_{k}"] * n), n, 0) for k in ("conformant", "divergent", "undetermined"))
        f5 = "/".join(pct(round(r[f"eps5mm_{k}"] * n), n, 0) for k in ("conformant", "divergent", "undetermined"))
        L.append(f"{lab} & {lvs} & {r['lambda_bias_pct_median']:.3f} & {r['lambda_sd_pct']:.3f} & {r['coverage_widened']:.3f} & {r['gate_pass']:.2f} & {f1} & {f5} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    open(os.path.join(out, "anchor_sens_v017.tex"), "w").write("\n".join(L) + "\n")


if __name__ == "__main__":
    if os.path.exists(os.path.join(V17, "analysis", "confirmatory_v017.json")):
        loss_main(); confirm()
    if os.path.exists(os.path.join(V17, "studies", "boundary_sensitivity.json")):
        boundary_sens()
    if os.path.exists(os.path.join(V17, "studies", "anchor_sensitivity.json")):
        anchor_sens()
    print("written to", out)
