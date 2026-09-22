#!/usr/bin/env python3
"""RC14: LaTeX tables of the 0.1.8 campaigns and studies (validation/v0.1.8/{analysis,studies}).
Usage: python3 validation/tables_v018.py <outdir>"""
import json
import os
import statistics
import sys
from decimal import Decimal, ROUND_HALF_UP

HERE = os.path.dirname(os.path.abspath(__file__))
V18 = os.path.join(HERE, "v0.1.8")
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(V18, "analysis")
os.makedirs(out, exist_ok=True)


def pct(k, n, nd=1):
    q = Decimal(1).scaleb(-nd)
    return str((Decimal(100 * k) / Decimal(n)).quantize(q, rounding=ROUND_HALF_UP))


def write(name, lines):
    open(os.path.join(out, name), "w").write("\n".join(lines) + "\n")


def frame_guard():
    d = json.load(open(os.path.join(V18, "studies", "frame_guard.json")))
    L = [r"\begin{tabular}{@{}rrrcrrrrl@{}}\toprule",
         r" & & & & \multicolumn{2}{c}{0.1.7 (back to back)} & \multicolumn{2}{c}{0.1.8 (guard)} & \\\cmidrule(lr){5-6}\cmidrule(lr){7-8}",
         r"$\sigma$ (mm) & $\phi$ & $\rho_{\mathrm{trials}}$ & $E/\varepsilon$ & false (\%) & decided (\%) & false (\%) & decided (\%) & withheld (unresolved/budget/$n_{\mathrm{eff}}$) \\\midrule"]
    prev = None
    for r in d["rows"]:
        if prev is not None and r["sigma_mm"] != prev:
            L.append(r"\addlinespace")
        prev = r["sigma_mm"]
        n = r["n_rep"]
        fk = "false_C" if float(r["ratio"]) > 1 else "false_D"
        w = r["withheld"]
        L.append(f"{r['sigma_mm']:g} & {r['phi']:g} & {r['rho_trials_back_to_back']:.2f} & {r['ratio']} & {pct(r['v017'][fk], n)} & {pct(n - r['v017']['U'], n, 0)} & "
                 f"{pct(r['v018'][fk], n)} & {pct(n - r['v018']['U'], n, 0)} & {w['unresolved']}/{w['budget']}/{w['neff']} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    write("frame_guard_v018.tex", L)


FAC_LAB = {"sigma_t_mm": r"pose noise $\sigma_t$ (mm)", "step_rad": r"wrist step (rad)", "k": "trials $k$", "coupling": "coupling (reported)",
           "reach": "yaw reach (reported)", "theta_deg": r"axes angle ($^\circ$)", "u_L": "$u_L$", "L_impl_ratio": r"$L_{\mathrm{impl}}/L$",
           "kappa": r"joint-reading scale $\kappa$"}


def anchor_poe():
    d = json.load(open(os.path.join(V18, "studies", "anchor_poe.json")))
    L = [r"\begin{tabular}{@{}llrrrrll@{}}\toprule",
         r"Factor & Level & bias (\%) & s.d. (\%) & coverage & gates & C/D/U at 1 mm (\%) & C/D/U at 5 mm (\%) \\\midrule"]
    prev = None
    for r in d["rows"]:
        if prev is not None and r["factor"] != prev:
            L.append(r"\addlinespace")
        lab = FAC_LAB[r["factor"]] if r["factor"] != prev else ""
        prev = r["factor"]
        lv = r["level"]
        lvs = f"{lv:.4f}" if r["factor"] == "L_impl_ratio" else f"{lv:g}"
        n = r["n"]
        f1 = "/".join(pct(round(r[f"eps1mm_{k}"] * n), n, 0) for k in ("conformant", "divergent", "undetermined"))
        f5 = "/".join(pct(round(r[f"eps5mm_{k}"] * n), n, 0) for k in ("conformant", "divergent", "undetermined"))
        L.append(f"{lab} & {lvs} & {r['lambda_bias_pct_median']:.3f} & {r['lambda_sd_pct']:.3f} & {r['coverage_widened']:.2f} & {r['gate_pass']:.2f} & {f1} & {f5} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    write("anchor_poe_v018.tex", L)


def oc():
    d = json.load(open(os.path.join(V18, "studies", "operating_characteristic.json")))
    ratios = ["0.80", "0.90", "0.95", "1.00", "1.05", "1.10", "1.20"]
    idx = {(r["probe"], r["sigma_mm"], r["trials"], r["ratio"]): r for r in d["rows"]}
    L = [r"\begin{tabular}{@{}lrr" + "c" * len(ratios) + r"@{}}\toprule",
         r" & & & \multicolumn{" + str(len(ratios)) + r"}{c}{decided / false among decided (\%) at $E/\varepsilon$} \\\cmidrule(l){4-" + str(3 + len(ratios)) + "}",
         r"Probe & $\sigma$ (mm) & $n$ & " + " & ".join(ratios) + r" \\\midrule"]
    for probe in ("frame", "scale"):
        for sigma in (0.02, 0.1, 0.2):
            for n in sorted({k[2] for k in idx if k[0] == probe}):
                cells = []
                for rs in ratios:
                    r = idx[(probe, sigma, n, rs)]
                    det = r["n_rep"] - r["U"]
                    fk = "false_C" if float(rs) > 1 else "false_D"
                    cells.append(f"{pct(det, r['n_rep'], 0)}/" + (pct(r[fk], det, 0) if det else "--"))
                L.append(f"{probe.capitalize() if (sigma == 0.02 and n == min(k[2] for k in idx if k[0] == probe)) else ''} & {sigma:g} & {n} & " + " & ".join(cells) + r" \\")
            if not (probe == "scale" and sigma == 0.2):
                L.append(r"\addlinespace")
    L += [r"\bottomrule\end{tabular}"]
    write("oc_v018.tex", L)


def oc_main():
    """Main-text table: decision rate and error among determinate verdicts near the boundary (Monte Carlo, guard study,
    campaign F and the v0.1.6 live boundary runs)."""
    import csv
    ratios = ["0.90", "0.95", "0.98", "1.00", "1.02", "1.05", "1.10"]
    oc = {(r["probe"], r["sigma_mm"], r["trials"], r["ratio"]): r for r in json.load(open(os.path.join(V18, "studies", "operating_characteristic.json")))["rows"]}
    fg = {(r["sigma_mm"], r["phi"], r["ratio"]): r for r in json.load(open(os.path.join(V18, "studies", "frame_guard.json")))["rows"]}
    camp = json.load(open(os.path.join(V18, "analysis", "campaigns_v018.json")))["F"]
    live16 = {(r["probe"], r["noise_model"], r["sigma"], r["ratio"]): r for r in csv.DictReader(open(os.path.join(HERE, "v0.1.6", "analysis", "B_live_boundary.csv")))}

    def fk(rs):
        return "false_C" if float(rs) > 1 else "false_D"

    def mc(c, n, rs):
        det = n - c["U"]
        return f"{pct(det, n, 1 if 0 < 100 * det < n else 0)}/" + (pct(c[fk(rs)], det, 0) if det else "--")

    def cnts(runs, rs):
        det = [x for x in runs if x["class"] != "U"]
        return f"{len(det)}/" + (str(sum(x['class'] in ('fC', 'fD') for x in det)) if det else "--")

    def live(probe, sigma, rs):
        r = live16.get((probe, "gaussian", sigma, rs))
        if not r:
            return ""
        det = int(r["n"]) - int(r["U"])
        return f"{det}/" + (str(int(r["false_C"]) + int(r["false_D"])) if det else "--")

    rows = []
    rows.append(("Frame", "G 0.02", [mc(oc[("frame", 0.02, 10, rs)], oc[("frame", 0.02, 10, rs)]["n_rep"], rs) for rs in ratios]))
    for proc, lab in (("v017", "b2b"), ("v018", "guard")):
        rows.append(("", r"AR " + lab, [mc(fg[(0.02, 0.99, rs)][proc], fg[(0.02, 0.99, rs)]["n_rep"], rs) if (0.02, 0.99, rs) in fg else "" for rs in ratios]))
        pr = "0.1.7" if proc == "v017" else "0.1.8"
        rows.append(("", r"\quad live$^b$", [cnts(camp[f"{pr}|0.99|{rs}"], rs) if f"{pr}|0.99|{rs}" in camp else "" for rs in ratios]))
    rows.append(("", "G 0.1", [mc(oc[("frame", 0.1, 10, rs)], oc[("frame", 0.1, 10, rs)]["n_rep"], rs) for rs in ratios]))
    rows.append(("Scale", "G 0.02", [mc(oc[("scale", 0.02, 9, rs)], oc[("scale", 0.02, 9, rs)]["n_rep"], rs) for rs in ratios]))
    rows.append(("", "G 0.1", [mc(oc[("scale", 0.1, 9, rs)], oc[("scale", 0.1, 9, rs)]["n_rep"], rs) for rs in ratios]))

    L = [r"\begin{tabular}{@{}ll" + "c" * len(ratios) + r"@{}}\toprule",
         r" & & \multicolumn{" + str(len(ratios)) + r"}{c}{$E/\varepsilon$} \\\cmidrule(l){3-" + str(2 + len(ratios)) + "}",
         r"Probe & Noise & " + " & ".join(ratios) + r" \\\midrule"]
    for a, b, cells in rows:
        if a == "Scale":
            L.append(r"\addlinespace")
        L.append(f"{a} & {b} & " + " & ".join(cells) + r" \\")
    L += [r"\bottomrule\end{tabular}"]
    write("oc_main_v018.tex", L)


def deadlines():
    d = json.load(open(os.path.join(V18, "studies", "deadline_resolution.json")))
    L = [r"\begin{tabular}{@{}lrrrr@{}}\toprule",
         r"Runs & $n$ & sound: confirm from $\tau_w+{}$ & sound: refute below $\tau_w-{}$ & conditional: confirm from $\tau_w+{}$ \\\midrule"]
    lab = {"none": "confirmatory, no loss", "iid2": "confirmatory, iid 2\\%", "iid5": "confirmatory, iid 5\\%", "ge5": "confirmatory, GE 5\\%"}

    def rng(v):
        v = [x * 1e3 for x in v if x is not None]
        if not v:
            return "--"
        return f"{statistics.median(v):.0f} ({min(v):.0f}--{max(v):.0f})"
    for k in ("none", "iid2", "iid5", "ge5"):
        rows = [r for r in d["confirmatory_rows"] if r["loss"] == k]
        L.append(f"{lab[k]} & {len(rows)} & {rng([r['sound_confirm_margin_s'] for r in rows])} & {rng([r['sound_refute_margin_s'] for r in rows])} & "
                 f"{rng([r.get('conditional_confirm_margin_s') for r in rows])} \\\\")
    L.append(r"\addlinespace")
    for r in sorted(d["archived_rows"], key=lambda r: (r["mode"], r["tau_w_s"], r["run"])):
        L.append(f"\\code{{{r['run'].replace('_', chr(92) + '_')}}} ({r['mode']}, $\\tau_w={r['tau_w_s'] * 1e3:.0f}$\\,ms) & 1 & {r['sound_confirm_margin_s'] * 1e3:.0f} & "
                 f"{r['sound_refute_margin_s'] * 1e3:.0f} & {r['conditional_confirm_margin_s'] * 1e3:.0f} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    write("deadline_v018.tex", L)


def campaigns():
    d = json.load(open(os.path.join(V18, "analysis", "campaigns_v018.json")))
    esc = lambda t: t.replace("_", r"\_").replace("%", r"\%").replace(">=", r"$\ge$").replace("+-", r"$\pm$")
    L = [r"\begin{tabularx}{\textwidth}{@{}lXl@{}}\toprule", r"ID & Prediction & Outcome \\\midrule"]
    for p in d["predictions"]:
        L.append(f"{p['id']} & {esc(p['prediction'])} & {'matched' if p['matched'] else 'deviates'} \\\\")
    L += [r"\bottomrule\end{tabularx}"]
    write("predictions_v018.tex", L)
    # F cells
    L = [r"\begin{tabular}{@{}llrrrrrrl@{}}\toprule",
         r"Procedure & $\phi$ & $E/\varepsilon$ & C & D & U & false & withheld & planned spacing (samples) \\\midrule"]
    for key in sorted(d["F"], key=lambda k: (k.split("|")[1], k.split("|")[2], k.split("|")[0])):
        pr, phi, rs = key.split("|"); v = d["F"][key]
        c = sum(x["outcome"] == "conformant" for x in v); dv = sum(x["outcome"] == "divergent" for x in v); u = sum(x["outcome"] == "undetermined" for x in v)
        fl = sum(x["class"] in ("fC", "fD") for x in v); wh = sum(x["ok"] is False for x in v)
        sp = [x["spacing"] for x in v if x["spacing"] is not None and x["ok"] is not False]  # runs whose trials were taken
        sps = "--" if pr == "0.1.7" or not sp else (str(sp[0]) if min(sp) == max(sp) else f"{min(sp)}--{max(sp)} (median {statistics.median(sp):.0f})")
        L.append(f"{pr} & {float(phi):g} & {rs} & {c} & {dv} & {u} & {fl} & {wh if pr == '0.1.8' else '--'} & {sps} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    write("F_v018.tex", L)
    # S runs
    if d.get("S"):
        L = [r"\begin{tabular}{@{}llllll@{}}\toprule",
             r"Release & Run & Unit verdict & $d_{\mathrm{int}}$ (units) & $\hat\lambda$ [95\%, not widened] (m/unit) & Gates \\\midrule"]
        for v in ("v1", "v2"):
            for x in d["S"].get(v, []):
                ci = x.get("lambda_ci") or [None, None]
                lam = "--" if x["lambda"] is None else f"{x['lambda']:.4f}" + ("" if ci[0] is None else f" [{ci[0]:.4f}, {ci[1]:.4f}]")
                di = "--" if x["d_int_if"] is None else f"{x['d_int_if']:.6f}"
                run = x["run"].split("/")[-2] + ", " + x["case"].replace("_geometry", "").replace("_", " ")
                if x["gates"]:
                    g = "passed"
                else:
                    import re as _re
                    kinds = {}
                    for f in x.get("failures") or []:
                        tr_ = f.split(":")[0]
                        k = ("translation residual" if "translation residual" in f else "rotation residual" if "rotation residual" in f
                             else "axes angle" if "wrist axes" in f else "excitation" if "excit" in f or "moved" in f else "other")
                        kinds.setdefault(k, set()).add(tr_)
                    g = "failed: " + "; ".join(f"{k} ({len(v)}/5)" for k, v in sorted(kinds.items()))
                L.append(f"{v} & {run} & {x['outcome']} & {di} & {lam} & {g} \\\\")
        L += [r"\bottomrule\end{tabular}"]
        write("S_v018.tex", L)


def traces():
    d = json.load(open(os.path.join(V18, "analysis", "src_traces_v018.json")))
    L = [r"\begin{tabular}{@{}llrrrrrl@{}}\toprule",
         r"Release & Launch & samples & position s.d. $x/y/z$ (mm) & yaw s.d. (rad) & averaging offset (mm) & $\tau_{\mathrm{int}}$ & planned spacing \\\midrule"]
    for r in d["rows"]:
        mm = 100.0 if r["version"] == "v1" else 1000.0  # already converted in pos_sd_mm
        ps = "/".join(f"{x:.3f}" for x in r["pos_sd_mm"])
        lau = r["trace"].split("/")[-2].replace("launch", "")
        pl = r["plan"]
        tau = "--" if pl["deterministic"] else f"{pl['tau_int']:.1f}"
        sp = f"{pl['spacing']}" + (" (deterministic)" if pl["deterministic"] else "")
        off = "--" if r.get("window_average_offset_mm_median") is None else f"{r['window_average_offset_mm_median']:.3f}"
        L.append(f"{r['version']} & {lau} & {r['samples']} & {ps} & {r['yaw_sd_rad']:.3f} & {off} & {tau} & {sp} \\\\")
    L += [r"\bottomrule\end{tabular}"]
    write("traces_v018.tex", L)


if __name__ == "__main__":
    for f, fn in (("frame_guard.json", frame_guard), ("anchor_poe.json", anchor_poe), ("operating_characteristic.json", oc)):
        if os.path.exists(os.path.join(V18, "studies", f)):
            fn()
    if os.path.exists(os.path.join(V18, "studies", "deadline_resolution.json")):
        deadlines()
    if os.path.exists(os.path.join(V18, "analysis", "campaigns_v018.json")):
        oc_main()
        campaigns()
    if os.path.exists(os.path.join(V18, "analysis", "src_traces_v018.json")):
        traces()
    print("written to", out)
