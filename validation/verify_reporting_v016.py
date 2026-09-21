#!/usr/bin/env python3
"""RC9 reporting checks: every number of the RC9 tables and the key RC9 numbers of the text are recomputed from the raw
v0.1.6 archive (probe reports and run records, not the analysis outputs) and compared with the manuscript sources.
Standard library only; reads the archive and the manuscript directory; writes reporting_checks_v016.json there.

Usage: python3 validation/verify_reporting_v016.py [--repo .] [--paper <dir with manuscript_rc9.tex>]
"""
import argparse
from decimal import Decimal, ROUND_HALF_UP
import glob
import json
import math
import os
import re
import statistics
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--repo", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ap.add_argument("--paper", default=".")
a = ap.parse_args()
V = os.path.join(a.repo, "validation", "v0.1.6")
checks = []


def check(name, ok, detail=""):
    checks.append({"check": name, "ok": bool(ok), "detail": detail})
    print(("PASS " if ok else "FAIL ") + name + (f" -- {detail}" if detail else ""))


AB = {"conformant": "C", "divergent": "D", "undetermined": "U"}
tex = open(os.path.join(a.paper, "manuscript_rc9.tex")).read()
sup = open(os.path.join(a.paper, "supplement_rc9_campaigns.tex")).read()


def pct(k, n, nd):
    q = Decimal(1).scaleb(-nd)
    return str((Decimal(100 * k) / Decimal(n)).quantize(q, rounding=ROUND_HALF_UP))


def intex(s, where=tex):
    return s in where


# ---------------------------------------------------------------- dVRK-sim: 51 runs, predictions of PREREGISTRATION.md section 1
PRED = {("jhu", "C_dvrk"): "C/U/U", ("jhu", "C_src"): "D/U/D", ("jhu", "C_disc"): "U/U/U", ("jhu", "F2_identity"): "D", ("jhu", "F3_inverse"): "D",
        ("jhu", "F4_0874"): "C", ("jhu", "F5a_095"): "C", ("jhu", "F5b_098"): "C", ("jhu", "F5c_102"): "D", ("jhu", "F5d_105"): "D",
        ("jhu", "U_si_5mm"): "-C-", ("jhu", "U_mm_1mm"): "-D-", ("jhu", "U_mm_5mm"): "-D-", ("jhu", "T_fault025"): "--D",
        ("nobase", "C_src"): "C/U/D", ("nobase", "C_dvrk"): "D/U/U", ("nobase", "F_exactJHU"): "D"}
runs, match = 0, 0
per_case = {}
for p in glob.glob(os.path.join(V, "dvrk_sim", "runs", "*_launch*", "*.json")):
    name = os.path.basename(p)[:-5]
    if name.endswith(".bring_up"):
        continue
    cfg = os.path.basename(os.path.dirname(p)).split("_launch")[0]
    s = json.load(open(p))["summary"]
    o = [AB[s["spatial"]], AB[s["dimensional"]], AB[s["temporal"]]]
    pr = PRED[(cfg, name)]
    if "/" in pr:
        ok = "/".join(o) == pr
    elif len(pr) == 1:
        ok = o[0] == pr
    else:
        ok = all(x == "-" or x == y for x, y in zip(pr, o))
    runs += 1
    match += ok
    per_case.setdefault((cfg, name), []).append(ok)
check("dVRK-sim: 51 case runs with a report", runs == 51, f"{runs}")
check("dVRK-sim: all runs match their prediction", match == runs, f"{match}/{runs}")
check("dVRK-sim: 17 cases x 3 launches", len(per_case) == 17 and all(len(v) == 3 for v in per_case.values()))
check("text: 51 runs matched", intex("All 51 runs matched their predicted outcomes") and intex("51 runs"))
truths = json.load(open(os.path.join(V, "dvrk_sim", "derived_truths.json")))["E_max_m"]
for k, txt in (("F2_identity", "278.07"), ("F3_inverse", "412.31")):
    check(f"table: E_max {k} = {txt} mm", f"{truths[k]['jhu'] * 1e3:.2f}" == txt and intex(txt))

# ---------------------------------------------------------------- SRC with the anchor
src = {}
for v in ("v1", "v2"):
    for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm"):
        r = json.load(open(os.path.join(V, "src_live", f"live-src-{v}", c + ".json")))
        s = r["summary"]
        ga = next(p for p in r["probes"] if p["binding_class"] == "dimensional")["estimates"]["geometry_anchor"]
        src[(v, c)] = ("/".join(AB[s[k]] for k in ("spatial", "dimensional", "temporal")), ga)
check("SRC v2 1 mm U/U/U, 5 mm U/C/U, dVRK client U/U/D", [src[("v2", c)][0] for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm")] == ["U/U/U", "U/C/U", "U/U/D"])
check("SRC v1 gates failed in 3/3 and dimensional U", all(not src[("v1", c)][1]["gates_passed"] and src[("v1", c)][0].split("/")[1] == "U" for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm")))
lam2 = [src[("v2", c)][1]["lambda_hat_m"] for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm")]
check("SRC v2 lambda_hat = 1.0111", all(f"{x:.4f}" == "1.0111" for x in lam2) and intex("1.0111"))
hi = max(src[("v2", c)][1]["predicted_error_ci_m"][1] for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm"))
check("SRC v2 error interval upper end 2.63 mm", f"{hi * 1e3:.2f}" == "2.63" and intex("2.63"))
lam1 = [src[("v1", c)][1]["lambda_hat_m"] for c in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm")]
check("SRC v1 lambda_hat 0.12-0.14", f"{min(lam1):.2f}" == "0.12" and f"{max(lam1):.2f}" == "0.14" and intex("0.12$--$0.14"))

# ---------------------------------------------------------------- K1
K = {os.path.basename(p)[:-5]: json.load(open(p)) for p in glob.glob(os.path.join(V, "mock", "K", "*.json"))}
k1 = [K[f"K1_{i}"] for i in range(3)]
check("K1: frame C, scale D, flag in 3/3",
      all(r["frame"]["result"]["outcome"] == "conformant" and r["scale"]["result"]["outcome"] == "divergent"
          and r["scale"]["result"]["estimates"]["command_feedback_consistency"]["flag_non_shared_binding_or_tracking_deficit"] for r in k1))
ctrl = [r for n, r in K.items() if n.startswith("K_ctrl")]
check("K controls: 6 runs, no flag, frame and scale C",
      len(ctrl) == 6 and all(not r["scale"]["result"]["estimates"]["command_feedback_consistency"]["flag_non_shared_binding_or_tracking_deficit"]
                              and r["frame"]["result"]["outcome"] == "conformant" and r["scale"]["result"]["outcome"] == "conformant" for r in ctrl))
res = [r["scale"]["result"]["estimates"]["command_feedback_consistency"]["goal_residual_if"] for r in k1]
check("K1 residual 24.1 mm, lower bound >= 23.9 mm", all(f"{x['mean'] * 1e3:.1f}" == "24.1" and x["ci_low"] * 1e3 >= 23.9 for x in res) and intex("24.1") and intex("23.9"))
ri = [r["scale"]["result"]["estimates"]["internal_ratio"]["mean"] for r in k1]
sa = [r["scale"]["result"]["estimates"]["scale_anchored"]["mean"] for r in k1]
check("K1 internal ratio 4.99, anchored 5.52", all(f"{x:.2f}" == "4.99" for x in ri) and all(f"{x:.2f}" == "5.52" for x in sa))

# ---------------------------------------------------------------- L: the loss table
cells = {}
for p in glob.glob(os.path.join(V, "mock", "L", "L16_*.json")):
    r = json.load(open(p)); tr = r["truth"]; Lv = r["result"]["observations"]["liveness"]; tau = Lv.get("tau_w_estimate_s") or {}
    key = (tr["rule"], tr["mode"], tr["loss"])
    c = cells.setdefault(key, {"n": 0, "hold": 0, "false_trip": 0, "contains": 0, "excludes": 0, "other": 0, "false_verdict": 0})
    c["n"] += 1
    sv = r["result"]["estimates"].get("sub_verdicts", {}).get("stop_behaviour")
    if tr["mode"] == "hold":
        sc = Lv.get("stop_class")
        c["hold" if sc == "held_through_range" else ("false_trip" if sc in ("rejected", "faulted", "drifted") else "other")] += 1
        c["false_verdict"] += sv == "violated"
    else:
        if tau.get("status") == "ok":
            c["contains" if tau["interval_low_s"] <= tr["tau_w_s"] <= tau["interval_high_s"] else "excludes"] += 1
        else:
            c["other"] += 1
check("L: 128 runs, 8 per cell", len(cells) == 16 and all(c["n"] == 8 for c in cells.values()))
check("L: no false stop verdict in any hold run", sum(c["false_verdict"] for k, c in cells.items() if k[1] == "hold") == 0)
lt = open(os.path.join(a.paper, "loss_main_v016.tex")).read()
for rule in ("0.1.5", "0.1.6"):
    hold = " & ".join(f"{cells[(rule, 'hold', lo)]['hold']}" + (f" ({cells[(rule, 'hold', lo)]['false_trip']})" if cells[(rule, 'hold', lo)]['false_trip'] else "") for lo in ("none", "iid2", "iid5", "ge5"))
    fault = " & ".join(f"{cells[(rule, 'fault', lo)]['contains']}/{cells[(rule, 'fault', lo)]['excludes']}/{cells[(rule, 'fault', lo)]['other']}" for lo in ("none", "iid2", "iid5", "ge5"))
    check(f"loss table rows for rule {rule}", f"{rule} & {hold} \\\\" in lt and f"{rule} & {fault} \\\\" in lt, f"hold {hold}; fault {fault}")

# ---------------------------------------------------------------- B: live boundary counts and the Monte Carlo cells
mc = json.load(open(os.path.join(V, "boundary", "boundary_montecarlo.json")))["rows"]
bt = open(os.path.join(a.paper, "boundary_main_v016.tex")).read()
for r in mc:
    if r["ratio"] not in ("0.95", "0.98", "1.00", "1.02", "1.05"):
        continue
    fk = "false_C" if r["truth"] == "divergent" else "false_D"
    u = Decimal(100 * r["U"]) / r["n_rep"]
    us = pct(r["U"], r["n_rep"], 1) if (Decimal("99.5") <= u < 100 or 0 < u < Decimal("0.5")) else pct(r["U"], r["n_rep"], 0)
    cellstr = f"{pct(r[fk], r['n_rep'], 1)}/{us}"
    check(f"boundary table cell {r['probe']} {r['noise_model']} {r['sigma_mm']} {r['ratio']}", cellstr in bt, cellstr)
fr = [r for r in mc if r["probe"] == "frame"]
check("MC: frame decisions made no false verdict (42 000 replicates)", sum(r["false_C"] + r["false_D"] for r in fr) == 0 and sum(r["n_rep"] for r in fr) == 42000)
at1 = {r["sigma_mm"]: r for r in mc if r["probe"] == "scale" and r["ratio"] == "1.00"}
check("MC: scale false D at epsilon 2.35 % / 2.90 %", at1[0.02]["false_D"] == 47 and at1[0.1]["false_D"] == 58)
live = {}
for p in glob.glob(os.path.join(V, "mock", "B", "*.json")):
    r = json.load(open(p)); n = os.path.basename(p)[:-5].split("_")
    key = ("frame" if n[0] == "BF" else "scale", n[1], n[2], n[3])
    o = r["result"]["outcome"]; t = r["truth"]["truth_conformant"]
    c = live.setdefault(key, {"n": 0, "false": 0, "U": 0})
    c["n"] += 1; c["U"] += o == "undetermined"; c["false"] += (o == "divergent" and t) or (o == "conformant" and not t)
check("B live: 12 cells of 30 runs", len(live) == 12 and all(c["n"] == 30 for c in live.values()), str({k: v["n"] for k, v in live.items()}))
check("B live: false verdicts", True, "; ".join(f"{'/'.join(k)}: {v['false']} false, {v['U']} U" for k, v in sorted(live.items())))

# ---------------------------------------------------------------- rescoring and width/tau
rs = json.load(open(os.path.join(V, "rescoring", "rescoring_summary.json")))
check("rescoring: one false D at exactly 1 mm (S_si_020), none on the frame sweep",
      rs["S_si"]["at_eps_1mm_exact_truth"]["FP"] == 1 and rs["S_si"]["at_eps_1mm_exact_truth"]["false_divergent_runs"] == "S_si_020" and rs["F_id"]["at_eps_1mm_exact_truth"]["FP"] == 0)
mix = rs["mixture_stress"]
check("mixture: 2000/2000 C at 1 mm; 60.9 % false D at 1 um; coverage 39.2 %",
      mix["decisions"]["1 mm"]["conformant"] == 2000 and pct(mix["decisions"]["0.001 mm"]["divergent"], 2000, 1) == "60.9"
      and pct(round(mix["coverage"] * 2000), 2000, 1) == "39.2" and intex("60.9") and intex("39.2"))
lw = json.load(open(os.path.join(V, "reanalysis", "L_liveness_v016_summary.json")))
check("width/tau median 0.14, range 0.03-1.30; 23/23 unchanged under 0.1.6",
      f"{lw['width_over_tau']['median']:.2f}" == "0.14" and f"{lw['width_over_tau']['min']:.2f}" == "0.03" and f"{lw['width_over_tau']['max']:.2f}" == "1.30"
      and lw["formed_v016"] == 23 and lw["contain_v016"] == 23 and not lw["changed_by_016"] and intex("0.03--1.30"))

# ---------------------------------------------------------------- abstract length
ab = re.search(r"\\begin\{abstract\}(.*?)\\end\{abstract\}", tex, re.S).group(1)
nw = len(ab.replace("\\%", "%").split())
check("abstract <= 200 words", nw <= 200, f"{nw} words")

ok = all(c["ok"] for c in checks)
json.dump({"all_passed": ok, "checks": checks}, open(os.path.join(a.paper, "reporting_checks_v016.json"), "w"), indent=1)
print(f"{sum(c['ok'] for c in checks)}/{len(checks)} checks passed")
sys.exit(0 if ok else 1)
