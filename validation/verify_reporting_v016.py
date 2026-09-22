#!/usr/bin/env python3
"""RC9/RC10 reporting checks: every number of the v0.1.6 tables and the key v0.1.6 numbers of the text are recomputed
from the raw v0.1.6 archive (probe reports and run records, not the analysis outputs) and compared with the manuscript
sources.  Standard library only; reads the archive and the manuscript directory; writes reporting_checks_v016.json there.

RC10 (review of the RC9 copy): the live interval coverage is recomputed with explicit missing-value tests (RC9's
analysis turned a legitimate 0.0 lower end into 1 and reported 0.83--0.90 instead of 0.93--1.00; RC9's checks had not
covered coverage), the live boundary counts are checked against the supplement's text instead of only printed, the
analysis output B_live_boundary.csv is cross-checked against the raw recomputation, and the RC10 wording corrections
are checked.

RC13: the confirmatory v0.1.7 campaign (validation/v0.1.7/mock) is recomputed from its raw records, the sensitivity studies from
their outputs, and the RC13 wording is checked (reads <paper>/supplement_<tag>_v017.tex as well).

RC14: the v0.1.8 campaigns F, D and S (validation/v0.1.8) are recomputed from their raw records, the four offline studies
from their outputs, every v0.1.8 table of the paper is compared with a fresh run of tables_v018.py, and the RC14 wording is
checked (reads <paper>/supplement_<tag>_v018.tex and oc_main_v018.tex / loss_main_v018.tex as well).  Checks of earlier
rounds whose text RC14 moved to the supplement read it there (R14 switch).

RC12: the counts of the dVRK-sim text are checked as cases and launches, and the post hoc v0.1.7 re-derivation (consistency
gate; sound fault bound) is recomputed from the raw archive with the package's ROS-free decision functions and compared
with the text and with validation/v0.1.7/.

Usage: python3 validation/verify_reporting_v016.py [--repo .] [--paper <dir>] [--tag rc14]
       (reads <paper>/manuscript_<tag>.tex and <paper>/supplement_<tag>_campaigns.tex)
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
ap.add_argument("--tag", default="rc14", help="manuscript tag: reads manuscript_<tag>.tex and supplement_<tag>_campaigns.tex")
a = ap.parse_args()
R14 = a.tag not in ("rc9", "rc10", "rc11", "rc12", "rc13")  # RC14 moved version history and some numbers to the supplement
V = os.path.join(a.repo, "validation", "v0.1.6")
checks = []


def check(name, ok, detail=""):
    checks.append({"check": name, "ok": bool(ok), "detail": detail})
    print(("PASS " if ok else "FAIL ") + name + (f" -- {detail}" if detail else ""))


AB = {"conformant": "C", "divergent": "D", "undetermined": "U"}
tex = open(os.path.join(a.paper, f"manuscript_{a.tag}.tex")).read()
sup = open(os.path.join(a.paper, f"supplement_{a.tag}_campaigns.tex")).read()
_s18 = os.path.join(a.paper, f"supplement_{a.tag}_v018.tex")
sup18 = open(_s18).read() if os.path.exists(_s18) else ""
_oc = os.path.join(a.paper, "oc_main_v018.tex")
ocmain = open(_oc).read() if os.path.exists(_oc) else ""


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
check("text: 17 cases matched in each of three launches (51 runs)", len(per_case) == 17
      and intex("All 17 cases matched their predicted outcomes in each of the three launches (51 runs") and intex("all pre-registered cases matched" if R14 else "all 17 pre-registered cases matched"))
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
lt = open(os.path.join(a.paper, "loss_main_v017.tex" if os.path.exists(os.path.join(a.paper, "loss_main_v017.tex")) else "loss_main_v016.tex")).read()
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


def endpoint(v):
    """An interval endpoint as a float, or None when missing or NaN.  0.0 is a legitimate endpoint: no truthiness test."""
    if v is None:
        return None
    v = float(v)
    return None if math.isnan(v) else v


live = {}
for p in glob.glob(os.path.join(V, "mock", "B", "*.json")):
    r = json.load(open(p)); n = os.path.basename(p)[:-5].split("_")
    key = ("frame" if n[0] == "BF" else "scale", n[1], n[2], n[3])
    o = r["result"]["outcome"]; t = r["truth"]["truth_conformant"]
    c = live.setdefault(key, {"n": 0, "cC": 0, "cD": 0, "fC": 0, "fD": 0, "U": 0, "covered": 0, "missing": 0, "lo_zero": 0})
    c["n"] += 1
    c["U" if o == "undetermined" else ("cC" if (o == "conformant" and t) else "fC" if o == "conformant" else "fD" if t else "cD")] += 1
    est = r["result"]["estimates"]
    if key[0] == "frame":
        iv = est.get("spatial_decision") or {}
        lo, hi = endpoint(iv.get("ci_low_m")), endpoint(iv.get("ci_high_m"))
    else:
        iv = est.get("predicted_error_at_workspace_edge_m") or {}
        lo, hi = endpoint(iv.get("ci_low")), endpoint(iv.get("ci_high"))
    if lo is None or hi is None:
        c["missing"] += 1
    else:
        c["lo_zero"] += lo == 0.0
        c["covered"] += lo - 1e-9 <= r["truth"]["E_true_m"] <= hi + 1e-9
check("B live: 12 cells of 30 runs", len(live) == 12 and all(c["n"] == 30 for c in live.values()), str({k: v["n"] for k, v in live.items()}))
check("B live: every run reports both interval endpoints", sum(c["missing"] for c in live.values()) == 0,
      f"{sum(c['lo_zero'] for c in live.values())} lower endpoints are exactly 0.0")
# the supplement's per-cell counts (S4.6, live confirmation), generated from the raw records
LC = {k: v for k, v in live.items()}
g = lambda pr, mo, sg, ra: LC[(pr, mo, sg, ra)]
f02 = [g("frame", "gaussian", "0.02mm", x) for x in ("0.95", "1.00", "1.05")]
check("B live text: frame 0.02 mm cells", f02[0]["cC"] == 30 and f02[1]["U"] == 30 and f02[2]["cD"] == 30
      and intex("30 C, 30 U and 30 D at 0.95, 1.00 and $1.05\\varepsilon$", sup), str(f02))
f10 = [g("frame", "gaussian", "0.1mm", x) for x in ("0.95", "1.00", "1.05")]
check("B live text: frame 0.1 mm cells", all(c["U"] == 30 for c in f10) and intex("frame at \\SI{0.1}{mm}: 30 U in each cell", sup), str(f10))
sc = [g("scale", "gaussian", "0.1mm", x) for x in ("0.95", "1.00", "1.05")]
sc_txt = (f"scale: {sc[0]['cC']} C and {sc[0]['U']} U at $0.95\\varepsilon$; {sc[1]['cC']} C, {sc[1]['fD']} false D and {sc[1]['U']} U at "
          f"$\\varepsilon$; {sc[2]['U']} U at $1.05\\varepsilon$")
check("B live text: scale cells", sc[0]["cC"] + sc[0]["U"] == 30 and sc[1]["cC"] + sc[1]["fD"] + sc[1]["U"] == 30 and sc[2]["U"] == 30 and intex(sc_txt, sup), sc_txt)
mx = [g("frame", "mixture", "0.001mm", x) for x in ("0.95", "1.00", "1.05")]
mx_txt = f"It gave {mx[0]['cC']} C and {mx[0]['U']} U at $0.95\\varepsilon$, {mx[1]['cC']} C and {mx[1]['U']} U at $\\varepsilon$, and {mx[2]['cD']} D and {mx[2]['U']} U at $1.05\\varepsilon$, with no false verdict."
check("B live text: mixture cells, no false verdict", all(c["fC"] + c["fD"] == 0 for c in mx) and intex(mx_txt, sup), mx_txt)
_bl = (f" & \\quad live$^b$ &  & {30 - sc[0]['U']}/0 &  & {30 - sc[1]['U']}/{sc[1]['fD']} &  & {30 - sc[2]['U']}/" + ("--" if sc[2]["U"] == 30 else "0") + " &  \\\\")
check("B live: the only false verdict is one false D, scale at epsilon", sum(c["fC"] + c["fD"] for c in live.values()) == 1 and sc[1]["fD"] == 1
      and (intex(sc_txt, sup) if R14 else intex("for scale, one false divergence occurred in 30 runs at $\\varepsilon$")))
check("B live: one correct conformant scale verdict exactly at epsilon", sc[1]["cC"] == 1)
# live interval coverage (RC10 point 1)
cov = [c["covered"] for c in sc]
lo_c, hi_c = min(cov) / 30, max(cov) / 30
cov_txt = f"Live scale coverage was {lo_c:.2f}--{hi_c:.2f} ({cov[0]}, {cov[1]} and {cov[2]} of 30 at 0.95, 1.00 and $1.05\\varepsilon$)"
check("B live coverage: scale 30/30, 28/30, 30/30", cov == [30, 28, 30], str(cov))
check("B live coverage: supplement states 0.93--1.00 with counts", intex(cov_txt, sup) and not intex("0.83--0.90", sup), cov_txt)
check("B live coverage: frame Gaussian 180/180", sum(g("frame", "gaussian", s_, x)["covered"] for s_ in ("0.02mm", "0.1mm") for x in ("0.95", "1.00", "1.05")) == 180)
csvp = os.path.join(V, "analysis", "B_live_boundary.csv")
import csv as _csv
arows = {(r_["probe"], r_["noise_model"], r_["sigma"], r_["ratio"]): r_ for r_ in _csv.DictReader(open(csvp))}
check("B analysis CSV: covered and coverage equal the raw recomputation in all 12 cells",
      len(arows) == 12 and all(int(arows[k]["covered"]) == v["covered"] and abs(float(arows[k]["coverage"]) - v["covered"] / v["n"]) < 1e-12 for k, v in live.items()),
      "; ".join(f"{'/'.join(k)} {v['covered']}/{v['n']}" for k, v in sorted(live.items())))
# Monte Carlo coverage ranges quoted in the supplement
sc_mc = [r for r in mc if r["probe"] == "scale"]
sc_cov = (min(r["interval_coverage"] for r in sc_mc), max(r["interval_coverage"] for r in sc_mc))
check("MC: scale coverage 0.949--0.958 (quoted under the simulated model)", f"{sc_cov[0]:.3f}--{sc_cov[1]:.3f}" == "0.949--0.958" and intex("0.949--0.958", sup), f"{sc_cov}")
fg = [r for r in fr if r["noise_model"] == "gaussian"]
fm = [r for r in fr if r["noise_model"] != "gaussian"]
check("MC: frame Gaussian coverage 1.000; mixture 39--41 %", all(r["interval_coverage"] == 1.0 for r in fg)
      and f"{100 * min(r['interval_coverage'] for r in fm):.0f}--{100 * max(r['interval_coverage'] for r in fm):.0f}" == "39--41" and intex("39--41\\%", sup),
      f"mixture {min(r['interval_coverage'] for r in fm)}..{max(r['interval_coverage'] for r in fm)}")

# ---------------------------------------------------------------- rescoring and width/tau
rs = json.load(open(os.path.join(V, "rescoring", "rescoring_summary.json")))
check("rescoring: one false D at exactly 1 mm (S_si_020), none on the frame sweep",
      rs["S_si"]["at_eps_1mm_exact_truth"]["FP"] == 1 and rs["S_si"]["at_eps_1mm_exact_truth"]["false_divergent_runs"] == "S_si_020" and rs["F_id"]["at_eps_1mm_exact_truth"]["FP"] == 0)
mix = rs["mixture_stress"]
check("mixture: 2000/2000 C at 1 mm; 60.9 % false D at 1 um; coverage 39.2 %",
      mix["decisions"]["1 mm"]["conformant"] == 2000 and pct(mix["decisions"]["0.001 mm"]["divergent"], 2000, 1) == "60.9"
      and pct(round(mix["coverage"] * 2000), 2000, 1) == "39.2" and (intex("1217 divergent (all false)", sup) if R14 else intex("60.9")) and intex("39.2"))
lw = json.load(open(os.path.join(V, "reanalysis", "L_liveness_v016_summary.json")))
check("width/tau median 0.14, range 0.03-1.30; 23/23 unchanged under 0.1.6",
      f"{lw['width_over_tau']['median']:.2f}" == "0.14" and f"{lw['width_over_tau']['min']:.2f}" == "0.03" and f"{lw['width_over_tau']['max']:.2f}" == "1.30"
      and lw["formed_v016"] == 23 and lw["contain_v016"] == 23 and not lw["changed_by_016"] and intex("0.03--1.30"))

# ---------------------------------------------------------------- RC10 wording corrections (review of the RC9 copy, points 2-4)
check("text: closed-boundary sentence (divergence erroneous, conformance correct, undetermined an abstention)",
      (intex("At $E=\\varepsilon$ every determinate verdict comes from an interval that excludes the truth") if R14 else
       intex("At the closed boundary, divergence is erroneous; conformance is correct, and undetermined is an abstention."))
      and not intex("only false divergent or undetermined verdicts are possible"))
check("text: scale calibration stated only under the tested Monte Carlo model",
      intex("Under the tested model it is calibrated" if R14 else "Under the tested Monte Carlo model") and not intex("The decision is calibrated") and not intex("The scale interval is calibrated", sup)
      and intex("under the simulated trial model", sup))
check("text: repetition could reduce the cost, subject to validation", intex("could narrow the margin, subject to validation" if R14 else "could reduce that cost, subject to validation")
      and not intex("would avoid that cost"))
exc = []
for p in glob.glob(os.path.join(V, "mock", "L", "L16_*fault*.json")):
    r = json.load(open(p)); tr = r["truth"]; tau = r["result"]["observations"]["liveness"].get("tau_w_estimate_s") or {}
    if tau.get("status") == "ok" and not (tau["interval_low_s"] <= tr["tau_w_s"] <= tau["interval_high_s"]):
        exc.append((tr["rule"], tr["loss"], tau["interval_low_s"], tau["interval_high_s"], tr["tau_w_s"], r["result"]["estimates"]["sub_verdicts"]["stop_behaviour"]))
check("L: exactly one excluding interval, rule 0.1.6 at 5 % loss, its stop sub-verdict correct (satisfied)",
      len(exc) == 1 and exc[0][0] == "0.1.6" and exc[0][1] in ("iid5", "ge5") and exc[0][5] == "satisfied", str(exc))
if exc:
    hz = f"between {math.ceil(exc[0][3] * 1e3)} and \\SI{{{exc[0][4] * 1e3:.0f}}}{{ms}}"
    check("text: zero false stop verdicts qualified; the excluding interval would misdecide a horizon " + hz,
          intex("at the tested declarations") and intex("does not establish") and intex(hz), hz)
n5 = sum(cells[("0.1.6", "fault", lo)]["n"] for lo in ("iid5", "ge5"))
e5 = sum(cells[("0.1.6", "fault", lo)]["excludes"] for lo in ("iid5", "ge5"))
t1 = f"under 5\\% loss, {e5} of {n5} fault intervals (0.1.6 rule) excluded $\\tau_w$"
row = [ln for ln in tex.splitlines() if ln.startswith("Liveness estimates enclose injected timeouts")]
check("Table I liveness row states the observed exclusion", e5 == 1 and n5 == 16 and len(row) == 1 and t1.lower() in row[0].lower(), t1)
logs = sorted(glob.glob(os.path.join(a.paper, "code", "pytest_final_*.log")))
if logs:
    last = open(logs[-1]).read().strip().splitlines()
    summ = [ln for ln in last if re.search(r"\d+ passed", ln) and " in " in ln][-1]
    np_ = int(re.search(r"(\d+) passed", summ).group(1)); m_ = re.search(r"(\d+) failed", summ); nf = int(m_.group(1)) if m_ else 0
    failed = re.findall(r"^FAILED (\S+)", "\n".join(last), re.M)
    t2 = f"{np_} passed and {nf} failed" if nf else f"all {np_} passed"
    check(f"supplement states the final test run ({os.path.basename(logs[-1])}): {t2}", intex(t2, sup)
          and all(f.split("::")[-1].replace("_", "\\_") in sup for f in failed), f"{summ}; failed: {failed}")
else:
    check("final test log present in <paper>/code/", False)

# ---------------------------------------------------------------- RC12: post hoc v0.1.7 re-derivation, recomputed from the raw archive
sys.path.insert(0, os.path.join(a.repo, "src"))
from crtk_conformance.dimensional import consistency_gate  # noqa: E402
from crtk_conformance.liveness import fault_observation_window, timeout_interval_from_trials  # noqa: E402
from crtk_conformance.probes.base import Outcome  # noqa: E402


def cflag(res):
    c = (res.get("estimates") or {}).get("command_feedback_consistency")
    return c.get("flag_non_shared_binding_or_tracking_deficit") if isinstance(c, dict) else None


diag = []
for p in glob.glob(os.path.join(V, "mock", "K", "*.json")):
    diag.append((os.path.basename(p), json.load(open(p))["scale"]["result"]))
for p in glob.glob(os.path.join(V, "mock", "B", "BS_*.json")):
    diag.append((os.path.basename(p), json.load(open(p))["result"]))
for p in glob.glob(os.path.join(V, "dvrk_sim", "runs", "*", "*.json")):
    for res in (json.load(open(p)).get("probes") or []):
        if cflag(res) is not None:
            diag.append((os.path.basename(p), res))
diag = [(n, r) for n, r in diag if cflag(r) is not None]
changed = [n for n, r in diag if consistency_gate(Outcome(r["outcome"]), bool(cflag(r)))[0].value != r["outcome"]]
check("v0.1.7 gate: 102 runs carry the diagnostic; only the 3 K1 unit verdicts change (divergent -> undetermined)",
      len(diag) == 102 and sorted(changed) == ["K1_0.json", "K1_1.json", "K1_2.json"]
      and (intex("no other archived verdict changes") and intex("102 archived runs", sup) if R14 else intex("102 archived runs")), f"{len(diag)}; {sorted(changed)}")
lrows = []
for p in glob.glob(os.path.join(V, "mock", "L", "L16_*_fault250_*.json")):
    d = json.load(open(p)); obs = d["result"]["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s") or {}
    if "latency_allowance_s" not in tau:
        lrows.append(None); continue
    iv = timeout_interval_from_trials(Lv["trials"], L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=float(R.get("feedback_period_s") or 0.01),
                                      hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"], baseline_loss=int((Lv.get("baseline_command_loss") or {}).get("lost", 0)),
                                      rule="0.1.7", fault_window_s=fault_observation_window(Lv["response_timeout_s"], tau["granularity_allowance_s"]))
    ok = iv["status"] == "formed" and iv["interval_low_s"] > R["resolution_floor_s"]
    lrows.append({"ok": ok, "contains": ok and iv["interval_low_s"] <= d["truth"]["tau_w_s"] <= iv["interval_high_s"], "loss": d["truth"]["loss"],
                  "w": (iv["interval_high_s"] - iv["interval_low_s"]) * 1e3, "released": tau.get("status"), "hi245": ok and iv["interval_high_s"] <= 0.245})
lr = [r for r in lrows if r]
w0 = statistics.median([r["w"] for r in lr if r["loss"] == "none"])
n_contra = sum(1 for r in lr if r["released"] == "inconsistent")
check("v0.1.7 bound: 64/64 fault-policy loss intervals formed and containing; none decides a 245-ms horizon satisfied",
      len(lrows) == 64 and len(lr) == 64 and all(r["ok"] and r["contains"] for r in lr) and not any(r["hi245"] for r in lr) and intex("Every interval is formed and contains \\SI{250}{ms}", sup), f"{len(lr)} formed")
check(f"v0.1.7 bound: median width without loss {w0:.0f} ms; {n_contra} contradictory 0.1.6 intervals become determinate",
      f"{w0:.0f}" == "334" and intex("The median width without loss is \\SI{334}{ms} (0.1.6: \\SI{45}{ms})", sup) and n_contra == 10 and intex("the ten runs whose 0.1.6 intervals were contradictory", sup), f"{w0:.1f}; {n_contra}")
rj = json.load(open(os.path.join(a.repo, "validation", "v0.1.7", "rederivation_v017.json")))["summary"]
av = rj["archived_v013"]
check("v0.1.7 archived: 23/23 contain; fault width/tau median 2.1; overall 0.53; range 0.05-10.7 (validation/v0.1.7)",
      av["formed_v017"] == 23 and av["contain_v017"] == 23 and f"{av['width_over_tau_v017']['fault_median']:.1f}" == "2.1" and intex("median 2.1 times $\\tau_w$")
      and f"{av['width_over_tau_v017']['median']:.2f}" == "0.53" and intex("from a median of 0.14 to 0.53", sup)
      and f"{av['width_over_tau_v017']['min']:.2f}--{av['width_over_tau_v017']['max']:.1f}" == "0.05--10.7" and intex("range 0.05--10.7", sup), json.dumps(av["width_over_tau_v017"]))
check("v0.1.7 archived: horizon claims unchanged (0.1 violated, 0.25 undetermined, 1 s satisfied)",
      [av["fault_horizon_claims"][k][f"v017_fault_horizon_{h}"] for k, h in (("L_horizon_fault_0100", "0.1"), ("L_horizon_fault_0250", "0.25"), ("L_horizon_fault_1000", "1.0"))] == ["violated", "undetermined", "satisfied"]
      and intex("violated, undetermined and satisfied under every rule" + ("." if R14 else ", v0.1.7 included")))
ratios = sorted({r["ratio"] for r in mc if r["probe"] == "frame"})
check("MC accounting: 42 000 frame replicates = 7 ratios x 3 noise models x 2000; Table III shows five ratios",
      len(ratios) == 7 and sum(r["n_rep"] for r in mc if r["probe"] == "frame") == 42000
      and (intex("nor in 42\\,000 v0.1.6 replicates, mixture included") and intex("42\\,000 replicates", sup) if R14 else
           intex("0 of 42\\,000 replicates over the supplement's full grid of seven ratios")), str(ratios))
check("text: RC12 wording (illustrative parameters; stationary arm; ROS 1 scope; SRC v1 contingency)",
      intex("illustrative engineering settings, not task-derived or clinical thresholds") and intex("The pairing requires a still arm" if R14 else "The pairing assumes a stationary arm")
      and intex("the executable validation is ROS~1 only") and intex("counts the primary prediction as not matched") and not intex("reviewer-supplied"))

# ---------------------------------------------------------------- RC13: the confirmatory v0.1.7 campaign and the sensitivity studies,
# recomputed from the raw v0.1.7 records and study outputs, and the RC13 wording
V17 = os.path.join(a.repo, "validation", "v0.1.7")
s17p = os.path.join(a.paper, f"supplement_{a.tag}_v017.tex")
sup17 = open(s17p).read() if os.path.exists(s17p) else ""
from crtk_conformance.liveness import fault_horizon_decision  # noqa: E402
K7 = {os.path.basename(p)[:-5]: json.load(open(p)) for p in glob.glob(os.path.join(V17, "mock", "K", "K17_*.json"))}
k7 = [K7[f"K17_K1_{i}"] for i in range(3) if f"K17_K1_{i}" in K7]
c7 = [r for n, r in K7.items() if "_ctrl_" in n]


def kcf(r):
    return r["scale"]["result"]["estimates"]["command_feedback_consistency"]


check("RC13 K: 9 confirmatory runs (3 K1, 6 controls)", len(K7) == 9 and len(k7) == 3 and len(c7) == 6, str(sorted(K7)))
check("RC13 K1: flag raised, unit withheld (ungated divergent), frame C, command-semantic U, dimensional summary U, assumption contradicted, 3/3",
      all(kcf(r)["flag_non_shared_binding_or_tracking_deficit"] and r["scale"]["result"]["outcome"] == "undetermined"
          and r["scale"]["result"]["estimates"]["unit_outcome_without_consistency_gate"] == "divergent" and r["frame"]["result"]["outcome"] == "conformant"
          and r["verdict_kinds"]["spatial_command_semantic"] == "undetermined" and r["report_summary"]["dimensional"] == "undetermined"
          and r["verdict_kinds"]["shared_binding_assumption"].startswith("contradicted") for r in k7))
check("RC13 K controls: no flag; frame, unit and command-semantic reading conformant, 6/6",
      all(not kcf(r)["flag_non_shared_binding_or_tracking_deficit"] and r["frame"]["result"]["outcome"] == r["scale"]["result"]["outcome"]
          == r["verdict_kinds"]["spatial_command_semantic"] == "conformant" for r in c7))
g7 = [kcf(r)["goal_residual_if"] for r in k7]
ri7 = [r["scale"]["result"]["estimates"]["internal_ratio"]["mean"] for r in k7]
sa7 = [r["scale"]["result"]["estimates"]["scale_anchored"]["mean"] for r in k7]
pe7 = [r["scale"]["result"]["estimates"]["predicted_error_at_workspace_edge_m"]["mean"] for r in k7]
fh7 = [r["frame"]["result"]["estimates"]["spatial_decision"]["ci_high_m"] for r in k7]
cr7 = [kcf(r)["goal_residual_if"]["mean"] for r in c7]
check("RC13 K1 numbers: residual 24.1 mm (>= 23.9 at 95 %), ratio 4.99, anchored 5.52, 452 mm ungated error, frame interval [0, 0.04] mm; controls < 8 um",
      all(f"{x['mean'] * 1e3:.1f}" == "24.1" and x["ci_low"] * 1e3 >= 23.9 for x in g7) and all(f"{x:.2f}" == "4.99" for x in ri7)
      and all(f"{x:.2f}" == "5.52" for x in sa7) and all(f"{x * 1e3:.0f}" == "452" for x in pe7) and max(fh7) * 1e3 <= 0.04 and max(cr7) < 8e-6
      and intex("\\SI{24.1}{mm}") and intex("\\SI{23.9}{mm}") and intex("4.99") and intex("5.52") and intex("\\SI{452}{mm}") and intex("[0, 0.04]")
      and intex("\\SI{8}{\\micro m}") and intex("matched all five pre-registered predictions"),
      f"res {[round(x['mean'] * 1e3, 3) for x in g7]}; frame hi {max(fh7) * 1e3:.4f} mm; ctrl {max(cr7) * 1e6:.2f} um")
# L
L7, fail7 = {}, []
for p in glob.glob(os.path.join(V17, "mock", "L", "L17_*.json")):
    d = json.load(open(p))
    if "liveness" not in d["result"]["observations"]:
        fail7.append(os.path.basename(p)); continue
    L7[os.path.basename(p)[:-5]] = d
sf = glob.glob(os.path.join(V17, "mock", "L_failed_startup", "*.json"))
check("RC13 L: 64 runs with a liveness observation, 8 per cell; start-up failures archived apart and not pooled",
      len(L7) == 64 and not fail7 and all(sum(1 for n in L7 if f"_{pol}_{lo}_" in n) == 8 for pol in ("hold", "fault250") for lo in ("none", "iid2", "iid5", "ge5")),
      f"{len(L7)} runs; failed in pool {fail7}; archived start-up failures {[os.path.basename(x) for x in sf]}")
hold7 = [d for n, d in L7.items() if "_hold_" in n]
fault7 = {n: d for n, d in L7.items() if "_fault250_" in n}
check("RC13 L hold: 31/32 held through the range and satisfied; no trip class",
      len(hold7) == 32 and 31 == sum(d["result"]["observations"]["liveness"]["stop_class"] == "held_through_range"
                               and d["result"]["estimates"]["sub_verdicts"]["stop_behaviour"] == "satisfied" for d in hold7)
      and not any(d["result"]["observations"]["liveness"]["stop_class"] in ("rejected", "faulted", "drifted") for d in hold7))
fr7 = {}
for n, d in fault7.items():
    tau = d["result"]["observations"]["liveness"].get("tau_w_estimate_s") or {}
    ce = tau.get("conditional_estimate_s") or {}
    tt = d["truth"]["tau_w_s"]
    ok = tau.get("status") == "ok"
    fr7[n] = {"ok": ok, "contains": ok and tau["interval_low_s"] <= tt <= tau["interval_high_s"], "w": (tau["interval_high_s"] - tau["interval_low_s"]) * 1e3 if ok else None,
              "loss": d["truth"]["loss"], "sv": d["result"]["estimates"]["sub_verdicts"]["stop_behaviour"],
              "d245": fault_horizon_decision(tau, 0.245, 1.0 / 100 + 0.005)[0] if ok else None,
              "cond": ce.get("status"), "cw": (ce["interval_high_s"] - ce["interval_low_s"]) * 1e3 if ce.get("status") == "formed" else None,
              "cc": (ce["interval_low_s"] <= tt <= ce["interval_high_s"]) if ce.get("status") == "formed" else None,
              "cd245": fault_horizon_decision(dict(ce, status="ok"), 0.245, 1.0 / 100 + 0.005)[0] if ce.get("status") == "formed" else None,
              "lost": ((d["result"]["observations"]["liveness"].get("baseline_command_loss") or {}).get("lost") or 0)}
nf7 = sum(r["ok"] for r in fr7.values()); nc7 = sum(r["contains"] for r in fr7.values())
check("RC13 L fault: every interval formed and containing 250 ms; stop sub-verdict satisfied in all; no 245-ms horizon decided satisfied",
      len(fr7) == 32 and nf7 == 32 and nc7 == 32 and all(r["sv"] == "satisfied" for r in fr7.values()) and not any(r["d245"] == "satisfied" for r in fr7.values()),
      f"{nc7}/{nf7} of {len(fr7)}")
w7 = {lo: statistics.median([r["w"] for r in fr7.values() if r["loss"] == lo and r["ok"]]) for lo in ("none", "iid2", "iid5", "ge5")}
cw7 = statistics.median([r["cw"] for r in fr7.values() if r["loss"] == "none" and r["cw"] is not None])
ce_ex = sorted(n for n, r in fr7.items() if r["cc"] is False)
ce_245 = sorted(n for n, r in fr7.items() if r["cd245"] == "satisfied")
check("RC13 L fault: a conditional estimate reported with every formed interval", all(r["cond"] is not None for r in fr7.values() if r["ok"]))
print("RC13 L widths (median, ms):", {k: round(v, 1) for k, v in w7.items()}, "conditional none:", round(cw7, 1), "cond excluding:", ce_ex, "cond 245 satisfied:", ce_245)
lt7p = os.path.join(a.paper, "loss_main_v018.tex" if R14 else "loss_main_v017.tex")
lt7 = open(lt7p).read() if os.path.exists(lt7p) else ""
h7 = " & ".join(str(sum(1 for n, d in L7.items() if f"_hold_{lo}_" in n and d["result"]["observations"]["liveness"]["stop_class"] == "held_through_range"))
               + ("$^f$" if any(f"_hold_{lo}_" in n and d["result"]["observations"]["liveness"]["stop_class"] == "not_observable" for n, d in L7.items()) else "") for lo in ("none", "iid2", "iid5", "ge5"))
f7 = " & ".join(f"{sum(r['contains'] for r in fr7.values() if r['loss'] == lo)}/{sum(r['ok'] and not r['contains'] for r in fr7.values() if r['loss'] == lo)}/{sum(not r['ok'] for r in fr7.values() if r['loss'] == lo)}" for lo in ("none", "iid2", "iid5", "ge5"))
_r7 = "sound$^d$" if R14 else "0.1.7$^d$"
check("RC13 loss table: v0.1.7 rows equal the raw recomputation", f"{_r7} & {h7} \\\\" in lt7 and f"{_r7} & {f7} \\\\" in lt7, f"hold {h7}; fault {f7}")
for rule in (("0.1.6",) if R14 else ("0.1.5", "0.1.6")):
    hold = " & ".join(f"{cells[(rule, 'hold', lo)]['hold']}" + (f" ({cells[(rule, 'hold', lo)]['false_trip']})" if cells[(rule, 'hold', lo)]['false_trip'] else "") for lo in ("none", "iid2", "iid5", "ge5"))
    fault = " & ".join(f"{cells[(rule, 'fault', lo)]['contains']}/{cells[(rule, 'fault', lo)]['excludes']}/{cells[(rule, 'fault', lo)]['other']}" for lo in ("none", "iid2", "iid5", "ge5"))
    check(f"RC13 loss table rows for rule {rule} (v0.1.6 campaign)", f"{rule} & {hold} \\\\" in lt7 and f"{rule} & {fault} \\\\" in lt7, f"hold {hold}; fault {fault}")
wtxt = f"\\SI{{{w7['none']:.0f}}}{{ms}}"
if R14:
    _wr = "width$^e$ (ms) & 0.1.6 & 46 & 333 & 243 & 275 \\\\\n & sound & " + " & ".join(f"{w7[lo]:.0f}" for lo in ("none", "iid2", "iid5", "ge5")) + " \\\\"
    check(f"RC14 loss table: sound-bound median widths {[round(w7[lo]) for lo in ('none', 'iid2', 'iid5', 'ge5')]} ms from the raw records", _wr in lt7, _wr)
else:
    check(f"RC13 text: median v0.1.7 width without loss {wtxt}, conditional {cw7:.0f} ms", intex(wtxt) and intex(f"\\SI{{{cw7:.0f}}}{{ms}}"), f"{w7['none']:.1f} / {cw7:.1f}")
an = json.load(open(os.path.join(V17, "analysis", "confirmatory_v017.json")))
check("RC13 analysis output: 11 of 13 pre-registered predictions matched; L-1 and L-2 deviate (analyze_v017.py)", len(an["predictions"]) == 13
      and sorted(p["id"] for p in an["predictions"] if not p["matched"]) == ["L-1", "L-2"] and intex("Eleven of the thirteen predictions matched", sup17),
      str([p["id"] for p in an["predictions"] if not p["matched"]]))

bs = json.load(open(os.path.join(V17, "studies", "boundary_sensitivity.json")))["rows"]
bsi = {(r["probe"], r["model"], r["ratio"]): r for r in bs}
bst = open(os.path.join(a.paper, "boundary_sens_v017.tex")).read() if os.path.exists(os.path.join(a.paper, "boundary_sens_v017.tex")) else ""
cells_ok = all(f"{pct(bsi[(pr, m, rt)]['false_C' if rt == '1.05' else 'false_D'], bsi[(pr, m, rt)]['n_rep'], 1)}/{pct(bsi[(pr, m, rt)]['U'], bsi[(pr, m, rt)]['n_rep'], 0)}" in bst
               for pr in ("frame", "scale") for m in ("gauss", "ar1-0.5", "ar1-0.9", "bias", "drift", "t3") for rt in ("0.95", "1.00", "1.05"))
scale_max = max(bsi[("scale", m, "1.00")]["false_D"] / bsi[("scale", m, "1.00")]["n_rep"] for m in ("gauss", "ar1-0.5", "ar1-0.9", "bias", "drift", "t3"))
f9 = bsi[("frame", "ar1-0.9", "1.00")]
frame_other = sum(bsi[("frame", m, rt)]["false_C"] + bsi[("frame", m, rt)]["false_D"] for m in ("gauss", "bias", "drift", "t3") for rt in ("0.95", "1.00", "1.05"))
check("RC13 boundary sensitivity: table cells from the study output; scale <= 2.8 % false D at epsilon in every model; frame 11 % at AR 0.9 (coverage 0.82), none under bias/drift/t3",
      len(bs) == 36 and cells_ok and pct(round(scale_max * 2000), 2000, 1) == "2.8" and f"{100 * f9['false_D'] / f9['n_rep']:.0f}" == "11" and f"{f9['coverage']:.3f}" == "0.815"
      and frame_other == 0 and ((intex("at most 2.8\\% in every model", sup17) and intex("in 11.1\\%", sup17) and intex("coverage 0.815", sup17)) if R14 else
                                (intex("at or below 2.8\\% false divergence") and intex("11\\% of replicates at $\\varepsilon$, with coverage 0.815"))),
      f"scale max {scale_max:.4f}; frame AR0.9 {f9['false_D']}/{f9['n_rep']}, cov {f9['coverage']}")
an_ = {(r["factor"], r["level"]): r for r in json.load(open(os.path.join(V17, "studies", "anchor_sensitivity.json")))["rows"]}
ok_c = an_[("c", 0.02)]["gate_pass"] == 0 and an_[("c", 0.05)]["gate_pass"] == 0 and an_[("c", 0.1)]["gate_pass"] == 0 and an_[("c", 0.01)]["gate_pass"] == 1 and an_[("c", 0.005)]["gate_pass"] == 1 \
    and abs(an_[("c", 0.01)]["lambda_bias_pct_median"]) < 0.1
n05 = an_[("sigma_t_mm", 0.05)]["gate_pass"]
l97 = an_[("L_impl_ratio", 0.97)]
l99 = an_[("L_impl_ratio", 9.0 / 9.1)]
check("RC13 anchor sensitivity: coupling >= 2 % fails the gates, <= 1 % passes unbiased; 0.05-mm noise fails 83 %; 3 % length mismatch false D 23 % at 1 mm; 1.1 % covered",
      ok_c and f"{100 * (1 - n05):.0f}" == "83" and f"{100 * l97['eps1mm_divergent']:.0f}" == "23" and l97["gate_pass"] == 1 and l99["coverage_widened"] >= 0.95
      and ((intex("2\\% or more failed the axes-angle gate in every replicate", sup17) and intex("only 17\\% passed", sup17)
            and intex("falsely divergent in 23\\% of replicates", sup17) and intex("(single-joint anchor: 23\\%)", sup18)) if R14 else
           (intex("from 2\\% of the step it failed the axis-angle gate in every replicate") and intex("failed the gates in 83\\% of replicates")
            and intex("a 3\\% mismatch gave false divergence at \\SI{1}{mm} in 23\\% of replicates"))), f"noise pass {n05}; L0.97 D {l97['eps1mm_divergent']}; L9.0 cov {l99['coverage_widened']}")
import subprocess
try:
    rc = subprocess.run(["git", "-C", a.repo, "log", "--format=%h", "-1", "--", "validation/v0.1.7/RESULTS.md"], capture_output=True, text=True).stdout.strip()
except Exception:
    rc = ""
check("RC13 chronology: the results commit in the supplement is the commit that added the confirmatory results", bool(rc) and f"\\code{{{rc}}}" in (sup18 if R14 else sup17), rc)
if not R14:
    check("RC13 abstract: 32 timeouts contained, two excluded by the earlier bound; no revision-history wording",
          intex("contained all 32 injected timeouts, where an earlier bound excluded two") and nc7 == 32 and len(ce_ex) == 2
          and not re.search(r"\bnow (withholds|contains)\b", tex))
    check("RC13 wording: v0.1.7 is the evaluated system; no 'v0.1.6 throughout' or 'since v0.1.7'; Table II in v0.1.7",
          intex("The evaluated system is \\code{crtk-conformance} v0.1.7") and not intex("v0.1.6 throughout") and not intex("since v0.1.7")
          and intex("\\caption{Decision evidence in v0.1.7."))
    check("RC13 wording: chronology in four steps; evidence taxonomy (i)-(iv); one-sided gate",
          intex("The evidence was produced in four steps") and intex("(iv) Independent physical anchors") and intex("an unraised flag does not establish \\tA{}")
          and intex("An unraised flag is weak evidence for \\tA"))
else:
    check("RC14 abstract: all 32 injected timeouts contained; no revision-history wording", intex("all 32 injected timeouts") and nc7 == 32
          and not re.search(r"\bnow (withholds|contains)\b", tex))
    check("RC14 wording: v0.1.8 is the evaluated system; Table II in v0.1.8; the conditional estimate excluded tau_w in 2 of 32 confirmatory runs",
          intex("The evaluated system is \\code{crtk-conformance} v0.1.8") and intex("\\caption{Decision evidence in v0.1.8.") and not intex("since v0.1.7")
          and len(ce_ex) == 2 and intex("excluded the timeout in 2 of 32 runs"))
    check("RC14 wording: rule revisions tested on fresh runs; evidence taxonomy (i)-(iv); one-sided gate",
          intex("Each revision of a decision rule was introduced after an external review") and intex("(iv) Independent physical anchors")
          and intex("an unraised flag does not establish \\tA{}") and intex("An unraised flag is weak evidence for \\tA"))
check("RC13 wording: TOST, observational equivalence and the oracle problem are cited",
      intex("\\cite{schuirmann1987tost}") and intex("\\cite{bellman1970structural}") and intex("\\cite{barr2015oracle}"))
check("RC13 wording: freshness naming (Table II row; no rate-conformance verdict)",
      intex("Freshness & Inferred source age") and not intex("Rate & Inferred source age") and not intex("rate conformance") and intex("freshness requirement"))
check("RC13 Table V: primary (contingency) scoring and client definitions",
      (intex("Primary (cont.)") and intex("0/3 (3/3)") and intex("3/9 (6/9)") if R14 else intex("Primary (contingency)") and intex("0/2 (2/2)") and intex("0/1 (1/1)"))
      and intex("\\emph{dVRK-authored} client declares") and intex("\\emph{SRC-authored} client declares"))
check("RC13 deviation reported: one undetermined hold run (calibration lost 5 of 30)",
      intex("its calibration lost 5 of 30 commands") and intex("answered 25 of 30 commands", sup17)
      and sum(1 for d in hold7 if d["result"]["observations"]["liveness"]["stop_class"] == "not_observable") == 1
      and any(30 - d["result"]["observations"]["resolution"]["latency_probes_responded"] == 5 for d in hold7 if d["result"]["observations"]["liveness"]["stop_class"] == "not_observable"))

# ---------------------------------------------------------------- RC14: the v0.1.8 campaigns (recomputed from the raw records), the offline
# studies (from their outputs) and the RC14 wording
if R14:
    V18 = os.path.join(a.repo, "validation", "v0.1.8")
    # F, raw records
    F18 = {}
    for p in glob.glob(os.path.join(V18, "mock", "F", "F18_*.json")):
        d = json.load(open(p)); tr = d["truth"]; r = d["result"]; cpn = r["observations"].get("correlation_plan") or {}
        o = r["outcome"]; tc = tr["truth_conformant"]
        cl = "U" if o == "undetermined" else (("cC" if tc else "fC") if o == "conformant" else ("fD" if tc else "cD"))
        F18.setdefault((tr["procedure"], tr["phi"], tr["ratio"]), []).append({"cl": cl, "ok": cpn.get("ok"), "sp": cpn.get("spacing_samples"), "tau": cpn.get("tau_int_samples")})
    nF = lambda k, c=None: sum(1 for x in F18.get(k, []) if c is None or x["cl"] == c)
    check("RC14 F: 160 runs, 20 per cell", len(F18) == 8 and all(len(v) == 20 for v in F18.values()), str({k: len(v) for k, v in F18.items()}))
    g99 = F18[("0.1.8", 0.99, "0.95")] + F18[("0.1.8", 0.99, "1.00")]
    wh = sum(x["ok"] is False for x in g99)
    tres = [x["tau"] for x in g99 if x["ok"]]
    check("RC14 F: back to back 4/20 false D at epsilon (8 decided); guard 0 false of 40, 23 withheld, all 20 at epsilon U; iid 20/20 C at 0.95 both; spacing 5 in all 40 iid guard runs",
          nF(("0.1.7", 0.99, "1.00"), "fD") == 4 and 20 - nF(("0.1.7", 0.99, "1.00"), "U") == 8 and not any(x["cl"] in ("fC", "fD") for x in g99) and wh == 23
          and nF(("0.1.8", 0.99, "1.00"), "U") == 20 and nF(("0.1.7", 0.0, "0.95"), "cC") == 20 and nF(("0.1.8", 0.0, "0.95"), "cC") == 20
          and all(x["sp"] == 5 for k in (("0.1.8", 0.0, "0.95"), ("0.1.8", 0.0, "1.00")) for x in F18[k])
          and intex("4 of 20 runs at $\\varepsilon$ were falsely divergent, half of the determinate verdicts") and intex("none of 40 was false and 23 were withheld"),
          f"withheld {wh}")
    check(f"RC14 F: resolved guard runs underestimated tau_int ({min(tres):.0f}--{max(tres):.0f} samples)",
          f"{min(tres):.0f}--{max(tres):.0f}" == "31--137" and intex("(31--137 samples)") and intex("from 31 to 137 samples", sup18))
    an18 = json.load(open(os.path.join(V18, "analysis", "campaigns_v018.json")))
    pr18 = {p_["id"]: p_["matched"] for p_ in an18["predictions"]}
    check("RC14 analysis output: every pre-registered prediction of F, D and S", sorted(pr18) == ["D-1", "D-2", "F-1", "F-2", "F-3", "F-4", "F-5", "F-6", "S-1", "S-2", "S-3"],
          str(pr18))
    check("RC14 F and D predictions matched (analyze_v018.py)", all(pr18[k] for k in pr18 if k[0] in "FD") and intex("matched all six pre-registered predictions")
          and intex("matching both pre-registered predictions"), str({k: v for k, v in pr18.items() if k[0] in "FD"}))
    # D, raw records
    D18 = {}
    for p in glob.glob(os.path.join(V18, "dvrk_sim", "D", "runs", "*", "*.json")):
        if p.endswith(".bring_up.json"):
            continue
        for r in json.load(open(p)).get("probes") or []:
            if r.get("probe") in ("FrameSemanticsProbe", "GeometryAnchorProbe"):
                D18.setdefault(os.path.basename(p)[:-5], []).append(r)
    dg = [r["estimates"]["geometry_anchor"] for k, v in D18.items() if k.startswith("U") for r in v]
    check("RC14 D: 27 runs; frame plans deterministic; anchor gates passed with d_int 9.1000 mm in all 9",
          sum(len(v) for v in D18.values()) == 27 and all(r["observations"]["correlation_plan"]["deterministic"] for k, v in D18.items() if k.startswith("F") for r in v)
          and len(dg) == 9 and all(x["gates_passed"] and abs(x["d_int_mean_if"] - 0.0091) < 1e-9 for x in dg) and intex("\\SI{9.1000}{mm}"))
    # studies
    fgs = {(r["sigma_mm"], r["phi"], r["ratio"]): r for r in json.load(open(os.path.join(V18, "studies", "frame_guard.json")))["rows"]}
    v17f = lambda sg, ph, rt: fgs[(sg, ph, rt)]["v017_false_rate"]
    v18max = max(r["v018_false_rate"] for r in fgs.values())
    v18off = max(r["v018_false_rate"] for r in fgs.values() if r["ratio"] != "1.00")
    whf = [fgs[(0.02, 0.99, rt)]["withheld"]["unresolved"] / 1000 for rt in ("0.95", "1.00", "1.05")]
    check("RC14 guard study: back to back 20 % (0.02 mm) and 22 % (0.1 mm) false D at epsilon, phi 0.99; guard <= 0.2 %, 0 off the boundary; 37--41 % withheld; decided 82 % -> 26 %",
          f"{100 * v17f(0.02, 0.99, '1.00'):.0f}" == "20" and f"{100 * v17f(0.1, 0.99, '1.00'):.0f}" == "22" and v18max <= 0.002 and v18off == 0
          and f"{100 * min(whf):.0f}--{100 * max(whf):.0f}" == "37--41" and f"{100 * fgs[(0.02, 0.99, '0.95')]['v017_decided']:.0f}" == "82"
          and f"{100 * fgs[(0.02, 0.99, '0.95')]['v018_decided']:.0f}" == "26" and f"{fgs[(0.1, 0.99, '0.95')]['v017_false_given_decided']:.2f}" == "0.34"
          and intex("in 20\\% of replicates at \\SI{0.02}{mm} and 22\\% at \\SI{0.1}{mm}") and intex("at most 0.2\\% in every cell") and intex("37--41\\%") and intex("from 82\\% to 26\\%"),
          f"max guard {v18max}; withheld {whf}")
    oc = {(r["probe"], r["sigma_mm"], r["trials"], r["ratio"]): r for r in json.load(open(os.path.join(V18, "studies", "operating_characteristic.json")))["rows"]}
    ocf = lambda *k: oc[k]
    fr_false = sum(r["false_C"] + r["false_D"] for k, r in oc.items() if k[0] == "frame")
    c1 = f"{100 * ocf('scale', 0.02, 9, '1.00')['false']:.1f}" == "2.2" and f"{100 * ocf('scale', 0.1, 9, '1.00')['false']:.1f}" == "3.3"
    c2 = f"{100 * ocf('scale', 0.02, 9, '1.05')['decided']:.0f}--{100 * ocf('scale', 0.02, 9, '0.95')['decided']:.0f}" == "43--48" \
        and ocf('scale', 0.02, 9, '0.95')['false'] == 0 and ocf('scale', 0.02, 9, '1.05')['false'] == 0
    c3 = max(ocf('scale', 0.1, 9, rt)['decided'] for rt in ("0.90", "0.95", "0.98", "1.00", "1.02", "1.05", "1.10")) <= 0.13 \
        and f"{100 * ocf('scale', 0.1, 9, '0.95')['false_given_decided']:.0f}--{100 * ocf('scale', 0.1, 9, '1.05')['false_given_decided']:.0f}" == "13--21"
    c4 = f"{100 * ocf('frame', 0.1, 10, '1.05')['decided']:.1f}--{100 * ocf('frame', 0.1, 10, '0.95')['decided']:.1f}" == "0.4--0.5" \
        and f"{100 * ocf('frame', 0.1, 40, '1.05')['decided']:.0f}--{100 * ocf('frame', 0.1, 40, '0.95')['decided']:.0f}" == "87--88" \
        and ocf('frame', 0.02, 10, '0.95')['decided'] == 1 and ocf('frame', 0.02, 10, '1.05')['decided'] == 1
    check("RC14 operating characteristic: frame never false; scale 2.2/3.3 % at epsilon; 43--48 % decided at 5 %; <= 13 % decided and 13--21 % false among decided at 0.1 mm; frame 0.4--0.5 % -> 87--88 % with 40 trials",
          fr_false == 0 and c1 and c2 and c3 and c4 and intex("2.2\\% and 3.3\\%") and intex("43--48\\%") and intex("13--21\\%") and intex("0.4--0.5\\%") and intex("87--88\\%"),
          f"{c1} {c2} {c3} {c4}")
    import subprocess as _sp
    _tmp = os.path.join(a.paper, "build", "tables_check")
    _sp.run([sys.executable, os.path.join(a.repo, "validation", "tables_v018.py"), _tmp], capture_output=True)
    same = {f: open(os.path.join(_tmp, f)).read() == open(os.path.join(a.paper, f)).read() for f in os.listdir(_tmp) if os.path.exists(os.path.join(a.paper, f))}
    check("RC14 tables: every v0.1.8 table in the paper equals a fresh run of tables_v018.py", same and all(same.values()), str(same))
    ap_ = {(r["factor"], r["level"]): r for r in json.load(open(os.path.join(V18, "studies", "anchor_poe.json")))["rows"]}
    check("RC14 anchor study: noise 0.02 mm all pass, 0.05 mm 5 %, 0.1 mm 0 %; reach 0.44 unchanged; coupling 5/10 % fail 8/14 %; kappa 0.98 -2 % bias passes, 0.95 false D 100 %; 3 % length false D 60 %",
          ap_[("sigma_t_mm", 0.02)]["gate_pass"] == 1 and f"{100 * ap_[('sigma_t_mm', 0.05)]['gate_pass']:.0f}" == "5" and ap_[("sigma_t_mm", 0.1)]["gate_pass"] == 0
          and ap_[("reach", 0.44)]["gate_pass"] == 1 and ap_[("reach", 0.44)]["eps5mm_conformant"] == 1
          and f"{100 * (1 - ap_[('coupling', 0.05)]['gate_pass']):.0f}/{100 * (1 - ap_[('coupling', 0.1)]['gate_pass']):.0f}" == "8/14"
          and ap_[("kappa", 0.98)]["gate_pass"] == 1 and f"{ap_[('kappa', 0.98)]['lambda_bias_pct_median']:.1f}" == "-2.0" and ap_[("kappa", 0.95)]["eps1mm_divergent"] == 1
          and f"{100 * ap_[('L_impl_ratio', 0.97)]['eps1mm_divergent']:.0f}" == "60" and ap_[("L_impl_ratio", 0.97)]["gate_pass"] == 1
          and intex("at \\SI{0.05}{mm} they passed in 5\\%") and intex("8--14\\%") and intex("in 60\\%"))
    dl = json.load(open(os.path.join(V18, "studies", "deadline_resolution.json")))
    sm = dl["summary"]["confirmatory"]
    arch_f = [r for r in dl["archived_rows"] if r["mode"] == "fault"]
    arch_r = [r for r in dl["archived_rows"] if r["mode"] != "fault"]
    wait = sorted(r["sound_confirm_margin_s"] for r in arch_f if r["run"] not in ("L_delayed", "L_slowfb"))
    check("RC14 deadlines: sound confirm 310 / 125 / 249 ms, refute 18 ms, conditional 21 ms; archived 0.51--0.52 s, 1.77 s delayed; drift band <= 36 ms; replay consistent",
          [f"{sm[k]['sound_confirm_margin_median_ms']:.0f}" for k in ("none", "iid5", "ge5")] == ["310", "125", "249"]
          and f"{sm['none']['sound_refute_margin_median_ms']:.0f}" == "18" and f"{sm['none']['conditional_confirm_margin_median_ms']:.0f}" == "21"
          and f"{wait[0]:.2f}--{wait[-1]:.2f}" == "0.51--0.52" and any(r["run"] == "L_delayed" and f"{r['sound_confirm_margin_s']:.2f}" == "1.77" for r in arch_f)
          and max(r["sound_confirm_margin_s"] + r["sound_refute_margin_s"] for r in arch_r) <= 0.036
          and all(r["replay_at_hi"] == "satisfied" and r["replay_below_hi"] != "satisfied" for r in dl["confirmatory_rows"])
          and intex("$\\tau_w+\\SI{310}{ms}$") and intex("125 and \\SI{249}{ms}") and intex("\\SI{18}{ms} (median)") and intex("$\\tau_w+\\SI{21}{ms}$")
          and intex("0.51--\\SI{0.52}{s}") and intex("\\SI{1.77}{s}") and intex("at most \\SI{36}{ms} wide"), f"archived waits {wait[0]:.3f}..{wait[-1]:.3f}")
    # S, raw records
    S18 = {}
    for v in ("v1", "v2"):
        for p in glob.glob(os.path.join(V18, "src_live", f"live-src-{v}", "launch*", "*_geometry_*.json")):
            for r in json.load(open(p)).get("probes") or []:
                if r.get("probe") == "GeometryAnchorProbe":
                    S18.setdefault(v, []).append((os.path.basename(p)[:-5], r["outcome"], r["estimates"].get("geometry_anchor") or {}))
    s1 = S18.get("v1", []); s2 = S18.get("v2", [])
    lam1 = [g_["lambda_hat_m"] for _, _, g_ in s1 if g_.get("lambda_hat_m")]
    lam2 = [g_["lambda_hat_m"] for _, _, g_ in s2 if g_.get("lambda_hat_m")]
    check("RC14 S: 9 runs per release", len(s1) == 9 and len(s2) == 9, f"{len(s1)} / {len(s2)}")
    s1l = {}
    for p in glob.glob(os.path.join(V18, "src_live", "live-src-v1", "launch*", "*_geometry_*.json")):
        for r in json.load(open(p)).get("probes") or []:
            if r.get("probe") == "GeometryAnchorProbe":
                s1l.setdefault(os.path.basename(os.path.dirname(p)), []).append((r["outcome"], r["estimates"].get("geometry_anchor") or {}))
    l1 = s1l.get("launch1", []); l23 = s1l.get("launch2", []) + s1l.get("launch3", [])
    lam23 = [g_["lambda_hat_m"] for _, g_ in l23]
    err1 = [g_["predicted_error_ci_m"] for _, g_ in l1]
    check("RC14 S-1 (deviation): SRC v1.0.0 launch 1 gates passed and divergent 3/3 (lambda 0.1011); launches 2-3 gates failed and undetermined 6/6 (lambda 0.096-0.101); no false verdict",
          len(l1) == 3 and all(o == "divergent" and g_["gates_passed"] and f"{g_['lambda_hat_m']:.4f}" == "0.1011" for o, g_ in l1)
          and len(l23) == 6 and all(o == "undetermined" and not g_["gates_passed"] for o, g_ in l23)
          and f"{min(lam23):.3f}--{max(lam23):.3f}" == "0.096--0.101" and not pr18.get("S-1")
          and f"{min(e[0] for e in err1) * 1e3:.1f}--{max(e[1] for e in err1) * 1e3:.1f}" == "89.7--90.0"
          and intex("in one of three launches") and intex("0.096--\\SI{0.101}{m} per unit") and intex("89.7--\\SI{90.0}{mm}") and intex("3/9 (6/9)"),
          f"lambda 2-3 {min(lam23):.4f}..{max(lam23):.4f}")
    P2 = {"src_client_geometry_1mm": "undetermined", "src_client_geometry_5mm": "conformant", "dvrk_client_geometry_1mm": "undetermined"}
    check("RC14 S-2: SRC v2.0.0 gates passed; SRC 1 mm U, 5 mm C, dVRK 1 mm U in 9/9; lambda 1.0111", len(s2) == 9 and pr18.get("S-2")
          and all(o == P2[n] and g_.get("gates_passed") and f"{g_['lambda_hat_m']:.4f}" == "1.0111" for n, o, g_ in s2),
          f"lambda {min(lam2) if lam2 else None}..{max(lam2) if lam2 else None}")
    fs = sorted(glob.glob(os.path.join(V18, "src_live", "live-src-v2", "failed_startup_launch2", "*_geometry_*.json")))
    check("RC14 S: the v2.0.0 start-up failure (launch 2) is archived apart, not pooled, and was repeated",
          len(fs) == 3 and all(any("measured_cp missing" in str(x) for x in (r.get("notes") or [])) for p in fs for r in json.load(open(p))["probes"] if r.get("probe") == "GeometryAnchorProbe")
          and os.path.exists(os.path.join(V18, "src_live", "live-src-v2", "launch2", "resting_trace.json")) and intex("repeated", sup18))
    trj = json.load(open(os.path.join(V18, "analysis", "src_traces_v018.json")))["rows"]
    tv1 = {r["trace"].split("/")[-2]: r for r in trj if r["version"] == "v1"}
    check("RC14 S-3 and traces: >= 1000 samples in all 6; v1 yaw chatter 0.038/0.349/0.116 rad; averaging offset 0.65 mm in launch 2; v2 constant",
          pr18.get("S-3") and len(trj) == 6 and all(r["samples"] >= 1000 for r in trj)
          and [f"{tv1[k]['yaw_sd_rad']:.3f}" for k in ("launch1", "launch2", "launch3")] == ["0.038", "0.349", "0.116"]
          and f"{tv1['launch2']['window_average_offset_mm_median']:.2f}" == "0.65" and all(r["plan"]["deterministic"] for r in trj if r["version"] == "v2")
          and intex("0.35 and \\SI{0.12}{rad}") and intex("\\SI{0.65}{mm}"))
    try:
        rc18 = _sp.run(["git", "-C", a.repo, "log", "--diff-filter=A", "--format=%h", "-1", "--", "validation/v0.1.8/RESULTS.md"], capture_output=True, text=True).stdout.strip()
    except Exception:
        rc18 = ""
    check("RC14 chronology: the v0.1.8 results commit in the supplement is the commit that added RESULTS.md", bool(rc18) and f"\\code{{{rc18}}}" in sup18, rc18)
    sj = json.load(open(os.path.join(V18, "analysis", "single_joint_on_S_v018.json")))["rows"]
    sj1 = [r for r in sj if r["release"] == "v1" and "/launch1/" in r["run"]]
    sj23 = [r for r in sj if r["release"] == "v1" and "/launch1/" not in r["run"]]
    sjl = [r["single_joint_lambda_hat_m"] for r in sj23]
    check("RC14 exploratory: single-joint estimator on the S poses: v1 launch 1 divergent 2/3; launches 2-3 undetermined 6/6 with lambda 0.105-0.125; v2 agrees 9/9; stated as exploratory",
          sj and sum(r["single_joint_outcome"] == "divergent" for r in sj1) == 2 and len(sj1) == 3 and all(r["single_joint_outcome"] == "undetermined" for r in sj23)
          and f"{min(sjl):.3f}--{max(sjl):.3f}" == "0.105--0.125"
          and all(r["single_joint_outcome"] == r["joint_space_outcome"] for r in sj if r["release"] == "v2")
          and intex("also decided divergent in two of the three runs") and intex("(exploratory, after the campaign)", sup18) and intex("0.105--\\SI{0.125}{m}"),
          f"{min(sjl):.4f}..{max(sjl):.4f}")
    pbh = json.load(open(os.path.join(V18, "analysis", "poe_scipy_budget_host.json")))
    pbc = json.load(open(os.path.join(V18, "analysis", "poe_scipy_budget_container.json")))
    lam_ok = max(abs(x["refit_lambda"] / x["archived_lambda"] - 1) for x in pbh["campaign_refits"] if x["archived_gates"])
    c10 = {r["factor"] + str(r["level"]): r["gate_pass"] for r in pbc["study_rows"]}; h10 = {r["factor"] + str(r["level"]): r["gate_pass"] for r in pbh["study_rows"]}
    check("RC14 SciPy budget: 81/135 campaign fits stopped in the image; converged refit (host) keeps every gate outcome and verdict, lambda to 7e-8; study 10 % coupling 89 % -> 76 %",
          pbc["campaign_fits_stopped_at_budget"] == 81 and pbc["campaign_fits"] == 135 and pbh["campaign_fits_stopped_at_budget"] == 0 and pbh["campaign_outcomes_equal"]
          and lam_ok < 7e-8 and f"{100 * h10['coupling0.1']:.0f}/{100 * c10['coupling0.1']:.0f}" == "89/76" and not any(r["eps1mm_divergent"] or r["eps5mm_divergent"] for r in pbc["study_rows"] + pbh["study_rows"])
          and intex("changed no verdict") and intex("from 89\\% to 76\\%", sup18) and intex("81 of the 135 fits", sup18),
          f"lambda {lam_ok:.1e}; {pbh['scipy']} vs {pbc['scipy']}")
    check("RC14 wording: v0.1.8 in the abstract-level claims; freshness tied to its observation model; recommendation to timestamp state transitions",
          intex("a limit of that observation model rather than of freshness testing") and intex("timestamp state transitions") and intex("Decidable deadlines")
          and intex("Correlated trials") and intex("Decision rate and error among determinate verdicts"))

# ---------------------------------------------------------------- abstract length
ab = re.search(r"\\begin\{abstract\}(.*?)\\end\{abstract\}", tex, re.S).group(1)
nw = len(ab.replace("\\%", "%").split())
check("abstract <= 200 words", nw <= 200, f"{nw} words")

ok = all(c["ok"] for c in checks)
json.dump({"all_passed": ok, "checks": checks}, open(os.path.join(a.paper, "reporting_checks_v016.json"), "w"), indent=1)
print(f"{sum(c['ok'] for c in checks)}/{len(checks)} checks passed")
sys.exit(0 if ok else 1)
