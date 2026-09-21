#!/usr/bin/env python3
"""RC9/RC10 reporting checks: every number of the v0.1.6 tables and the key v0.1.6 numbers of the text are recomputed
from the raw v0.1.6 archive (probe reports and run records, not the analysis outputs) and compared with the manuscript
sources.  Standard library only; reads the archive and the manuscript directory; writes reporting_checks_v016.json there.

RC10 (review of the RC9 copy): the live interval coverage is recomputed with explicit missing-value tests (RC9's
analysis turned a legitimate 0.0 lower end into 1 and reported 0.83--0.90 instead of 0.93--1.00; RC9's checks had not
covered coverage), the live boundary counts are checked against the supplement's text instead of only printed, the
analysis output B_live_boundary.csv is cross-checked against the raw recomputation, and the RC10 wording corrections
are checked.

RC12: the counts of the dVRK-sim text are checked as cases and launches, and the post hoc v0.1.7 re-derivation (consistency
gate; sound fault bound) is recomputed from the raw archive with the package's ROS-free decision functions and compared
with the text and with validation/v0.1.7/.

Usage: python3 validation/verify_reporting_v016.py [--repo .] [--paper <dir>] [--tag rc12]
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
ap.add_argument("--tag", default="rc12", help="manuscript tag: reads manuscript_<tag>.tex and supplement_<tag>_campaigns.tex")
a = ap.parse_args()
V = os.path.join(a.repo, "validation", "v0.1.6")
checks = []


def check(name, ok, detail=""):
    checks.append({"check": name, "ok": bool(ok), "detail": detail})
    print(("PASS " if ok else "FAIL ") + name + (f" -- {detail}" if detail else ""))


AB = {"conformant": "C", "divergent": "D", "undetermined": "U"}
tex = open(os.path.join(a.paper, f"manuscript_{a.tag}.tex")).read()
sup = open(os.path.join(a.paper, f"supplement_{a.tag}_campaigns.tex")).read()


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
      and intex("All 17 cases matched their predicted outcomes in each of the three launches (51 runs") and intex("all 17 pre-registered cases matched"))
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
check("B live: the only false verdict is one false D, scale at epsilon", sum(c["fC"] + c["fD"] for c in live.values()) == 1 and sc[1]["fD"] == 1
      and intex("for scale, one false divergence occurred in 30 runs at $\\varepsilon$"))
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
      and pct(round(mix["coverage"] * 2000), 2000, 1) == "39.2" and intex("60.9") and intex("39.2"))
lw = json.load(open(os.path.join(V, "reanalysis", "L_liveness_v016_summary.json")))
check("width/tau median 0.14, range 0.03-1.30; 23/23 unchanged under 0.1.6",
      f"{lw['width_over_tau']['median']:.2f}" == "0.14" and f"{lw['width_over_tau']['min']:.2f}" == "0.03" and f"{lw['width_over_tau']['max']:.2f}" == "1.30"
      and lw["formed_v016"] == 23 and lw["contain_v016"] == 23 and not lw["changed_by_016"] and intex("0.03--1.30"))

# ---------------------------------------------------------------- RC10 wording corrections (review of the RC9 copy, points 2-4)
check("text: closed-boundary sentence (divergence erroneous, conformance correct, undetermined an abstention)",
      intex("At the closed boundary, divergence is erroneous; conformance is correct, and undetermined is an abstention.")
      and not intex("only false divergent or undetermined verdicts are possible"))
check("text: scale calibration stated only under the tested Monte Carlo model",
      intex("Under the tested Monte Carlo model") and not intex("The decision is calibrated") and not intex("The scale interval is calibrated", sup)
      and intex("under the simulated trial model", sup))
check("text: repetition could reduce the cost, subject to validation", intex("could reduce that cost, subject to validation") and not intex("would avoid that cost"))
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
check("Table I liveness row states the observed exclusion", e5 == 1 and n5 == 16 and len(row) == 1 and t1 in row[0], t1)
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
      len(diag) == 102 and sorted(changed) == ["K1_0.json", "K1_1.json", "K1_2.json"] and intex("102 archived runs"), f"{len(diag)}; {sorted(changed)}")
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
      len(lrows) == 64 and len(lr) == 64 and all(r["ok"] and r["contains"] for r in lr) and not any(r["hi245"] for r in lr) and intex("all 64 fault-policy intervals contain the timeout"), f"{len(lr)} formed")
check(f"v0.1.7 bound: median width without loss {w0:.0f} ms; {n_contra} contradictory 0.1.6 intervals become determinate",
      f"{w0:.0f}" == "334" and intex("from 45 to \\SI{334}{ms}") and n_contra == 10 and intex("the ten contradictory ones become determinate and correct"), f"{w0:.1f}; {n_contra}")
rj = json.load(open(os.path.join(a.repo, "validation", "v0.1.7", "rederivation_v017.json")))["summary"]
av = rj["archived_v013"]
check("v0.1.7 archived: 23/23 contain; fault width/tau median 2.1; overall 0.53; range 0.05-10.7 (validation/v0.1.7)",
      av["formed_v017"] == 23 and av["contain_v017"] == 23 and f"{av['width_over_tau_v017']['fault_median']:.1f}" == "2.1" and intex("median 2.1 times $\\tau_w$")
      and f"{av['width_over_tau_v017']['median']:.2f}" == "0.53" and intex("from a median of 0.14 to 0.53", sup)
      and f"{av['width_over_tau_v017']['min']:.2f}--{av['width_over_tau_v017']['max']:.1f}" == "0.05--10.7" and intex("range 0.05--10.7", sup), json.dumps(av["width_over_tau_v017"]))
check("v0.1.7 archived: horizon claims unchanged (0.1 violated, 0.25 undetermined, 1 s satisfied)",
      [av["fault_horizon_claims"][k][f"v017_fault_horizon_{h}"] for k, h in (("L_horizon_fault_0100", "0.1"), ("L_horizon_fault_0250", "0.25"), ("L_horizon_fault_1000", "1.0"))] == ["violated", "undetermined", "satisfied"]
      and intex("violated, undetermined and satisfied under every rule, v0.1.7 included"))
ratios = sorted({r["ratio"] for r in mc if r["probe"] == "frame"})
check("MC accounting: 42 000 frame replicates = 7 ratios x 3 noise models x 2000; Table III shows five ratios",
      len(ratios) == 7 and sum(r["n_rep"] for r in mc if r["probe"] == "frame") == 42000 and intex("0 of 42\\,000 replicates over the supplement's full grid of seven ratios"), str(ratios))
check("text: RC12 wording (illustrative parameters; stationary arm; ROS 1 scope; SRC v1 contingency)",
      intex("illustrative engineering settings, not task-derived or clinical thresholds") and intex("The pairing assumes a stationary arm")
      and intex("the executable validation is ROS~1 only") and intex("counts the primary prediction as not matched") and not intex("reviewer-supplied"))

# ---------------------------------------------------------------- abstract length
ab = re.search(r"\\begin\{abstract\}(.*?)\\end\{abstract\}", tex, re.S).group(1)
nw = len(ab.replace("\\%", "%").split())
check("abstract <= 200 words", nw <= 200, f"{nw} words")

ok = all(c["ok"] for c in checks)
json.dump({"all_passed": ok, "checks": checks}, open(os.path.join(a.paper, "reporting_checks_v016.json"), "w"), indent=1)
print(f"{sum(c['ok'] for c in checks)}/{len(checks)} checks passed")
sys.exit(0 if ok else 1)
