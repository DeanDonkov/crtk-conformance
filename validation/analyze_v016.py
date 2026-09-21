#!/usr/bin/env python3
"""RC9 analysis of the v0.1.6 campaigns against PREREGISTRATION.md.  Reads only; writes tables to v0.1.6/analysis.

    python3 validation/analyze_v016.py dvrk      # dVRK-sim campaign (sections 1 of the pre-registration)
    python3 validation/analyze_v016.py src       # SRC v1/v2 with the geometry anchor (section 2)
    python3 validation/analyze_v016.py K|B|L     # reference-node campaigns (section 3)
    python3 validation/analyze_v016.py all
"""
import csv
import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

import numpy as np
from scipy.stats import beta

HERE = os.path.dirname(os.path.abspath(__file__))
V16 = os.path.join(HERE, "v0.1.6")
OUT = os.path.join(V16, "analysis")
os.makedirs(OUT, exist_ok=True)
AB = {"conformant": "C", "divergent": "D", "undetermined": "U", None: "-"}


def cp(k, n, conf=0.95):
    a = (1 - conf) / 2
    lo = 0.0 if k == 0 else float(beta.ppf(a, k, n - k + 1))
    hi = 1.0 if k == n else float(beta.ppf(1 - a, k + 1, n - k))
    return lo, hi


def write_csv(path, rows, keys=None):
    keys = keys or list({k: None for r in rows for k in r}.keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore", lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def probe_of(rep, cls, name=None):
    for p in rep["probes"]:
        if p["binding_class"] == cls and (name is None or p.get("probe") == name or p.get("name") == name):
            return p
    return None


# ------------------------------------------------------------------ dVRK-sim
PRED_DVRK = {  # (config, case): (spatial, dimensional, temporal, {sub-verdict predictions})
    ("jhu", "C_dvrk"): ("C", "U", "U", {"state_machine": "satisfied", "stop_behaviour": "satisfied", "rate": "undetermined"}),
    ("jhu", "C_src"): ("D", "U", "D", {"state_machine": "violated", "stop_behaviour": "satisfied", "rate": "undetermined"}),
    ("jhu", "C_disc"): ("U", "U", "U", {}),
    ("jhu", "F2_identity"): ("D", None, None, {}), ("jhu", "F3_inverse"): ("D", None, None, {}), ("jhu", "F4_0874"): ("C", None, None, {}),
    ("jhu", "F5a_095"): ("C", None, None, {}), ("jhu", "F5b_098"): ("C", None, None, {}), ("jhu", "F5c_102"): ("D", None, None, {}), ("jhu", "F5d_105"): ("D", None, None, {}),
    ("jhu", "U_si_5mm"): (None, "C", None, {}), ("jhu", "U_mm_1mm"): (None, "D", None, {}), ("jhu", "U_mm_5mm"): (None, "D", None, {}),
    ("jhu", "T_fault025"): (None, None, "D", {"stop_behaviour": "violated"}),
    ("nobase", "C_src"): ("C", "U", "D", {"state_machine": "violated", "stop_behaviour": "satisfied"}),
    ("nobase", "C_dvrk"): ("D", "U", "U", {"state_machine": "satisfied", "stop_behaviour": "satisfied"}),
    ("nobase", "F_exactJHU"): ("D", None, None, {}),
}


def dvrk():
    base = os.path.join(V16, "dvrk_sim")
    truths = json.load(open(os.path.join(base, "derived_truths.json")))
    rows = []
    for ldir in sorted(glob.glob(os.path.join(base, "runs", "*_launch*"))):
        cfg, launch = os.path.basename(ldir).split("_launch")
        for path in sorted(glob.glob(os.path.join(ldir, "*.json"))):
            name = os.path.basename(path)[:-5]
            if name.endswith(".bring_up") or (cfg, name) not in PRED_DVRK:
                continue
            rep = json.load(open(path))
            s = rep["summary"]
            pred = PRED_DVRK[(cfg, name)]
            row = {"config": cfg, "launch": int(launch), "case": name, "spatial": AB[s.get("spatial")], "dimensional": AB[s.get("dimensional")], "temporal": AB[s.get("temporal")],
                   "pred_spatial": pred[0] or "-", "pred_dimensional": pred[1] or "-", "pred_temporal": pred[2] or "-"}
            sp = probe_of(rep, "spatial")
            if sp:
                sd = sp["estimates"].get("spatial_decision") or {}
                row["E_max_hat_mm"] = sd.get("e_max_m", float("nan")) * 1e3 if sd else None
                row["E_ci_mm"] = f"{sd['ci_low_m']*1e3:.4f}..{sd['ci_high_m']*1e3:.4f}" if sd else ""
                tk = {"C_src": "F2_identity" if cfg == "jhu" else None, "C_dvrk": None if cfg == "jhu" else "F_exactJHU"}.get(name, name)
                row["E_true_mm"] = (truths["E_max_m"][tk][cfg] * 1e3) if tk in truths["E_max_m"] else (0.0 if name in ("C_dvrk", "C_src") else None)
                row["pairs"] = sp["observations"].get("paired_samples") or sp["estimates"].get("paired_samples")
                row["zero_stamps_skipped"] = sp["observations"].get("zero_stamp_samples_skipped")
            dm = probe_of(rep, "dimensional")
            if dm:
                ga = dm["estimates"].get("geometry_anchor")
                if ga:
                    row["d_int_mm"] = ga.get("d_int_mean_if", float("nan")) * 1e3
                    row["lambda_hat"] = ga.get("lambda_hat_m")
                    row["lambda_ci_widened"] = ga.get("lambda_ci_widened_m")
                    row["e_s_ci_mm"] = [v * 1e3 for v in (ga.get("predicted_error_ci_m") or [])] or None
                    row["gates_passed"] = ga.get("gates_passed")
                cc = dm["estimates"].get("command_feedback_consistency")
                if cc:
                    row["consistency_flag"] = cc.get("flag_non_shared_binding_or_tracking_deficit")
                ir = dm["estimates"].get("internal_ratio")
                if ir:
                    row["internal_ratio"] = ir.get("mean")
            te = probe_of(rep, "temporal")
            if te:
                sv = te["estimates"].get("sub_verdicts", {})
                row.update({f"sv_{k}": v for k, v in sv.items()})
                Lv = te["observations"].get("liveness", {})
                row["stop_class"] = Lv.get("stop_class")
                sp_ = te["observations"].get("state_precondition", {})
                row["executed_when_disabled"] = sp_.get("executed_when_disabled")
                row["executed_when_enabled"] = sp_.get("executed_when_enabled")
                R = te["observations"].get("resolution", {})
                row["calib_responded"] = f"{R.get('latency_probes_responded')}/{R.get('calibration_commands', R.get('latency_probes_sent'))}"
                for k, v in pred[3].items():
                    row[f"pred_sv_{k}"] = v
            ok = all(row[c] == row["pred_" + c] for c in ("spatial", "dimensional", "temporal") if row["pred_" + c] != "-")
            ok = ok and all(row.get(f"sv_{k}") == v for k, v in pred[3].items())
            row["matches_prediction"] = ok
            rows.append(row)
    write_csv(os.path.join(OUT, "dvrk_sim_cases.csv"), rows)
    agg = defaultdict(list)
    for r in rows:
        agg[(r["config"], r["case"])].append(r)
    summ = []
    for (cfg, case), rs in sorted(agg.items(), key=lambda kv: (kv[0][0], list(PRED_DVRK).index(kv[0]))):
        summ.append({"config": cfg, "case": case, "launches": len(rs),
                     "outcomes": ";".join(sorted({f"{r['spatial']}/{r['dimensional']}/{r['temporal']}" for r in rs})),
                     "predicted": f"{rs[0]['pred_spatial']}/{rs[0]['pred_dimensional']}/{rs[0]['pred_temporal']}",
                     "all_match": all(r["matches_prediction"] for r in rs),
                     "E_max_hat_mm": ";".join(sorted({f"{r['E_max_hat_mm']:.4f}" for r in rs if r.get('E_max_hat_mm') is not None})),
                     "E_true_mm": rs[0].get("E_true_mm"),
                     "lambda_hat": ";".join(sorted({f"{r['lambda_hat']:.6f}" for r in rs if r.get('lambda_hat') is not None})),
                     "sub_verdicts": ";".join(sorted({"/".join(str(r.get(f"sv_{k}")) for k in ("state_machine", "stop_behaviour", "rate")) for r in rs if "sv_rate" in r}))})
    write_csv(os.path.join(OUT, "dvrk_sim_summary.csv"), summ)
    for s in summ:
        print(s)
    return rows, summ


# ------------------------------------------------------------------ SRC with the geometry anchor
PRED_SRC = {  # (release, case): (spatial, dimensional, temporal)
    ("v1", "src_client_geometry_1mm"): ("U", "D", "U"), ("v1", "src_client_geometry_5mm"): (None, "D", None), ("v1", "dvrk_client_geometry_1mm"): ("U", "D", "D"),
    ("v2", "src_client_geometry_1mm"): ("U", "U", "U"), ("v2", "src_client_geometry_5mm"): (None, "C", None), ("v2", "dvrk_client_geometry_1mm"): ("U", "U", "D"),
}


def src():
    rows = []
    for v in ("v1", "v2"):
        d = os.path.join(V16, "src_live", f"live-src-{v}")
        for case in ("src_client_geometry_1mm", "src_client_geometry_5mm", "dvrk_client_geometry_1mm"):
            p = os.path.join(d, case + ".json")
            if not os.path.exists(p):
                continue
            rep = json.load(open(p)); s = rep["summary"]; pred = PRED_SRC[(v, case)]
            row = {"release": v, "case": case, "spatial": AB[s.get("spatial")], "dimensional": AB[s.get("dimensional")], "temporal": AB[s.get("temporal")],
                   "pred": "/".join(x or "-" for x in pred)}
            dm = probe_of(rep, "dimensional")
            ga = (dm or {}).get("estimates", {}).get("geometry_anchor") or {}
            row.update(d_int_if=ga.get("d_int_mean_if"), lambda_hat=ga.get("lambda_hat_m"), lambda_ci=ga.get("lambda_ci_m"), lambda_ci_widened=ga.get("lambda_ci_widened_m"),
                       e_s_ci_mm=[None if x is None else x * 1e3 for x in ga.get("predicted_error_ci_m") or []] or None, gates_passed=ga.get("gates_passed"), gate_failures=ga.get("gate_failures"),
                       axes_angle_deg=ga.get("axes_angle_deg_mean"), n_ok=ga.get("n_trials_used"))
            te = probe_of(rep, "temporal")
            if te:
                row["sub_verdicts"] = te["estimates"].get("sub_verdicts")
            row["matches_prediction"] = all(a == b for a, b in zip((row["spatial"], row["dimensional"], row["temporal"]), pred) if b is not None)
            rows.append(row)
    json.dump(rows, open(os.path.join(OUT, "src_geometry.json"), "w"), indent=1)
    for r in rows:
        print(r)
    return rows


# ------------------------------------------------------------------ reference node
def load_dir(d, prefix=""):
    out = []
    for p in sorted(glob.glob(os.path.join(d, prefix + "*.json"))):
        out.append((os.path.basename(p)[:-5], json.load(open(p))))
    return out


def K():
    rows = []
    for name, r in load_dir(os.path.join(V16, "mock", "K")):
        fr, sc = r["frame"]["result"], r["scale"]["result"]
        est = sc["estimates"]
        cc = est.get("command_feedback_consistency", {})
        rows.append({"case": name, "what": r["what"], "frame": AB[fr["outcome"]], "frame_E_ci_mm": [x * 1e3 for x in (fr["estimates"].get("spatial_decision", {}).get("ci_low_m"), fr["estimates"].get("spatial_decision", {}).get("ci_high_m"))],
                     "scale": AB[sc["outcome"]], "internal_ratio": est.get("internal_ratio", {}).get("mean"), "internal_ratio_range": [est.get("internal_ratio", {}).get("min"), est.get("internal_ratio", {}).get("max")],
                     "s_hat": est.get("scale_anchored", {}).get("mean"), "flag": cc.get("flag_non_shared_binding_or_tracking_deficit"),
                     "goal_residual_mm": (cc.get("goal_residual_if") or {}).get("mean", float("nan")) * 1e3 if cc.get("goal_residual_if") else None,
                     "E_true_command_residual_mm": r["truth"]["E_residual_max_m"] * 1e3})
    write_csv(os.path.join(OUT, "K1_cases.csv"), rows)
    for x in rows:
        print(x)
    return rows


def classify(outcome, truth_conformant):
    if outcome == "undetermined":
        return "U"
    if outcome == "conformant":
        return "correct_C" if truth_conformant else "false_C"
    return "false_D" if truth_conformant else "correct_D"


def B():
    cells = defaultdict(Counter)
    cover = defaultdict(int)
    for name, r in load_dir(os.path.join(V16, "mock", "B")):
        parts = name.split("_")
        probe = "frame" if parts[0] == "BF" else "scale"
        model, sig, ratio = parts[1], parts[2], parts[3]
        tr = r["truth"]
        o = r["result"]["outcome"]
        cells[(probe, model, sig, ratio)][classify(o, tr["truth_conformant"])] += 1
        if probe == "frame":
            sd = r["result"]["estimates"].get("spatial_decision") or {}
            cover[(probe, model, sig, ratio)] += int(sd.get("ci_low_m", 1) - 1e-9 <= tr["E_true_m"] <= sd.get("ci_high_m", -1) + 1e-9)
        else:
            e = r["result"]["estimates"].get("predicted_error_at_workspace_edge_m", {})
            cover[(probe, model, sig, ratio)] += int((e.get("ci_low") or 1) - 1e-9 <= tr["E_true_m"] <= (e.get("ci_high") or -1) + 1e-9)
    mc = {}
    mcp = os.path.join(V16, "boundary", "boundary_montecarlo.json")
    if os.path.exists(mcp):
        for row in json.load(open(mcp))["rows"]:
            sig = f"{row['sigma_mm']:g}mm" if row["noise_model"] == "gaussian" else "0.001mm"
            mc[(row["probe"], row["noise_model"], sig, row["ratio"])] = row
    rows = []
    for key, c in sorted(cells.items()):
        n = sum(c.values())
        row = {"probe": key[0], "noise_model": key[1], "sigma": key[2], "ratio": key[3], "n": n, **{k: c.get(k, 0) for k in ("correct_C", "correct_D", "false_C", "false_D", "U")},
               "coverage": cover[key] / n}
        for k in ("false_C", "false_D", "U"):
            row[f"{k}_ci95"] = cp(c.get(k, 0), n)
        m = mc.get(key)
        if m:
            row["mc_false_D_rate"], row["mc_false_C_rate"], row["mc_U_rate"] = m["false_D_rate"], m["false_C_rate"], m["U_rate"]
            row["mc_within_live_ci"] = all(row[f"{k}_ci95"][0] <= m[f"{k}_rate"] <= row[f"{k}_ci95"][1] for k in ("false_C", "false_D", "U"))
        rows.append(row)
    write_csv(os.path.join(OUT, "B_live_boundary.csv"), rows)
    for x in rows:
        print(x)
    return rows


def L():
    groups = defaultdict(list)
    for name, r in load_dir(os.path.join(V16, "mock", "L")):
        tr = r["truth"]
        Lv = r["result"]["observations"]["liveness"]
        R = r["result"]["observations"]["resolution"]
        tau = Lv.get("tau_w_estimate_s") or {}
        sv = r["result"]["estimates"].get("sub_verdicts", {})
        if tr["mode"] == "hold":
            if Lv.get("stop_class") == "held_through_range":
                cls = "correct_hold"
            elif Lv.get("stop_class") in ("rejected", "faulted", "drifted"):
                cls = "false_trip"
            else:
                cls = "not_observable_or_U"
        else:
            st = tau.get("status")
            lo, hi = tau.get("interval_low_s"), tau.get("interval_high_s")
            if st == "ok" and lo is not None:
                cls = "contains" if lo <= tr["tau_w_s"] <= hi else "excludes"
            elif st in ("inconsistent",):
                cls = "contradictory"
            elif st in ("upper_bound",) and tau.get("tau_upper_bound_s") is not None:
                cls = "upper_bound_only" if tau["tau_upper_bound_s"] >= tr["tau_w_s"] else "upper_bound_excludes"
            else:
                cls = f"U:{st or Lv.get('stop_class')}"
        trials = Lv.get("trials", [])
        groups[(tr["rule"], tr["mode"], tr["loss"])].append({"run": name, "class": cls, "stop_verdict": sv.get("stop_behaviour"),
                                                               "calib": f"{R.get('latency_probes_responded')}/{R.get('calibration_commands') or R.get('latency_probes_sent')}",
                                                               "rejected_trials": sum(1 for t in trials if t["class"] == "rejected"),
                                                               "confirmed": sum(1 for t in trials if (t.get("confirmation") or {}).get("confirmed")),
                                                               "trials": len(trials), "wall_s": r["wall_s"],
                                                               "interval_ms": [None if tau.get("interval_low_s") is None else tau["interval_low_s"] * 1e3, None if tau.get("interval_high_s") is None else tau["interval_high_s"] * 1e3]})
    rows = []
    for key, rs in sorted(groups.items()):
        c = Counter(x["class"] for x in rs)
        wrong = c.get("false_trip", 0) + c.get("excludes", 0) + c.get("upper_bound_excludes", 0)
        rows.append({"rule": key[0], "policy": key[1], "loss": key[2], "n": len(rs), "classes": dict(c), "false": wrong, "false_ci95": cp(wrong, len(rs)),
                     "stop_verdicts": dict(Counter(x["stop_verdict"] for x in rs)), "calibration_loss_seen": sum(1 for x in rs if x["calib"].split("/")[0] != x["calib"].split("/")[1]),
                     "mean_trials": float(np.mean([x["trials"] for x in rs])), "mean_wall_s": float(np.mean([x["wall_s"] for x in rs])), "runs": rs})
    json.dump(rows, open(os.path.join(OUT, "L_loss.json"), "w"), indent=1)
    for x in rows:
        print({k: v for k, v in x.items() if k != "runs"})
    return rows


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("dvrk", "all"):
        dvrk()
    if what in ("src", "all"):
        src()
    if what in ("K", "all"):
        K()
    if what in ("B", "all"):
        B()
    if what in ("L", "all"):
        L()
