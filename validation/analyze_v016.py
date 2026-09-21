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


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("dvrk", "all"):
        dvrk()
