#!/usr/bin/env python3
"""Analysis of the 0.1.8 campaigns (validation/v0.1.8/PREREGISTRATION.md, section 4).  Written and committed with the
pre-registration, before any run.  Reads validation/v0.1.8/{mock/F, dvrk_sim/D, src_live}; prints every pre-registered
prediction with its outcome; writes validation/v0.1.8/analysis/campaigns_v018.json."""
import glob
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
V18 = os.path.join(HERE, "v0.1.8")
preds = []


def pred(pid, text, ok, detail=""):
    preds.append({"id": pid, "prediction": text, "matched": bool(ok), "observed": detail})
    print(("MATCH    " if ok else "DEVIATES ") + f"{pid}: {text} -- {detail}")


def cls(outcome, truth_c):
    if outcome == "undetermined":
        return "U"
    if outcome == "conformant":
        return "cC" if truth_c else "fC"
    return "fD" if truth_c else "cD"


# ---------------------------------------------------------------- F
F = {}
for p in sorted(glob.glob(os.path.join(V18, "mock", "F", "F18_*.json"))):
    d = json.load(open(p))
    tr = d["truth"]; r = d["result"]
    cp_ = r["observations"].get("correlation_plan") or {}
    key = (tr["procedure"], tr["phi"], tr["ratio"])
    F.setdefault(key, []).append({"run": os.path.basename(p)[:-5], "outcome": r["outcome"], "class": cls(r["outcome"], tr["truth_conformant"]),
                                  "spacing": cp_.get("spacing_samples"), "ok": cp_.get("ok"), "tau": cp_.get("tau_int_samples"), "reason": cp_.get("reason"),
                                  "ungated": r["estimates"].get("outcome_without_correlation_guard")})


def cnt(key, c):
    return sum(1 for x in F.get(key, []) if x["class"] == c)


def n(key):
    return len(F.get(key, []))


pred("F-1", "iid, both procedures: 0.95 conformant >= 19/20 each; 1.00 no false verdict 20/20 each",
     all(n((pr, 0.0, "0.95")) == 20 and cnt((pr, 0.0, "0.95"), "cC") >= 19 and n((pr, 0.0, "1.00")) == 20 and cnt((pr, 0.0, "1.00"), "fD") + cnt((pr, 0.0, "1.00"), "fC") == 0
         for pr in ("0.1.7", "0.1.8")),
     str({pr: (cnt((pr, 0.0, "0.95"), "cC"), cnt((pr, 0.0, "1.00"), "fD")) for pr in ("0.1.7", "0.1.8")}))
pred("F-2", "phi 0.99, 0.1.7, 1.00: at least one false divergence in 20 runs", n(("0.1.7", 0.99, "1.00")) == 20 and cnt(("0.1.7", 0.99, "1.00"), "fD") >= 1,
     f"{cnt(('0.1.7', 0.99, '1.00'), 'fD')} false D of {n(('0.1.7', 0.99, '1.00'))}")
f3 = [x for rs in ("0.95", "1.00") for x in F.get(("0.1.8", 0.99, rs), [])]
pred("F-3", "phi 0.99, 0.1.8: no false verdict in its 40 runs", len(f3) == 40 and not any(x["class"] in ("fC", "fD") for x in f3),
     f"{sum(x['class'] in ('fC', 'fD') for x in f3)} false of {len(f3)}")
pred("F-4", "phi 0.99, 0.1.8, 1.00: undetermined in >= 18/20", n(("0.1.8", 0.99, "1.00")) == 20 and cnt(("0.1.8", 0.99, "1.00"), "U") >= 18,
     f"{cnt(('0.1.8', 0.99, '1.00'), 'U')} U of {n(('0.1.8', 0.99, '1.00'))}")
f5 = [x for rs in ("0.95", "1.00") for x in F.get(("0.1.8", 0.0, rs), [])]
pred("F-5", "iid, 0.1.8: planned spacing 5 or 6 samples in all 40", len(f5) == 40 and all(x["spacing"] in (5, 6) for x in f5), str(sorted({x["spacing"] for x in f5})))
pred("F-6", "phi 0.99, 0.1.8: spacing >= 20 samples or verdict withheld, every run", len(f3) == 40 and all((x["ok"] and (x["spacing"] or 0) >= 20) or not x["ok"] for x in f3),
     f"spacings {sorted(x['spacing'] for x in f3 if x['ok'])}; withheld {sum(not x['ok'] for x in f3)}")

# ---------------------------------------------------------------- D
PRED_D = {"F2_identity": "divergent", "F4_0874": "conformant", "F5a_095": "conformant", "F5b_098": "conformant", "F5c_102": "divergent", "F5d_105": "divergent",
          "U_si_1mm": "undetermined", "U_si_5mm": "conformant", "U_mm_1mm": "divergent"}
D = {}
for p in sorted(glob.glob(os.path.join(V18, "dvrk_sim", "D", "runs", "*", "*.json"))):
    if p.endswith(".bring_up.json"):
        continue
    name = os.path.basename(p)[:-5]
    d = json.load(open(p))
    for r in d.get("probes") or []:
        if r.get("probe") in ("FrameSemanticsProbe", "GeometryAnchorProbe"):
            g = (r.get("estimates") or {}).get("geometry_anchor") or {}
            D.setdefault(name, []).append({"launch": os.path.basename(os.path.dirname(p)), "outcome": r["outcome"],
                                           "spacing": (r["observations"].get("correlation_plan") or {}).get("spacing_samples"),
                                           "deterministic": (r["observations"].get("correlation_plan") or {}).get("deterministic"),
                                           "gates": g.get("gates_passed"), "d_int_mm": (g.get("d_int_mean_if") or float("nan")) * 1e3, "lambda": g.get("lambda_hat_m")})
fr = [k for k in PRED_D if k.startswith("F")]
pred("D-1", "frame cases as v0.1.6 in 3/3 launches; deterministic residual (spacing 5) in all 18",
     all(len(D.get(k, [])) == 3 and all(x["outcome"] == PRED_D[k] for x in D[k]) for k in fr) and all(x["spacing"] == 5 and x["deterministic"] for k in fr for x in D.get(k, [])),
     str({k: [x["outcome"][0].upper() for x in D.get(k, [])] for k in fr}))
un = [k for k in PRED_D if k.startswith("U")]
pred("D-2", "anchor: gates pass, d_int 9.100 +- 0.001 mm, U_si 1 mm U, 5 mm C, U_mm 1 mm D, 3/3 launches",
     all(len(D.get(k, [])) == 3 and all(x["outcome"] == PRED_D[k] and x["gates"] and abs(x["d_int_mm"] - 9.1) <= 0.001 for x in D[k]) for k in un),
     str({k: [(x["outcome"][0].upper(), round(x["d_int_mm"], 6)) for x in D.get(k, [])] for k in un}))

# ---------------------------------------------------------------- S
S = {}
for v in ("v1", "v2"):
    for p in sorted(glob.glob(os.path.join(V18, "src_live", f"live-src-{v}", "launch*", "*_geometry_*.json"))):
        d = json.load(open(p))
        for r in d.get("probes") or []:
            if r.get("probe") == "GeometryAnchorProbe":
                g = r["estimates"].get("geometry_anchor") or {}
                S.setdefault(v, []).append({"run": os.path.relpath(p, V18), "case": os.path.basename(p)[:-5], "outcome": r["outcome"], "gates": g.get("gates_passed"),
                                            "lambda": g.get("lambda_hat_m"), "lambda_ci": g.get("lambda_ci_m"), "lambda_ci_widened": g.get("lambda_ci_widened_m"),
                                            "d_int_if": g.get("d_int_mean_if"), "failures": g.get("gate_failures"), "error_ci_mm": [None if x is None else x * 1e3 for x in (g.get("predicted_error_ci_m") or [None, None])]})
s1 = S.get("v1", [])
pred("S-1", "SRC v1.0.0: gates pass, lambda_hat in [0.0995, 0.1027], unit verdict divergent, 9/9",
     len(s1) == 9 and all(x["gates"] and x["lambda"] is not None and 0.0995 <= x["lambda"] <= 0.1027 and x["outcome"] == "divergent" for x in s1),
     str([(x["case"], x["outcome"][0].upper(), None if x["lambda"] is None else round(x["lambda"], 5), x["gates"]) for x in s1]))
PRED_S2 = {"src_client_geometry_1mm": "undetermined", "src_client_geometry_5mm": "conformant", "dvrk_client_geometry_1mm": "undetermined"}
s2 = S.get("v2", [])
pred("S-2", "SRC v2.0.0: gates pass, lambda_hat in [1.000, 1.022]; SRC 1 mm U, 5 mm C, dVRK 1 mm U, 9/9",
     len(s2) == 9 and all(x["gates"] and x["lambda"] is not None and 1.000 <= x["lambda"] <= 1.022 and x["outcome"] == PRED_S2[x["case"]] for x in s2),
     str([(x["case"], x["outcome"][0].upper(), None if x["lambda"] is None else round(x["lambda"], 5), x["gates"]) for x in s2]))
tr = {}
for p in sorted(glob.glob(os.path.join(V18, "src_live", "live-src-*", "launch*", "resting_trace.json"))):
    tr[os.path.relpath(p, V18)] = len(json.load(open(p))["measured_cp"])
pred("S-3", "secondary: each launch records >= 1000 measured_cp samples at rest", len(tr) == 6 and all(v >= 1000 for v in tr.values()), str(tr))

os.makedirs(os.path.join(V18, "analysis"), exist_ok=True)
summary = {"predictions": preds, "all_matched": all(p["matched"] for p in preds), "F": {f"{k[0]}|{k[1]}|{k[2]}": v for k, v in F.items()}, "D": D, "S": S, "traces": tr}
json.dump(summary, open(os.path.join(V18, "analysis", "campaigns_v018.json"), "w"), indent=1)
print(json.dumps({"all_matched": summary["all_matched"], "n_predictions": len(preds)}))
