#!/usr/bin/env python3
"""EXPLORATORY (after campaign S; not pre-registered): the single-joint estimator of 0.1.6/0.1.7 applied offline to the
poses recorded by the joint-space anchor in campaign S.  No ROS, no new run.

Each joint-space trial records the settled reference pose and the settled pose after every joint step
(observations.trials[].T_ref_measured, T_steps_measured; steps ordered joint-major, +delta then -delta).  The
single-joint estimator (dimensional.screw_axis, dimensional.geometry_anchor_decision, unchanged) is applied to the
+delta wrist-pitch and +delta wrist-yaw steps with the commanded step (0.25 rad) and its own gates, so that both
estimators are compared on the same data.  The step size differs from the 0.5 rad of the v0.1.6 campaign.

Usage: python3 validation/rederive_single_joint_S_v018.py [--out validation/v0.1.8/analysis]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance.dimensional import geometry_anchor_decision, screw_axis  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "analysis"))
a = ap.parse_args()
rows = []
for v in ("v1", "v2"):
    for p in sorted(glob.glob(os.path.join(HERE, "v0.1.8", "src_live", f"live-src-{v}", "launch*", "*_geometry_*.json"))):
        d = json.load(open(p))
        exp = d.get("expectations") or {}
        for r in d.get("probes") or []:
            if r.get("probe") != "GeometryAnchorProbe":
                continue
            ob = r["observations"]
            dim = ob["expectation"] if isinstance(ob.get("expectation"), dict) else {}
            deltas = ob["poe_steps"]["deltas"]
            ip, iy = ob["pitch_joint_index"], ob["yaw_joint_index"]
            P, Y = [], []
            for t in ob["trials"]:
                T0 = np.array(t["T_ref_measured"]); Ts = [np.array(x) for x in t["T_steps_measured"]]
                P.append(screw_axis(T0, Ts[2 * ip])); Y.append(screw_axis(T0, Ts[2 * iy]))
            case = os.path.basename(p)[:-5]
            tol_mm = 5.0 if case.endswith("5mm") else 1.0
            unit = dim.get("expected_unit_m", 1.0)
            tol = Tolerance(epsilon_m=tol_mm / 1000.0, workspace_radius_m=0.10, speed_m_s=0.05, client_rate_hz=100, jitter_max_s=0.005)
            dec = geometry_anchor_decision(P, Y, deltas[ip], 0.0091, 0.015, unit, tol)
            joint_space = r["outcome"]
            rows.append({"release": v, "run": os.path.relpath(p, os.path.join(HERE, "v0.1.8")), "joint_space_outcome": joint_space,
                         "single_joint_outcome": dec.outcome.value if hasattr(dec.outcome, "value") else str(dec.outcome),
                         "single_joint_lambda_hat_m": dec.lambda_hat_m, "single_joint_gate_failures": list(dec.gate_failures or [])})
            print(f"{v} {rows[-1]['run']}: joint-space {joint_space}; single-joint {rows[-1]['single_joint_outcome']} "
                  f"(lambda {dec.lambda_hat_m}); {len(rows[-1]['single_joint_gate_failures'])} gate failures "
                  f"{rows[-1]['single_joint_gate_failures'][:1]}")
os.makedirs(a.out, exist_ok=True)
json.dump({"exploratory": True, "rows": rows}, open(os.path.join(a.out, "single_joint_on_S_v018.json"), "w"), indent=1)
