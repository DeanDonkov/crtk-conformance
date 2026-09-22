#!/usr/bin/env python3
"""Resting pose traces of the SRC releases (campaign S, secondary outcome S-3): noise level and correlation time of
measured_cp at the geometry reference configuration, and the plan the 0.1.8 correlation guard would make from them.
Not pre-registered as an analysis (the traces were); descriptive only.  No ROS.

The frame probe itself cannot run on SRC (no local/measured_cp); the guard is applied here to the pose about its mean,
i.e. to what a paired residual would carry if the second channel were noise free.

Usage: python3 validation/src_traces_v018.py [--out validation/v0.1.8/analysis]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from crtk_conformance import geometry as G  # noqa: E402
from crtk_conformance.spatial import correlation_plan, correlation_guard, integrated_autocorr_time, residual_series  # noqa: E402

UNIT_MM = {"v1": 100.0, "v2": 1000.0}  # interface unit in mm (v1.0.0: 0.1 m, changelog; v2.0.0: SI)

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=os.path.join(HERE, "v0.1.8", "analysis"))
a = ap.parse_args()
rows = []
for p in sorted(glob.glob(os.path.join(HERE, "v0.1.8", "src_live", "live-src-*", "launch*", "resting_trace.json"))):
    d = json.load(open(p))
    v = d["version"]
    cp = np.array(d["measured_cp"], dtype=float)
    t = cp[:, 0]
    Ts = [G.make_pose(G.quat_to_rot(*r[5:9]), r[2:5]) for r in cp]
    ser = residual_series(Ts)
    ser_mm = ser.copy(); ser_mm[:, :3] *= UNIT_MM[v]
    rate = (len(t) - 1) / (t[-1] - t[0])
    taus = [integrated_autocorr_time(ser[:, j])[0] if np.std(ser[:, j]) > 1e-12 else None for j in range(6)]
    plan = correlation_plan(ser, 5, 10)
    ok, why = correlation_guard(plan, 10, rate)
    js = np.array(d["measured_js"], dtype=float)[:, 1:]
    yaw = js[:, -1] - js[:, -1].mean()
    avg_off = []
    for k in range(0, len(yaw) - 36, 36):  # 0.3-s windows at ~120 Hz, as the probe's settled averages
        dd = js[k:k + 36, -1] - js[k:k + 36, -1].mean()
        avg_off.append(10.6 * (1.0 - float(np.mean(np.cos(dd)))))
    rows.append({"trace": os.path.relpath(p, HERE), "version": v, "samples": len(cp), "rate_hz": rate,
                 "joint_sd": [float(x) for x in js.std(axis=0)], "yaw_sd_rad": float(yaw.std()),
                 "yaw_lag1": float(np.corrcoef(yaw[:-1], yaw[1:])[0, 1]) if yaw.std() > 0 else None,
                 "window_average_offset_mm_median": float(np.median(avg_off)) if avg_off else None,
                 "pos_sd_mm": [float(np.std(ser_mm[:, j])) for j in range(3)], "rot_sd_mrad": [float(np.std(ser[:, j])) * 1e3 for j in range(3, 6)],
                 "pos_range_mm": [float(np.ptp(ser_mm[:, j])) for j in range(3)],
                 "tau_int_samples": taus, "plan": {"deterministic": plan.deterministic, "resolved": plan.resolved, "tau_int": plan.tau_int_samples,
                                                   "spacing": plan.spacing_samples, "n_eff": plan.effective_trials, "guard_ok": ok, "reason": why or plan.reason}})
    r = rows[-1]
    print(f"{r['trace']}: {r['samples']} samples at {rate:.1f} Hz; pos sd (mm) {[round(x, 5) for x in r['pos_sd_mm']]}; rot sd (mrad) {[round(x, 4) for x in r['rot_sd_mrad']]}; "
          f"tau {[None if x is None else round(x, 1) for x in taus]}; plan {r['plan']}")
os.makedirs(a.out, exist_ok=True)
json.dump({"rows": rows}, open(os.path.join(a.out, "src_traces_v018.json"), "w"), indent=1)
