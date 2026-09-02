#!/usr/bin/env python3
"""Validation of crtk-conformance against crtk-mock with injected ground truth.

Usage:  python validation/run_validation.py [--quick] [--out validation/results]

Experiments (paper Section 7):
  F  frame probe:   grid of injected binding transforms x measurement noise
  S  scale probe:   grid of injected unit scales x noise (internal ratio vs anchored estimate)
  T  temporal probe: liveness timeouts (fault mode), released-drift mode, effective-rate limits,
                     state precondition on emulation presets
  R  robustness:    dropped messages, delayed responses, missing topics, no local/ topic
  P  presets:       one full CLI-equivalent run per preset (report JSON archived)

Every run archives the full probe result (all per-trial observations) as JSON so that threshold
sweeps and figures are computed offline by analyze.py without re-running anything.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import ros_master, mock_node  # noqa: E402

from crtk_conformance import __version__  # noqa: E402
from crtk_conformance.adapter import PlatformAdapter  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.report import build_report, _clean  # noqa: E402

NS = "/PSM1"
ANCHOR = f"{NS}/mock/ground_truth_cp"
TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10, speed_m_s=0.05, client_rate_hz=100.0, jitter_max_s=0.005)


def git_rev(path):
    try:
        return subprocess.check_output(["git", "-C", path, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def save(outdir, name, payload):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, name + ".json"), "w") as f:
        json.dump(_clean(payload), f, indent=1)


def run_probe(make_probe, preset, overrides, anchor=True):
    with mock_node(preset, overrides):
        a = PlatformAdapter(NS, anchor_topic=ANCHOR if anchor else None)
        disc = a.discover()
        t0 = time.time()
        r = make_probe(a).run()
        dt = time.time() - t0
        a.close()
    return {"preset": preset, "overrides": overrides, "discovery": disc, "result": r.to_dict(), "wall_s": dt}


# ------------------------------------------------------------------------------------------ F
def exp_frame(out, quick):
    rng = np.random.default_rng(42)
    direction = rng.normal(size=3); direction /= np.linalg.norm(direction)
    axis = [0.0, 0.0, 1.0]
    tnorms_mm = [0, 0.1, 0.5, 1, 2, 5, 20, 200] if not quick else [0, 1, 200]
    angles = [0, 1, 5, 30, 150] if not quick else [0, 30]
    noises_mm = [0.0, 0.02, 0.1] if not quick else [0.02]
    trials = 10 if not quick else 4
    k = 0
    for noise in noises_mm:
        for tn in tnorms_mm:
            for ang in angles:
                t = (direction * tn / 1000.0).tolist()
                over = {"bind_translation_m": t, "bind_axis": axis, "bind_angle_deg": ang, "noise_m": noise / 1000.0, "seed": k}
                rec = run_probe(lambda a: FrameSemanticsProbe(a, TOL, trials=trials, samples_per_trial=5), "reference", over)
                rec["truth"] = {"t_m": t, "t_norm_m": tn / 1000.0, "theta_deg": ang, "noise_m": noise / 1000.0,
                                "model_error_m": TOL.spatial_error(tn / 1000.0, math.radians(ang))}
                save(out, f"F_{k:03d}", rec)
                est = rec["result"]["estimates"]
                print(f"F{k:03d} t={tn}mm th={ang} noise={noise}mm -> {rec['result']['outcome']} |t_hat|={est.get('binding_translation_norm_m',{}).get('mean',float('nan'))*1e3:.3f}mm th_hat={est.get('binding_rotation_deg',{}).get('mean',float('nan')):.3f} ({rec['wall_s']:.1f}s)", flush=True)
                k += 1


# ------------------------------------------------------------------------------------------ S
def exp_scale(out, quick):
    scales = [1.0, 1.001, 1.01, 1.05, 1.1, 0.9, 0.5, 0.1, 10.0] if not quick else [1.0, 1.01, 0.1]
    noises_mm = [0.0, 0.02, 0.1] if not quick else [0.02]
    trials = 9 if not quick else 3
    k = 0
    for noise in noises_mm:
        for s in scales:
            over = {"unit_m": s, "noise_m": noise / 1000.0, "seed": 100 + k, "publish_local": False, "publish_T_b_w": True}
            rec = run_probe(lambda a: ScalingUnitsProbe(a, TOL, trials=trials, step_if=0.005, settle_s=0.6), "reference", over)
            rec["truth"] = {"s": s, "noise_m": noise / 1000.0, "model_error_m": TOL.dimensional_error(s)}
            save(out, f"S_{k:03d}", rec)
            est = rec["result"]["estimates"]
            print(f"S{k:03d} s={s} noise={noise}mm -> {rec['result']['outcome']} r_int={est['internal_ratio']['mean']:.4f} s_hat={est.get('scale_anchored',{}).get('mean',float('nan')):.4f} ({rec['wall_s']:.1f}s)", flush=True)
            k += 1
    # no-anchor control: the same divergent scale without an anchor must be undetermined
    over = {"unit_m": 0.1, "noise_m": 0.0, "seed": 999, "publish_local": False, "publish_T_b_w": True}
    rec = run_probe(lambda a: ScalingUnitsProbe(a, TOL, trials=trials, step_if=0.005, settle_s=0.6), "reference", over, anchor=False)
    rec["truth"] = {"s": 0.1, "noise_m": 0.0, "model_error_m": TOL.dimensional_error(0.1), "anchor": False}
    save(out, "S_noanchor", rec)
    print(f"S no-anchor s=0.1 -> {rec['result']['outcome']} r_int={rec['result']['estimates']['internal_ratio']['mean']:.4f}", flush=True)


# ------------------------------------------------------------------------------------------ T
def exp_temporal(out, quick):
    taus = [0.0, 0.05, 0.1, 0.25, 0.5, 1.0] if not quick else [0.0, 0.25]
    trials = 5 if not quick else 2
    k = 0
    for tau in taus:
        over = {"watchdog_s": tau, "watchdog_mode": "fault", "seed": 200 + k}
        rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=trials, gap_max_s=1.5, bisection_steps=6, rates_hz=(50, 100, 200, 500, 1000)), "reference", over)
        rec["truth"] = {"tau_w_s": tau, "mode": "fault"}
        save(out, f"T_live_{k:03d}", rec)
        print(f"T{k:03d} tau={tau} -> {rec['result']['observations']['liveness']['finding']} ({rec['wall_s']:.1f}s)", flush=True)
        k += 1
    # released-drift semantics (AMBF object-layer emulation)
    rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=trials, gap_max_s=1.5, rates_hz=(100,)), "emul-ambf-object-watchdog", {})
    rec["truth"] = {"tau_w_s": 0.5, "mode": "release", "drift_m_s": 0.02}
    save(out, "T_release", rec)
    print(f"T release -> {rec['result']['observations']['liveness']['finding']}", flush=True)
    # effective rate: loop x publish
    for j, (loop, pub) in enumerate([(120, 120), (120, 1000), (1500, 100), (1500, 1000)] if not quick else [(120, 1000)]):
        over = {"loop_rate_hz": loop, "publish_rate_hz": pub, "seed": 300 + j}
        rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, rates_hz=(50, 100, 200, 500, 1000)), "reference", over)
        rec["truth"] = {"loop_rate_hz": loop, "publish_rate_hz": pub}
        save(out, f"T_rate_{j:03d}", rec)
        eff = [(r["command_rate_hz"], round(r["effective_rate_hz"], 1)) for r in rec["result"]["observations"]["effective_rate"]["per_rate"]]
        print(f"T rate loop={loop} pub={pub} -> publish {rec['result']['observations']['effective_rate']['publish_rate_hz']:.1f} Hz eff {eff}", flush=True)
    # state precondition on presets
    for j, preset in enumerate(["reference", "emul-dvrk-jhu-psm2", "emul-src-v1"]):
        rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, rates_hz=(100,)), preset, {})
        rec["truth"] = {"preset": preset}
        save(out, f"T_state_{j:03d}", rec)
        print(f"T state {preset} -> {rec['result']['observations']['state_precondition']}", flush=True)


# ------------------------------------------------------------------------------------------ R
def exp_robustness(out, quick):
    cases = {
        "drop_10": {"drop_prob": 0.1},
        "drop_50": {"drop_prob": 0.5},
        "delay_50ms_jitter_20ms": {"response_delay_s": 0.05, "response_jitter_s": 0.02},
        "missing_measured_cp": {"publish_measured_cp": False},
        "no_local_topic": {"publish_local": False},
        "divergent_but_noisy": {"bind_translation_m": [0.002, 0, 0], "noise_m": 0.0005},
    }
    if quick:
        cases = {k: cases[k] for k in ("drop_50", "missing_measured_cp")}
    for name, over in cases.items():
        over = dict(over, seed=400)
        with mock_node("reference", over):
            a = PlatformAdapter(NS, anchor_topic=ANCHOR)
            disc = a.discover()
            rs = [FrameSemanticsProbe(a, TOL, trials=5).run(), ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.6).run(),
                  RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.5, rates_hz=(100, 500)).run()]
            a.close()
        rec = {"case": name, "overrides": over, "discovery": disc, "results": [r.to_dict() for r in rs]}
        save(out, f"R_{name}", rec)
        print(f"R {name} -> " + ", ".join(f"{r.binding_class}={r.outcome.value}" for r in rs), flush=True)


# ------------------------------------------------------------------------------------------ P
def exp_presets(out, quick):
    from crtk_mock.presets import PRESETS

    for name in PRESETS:
        with mock_node(name, {}):
            a = PlatformAdapter(NS, anchor_topic=ANCHOR)
            disc = a.discover()
            rs = [FrameSemanticsProbe(a, TOL, trials=5).run(), ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.6).run(),
                  RateSensitivityProbe(a, TOL, trials=2 if not quick else 1, gap_max_s=1.0, rates_hz=(50, 100, 200, 500, 1000)).run()]
            rep = build_report(NS, TOL, disc, rs, os.environ.get("ROS_MASTER_URI", ""))
            a.close()
        save(out, f"P_{name}", rep)
        print(f"P {name} -> {rep['summary']}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default="F,S,T,R,P")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    meta = {
        "crtk_conformance_version": __version__,
        "repo_commit": git_rev(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")),
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
        "tolerance": TOL.to_dict(),
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "quick": args.quick,
        "command": " ".join(sys.argv),
    }
    t0 = time.time()
    with ros_master(port=11611):
        if "F" in args.only:
            exp_frame(args.out, args.quick)
        if "S" in args.only:
            exp_scale(args.out, args.quick)
        if "T" in args.only:
            exp_temporal(args.out, args.quick)
        if "R" in args.only:
            exp_robustness(args.out, args.quick)
        if "P" in args.only:
            exp_presets(args.out, args.quick)
    meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    meta["wall_s"] = time.time() - t0
    save(args.out, "meta", meta)
    print("done in %.0f s" % meta["wall_s"])


if __name__ == "__main__":
    main()
