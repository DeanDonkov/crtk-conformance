#!/usr/bin/env python3
"""Validation campaign of crtk-conformance 0.1.3 against crtk-mock (plan: VALIDATION_PLAN_V013.md).

Usage:  python validation/run_validation_v013.py [--quick] [--out validation/v0.1.3/mock] [--only F,C,S,T,L,P,R,X,M]

Every run archives the full probe result (all per-trial observations, and for rate windows the send stamps and
the channel trace) as JSON, together with the injected truth and the declared expectation.  Every mock instance
of a temporal run writes its EVENT LOG (receive / drop / reject / ignore / apply / supersede / discard, with the
client's header stamp) to <run>.events.jsonl next to the result, so that analyze_v013.py derives the rate truth
from the implementation's own record of what it applied and when -- independently of the estimator (RC4
adversarial review, finding 5).  Decisions, sweeps and figures are computed offline by analyze_v013.py without
re-running anything.  Nothing here is a measurement of any platform.

Changes from the v0.1.2 campaign (RC4 adversarial review): rate cases with an ordered FIFO buffer slower than
the client (finding 1), a channel that reports the previous command (finding 3: the pre-existing setpoint), the
delayed-application cases scored on the applied age; liveness cases with the declared horizon below, across and
above the timeout interval for a fault and for a drift policy (finding 4); the latency allowance and the spatial
assumptions recorded in every report (findings 6, 7); all seeds independent of the earlier campaigns (offset
20000).
"""
from __future__ import annotations

import argparse
import hashlib
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

from crtk_conformance import __version__, geometry as G  # noqa: E402
from crtk_conformance.adapter import PlatformAdapter  # noqa: E402
from crtk_conformance.expectations import Expectations  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.report import build_report, _clean  # noqa: E402
from crtk_conformance.spatial import exact_max_error  # noqa: E402

NS = "/PSM1"
ANCHOR = f"{NS}/mock/ground_truth_cp"
TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10, speed_m_s=0.05, client_rate_hz=100.0, jitter_max_s=0.005)
JHU = {"bind_translation_m": [0.20, 0.0, 0.0], "bind_axis": [1.0, 0.0, 0.0], "bind_angle_deg": -150.0}


def jhu_expectation() -> Expectations:
    R = G.axis_angle([1, 0, 0], math.radians(-150.0))
    x, y, z, w = G.rot_to_quat(R)
    return Expectations.from_dict({"spatial": {"mode": "expected_transform", "translation_m": [0.20, 0, 0], "quaternion_xyzw": [x, y, z, w]}})


E_ID = Expectations.from_dict({"spatial": {"mode": "identity"}})
E_DISC = Expectations()
E_SI = Expectations.from_dict({"dimensional": {"mode": "si"}})
E_RATE = Expectations.from_dict({"temporal": {"rate": "required"}})
E_FAULT = Expectations.from_dict({"temporal": {"stop_behaviour": "fault"}})
E_HOLD = Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})
E_DRIFT = Expectations.from_dict({"temporal": {"stop_behaviour": "drift"}})
E_RELEASE = Expectations.from_dict({"temporal": {"stop_behaviour": "release"}})
SEED0 = 20000  # every seed of this campaign is offset from the v0.1.1 (0) and v0.1.2 (10000) campaigns'


def frame_truth(t_m, axis, angle_deg, extra=None):
    """Truth record of a frame case: exact maximum error eq. (3') and the eq. (3) bound, both at r_ws."""
    R = G.axis_angle(axis, math.radians(angle_deg))
    d = {"t_m": list(t_m), "t_norm_m": float(np.linalg.norm(t_m)), "theta_deg": angle_deg, "axis": list(axis),
         "exact_max_error_m": exact_max_error(R, np.asarray(t_m), TOL.workspace_radius_m),
         "bound_eq3_m": TOL.spatial_error(float(np.linalg.norm(t_m)), math.radians(angle_deg))}
    d.update(extra or {})
    return d


def git_rev(path):
    try:
        return subprocess.check_output(["git", "-C", path, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def save(outdir, name, payload):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, name + ".json"), "w") as f:
        json.dump(_clean(payload), f, indent=1)


def run_probe(make_probe, preset, overrides, anchor=True, expectation=None, event_log=None):
    overrides = dict(overrides or {})
    if event_log:
        overrides["event_log_path"] = event_log
        if os.path.exists(event_log):
            os.remove(event_log)
    with mock_node(preset, overrides):
        a = PlatformAdapter(NS, anchor_topic=ANCHOR if anchor else None)
        disc = a.discover()
        t0 = time.time()
        r = make_probe(a).run()
        dt = time.time() - t0
        a.close()
    return {"preset": preset, "overrides": overrides, "expectation": (expectation or E_DISC).to_dict(), "discovery": disc,
            "result": r.to_dict(), "wall_s": dt, "event_log": (os.path.basename(event_log) if event_log else None)}


def evlog(out, name):
    os.makedirs(out, exist_ok=True)
    return os.path.join(out, name + ".events.jsonl")


def log(msg):
    print(msg, flush=True)


# ------------------------------------------------------------------------------------------ F
def exp_frame(out, quick):
    rng = np.random.default_rng(SEED0 + 42)
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
                over = {"bind_translation_m": t, "bind_axis": axis, "bind_angle_deg": ang, "noise_m": noise / 1000.0, "seed": SEED0 + k}
                rec = run_probe(lambda a: FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=trials, samples_per_trial=5, expectations=E_ID), "reference", over, expectation=E_ID)
                rec["truth"] = frame_truth(t, axis, ang, {"noise_m": noise / 1000.0})
                save(out, f"F_id_{k:03d}", rec)
                est = rec["result"]["estimates"]
                sd = est.get("spatial_decision") or {}
                log(f"F_id_{k:03d} t={tn}mm th={ang} noise={noise}mm -> {rec['result']['outcome']} e_max={sd.get('e_max_m', float('nan'))*1e3:.3f}mm [{sd.get('ci_low_m', float('nan'))*1e3:.3f}..{sd.get('ci_high_m', float('nan'))*1e3:.3f}] truth {rec['truth']['exact_max_error_m']*1e3:.3f} ({rec['wall_s']:.1f}s)")
                k += 1
    # expected-transform runs (JHU-like binding), wrong expectation, discover-only, orientation noise, stamp skew
    cases = []
    for j, noise in enumerate(noises_mm):
        cases.append((f"F_exp_{j}", dict(JHU, noise_m=noise / 1000.0, seed=SEED0 + 500 + j), jhu_expectation(), {"binding": "JHU", "noise_m": noise / 1000.0, "expect": "JHU", "exact_max_error_m": 0.0}))
        cases.append((f"F_expwrong_{j}", dict(JHU, noise_m=noise / 1000.0, seed=SEED0 + 520 + j), E_ID, dict(frame_truth([0.20, 0, 0], [1, 0, 0], -150.0), binding="JHU", noise_m=noise / 1000.0, expect="identity")))
        cases.append((f"F_disc_{j}", {"bind_translation_m": [0.002, 0, 0], "bind_angle_deg": 5.0, "bind_axis": [0, 0, 1], "noise_m": noise / 1000.0, "seed": SEED0 + 540 + j}, E_DISC, {"binding": "2mm/5deg", "noise_m": noise / 1000.0, "expect": "discover_only"}))
    cases.append(("F_nolocal", {"publish_local": False, "publish_T_b_w": True, "seed": SEED0 + 560}, E_ID, {"binding": "identity", "local": False, "expect": "identity"}))
    for j, od in enumerate([0.1, 0.5, 2.0] if not quick else [0.5]):
        cases.append((f"F_orient_{j}", dict(JHU, orientation_noise_deg=od, noise_m=0.00002, seed=SEED0 + 570 + j), jhu_expectation(), {"binding": "JHU", "orientation_noise_deg": od, "noise_m": 0.00002, "expect": "JHU", "exact_max_error_m": 0.0}))
    for j, sk in enumerate([0.002, 0.030, 0.080] if not quick else [0.080]):
        cases.append((f"F_skew_{j}", dict(JHU, stamp_skew_s=sk, seed=SEED0 + 580 + j), jhu_expectation(), {"binding": "JHU", "stamp_skew_s": sk, "expect": "JHU", "pairing_window_s": 0.05}))
    # RC3 review, finding 1: translation along the rotation axis (t_par = 0.7 mm, theta = 0.3 deg): exact 0.874 mm < 1 mm,
    # eq. (3) bound 1.224 mm; the same magnitude perpendicular to the axis attains the bound (divergent at 1 mm)
    for j, noise in enumerate(noises_mm):
        cases.append((f"F_rev_par_{j}", {"bind_translation_m": [0, 0, 0.0007], "bind_axis": [0, 0, 1], "bind_angle_deg": 0.3, "noise_m": noise / 1000.0, "seed": SEED0 + 600 + j}, E_ID,
                      frame_truth([0, 0, 0.0007], [0, 0, 1], 0.3, {"noise_m": noise / 1000.0, "expect": "identity", "case": "reviewer counterexample: t parallel to the axis"})))
        cases.append((f"F_rev_perp_{j}", {"bind_translation_m": [0.0007, 0, 0], "bind_axis": [0, 0, 1], "bind_angle_deg": 0.3, "noise_m": noise / 1000.0, "seed": SEED0 + 610 + j}, E_ID,
                      frame_truth([0.0007, 0, 0], [0, 0, 1], 0.3, {"noise_m": noise / 1000.0, "expect": "identity", "case": "same magnitude perpendicular to the axis"})))
    # non-SI interface with an expected transform: the client declares the unit its expectation is expressed in
    e_jhu_01 = Expectations.from_dict(dict(jhu_expectation().to_dict(), dimensional={"mode": "si", "expected_unit_m": 0.1}))
    cases.append(("F_unit01_declared", dict(JHU, unit_m=0.1, seed=SEED0 + 620), e_jhu_01, {"binding": "JHU", "unit_m": 0.1, "expect": "JHU with expected_unit_m 0.1", "exact_max_error_m": 0.0}))
    cases.append(("F_unit01_assumed_si", dict(JHU, unit_m=0.1, seed=SEED0 + 621), jhu_expectation(), {"binding": "JHU", "unit_m": 0.1, "expect": "JHU assumed in metres (client wrong about the unit)",
                  "exact_max_error_m": exact_max_error(np.eye(3), np.array([2.0 - 0.20, 0, 0]), TOL.workspace_radius_m), "note": "residual translation (2.0 - 0.2) m along x, rotation as expected"}))
    # orientation tolerance declared: within (noise 0.1 deg, tolerance 1 deg) and violated (binding rotated 3 deg more than expected)
    e_jhu_ori = Expectations.from_dict(dict(jhu_expectation().to_dict()))
    e_jhu_ori.spatial.orientation_tolerance_deg = 1.0
    cases.append(("F_oritol_within", dict(JHU, orientation_noise_deg=0.1, noise_m=0.00002, seed=SEED0 + 630), e_jhu_ori, {"binding": "JHU", "orientation_noise_deg": 0.1, "orientation_tolerance_deg": 1.0, "residual_rotation_deg": 0.0, "exact_max_error_m": 0.0}))
    Rres = G.axis_angle([1, 0, 0], math.radians(-153.0)) @ G.invert(G.make_pose(G.axis_angle([1, 0, 0], math.radians(-150.0)), [0.2, 0, 0]))[:3, :3]
    cases.append(("F_oritol_violated", dict(JHU, bind_angle_deg=-153.0, noise_m=0.00002, seed=SEED0 + 631), e_jhu_ori, {"binding": "JHU rotated -153 deg", "orientation_tolerance_deg": 1.0, "residual_rotation_deg": 3.0,
                  "exact_max_error_m": exact_max_error(Rres, np.zeros(3), TOL.workspace_radius_m), "note": "positional effect of the 3 deg residual over r_ws: 2 sin(1.5 deg) x 0.1 m = 5.2 mm, so the positional verdict is divergent too"}))
    for name, over, exp, truth in cases:
        rec = run_probe(lambda a, e=exp: FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=trials, samples_per_trial=5, expectations=e), "reference", over, expectation=exp)
        rec["truth"] = truth
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} | {rec['result']['decision_basis'][:110]}")


# ------------------------------------------------------------------------------------------ C (spatial interval coverage)
def exp_coverage(out, quick):
    """Replicated frame-probe runs at fixed injected residuals: does the reported interval contain the true exact maximum
    error?  RC3 review finding 4 (the 0.1.1 interval had ~45 % coverage at zero residual).  Replicates share one mock
    instance per configuration (independent noise draws from one seeded stream); the probe's n = 10 trials each."""
    reps = 60 if not quick else 6
    trials = 10 if not quick else 4
    configs = [
        ("zero", {"bind_translation_m": [0, 0, 0], "bind_angle_deg": 0.0, "bind_axis": [0, 0, 1], "noise_m": 0.0001, "seed": SEED0 + 700}, [0, 0, 0], [0, 0, 1], 0.0),
        ("boundary", {"bind_translation_m": [0, 0, 0.001], "bind_angle_deg": 0.0, "bind_axis": [0, 0, 1], "noise_m": 0.0001, "seed": SEED0 + 701}, [0, 0, 0.001], [0, 0, 1], 0.0),
        ("rotation", {"bind_translation_m": [0.0007, 0, 0], "bind_angle_deg": 0.3, "bind_axis": [1, 0, 0], "noise_m": 0.00005, "orientation_noise_deg": 0.02, "seed": SEED0 + 702}, [0.0007, 0, 0], [1, 0, 0], 0.3),
        ("large", {"bind_translation_m": [0.002, 0, 0], "bind_angle_deg": 5.0, "bind_axis": [0, 0, 1], "noise_m": 0.0001, "seed": SEED0 + 703}, [0.002, 0, 0], [0, 0, 1], 5.0),
    ]
    for cname, over, t, axis, ang in configs:
        truth = frame_truth(t, axis, ang, {"noise_m": over["noise_m"], "orientation_noise_deg": over.get("orientation_noise_deg", 0.0), "replicates": reps, "trials": trials})
        results = []
        with mock_node("reference", over):
            a = PlatformAdapter(NS, anchor_topic=ANCHOR)
            disc = a.discover()
            for i in range(reps):
                r = FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=trials, samples_per_trial=5, expectations=E_ID).run().to_dict()
                sd = r["estimates"].get("spatial_decision") or {}
                results.append({"replicate": i, "outcome": r["outcome"], "spatial_decision": sd, "predicted_abs_error_at_workspace_edge_m": r["estimates"].get("predicted_abs_error_at_workspace_edge_m"),
                                "binding_translation_m": r["estimates"].get("binding_translation_m"), "binding_rotation_deg": r["estimates"].get("binding_rotation_deg")})
            a.close()
        hits = sum(1 for x in results if x["spatial_decision"] and x["spatial_decision"]["ci_low_m"] <= truth["exact_max_error_m"] <= x["spatial_decision"]["ci_high_m"])
        save(out, f"C_cov_{cname}", {"preset": "reference", "overrides": over, "expectation": E_ID.to_dict(), "discovery": disc, "truth": truth, "replicates": results, "coverage": hits / len(results)})
        log(f"C_cov_{cname}: {hits}/{len(results)} intervals contain the true e_max = {truth['exact_max_error_m']*1e3:.3f} mm; outcomes " +
            ", ".join(f"{o}:{sum(1 for x in results if x['outcome']==o)}" for o in ("conformant", "divergent", "undetermined")))


# ------------------------------------------------------------------------------------------ S
def exp_scale(out, quick):
    scales = [1.0, 1.001, 1.01, 1.05, 1.1, 0.9, 0.5, 0.1, 10.0] if not quick else [1.0, 1.01, 0.1]
    noises_mm = [0.0, 0.02, 0.1] if not quick else [0.02]
    trials = 9 if not quick else 3
    k = 0
    for noise in noises_mm:
        for s in scales:
            over = {"unit_m": s, "noise_m": noise / 1000.0, "seed": SEED0 + 100 + k, "publish_local": False, "publish_T_b_w": True}
            rec = run_probe(lambda a: ScalingUnitsProbe(a, TOL, trials=trials, step_if=0.005, settle_s=0.6, expectations=E_SI), "reference", over, expectation=E_SI)
            rec["truth"] = {"s": s, "noise_m": noise / 1000.0, "model_error_m": TOL.dimensional_error(s)}
            save(out, f"S_si_{k:03d}", rec)
            est = rec["result"]["estimates"]
            log(f"S_si_{k:03d} s={s} noise={noise}mm -> {rec['result']['outcome']} r_int={est['internal_ratio']['mean']:.4f} s_hat={est.get('scale_anchored',{}).get('mean',float('nan')):.4f}")
            k += 1
    extra = [
        ("S_noanchor", {"unit_m": 0.1, "seed": SEED0 + 999, "publish_local": False, "publish_T_b_w": True}, E_SI, False, {"s": 0.1, "anchor": False}),
        ("S_disc", {"unit_m": 0.1, "seed": SEED0 + 998, "publish_local": False, "publish_T_b_w": True}, E_DISC, True, {"s": 0.1, "expect": "discover_only"}),
        ("S_unit01", {"unit_m": 0.1, "seed": SEED0 + 997, "publish_local": False, "publish_T_b_w": True}, Expectations.from_dict({"dimensional": {"mode": "si", "expected_unit_m": 0.1}}), True, {"s": 0.1, "expect_unit_m": 0.1, "model_error_m": 0.0}),
    ]
    for j, an in enumerate([0.02, 0.1, 0.5] if not quick else [0.1]):
        extra.append((f"S_anchornoise_{j}", {"unit_m": 0.5, "anchor_noise_m": an / 1000.0, "seed": SEED0 + 980 + j, "publish_local": False, "publish_T_b_w": True}, E_SI, True, {"s": 0.5, "anchor_noise_m": an / 1000.0, "model_error_m": TOL.dimensional_error(0.5)}))
    for j, dp in enumerate([0.1, 0.5]):
        extra.append((f"S_drop_{j}", {"unit_m": 1.0, "drop_prob": dp, "seed": SEED0 + 990 + j, "publish_local": False, "publish_T_b_w": True}, E_SI, True, {"s": 1.0, "drop_prob": dp, "model_error_m": 0.0}))
    for name, over, exp, anchor, truth in extra:
        rec = run_probe(lambda a, e=exp: ScalingUnitsProbe(a, TOL, trials=trials, step_if=0.005, settle_s=0.6, expectations=e), "reference", over, anchor=anchor, expectation=exp)
        rec["truth"] = truth
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} | {rec['result']['decision_basis'][:100]}")


# ------------------------------------------------------------------------------------------ T (rate)
def _rate_log(name, rec):
    rows = rec["result"]["observations"]["effective_rate"]["per_rate"]

    def one(r):
        acc = r.get("acceptance") or {}
        lo, hi = acc.get("age_lower_s"), acc.get("age_upper_s")
        f = lambda v: f"{v*1e3:.0f}" if isinstance(v, (int, float)) and v == v else "nan"
        return f"{r['command_rate_requested_hz']:.0f}:app {acc.get('accepted')}/{r['commands_sent']} age[{f(lo)},{f(hi)}]ms[{str(acc.get('status'))[:3]}]"
    log(f"{name} -> {rec['result']['outcome']} rate={rec['result']['estimates'].get('sub_verdicts', {}).get('rate')} " + " ".join(one(r) for r in rows))


def exp_rate(out, quick):
    """Rate expectation decided on the source age of the APPLIED setpoint (setpoint_cp).  The truth of every window is
    derived by analyze_v013.py from the mock's event log (the largest source age of an applied command over the
    window, from the apply events matched to the probe's send stamps), not from the configuration; the configuration
    is archived as the case description."""
    combos = [(120, 120), (120, 1000), (1500, 100), (1500, 1000)] if not quick else [(120, 1000)]
    noises_mm = [0.0, 0.05, 0.2, 0.5] if not quick else [0.0, 0.5]
    rates = (50, 100, 200, 500, 1000) if not quick else (100, 500)
    mk = lambda a: RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=rates, expectations=E_RATE)
    j = 0
    for loop, pub in combos:
        for noise in noises_mm:
            name = f"T_rate_{j:03d}"
            over = {"loop_rate_hz": loop, "publish_rate_hz": pub, "noise_m": noise / 1000.0, "seed": SEED0 + 300 + j}
            rec = run_probe(mk, "reference", over, expectation=E_RATE, event_log=evlog(out, name))
            rec["truth"] = {"loop_rate_hz": loop, "publish_rate_hz": pub, "noise_m": noise / 1000.0, "accept_every_k": 1, "setpoint_channel": True, "case": "every command applied, latest wins"}
            save(out, name, rec); _rate_log(name, rec)
            j += 1
    extra = []
    # every k-th command applied (RC3 review, finding 2)
    for k, ak in enumerate([2, 3, 10, 100]):
        extra.append((f"T_accept_k{ak}", {"accept_every_k": ak, "max_speed_m_s": 0.05, "publish_rate_hz": 500, "loop_rate_hz": 1000, "seed": SEED0 + 350 + k}, {"accept_every_k": ak, "max_speed_m_s": 0.05, "setpoint_channel": True, "case": "every k-th command applied"}))
    extra.append(("T_final_only_fast", {"accept_every_k": 100, "publish_rate_hz": 500, "loop_rate_hz": 1000, "seed": SEED0 + 360}, {"accept_every_k": 100, "setpoint_channel": True, "case": "instantaneous controller applying every 100th command: crossings are not produced either"}))
    extra.append(("T_no_setpoint_channel", {"publish_setpoint_cp": False, "seed": SEED0 + 361}, {"accept_every_k": 1, "setpoint_channel": False, "case": "no applied-setpoint channel: undetermined"}))
    extra.append(("T_no_setpoint_channel_k100", {"publish_setpoint_cp": False, "accept_every_k": 100, "max_speed_m_s": 0.05, "seed": SEED0 + 362}, {"accept_every_k": 100, "setpoint_channel": False, "case": "the RC3 reviewer's case without the channel: undetermined, not satisfied"}))
    for k, sp in enumerate([0.05, 0.2] if not quick else [0.05]):
        extra.append((f"T_interp_{k}", {"max_speed_m_s": sp, "publish_rate_hz": 500, "loop_rate_hz": 1000, "seed": SEED0 + 320 + k}, {"max_speed_m_s": sp, "accept_every_k": 1, "setpoint_channel": True, "case": "rate-limited motion; the channel reports the applied goal, not the interpolated pose"}))
    for k, dp in enumerate([0.1, 0.5]):
        extra.append((f"T_drop_{k}", {"drop_prob": dp, "seed": SEED0 + 330 + k}, {"drop_prob": dp, "accept_every_k": 1, "setpoint_channel": True, "case": "commands dropped at random in the implementation callback (the realised drop sequence is in the event log)"}))
    for k, jit in enumerate([0.005, 0.02]):
        extra.append((f"T_delay_{k}", {"response_delay_s": 0.05, "response_jitter_s": jit, "seed": SEED0 + 340 + k}, {"response_delay_s": 0.05, "response_jitter_s": jit, "accept_every_k": 1, "setpoint_channel": True, "case": "every command applied 50 ms late (RC4 review, finding 1: applied late is not applied)"}))
    # RC4 review, finding 1: an ordered buffer slower than the client applies ever older commands while the channel
    # changes regularly; and one that keeps up
    for k, lr in enumerate([50, 80, 120, 1000]):
        extra.append((f"T_fifo_{lr}", {"queue_policy": "fifo", "loop_rate_hz": lr, "publish_rate_hz": 500, "seed": SEED0 + 370 + k}, {"queue_policy": "fifo", "loop_rate_hz": lr, "accept_every_k": 1, "setpoint_channel": True, "case": f"FIFO buffer applying at most one command per tick at {lr} Hz"}))
    # RC4 review, finding 3: the channel reports the previous command (a pre-existing setpoint) -- every 2nd
    # command with a 30 ms application delay: the sample after a send shows the command before it
    extra.append(("T_stale_previous", {"accept_every_k": 2, "response_delay_s": 0.03, "publish_rate_hz": 500, "loop_rate_hz": 1000, "seed": SEED0 + 380}, {"accept_every_k": 2, "response_delay_s": 0.03, "setpoint_channel": True, "case": "the channel shows the previous command at every send (pre-existing setpoint), applied 30 ms late"}))
    # jitter larger than the spacing with latest-wins: superseded commands are never applied
    extra.append(("T_jitter_large", {"response_delay_s": 0.02, "response_jitter_s": 0.06, "seed": SEED0 + 381}, {"response_delay_s": 0.02, "response_jitter_s": 0.06, "setpoint_channel": True, "case": "delay 20-80 ms, latest wins: superseded commands are never applied"}))
    # slow channel: publication sparser than the required period -- the bracket must widen, never pass
    extra.append(("T_sparse_channel", {"publish_rate_hz": 20, "seed": SEED0 + 382}, {"publish_rate_hz": 20, "accept_every_k": 1, "setpoint_channel": True, "case": "channel published at 20 Hz (period 50 ms > required 20 ms): the bracket cannot show satisfaction"}))
    for name, over, truth in extra:
        rec = run_probe(mk, "reference", over, expectation=E_RATE, event_log=evlog(out, name))
        rec["truth"] = truth
        save(out, name, rec); _rate_log(name, rec)


# ------------------------------------------------------------------------------------------ L (liveness)
def exp_liveness(out, quick):
    """Liveness: the estimate is an interval; the truth is the injected timeout.  The analysis checks containment and
    the horizon / eq. (7) verdicts against the injected policy (RC4 review, finding 4)."""
    trials = 5 if not quick else 2
    steps = 7 if not quick else 4
    need = 1.0 / TOL.client_rate_hz + TOL.jitter_max_s
    mk = lambda e: (lambda a: RateSensitivityProbe(a, TOL, trials=trials, gap_max_s=1.5, bisection_steps=steps, rates_hz=(100,), expectations=e))
    taus = [0.012, 0.015, 0.02, 0.03, 0.05, 0.1, 0.25, 0.5, 1.0] if not quick else [0.03, 0.25]
    for k, tau in enumerate(taus):
        name = f"L_fault_{k:03d}"
        over = {"watchdog_s": tau, "watchdog_mode": "fault", "seed": SEED0 + 200 + k}
        rec = run_probe(mk(E_FAULT), "reference", over, expectation=E_FAULT, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": tau, "mode": "fault", "eq7_need_s": need}
        save(out, name, rec)
        L = rec["result"]["observations"]["liveness"]
        log(f"{name} tau={tau} -> {rec['result']['outcome']} floor={rec['result']['observations']['resolution']['resolution_floor_s']*1e3:.1f}ms | {L['finding']}")
    k = 0
    for tau in ([0.1, 0.5] if not quick else [0.5]):
        for drift in ([0.02, 0.005] if not quick else [0.02]):
            over = {"watchdog_s": tau, "watchdog_mode": "release", "release_drift_m_s": drift, "seed": SEED0 + 220 + k, "state_machine": False, "require_enabled": False}
            for ename, e in (("hold", E_HOLD), ("drift", E_DRIFT)):
                name = f"L_release_{k:03d}_{ename}"
                rec = run_probe(mk(e), "reference", over, expectation=e, event_log=evlog(out, name))
                rec["truth"] = {"tau_w_s": tau, "mode": "release", "drift_m_s": drift, "expect": ename, "note": "drift speed is a validation parameter", "eq7_need_s": need}
                save(out, name, rec)
                log(f"{name} tau={tau} drift={drift} -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")
            k += 1
    # RC4 review, finding 4: the declared horizon below, across and above the timeout interval, fault and drift
    for h in ([0.1, 0.25, 1.0] if not quick else [0.1, 1.0]):
        e = Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": h}})
        name = f"L_horizon_fault_{int(h*1000):04d}"
        rec = run_probe(mk(e), "reference", {"watchdog_s": 0.25, "watchdog_mode": "fault", "seed": SEED0 + 260}, expectation=e, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.25, "mode": "fault", "horizon_s": h, "eq7_need_s": need, "case": "horizon " + ("below" if h < 0.25 else "at" if h == 0.25 else "above") + " the timeout"}
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} sub={rec['result']['estimates']['sub_verdicts']['stop_behaviour']} | {rec['result']['observations']['liveness']['finding']}")
    for h in ([0.2, 0.5, 1.5] if not quick else [0.2, 1.5]):
        e = Expectations.from_dict({"temporal": {"stop_behaviour": "drift", "horizon_s": h}})
        name = f"L_horizon_drift_{int(h*1000):04d}"
        rec = run_probe(mk(e), "reference", {"watchdog_s": 0.5, "watchdog_mode": "release", "release_drift_m_s": 0.02, "seed": SEED0 + 261, "state_machine": False, "require_enabled": False}, expectation=e, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.5, "mode": "release", "drift_m_s": 0.02, "horizon_s": h, "eq7_need_s": need, "case": "horizon " + ("below" if h < 0.5 else "at" if h == 0.5 else "above") + " the onset"}
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} sub={rec['result']['estimates']['sub_verdicts']['stop_behaviour']} | {rec['result']['observations']['liveness']['finding']}")
    # the RC4 reviewer's replay: a 1.0 s fault policy against 'fault within 0.1 s' (0.1.2 said satisfied)
    e = Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": 0.1}})
    rec = run_probe(mk(e), "reference", {"watchdog_s": 1.0, "watchdog_mode": "fault", "seed": SEED0 + 262}, expectation=e, event_log=evlog(out, "L_rc4_reviewer_horizon"))
    rec["truth"] = {"tau_w_s": 1.0, "mode": "fault", "horizon_s": 0.1, "eq7_need_s": need, "case": "RC4 review finding 4: must be violated"}
    save(out, "L_rc4_reviewer_horizon", rec)
    log(f"L_rc4_reviewer_horizon -> {rec['result']['outcome']} sub={rec['result']['estimates']['sub_verdicts']['stop_behaviour']}")
    # drift-free release: the pose does not move and the next command is acted on; from the pose alone this is a hold
    over = {"watchdog_s": 0.5, "watchdog_mode": "release", "release_drift_m_s": 0.0, "seed": SEED0 + 230, "state_machine": False, "require_enabled": False}
    for ename, e in (("hold", E_HOLD), ("release", E_RELEASE)):
        name = f"L_release_nodrift_{ename}"
        rec = run_probe(mk(e), "reference", over, expectation=e, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.5, "mode": "release", "drift_m_s": 0.0, "expect": ename, "note": "a release without drift is not observable from the pose"}
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")
    rec = run_probe(mk(E_HOLD), "reference", {"seed": SEED0 + 231}, expectation=E_HOLD, event_log=evlog(out, "L_hold"))
    rec["truth"] = {"tau_w_s": 0.0, "mode": "none"}
    save(out, "L_hold", rec)
    log(f"L_hold -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")
    e_h = Expectations.from_dict({"temporal": {"stop_behaviour": "hold", "horizon_s": 3.0}})
    rec = run_probe(mk(e_h), "reference", {"seed": SEED0 + 232}, expectation=e_h, event_log=evlog(out, "L_hold_horizon_beyond"))
    rec["truth"] = {"tau_w_s": 0.0, "mode": "none", "horizon_s": 3.0, "tested_s": 1.5}
    save(out, "L_hold_horizon_beyond", rec)
    log(f"L_hold_horizon_beyond -> {rec['result']['outcome']} | {rec['result']['decision_basis'][:120]}")
    over = {"watchdog_s": 2.0, "watchdog_mode": "fault", "seed": SEED0 + 233}
    for ename, e in (("hold", E_HOLD), ("fault", E_FAULT)):
        name = f"L_beyond_range_{ename}"
        rec = run_probe(mk(e), "reference", over, expectation=e, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 2.0, "mode": "fault", "expect": ename, "tested_s": 1.5, "note": "policy beyond the tested range"}
        save(out, name, rec)
        log(f"{name} -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")
    # a fault policy beyond the tested range with a horizon beyond it too: undetermined (not violated)
    e = Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": 3.0}})
    rec = run_probe(mk(e), "reference", {"watchdog_s": 2.0, "watchdog_mode": "fault", "seed": SEED0 + 234}, expectation=e, event_log=evlog(out, "L_beyond_range_fault_horizon"))
    rec["truth"] = {"tau_w_s": 2.0, "mode": "fault", "horizon_s": 3.0, "tested_s": 1.5, "note": "policy and horizon beyond the tested range: undetermined"}
    save(out, "L_beyond_range_fault_horizon", rec)
    log(f"L_beyond_range_fault_horizon -> {rec['result']['outcome']} sub={rec['result']['estimates']['sub_verdicts']['stop_behaviour']}")
    rec = run_probe(mk(E_FAULT), "reference", {"watchdog_s": 0.5, "watchdog_mode": "fault", "response_delay_s": 0.3, "seed": SEED0 + 240}, expectation=E_FAULT, event_log=evlog(out, "L_delayed"))
    rec["truth"] = {"tau_w_s": 0.5, "mode": "fault", "response_delay_s": 0.3}
    save(out, "L_delayed", rec)
    log(f"L_delayed -> {rec['result']['outcome']} timeout={rec['result']['observations']['resolution']['response_timeout_s']:.2f}s | {rec['result']['observations']['liveness']['finding']}")
    rec = run_probe(mk(E_FAULT), "reference", {"watchdog_s": 0.25, "watchdog_mode": "fault", "publish_rate_hz": 20, "seed": SEED0 + 250}, expectation=E_FAULT, event_log=evlog(out, "L_slowfb"))
    rec["truth"] = {"tau_w_s": 0.25, "mode": "fault", "publish_rate_hz": 20}
    save(out, "L_slowfb", rec)
    log(f"L_slowfb -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")
    # a client-supplied latency bound: the interval is unconditional (finding 6)
    mkb = lambda a: RateSensitivityProbe(a, TOL, trials=trials, gap_max_s=1.5, bisection_steps=steps, rates_hz=(100,), expectations=E_FAULT, latency_bound_s=0.005)
    rec = run_probe(mkb, "reference", {"watchdog_s": 0.25, "watchdog_mode": "fault", "seed": SEED0 + 251}, expectation=E_FAULT, event_log=evlog(out, "L_latency_bound"))
    rec["truth"] = {"tau_w_s": 0.25, "mode": "fault", "latency_bound_s": 0.005, "eq7_need_s": need, "note": "client-supplied one-way latency bound 5 ms"}
    save(out, "L_latency_bound", rec)
    log(f"L_latency_bound -> {rec['result']['outcome']} | {rec['result']['observations']['liveness']['finding']}")


# ------------------------------------------------------------------------------------------ P (state precondition)
def exp_state(out, quick):
    exps = {"required": Expectations.from_dict({"temporal": {"state_machine": "required"}}),
            "forbidden": Expectations.from_dict({"temporal": {"state_machine": "forbidden"}}),
            "discover": E_DISC}
    presets = ["reference", "emul-dvrk-jhu-psm2", "emul-src-v1"]
    for preset in presets:
        for ename, e in exps.items():
            rec = run_probe(lambda a, e=e: RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,), expectations=e), preset, {}, expectation=e, event_log=evlog(out, f"P_state_{preset}_{ename}"))
            rec["truth"] = {"preset": preset, "state_machine": preset != "emul-src-v1", "expect": ename}
            save(out, f"P_state_{preset}_{ename}", rec)
            log(f"P_state {preset} expect={ename} -> {rec['result']['outcome']} enable_latency={rec['result']['observations']['state_precondition'].get('enable',{}).get('enable_latency_s') if isinstance(rec['result']['observations']['state_precondition'].get('enable'),dict) else None}")
    rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,), expectations=exps["required"]), "reference", {"state_machine": False, "require_enabled": False}, expectation=exps["required"])
    rec["truth"] = {"state_machine": False, "expect": "required"}
    save(out, "P_state_missing_required", rec)
    log(f"P_state missing topic, required -> {rec['result']['outcome']}")


# ------------------------------------------------------------------------------------------ R (regressions)
def exp_regression(out, quick):
    e_all = Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}, "temporal": {"state_machine": "required", "stop_behaviour": "hold", "rate": "required"}})
    cases = {
        "R_noisy": ({"bind_translation_m": [0.002, 0, 0], "noise_m": 0.0005, "seed": SEED0 + 400}, e_all),
        "R_missing_cp": ({"publish_measured_cp": False, "seed": SEED0 + 401}, e_all),
        "R_nolocal": ({"publish_local": False, "seed": SEED0 + 402}, e_all),
        "R_drop_50": ({"drop_prob": 0.5, "seed": SEED0 + 403}, e_all),
        "R_delay_50ms_jitter_20ms": ({"response_delay_s": 0.05, "response_jitter_s": 0.02, "seed": SEED0 + 404}, e_all),
    }
    for name, (over, e) in cases.items():
        over = dict(over, event_log_path=evlog(out, name))
        with mock_node("reference", over):
            a = PlatformAdapter(NS, anchor_topic=ANCHOR)
            disc = a.discover()
            rs = [FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=5, expectations=e).run(), ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.6, expectations=e).run(),
                  RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.5, bisection_steps=3, rates_hz=(100, 500), expectations=e).run()]
            a.close()
        rec = {"case": name, "overrides": over, "expectation": e.to_dict(), "discovery": disc, "results": [r.to_dict() for r in rs], "event_log": name + ".events.jsonl"}
        save(out, name, rec)
        log(f"{name} -> " + ", ".join(f"{r.binding_class}={r.outcome.value}" for r in rs))


# ------------------------------------------------------------------------------------------ X (presets, full reports)
def exp_presets(out, quick):
    from crtk_mock.presets import PRESETS
    authored = {
        "reference": Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}, "temporal": {"state_machine": "required", "stop_behaviour": "hold", "rate": "required"}}),
        "emul-dvrk-jhu-psm2": Expectations.from_dict(dict(jhu_expectation().to_dict(), dimensional={"mode": "si"}, temporal={"state_machine": "required", "stop_behaviour": "hold", "rate": "required"})),
        "emul-src-v1": Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}, "temporal": {"state_machine": "forbidden", "stop_behaviour": "hold", "rate": "required"}}),
        "emul-src-v2": Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}, "temporal": {"state_machine": "forbidden", "stop_behaviour": "hold", "rate": "required"}}),
        "emul-ambf-object-watchdog": Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}, "temporal": {"state_machine": "forbidden", "stop_behaviour": "hold", "rate": "required"}}),
    }
    for name in PRESETS:
        for label, e in (("discover", E_DISC), ("authored", authored[name])):
            with mock_node(name, {"event_log_path": evlog(out, f"PRESET_{name}_{label}")}):
                a = PlatformAdapter(NS, anchor_topic=ANCHOR)
                disc = a.discover()
                rs = [FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=5, expectations=e).run(), ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.6, expectations=e).run(),
                      RateSensitivityProbe(a, TOL, trials=2 if not quick else 1, gap_max_s=1.0, bisection_steps=4, rates_hz=(50, 100, 200, 500, 1000), expectations=e).run()]
                rep = build_report(NS, TOL, disc, rs, os.environ.get("ROS_MASTER_URI", ""), expectations=e, parameters={"campaign": "v0.1.3 mock", "preset": name, "label": label, "event_log": f"PRESET_{name}_{label}.events.jsonl"})
                a.close()
            save(out, f"PRESET_{name}_{label}", rep)
            log(f"PRESET {name} [{label}] -> {rep['summary']}")


# ------------------------------------------------------------------------------------------ M (model check)
def exp_model_check(out, quick):
    """Execute absolute and incremental commands against the mock with the JHU binding injected (T_bind = base_frame,
    rotation -150 deg about x) and compare the executed (ground-truth) error with the model, whose transform is
    T = base_frame^-1.  Implementation verification: the mock and the model implement the same kinematics."""
    from crtk_conformance.probes.common import ensure_enabled, wait_settled
    from geometry_msgs.msg import PoseStamped

    Rb = G.axis_angle([1, 0, 0], math.radians(-150.0)); tb = np.array([0.20, 0.0, 0.0])
    T_base = G.make_pose(Rb, tb)
    T_model = G.invert(T_base)  # the manuscript's T: p in the unqualified frame -> base_frame^-1 p in local
    R, t = T_model[:3, :3], T_model[:3, 3]
    cases = []
    over = dict(JHU, seed=SEED0 + 500)
    with mock_node("reference", over):
        a = PlatformAdapter(NS, anchor_topic=ANCHOR); a.discover()
        buf = a.subscribe("measured_cp"); truth = a.subscribe("anchor", PoseStamped, full_topic=ANCHOR)
        ensure_enabled(a); a.wait_for(truth, 1.0)
        for pc in ([0.0, 0.02, 0.05, 0.10] if not quick else [0.0, 0.10]):
            for kind, vec in (("perp", np.array([0, 0, 1.0])), ("axis", np.array([1.0, 0, 0]))):
                p_c = vec * pc
                a.servo_cp(G.make_pose(None, p_c)); time.sleep(0.15); wait_settled(a, buf, 0.5)
                ex = a.latest_pose(truth, 1.0)[:3, 3]
                cases.append({"kind": "absolute_" + kind, "p_c_norm_m": pc, "measured_error_m": float(np.linalg.norm(ex - p_c)),
                              "predicted_error_m": G.m1_positional_error(R, t, p_c), "upper_bound_m": G.m1_upper_bound(R, t, pc), "lower_bound_exact_m": G.m1_lower_bound_exact(R, t),
                              "exact_max_over_ball_m": exact_max_error(R, t, pc)})
        for step in ([0.001, 0.005, 0.02] if not quick else [0.005]):
            for vec in (np.array([0, 0, 1.0]), np.array([1.0, 0, 0])):
                a.servo_cp(G.make_pose(None, [0, 0, 0])); time.sleep(0.15); wait_settled(a, buf, 0.5)
                p0_true = a.latest_pose(truth, 1.0)[:3, 3]
                m0 = a.latest_pose(buf, 1.0)
                goal = m0.copy(); goal[:3, 3] += vec * step
                a.servo_cp(goal); time.sleep(0.15); wait_settled(a, buf, 0.5)
                p1_true = a.latest_pose(truth, 1.0)[:3, 3]
                executed_step = p1_true - p0_true
                cases.append({"kind": "incremental_" + ("perp" if vec[2] else "axis"), "step_m": step,
                              "measured_error_m": float(np.linalg.norm(executed_step - vec * step)),
                              "predicted_error_m": float(np.linalg.norm((R - np.eye(3)) @ (vec * step))),
                              "upper_bound_m": G.m1_incremental_bound(R, step)})
        a.close()
    with mock_node("reference", {"unit_m": 0.1, "seed": SEED0 + 501}):
        a = PlatformAdapter(NS, anchor_topic=ANCHOR); a.discover()
        buf = a.subscribe("measured_cp"); truth = a.subscribe("anchor", PoseStamped, full_topic=ANCHOR)
        ensure_enabled(a); a.wait_for(truth, 1.0)
        for d in ([0.0, 0.02, 0.05, 0.10] if not quick else [0.10]):
            p_c = np.array([0, d, 0.0])
            a.servo_cp(G.make_pose(None, p_c)); time.sleep(0.15); wait_settled(a, buf, 0.5)
            ex = a.latest_pose(truth, 1.0)[:3, 3]
            cases.append({"kind": "scale_absolute", "p_c_norm_m": d, "measured_error_m": float(np.linalg.norm(ex - p_c)), "predicted_error_m": G.m2_error(0.1, d)})
        a.close()
    # composition example, eq. (10): s = 0.1, JHU T, p_c = (0, 0, 0.1)
    with mock_node("reference", dict(JHU, unit_m=0.1, seed=SEED0 + 502)):
        a = PlatformAdapter(NS, anchor_topic=ANCHOR); a.discover()
        buf = a.subscribe("measured_cp"); truth = a.subscribe("anchor", PoseStamped, full_topic=ANCHOR)
        ensure_enabled(a); a.wait_for(truth, 1.0)
        p_c = np.array([0, 0, 0.10])
        a.servo_cp(G.make_pose(None, p_c)); time.sleep(0.15); wait_settled(a, buf, 0.5)
        ex = a.latest_pose(truth, 1.0)[:3, 3]
        s = 0.1
        cases.append({"kind": "composed_absolute", "p_c_norm_m": 0.10, "measured_error_m": float(np.linalg.norm(ex - p_c)),
                      "predicted_error_m": float(np.linalg.norm((s * R - np.eye(3)) @ p_c + t)),
                      "upper_bound_m": float(np.linalg.norm(t) + (abs(1 - s) + 2 * s * math.sin(math.radians(150) / 2)) * 0.10)})
        a.close()
    save(out, "M_model_check", {"binding": {"base_frame_t_m": tb.tolist(), "base_frame_angle_deg": -150.0, "model_T": "base_frame^-1", "model_t_m": t.tolist(), "model_theta_deg": math.degrees(G.rotation_angle(R))}, "cases": cases})
    for c in cases:
        log(f"M {c['kind']:18s} " + " ".join(f"{k}={v*1e3:.2f}mm" for k, v in c.items() if k.endswith("_m") and isinstance(v, float)))


def file_hashes(root):
    h = {}
    for dp, _, fs in os.walk(root):
        for f in sorted(fs):
            p = os.path.join(dp, f)
            if (f.endswith(".json") or f.endswith(".jsonl")) and f != "meta.json":
                h[os.path.relpath(p, root)] = hashlib.sha256(open(p, "rb").read()).hexdigest()
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.3", "mock"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default="F,C,S,T,L,P,R,X,M")
    ap.add_argument("--port", type=int, default=11611)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    import scipy, jsonschema
    try:
        import matplotlib
        mpl = matplotlib.__version__
    except Exception:
        mpl = None
    meta = {
        "crtk_conformance_version": __version__,
        "repo_commit": git_rev(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")),
        "python": sys.version, "platform": platform.platform(),
        "numpy": np.__version__, "scipy": scipy.__version__, "jsonschema": getattr(jsonschema, "__version__", None), "matplotlib": mpl,
        "ros_distro": os.environ.get("ROS_DISTRO"), "container_image": os.environ.get("RC3_CONTAINER_IMAGE"), "container_digest": os.environ.get("RC3_CONTAINER_DIGEST"),
        "ros_source_manifest": os.environ.get("RC3_ROS_MANIFEST"),
        "tolerance": TOL.to_dict(), "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "quick": args.quick,
        "command": " ".join(sys.argv), "plan": "VALIDATION_PLAN_V013.md",
    }
    t0 = time.time()
    with ros_master(port=args.port):
        if "F" in args.only:
            exp_frame(args.out, args.quick)
        if "C" in args.only:
            exp_coverage(args.out, args.quick)
        if "S" in args.only:
            exp_scale(args.out, args.quick)
        if "T" in args.only:
            exp_rate(args.out, args.quick)
        if "L" in args.only:
            exp_liveness(args.out, args.quick)
        if "P" in args.only:
            exp_state(args.out, args.quick)
        if "R" in args.only:
            exp_regression(args.out, args.quick)
        if "X" in args.only:
            exp_presets(args.out, args.quick)
        if "M" in args.only:
            exp_model_check(args.out, args.quick)
    meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    meta["wall_s"] = time.time() - t0
    meta["sha256"] = file_hashes(args.out)
    save(args.out, "meta", meta)
    log("done in %.0f s" % meta["wall_s"])


if __name__ == "__main__":
    main()
