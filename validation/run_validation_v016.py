"""RC9 reference-node campaigns of crtk-conformance 0.1.6 (plan: validation/v0.1.6/PREREGISTRATION.md, section 3).

Usage (inside the campaign image; this repository mounted at /repo):
    python3 validation/run_validation_v016.py --only K    # WP4: correct feedback, wrong command binding (+ shared-binding controls)
    python3 validation/run_validation_v016.py --only B    # WP5: live boundary confirmation (frame, scale)
    python3 validation/run_validation_v016.py --only L    # WP6: liveness under command loss, rules 0.1.5 and 0.1.6

Every run archives the full probe result with the injected truth and the declared expectation, and the event log of
temporal runs, as in the v0.1.3 campaign.  Seeds are offset to 30000 (independent of every earlier campaign).
Nothing here is a measurement of any platform.
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
from fractions import Fraction

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from harness import ros_master  # noqa: E402
from run_validation_v013 import run_probe as _run_probe, save, log, git_rev, NS, ANCHOR  # noqa: E402


def run_probe(*args, **kw):
    """run_validation_v013.run_probe, retried (at most twice) when the probe found the node's topics missing: a start-up
    race of the freshly spawned reference node (first L run of 21 Sep 2026), not a measurement.  Retries are logged in
    the record."""
    retries = []
    for attempt in range(3):
        rec = _run_probe(*args, **kw)
        if rec["result"].get("decision_basis") not in ("missing topics", "no data", "no measured_cp data", "missing measured_cp", "no local data"):
            break
        retries.append(rec["result"].get("decision_basis"))
        log(f"start-up failure ({rec['result'].get('decision_basis')}): retrying")
    rec["startup_retries"] = retries
    return rec

from crtk_conformance import __version__, geometry as G  # noqa: E402
from crtk_conformance.expectations import Expectations  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.spatial import exact_max_error  # noqa: E402

TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.10, speed_m_s=0.05, client_rate_hz=100.0, jitter_max_s=0.005)
JHU = {"bind_translation_m": [0.20, 0.0, 0.0], "bind_axis": [1.0, 0.0, 0.0], "bind_angle_deg": -150.0}
SEED0 = 30000
E_AXIS = [0.48, 0.60, 0.64]


def jhu_si_expectation() -> Expectations:
    R = G.axis_angle([1, 0, 0], math.radians(-150.0))
    x, y, z, w = G.rot_to_quat(R)
    return Expectations.from_dict({"spatial": {"mode": "expected_transform", "translation_m": [0.20, 0, 0], "quaternion_xyzw": [x, y, z, w]},
                                   "dimensional": {"mode": "si"}})


# ------------------------------------------------------------------ WP4
def exp_K(out):
    """K1: feedback binding JHU (declared correctly), command binding JHU * E (E = 20 mm, 10 deg about E_AXIS)."""
    E = G.make_pose(G.axis_angle(E_AXIS, math.radians(10.0)), np.array(E_AXIS) / np.linalg.norm(E_AXIS) * 0.020)
    T_jhu = G.make_pose(G.axis_angle([1, 0, 0], math.radians(-150.0)), [0.20, 0, 0])
    C = T_jhu @ E
    # express C as translation + axis-angle for the mock
    from scipy.spatial.transform import Rotation as Rot
    rv = Rot.from_matrix(C[:3, :3]).as_rotvec()
    ang = float(np.linalg.norm(rv))
    cmd = {"cmd_bind_translation_m": [float(v) for v in C[:3, 3]], "cmd_bind_axis": [float(v) for v in rv / ang], "cmd_bind_angle_deg": math.degrees(ang)}
    exp = jhu_si_expectation()
    cases = [(f"K1_{k}", "reference", dict(JHU, **cmd, noise_m=2e-5, seed=SEED0 + 10 + k), exp, "K1: command binding JHU*E, feedback JHU") for k in range(3)]
    cases += [(f"K_ctrl_reference_{k}", "reference", {"noise_m": 2e-5, "seed": SEED0 + 20 + k}, Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}}),
               "shared binding (identity)") for k in range(3)]
    cases += [(f"K_ctrl_jhu_{k}", "emul-dvrk-jhu-psm2", {"noise_m": 2e-5, "seed": SEED0 + 30 + k}, exp, "shared binding (JHU)") for k in range(3)]
    for name, preset, over, e, what in cases:
        rf = run_probe(lambda a, e=e: FrameSemanticsProbe(a, TOL, trials=10, samples_per_trial=5, expectations=e), preset, over, expectation=e)
        rs = run_probe(lambda a, e=e: ScalingUnitsProbe(a, TOL, trials=9, step_if=0.005, settle_s=0.6, expectations=e), preset, over, expectation=e)
        cc = rs["result"]["estimates"].get("command_feedback_consistency", {})
        rec = {"case": name, "what": what, "preset": preset, "overrides": over, "frame": rf, "scale": rs,
               "truth": {"command_binding_differs": name.startswith("K1"), "E_residual_max_m": exact_max_error(E[:3, :3], E[:3, 3], 0.10) if name.startswith("K1") else 0.0}}
        save(out, name, rec)
        log(f"{name}: frame {rf['result']['outcome']} | scale {rs['result']['outcome']} r_int {rs['result']['estimates'].get('internal_ratio',{}).get('mean')} "
            f"s_hat {rs['result']['estimates'].get('scale_anchored',{}).get('mean')} | consistency flag {cc.get('flag_non_shared_binding_or_tracking_deficit')}")


# ------------------------------------------------------------------ WP5
def exp_B(out, n_rep):
    E_ID = Expectations.from_dict({"spatial": {"mode": "identity"}})
    E_SI = Expectations.from_dict({"dimensional": {"mode": "si"}})
    k = 0
    for model, noise in (("gaussian", 2e-5), ("gaussian", 1e-4), ("mixture", 1e-6)):
        for rs in ("0.95", "1.00", "1.05"):
            ratio = Fraction(rs)
            t = float(ratio * Fraction(1, 1000))
            for j in range(n_rep):
                name = f"BF_{model}_{noise*1e3:g}mm_{rs}_{j:02d}"
                over = {"bind_translation_m": [t, 0.0, 0.0], "bind_angle_deg": 0.0, "bind_axis": [0, 0, 1], "noise_m": noise, "seed": SEED0 + 1000 + k,
                        "noise_model": model}
                k += 1
                rec = run_probe(lambda a: FrameSemanticsProbe(a, TOL, trials=10, samples_per_trial=5, expectations=E_ID), "reference", over, expectation=E_ID)
                rec["truth"] = {"ratio": rs, "E_true_m": t, "truth_conformant": ratio <= 1, "noise_model": model, "noise_m": noise}
                save(out, name, rec)
                sd = rec["result"]["estimates"].get("spatial_decision", {})
                log(f"{name}: {rec['result']['outcome']} [{sd.get('ci_low_m', float('nan'))*1e3:.4f}..{sd.get('ci_high_m', float('nan'))*1e3:.4f}] mm")
    for rs in ("0.95", "1.00", "1.05"):
        ratio = Fraction(rs)
        s = float(1 + ratio * Fraction(1, 1000) / Fraction(1, 10))
        for j in range(n_rep):
            name = f"BS_gaussian_0.1mm_{rs}_{j:02d}"
            over = {"unit_m": s, "noise_m": 1e-4, "seed": SEED0 + 5000 + k, "publish_local": False, "publish_T_b_w": True}
            k += 1
            rec = run_probe(lambda a: ScalingUnitsProbe(a, TOL, trials=9, step_if=0.005, settle_s=0.6, expectations=E_SI), "reference", over, expectation=E_SI)
            rec["truth"] = {"ratio": rs, "s": s, "E_true_m": float(ratio * Fraction(1, 1000)), "truth_conformant": ratio <= 1, "noise_m": 1e-4}
            save(out, name, rec)
            e = rec["result"]["estimates"].get("predicted_error_at_workspace_edge_m", {})
            log(f"{name}: {rec['result']['outcome']} [{(e.get('ci_low') or float('nan'))*1e3:.4f}..{(e.get('ci_high') or float('nan'))*1e3:.4f}] mm")


# ------------------------------------------------------------------ WP6
LOSS = {
    "none": {},
    "iid2": {"drop_prob": 0.02},
    "iid5": {"drop_prob": 0.05},
    "ge5": {"drop_model": "gilbert_elliott", "ge_p_gb": 0.0105, "ge_p_bg": 0.2, "ge_loss_good": 0.0, "ge_loss_bad": 1.0},
}
POLICY = {
    "hold": ({"watchdog_s": 0.0}, Expectations.from_dict({"temporal": {"stop_behaviour": "hold", "horizon_s": 2.0}}), {"tau_w_s": None, "mode": "hold"}),
    "fault250": ({"watchdog_s": 0.25, "watchdog_mode": "fault"}, Expectations.from_dict({"temporal": {"stop_behaviour": "fault"}}), {"tau_w_s": 0.25, "mode": "fault"}),
}


def exp_L(out, n_rep, rules=("0.1.5", "0.1.6"), policies=("hold", "fault250"), losses=("none", "iid2", "iid5", "ge5")):
    k = 0
    for rule in rules:
        n_calib = 12 if rule == "0.1.5" else 30
        for pol in policies:
            over0, exp, truth = POLICY[pol]
            for loss in losses:
                for j in range(n_rep):
                    name = f"L16_{rule.replace('.', '')}_{pol}_{loss}_{j:02d}"
                    ev = os.path.join(out, name + ".events.jsonl")
                    over = dict(over0, **LOSS[loss], seed=SEED0 + 20000 + k)
                    k += 1
                    rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=5, gap_max_s=2.0, expectations=exp, step_if=0.002, calibration_commands=n_calib,
                                                                   liveness_rule=rule, skip_rate_sweep=True), "reference", over, expectation=exp, event_log=ev)
                    rec["truth"] = dict(truth, loss=loss, rule=rule)
                    save(out, name, rec)
                    L = rec["result"]["observations"].get("liveness")
                    if L is None:  # no liveness observation (e.g. the node was not discovered): recorded, not retried
                        log(f"{name}: NO LIVENESS OBSERVATION: {rec['result']['decision_basis']} {rec['result'].get('notes')}")
                        continue
                    tau = L.get("tau_w_estimate_s") or {}
                    log(f"{name}: class {L.get('stop_class')} status {tau.get('status')} [{tau.get('interval_low_s')}, {tau.get('interval_high_s')}] "
                        f"outcome {rec['result']['outcome']} ({rec['wall_s']:.0f}s)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="K,B,L")
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.6", "mock"))
    ap.add_argument("--n-boundary", type=int, default=30)
    ap.add_argument("--n-loss", type=int, default=8)
    ap.add_argument("--port", type=int, default=11411)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    meta = {"crtk_conformance_version": __version__, "repo_commit": git_rev(os.path.join(HERE, "..")), "python": sys.version, "platform": platform.platform(),
            "numpy": np.__version__, "tolerance": TOL.to_dict(), "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "only": a.only,
            "n_boundary": a.n_boundary, "n_loss": a.n_loss}
    json.dump(meta, open(os.path.join(a.out, f"meta_{a.only.replace(',', '')}.json"), "w"), indent=1)
    with ros_master(a.port):
        for part in a.only.split(","):
            t0 = time.time()
            if part == "K":
                exp_K(os.path.join(a.out, "K"))
            elif part == "B":
                exp_B(os.path.join(a.out, "B"), a.n_boundary)
            elif part == "L":
                exp_L(os.path.join(a.out, "L"), a.n_loss)
            log(f"part {part} done in {time.time()-t0:.0f} s")
