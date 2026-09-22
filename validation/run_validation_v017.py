"""Confirmatory reference-node campaign of crtk-conformance 0.1.7 (plan: validation/v0.1.7/PREREGISTRATION.md).

The two 0.1.7 decision rules (consistency gate; sound fault bound) were introduced after the pre-registered v0.1.6
campaigns and first re-derived offline (validation/rederive_v017.py).  This campaign re-runs, with fresh seeds, the only
two experiments whose outcomes those rules can change -- WP4 (K1 and its shared-binding controls) and WP6 (liveness
under command loss) -- with the frozen 0.1.7 code, so that 0.1.7 is the evaluated system.

Usage (inside the campaign image; this repository mounted at /repo):
    python3 validation/run_validation_v017.py --only K,L

Seeds start at 70000 (independent of every earlier campaign).  Nothing here is a measurement of any platform.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import platform
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from harness import ros_master  # noqa: E402
from run_validation_v013 import save, log, git_rev, NS  # noqa: E402
from run_validation_v016 import run_probe, TOL, JHU, E_AXIS, LOSS, POLICY, jhu_si_expectation  # noqa: E402

from crtk_conformance import __version__, geometry as G  # noqa: E402
from crtk_conformance.expectations import Expectations  # noqa: E402
from crtk_conformance.probes.base import Outcome, ProbeResult  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.report import build_report  # noqa: E402
from crtk_conformance.spatial import exact_max_error  # noqa: E402

SEED0 = 70000
RERUN = set()


def as_result(d: dict) -> ProbeResult:
    """A ProbeResult rebuilt from its archived dict (to form the report-level summary and verdict kinds)."""
    names = {f.name for f in dataclasses.fields(ProbeResult)}
    kw = {k: v for k, v in d.items() if k in names}
    kw["outcome"] = Outcome(d["outcome"])
    return ProbeResult(**kw)


# ------------------------------------------------------------------ WP4 (0.1.7)
def exp_K(out):
    """As validation/run_validation_v016.exp_K, with the 0.1.7 consistency gate on and the report-level verdict kinds."""
    E = G.make_pose(G.axis_angle(E_AXIS, math.radians(10.0)), np.array(E_AXIS) / np.linalg.norm(E_AXIS) * 0.020)
    T_jhu = G.make_pose(G.axis_angle([1, 0, 0], math.radians(-150.0)), [0.20, 0, 0])
    C = T_jhu @ E
    from scipy.spatial.transform import Rotation as Rot
    rv = Rot.from_matrix(C[:3, :3]).as_rotvec()
    ang = float(np.linalg.norm(rv))
    cmd = {"cmd_bind_translation_m": [float(v) for v in C[:3, 3]], "cmd_bind_axis": [float(v) for v in rv / ang], "cmd_bind_angle_deg": math.degrees(ang)}
    exp = jhu_si_expectation()
    cases = [(f"K17_K1_{k}", "reference", dict(JHU, **cmd, noise_m=2e-5, seed=SEED0 + 10 + k), exp, "K1: command binding JHU*E, feedback JHU") for k in range(3)]
    cases += [(f"K17_ctrl_reference_{k}", "reference", {"noise_m": 2e-5, "seed": SEED0 + 20 + k}, Expectations.from_dict({"spatial": {"mode": "identity"}, "dimensional": {"mode": "si"}}),
               "shared binding (identity)") for k in range(3)]
    cases += [(f"K17_ctrl_jhu_{k}", "emul-dvrk-jhu-psm2", {"noise_m": 2e-5, "seed": SEED0 + 30 + k}, exp, "shared binding (JHU)") for k in range(3)]
    for name, preset, over, e, what in cases:
        if RERUN and name not in RERUN:
            continue
        rf = run_probe(lambda a, e=e: FrameSemanticsProbe(a, TOL, correlation_guard=False, trials=10, samples_per_trial=5, expectations=e), preset, over, expectation=e)
        rs = run_probe(lambda a, e=e: ScalingUnitsProbe(a, TOL, trials=9, step_if=0.005, settle_s=0.6, expectations=e, consistency_gate=True), preset, over, expectation=e)
        rep = build_report(NS, TOL, rf.get("discovery") or {"topics": {}}, [as_result(rf["result"]), as_result(rs["result"])], expectations=e)
        cc = rs["result"]["estimates"].get("command_feedback_consistency", {})
        rec = {"case": name, "what": what, "preset": preset, "overrides": over, "frame": rf, "scale": rs,
               "report_summary": rep["summary"], "verdict_kinds": rep["verdict_kinds"], "report_version": rep["version"],
               "truth": {"command_binding_differs": "_K1_" in name, "E_residual_max_m": exact_max_error(E[:3, :3], E[:3, 3], 0.10) if "_K1_" in name else 0.0}}
        save(out, name, rec)
        log(f"{name}: frame {rf['result']['outcome']} | scale {rs['result']['outcome']} (ungated {rs['result']['estimates'].get('unit_outcome_without_consistency_gate')}) "
            f"| flag {cc.get('flag_non_shared_binding_or_tracking_deficit')} | kinds {rep['verdict_kinds']['spatial_command_semantic']} / {rep['summary']['dimensional']}")


# ------------------------------------------------------------------ WP6 (0.1.7)
def exp_L(out, n_rep, policies=("hold", "fault250"), losses=("none", "iid2", "iid5", "ge5")):
    k = 0
    for pol in policies:
        over0, exp, truth = POLICY[pol]
        for loss in losses:
            for j in range(n_rep):
                name = f"L17_017_{pol}_{loss}_{j:02d}"
                ev = os.path.join(out, name + ".events.jsonl")
                over = dict(over0, **LOSS[loss], seed=SEED0 + 20000 + k)
                k += 1
                if RERUN and name not in RERUN:
                    continue
                rec = run_probe(lambda a: RateSensitivityProbe(a, TOL, trials=5, gap_max_s=2.0, expectations=exp, step_if=0.002, calibration_commands=30,
                                                               liveness_rule="0.1.7", skip_rate_sweep=True), "reference", over, expectation=exp, event_log=ev)
                rec["truth"] = dict(truth, loss=loss, rule="0.1.7")
                save(out, name, rec)
                L = rec["result"]["observations"].get("liveness")
                if L is None:
                    log(f"{name}: NO LIVENESS OBSERVATION: {rec['result']['decision_basis']} {rec['result'].get('notes')}")
                    continue
                tau = L.get("tau_w_estimate_s") or {}
                ce = tau.get("conditional_estimate_s") or {}
                log(f"{name}: class {L.get('stop_class')} status {tau.get('status')} [{tau.get('interval_low_s')}, {tau.get('interval_high_s')}] "
                    f"conditional [{ce.get('interval_low_s')}, {ce.get('interval_high_s')}] sub {rec['result']['estimates'].get('sub_verdicts', {}).get('stop_behaviour')} "
                    f"({rec['wall_s']:.0f}s)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="K,L")
    ap.add_argument("--out", default=os.path.join(HERE, "v0.1.7", "mock"))
    ap.add_argument("--n-loss", type=int, default=8)
    ap.add_argument("--port", type=int, default=11417)
    ap.add_argument("--rerun", default=None, help="comma-separated run names to re-run (same seeds), e.g. runs lost to a start-up failure")
    a = ap.parse_args()
    if a.rerun:
        RERUN.update(a.rerun.split(","))
    os.makedirs(a.out, exist_ok=True)
    meta = {"crtk_conformance_version": __version__, "repo_commit": git_rev(os.path.join(HERE, "..")), "python": sys.version, "platform": platform.platform(),
            "numpy": np.__version__, "tolerance": TOL.to_dict(), "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "only": a.only,
            "n_loss": a.n_loss, "seed0": SEED0, "rerun": sorted(RERUN)}
    json.dump(meta, open(os.path.join(a.out, f"meta_{a.only.replace(',', '')}{'_rerun' if RERUN else ''}.json"), "w"), indent=1)
    with ros_master(a.port):
        from harness import mock_node
        from crtk_conformance.adapter import PlatformAdapter
        for _ in range(3):  # discarded warm-up start-up (as in the v0.1.6 campaign)
            with mock_node("reference", {"seed": 1}):
                ok = PlatformAdapter(NS, anchor_topic=None).discover()["topics"]["servo_cp"]["present"]
            log(f"warm-up: node discovered = {ok}")
            if ok:
                break
        for part in a.only.split(","):
            t0 = time.time()
            if part == "K":
                exp_K(os.path.join(a.out, "K"))
            elif part == "L":
                exp_L(os.path.join(a.out, "L"), a.n_loss)
            log(f"part {part} done in {time.time()-t0:.0f} s")
