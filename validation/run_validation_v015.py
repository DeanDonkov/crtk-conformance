#!/usr/bin/env python3
"""Validation campaign of crtk-conformance 0.1.5 against crtk-mock (plan: validation/v0.1.5/VALIDATION_PLAN_V015.md).

Usage:  python validation/run_validation_v015.py [--quick] [--out validation/v0.1.5/mock] [--only F,C,S,T,L,P,R,X,M,V]

The campaign design is that of v0.1.3 (run_validation_v013.py, reused unchanged: same cases, same mock presets, same
analysis in analyze_v013.py) with every seed offset by 30000 so that no random stream is shared with the v0.1.1 (0),
v0.1.2 (10000) or v0.1.3 (20000) campaigns.  It verifies the 0.1.5 liveness changes (RC7 adversarial review, findings
F1, F8, F10) on fresh measurements; the v0.1.3 archive remains the reported measurement set of the manuscript and is
not modified.

Added experiment V (0.1.5 verification cases, liveness focus):
  V_delayed_{k}        fault policy tau_w = 0.5 s with the mock applying every command 50 / 100 / 300 ms late: the last
                       streamed command's response latency r_i must be measured from an observed departure (F1); the
                       0.1.3 rule read it from the still-executing in-band stream (archived L_delayed: 0.2-9.7 ms);
  V_delayed_drift_{k}  release policy with drift, tau_w = 0.5 s, commands applied 100 / 300 ms late: onset-based lower
                       bound with the granularity term G (F8) under delayed application;
  V_drop_hold_{k}      no stop policy, commands dropped at random (p = 0.2, 0.5), client expects a hold: the post-gap
                       non-responses are confounded by baseline command loss and the expectation must be left
                       undetermined, not violated (F10; the archived R_drop_50 was violated in 0.1.3);
  V_drop_fault         fault policy tau_w = 0.25 s with p = 0.3 random drops, client expects a fault: the tripping class
                       is 'faulted' (operating state visible), which command loss cannot produce, so the verdict is
                       formed despite the loss;
  V_drop_reject        the same policy without the state machine (tripping class 'rejected') under p = 0.3 drops: the
                       0.1.5 rule cannot distinguish a rejection policy from loss and leaves the run undetermined --
                       the documented cost of the rule.
"""
from __future__ import annotations

import argparse
import os
import platform
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_validation_v013 as V13  # noqa: E402
from harness import ros_master  # noqa: E402

from crtk_conformance import __version__  # noqa: E402
from crtk_conformance.expectations import Expectations  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402

V13.SEED0 = 30000  # every seed of this campaign is offset from the v0.1.1 (0), v0.1.2 (10000) and v0.1.3 (20000) campaigns'
TOL, E_FAULT, E_HOLD, E_DRIFT = V13.TOL, V13.E_FAULT, V13.E_HOLD, V13.E_DRIFT
run_probe, save, evlog, log = V13.run_probe, V13.save, V13.evlog, V13.log


def _liveness_summary(rec):
    L = rec["result"]["observations"]["liveness"]
    tau = L.get("tau_w_estimate_s") or {}
    iv = (f"[{tau['interval_low_s']*1e3:.1f}, {tau['interval_high_s']*1e3:.1f}] ms" if tau.get("status") == "ok" else str(tau.get("status")))
    r = [t.get("last_stream_latency_s") for t in L.get("trials", [])]
    r = [x for x in r if isinstance(x, float) and x == x]
    dep = sum(1 for t in L.get("trials", []) if t.get("last_stream_departure_observed"))
    return (f"{rec['result']['outcome']} sub={rec['result']['estimates']['sub_verdicts'].get('stop_behaviour')} interval={iv} "
            f"r_i={'/'.join(f'{x*1e3:.1f}' for x in r) or '-'} ms departures={dep}/{len(L.get('trials', []))} "
            f"confounded={L.get('rejection_confounded_by_command_loss')} | {L.get('finding')}")


def exp_v015(out, quick):
    trials = 5 if not quick else 2
    steps = 7 if not quick else 4
    S = V13.SEED0
    mk = lambda e: (lambda a: RateSensitivityProbe(a, TOL, trials=trials, gap_max_s=1.5, bisection_steps=steps, rates_hz=(100,), expectations=e))
    for k, d in enumerate([0.05, 0.1, 0.3] if not quick else [0.3]):
        name = f"V_delayed_{k}"
        rec = run_probe(mk(E_FAULT), "reference", {"watchdog_s": 0.5, "watchdog_mode": "fault", "response_delay_s": d, "seed": S + 800 + k}, expectation=E_FAULT, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.5, "mode": "fault", "response_delay_s": d, "case": "F1: r_i measured from an observed departure; the 0.1.3 rule read the in-band stream"}
        save(out, name, rec)
        log(f"{name} delay={d} -> {_liveness_summary(rec)}")
    for k, d in enumerate([0.1, 0.3] if not quick else [0.3]):
        name = f"V_delayed_drift_{k}"
        over = {"watchdog_s": 0.5, "watchdog_mode": "release", "release_drift_m_s": 0.02, "response_delay_s": d, "seed": S + 810 + k, "state_machine": False, "require_enabled": False}
        rec = run_probe(mk(E_DRIFT), "reference", over, expectation=E_DRIFT, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.5, "mode": "release", "drift_m_s": 0.02, "response_delay_s": d, "case": "F8: onset lower bound with G, delayed application"}
        save(out, name, rec)
        log(f"{name} delay={d} -> {_liveness_summary(rec)}")
    for k, p in enumerate([0.2, 0.5] if not quick else [0.5]):
        name = f"V_drop_hold_{k}"
        e = Expectations.from_dict({"temporal": {"stop_behaviour": "hold", "horizon_s": 0.5}})
        rec = run_probe(mk(e), "reference", {"drop_prob": p, "seed": S + 820 + k}, expectation=e, event_log=evlog(out, name))
        rec["truth"] = {"tau_w_s": 0.0, "mode": "none", "drop_prob": p, "case": "F10: no stop policy; non-responses are command loss; must be undetermined, not violated"}
        save(out, name, rec)
        log(f"{name} p={p} -> {_liveness_summary(rec)}")
    name = "V_drop_fault"
    rec = run_probe(mk(E_FAULT), "reference", {"watchdog_s": 0.25, "watchdog_mode": "fault", "drop_prob": 0.3, "seed": S + 830}, expectation=E_FAULT, event_log=evlog(out, name))
    rec["truth"] = {"tau_w_s": 0.25, "mode": "fault", "drop_prob": 0.3, "case": "F10: a fault visible in the operating state is not confounded by command loss"}
    save(out, name, rec)
    log(f"{name} -> {_liveness_summary(rec)}")
    name = "V_drop_reject"
    over = {"watchdog_s": 0.25, "watchdog_mode": "fault", "drop_prob": 0.3, "seed": S + 831, "state_machine": False, "require_enabled": False}
    rec = run_probe(mk(E_FAULT), "reference", over, expectation=E_FAULT, event_log=evlog(out, name))
    rec["truth"] = {"tau_w_s": 0.25, "mode": "fault", "drop_prob": 0.3, "state_machine": False, "case": "F10: a rejection policy under command loss is indistinguishable from the loss: undetermined (documented cost)"}
    save(out, name, rec)
    log(f"{name} -> {_liveness_summary(rec)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "v0.1.5", "mock"))
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", default="F,C,S,T,L,P,R,X,M,V")
    ap.add_argument("--port", type=int, default=11615)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    import scipy, jsonschema
    try:
        import matplotlib
        mpl = matplotlib.__version__
    except Exception:
        mpl = None
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    meta = {
        "crtk_conformance_version": __version__,
        "repo_commit": V13.git_rev(root), "repo_dirty": V13.subprocess.call(["git", "-C", root, "diff", "--quiet", "HEAD"]) != 0,
        "python": sys.version, "platform": platform.platform(),
        "numpy": np.__version__, "scipy": scipy.__version__, "jsonschema": getattr(jsonschema, "__version__", None), "matplotlib": mpl,
        "ros_distro": os.environ.get("ROS_DISTRO"), "container_image": os.environ.get("RC3_CONTAINER_IMAGE"), "container_digest": os.environ.get("RC3_CONTAINER_DIGEST"),
        "ros_source_manifest": os.environ.get("RC3_ROS_MANIFEST"),
        "tolerance": TOL.to_dict(), "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "quick": args.quick,
        "command": " ".join(sys.argv), "plan": "validation/v0.1.5/VALIDATION_PLAN_V015.md", "seed_offset": V13.SEED0,
        "design": "run_validation_v013.py experiments F,C,S,T,L,P,R,X,M unchanged (seeds +30000) plus experiment V (0.1.5 verification)",
    }
    t0 = time.time()
    with ros_master(port=args.port):
        for key, fn in (("F", V13.exp_frame), ("C", V13.exp_coverage), ("S", V13.exp_scale), ("T", V13.exp_rate), ("L", V13.exp_liveness),
                        ("P", V13.exp_state), ("R", V13.exp_regression), ("X", V13.exp_presets), ("M", V13.exp_model_check), ("V", exp_v015)):
            if key in args.only:
                fn(args.out, args.quick)
    meta["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    meta["wall_s"] = time.time() - t0
    meta["sha256"] = V13.file_hashes(args.out)
    save(args.out, "meta", meta)
    log("done in %.0f s" % meta["wall_s"])


if __name__ == "__main__":
    main()
