"""Command-line entry point: `crtk-conformance run --namespace /PSM1 --tolerance-mm 1 [--expectations expectations.yaml] ...`

0.1.1: the client's expectations are read from a YAML file (see expectations.py).  Without one every class
is discover-only: the probes report what they observe and return `undetermined`.  Implementation constants
that affect an outcome are exposed as options and recorded in the report.
"""
from __future__ import annotations

import argparse
import json
import os
import sys


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="crtk-conformance", description="Black-box semantic conformance probes for a CRTK/ROS arm interface (ROS 1).")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="run the probes against a namespace")
    r.add_argument("--namespace", default="/PSM1")
    r.add_argument("--tolerance-mm", type=float, required=True, help="task-space tolerance epsilon (mm); drives every decision threshold")
    r.add_argument("--workspace-radius-m", type=float, default=0.10)
    r.add_argument("--speed-mm-s", type=float, default=50.0)
    r.add_argument("--client-rate-hz", type=float, default=100.0)
    r.add_argument("--jitter-max-ms", type=float, default=5.0)
    r.add_argument("--expectations", default=None, help="YAML file declaring what the client expects per binding class (default: discover-only)")
    r.add_argument("--trials", type=int, default=10, help="trials for the frame and scale probes")
    r.add_argument("--temporal-trials", type=int, default=5, help="trials for the liveness sub-probe (default 5)")
    r.add_argument("--anchor-topic", default=None, help="PoseStamped topic with an out-of-band SI reference pose of the same tool (enables the unit estimate)")
    r.add_argument("--expect-state-machine", choices=["yes", "no", "any"], default=None, help="(legacy) overrides temporal.state_machine in the expectations file")
    r.add_argument("--probes", default="frame,scale,rate", help="comma-separated: frame, scale, rate, geometry (0.1.6: instrument-geometry unit anchor; needs dimensional.mode geometry_anchor)")
    r.add_argument("--geometry-trials", type=int, default=5, help="trials of the instrument-geometry anchor (0.1.6)")
    r.add_argument("--geometry-settle-s", type=float, default=1.0, help="settle time per joint step of the geometry anchor (0.1.6)")
    r.add_argument("--calibration-commands", type=int, default=30, help="commands sent without a preceding silence to estimate baseline command loss (0.1.6; 0.1.5 used 12)")
    r.add_argument("--liveness-rule", choices=["0.1.7", "0.1.6", "0.1.5"], default="0.1.7", help="0.1.7: the 0.1.6 confirmation rule, and every faulted trial bounded by the time its FAULT was observed (sound under command loss); 0.1.6: a non-response counts as a stop policy only when confirmed at the same gap (see liveness.py); 0.1.5: the archived rule")
    r.add_argument("--no-consistency-gate", action="store_true", help="(0.1.6 behaviour) keep a unit verdict when the command/feedback consistency diagnostic is raised")
    r.add_argument("--no-correlation-guard", action="store_true", help="(0.1.7 behaviour) frame probe: back-to-back trials, no resting window and no effective-sample-size guard")
    r.add_argument("--anchor-method", choices=["poe", "single_joint"], default=None, help="geometry anchor: 'poe' (0.1.8 default: joint-space fit to the measured joint changes) or 'single_joint' (0.1.6/0.1.7); default: the declaration's dimensional.anchor_method")
    r.add_argument("--enable-timeout-s", type=float, default=3.0, help="how long ensure_enabled waits for ENABLED and homed after enable/home (0.1.6; implementation constant)")
    r.add_argument("--skip-rate-sweep", action="store_true", help="skip the effective-rate diagnostic sweep (campaign speed; the rate sub-verdict is undetermined in every case)")
    r.add_argument("--gap-max-s", type=float, default=2.0)
    r.add_argument("--gap-min-s", type=float, default=0.0, help="smallest gap to request (the measured resolution floor applies if larger)")
    r.add_argument("--bisection-steps", type=int, default=7)
    r.add_argument("--rates-hz", default="50,100,200,500,1000")
    r.add_argument("--response-timeout-s", type=float, default=None, help="post-gap response timeout (default: 5 x measured p95 response latency, >= 0.2 s)")
    r.add_argument("--rate-match-tolerance-mm", type=float, default=None, help="matching tolerance for the rate estimator (default: 5 x measured resting noise)")
    r.add_argument("--rate-window-s", type=float, default=1.0)
    r.add_argument("--rate-max-step-mm", type=float, default=50.0, help="largest excursion of the rate sweep from the start pose (interface units x 1e-3)")
    r.add_argument("--settle-s", type=float, default=1.0, help="settle time per scale-probe step")
    r.add_argument("--still-tol-mm", type=float, default=0.01, help="motion below this is 'no response' (scale probe) / 'still' (settling)")
    r.add_argument("--latency-bound-s", type=float, default=None, help="externally justified bound on the one-way transport latency: used by the liveness interval for the transaction that draws no response, and subtracted from the rate verdict's age lower bound; without it the liveness interval uses the run maximum of the observed response latencies and is labelled conditional, and a violated rate verdict is labelled conditional on the channel's transport delay (0.1.3)")
    r.add_argument("--pairing-window-ms", type=float, default=50.0, help="max |stamp difference| for a measured_cp / local/measured_cp pair (frame probe)")
    r.add_argument("--step-mm", type=float, default=5.0, help="scale-probe step in interface units x 1e-3")
    r.add_argument("--temporal-step-mm", type=float, default=2.0, help="temporal-probe step in interface units x 1e-3")
    r.add_argument("--out", default="report.json")
    r.add_argument("--quiet", action="store_true")
    return p


def run(args) -> int:
    from .adapter import PlatformAdapter
    from .expectations import Expectations
    from .thresholds import Tolerance
    from .probes.frame import FrameSemanticsProbe
    from .probes.scale import ScalingUnitsProbe
    from .probes.rate import RateSensitivityProbe
    from .probes.geometry import GeometryAnchorProbe
    from .report import build_report, text_summary

    tol = Tolerance(
        epsilon_m=args.tolerance_mm / 1000.0,
        workspace_radius_m=args.workspace_radius_m,
        speed_m_s=args.speed_mm_s / 1000.0,
        client_rate_hz=args.client_rate_hz,
        jitter_max_s=args.jitter_max_ms / 1000.0,
    )
    exp = Expectations.load(args.expectations)
    if args.expect_state_machine is not None:
        exp.temporal.state_machine = {"yes": "required", "no": "forbidden", "any": "any"}[args.expect_state_machine]
    from .probes import common as _common
    _common.ENABLE_TIMEOUT_S = float(args.enable_timeout_s)
    a = PlatformAdapter(args.namespace, anchor_topic=args.anchor_topic)
    disc = a.discover()
    results = []
    wanted = [s.strip() for s in args.probes.split(",") if s.strip()]
    if "frame" in wanted:
        results.append(FrameSemanticsProbe(a, tol, trials=args.trials, expectations=exp, pairing_window_s=args.pairing_window_ms / 1000.0,
                                           correlation_guard=not args.no_correlation_guard).run())
    if "scale" in wanted:
        results.append(ScalingUnitsProbe(a, tol, trials=args.trials, step_if=args.step_mm / 1000.0, settle_s=args.settle_s, expectations=exp,
                                         still_tol_m=args.still_tol_mm / 1000.0, consistency_gate=not args.no_consistency_gate).run())
    if "geometry" in wanted:
        results.append(GeometryAnchorProbe(a, tol, trials=args.geometry_trials, settle_s=args.geometry_settle_s, expectations=exp,
                                           method=args.anchor_method).run())
    if "rate" in wanted:
        rates = tuple(float(x) for x in args.rates_hz.split(",") if x.strip())
        results.append(RateSensitivityProbe(a, tol, trials=args.temporal_trials, gap_max_s=args.gap_max_s, gap_min_s=args.gap_min_s,
                                            bisection_steps=args.bisection_steps, rates_hz=rates, expectations=exp,
                                            step_if=args.temporal_step_mm / 1000.0, response_timeout_s=args.response_timeout_s,
                                            rate_match_tolerance_m=(args.rate_match_tolerance_mm / 1000.0) if args.rate_match_tolerance_mm is not None else None,
                                            rate_window_s=args.rate_window_s, rate_max_step_if=args.rate_max_step_mm / 1000.0, still_tol_m=args.still_tol_mm / 1000.0,
                                            latency_bound_s=args.latency_bound_s, calibration_commands=args.calibration_commands,
                                            liveness_rule=args.liveness_rule, skip_rate_sweep=args.skip_rate_sweep).run())
    params = {k: v for k, v in vars(args).items() if k not in ("cmd",)}
    rep = build_report(args.namespace, tol, disc, results, os.environ.get("ROS_MASTER_URI", ""), expectations=exp, parameters=params,
                       consistency_gate=not args.no_consistency_gate)
    with open(args.out, "w") as f:
        json.dump(rep, f, indent=2)
    if not args.quiet:
        print(text_summary(rep))
        print(f"report written to {args.out}")
    a.close()
    return 0


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "run":
        return run(args)
    return 1


if __name__ == "__main__":
    sys.exit(main())
