"""Command-line entry point: `crtk-conformance run --namespace /PSM1 --tolerance-mm 1 ...`"""
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
    r.add_argument("--trials", type=int, default=10)
    r.add_argument("--anchor-topic", default=None, help="PoseStamped topic with an out-of-band SI reference pose of the same tool (enables the unit estimate)")
    r.add_argument("--expect-state-machine", choices=["yes", "no", "any"], default="any")
    r.add_argument("--probes", default="frame,scale,rate")
    r.add_argument("--gap-max-s", type=float, default=2.0)
    r.add_argument("--out", default="report.json")
    r.add_argument("--quiet", action="store_true")
    return p


def run(args) -> int:
    from .adapter import PlatformAdapter
    from .thresholds import Tolerance
    from .probes.frame import FrameSemanticsProbe
    from .probes.scale import ScalingUnitsProbe
    from .probes.rate import RateSensitivityProbe
    from .report import build_report, text_summary

    tol = Tolerance(
        epsilon_m=args.tolerance_mm / 1000.0,
        workspace_radius_m=args.workspace_radius_m,
        speed_m_s=args.speed_mm_s / 1000.0,
        client_rate_hz=args.client_rate_hz,
        jitter_max_s=args.jitter_max_ms / 1000.0,
    )
    a = PlatformAdapter(args.namespace, anchor_topic=args.anchor_topic)
    disc = a.discover()
    results = []
    wanted = [s.strip() for s in args.probes.split(",") if s.strip()]
    if "frame" in wanted:
        results.append(FrameSemanticsProbe(a, tol, trials=args.trials).run())
    if "scale" in wanted:
        results.append(ScalingUnitsProbe(a, tol, trials=args.trials).run())
    if "rate" in wanted:
        results.append(RateSensitivityProbe(a, tol, trials=max(3, args.trials // 2), gap_max_s=args.gap_max_s, expect_state_machine=args.expect_state_machine).run())
    rep = build_report(args.namespace, tol, disc, results, os.environ.get("ROS_MASTER_URI", ""))
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
