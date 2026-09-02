"""ScalingUnitsProbe — dimensional binding class.

Two estimators, kept separate because the paper's error model (Section 5.2) proves they answer
different questions:

  internal consistency  r_int = ||delta p_measured|| / ||delta p_commanded||, both read through the
                        interface. Equals 1 for *any* uniform unit scale (feedback invisibility).
                        It detects only a mismatch between command and measurement scaling.
  anchored estimate     s_hat = ||delta p_anchor|| / ||delta p_commanded (interpreted in metres)||,
                        where the anchor is an out-of-band reference with known physical units
                        (an external tracker; in validation, the mock's ground truth). This is the
                        only estimator that can detect a uniform scale divergence.

Without an anchor the dimensional outcome is UNDETERMINED by construction; the internal ratio
is still reported.
"""
from __future__ import annotations

import math
import time
from typing import List

import numpy as np

from geometry_msgs.msg import PoseStamped

from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..stats import estimate
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult, decide
from .common import ensure_enabled, step_and_measure


class ScalingUnitsProbe:
    name = "ScalingUnitsProbe"

    def __init__(self, adapter: PlatformAdapter, tol: Tolerance, trials: int = 10, step_if: float = 0.005, settle_s: float = 1.0):
        """step_if: commanded displacement in *interface units* (the probe does not know the unit)."""
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.step = step_if
        self.settle = settle_s

    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "dimensional", Outcome.UNDETERMINED)
        disc = self.a.discovery or self.a.discover()
        if not (disc["topics"]["measured_cp"]["present"] and disc["topics"]["servo_cp"]["present"]):
            res.notes.append("measured_cp or servo_cp missing")
            res.decision_basis = "missing topics"
            res.duration_s = time.time() - t0
            return res
        buf = self.a.subscribe("measured_cp")
        res.observations["enable"] = ensure_enabled(self.a)
        anchor_buf = None
        if self.a.anchor_topic:
            anchor_buf = self.a.subscribe("anchor", PoseStamped, full_topic=self.a.anchor_topic)
            if self.a.wait_for(anchor_buf, 1.0) is None:
                res.notes.append(f"anchor topic {self.a.anchor_topic} configured but silent; anchored estimate unavailable")
                anchor_buf = None
        axes = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
        r_int: List[float] = []
        s_anc: List[float] = []
        per_axis = {0: [], 1: [], 2: []}
        latencies: List[float] = []
        trial_log = []
        for k in range(self.trials):
            ax = k % 3
            delta = self.step * axes[ax] * (1 if (k // 3) % 2 == 0 else -1)
            pa0 = self.a.latest_pose(anchor_buf, 1.0) if anchor_buf is not None else None
            r = step_and_measure(self.a, buf, delta, self.settle)
            if not r["ok"]:
                trial_log.append({"trial": k, "ok": False, "reason": r["reason"]})
                continue
            ratio = float(np.linalg.norm(r["delta_meas"]) / np.linalg.norm(delta))
            r_int.append(ratio)
            per_axis[ax].append(ratio)
            if not math.isnan(r["first_motion_s"]):
                latencies.append(r["first_motion_s"])
            entry = {"trial": k, "ok": True, "axis": ax, "delta_cmd_if": delta.tolist(), "delta_meas_if": r["delta_meas"].tolist(), "r_int": ratio, "first_motion_s": r["first_motion_s"]}
            if anchor_buf is not None and pa0 is not None:
                time.sleep(0.05)
                pa1 = self.a.latest_pose(anchor_buf, 1.0)
                if pa1 is not None:
                    d_anc = pa1[:3, 3] - pa0[:3, 3]
                    s = float(np.linalg.norm(d_anc) / np.linalg.norm(delta))  # delta interpreted as metres
                    s_anc.append(s)
                    entry["delta_anchor_m"] = d_anc.tolist()
                    entry["s_anchored"] = s
            trial_log.append(entry)
            # return to start
            back = r["p0"]
            self.a.servo_cp(back)
            time.sleep(self.settle * 0.5)
        res.observations["trials"] = trial_log
        e_int = estimate(r_int)
        res.estimates["internal_ratio"] = e_int.to_dict()
        res.estimates["internal_ratio_per_axis"] = {str(k): estimate(v).to_dict() for k, v in per_axis.items()}
        if latencies:
            res.estimates["response_latency_s"] = estimate(latencies).to_dict()
        if s_anc:
            e_s = estimate(s_anc)
            preds = [self.tol.dimensional_error(s) for s in s_anc]
            e_pred = estimate(preds)
            res.estimates["scale_anchored"] = e_s.to_dict()
            res.estimates["predicted_error_at_workspace_edge_m"] = e_pred.to_dict()
            res.predicted_error_m = e_pred.mean
            res.predicted_error_ci = [e_pred.ci_low, e_pred.ci_high]
            res.outcome = decide(e_pred.ci_low, e_pred.ci_high, self.tol.epsilon_m)
            res.decision_basis = (
                f"eq. (M2.1) with anchored s_hat = {e_s.mean:.4f} (95% CI {e_s.ci_low:.4f}..{e_s.ci_high:.4f}): "
                f"|1-s| r_ws = {e_pred.mean*1e3:.3f} mm vs epsilon = {self.tol.epsilon_m*1e3:.3f} mm"
            )
        else:
            res.decision_basis = "no unit anchor: uniform scale is invariant under every interface ratio (Section 5.2) -> undetermined by construction"
            res.notes.append(f"internal consistency ratio {e_int.mean:.4f} (n={e_int.n}) says nothing about the unit; supply --anchor-topic to test units")
            if e_int.n and abs(e_int.mean - 1.0) > 3 * max(e_int.std, 1e-6):
                res.notes.append("command and measurement scaling are inconsistent with each other (r_int != 1)")
        res.duration_s = time.time() - t0
        return res
