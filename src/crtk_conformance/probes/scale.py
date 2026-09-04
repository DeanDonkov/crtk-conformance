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

0.1.1: the decision is taken only against a declared expectation (`dimensional.mode: si`, with the unit
the client assumes, `expected_unit_m`); in discover-only mode the anchored estimate is reported without a
verdict.  A trial in which no motion is detected within the settle time is recorded as `no_response` and
excluded from the ratio (it is not a zero-motion measurement); fewer than 3 valid trials -> UNDETERMINED.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

from geometry_msgs.msg import PoseStamped

from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..stats import estimate
from ..thresholds import Tolerance
from ..expectations import Expectations
from .base import Outcome, ProbeResult, decide
from .common import ensure_enabled, step_and_measure, stream_goal

MIN_VALID_TRIALS = 3


class ScalingUnitsProbe:
    name = "ScalingUnitsProbe"

    def __init__(self, adapter: PlatformAdapter, tol: Tolerance, trials: int = 10, step_if: float = 0.005, settle_s: float = 1.0,
                 expectations: Optional[Expectations] = None, still_tol_m: float = 1e-5):
        """step_if: commanded displacement in *interface units* (the probe does not know the unit)."""
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.step = step_if
        self.settle = settle_s
        self.exp = expectations or Expectations()
        self.still_tol = still_tol_m  # implementation constant: motion below this is 'no response'

    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "dimensional", Outcome.UNDETERMINED)
        res.observations["expectation"] = self.exp.to_dict()["dimensional"]
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
        # resting noise of the feedback channel: scales the step and the still tolerance (0.1.1)
        from ..rate_estimator import estimate_noise_sigma
        buf.clear()
        t0r = time.monotonic()
        time.sleep(1.0)
        rest = buf.since(t0r)
        P = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in rest]) if rest else np.zeros((0, 3))
        sigma_hat = estimate_noise_sigma(P) if len(P) >= 3 else 0.0
        step_used = max(self.step, 20.0 * sigma_hat)  # implementation rule: the step is at least 20 x the resting noise
        still_tol = max(self.still_tol, 3.0 * sigma_hat)
        res.observations["resting_noise_sigma_if"] = sigma_hat
        res.observations["step_used_if"] = step_used
        res.observations["still_tol_used_if"] = still_tol
        if step_used > self.step:
            res.notes.append(f"step enlarged from {self.step} to {step_used:.4g} interface units (20 x resting noise sigma_hat = {sigma_hat:.3g})")
        axes = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
        r_int: List[float] = []
        s_anc: List[float] = []
        per_axis = {0: [], 1: [], 2: []}
        latencies: List[float] = []
        trial_log = []
        no_response = 0
        hold = self.a.latest_pose(buf, 1.0)
        for k in range(self.trials):
            ax = k % 3
            delta = step_used * axes[ax] * (1 if (k // 3) % 2 == 0 else -1)
            pa0 = None
            if anchor_buf is not None:
                t0a = time.monotonic(); stream_goal(self.a, hold, t0a + 0.3, self.tol.client_rate_hz)
                A0 = [pose_msg_to_matrix(m)[:3, 3] for _, m in anchor_buf.since(t0a)]
                if A0:
                    pa0 = np.eye(4); pa0[:3, 3] = np.mean(A0, axis=0)
            r = step_and_measure(self.a, buf, delta, self.settle, still_tol=still_tol, stream_hz=self.tol.client_rate_hz, hold=hold)
            if not r["ok"]:
                trial_log.append({"trial": k, "ok": False, "reason": r["reason"]})
                continue
            if math.isnan(r["first_motion_s"]) and np.linalg.norm(r["delta_meas"]) <= still_tol:
                no_response += 1
                trial_log.append({"trial": k, "ok": False, "reason": "no_response: no motion above still tolerance within settle time"})
                continue
            ratio = float(np.linalg.norm(r["delta_meas"]) / np.linalg.norm(delta))
            r_int.append(ratio)
            per_axis[ax].append(ratio)
            if not math.isnan(r["first_motion_s"]):
                latencies.append(r["first_motion_s"])
            entry = {"trial": k, "ok": True, "axis": ax, "delta_cmd_if": delta.tolist(), "delta_meas_if": r["delta_meas"].tolist(), "r_int": ratio, "first_motion_s": r["first_motion_s"],
                     "n_pre": r["n_pre"], "n_post": r["n_post"], "post_std_if": r["post_std_m"]}
            if anchor_buf is not None and pa0 is not None:
                A1 = [pose_msg_to_matrix(m)[:3, 3] for _, m in anchor_buf.since(time.monotonic() - 0.3)]
                pa1 = None
                if A1:
                    pa1 = np.eye(4); pa1[:3, 3] = np.mean(A1, axis=0)
                if pa1 is not None:
                    d_anc = pa1[:3, 3] - pa0[:3, 3]
                    s = float(np.linalg.norm(d_anc) / np.linalg.norm(delta))  # delta interpreted as metres
                    s_anc.append(s)
                    entry["delta_anchor_m"] = d_anc.tolist()
                    entry["s_anchored"] = s
            trial_log.append(entry)
            # return to start (streamed, as a servo client would); the start pose is then held during the next pre-window
            back = r["p0"]
            stream_goal(self.a, back, time.monotonic() + self.settle * 0.5, self.tol.client_rate_hz)
            hold = back
        res.observations["trials"] = trial_log
        res.observations["no_response_trials"] = no_response
        res.observations["valid_trials"] = len(r_int)
        e_int = estimate(r_int)
        res.estimates["internal_ratio"] = e_int.to_dict()
        res.estimates["internal_ratio_per_axis"] = {str(k): estimate(v).to_dict() for k, v in per_axis.items()}
        if latencies:
            res.estimates["response_latency_s"] = estimate(latencies).to_dict()
        if len(r_int) < MIN_VALID_TRIALS:
            res.decision_basis = f"only {len(r_int)} valid trial(s) ({no_response} no-response): undetermined"
            res.notes.append("fewer than %d trials produced a response; the implementation may drop or reject commands" % MIN_VALID_TRIALS)
        elif s_anc:
            e_s = estimate(s_anc)
            res.estimates["scale_anchored"] = e_s.to_dict()
            if not self.exp.dimensional.declared:
                res.decision_basis = "no dimensional expectation declared (discover-only): anchored s_hat reported, no conformance verdict"
                res.notes.append(f"anchored unit estimate s_hat = {e_s.mean:.4f} metres per interface unit (n={e_s.n})")
            else:
                u = self.exp.dimensional.expected_unit_m
                # scale divergence relative to the unit the client assumes: s = s_hat / u; predicted error |1 - s| r_ws
                # from the *mean* estimate with its CI mapped through the model (per-trial |1 - s_i| would bias upward).
                cands = [self.tol.dimensional_error(v / u) for v in (e_s.ci_low, e_s.ci_high)]
                lo = 0.0 if (e_s.ci_low <= u <= e_s.ci_high) else min(cands)
                from ..stats import Estimate
                e_pred = Estimate(e_s.n, self.tol.dimensional_error(e_s.mean / u), float("nan"), lo, max(cands), e_s.alpha)
                res.estimates["expected_unit_m"] = u
                res.estimates["predicted_error_at_workspace_edge_m"] = e_pred.to_dict()
                res.predicted_error_m = e_pred.mean
                res.predicted_error_ci = [e_pred.ci_low, e_pred.ci_high]
                res.outcome = decide(e_pred.ci_low, e_pred.ci_high, self.tol.epsilon_m)
                res.decision_basis = (
                    f"eq. (6) with anchored s_hat = {e_s.mean:.4f} (95% CI {e_s.ci_low:.4f}..{e_s.ci_high:.4f}) against the client's unit {u}: "
                    f"|1-s| r_ws = {e_pred.mean*1e3:.3f} mm vs epsilon = {self.tol.epsilon_m*1e3:.3f} mm"
                )
        else:
            res.decision_basis = "no unit anchor: uniform scale is invariant under every interface ratio (Section 5.2) -> undetermined by construction"
            res.notes.append(f"internal consistency ratio {e_int.mean:.4f} (n={e_int.n}) says nothing about the unit; supply --anchor-topic to test units")
            if e_int.n and abs(e_int.mean - 1.0) > 3 * max(e_int.std, 1e-6):
                res.notes.append("command and measurement scaling are inconsistent with each other (r_int != 1)")
        res.duration_s = time.time() - t0
        return res
