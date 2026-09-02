"""RateSensitivityProbe — temporal binding class (state precondition, command liveness, rate).

Sub-probes, each reported separately:

  A. state precondition   Is servo_cp executed without an operating-state transition? Is there an
                          operating_state topic at all? (CRTK does not mandate a state machine.)
  B. liveness policy      After streaming commands, pause for a gap g, then send one command. If it
                          is not executed, a liveness policy tripped. Bisection over g estimates
                          tau_w. If commands are executed after the largest gap tested, the report
                          says "no liveness policy detected up to g_max" — a scoped statement, not a
                          claim that none exists. Drift during the gap is reported separately
                          ("released" semantics).
  C. effective rate       Command distinct setpoints at f and count distinct executed positions per
                          second through measured_cp. The observation is bounded by the measured_cp
                          publish rate, which is also measured and reported.

Decision (temporal): the client's stated rate and jitter bound must satisfy eq. (4) against the
estimated tau_w (if any), and the effective command rate must reach the rate eq. (6) requires for
the stated tolerance and speed. The state-precondition finding is compared with what the user
declares the client expects (--expect-state-machine yes|no|any).
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..stats import estimate, rate_estimate
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult
from .common import ensure_enabled, wait_settled


class RateSensitivityProbe:
    name = "RateSensitivityProbe"

    def __init__(
        self,
        adapter: PlatformAdapter,
        tol: Tolerance,
        trials: int = 5,
        gap_max_s: float = 2.0,
        gap_min_s: float = 0.02,
        bisection_steps: int = 6,
        rates_hz=(50, 100, 200, 500, 1000),
        expect_state_machine: str = "any",
        step_if: float = 0.002,
    ):
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.gap_max = gap_max_s
        self.gap_min = gap_min_s
        self.bisect = bisection_steps
        self.rates = rates_hz
        self.expect_sm = expect_state_machine
        self.step = step_if

    # ------------------------------------------------------------------ helpers
    def _executed(self, buf, goal_T, timeout=0.4, tol=None) -> bool:
        tol = tol or self.step * 0.25
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.a.wait_for(buf, 0.1)
            if msg is None:
                continue
            T = pose_msg_to_matrix(msg)
            if np.linalg.norm(T[:3, 3] - goal_T[:3, 3]) < tol:
                return True
        return False

    def _stream(self, buf, base_T, duration_s=0.3, rate_hz=100.0) -> float:
        """Stream small oscillating commands to keep any liveness policy fed; return the time of the last command."""
        period = 1.0 / rate_hz
        t_end = time.monotonic() + duration_s
        k = 0
        while time.monotonic() < t_end:
            T = base_T.copy()
            T[0, 3] += 0.25 * self.step * math.sin(2 * math.pi * k / 20.0)
            self.a.servo_cp(T)
            k += 1
            time.sleep(period)
        self.a.servo_cp(base_T)
        return time.monotonic()

    def _recover(self):
        if self.a.has("operating_state"):
            ensure_enabled(self.a, timeout_s=2.0)

    # ------------------------------------------------------------------ A
    def probe_state_precondition(self, buf) -> dict:
        out = {"operating_state_present": self.a.has("operating_state"), "state_command_present": self.a.has("state_command")}
        pose = self.a.latest_pose(buf, 1.0)
        if pose is None:
            msg = self.a.wait_for(buf, 1.0)
            if msg is None:
                out["error"] = "no measured_cp"
                return out
            pose = pose_msg_to_matrix(msg)
        if out["operating_state_present"]:
            self.a.state_command("disable")
            time.sleep(0.2)
            st = self.a.operating_state(0.5)
            out["state_after_disable"] = None if st is None else st.state
            goal = pose.copy()
            goal[0, 3] += self.step
            self.a.servo_cp(goal)
            out["executed_when_disabled"] = self._executed(buf, goal)
            self.a.servo_cp(pose)
            time.sleep(0.1)
            info = ensure_enabled(self.a)
            out["enable"] = info
            goal = pose.copy()
            goal[1, 3] += self.step
            self.a.servo_cp(goal)
            out["executed_when_enabled"] = self._executed(buf, goal)
            self.a.servo_cp(pose)
            time.sleep(0.1)
        else:
            goal = pose.copy()
            goal[0, 3] += self.step
            self.a.servo_cp(goal)
            out["executed_without_state_machine"] = self._executed(buf, goal)
            self.a.servo_cp(pose)
            time.sleep(0.1)
        return out

    # ------------------------------------------------------------------ B
    def _gap_trial(self, buf, base_T, gap_s: float) -> dict:
        """Stream, then stay silent for gap_s measured from the last command, then send one command."""
        self._recover()
        t_last = self._stream(buf, base_T)
        time.sleep(0.05)
        p_before = self.a.latest_pose(buf, 1.0)
        remaining = t_last + gap_s - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        p_after = self.a.latest_pose(buf, 1.0)
        drift = float(np.linalg.norm(p_after[:3, 3] - p_before[:3, 3])) if (p_before is not None and p_after is not None) else float("nan")
        goal = base_T.copy()
        goal[2, 3] += self.step
        t_send = time.monotonic()
        self.a.servo_cp(goal)
        actual_gap = t_send - t_last
        ok = self._executed(buf, goal)
        st = self.a.operating_state(0.2) if self.a.has("operating_state") else None
        self.a.servo_cp(base_T)
        time.sleep(0.05)
        return {"gap_s": actual_gap, "gap_requested_s": gap_s, "executed": ok, "drift_m": drift, "state": None if st is None else st.state}

    def probe_liveness(self, buf, base_T) -> dict:
        out = {"gap_max_s": self.gap_max, "trials": []}
        # 1) largest gap, n times
        big = [self._gap_trial(buf, base_T, self.gap_max) for _ in range(self.trials)]
        out["trials"] += big
        n_exec = sum(1 for b in big if b["executed"])
        out["executed_after_gap_max"] = rate_estimate(n_exec, len(big))
        drifts = [b["drift_m"] for b in big if not math.isnan(b["drift_m"])]
        out["drift_during_gap_max_m"] = estimate(drifts).to_dict() if drifts else None
        if n_exec == len(big):
            out["tau_w_estimate_s"] = None
            out["finding"] = f"no liveness policy detected up to {self.gap_max} s (n={len(big)})"
            if drifts and estimate(drifts).mean > 4 * self.step * 0.25:
                out["finding"] += "; pose drifted during the gap (released semantics)"
            return out
        # 2) bisection per trial
        taus: List[float] = []
        for _ in range(self.trials):
            lo, hi = self.gap_min, self.gap_max
            # ensure lo passes
            r = self._gap_trial(buf, base_T, lo)
            out["trials"].append(r)
            if not r["executed"]:
                taus.append(r["gap_s"])
                continue
            lo_actual, hi_actual = r["gap_s"], self.gap_max
            for _ in range(self.bisect):
                mid = 0.5 * (lo + hi)
                r = self._gap_trial(buf, base_T, mid)
                out["trials"].append(r)
                if r["executed"]:
                    lo, lo_actual = mid, r["gap_s"]
                else:
                    hi, hi_actual = mid, r["gap_s"]
            taus.append(0.5 * (lo_actual + hi_actual))
        e = estimate(taus)
        out["tau_w_estimate_s"] = e.to_dict()
        out["finding"] = f"liveness policy detected: tau_w ~ {e.mean:.3f} s (95% CI {e.ci_low:.3f}..{e.ci_high:.3f}, n={e.n})"
        return out

    # ------------------------------------------------------------------ C
    def probe_effective_rate(self, buf, base_T) -> dict:
        out = {"publish_rate_hz": None, "per_rate": []}
        # measured_cp publish rate
        c0 = buf.count
        t0 = time.monotonic()
        time.sleep(1.0)
        out["publish_rate_hz"] = (buf.count - c0) / (time.monotonic() - t0)
        for f in self.rates:
            self._recover()
            period = 1.0 / f
            n = int(f * 1.0)
            buf.clear()
            t_start = time.monotonic()
            seen = set()
            for k in range(n):
                T = base_T.copy()
                T[0, 3] += self.step * (k + 1) / n
                self.a.servo_cp(T)
                dt = t_start + (k + 1) * period - time.monotonic()
                if dt > 0:
                    time.sleep(dt)
            t_end = time.monotonic()
            time.sleep(0.2)
            for _, m in buf.since(t_start):
                seen.add(round(m.pose.position.x, 9))
            distinct = max(0, len(seen) - 1)
            eff = distinct / (t_end - t_start)
            out["per_rate"].append({
                "command_rate_hz": f,
                "commands_sent": n,
                "distinct_positions_observed": distinct,
                "effective_rate_hz": eff,
                "observation_bounded_by_publish_rate": eff >= 0.9 * out["publish_rate_hz"],
                "zoh_error_bound_m": self.tol.zoh_error(min(f, eff if eff > 0 else f)),
            })
            self.a.servo_cp(base_T)
            time.sleep(0.1)
        return out

    # ------------------------------------------------------------------ run
    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "temporal", Outcome.UNDETERMINED)
        disc = self.a.discovery or self.a.discover()
        if not (disc["topics"]["measured_cp"]["present"] and disc["topics"]["servo_cp"]["present"]):
            res.notes.append("measured_cp or servo_cp missing")
            res.decision_basis = "missing topics"
            res.duration_s = time.time() - t0
            return res
        buf = self.a.subscribe("measured_cp")
        if self.a.wait_for(buf, 2.0) is None:
            res.notes.append("no measured_cp data")
            res.decision_basis = "no data"
            res.duration_s = time.time() - t0
            return res
        A = self.probe_state_precondition(buf)
        ensure_enabled(self.a)
        base_T, _ = wait_settled(self.a, buf, 1.0)
        if base_T is None:
            base_T = self.a.latest_pose(buf, 2.0)
        B = self.probe_liveness(buf, base_T)
        C = self.probe_effective_rate(buf, base_T)
        res.observations = {"state_precondition": A, "liveness": B, "effective_rate": C}

        # decision
        notes = []
        divergent = False
        undetermined = False
        f_req = self.tol.required_rate_hz()
        res.estimates["required_rate_hz_from_tolerance"] = f_req
        tau = B.get("tau_w_estimate_s")
        if tau is not None:
            ok = self.tol.liveness_ok(tau["ci_low"])
            res.estimates["liveness_margin_ok"] = ok
            if not ok:
                divergent = True
                notes.append(f"eq. (4) violated: client period 1/{self.tol.client_rate_hz} + J_max {self.tol.jitter_max_s} s >= tau_w {tau['mean']:.3f} s")
        else:
            res.estimates["liveness_margin_ok"] = None
        # effective rate at the client's rate
        eff_at_client = None
        for r in C["per_rate"]:
            if abs(r["command_rate_hz"] - self.tol.client_rate_hz) < 1e-6:
                eff_at_client = r
        if eff_at_client is None and C["per_rate"]:
            eff_at_client = min(C["per_rate"], key=lambda r: abs(r["command_rate_hz"] - self.tol.client_rate_hz))
        if eff_at_client is not None:
            eff = eff_at_client["effective_rate_hz"]
            res.estimates["effective_rate_at_client_rate_hz"] = eff
            res.estimates["zoh_error_at_client_rate_m"] = self.tol.zoh_error(eff) if eff > 0 else None
            if eff_at_client["observation_bounded_by_publish_rate"] and eff < f_req:
                undetermined = True
                notes.append("effective rate observation is bounded by the measured_cp publish rate; rate conformance cannot be decided")
            elif eff < f_req and eff > 0:
                divergent = True
                notes.append(f"effective command rate {eff:.1f} Hz < required {f_req:.1f} Hz (eq. 6)")
        # state precondition vs expectation
        sm = A.get("operating_state_present")
        if self.expect_sm == "yes" and not sm:
            divergent = True
            notes.append("client expects an operating-state machine; none exposed")
        if self.expect_sm == "no" and sm and A.get("executed_when_disabled") is False:
            divergent = True
            notes.append("client expects commands to be executed without enabling; implementation rejects them when disabled")
        if divergent:
            res.outcome = Outcome.DIVERGENT
        elif undetermined:
            res.outcome = Outcome.UNDETERMINED
        else:
            res.outcome = Outcome.CONFORMANT
        res.decision_basis = "; ".join(notes) if notes else "eq. (4) satisfied or no liveness policy detected; effective rate >= v/epsilon; state precondition consistent with expectation"
        res.notes += notes
        res.duration_s = time.time() - t0
        return res
