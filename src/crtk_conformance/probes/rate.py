"""RateSensitivityProbe — temporal binding class (state precondition, command liveness / stop behaviour, rate).

Version 0.1.1 (designs: rc3/LIVENESS_PROBE_DESIGN.md, rc3/RATE_ESTIMATOR_DESIGN.md).

Sub-probes, each reported separately:

  A. state precondition   Is there an operating_state topic? Is servo_cp executed when DISABLED and
                          after enable/home (or without any state machine)?  Enable latency.
  B. liveness / stop      After streaming commands, stay silent for a gap g, then send one command.  The
     behaviour            probe first measures its own timing resolution (sleep, send, feedback period,
                          response latency) and reports a resolution floor r; gaps below r are
                          `below_resolution`.  During the gap the pose is recorded and the stop behaviour
                          is classified as hold / release / rejected|fault / not_observable (a command counts as
                          rejected only when no motion toward it is observed; a tracking error is not a
                          rejection: attained and responded are reported separately); the trip gap
                          tau_w is estimated by bisection between realised gaps (monotonic clock) and is
                          reported only with n >= 3 trials and a CI lower bound above the floor.
  C. observable rate      Command distinct setpoints at f and classify feedback samples to the nearest
                          commanded target within a matching tolerance (user-supplied or estimated from
                          the resting noise); count monotone target transitions.  Reports the client's
                          achieved rate, the feedback publish rate, matched/unmatched counts and whether
                          the observation is channel-bounded.  Optional secondary channel on setpoint_cp.

Decision (temporal): only against the client's *declared* expectations (expectations.py):
  state_machine required|forbidden, stop_behaviour hold|release|fault, rate required.  With no declared
  temporal expectation the probe reports its observations and returns UNDETERMINED.

Implementation constants (all exposed as constructor / CLI parameters; none is a task threshold):
  stream duration 0.3 s at 100 Hz, executed-tolerance 0.25 * step, resolution-sample counts (20 sleeps,
  10 latency probes), hold tolerance max(4 sigma_hat, 0.5 step), rate window 1 s per nominal rate.
"""
from __future__ import annotations

import math
import statistics
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..expectations import Expectations, TemporalExpectation, combine
from ..rate_estimator import (
    RateEstimate,
    estimate_noise_sigma,
    estimate_rate,
    match_tolerance_from_sigma,
    rate_subverdict,
)
from ..stats import estimate, rate_estimate
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult
from .common import ensure_enabled, wait_settled

STOP_CLASSES = ("hold", "release", "rejected", "fault", "not_observable", "no_policy_within_range")


class RateSensitivityProbe:
    name = "RateSensitivityProbe"

    def __init__(
        self,
        adapter: PlatformAdapter,
        tol: Tolerance,
        trials: int = 5,
        gap_max_s: float = 2.0,
        gap_min_s: float = 0.0,
        bisection_steps: int = 7,
        rates_hz=(50, 100, 200, 500, 1000),
        expectations: Optional[Expectations] = None,
        step_if: float = 0.002,
        response_timeout_s: Optional[float] = None,
        rate_match_tolerance_m: Optional[float] = None,
        rate_window_s: float = 1.0,
        rate_max_step_if: float = 0.05,
        stream_rate_hz: float = 100.0,
        stream_duration_s: float = 0.3,
        expect_state_machine: Optional[str] = None,  # legacy alias: 'yes' | 'no' | 'any'
    ):
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.gap_max = gap_max_s
        self.gap_min = gap_min_s
        self.bisect = bisection_steps
        self.rates = rates_hz
        self.exp = expectations or Expectations()
        if expect_state_machine is not None:
            self.exp.temporal.state_machine = {"yes": "required", "no": "forbidden", "any": "any"}[expect_state_machine]
        self.step = step_if
        self.user_response_timeout = response_timeout_s
        self.user_delta = rate_match_tolerance_m
        self.rate_window = rate_window_s
        self.rate_max_step = rate_max_step_if  # never command a rate sweep farther than this from the start pose (interface units)
        self.stream_rate = stream_rate_hz
        self.stream_duration = stream_duration_s
        # measured during run()
        self.resolution: Dict[str, float] = {}
        self.response_timeout = 0.4
        self.sigma_hat = 0.0
        self.hold_tol = 0.5 * step_if
        self.probe_axis, self.probe_sign = 2, 1  # replaced by the best-tracked direction in measure_resolution()

    # ------------------------------------------------------------------ helpers
    def _executed(self, buf, goal_T, timeout: Optional[float] = None, tol=None) -> Tuple[bool, float]:
        """Wait until measured_cp is within tol of goal_T; return (attained, time_to_attain)."""
        r = self._response(buf, goal_T, timeout=timeout, tol=tol)
        return r["attained"], r["time_to_attain_s"]

    def _response(self, buf, goal_T, timeout: Optional[float] = None, tol=None, p_ref=None) -> dict:
        """Observe the response to a command.

        attained: some sample came within tol of the goal.  responded: attained, or the distance to the goal, taken
        at the mean of the last 0.1 s (or last 5 samples) of the observation window, decreased by more than
        max(3 sigma_hat, 0.2 x initial distance) — i.e. the implementation acted on the command even if it did not
        reach it (tracking error, gravity sag, clipping).  A tracking error is not a rejection.
        """
        tol = tol or self.step * 0.25
        timeout = self.response_timeout if timeout is None else timeout
        t0 = time.monotonic()
        deadline = t0 + timeout
        g = goal_T[:3, 3]
        if p_ref is None:
            cur = self.a.latest_pose(buf, 1.0)
            p_ref = cur[:3, 3] if cur is not None else None
        d0 = None if p_ref is None else float(np.linalg.norm(p_ref - g))
        closest = float("inf")
        moved = 0.0
        attained, t_att, t_resp = False, float("nan"), float("nan")
        thr = None if d0 is None else max(3.0 * self.sigma_hat, 0.2 * d0)
        hist: List[Tuple[float, np.ndarray]] = []
        while time.monotonic() < deadline:
            msg = self.a.wait_for(buf, min(0.05, max(0.0, deadline - time.monotonic())))
            if msg is None:
                continue
            now = time.monotonic()
            p = pose_msg_to_matrix(msg)[:3, 3]
            hist.append((now, p))
            d = float(np.linalg.norm(p - g))
            closest = min(closest, d)
            if p_ref is not None:
                moved = max(moved, float(np.linalg.norm(p - p_ref)))
                if math.isnan(t_resp) and thr is not None:
                    recent = [q for (tq, q) in hist if tq >= now - 0.1][-5:] or [p]
                    if d0 - float(np.linalg.norm(np.mean(recent, axis=0) - g)) > thr:
                        t_resp = now - t0
            if d < tol:
                attained, t_att = True, now - t0
                if math.isnan(t_resp):
                    t_resp = t_att
                break
        settled_d = float("nan")
        if hist:
            last = [q for (tq, q) in hist if tq >= hist[-1][0] - 0.1][-5:] or [hist[-1][1]]
            settled_d = float(np.linalg.norm(np.mean(last, axis=0) - g))
        reduction = float("nan") if d0 is None or math.isnan(settled_d) else d0 - settled_d
        responded = attained or (thr is not None and not math.isnan(reduction) and reduction > thr)
        return {"attained": attained, "responded": bool(responded), "closest_m": closest, "moved_m": moved, "settled_distance_m": settled_d,
                "initial_distance_m": d0, "reduction_m": reduction, "time_to_attain_s": t_att, "time_to_respond_s": t_resp}

    def _stream(self, buf, base_T) -> Tuple[float, np.ndarray]:
        """Stream small oscillating commands to keep any liveness policy fed; return (t_last, last setpoint)."""
        period = 1.0 / self.stream_rate
        t_end = time.monotonic() + self.stream_duration
        k = 0
        while time.monotonic() < t_end:
            T = base_T.copy()
            T[0, 3] += 0.25 * self.step * math.sin(2 * math.pi * k / 20.0)
            self.a.servo_cp(T)
            k += 1
            time.sleep(period)
        self.a.servo_cp(base_T)
        return time.monotonic(), base_T[:3, 3].copy()

    def _recover(self, buf, base_T):
        if self.a.has("operating_state"):
            ensure_enabled(self.a, timeout_s=2.0)
        self.a.servo_cp(base_T)
        self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))

    # ------------------------------------------------------------------ resolution
    def measure_resolution(self, buf, base_T) -> Dict[str, float]:
        out: Dict[str, float] = {}
        # sleep resolution
        errs = []
        for req in (0.001, 0.002, 0.005, 0.010):
            for _ in range(5):
                t0 = time.monotonic()
                time.sleep(req)
                errs.append(time.monotonic() - t0 - req)
        errs.sort()
        out["sleep_error_median_s"] = float(statistics.median(errs))
        out["sleep_error_p95_s"] = float(errs[int(0.95 * (len(errs) - 1))])
        # send resolution: interval between consecutive publications
        ts = []
        for _ in range(20):
            ts.append(time.monotonic())
            self.a.servo_cp(base_T)
        d = sorted(np.diff(ts).tolist())
        out["send_interval_median_s"] = float(statistics.median(d))
        out["send_interval_p95_s"] = float(d[int(0.95 * (len(d) - 1))])
        # feedback period and resting noise
        buf.clear()
        t0 = time.monotonic()
        time.sleep(1.0)
        samples = buf.since(t0)
        out["feedback_period_s"] = (time.monotonic() - t0) / max(1, len(samples))
        P = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in samples]) if samples else np.zeros((0, 3))
        self.sigma_hat = estimate_noise_sigma(P) if len(P) >= 3 else 0.0
        out["resting_noise_sigma_m"] = self.sigma_hat
        # the probe step is at least 20 x the resting noise, so that attainment (0.25 step) is 5 sigma above the jitter
        out["step_requested_m"] = self.step
        self.step = max(self.step, 20.0 * self.sigma_hat)
        out["step_used_m"] = self.step
        self.hold_tol = max(4.0 * self.sigma_hat, 0.5 * self.step)
        # response latency: small command -> first reflecting sample; also pick the probe direction (axis and sign)
        # that the implementation tracks best, so that gap and state trials are not defeated by a joint limit or a
        # poorly tracked axis (found on the live SRC v1.0.0 instance, whose insertion joint sat at its limit)
        lat = []
        att = 0
        ratios: Dict[str, List[float]] = {}
        dirs = [(ax, sg) for ax in (0, 1, 2) for sg in (1, -1)]
        for k in range(12):
            ax, sg = dirs[k % len(dirs)]
            cur = self.a.latest_pose(buf, 1.0)
            goal = (cur if cur is not None else base_T).copy()
            goal[ax, 3] += self.step * sg
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(1.0, 5 * self.response_timeout))
            if r["responded"] and not math.isnan(r["time_to_respond_s"]):
                lat.append(r["time_to_respond_s"])
            att += r["attained"]
            if r["initial_distance_m"]:
                ratios.setdefault(f"{ax}{'+' if sg > 0 else '-'}", []).append(float(r["reduction_m"] / r["initial_distance_m"]))
            self.a.servo_cp(base_T)
            self._response(buf, base_T, timeout=max(1.0, 5 * self.response_timeout))
        out["latency_probes_attained"] = att
        out["latency_probes_responded"] = len(lat)
        out["tracking_ratio_by_direction"] = {k: float(np.mean(v)) for k, v in ratios.items()}
        best = max(out["tracking_ratio_by_direction"].items(), key=lambda kv: kv[1])[0] if ratios else "2+"
        self.probe_axis, self.probe_sign = int(best[0]), (1 if best[1] == "+" else -1)
        out["probe_direction"] = best
        if lat:
            lat.sort()
            out["response_latency_median_s"] = float(statistics.median(lat))
            out["response_latency_p95_s"] = float(lat[int(0.95 * (len(lat) - 1))])
            # settling takes longer than the first response; the timeout must cover it
            out["settle_hint_s"] = float(max(lat)) * 3.0
        else:
            out["response_latency_median_s"] = float("nan")
            out["response_latency_p95_s"] = float("nan")
        out["resolution_floor_s"] = 2.0 * (out["sleep_error_p95_s"] + out["send_interval_p95_s"]) + out["feedback_period_s"]
        # response timeout: user value or 5 x p95 latency, at least 0.2 s
        lat_p95 = out["response_latency_p95_s"]
        auto = 5.0 * lat_p95 if not math.isnan(lat_p95) else 0.4
        self.response_timeout = max(self.user_response_timeout or 0.0, auto, 0.3)
        out["response_timeout_s"] = self.response_timeout
        out["hold_tolerance_m"] = self.hold_tol
        self.resolution = out
        return out

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
            goal[self.probe_axis, 3] += self.step * self.probe_sign
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(0.5, self.response_timeout), p_ref=pose[:3, 3])
            out["executed_when_disabled"] = r["responded"]
            out["attained_when_disabled"] = r["attained"]
            self.a.servo_cp(pose)
            time.sleep(0.1)
            info = ensure_enabled(self.a)
            out["enable"] = info
            goal = pose.copy()
            goal[self.probe_axis, 3] += self.step * self.probe_sign
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(0.5, self.response_timeout), p_ref=pose[:3, 3])
            out["executed_when_enabled"] = r["responded"]
            out["attained_when_enabled"] = r["attained"]
            self.a.servo_cp(pose)
            time.sleep(0.1)
        else:
            goal = pose.copy()
            goal[self.probe_axis, 3] += self.step * self.probe_sign
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(0.5, self.response_timeout), p_ref=pose[:3, 3])
            out["executed_without_state_machine"] = r["responded"]
            out["attained_without_state_machine"] = r["attained"]
            out["closest_approach_m"] = r["closest_m"]
            self.a.servo_cp(pose)
            time.sleep(0.1)
        return out

    # ------------------------------------------------------------------ B
    def _gap_trial(self, buf, base_T, gap_s: float) -> dict:
        """Stream, stay silent for gap_s from the last command (no fixed sleep), send one command, classify."""
        self._recover(buf, base_T)
        t_last, p_last = self._stream(buf, base_T)
        buf.clear()
        deadline = t_last + gap_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.001))
        goal = base_T.copy()
        goal[self.probe_axis, 3] += self.step * self.probe_sign
        t_send = time.monotonic()
        gap_samples = buf.since(t_last)
        p_pre = pose_msg_to_matrix(gap_samples[-1][1])[:3, 3] if gap_samples else p_last
        self.a.servo_cp(goal)
        actual_gap = t_send - t_last
        resp = self._response(buf, goal, p_ref=p_pre)
        ok, t_exec = resp["attained"], resp["time_to_attain_s"]
        st = self.a.operating_state(0.2) if self.a.has("operating_state") else None
        state = None if st is None else st.state
        drifts = [float(np.linalg.norm(pose_msg_to_matrix(m)[:3, 3] - p_last)) for _, m in gap_samples]
        drift = max(drifts) if drifts else float("nan")
        n_gap = len(gap_samples)
        fp = self.resolution.get("feedback_period_s", 0.01)
        if not resp["responded"]:
            cls = "fault" if state in ("FAULT", "DISABLED") else "rejected"
        elif n_gap < 2 and gap_s >= 2 * fp:
            cls = "not_observable"
        elif drifts and drift > self.hold_tol:
            cls = "release"
        else:
            cls = "hold"
        self.a.servo_cp(base_T)
        self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))
        return {"gap_requested_s": gap_s, "gap_s": actual_gap, "below_resolution": gap_s < self.resolution.get("resolution_floor_s", 0.0),
                "executed": ok, "responded": resp["responded"], "closest_approach_m": resp["closest_m"], "moved_m": resp["moved_m"],
                "initial_distance_m": resp["initial_distance_m"], "settled_distance_m": resp["settled_distance_m"], "reduction_m": resp["reduction_m"],
                "time_to_execute_s": t_exec, "drift_m": drift, "samples_in_gap": n_gap, "class": cls, "state": state}

    @staticmethod
    def _tripped(r: dict) -> bool:
        return r["class"] in ("rejected", "fault", "release")

    def probe_liveness(self, buf, base_T) -> dict:
        r_floor = self.resolution.get("resolution_floor_s", 0.0)
        out = {"gap_max_s": self.gap_max, "resolution_floor_s": r_floor, "response_timeout_s": self.response_timeout,
               "hold_tolerance_m": self.hold_tol, "trials": []}
        big = [self._gap_trial(buf, base_T, self.gap_max) for _ in range(self.trials)]
        out["trials"] += big
        n_trip = sum(1 for b in big if self._tripped(b))
        out["tripped_at_gap_max"] = rate_estimate(n_trip, len(big))
        drifts = [b["drift_m"] for b in big if not math.isnan(b["drift_m"])]
        out["drift_during_gap_max_m"] = estimate(drifts).to_dict() if drifts else None
        classes = [b["class"] for b in big]
        if n_trip == 0:
            out["stop_class"] = "not_observable" if all(c == "not_observable" for c in classes) else "no_policy_within_range"
            out["tau_w_estimate_s"] = None
            out["finding"] = (f"no stop policy detected up to {self.gap_max} s (n={len(big)}): post-gap commands executed, pose held"
                              if out["stop_class"] == "no_policy_within_range" else f"stop behaviour not observable (measured_cp too sparse during the gap)")
            return out
        # majority class among tripped trials
        trip_classes = [b["class"] for b in big if self._tripped(b)]
        out["stop_class"] = max(set(trip_classes), key=trip_classes.count)
        # bisection per trial between the resolution floor and gap_max
        lo0 = max(self.gap_min, r_floor)
        taus: List[float] = []
        widths: List[float] = []
        below = 0
        for _ in range(self.trials):
            r = self._gap_trial(buf, base_T, lo0)
            out["trials"].append(r)
            if self._tripped(r):
                below += 1  # trips even at the smallest resolvable gap
                continue
            lo, hi = lo0, self.gap_max
            lo_real, hi_real = r["gap_s"], big[0]["gap_s"]
            for _ in range(self.bisect):
                mid = 0.5 * (lo + hi)
                r = self._gap_trial(buf, base_T, mid)
                out["trials"].append(r)
                if self._tripped(r):
                    hi, hi_real = mid, r["gap_s"]
                else:
                    lo, lo_real = mid, r["gap_s"]
            taus.append(0.5 * (lo_real + hi_real))
            widths.append(hi_real - lo_real)
        out["trials_below_resolution"] = below
        if below > 0 and not taus:
            out["tau_w_estimate_s"] = {"upper_bound_s": lo0, "n": below}
            out["finding"] = f"{out['stop_class']} policy trips at every gap down to the resolution floor {lo0*1e3:.1f} ms: tau_w <= {lo0*1e3:.1f} ms (below resolution, n={below})"
            return out
        e = estimate(taus)
        out["bisection_width_s"] = estimate(widths).to_dict() if widths else None
        if e.n < 3 or not (e.ci_low > r_floor):
            out["tau_w_estimate_s"] = dict(e.to_dict(), status="undetermined",
                                           reason=("n < 3" if e.n < 3 else "CI lower bound not above the resolution floor"))
            out["finding"] = f"{out['stop_class']} policy observed but tau_w undetermined ({out['tau_w_estimate_s']['reason']}; n={e.n})"
            return out
        out["tau_w_estimate_s"] = dict(e.to_dict(), status="ok")
        out["finding"] = f"{out['stop_class']} policy detected: tau_w ~ {e.mean:.3f} s (95% CI {e.ci_low:.3f}..{e.ci_high:.3f}, n={e.n})"
        return out

    # ------------------------------------------------------------------ C
    def probe_effective_rate(self, buf, base_T) -> dict:
        out: Dict[str, object] = {"per_rate": []}
        if self.user_delta is not None:
            delta, src = self.user_delta, "user"
        else:
            delta, src = match_tolerance_from_sigma(self.sigma_hat), f"5 x resting noise sigma_hat ({self.sigma_hat:.2e} m), floor 1e-6 m"
        out["match_tolerance_m"] = delta
        out["match_tolerance_source"] = src
        sp_buf = self.a.subscribe("setpoint_cp") if self.a.has("setpoint_cp") else None
        for f in self.rates:
            self._recover(buf, base_T)
            period = 1.0 / f
            n = int(f * self.rate_window)
            # target spacing must exceed 4 delta; enlarge the step, then reduce n if the step would be unreasonable
            step = self.step
            spacing = step / n
            note = ""
            if spacing < 4 * delta:
                step = 4 * delta * n
                if step > self.rate_max_step:  # do not command farther than rate_max_step; reduce the number of targets instead
                    n = max(2, int(self.rate_max_step / (4 * delta)))
                    step = 4 * delta * n
                    period = self.rate_window / n
                    note = f"targets reduced to {n} (spacing 4 delta) because the match tolerance is large"
                spacing = step / n
            targets = np.tile(base_T[:3, 3], (n, 1))
            targets[:, 0] += step * (np.arange(n) + 1) / n
            buf.clear()
            if sp_buf is not None:
                sp_buf.clear()
            send_times = []
            t_start = time.monotonic()
            for k in range(n):
                T = base_T.copy()
                T[0, 3] = targets[k, 0]
                send_times.append(time.monotonic())
                self.a.servo_cp(T)
                dt = t_start + (k + 1) * period - time.monotonic()
                if dt > 0:
                    time.sleep(dt)
            t_end_cmd = time.monotonic()
            # tail: wait until the last target is reached or the response timeout expires
            self._executed(buf, T, timeout=self.response_timeout, tol=delta)
            time.sleep(max(0.05, 2 * self.resolution.get("feedback_period_s", 0.01)))
            t_end = time.monotonic()
            samples = buf.since(t_start)
            P = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in samples]) if samples else np.zeros((0, 3))
            ts = np.array([t for t, _ in samples])
            est = estimate_rate(targets, P, ts, np.array(send_times), float(f), delta, src, t_start, t_end)
            row = est.to_dict()
            row["step_used_m"] = step
            row["command_window_s"] = t_end_cmd - t_start
            if note:
                row["note"] = note
            row["zoh_error_bound_m"] = self.tol.zoh_error(est.observable_rate_hz) if est.observable_rate_hz > 0 else None
            if sp_buf is not None:
                sps = sp_buf.since(t_start)
                if sps:
                    SP = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in sps])
                    est_sp = estimate_rate(targets, SP, np.array([t for t, _ in sps]), np.array(send_times), float(f), delta, src, t_start, t_end)
                    row["setpoint_cp_accepted_rate_hz"] = est_sp.observable_rate_hz
                    row["setpoint_cp_transitions"] = est_sp.transitions
            out["per_rate"].append(row)
            self.a.servo_cp(base_T)
            self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))
        return out

    # ------------------------------------------------------------------ run
    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "temporal", Outcome.UNDETERMINED)
        res.observations["expectation"] = self.exp.to_dict()["temporal"]
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
        ensure_enabled(self.a)
        base_T, _ = wait_settled(self.a, buf, 1.0)
        if base_T is None:
            base_T = self.a.latest_pose(buf, 2.0)
        R = self.measure_resolution(buf, base_T)
        A = self.probe_state_precondition(buf)
        ensure_enabled(self.a)
        base_T, _ = wait_settled(self.a, buf, 1.0)
        if base_T is None:
            base_T = self.a.latest_pose(buf, 2.0)
        B = self.probe_liveness(buf, base_T)
        C = self.probe_effective_rate(buf, base_T)
        res.observations.update({"state_precondition": A, "resolution": R, "liveness": B, "effective_rate": C})

        # ---- sub-verdicts against declared expectations
        te: TemporalExpectation = self.exp.temporal
        sub: Dict[str, Optional[str]] = {"state_machine": None, "stop_behaviour": None, "rate": None}
        notes = []
        f_req = self.tol.required_rate_hz()
        res.estimates["required_rate_hz_from_tolerance"] = f_req
        sm_present = bool(A.get("operating_state_present"))
        if te.state_machine == "required":
            if sm_present and A.get("executed_when_disabled") is False and A.get("executed_when_enabled") is True:
                sub["state_machine"] = "satisfied"
            elif not sm_present or A.get("executed_when_disabled") is True:
                sub["state_machine"] = "violated"
                notes.append("client requires an operating-state precondition; " + ("no operating_state topic exposed" if not sm_present else "commands executed while DISABLED"))
            else:
                sub["state_machine"] = "undetermined"
        elif te.state_machine == "forbidden":
            if not sm_present or A.get("executed_when_disabled") is True:
                sub["state_machine"] = "satisfied"
            elif A.get("executed_when_disabled") is False:
                sub["state_machine"] = "violated"
                notes.append("client expects commands to execute without enabling; implementation rejects them when DISABLED")
            else:
                sub["state_machine"] = "undetermined"
        stop_class = B.get("stop_class")
        tau = B.get("tau_w_estimate_s")
        res.estimates["stop_class"] = stop_class
        res.estimates["tau_w_estimate_s"] = tau
        if te.stop_behaviour != "any":
            if stop_class in (None, "not_observable"):
                sub["stop_behaviour"] = "undetermined"
            elif te.stop_behaviour == "hold":
                sub["stop_behaviour"] = "satisfied" if stop_class == "no_policy_within_range" else "violated"
                if sub["stop_behaviour"] == "violated":
                    notes.append(f"client expects the last setpoint to be held when commands stop; observed {stop_class}")
            else:  # release | fault expected
                want = ("release",) if te.stop_behaviour == "release" else ("fault", "rejected")
                if stop_class in want:
                    ok_margin = True
                    if isinstance(tau, dict) and tau.get("status") == "ok":
                        ok_margin = self.tol.liveness_ok(tau["ci_low"])
                        if not ok_margin:
                            notes.append(f"eq. (7) violated: client period 1/{self.tol.client_rate_hz} + J_max {self.tol.jitter_max_s} s >= tau_w CI low {tau['ci_low']:.3f} s")
                    elif isinstance(tau, dict) and "upper_bound_s" in tau:
                        ok_margin = None
                    sub["stop_behaviour"] = "satisfied" if ok_margin is True else ("violated" if ok_margin is False else "undetermined")
                elif stop_class == "no_policy_within_range":
                    sub["stop_behaviour"] = "violated"
                    notes.append(f"client expects a {te.stop_behaviour} stop policy; none detected up to {self.gap_max} s")
                else:
                    sub["stop_behaviour"] = "violated"
                    notes.append(f"client expects {te.stop_behaviour} stop behaviour; observed {stop_class}")
        # rate at the client's declared rate
        rows = C["per_rate"]
        at_client = None
        if rows:
            at_client = min(rows, key=lambda r: abs(r["command_rate_requested_hz"] - self.tol.client_rate_hz))
            res.estimates["observable_rate_at_client_rate_hz"] = at_client["observable_rate_hz"]
            res.estimates["client_rate_achieved_hz"] = at_client["client_rate_achieved_hz"]
            res.estimates["publish_rate_hz"] = at_client["publish_rate_hz"]
            res.estimates["zoh_error_at_client_rate_m"] = at_client.get("zoh_error_bound_m")
        if te.rate == "required":
            est = None
            if at_client is not None:
                keys = RateEstimate.__dataclass_fields__.keys()
                est = RateEstimate(**{k: at_client[k] for k in keys})
            sub["rate"] = rate_subverdict(est, f_req)
            if sub["rate"] == "violated":
                notes.append(f"observable command rate {at_client['observable_rate_hz']:.1f} Hz < required {f_req:.1f} Hz (eq. 9) and the channel is not the limit")
            elif sub["rate"] == "undetermined" and at_client is not None:
                notes.append("rate expectation cannot be decided: " + (at_client.get("reason") or f"observation bounded by {at_client['observation_bounded_by']}"))
        res.estimates["sub_verdicts"] = sub
        res.outcome = Outcome(combine(sub))
        if not te.declared:
            res.decision_basis = "no temporal expectation declared (discover-only): observations reported, no conformance verdict"
        elif res.outcome == Outcome.CONFORMANT:
            res.decision_basis = "every declared temporal expectation satisfied: " + ", ".join(k for k, v in sub.items() if v == "satisfied")
        else:
            res.decision_basis = "; ".join(notes) if notes else "declared expectation(s) undetermined: " + ", ".join(k for k, v in sub.items() if v == "undetermined")
        res.notes += notes
        res.duration_s = time.time() - t0
        return res
