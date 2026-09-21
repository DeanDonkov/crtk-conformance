"""RateSensitivityProbe — temporal binding class (state precondition, command liveness / stop behaviour, rate).

Version 0.1.3 (designs: rc3/LIVENESS_PROBE_DESIGN.md, rc3/RATE_ESTIMATOR_DESIGN.md, revised in rc4/; 0.1.3 changes in CHANGELOG.md).
0.1.5 (RC7 adversarial review): the response latency of the last streamed command is accepted only after its departure
was observed (last_command_latency); the onset-based drift lower bound carries the granularity term G; post-gap
non-responses are not read as a stop policy when calibration commands sent without a silence were also unanswered
(timeout_interval_from_trials is the interval logic as a pure, replayable function).

Sub-probes, each reported separately:

  A. state precondition   Is there an operating_state topic? Is servo_cp executed when DISABLED and
                          after enable/home (or without any state machine)?  Enable latency.
  B. liveness / stop      After streaming commands, stay silent for a gap g, then send one command.  The
     behaviour            probe first measures its own timing resolution (sleep, send, feedback period,
                          response latency) and reports a resolution floor r; gaps below r are
                          `below_resolution`.  During the gap the pose is recorded and the behaviour is
                          classified OBSERVATIONALLY as held / drifted / rejected / faulted / not_observable
                          (a command counts as rejected only when no motion toward it is observed; a tracking
                          error is not a rejection).  For a tripping policy the timeout is bracketed by
                          bisection between realised gaps (monotonic clock); the drift onset inside the gap
                          tightens the bracket for a drift.  0.1.2: the timeout is reported as an interval
                          that CONTAINS it under a deterministic-timeout model (union of the trial brackets
                          widened by a transport-latency allowance), not as the repeatability of midpoints.
  C. rate                 Command distinct setpoints at f.  Feedback samples are classified to the nearest
                          commanded target and monotone target crossings counted -- a DIAGNOSTIC only, since a
                          controller moving continuously to a late command crosses the earlier targets.
                          0.1.2: the verdict uses the accepted-command channel setpoint_cp: acceptance events
                          and the longest stale interval between them (what eq. (9) needs); an interface
                          without setpoint_cp gets no rate verdict.

Decision (temporal): only against the client's *declared* expectations (expectations.py):
  state_machine required|forbidden, stop_behaviour hold|fault|drift|release with a horizon, rate required.
  With no declared temporal expectation the probe reports its observations and returns UNDETERMINED.
  `hold` is decided as "held within the hold tolerance for silences up to the tested range and responded
  afterwards"; a longer timeout is never excluded and the report says so.  `release` is undetermined from
  the pose alone.  The eq. (7) timing margin is checked against the timeout INTERVAL (satisfied only below its
  lower end, violated only above its upper end).

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
from ..liveness import attributable, drift_evaluated, last_command_latency, required_confirmations, timeout_interval_from_trials  # noqa: F401 (0.1.5/0.1.6: ROS-free liveness decision logic)
from ..rate_estimator import (
    AppliedAgeEstimate,
    estimate_applied_age,
    RateEstimate,
    estimate_noise_sigma,
    estimate_rate,
    match_tolerance_from_sigma,
    rate_subverdict,
    RATE_VERDICT_WITHDRAWN_NOTE,
)
from ..stats import estimate, rate_estimate
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult
from .common import ensure_enabled, wait_settled

STOP_CLASSES = ("held", "drifted", "rejected", "faulted", "not_observable", "held_through_range")  # 0.1.2: observational names


MIN_RATE_TARGETS = 5  # fewest separable targets from which an observable rate is reported


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
        still_tol_m: float = 1e-5,
        latency_bound_s: Optional[float] = None,
        calibration_commands: int = 30,
        liveness_rule: str = "0.1.6",
        confirm_alpha: float = 0.01,
        confirm_max: int = 4,
        skip_rate_sweep: bool = False,
    ):
        self.a = adapter
        # 0.1.6 (RC9 / external review, point 4): calibration commands sent without a preceding silence (0.1.5: 12), the
        # liveness rule ('0.1.6': a non-response without a state change counts as a stop policy only when it is confirmed
        # at the same gap, liveness.required_confirmations; '0.1.5': the archived rule) and its confirmation parameters
        self.n_calib = int(calibration_commands)
        self.liveness_rule = liveness_rule
        self.confirm_alpha = confirm_alpha
        self.confirm_max = confirm_max
        self.skip_rate_sweep = skip_rate_sweep
        self._r_confirm = None
        self.latency_bound_s = latency_bound_s  # 0.1.3: an externally justified bound on the one-way transport latency; None = use the run maximum of the observed response latencies and label the estimate conditional
        self._latencies: List[float] = []  # every response latency observed during the run (calibration probes, post-gap responses, stream tails)
        self.still_tol = still_tol_m  # implementation constant: motion below max(this, 3 sigma_hat) is 'still' (drift onset threshold)
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

    def _stream(self, buf, base_T) -> Tuple[float, np.ndarray, float]:
        """Stream small oscillating commands to keep any liveness policy fed; return (t_last, last setpoint, t_half).

        t_last is the send time of the last command (the return to base), t_half the send time of the half-step
        offset that precedes it (0.1.5: the departure to that offset must be observed before the return to base can
        be read as the response to the last command, see last_command_latency)."""
        period = 1.0 / self.stream_rate
        t_end = time.monotonic() + self.stream_duration
        k = 0
        while time.monotonic() < t_end:
            T = base_T.copy()
            T[0, 3] += 0.25 * self.step * math.sin(2 * math.pi * k / 20.0)
            self.a.servo_cp(T)
            k += 1
            time.sleep(period)
        # 0.1.3: the stream ends with an OBSERVABLE last transaction -- a half-step offset followed by the return to
        # base -- so that the latency of the last command before the silence can be bounded from the feedback
        # observed during the gap itself (its response latency bounds its one-way transport latency)
        T = base_T.copy(); T[self.probe_axis, 3] += 0.5 * self.step * self.probe_sign
        t_half = time.monotonic()
        self.a.servo_cp(T)
        time.sleep(period)
        self.a.servo_cp(base_T)
        return time.monotonic(), base_T[:3, 3].copy(), t_half

    def _recover(self, buf, base_T):
        if self.a.has("operating_state"):
            ensure_enabled(self.a, timeout_s=2.0)
        self.a.servo_cp(base_T)
        self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))

    # ------------------------------------------------------------------ resolution
    def _latency_allowance(self):
        """(L, source, conditional): the one-way transport allowance used where a transaction cannot bound itself --
        a client-supplied bound, else the run maximum of the observed response latencies (a sample maximum, not a
        bound: the result is labelled conditional), else unknown (nan)."""
        if self.latency_bound_s is not None:
            return float(self.latency_bound_s), "client-supplied one-way latency bound", False
        if self._latencies:
            return float(max(self._latencies)), "run maximum of %d observed response latencies (not a bound)" % len(self._latencies), True
        return float("nan"), "no response latency observed", True

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
        # feedback period and resting noise, measured while HOLDING the current pose under a command stream at the
        # client rate: a silence-triggered stop policy (release with drift) must not be allowed to fire during the noise
        # window, otherwise the drift is mistaken for measurement noise and the hold tolerance inflates (found in the
        # v0.1.1 mock campaign, first run: a 0.1 s release policy was missed for exactly this reason).
        buf.clear()
        t0 = time.monotonic()
        period = 1.0 / max(1.0, self.tol.client_rate_hz)
        k = 0
        while time.monotonic() - t0 < 1.0:
            self.a.servo_cp(base_T)
            k += 1
            dt = t0 + k * period - time.monotonic()
            if dt > 0:
                time.sleep(dt)
        samples = buf.since(t0)
        out["feedback_period_s"] = (time.monotonic() - t0) / max(1, len(samples))
        out["resting_noise_measured_under_hold_stream_hz"] = float(self.tol.client_rate_hz)
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
        # 0.1.2: a stop policy may have fired during the silent settling before this call (every v0.1.1 fault case with
        # tau_w < 1 s had no latency measurement for this reason); recover before the first probe and after any
        # probe that did not respond, so that the latency allowance of the liveness interval is measured
        self._recover(buf, base_T)
        for k in range(self.n_calib):  # 0.1.6: self.n_calib (0.1.5: 12)
            ax, sg = dirs[k % len(dirs)]
            cur = self.a.latest_pose(buf, 1.0)
            goal = (cur if cur is not None else base_T).copy()
            goal[ax, 3] += self.step * sg
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(1.0, 5 * self.response_timeout))
            if r["responded"] and not math.isnan(r["time_to_respond_s"]):
                lat.append(r["time_to_respond_s"])
                self._latencies.append(float(r["time_to_respond_s"]))
            else:
                self._recover(buf, base_T)
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
        """Stream, stay silent for gap_s from the last command (no fixed sleep), send one command, classify.

        0.1.2: observational classes `held` / `drifted` / `rejected` / `faulted` / `not_observable`; the drift
        onset (first sustained departure from the settled pose by more than max(3 sigma_hat, still tolerance)) is
        located inside the gap, so that a drift policy can be timed without the fixed 0.2 s window of 0.1.1."""
        self._recover(buf, base_T)
        t_last, p_last, t_half = self._stream(buf, base_T)
        # 0.1.5: the operating state at the start of the silence, from the latest state message already received (no
        # waiting, so the gap timing is untouched).  A silence that starts in FAULT/DISABLED -- the recovery before the
        # stream failed, e.g. its enable or return-to-base command was lost -- cannot evidence a silence-triggered policy
        state0 = None
        if self.a.has("operating_state"):
            d0s = self.a.subscribe("operating_state").latest()
            state0 = None if d0s is None else d0s[1].state
        # 0.1.5: the buffer is not cleared here -- the samples between the half-step send and t_last are needed by the
        # departure guard of last_command_latency(); gap samples are still selected by time (since t_last)
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
        # 0.1.5: when the state was read, relative to the last streamed send.  Under baseline command loss the post-gap
        # command itself may have been lost, and a FAULT seen afterwards then bounds tau_w by this time, not by the gap
        t_state = time.monotonic() - t_last
        # settled reference: samples in the first max(0.1 s, settle_hint) of the gap (the stream's tracking error
        # settles there; measuring from the last streamed setpoint would confuse it with a drift -- live SRC v1.0.0)
        fp = self.resolution.get("feedback_period_s", 0.01)
        ref_len = max(0.1, float(self.resolution.get("settle_hint_s", 0.0) or 0.0))
        gap_pos = [(tq - t_last, pose_msg_to_matrix(m)[:3, 3]) for tq, m in gap_samples]
        # 0.1.3: latency of the last streamed transaction (half-step offset -> base): the first gap sample that has
        # come back within half of that offset to the base pose; NaN if the gap ended before it could be seen.
        # 0.1.5: accepted only after the departure to the half-step offset was observed (last_command_latency): an
        # implementation that applies commands later than one feedback period is still executing the in-band stream
        # when the gap starts, and the first in-band sample is then NOT the response to the last command.
        # Only samples up to the post-gap send can be attributed to the last streamed command: after it, an in-band
        # sample may be the pose passing the base on its way to the post-gap goal (live SRC v1.0.0: a 2.05-s "response"
        # read after a 2.0-s silence).  A response that arrives later is not attributable and L is used.
        half = 0.25 * self.step
        since_half = [(tq, pose_msg_to_matrix(m)[:3, 3]) for tq, m in buf.since(t_half) if tq <= t_send]
        r_last, departure_seen = last_command_latency(since_half, t_last, p_last, half + 3.0 * self.sigma_hat)
        if not math.isnan(r_last):
            self._latencies.append(r_last)
        if resp["responded"] and not math.isnan(resp["time_to_respond_s"]):
            self._latencies.append(float(resp["time_to_respond_s"]))
        # the reference is the settled pose at the END of the window (the last command's execution and the stream's
        # tracking error land in its first part); motion across the window is flagged (settling or an early drift)
        ref_pts = [q for tq, q in gap_pos if 0.5 * ref_len < tq <= ref_len] or [q for tq, q in gap_pos if tq <= ref_len] or ([gap_pos[0][1]] if gap_pos else [])
        early_pts = [q for tq, q in gap_pos if tq <= 0.25 * ref_len and (math.isnan(r_last) or tq > r_last)]  # after the last command has settled
        after = [(tq, q) for tq, q in gap_pos if tq > ref_len]
        drift, onset, speed, ref_motion, thr = float("nan"), None, None, False, max(3.0 * self.sigma_hat, self.still_tol)
        if ref_pts and after:
            ref = np.mean(ref_pts, axis=0)
            d = [float(np.linalg.norm(q - ref)) for _, q in after]
            drift = max(d)
            # onset: first sample exceeding thr that is followed by two more exceedances (a spike is not a drift)
            for k in range(len(d) - 2):
                if d[k] > thr and d[k + 1] > thr and d[k + 2] > thr:
                    onset = float(after[k][0])
                    break
            if onset is not None:
                # drift speed: least-squares slope of the departure over the samples from the onset on (m/s)
                tt = np.array([tq for tq, _ in after[k:]]); dd = np.array(d[k:])
                if len(tt) >= 2 and np.ptp(tt) > 0:
                    speed = float(np.polyfit(tt, dd, 1)[0])
            if early_pts:
                ref_motion = float(np.linalg.norm(np.mean(early_pts, axis=0) - ref)) > thr
        n_gap = len(gap_samples)
        # 0.1.5: (i) the drift can only be evaluated on samples after the reference window; (ii) an onset can only be
        # located if the reference window itself was still (a drift that began inside it is censored: the reference is
        # then a mean over drifting samples and the first sample after the window already exceeds the threshold);
        # (iii) the response to the post-gap command can only be observed if the pose was not already at its goal (the
        # previous return to base may have been lost); (iv) the silence must have started in an enabled state.
        drift_eval = bool(ref_pts and after)
        still_thr = max(4.0 * self.sigma_hat, self.still_tol)
        ref_still = bool(ref_pts) and (max(float(np.linalg.norm(q - np.mean(ref_pts, axis=0))) for q in ref_pts) <= still_thr)
        d_pre_goal = float(np.linalg.norm(p_pre - goal[:3, 3]))
        not_observable_reason = None
        if state0 in ("FAULT", "DISABLED"):
            not_observable_reason = f"operating state {state0} at the start of the silence (recovery before the stream failed)"
        elif d_pre_goal < 0.25 * self.step:
            not_observable_reason = "pose already at the post-gap goal when the silence ended (previous return to base lost): the response cannot be observed"
        if not_observable_reason is not None:
            cls = "not_observable"
        elif not resp["responded"]:
            cls = "faulted" if state in ("FAULT", "DISABLED") else "rejected"
        elif (n_gap < 2 and gap_s >= 2 * fp) or (not after and gap_s >= ref_len + 0.1):
            cls = "not_observable"
            not_observable_reason = "too few feedback samples during the silence" if n_gap < 2 else "no feedback sample after the reference window"
        elif after and drift > self.hold_tol:
            cls = "drifted"
        else:
            cls = "held"
        self.a.servo_cp(base_T)
        self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))
        return {"gap_requested_s": gap_s, "gap_s": actual_gap, "below_resolution": gap_s < self.resolution.get("resolution_floor_s", 0.0),
                "executed": ok, "responded": resp["responded"], "closest_approach_m": resp["closest_m"], "moved_m": resp["moved_m"],
                "initial_distance_m": resp["initial_distance_m"], "settled_distance_m": resp["settled_distance_m"], "reduction_m": resp["reduction_m"],
                "time_to_execute_s": t_exec, "time_to_respond_s": resp["time_to_respond_s"], "last_stream_latency_s": r_last,
                "last_stream_departure_observed": departure_seen, "state_observed_at_s": t_state, "state_at_gap_start": state0,
                "distance_to_goal_at_gap_end_m": d_pre_goal, "not_observable_reason": not_observable_reason,
                "drift_m": drift, "drift_onset_s": onset, "drift_speed_m_s": speed, "onset_threshold_m": thr,
                "reference_window_motion": ref_motion, "reference_window_still": ref_still, "drift_evaluated": drift_eval, "reference_window_s": ref_len,
                "samples_in_gap": n_gap, "class": cls, "state": state}

    def _tripped(self, r: dict) -> bool:
        if self.liveness_rule == "0.1.6" and r["class"] == "rejected":
            return bool((r.get("confirmation") or {}).get("confirmed"))
        return r["class"] in ("rejected", "faulted", "drifted")

    def _trial(self, buf, base_T, gap_s: float, sink: list) -> dict:
        """0.1.6: a gap trial with the confirmation rule.  Under rule 0.1.6 a trial classified `rejected` (no response and
        no state change: indistinguishable from the loss of the post-gap command) is repeated at the same requested gap,
        each repeat preceded by its own answered stream (the stream that starts every _gap_trial), until the required
        number r of rejections is reached or a repeat is not a rejection.  Every attempt is recorded in `sink` (the
        trial list the interval is formed from); the rejections of the group carry confirmation.confirmed = (count >= r).
        A repeat that is held is a passing trial at that gap; one that faulted or drifted is attributable on its own.
        Returns the record that stands for the gap in the bisection: the confirmed rejection, a tripping repeat, a
        passing repeat, or the unconfirmed rejection (which moves neither end of the bracket)."""
        r = self._gap_trial(buf, base_T, gap_s)
        sink.append(r)
        if self.liveness_rule != "0.1.6" or r["class"] != "rejected":
            return r
        group = [r]
        need = self._r_confirm
        if need is None:
            r["confirmation"] = {"required": None, "rejected_attempts": 1, "confirmed": False,
                                 "reason": "baseline command loss too high for confirmation (required repeats exceed the maximum): unattributable"}
            return r
        other = None
        while sum(1 for g in group if g["class"] == "rejected") < need:
            rr = self._gap_trial(buf, base_T, gap_s)
            rr["repeat_of_gap_s"] = gap_s
            sink.append(rr)
            if rr["class"] != "rejected":
                other = rr
                break
            group.append(rr)
        n_rej = sum(1 for g in group if g["class"] == "rejected")
        confirmed = n_rej >= need
        for g in group:
            g["confirmation"] = {"required": need, "rejected_attempts": n_rej, "confirmed": confirmed}
        if confirmed:
            return r
        if other is not None and other["class"] in ("held", "faulted", "drifted"):
            return other
        return r

    def _is_passing(self, r: dict) -> bool:
        return r["class"] == "held"

    def probe_liveness(self, buf, base_T) -> dict:
        """0.1.3: the timeout is reported as an INTERVAL that contains it under a deterministic-timeout model, with
        every allowance either bounded by the same transaction it concerns or declared as an assumption.

        Silence is measured between the probe's own sends; the implementation sees it start when the last streamed
        command arrives (one-way latency l1) and end when the post-gap command arrives (l2).  Per trial:
          * passing (held, next command acted on) with realised silence g:  tau_w > g - l1 - G - t_det, and l1 is
            bounded by the RESPONSE latency of that same last command, r_last, observed in the feedback during the
            gap (a response duration bounds the transport duration of its own transaction; RC4 review, finding 6);
          * tripping by rejection/fault at silence g:  tau_w <= g + l2; the post-gap command drew no response, so l2
            cannot be bounded by its own transaction and the allowance L is used (conditional, below);
          * tripping by drift with onset o after the last send:  tau_w <= o (the drift cannot start before the policy
            fires and cannot be seen before it starts: no latency term); and tau_w >= o - l1 - thr / v_min - fp - l_fb
            with l1 <= r_last and the feedback transport l_fb <= L (conditional).
        L is an externally justified one-way latency bound given by the client (`latency_bound_s`); without one the
        run maximum of every response latency observed (calibration probes, stream tails, post-gap responses) is
        used and the estimate is labelled CONDITIONAL on one-way latencies not exceeding it -- a sample maximum is
        not a bound.  G (one feedback period, the assumed evaluation granularity), t_det = hold_tol / v_min + fp for
        a drift and v_min (the slowest drift speed observed, assumed to hold during detection) are assumptions and
        are listed in the estimate.  The interval is the intersection of all bounds; an empty intersection is
        reported as `inconsistent`."""
        r_floor = self.resolution.get("resolution_floor_s", 0.0)
        fp = float(self.resolution.get("feedback_period_s", 0.01) or 0.01)
        G = fp
        out = {"gap_max_s": self.gap_max, "resolution_floor_s": r_floor, "response_timeout_s": self.response_timeout,
               "hold_tolerance_m": self.hold_tol, "granularity_allowance_s": G, "trials": [], "liveness_rule": self.liveness_rule}
        # 0.1.6: the confirmation requirement follows from the calibration (commands sent without a preceding silence)
        n_resp0 = int(self.resolution.get("latency_probes_responded", self.n_calib) or 0)
        if self.liveness_rule == "0.1.6":
            self._r_confirm, p_up = required_confirmations(self.n_calib, max(0, self.n_calib - n_resp0), self.confirm_alpha, self.confirm_max)
            out["confirmation_rule"] = {"calibration_commands": self.n_calib, "lost": max(0, self.n_calib - n_resp0), "loss_upper_bound_95": p_up,
                                        "alpha": self.confirm_alpha, "required_rejections_per_gap": self._r_confirm, "max_repeats": self.confirm_max}
        big = [self._trial(buf, base_T, self.gap_max, out["trials"]) for _ in range(self.trials)]
        n_trip = sum(1 for b in big if self._tripped(b))
        out["tripped_at_gap_max"] = rate_estimate(n_trip, len(big))
        drifts = [b["drift_m"] for b in big if not math.isnan(b["drift_m"])]
        out["drift_during_gap_max_m"] = estimate(drifts).to_dict() if drifts else None
        classes = [b["class"] for b in big]

        allowance = self._latency_allowance
        n_big_records = len(out["trials"])  # 0.1.6: every record of the gap_max trials, confirmation repeats included
        unconfirmed = [b for b in big if b["class"] == "rejected" and not self._tripped(b)]
        if n_trip == 0 and unconfirmed:
            # 0.1.6: non-responses that could not be confirmed (baseline loss too high to require a finite number of repeats)
            # are neither passes nor trips: the stop behaviour is not observable
            out["stop_class"] = "not_observable"
            out["tau_w_estimate_s"] = None
            L, Lsrc, cond = allowance()
            out["latency_allowance_s"], out["latency_allowance_source"], out["latency_allowance_conditional"] = L, Lsrc, cond
            out["finding"] = (f"{len(unconfirmed)} non-response(s) at the largest gap could not be confirmed "
                              f"({(unconfirmed[0].get('confirmation') or {}).get('reason', 'unconfirmed')}): stop behaviour not observable")
            return out
        if n_trip == 0:
            out["stop_class"] = "not_observable" if all(c == "not_observable" for c in classes) else "held_through_range"
            out["tau_w_estimate_s"] = None
            L, Lsrc, cond = allowance()
            out["latency_allowance_s"], out["latency_allowance_source"], out["latency_allowance_conditional"] = L, Lsrc, cond
            out["finding"] = (f"held within {self.hold_tol:.4g} interface units for silences up to {self.gap_max} s and responded afterwards (n={len(big)}); "
                              f"a stop policy with a longer timeout is not excluded"
                              if out["stop_class"] == "held_through_range" else "stop behaviour not observable (measured_cp too sparse during the gap)")
            return out
        trip_classes = [b["class"] for b in big if self._tripped(b)]
        out["stop_class"] = max(set(trip_classes), key=trip_classes.count)
        # bisection per run between the resolution floor and gap_max; each run yields a bracket of realised gaps
        lo0 = max(self.gap_min, r_floor)
        brackets: List[List[float]] = []
        below = 0

        def hi_of(r):
            # the time by which the trip is evidenced: the realised gap, or the drift onset inside the gap
            if r["class"] == "drifted" and r.get("drift_onset_s") is not None:
                return min(r["gap_s"], r["drift_onset_s"])
            return r["gap_s"]
        for _ in range(self.trials):
            r = self._trial(buf, base_T, lo0, out["trials"])
            if self._tripped(r):
                below += 1  # trips even at the smallest resolvable gap
                continue
            lo, hi = lo0, self.gap_max
            lo_real = r["gap_s"]
            hi_real = min(hi_of(b) for b in big if self._tripped(b))
            for _ in range(self.bisect):
                if hi_real - lo_real <= max(2 * fp, 0.5 * r_floor):
                    break  # the bracket is already at the feedback resolution
                mid = 0.5 * (lo + hi)
                r = self._trial(buf, base_T, mid, out["trials"])
                if r["class"] == "not_observable" or (r["class"] == "rejected" and not self._tripped(r)):
                    continue  # 0.1.5: an unobservable trial moves neither end of the bracket; 0.1.6: nor does an unconfirmed rejection
                if self._tripped(r):
                    hi, hi_real = mid, min(hi_real, hi_of(r))
                else:
                    lo, lo_real = mid, max(lo_real, r["gap_s"])
            brackets.append([lo_real, hi_real])
        out["trials_below_resolution"] = below
        out["brackets_s"] = brackets
        L, Lsrc, cond = allowance()
        out["latency_allowance_s"], out["latency_allowance_source"], out["latency_allowance_conditional"] = L, Lsrc, cond
        # 0.1.5 (RC7 review, finding F10): a post-gap command that draws no response is evidence of a stop policy only if
        # commands sent WITHOUT a preceding silence are answered.  The calibration of measure_resolution() sent 12 such
        # commands; every one that drew no response is a baseline command loss that rejection-class trials inherit.
        # A FAULT seen in the operating state and a drift seen in the pose are not producible by loss and remain
        # attributable (a faulted trial then bounds tau_w by the time its state was observed, since the post-gap
        # command may itself have been lost); a `rejected` trial is not, and the run is confounded when no attributable
        # tripping trial remains.
        n_probes = self.n_calib  # 0.1.6: the configured count (0.1.5: 12)
        n_resp = int(self.resolution.get("latency_probes_responded", n_probes) or 0)
        baseline_loss = max(0, n_probes - n_resp)
        tripped_trials = [b for b in out["trials"] if self._tripped(b)]
        n_attrib = sum(1 for b in tripped_trials if attributable(b, baseline_loss, self.liveness_rule))
        confounded = baseline_loss > 0 and n_attrib == 0
        out["baseline_command_loss"] = {"calibration_commands": n_probes, "responded": n_resp, "lost": baseline_loss,
                                        "tripped_trials": len(tripped_trials), "attributable_tripped_trials": n_attrib}
        out["rejection_confounded_by_command_loss"] = confounded
        if baseline_loss > 0:
            out["confound_note"] = (f"{baseline_loss} of {n_probes} calibration commands sent without any preceding silence drew no response: "
                                    "post-gap non-responses cannot be attributed to a silence-triggered stop policy (random command loss "
                                    "is indistinguishable from rejection through the pose alone)"
                                    + ("" if confounded else f"; {n_attrib} of {len(tripped_trials)} tripped trials show the policy in the operating state or in the pose and are used, "
                                       f"the {len(tripped_trials) - n_attrib} non-responses without a state change are not"))
        if below > 0 and not brackets:
            status = "undetermined" if confounded else "upper_bound"
            below_trials = [b for b in out["trials"][n_big_records:] if self._tripped(b)]
            ub = lo0 + L
            if baseline_loss > 0 and not confounded:
                highs = [float(b["state_observed_at_s"]) for b in below_trials if b["class"] == "faulted" and b.get("state_observed_at_s") is not None]
                highs += [float(min(b["gap_s"], b["drift_onset_s"])) for b in below_trials if b["class"] == "drifted" and b.get("drift_onset_s") is not None]
                if not highs:
                    status = "undetermined"
                else:
                    ub = min(highs)
            out["tau_w_estimate_s"] = {"upper_bound_s": ub, "n": below, "status": status, "latency_allowance_s": L, "latency_allowance_source": Lsrc, "conditional": cond}
            if status == "undetermined":
                out["tau_w_estimate_s"]["reason"] = out.get("confound_note")
                out["finding"] = f"no response at every gap down to the resolution floor, but {out.get('confound_note')}"
            else:
                out["finding"] = f"{out['stop_class']} at every gap down to the resolution floor {lo0*1e3:.1f} ms: tau_w <= {ub*1e3:.1f} ms (below resolution, n={below})"
            return out
        # ---- the interval from all trials (timeout_interval_from_trials; replayable offline)
        trials = out["trials"]
        assumptions = [f"the implementation evaluates its stop policy at least once per feedback period (G = {G*1e3:.1f} ms)",
                       "the timeout is deterministic (one value tau_w)"]
        if cond:
            assumptions.append(f"one-way transport latencies do not exceed L = {L*1e3:.1f} ms, the run maximum of the observed response latencies (a sample maximum, not a bound; supply --latency-bound-s for a justified bound)")
        else:
            assumptions.append(f"one-way transport latencies do not exceed the client-supplied bound L = {L*1e3:.1f} ms")
        est = {"n": len(brackets), "brackets_s": brackets, "latency_allowance_s": L, "latency_allowance_source": Lsrc, "conditional": cond,
               "granularity_allowance_s": G, "feedback_period_s": fp, "drift_speed_min_m_s": None, "detection_delay_s": 0.0,
               "last_stream_latencies_s": [t.get("last_stream_latency_s") for t in trials],
               "last_stream_departures_observed": [bool(t.get("last_stream_departure_observed")) for t in trials],
               "interval_semantics": ("intersection over trials of [passing gap - r_last - G - t_det, trip evidence + L] (rejection/fault) or "
                                      "[onset - r_last - G - thr / v_min - fp - L, onset] (drift); r_last = the response latency of the last streamed "
                                      "command of that trial, accepted only after its departure to the half-step offset was observed (0.1.5), else L; "
                                      "L = the latency allowance for the transactions that drew no response; under baseline command loss a rejected trial "
                                      "contributes no bound and a faulted trial bounds tau_w by the time its FAULT state was observed (0.1.5); contains tau_w "
                                      "under the listed assumptions")}
        if math.isnan(L):
            est.update(status="undetermined", reason="no response latency observed in the run and no latency bound supplied: the allowance is unknown", assumptions=assumptions)
            out["tau_w_estimate_s"] = est
            out["finding"] = f"{out['stop_class']} observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
            return out
        iv = timeout_interval_from_trials(trials, L=L, G=G, fp=fp, hold_tol=self.hold_tol, stop_class=out["stop_class"], baseline_loss=baseline_loss,
                                          rule=self.liveness_rule)
        est.update({k: iv[k] for k in ("drift_speed_min_m_s", "detection_delay_s", "n_trials_with_own_last_latency", "onset_based_lower_bounds_s",
                                       "reference_window_motion_trials", "passing_lower_bounds_s", "tripping_upper_bounds_s", "lower_bound_uses_L_for_all_trials",
                                       "unattributable_trials", "attributable_tripping_trials")})
        if out["stop_class"] == "drifted":
            if iv["drift_speed_min_m_s"] is None:
                est.update(status="undetermined", reason="drift speed not estimable (too few samples after the onset)", assumptions=assumptions)
                out["tau_w_estimate_s"] = est
                out["finding"] = f"drifted observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
                return out
            assumptions.append(f"the drift speed during detection is at least the slowest observed, v_min = {iv['drift_speed_min_m_s']*1e3:.2f} mm/s (t_det = {iv['detection_delay_s']*1e3:.0f} ms)")
        if est["n_trials_with_own_last_latency"] == 0 and cond:
            assumptions.append("no trial's last streamed command showed an observable departure before its return to base, so every lower bound uses the allowance L instead of a measured response latency")
        est.update(interval_low_s=iv["interval_low_s"], interval_high_s=iv["interval_high_s"], point_s=iv["point_s"],
                   bracket_width_median_s=float(np.median([b[1] - b[0] for b in brackets])), assumptions=assumptions)
        if confounded:
            est.update(status="undetermined", reason=out["confound_note"])
            out["tau_w_estimate_s"] = est
            out["finding"] = f"post-gap non-responses observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
            return out
        if iv["status"] == "inconsistent":
            est.update(status="inconsistent", reason=f"lower bound {iv['interval_low_s']:.4f} s above upper bound {iv['interval_high_s']:.4f} s: the deterministic-timeout model with the listed allowances does not describe the observations")
            out["tau_w_estimate_s"] = est
            out["finding"] = f"{out['stop_class']} observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
            return out
        if len(brackets) < 3 or not (est["interval_low_s"] > r_floor):
            est.update(status="undetermined", reason=("n < 3" if len(brackets) < 3 else "interval lower bound not above the resolution floor"))
            out["tau_w_estimate_s"] = est
            out["finding"] = f"{out['stop_class']} observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
            return out
        est["status"] = "ok"
        out["tau_w_estimate_s"] = est
        out["finding"] = (f"{out['stop_class']} policy detected: tau_w in [{est['interval_low_s']:.3f}, {est['interval_high_s']:.3f}] s "
                          f"(midpoint {est['point_s']:.3f} s, n={len(brackets)}; L {L*1e3:.0f} ms {'conditional' if cond else 'client-supplied'}, G {G*1e3:.0f} ms, t_det {est['detection_delay_s']*1e3:.0f} ms)")
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
            # target spacing must exceed 4 delta; enlarge the step, and if that would exceed the allowed excursion
            # send fewer targets at the SAME requested rate (shorter window).  Fewer than MIN_RATE_TARGETS
            # separable targets within the excursion -> undetermined (targets_not_separable); nothing is sent.
            step = self.step
            spacing = step / n
            note = ""
            if spacing < 4 * delta:
                step = 4 * delta * n
                if step > self.rate_max_step:
                    n_max = int(self.rate_max_step / (4 * delta))
                    if n_max < MIN_RATE_TARGETS:
                        row = {"command_rate_requested_hz": float(f), "commands_sent": 0, "client_rate_achieved_hz": 0.0, "window_s": 0.0,
                               "publish_rate_hz": (1.0 / self.resolution["feedback_period_s"]) if self.resolution.get("feedback_period_s") else 0.0, "samples_total": 0, "samples_matched": 0,
                               "samples_unmatched": 0, "unmatched_fraction": 0.0, "targets_reached": 0, "transitions": 0, "observable_rate_hz": 0.0,
                               "match_tolerance_m": delta, "match_tolerance_source": src, "target_spacing_m": 4 * delta, "observation_bounded_by": "none",
                               "status": "undetermined", "reason": "targets_not_separable",
                               "note": f"{n} targets at spacing >= 4 delta = {4 * delta:.3g} would need an excursion of {step:.3g} > max {self.rate_max_step:.3g}; "
                                       f"only {n_max} separable targets fit (minimum {MIN_RATE_TARGETS}); no commands sent",
                               "step_used_m": None, "command_window_s": 0.0, "zoh_error_bound_m": None,
                               "feedback_target_crossings_hz": 0.0, "feedback_count_is_evidence_of_execution": False,
                               "acceptance": AppliedAgeEstimate(channel="setpoint_cp" if sp_buf is not None else "none", commands_sent=0, accepted=0, accepted_fraction=0.0, out_of_order_events=0,
                                                                first_application_delay_s=float("nan"), age_lower_s=float("nan"), age_upper_s=float("nan"), max_update_gap_s=float("nan"), channel_period_s=float("nan"),
                                                                samples_in_window=0, client_rate_achieved_hz=0.0, client_max_send_gap_s=float("nan"), status="undetermined", reason="targets_not_separable").to_dict()}
                        out["per_rate"].append(row)
                        continue
                    n = n_max
                    step = 4 * delta * n
                    note = f"targets reduced to {n} (spacing 4 delta) at the requested rate: window shortened to {n / f:.3g} s to stay within the excursion {self.rate_max_step:.3g}"
                spacing = step / n
            targets = np.tile(base_T[:3, 3], (n, 1))
            targets[:, 0] += step * (np.arange(n) + 1) / n
            buf.clear()
            if sp_buf is not None:
                sp_buf.clear()
            send_times = []
            send_stamps = []
            t_start = time.monotonic()
            for k in range(n):
                T = base_T.copy()
                T[0, 3] = targets[k, 0]
                send_times.append(time.monotonic())
                send_stamps.append(self.a.servo_cp(T))
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
            row["zoh_error_bound_m"] = None  # 0.1.3: set below from the source-age bracket of the applied setpoint (eq. 9 with the age, not a mean rate or an update gap)
            # 0.1.2: the feedback count above is a DIAGNOSTIC (target crossings); the verdict uses the applied-setpoint
            # channel setpoint_cp when the implementation exposes one, and is undetermined otherwise
            row["feedback_target_crossings_hz"] = est.observable_rate_hz
            row["feedback_count_is_evidence_of_execution"] = False
            acc = None
            trace = {"send_times_mono": [round(float(x), 6) for x in send_times], "send_stamps_wall": [round(float(x), 6) for x in send_stamps],
                     "targets_x": [round(float(x), 7) for x in targets[:, 0]], "window_end_mono": round(float(t_end), 6)}
            if sp_buf is not None:
                sps = sp_buf.since(t_start - 1.0)  # include the pre-window state of the channel
                SP = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in sps]) if sps else np.zeros((0, 3))
                sts = np.array([t for t, _ in sps])
                # the lower bound of the age subtracts the feedback-transport allowance: a client-supplied one-way
                # bound when given; otherwise none (0), and a violated verdict is then conditional on the transport
                # delay of the channel being smaller than the margin by which the bound exceeds the period.  (The run
                # maximum of the observed response latencies is NOT used here: a response latency includes the
                # implementation's own application delay, i.e. the very quantity under test.)
                if self.latency_bound_s is not None:
                    L_fb, L_src, L_cond = float(self.latency_bound_s), "client-supplied one-way latency bound", False
                else:
                    L_fb, L_src, L_cond = 0.0, "none: age_lower is the age at the probe's receipt of the sample (transport delay of the channel not subtracted)", True
                acc = estimate_applied_age(targets, SP, sts, np.array(send_times), delta, t_end, feedback_latency_allowance_s=L_fb)
                row["feedback_latency_allowance_source"] = L_src
                row["feedback_latency_allowance_conditional"] = L_cond
                trace["setpoint_times_mono"] = [round(float(x), 6) for x in sts]
                trace["setpoint_positions"] = [[round(float(v), 7) for v in p] for p in SP]
            else:
                acc = AppliedAgeEstimate("none", int(len(send_times)), 0, 0.0, 0, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0,
                                         row["client_rate_achieved_hz"], float(np.max(np.diff(send_times))) if len(send_times) > 1 else float("nan"),
                                         "undetermined", "no_applied_setpoint_channel: the interface publishes no setpoint_cp")
            row["acceptance"] = acc.to_dict()
            row["trace"] = trace  # 0.1.3: every send and every channel sample, so that each acceptance and age can be audited
            if acc.status == "ok" and not math.isnan(acc.age_upper_s):
                row["zoh_error_bound_m"] = self.tol.speed_m_s * acc.age_upper_s
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
        C = {"skipped": True, "reason": "--skip-rate-sweep (0.1.6 campaign option); the rate sub-verdict is undetermined in every case"} if self.skip_rate_sweep else self.probe_effective_rate(buf, base_T)
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
        horizon = te.horizon_s if te.horizon_s is not None else self.gap_max
        res.estimates["stop_horizon_s"] = horizon
        tested_to = self.gap_max
        beyond_tested = horizon > tested_to + 1e-9
        beyond_note = f"the client's stop expectation is claimed up to {horizon} s of silence but the probe tested silences only up to {tested_to} s"
        if te.stop_behaviour != "any":
            if stop_class in (None, "not_observable"):
                sub["stop_behaviour"] = "undetermined"
            elif te.stop_behaviour == "release":
                sub["stop_behaviour"] = "undetermined"
                notes.append(f"a 'release' (actuation released) expectation cannot be decided from the pose alone; observed stop behaviour: {stop_class}")
            elif te.stop_behaviour == "hold":
                if stop_class == "held_through_range":
                    # a hold is only evidenced up to the longest tested silence: a horizon beyond it is undetermined
                    if beyond_tested:
                        sub["stop_behaviour"] = "undetermined"
                        notes.append(beyond_note)
                    else:
                        sub["stop_behaviour"] = "satisfied"
                        notes.append(f"held within {self.hold_tol:.4g} interface units for silences up to {tested_to} s (claimed horizon {horizon} s) and responded afterwards")
                else:
                    # a trip observed: violated if the policy is evidenced within the horizon (trip evidence time
                    # + L is an upper bound of tau_w); satisfied only if the interval's lower end is beyond it
                    L_ = float(B.get("latency_allowance_s") or 0.0)

                    loss_ = int((B.get("baseline_command_loss") or {}).get("lost", 0) or 0)

                    def _hi(t):
                        # a drift onset needs no latency term (the drift cannot precede the policy); a rejection/fault
                        # is evidenced by a command that drew no response, so the allowance L is added; under baseline
                        # command loss a fault is evidenced by the time its state was observed (0.1.5)
                        if t["class"] == "drifted" and t.get("drift_onset_s") is not None:
                            return min(t["gap_s"], t["drift_onset_s"])
                        if t["class"] == "faulted" and loss_ > 0:
                            return float(t["state_observed_at_s"])
                        return t["gap_s"] + L_
                    trips = [t for t in B.get("trials", []) if self._tripped(t) and attributable(t, loss_)]
                    trip_hi = min((_hi(t) for t in trips), default=None)
                    tau_lo = tau.get("interval_low_s") if isinstance(tau, dict) and tau.get("status") == "ok" else None
                    if B.get("rejection_confounded_by_command_loss"):
                        # 0.1.5: the non-responses that would evidence the trip also occur without a silence
                        sub["stop_behaviour"] = "undetermined"
                        notes.append("hold expectation undetermined: " + str(B.get("confound_note")))
                    elif trip_hi is not None and trip_hi <= horizon + 1e-9:
                        sub["stop_behaviour"] = "violated"
                        notes.append(f"client expects the pose to be held for silences up to {horizon} s; observed {stop_class} evidenced by {trip_hi:.3f} s of silence")
                    elif tau_lo is not None and tau_lo > horizon:
                        sub["stop_behaviour"] = "satisfied"
                        notes.append(f"{stop_class} policy only beyond the claimed horizon {horizon} s (tau_w >= {tau_lo:.3f} s)")
                    else:
                        sub["stop_behaviour"] = "undetermined"
                        notes.append(f"{stop_class} observed near the claimed horizon {horizon} s; whether the policy fires within it is undetermined")
            else:  # fault | drift expected
                want = ("faulted", "rejected") if te.stop_behaviour == "fault" else ("drifted",)
                if stop_class in want:
                    # two independent conditions (RC4 review, finding 4): (i) the policy fires WITHIN the claimed
                    # horizon -- decided on the timeout interval, not on the class alone; (ii) eq. (7): the client's
                    # own stream survives the policy -- decided against the interval, not a point
                    need = 1.0 / self.tol.client_rate_hz + self.tol.jitter_max_s
                    within = None
                    ok_margin = None
                    if isinstance(tau, dict) and tau.get("status") == "ok":
                        if tau["interval_high_s"] <= horizon + 1e-9:
                            within = True
                        elif tau["interval_low_s"] > horizon + 1e-9:
                            within = False
                            notes.append(f"the {stop_class} policy fires only after the claimed horizon {horizon} s (tau_w interval [{tau['interval_low_s']:.3f}, {tau['interval_high_s']:.3f}] s)")
                        else:
                            notes.append(f"whether the {stop_class} policy fires within the claimed horizon {horizon} s is undetermined (tau_w interval [{tau['interval_low_s']:.3f}, {tau['interval_high_s']:.3f}] s straddles it)")
                        if need < tau["interval_low_s"]:
                            ok_margin = True
                        elif need >= tau["interval_high_s"]:
                            ok_margin = False
                            notes.append(f"eq. (7) violated: client period + J_max = {need:.3f} s >= tau_w interval high {tau['interval_high_s']:.3f} s")
                        else:
                            notes.append(f"eq. (7) undecided: client period + J_max = {need:.3f} s lies inside the tau_w interval [{tau['interval_low_s']:.3f}, {tau['interval_high_s']:.3f}] s")
                    elif isinstance(tau, dict) and tau.get("status") == "upper_bound":
                        within = True if tau["upper_bound_s"] <= horizon + 1e-9 else None
                        ok_margin = False if need >= tau["upper_bound_s"] else None
                    if within is True and ok_margin is True:
                        sub["stop_behaviour"] = "satisfied"
                    elif within is False or ok_margin is False:
                        sub["stop_behaviour"] = "violated"
                    else:
                        sub["stop_behaviour"] = "undetermined"
                    if isinstance(tau, dict) and tau.get("conditional"):
                        notes.append(f"the tau_w interval is conditional on the listed assumptions (latency allowance {tau.get('latency_allowance_s', float('nan'))*1e3:.0f} ms is a run maximum, not a bound)")
                elif stop_class == "held_through_range":
                    # no trip within the tested silences: violated when the horizon lies inside the tested range,
                    # undetermined when the policy could still fire between the tested range and the horizon
                    if beyond_tested:
                        sub["stop_behaviour"] = "undetermined"
                        notes.append(beyond_note)
                    else:
                        sub["stop_behaviour"] = "violated"
                        notes.append(f"client expects a {te.stop_behaviour} stop policy within {horizon} s of silence; the pose was held and commands acted on up to {tested_to} s")
                elif B.get("rejection_confounded_by_command_loss"):
                    sub["stop_behaviour"] = "undetermined"
                    notes.append(f"client expects {te.stop_behaviour} stop behaviour; the observed non-responses are confounded: " + str(B.get("confound_note")))
                else:
                    sub["stop_behaviour"] = "violated"
                    notes.append(f"client expects {te.stop_behaviour} stop behaviour; observed {stop_class}")
        # rate at the client's declared rate
        rows = C["per_rate"]
        at_client = None
        if rows:
            at_client = min(rows, key=lambda r: abs(r["command_rate_requested_hz"] - self.tol.client_rate_hz))
            res.estimates["feedback_target_crossings_at_client_rate_hz"] = at_client["observable_rate_hz"]  # diagnostic, not execution evidence
            res.estimates["client_rate_achieved_hz"] = at_client["client_rate_achieved_hz"]
            res.estimates["publish_rate_hz"] = at_client["publish_rate_hz"]
            acc0 = at_client.get("acceptance") or {}
            res.estimates["zoh_error_at_client_rate_m"] = (self.tol.speed_m_s * acc0["age_upper_s"]) if acc0.get("status") == "ok" and acc0.get("age_upper_s") is not None and not (isinstance(acc0.get("age_upper_s"), float) and math.isnan(acc0["age_upper_s"])) else None
        if te.rate == "required":
            acc = None
            if at_client is not None and at_client.get("acceptance"):
                acc = AppliedAgeEstimate(**at_client["acceptance"])
            # 0.1.4: the rate class is a diagnostic, never a verdict -- rate_subverdict() returns "undetermined"
            # unconditionally.  The measurement below is unchanged from 0.1.3; only the label is withdrawn.
            sub["rate"] = rate_subverdict(acc, f_req)
            if acc is not None:
                res.estimates["applied_setpoint_age_at_client_rate"] = acc.to_dict()
            res.estimates["rate_note"] = RATE_VERDICT_WITHDRAWN_NOTE
            notes.append("rate expectation not decided: " + RATE_VERDICT_WITHDRAWN_NOTE)
            if acc is not None and acc.status == "ok" and not math.isnan(acc.age_lower_s) and not math.isnan(acc.age_upper_s):
                diag = ("diagnostic: source age of the applied setpoint bracketed [%.1f, %.1f] ms against a required period of %.0f ms "
                        "(setpoint_cp channel, %d of %d commands applied)" % (acc.age_lower_s * 1e3, acc.age_upper_s * 1e3, 1e3 / f_req, acc.accepted, acc.commands_sent))
                if not math.isnan(acc.client_rate_achieved_hz):
                    diag += "; the client's own stream sustained %.0f Hz" % acc.client_rate_achieved_hz
                    if not math.isnan(acc.client_max_send_gap_s):
                        diag += " with send gaps <= %.0f ms" % (acc.client_max_send_gap_s * 1e3)
                if at_client.get("feedback_latency_allowance_conditional", True):
                    diag += "; the lower end is the age at the probe's receipt of the channel sample, above the publication-side age by the unbounded transport delay of setpoint_cp (--latency-bound-s subtracts a client-supplied bound)"
                notes.append(diag)
            elif acc is not None and acc.reason:
                notes.append("diagnostic: no source-age bracket for this run -- " + acc.reason)
            elif at_client is not None and at_client.get("reason"):
                notes.append("diagnostic: no source-age bracket for this run -- " + str(at_client["reason"]))
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
