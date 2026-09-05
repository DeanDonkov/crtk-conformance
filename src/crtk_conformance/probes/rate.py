"""RateSensitivityProbe — temporal binding class (state precondition, command liveness / stop behaviour, rate).

Version 0.1.2 (designs: rc3/LIVENESS_PROBE_DESIGN.md, rc3/RATE_ESTIMATOR_DESIGN.md, both revised in rc4/).

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
from ..rate_estimator import (
    AcceptanceEstimate,
    estimate_acceptance,
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
    ):
        self.a = adapter
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
        for k in range(12):
            ax, sg = dirs[k % len(dirs)]
            cur = self.a.latest_pose(buf, 1.0)
            goal = (cur if cur is not None else base_T).copy()
            goal[ax, 3] += self.step * sg
            self.a.servo_cp(goal)
            r = self._response(buf, goal, timeout=max(1.0, 5 * self.response_timeout))
            if r["responded"] and not math.isnan(r["time_to_respond_s"]):
                lat.append(r["time_to_respond_s"])
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
        # settled reference: samples in the first max(0.1 s, settle_hint) of the gap (the stream's tracking error
        # settles there; measuring from the last streamed setpoint would confuse it with a drift -- live SRC v1.0.0)
        fp = self.resolution.get("feedback_period_s", 0.01)
        ref_len = max(0.1, float(self.resolution.get("settle_hint_s", 0.0) or 0.0))
        gap_pos = [(tq - t_last, pose_msg_to_matrix(m)[:3, 3]) for tq, m in gap_samples]
        # the reference is the settled pose at the END of the window (the last command's execution and the stream's
        # tracking error land in its first part); motion across the window is flagged (settling or an early drift)
        ref_pts = [q for tq, q in gap_pos if 0.5 * ref_len < tq <= ref_len] or [q for tq, q in gap_pos if tq <= ref_len] or ([gap_pos[0][1]] if gap_pos else [])
        early_pts = [q for tq, q in gap_pos if tq <= 0.25 * ref_len]
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
                if len(tt) >= 2 and tt.ptp() > 0:
                    speed = float(np.polyfit(tt, dd, 1)[0])
            if early_pts:
                ref_motion = float(np.linalg.norm(np.mean(early_pts, axis=0) - ref)) > thr
        n_gap = len(gap_samples)
        if not resp["responded"]:
            cls = "faulted" if state in ("FAULT", "DISABLED") else "rejected"
        elif (n_gap < 2 and gap_s >= 2 * fp) or (not after and gap_s >= ref_len + 0.1):
            cls = "not_observable"
        elif after and drift > self.hold_tol:
            cls = "drifted"
        else:
            cls = "held"
        self.a.servo_cp(base_T)
        self._executed(buf, base_T, timeout=max(0.5, self.response_timeout))
        return {"gap_requested_s": gap_s, "gap_s": actual_gap, "below_resolution": gap_s < self.resolution.get("resolution_floor_s", 0.0),
                "executed": ok, "responded": resp["responded"], "closest_approach_m": resp["closest_m"], "moved_m": resp["moved_m"],
                "initial_distance_m": resp["initial_distance_m"], "settled_distance_m": resp["settled_distance_m"], "reduction_m": resp["reduction_m"],
                "time_to_execute_s": t_exec, "drift_m": drift, "drift_onset_s": onset, "drift_speed_m_s": speed, "onset_threshold_m": thr,
                "reference_window_motion": ref_motion, "reference_window_s": ref_len,
                "samples_in_gap": n_gap, "class": cls, "state": state}

    @staticmethod
    def _tripped(r: dict) -> bool:
        return r["class"] in ("rejected", "faulted", "drifted")

    def probe_liveness(self, buf, base_T) -> dict:
        """0.1.2: the timeout is reported as an INTERVAL that contains it under a deterministic-timeout model.

        Every trial is evidence about tau_w:
          * a passing trial (pose held, next command acted on) with realised silence g says the policy had not
            fired, or had not yet become visible, by g:  tau_w >= g - L - G - t_detect;
          * a tripping trial says the policy had fired by the time the trip became visible, h (the realised gap
            for a rejection/fault, the drift onset for a drift):  tau_w <= h + L;
          * a drift onset additionally bounds tau_w from below: the departure needs thr / v to exceed the onset
            threshold plus one feedback period to be sampled:  tau_w >= onset - thr / v_min - fp - L.
        Allowances (all recorded in the estimate): L = the measured p95 response latency (an upper bound on the
        one-way transport latency the probe cannot observe); G = one feedback period, the assumed granularity at
        which the implementation evaluates its policy (a polled watchdog cannot be resolved below its poll
        period; the assumption is that it is evaluated at least once per feedback period); t_detect = 0 for a
        rejection/fault and hold_tol / v_min + fp for a drift (a passing drift trial only says the departure was
        still below the hold tolerance when the next command arrived), with v_min the smallest drift speed
        estimated over the drifting trials.  The interval is the intersection of these bounds over all trials;
        a lower bound above an upper bound means the deterministic model does not describe the implementation
        (status `inconsistent`, undetermined).  A trial with motion inside the reference window (settling, or a drift
        that began there) contributes no onset-based bound.  The per-run brackets are kept for the record."""
        r_floor = self.resolution.get("resolution_floor_s", 0.0)
        fp = float(self.resolution.get("feedback_period_s", 0.01) or 0.01)
        L_raw = self.resolution.get("response_latency_p95_s")
        L_known = L_raw is not None and math.isfinite(float(L_raw))
        L = float(L_raw) if L_known else 0.0
        G = fp
        out = {"gap_max_s": self.gap_max, "resolution_floor_s": r_floor, "response_timeout_s": self.response_timeout,
               "hold_tolerance_m": self.hold_tol, "latency_allowance_s": (L if L_known else None), "granularity_allowance_s": G, "trials": []}
        big = [self._gap_trial(buf, base_T, self.gap_max) for _ in range(self.trials)]
        out["trials"] += big
        n_trip = sum(1 for b in big if self._tripped(b))
        out["tripped_at_gap_max"] = rate_estimate(n_trip, len(big))
        drifts = [b["drift_m"] for b in big if not math.isnan(b["drift_m"])]
        out["drift_during_gap_max_m"] = estimate(drifts).to_dict() if drifts else None
        classes = [b["class"] for b in big]
        if n_trip == 0:
            out["stop_class"] = "not_observable" if all(c == "not_observable" for c in classes) else "held_through_range"
            out["tau_w_estimate_s"] = None
            out["finding"] = (f"held within {self.hold_tol*1e3:.2f} mm for silences up to {self.gap_max} s and responded afterwards (n={len(big)}); "
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
            r = self._gap_trial(buf, base_T, lo0)
            out["trials"].append(r)
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
                r = self._gap_trial(buf, base_T, mid)
                out["trials"].append(r)
                if self._tripped(r):
                    hi, hi_real = mid, min(hi_real, hi_of(r))
                else:
                    lo, lo_real = mid, max(lo_real, r["gap_s"])
            brackets.append([lo_real, hi_real])
        out["trials_below_resolution"] = below
        out["brackets_s"] = brackets
        if below > 0 and not brackets:
            out["tau_w_estimate_s"] = {"upper_bound_s": lo0 + L, "n": below, "status": "upper_bound", "latency_allowance_s": L}
            out["finding"] = f"{out['stop_class']} at every gap down to the resolution floor {lo0*1e3:.1f} ms: tau_w <= {(lo0 + L)*1e3:.1f} ms (below resolution, n={below})"
            return out
        # ---- the interval from all trials
        trials = out["trials"]
        passing = [t for t in trials if t["class"] == "held"]
        tripping = [t for t in trials if self._tripped(t)]
        drifted = [t for t in tripping if t["class"] == "drifted"]
        speeds = [t["drift_speed_m_s"] for t in drifted if t.get("drift_speed_m_s") is not None and t["drift_speed_m_s"] > 0]
        v_min = min(speeds) if speeds else None
        est = {"n": len(brackets), "brackets_s": brackets, "latency_allowance_s": (L if L_known else None), "granularity_allowance_s": G, "feedback_period_s": fp,
               "drift_speed_min_m_s": v_min, "detection_delay_s": 0.0,
               "interval_semantics": ("intersection over trials of [passing gap - L - G - t_detect, trip evidence + L] and, for drifts, "
                                      "[onset - thr / v_min - fp - L, onset + L]; contains tau_w under a deterministic timeout evaluated at least once per feedback period")}
        if not L_known:
            est.update(status="undetermined", reason="response latency not measured (no latency probe responded): the latency allowance of the interval is unknown")
            out["tau_w_estimate_s"] = est
            out["finding"] = f"{out['stop_class']} observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
            return out
        if out["stop_class"] == "drifted":
            if v_min is None:
                est.update(status="undetermined", reason="drift speed not estimable (too few samples after the onset)")
                out["tau_w_estimate_s"] = est
                out["finding"] = f"drifted observed but tau_w undetermined ({est['reason']}; n={len(brackets)})"
                return out
            est["detection_delay_s"] = self.hold_tol / v_min + fp
        lows = [t["gap_s"] - L - G - est["detection_delay_s"] for t in passing]
        highs = [hi_of(t) + L for t in tripping]
        onset_lows = []
        for t in drifted:
            if t.get("drift_onset_s") is not None and not t.get("reference_window_motion") and v_min is not None and t.get("onset_threshold_m") is not None:
                onset_lows.append(t["drift_onset_s"] - t["onset_threshold_m"] / v_min - fp - L)
        est["onset_based_lower_bounds_s"] = onset_lows
        est["reference_window_motion_trials"] = sum(1 for t in drifted if t.get("reference_window_motion"))
        lo_all = max(lows + onset_lows) if (lows or onset_lows) else 0.0
        hi_all = min(highs)
        est.update(interval_low_s=max(0.0, lo_all), interval_high_s=hi_all, point_s=0.5 * (max(0.0, lo_all) + hi_all),
                   bracket_width_median_s=float(np.median([b[1] - b[0] for b in brackets])))
        if lo_all > hi_all:
            est.update(status="inconsistent", reason=f"lower bound {lo_all:.4f} s above upper bound {hi_all:.4f} s: the deterministic-timeout model does not describe the observations")
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
                          f"(midpoint {est['point_s']:.3f} s, n={len(brackets)}, allowances L {L*1e3:.0f} ms, G {G*1e3:.0f} ms, t_detect {est['detection_delay_s']*1e3:.0f} ms)")
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
                               "acceptance": AcceptanceEstimate(channel="setpoint_cp" if sp_buf is not None else "none", commands_sent=0, accepted=0, accepted_fraction=0.0,
                                                                first_acceptance_delay_s=float("nan"), max_stale_s=None, mean_acceptance_rate_hz=0.0, channel_period_s=None,
                                                                client_rate_achieved_hz=0.0, status="undetermined", reason="targets_not_separable").to_dict()}
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
            row["zoh_error_bound_m"] = None  # 0.1.2: set below from the longest stale interval of accepted commands (eq. 9 needs the max gap, not a mean rate)
            # 0.1.2: the feedback count above is a DIAGNOSTIC (target crossings); the verdict uses the accepted-command
            # channel setpoint_cp when the implementation exposes one, and is undetermined otherwise
            row["feedback_target_crossings_hz"] = est.observable_rate_hz
            row["feedback_count_is_evidence_of_execution"] = False
            acc = None
            if sp_buf is not None:
                sps = sp_buf.since(t_start)
                SP = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in sps]) if sps else np.zeros((0, 3))
                acc = estimate_acceptance(targets, SP, np.array([t for t, _ in sps]), np.array(send_times), delta, t_end)
            else:
                acc = AcceptanceEstimate("none", int(len(send_times)), 0, 0.0, float("nan"), float("nan"), 0.0, float("nan"),
                                         row["client_rate_achieved_hz"], "undetermined", "no_accepted_command_channel: the interface publishes no setpoint_cp")
            row["acceptance"] = acc.to_dict()
            if acc.status == "ok" and acc.accepted > 0 and not math.isnan(acc.max_stale_s):
                row["zoh_error_bound_m"] = self.tol.speed_m_s * acc.max_stale_s
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
        horizon = te.horizon_s if te.horizon_s is not None else self.gap_max
        res.estimates["stop_horizon_s"] = horizon
        tested_to = self.gap_max
        if te.stop_behaviour != "any":
            if horizon > tested_to + 1e-9:
                sub["stop_behaviour"] = "undetermined"
                notes.append(f"the client's stop expectation is claimed up to {horizon} s of silence but the probe tested silences only up to {tested_to} s")
            elif stop_class in (None, "not_observable"):
                sub["stop_behaviour"] = "undetermined"
            elif te.stop_behaviour == "release":
                sub["stop_behaviour"] = "undetermined"
                notes.append(f"a 'release' (actuation released) expectation cannot be decided from the pose alone; observed stop behaviour: {stop_class}")
            elif te.stop_behaviour == "hold":
                if stop_class == "held_through_range":
                    sub["stop_behaviour"] = "satisfied"
                    notes.append(f"held within {self.hold_tol*1e3:.2f} mm for silences up to {tested_to} s (claimed horizon {horizon} s) and responded afterwards")
                else:
                    # a trip observed: violated if the policy is evidenced within the horizon (trip evidence time
                    # + L is an upper bound of tau_w); satisfied only if the interval's lower end is beyond it
                    def _hi(t):
                        return min(t["gap_s"], t["drift_onset_s"]) if t["class"] == "drifted" and t.get("drift_onset_s") is not None else t["gap_s"]
                    trips = [t for t in B.get("trials", []) if self._tripped(t)]
                    L_ = float(B.get("latency_allowance_s") or 0.0)
                    trip_hi = min((_hi(t) + L_ for t in trips), default=None)
                    tau_lo = tau.get("interval_low_s") if isinstance(tau, dict) and tau.get("status") == "ok" else None
                    if trip_hi is not None and trip_hi <= horizon + 1e-9:
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
                    ok_margin = True
                    if isinstance(tau, dict) and tau.get("status") == "ok":
                        # eq. (7) against the INTERVAL: satisfied only if the client's period + jitter is below the
                        # lower end (which contains tau_w); violated only if it is above the upper end
                        need = 1.0 / self.tol.client_rate_hz + self.tol.jitter_max_s
                        if need < tau["interval_low_s"]:
                            ok_margin = True
                        elif need >= tau["interval_high_s"]:
                            ok_margin = False
                            notes.append(f"eq. (7) violated: client period + J_max = {need:.3f} s >= tau_w interval high {tau['interval_high_s']:.3f} s")
                        else:
                            ok_margin = None
                            notes.append(f"eq. (7) undecided: client period + J_max = {need:.3f} s lies inside the tau_w interval [{tau['interval_low_s']:.3f}, {tau['interval_high_s']:.3f}] s")
                    elif isinstance(tau, dict) and tau.get("status") == "upper_bound":
                        need = 1.0 / self.tol.client_rate_hz + self.tol.jitter_max_s
                        ok_margin = False if need >= tau["upper_bound_s"] else None
                    elif isinstance(tau, dict):
                        ok_margin = None
                    sub["stop_behaviour"] = "satisfied" if ok_margin is True else ("violated" if ok_margin is False else "undetermined")
                elif stop_class == "held_through_range":
                    sub["stop_behaviour"] = "violated"
                    notes.append(f"client expects a {te.stop_behaviour} stop policy within {horizon} s of silence; the pose was held and commands acted on up to {tested_to} s")
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
            res.estimates["zoh_error_at_client_rate_m"] = (self.tol.speed_m_s * acc0["max_stale_s"]) if acc0.get("status") == "ok" and acc0.get("max_stale_s") is not None and not (isinstance(acc0.get("max_stale_s"), float) and math.isnan(acc0["max_stale_s"])) else None
        if te.rate == "required":
            acc = None
            if at_client is not None and at_client.get("acceptance"):
                acc = AcceptanceEstimate(**at_client["acceptance"])
            sub["rate"] = rate_subverdict(acc, f_req)
            if acc is not None:
                res.estimates["accepted_commands_at_client_rate"] = acc.to_dict()
            if sub["rate"] == "violated":
                notes.append(f"longest stale interval between accepted commands {acc.max_stale_s*1e3:.0f} ms > required period {1e3/f_req:.0f} ms (eq. 9; setpoint_cp channel, {acc.accepted} of {acc.commands_sent} commands accepted) while the client sustained {acc.client_rate_achieved_hz:.0f} Hz")
            elif sub["rate"] == "satisfied":
                notes.append(f"longest stale interval between accepted commands {acc.max_stale_s*1e3:.1f} ms <= required period {1e3/f_req:.0f} ms (setpoint_cp channel, {acc.accepted} of {acc.commands_sent} accepted)")
            elif sub["rate"] == "undetermined" and at_client is not None:
                why = (acc.reason if acc is not None and acc.reason else None) or at_client.get("reason") or "client rate below the required rate: the client, not the implementation, is the limit"
                notes.append("rate expectation cannot be decided: " + why)
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
