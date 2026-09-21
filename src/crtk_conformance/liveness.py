"""Liveness interval logic that needs no ROS (0.1.5): the departure-guarded response latency of the last streamed
command and the deterministic-timeout enclosure formed from the gap trials.  probes/rate.py collects the observations;
these two functions decide, so that the decision can be replayed offline on archived trial records
(validation/reanalyze_liveness_v015.py, tests/test_liveness_v015.py).
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def last_command_latency(samples, t_last: float, p_last: np.ndarray, band: float) -> Tuple[float, bool]:
    """Response latency of the last streamed command (the return to base after the half-step offset), 0.1.5.

    samples: (t_mono, position) pairs observed from the half-step send up to the post-gap send (a sample after the
    post-gap command was sent cannot be attributed to the last streamed command); t_last: send time of the
    return-to-base command; p_last: the base position; band: the in-band radius (0.25 x step + 3 sigma_hat).

    0.1.3 took the first sample after t_last that lay within the band as the response.  That is only the response when
    the implementation applies commands within one feedback period: an implementation that applies late is still
    executing the earlier (in-band) stream when the first gap sample arrives, so the "response" was read before the
    last command had been applied (RC7 adversarial review, finding F1: 0.2-9.7 ms against ~310 ms responses in the
    archived 300-ms delay case).  0.1.5 accepts a sample as the response only if a sample OUTSIDE the band -- the
    departure to the half-step offset -- was observed before it: the pose leaves the band only once the half-step is
    applied and returns only once the last command is.  Returns (latency after t_last, departure_observed); the
    latency is NaN when no departure was observed, and the caller then falls back to the latency allowance L."""
    departed = False
    for tq, q in samples:
        outside = float(np.linalg.norm(np.asarray(q, dtype=float) - p_last)) > band
        if outside:
            departed = True
        elif departed and tq >= t_last:
            return float(tq - t_last), True
    return float("nan"), departed


def drift_evaluated(trial) -> bool:
    """Whether the trial's silence was long enough for the drift to be evaluated (feedback samples after the settled
    reference window).  0.1.5 records the flag; an archived record without it is judged by its gap against its
    reference window.  A `held` trial whose drift was never evaluated is not a passing trial for a drift policy: the
    post-gap command is acted on after a release, so the response alone does not show that the policy had not fired."""
    if "drift_evaluated" in trial:
        return bool(trial["drift_evaluated"])
    return float(trial.get("gap_s", 0.0)) > float(trial.get("reference_window_s", 0.0) or 0.0)


def attributable(trial, baseline_loss: int) -> bool:
    """Whether a tripped trial's evidence can be attributed to a silence-triggered stop policy (0.1.5, RC7 review,
    finding F10).  Without baseline command loss every tripped trial is.  With it, a non-response without a visible
    state change (`rejected`) may be the loss of the post-gap command itself and is not; a `faulted` trial is, when the
    time of the FAULT observation was recorded (the fault is seen in the operating state, which loss cannot forge);
    a `drifted` trial is (the drift is seen in the pose)."""
    if trial["class"] not in ("rejected", "faulted", "drifted"):
        return False
    if baseline_loss <= 0:
        return True
    if trial["class"] == "rejected":
        return False
    if trial["class"] == "faulted":
        return trial.get("state_observed_at_s") is not None
    return True


def timeout_interval_from_trials(trials, L: float, G: float, fp: float, hold_tol: float, stop_class: str, baseline_loss: int = 0):
    """The deterministic-timeout enclosure from the gap trials (0.1.5; the interval logic of probe_liveness, kept as a
    pure function so that it can be replayed offline on archived trial records).

    trials: the per-trial dicts of probe_liveness (class, gap_s, last_stream_latency_s, drift_onset_s, drift_speed_m_s,
    onset_threshold_m, reference_window_motion, state_observed_at_s); L: latency allowance for transactions that drew
    no response (a client-supplied bound, or the run maximum -- conditional); G: policy-evaluation granularity; fp:
    feedback period; hold_tol: the hold tolerance (drift detection allowance); baseline_loss: the number of calibration
    commands (sent without any preceding silence) that drew no response.  Returns a dict with interval_low_s /
    interval_high_s / point_s and the bounds it was formed from, or status 'inconsistent'.

    Bounds (all times from the probe's own sends; l1 = one-way transport latency of the last streamed command,
    bounded by that command's response latency r_last when the departure guard observed it, else by L):
      passing trial (held, next command acted on), silence g:   tau_w > g - l1 - G - t_det
      rejection / fault at silence g:                            tau_w <= g + L
      drift with onset o:                                        tau_w <= o
                                                                  tau_w >= o - l1 - G - thr / v_min - fp - L
    0.1.5: the onset lower bound now carries the granularity term G that the passing-trial bound already carried
    (RC7 review, finding F8); the policy may fire up to G after the timeout elapsed, the drift then needs thr / v_min
    to exceed the onset threshold, its sample is published within fp and received after the feedback transport.
    Under baseline command loss (finding F10) a `rejected` trial is unattributable and contributes no bound, and a
    `faulted` trial bounds tau_w by the time its FAULT state was observed (the post-gap command may have been lost, so
    the policy may have fired during the response wait): tau_w <= t_state - l1 <= t_state.
    For a drift policy a `held` trial is a passing trial only if its drift was evaluated (drift_evaluated), and an
    onset supplies a lower bound only if the reference window before it was still (reference_window_still): a drift
    that began inside the reference window is censored, its onset is then the first observable sample and bounds
    tau_w from above only (v0.1.5 verification campaign, delayed-application drift case)."""
    passing = [t for t in trials if t["class"] == "held" and (stop_class != "drifted" or drift_evaluated(t))]
    tripping = [t for t in trials if attributable(t, baseline_loss)]
    unattributable = [t for t in trials if t["class"] in ("rejected", "faulted", "drifted") and not attributable(t, baseline_loss)]
    drifted = [t for t in tripping if t["class"] == "drifted"]
    speeds = [t["drift_speed_m_s"] for t in drifted if t.get("drift_speed_m_s") is not None and t["drift_speed_m_s"] > 0]
    v_min = min(speeds) if speeds else None
    t_det = 0.0
    if stop_class == "drifted" and v_min is not None:
        t_det = hold_tol / v_min + fp

    def l1_of(t):
        r = t.get("last_stream_latency_s")
        return float(r) if (r is not None and not (isinstance(r, float) and math.isnan(r))) else L

    def hi_of(t):
        if t["class"] == "drifted" and t.get("drift_onset_s") is not None:
            return min(t["gap_s"], t["drift_onset_s"])
        return t["gap_s"]

    def high_of(t):
        if t["class"] == "drifted":
            return hi_of(t)
        if t["class"] == "faulted" and baseline_loss > 0:
            return float(t["state_observed_at_s"])
        return hi_of(t) + L

    lows = [t["gap_s"] - l1_of(t) - G - t_det for t in passing]
    highs = [high_of(t) for t in tripping]
    onset_lows = []
    for t in drifted:
        if t.get("drift_onset_s") is not None and not t.get("reference_window_motion") and t.get("reference_window_still", True) and v_min is not None and t.get("onset_threshold_m") is not None:
            onset_lows.append(t["drift_onset_s"] - l1_of(t) - G - t["onset_threshold_m"] / v_min - fp - L)
    lo_all = max(lows + onset_lows) if (lows or onset_lows) else 0.0
    hi_all = min(highs) if highs else float("inf")
    est = {"drift_speed_min_m_s": v_min, "detection_delay_s": t_det, "passing_lower_bounds_s": lows, "onset_based_lower_bounds_s": onset_lows,
           "tripping_upper_bounds_s": highs, "n_trials_with_own_last_latency": sum(1 for t in passing + drifted if t.get("last_stream_latency_s") is not None and not (isinstance(t.get("last_stream_latency_s"), float) and math.isnan(t["last_stream_latency_s"]))),
           "reference_window_motion_trials": sum(1 for t in drifted if t.get("reference_window_motion")),
           "unattributable_trials": len(unattributable), "attributable_tripping_trials": len(tripping),
           "interval_low_s": max(0.0, lo_all), "interval_high_s": hi_all, "point_s": 0.5 * (max(0.0, lo_all) + hi_all) if math.isfinite(hi_all) else float("nan"),
           "lower_bound_uses_L_for_all_trials": all(l1_of(t) == L for t in passing + drifted) if (passing or drifted) else None,
           "status": "inconsistent" if lo_all > hi_all else ("no_attributable_trip" if not tripping else "formed")}
    return est
