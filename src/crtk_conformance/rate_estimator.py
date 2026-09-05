"""Noise-robust observable command/state rate estimator (0.1.1; design in rc3/RATE_ESTIMATOR_DESIGN.md).

The probe commands a known sequence of target positions g_1 .. g_n.  Every feedback sample is classified
to the nearest commanded target within a matching tolerance delta (user-supplied, or estimated from the
resting noise of the feedback channel); samples farther than delta from every target are *unmatched*.
The matched index sequence is scanned in receive order and a transition is counted each time the index
rises above every index seen before (monotone progress).  Consequences:

  * transitions <= n always (indices come from the commanded set, progress is monotone);
  * measurement noise below delta creates no transition; noise near delta makes samples unmatched;
  * a controller that interpolates between targets does not inflate the count;
  * a large unmatched fraction yields `undetermined` rather than a number.

This module is pure Python/numpy and is unit-tested with synthetic feedback (tests/test_rate_estimator.py).

0.1.2 (RC3 adversarial review, finding 2): the feedback-based count is a DIAGNOSTIC -- the *feedback
target-crossing rate* -- and is never the basis of a verdict.  A controller moving continuously towards one
late command crosses every earlier target, so crossings do not identify accepted commands.  The verdict uses
the accepted-command channel `setpoint_cp` (AcceptanceEstimate) and the longest stale interval between
acceptances, which is the quantity the zero-order-hold bound (eq. 9) needs; an interface without
`setpoint_cp` gets no rate verdict (undetermined) because the outside observer cannot tell.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Sequence

import numpy as np

UNMATCHED_UNDETERMINED_FRACTION = 0.5  # more than half the samples unmatched -> undetermined
BOUND_FRACTION = 0.9  # observable rate >= 0.9 * min(client, publish) -> the observation is channel-bounded


@dataclass
class RateEstimate:
    command_rate_requested_hz: float
    commands_sent: int
    client_rate_achieved_hz: float
    window_s: float  # rate window: first command -> last observed transition (>= command span + one period)
    publish_rate_hz: float
    samples_total: int
    samples_matched: int
    samples_unmatched: int
    unmatched_fraction: float
    targets_reached: int
    transitions: int
    observable_rate_hz: float
    match_tolerance_m: float
    match_tolerance_source: str
    target_spacing_m: float
    observation_bounded_by: str  # 'client' | 'publish' | 'none'
    status: str  # 'ok' | 'undetermined'
    reason: str = ""

    def to_dict(self) -> Dict:
        return asdict(self)


def estimate_noise_sigma(positions: np.ndarray) -> float:
    """Robust per-axis noise standard deviation of a resting feedback stream (MAD * 1.4826, max over axes)."""
    P = np.asarray(positions, dtype=float)
    if P.ndim != 2 or P.shape[0] < 3:
        return 0.0
    med = np.median(P, axis=0)
    mad = np.median(np.abs(P - med), axis=0)
    return float(np.max(mad) * 1.4826)


def match_tolerance_from_sigma(sigma: float, k: float = 5.0, floor: float = 1e-6) -> float:
    """delta = max(k sigma, floor); k = 5 gives a Gaussian false-unmatch probability below 6e-7 per sample."""
    return max(k * sigma, floor)


def classify_samples(samples: np.ndarray, targets: np.ndarray, delta: float) -> np.ndarray:
    """Nearest-target index per sample, or -1 when farther than delta from every target."""
    S = np.asarray(samples, dtype=float)
    G = np.asarray(targets, dtype=float)
    if S.size == 0:
        return np.zeros(0, dtype=int)
    d = np.linalg.norm(S[:, None, :] - G[None, :, :], axis=2)  # (n_samples, n_targets)
    idx = np.argmin(d, axis=1)
    best = d[np.arange(S.shape[0]), idx]
    idx = idx.astype(int)
    idx[best > delta] = -1
    return idx


def count_transitions(indices: Sequence[int]) -> Dict[str, int]:
    """Monotone target transitions and distinct targets reached from a matched index sequence.

    A transition is counted every time the matched index exceeds every index seen before, *including*
    the first matched target: the probe starts away from every target, so reaching target 0 is itself an
    executed command.  Hence transitions <= number of targets.
    """
    seen_max = -1
    transitions = 0
    last_i = -1
    reached = set()
    for i, k in enumerate(indices):
        if k < 0:
            continue
        reached.add(int(k))
        if k > seen_max:
            transitions += 1
            seen_max = k
            last_i = i
    return {"transitions": transitions, "targets_reached": len(reached), "last_transition_sample": last_i}


def estimate_rate(
    targets: np.ndarray,
    sample_positions: np.ndarray,
    sample_times: np.ndarray,
    send_times: np.ndarray,
    command_rate_requested_hz: float,
    delta: float,
    delta_source: str,
    window_start: float,
    window_end: float,
) -> RateEstimate:
    """Estimate the observable command/state rate for one command window.

    targets: (n, 3) commanded positions in the order sent; sample_positions/times: feedback samples received
    from window_start to window_end (times on the same monotonic clock as send_times); send_times: the
    probe's own publication times.  The probe starts away from every target, so the first target reached
    counts as one transition.
    """
    G = np.asarray(targets, dtype=float)
    n = int(G.shape[0])
    S = np.asarray(sample_positions, dtype=float).reshape(-1, 3)
    st = np.asarray(sample_times, dtype=float)
    send = np.asarray(send_times, dtype=float)
    obs_window = max(window_end - window_start, 1e-9)  # observation window (commands + tail)
    achieved = (len(send) - 1) / (send[-1] - send[0]) if len(send) > 1 and send[-1] > send[0] else float("nan")
    publish = S.shape[0] / obs_window if S.shape[0] else 0.0
    # rate window: from the first command to the last observed transition, never shorter than the command
    # span plus one command period (so that n instantaneous executions at rate f give exactly f)
    period = (send[-1] - send[0]) / (len(send) - 1) if len(send) > 1 and send[-1] > send[0] else 1.0 / max(command_rate_requested_hz, 1e-9)
    window = (send[-1] - send[0]) + period if len(send) else obs_window
    spacing = float(np.min(np.linalg.norm(np.diff(G, axis=0), axis=1))) if n > 1 else float("inf")
    if spacing <= 2.0 * delta:
        return RateEstimate(command_rate_requested_hz, n, achieved, obs_window, publish, int(S.shape[0]), 0, int(S.shape[0]), 1.0, 0, 0, 0.0,
                            delta, delta_source, spacing, "none", "undetermined", "targets_not_separable: target spacing <= 2 * match tolerance")
    if S.shape[0] == 0:
        return RateEstimate(command_rate_requested_hz, n, achieved, obs_window, 0.0, 0, 0, 0, 1.0, 0, 0, 0.0,
                            delta, delta_source, spacing, "none", "undetermined", "no_feedback")
    idx = classify_samples(S, G, delta)
    matched = int(np.sum(idx >= 0))
    unmatched = int(S.shape[0] - matched)
    frac = unmatched / S.shape[0]
    c = count_transitions(list(idx))
    transitions, reached = c["transitions"], c["targets_reached"]
    transitions = min(transitions, n)
    if c["last_transition_sample"] >= 0 and len(send):
        window = max(window, float(st[c["last_transition_sample"]] - send[0]))
    rate = transitions / max(window, 1e-9)
    bounded = "none"
    ref = min(achieved if not math.isnan(achieved) else float("inf"), publish)
    if ref != float("inf") and rate >= BOUND_FRACTION * ref:
        bounded = "client" if (not math.isnan(achieved) and achieved <= publish) else "publish"
    status, reason = "ok", ""
    if frac > UNMATCHED_UNDETERMINED_FRACTION:
        status, reason = "undetermined", f"unmatched_fraction {frac:.2f} > {UNMATCHED_UNDETERMINED_FRACTION}: feedback does not settle on the commanded targets within the match tolerance"
    return RateEstimate(command_rate_requested_hz, n, achieved, window, publish, int(S.shape[0]), matched, unmatched, frac,
                        reached, transitions, rate, delta, delta_source, spacing, bounded, status, reason)


@dataclass
class AcceptanceEstimate:
    """0.1.2: what the ACCEPTED-command channel (setpoint_cp) shows for one command window.

    The feedback-based RateEstimate counts target crossings of measured_cp; a controller that moves continuously
    to a late command crosses the earlier targets without having accepted them (RC3 adversarial review,
    finding 2), so crossings are not evidence of execution.  setpoint_cp reports the goal the implementation
    is currently tracking, so a change of setpoint_cp to a commanded target is an acceptance event.  The
    quantity that bounds the zero-order-hold error is not the mean rate but the LONGEST STALE INTERVAL
    between acceptances (including the interval from the first command to the first acceptance), which is
    what the verdict uses: v * max_stale_s <= epsilon  <=>  max_stale_s <= 1 / f_required.
    """
    channel: str  # 'setpoint_cp' | 'none'
    commands_sent: int
    accepted: int  # distinct commanded targets that appeared on the channel, in order
    accepted_fraction: float
    first_acceptance_delay_s: float  # first command sent -> first acceptance seen
    max_stale_s: float  # longest interval between consecutive acceptance events (incl. the first delay)
    mean_acceptance_rate_hz: float
    channel_period_s: float  # publication period of the channel: acceptance times are quantised to it
    client_rate_achieved_hz: float
    status: str  # 'ok' | 'undetermined'
    reason: str = ""
    channel_unmatched_fraction: float = 0.0  # setpoint samples matching no commanded target: a low-level interpolator would show many

    def to_dict(self) -> Dict:
        return asdict(self)


def estimate_acceptance(targets: np.ndarray, setpoint_positions: np.ndarray, setpoint_times: np.ndarray, send_times: np.ndarray,
                        delta: float, window_end: float) -> AcceptanceEstimate:
    """Acceptance events from the setpoint_cp channel: the first sample at which the channel reports each
    commanded target (within delta; targets are separated by > 2 delta by construction of the probe)."""
    G = np.asarray(targets, dtype=float)
    n = int(G.shape[0])
    S = np.asarray(setpoint_positions, dtype=float).reshape(-1, 3)
    st = np.asarray(setpoint_times, dtype=float)
    send = np.asarray(send_times, dtype=float)
    achieved = (len(send) - 1) / (send[-1] - send[0]) if len(send) > 1 and send[-1] > send[0] else float("nan")
    if S.shape[0] < 2:
        return AcceptanceEstimate("setpoint_cp", n, 0, 0.0, float("nan"), float("nan"), 0.0, float("nan"), achieved, "undetermined", "no_setpoint_samples")
    period = float(np.median(np.diff(st))) if S.shape[0] > 1 else float("nan")
    idx = classify_samples(S, G, max(delta, 1e-9))
    # the channel must be piecewise constant on the commanded targets (CRTK: the current setpoint to the low-level
    # controller); a channel that reports intermediate values is an interpolating low-level controller, on which
    # an acceptance cannot be told from a pass-through, exactly as for feedback crossings -> undetermined
    in_window = st >= float(send[0]) if len(send) else np.ones(len(st), dtype=bool)
    unmatched = float(np.mean(idx[in_window] < 0)) if in_window.any() else 0.0
    # acceptance event = first sample whose classification moves to a new target index (monotone, as sent)
    events = []
    last = -1
    for k, j in enumerate(idx):
        if j >= 0 and j > last:
            events.append((float(st[k]), int(j)))
            last = int(j)
    accepted = len(events)
    if unmatched > UNMATCHED_UNDETERMINED_FRACTION:
        return AcceptanceEstimate("setpoint_cp", n, accepted, accepted / n, float("nan"), float("nan"), 0.0, period, achieved, "undetermined",
                                  "setpoint_cp is not piecewise constant on the commanded targets (%.0f%% of samples match none): an interpolating low-level controller cannot be told from a partial acceptor" % (100 * unmatched), unmatched)
    if accepted == 0:
        return AcceptanceEstimate("setpoint_cp", n, 0, 0.0, float("nan"), float(window_end - send[0]) if len(send) else float("nan"), 0.0, period, achieved,
                                  "ok", "no commanded target was ever reported on setpoint_cp", unmatched)
    times = [t for t, _ in events]
    gaps = [times[0] - float(send[0])] + [b - a for a, b in zip(times[:-1], times[1:])]
    # commands not accepted after the last acceptance leave the channel stale until the last command was sent: that
    # tail counts (up to the last send time, not to the end of the observation window, which includes the probe's
    # own settling wait after the last command)
    tail = float(send[-1] - times[-1]) if len(send) else 0.0
    max_stale = max(gaps + ([tail] if (accepted < n and tail > 0) else []))
    span = times[-1] - float(send[0])
    rate = accepted / span if span > 0 else float("nan")
    return AcceptanceEstimate("setpoint_cp", n, accepted, accepted / n, gaps[0], float(max_stale), float(rate) if not math.isnan(rate) else 0.0, period, achieved, "ok", "", unmatched)


def rate_subverdict(acc: Optional[AcceptanceEstimate], f_required_hz: float) -> str:
    """'satisfied' | 'violated' | 'undetermined' for the rate part of a declared rate expectation (0.1.2).

    Decided on the accepted-command channel only.  satisfied: the longest stale interval is within the required
    period 1/f_req (allowing one channel publication period of quantisation).  violated: the longest stale
    interval exceeds the period by more than the quantisation AND the client itself sustained the required rate
    (otherwise the client, not the implementation, is the limit).  Without a setpoint channel: undetermined
    (feedback target crossings are reported as a diagnostic, never as a verdict)."""
    if acc is None or acc.channel != "setpoint_cp" or acc.status != "ok":
        return "undetermined"
    req_period = 1.0 / f_required_hz
    q = acc.channel_period_s if not math.isnan(acc.channel_period_s) else 0.0
    if acc.accepted == 0:
        return "violated" if (not math.isnan(acc.client_rate_achieved_hz) and acc.client_rate_achieved_hz >= f_required_hz) else "undetermined"
    if acc.max_stale_s <= req_period + q:
        return "satisfied"
    if not math.isnan(acc.client_rate_achieved_hz) and acc.client_rate_achieved_hz >= f_required_hz:
        return "violated"
    return "undetermined"


def crossing_subverdict_legacy(est: Optional[RateEstimate], f_required_hz: float) -> str:
    """The 0.1.1 rule, kept ONLY so that the archived v0.1.1 reports can be re-read; it credited feedback target
    crossings as executed commands and is not used for any verdict in 0.1.2."""
    if est is None or est.status != "ok":
        return "undetermined"
    if est.observable_rate_hz >= f_required_hz:
        return "satisfied"
    return "violated" if est.observation_bounded_by == "none" else "undetermined"
