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
The quantity estimated is the *observable effective command/state rate* through the feedback channel; it
is bounded by the client's achieved rate, the execution rate and the feedback publish rate, and it does not
identify the internal controller rate.
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


def rate_subverdict(est: Optional[RateEstimate], f_required_hz: float) -> str:
    """'satisfied' | 'violated' | 'undetermined' for the rate part of a declared rate expectation."""
    if est is None or est.status != "ok":
        return "undetermined"
    if est.observable_rate_hz >= f_required_hz:
        return "satisfied"  # meeting f_req is sufficient even when the channel bounds the observation
    # below the required rate: only decide 'violated' when the channel is not the limit
    return "violated" if est.observation_bounded_by == "none" else "undetermined"
