"""Noise-robust observable command/state rate estimator (0.1.1; design in rc3/RATE_ESTIMATOR_DESIGN.md) and, since 0.1.3, the
applied-setpoint source-age estimator on which the rate verdict rests (AppliedAgeEstimate).

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
the applied-setpoint channel `setpoint_cp`.  0.1.2 decided on the longest interval between changes of that
channel; 0.1.3 (RC4 adversarial review, findings 1-3) decides on the SOURCE AGE of the applied setpoint
(AppliedAgeEstimate), bracketed between channel samples with causal target matching, which is the quantity the
zero-order-hold bound (eq. 9) needs; an interface without `setpoint_cp` gets no rate verdict (undetermined)
because the outside observer cannot tell.
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
class AppliedAgeEstimate:
    """0.1.3: what the APPLIED-setpoint channel (setpoint_cp, CRTK: 'current setpoint to low-level controller') shows
    for one command window, expressed as the SOURCE AGE of the applied setpoint.

    0.1.2 decided on the longest interval between changes of the channel; the RC4 adversarial review (finding 1)
    showed that a stream can change regularly while applying ever older commands (an ordered queue at half the
    client rate), so the update cadence does not bound the trajectory lag.  What bounds it is the age of the
    applied target: at time t the low-level controller holds the target of command j(t), sent at s_{j(t)}; for a
    trajectory of speed <= v the intended position has moved by at most v (t - s_{j(t)}) since, so
    e_ZOH(t) <= v * a(t) with a(t) = t - s_{j(t)} (eq. 9 with the source age in place of the period).

    The channel is observed at sample times t_k with causally matched targets j_k (a sample may match target j
    only if s_j <= t_k; finding 3).  Between samples the applied target is unobserved, so the age is bracketed:
        age_lower = max_k (t_k - s_{j_k})           (the age at the sample itself)
        age_upper = max_k (t_{k+1} - s_{j_k})       (the target may have been held until the next sample)
    with the window [s_0, s_{n-1}] and the pre-window channel state (a setpoint left over from before the first
    command) counted as 'nothing of this window applied yet', i.e. an age of t - s_0.  The verdict (finding 2)
    passes only when age_upper meets the required period and fails only when age_lower exceeds it; publication
    uncertainty widens the bracket, it is never added to the allowed period.

    The sample times t_k are the probe's RECEIPT times.  A sample received at t_k was published at t_k - d_k with
    d_k >= 0 the feedback transport delay, and target j_k was applied at that publication time; the age at the
    sample therefore overstates the true age of the applied setpoint by up to d_k, so the lower bound subtracts
    a feedback-latency allowance (the probe's latency allowance L, see probe_liveness; 0 when none is known, in
    which case age_lower is the age at receipt).  The upper bound needs no allowance (d_k >= 0 only makes the
    hold shorter than the receipt-time interval).
    """
    channel: str  # 'setpoint_cp' | 'none'
    commands_sent: int
    accepted: int  # distinct commanded targets that appeared on the channel causally, counted in order of first appearance
    accepted_fraction: float
    out_of_order_events: int  # channel returned to an older target after a newer one had appeared
    first_application_delay_s: float  # first command sent -> first sample showing a target of this window
    age_lower_s: float  # largest observed source age of the applied setpoint
    age_upper_s: float  # largest source age the applied setpoint may have reached between observations
    max_update_gap_s: float  # diagnostic: longest interval between changes of the channel (the 0.1.2 quantity)
    channel_period_s: float  # median publication period of the channel (diagnostic; the bracket uses actual sample times)
    samples_in_window: int
    client_rate_achieved_hz: float
    client_max_send_gap_s: float  # longest interval between the client's own consecutive sends: a client stall is not the implementation's
    status: str  # 'ok' | 'undetermined'
    reason: str = ""
    unmatched_after_first_fraction: float = 0.0  # samples matching no sent target after the first application: an interpolating low-level setpoint
    age_lower_at_receipt_s: float = float("nan")  # the lower bound before the feedback-transport allowance (age at the probe's receipt of the sample)
    feedback_latency_allowance_s: float = 0.0  # allowance subtracted from age_lower: a sample received at t_k was published at t_k - (transport delay)
    window_start_s: float = float("nan")  # w0: the first send (monotonic clock of the probe)
    window_end_s: float = float("nan")  # w1: the first sample showing the last target applied, else the observation end

    def to_dict(self) -> Dict:
        return asdict(self)


def _causal_match(sample: np.ndarray, targets: np.ndarray, send: np.ndarray, t: float, delta: float) -> int:
    """Index of the nearest commanded target within delta among the targets already sent at time t; -1 if none."""
    sent = np.nonzero(send <= t + 1e-12)[0]
    if len(sent) == 0:
        return -1
    d = np.linalg.norm(targets[sent] - sample, axis=1)
    k = int(np.argmin(d))
    return int(sent[k]) if d[k] <= delta else -1


def estimate_applied_age(targets: np.ndarray, setpoint_positions: np.ndarray, setpoint_times: np.ndarray, send_times: np.ndarray,
                         delta: float, window_end: Optional[float] = None, feedback_latency_allowance_s: float = 0.0) -> AppliedAgeEstimate:
    """Source-age bracket of the applied setpoint over the command window.

    The window starts at the first send and ends when the channel first shows the LAST target applied (the client's
    final intent is met) or, if it never does, at window_end (the end of the observation; default: the last channel
    sample).  setpoint samples may start before the window (the pre-window state is used to bracket the age up to
    the first in-window sample)."""
    G = np.asarray(targets, dtype=float)
    n = int(G.shape[0])
    S = np.asarray(setpoint_positions, dtype=float).reshape(-1, 3)
    st = np.asarray(setpoint_times, dtype=float)
    send = np.asarray(send_times, dtype=float)
    achieved = (len(send) - 1) / (send[-1] - send[0]) if len(send) > 1 and send[-1] > send[0] else float("nan")
    send_gap = float(np.max(np.diff(send))) if len(send) > 1 else float("nan")
    if n == 0 or len(send) == 0:
        return AppliedAgeEstimate("setpoint_cp", n, 0, 0.0, 0, float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), 0, achieved, send_gap, "undetermined", "no_commands")
    order = np.argsort(st)
    st, S = st[order], S[order]
    w0 = float(send[0])
    w_obs = float(window_end) if window_end is not None else (float(st[-1]) if len(st) else float(send[-1]))
    period = float(np.median(np.diff(st))) if len(st) > 1 else float("nan")
    # window end: the first sample at or after the last send that shows the last target applied, else the observation end
    w1 = w_obs
    for k in np.nonzero(st >= float(send[-1]))[0]:
        if np.linalg.norm(S[k] - G[-1]) <= delta:
            w1 = float(st[k]); break
    w1 = max(w1, float(send[-1]))
    in_win = (st >= w0) & (st <= w1)
    pre = np.nonzero(st < w0)[0]
    if in_win.sum() < 2:
        return AppliedAgeEstimate("setpoint_cp", n, 0, 0.0, 0, float("nan"), float("nan"), float("nan"), float("nan"), period, int(in_win.sum()), achieved, send_gap,
                                  "undetermined", "fewer than two setpoint_cp samples inside the command window")
    # matched index per in-window sample (causal); -1 = no sent target within delta
    idx = np.array([_causal_match(S[k], G, send, st[k], delta) for k in np.nonzero(in_win)[0]])
    tw = st[in_win]
    # source time per sample: the matched target's send time; before the first application, the first command's
    # send time (nothing of this window has been applied: the oldest unmet intent is command 0)
    first_app = next((i for i, j in enumerate(idx) if j >= 0), None)
    if first_app is None:
        # the channel never showed a target of this window: the applied setpoint is at least as old as the window
        L_fb = float(feedback_latency_allowance_s) if (feedback_latency_allowance_s is not None and not math.isnan(feedback_latency_allowance_s)) else 0.0
        return AppliedAgeEstimate("setpoint_cp", n, 0, 0.0, 0, float("nan"), max(0.0, w1 - w0 - L_fb), w1 - w0, w1 - w0, period, int(in_win.sum()), achieved, send_gap,
                                  "ok", "no commanded target of this window was ever reported on setpoint_cp", 0.0, w1 - w0, L_fb, w0, w1)
    after = idx[first_app:]
    unmatched_after = float(np.mean(after < 0))
    if unmatched_after > UNMATCHED_UNDETERMINED_FRACTION:
        return AppliedAgeEstimate("setpoint_cp", n, 0, 0.0, 0, float(tw[first_app] - w0), float("nan"), float("nan"), float("nan"), period, int(in_win.sum()), achieved, send_gap,
                                  "undetermined", "setpoint_cp is not piecewise constant on the commanded targets (%.0f%% of samples after the first application match none): "
                                  "an interpolating low-level setpoint cannot be told from a partial acceptor" % (100 * unmatched_after), unmatched_after)
    # source time per sample: before the first application the oldest unmet intent (command 0); a matched sample
    # its target's send time; an unmatched sample after the first application (an intermediate value, tolerated up
    # to the fraction above) is not evidence of a fresher setpoint, so it continues the previous hold for the upper
    # bound and contributes nothing to the lower bound
    src = np.empty(len(idx)); known = np.ones(len(idx), dtype=bool)
    cur = w0
    for k, j in enumerate(idx):
        if j >= 0:
            cur = float(send[j])
        elif k >= first_app:
            known[k] = False
        src[k] = cur
    nxt = np.append(tw[1:], w1)
    ages_at = (tw - src)[known]
    ages_to = np.maximum(nxt - src, 0.0)
    # the stretch from the window start to the first in-window sample: the pre-window state (if any) is held
    # there; whatever it is, nothing of this window has been applied before the first sample that shows it
    lead_upper = float(tw[0] - w0)
    L_fb = float(feedback_latency_allowance_s) if (feedback_latency_allowance_s is not None and not math.isnan(feedback_latency_allowance_s)) else 0.0
    age_lower_receipt = float(np.max(ages_at))
    age_lower = max(0.0, age_lower_receipt - L_fb)
    age_upper = float(max(np.max(ages_to), lead_upper))
    # accepted count and ordering (informational)
    accepted, last, ooo = 0, -1, 0
    for j in idx:
        if j < 0:
            continue
        if j > last:
            accepted += 1; last = int(j)
        elif j < last:
            ooo += 1
    # update-gap diagnostic (the 0.1.2 quantity): changes of the matched index
    change_times = [tw[0]] + [tw[k] for k in range(1, len(idx)) if idx[k] != idx[k - 1]]
    gaps = np.diff(np.array(change_times + [w1])) if len(change_times) else np.array([w1 - w0])
    return AppliedAgeEstimate("setpoint_cp", n, accepted, accepted / n, ooo, float(tw[first_app] - w0), age_lower, age_upper, float(np.max(gaps)) if len(gaps) else float("nan"),
                              period, int(in_win.sum()), achieved, send_gap, "ok", "", unmatched_after, age_lower_receipt, L_fb, w0, w1)


def rate_subverdict(est: Optional[AppliedAgeEstimate], f_required_hz: float) -> str:
    """'satisfied' | 'violated' | 'undetermined' for the rate part of a declared rate expectation (0.1.3).

    Decided on the source-age bracket of the applied setpoint (eq. 9: e_ZOH <= v * age).  satisfied: the upper
    end of the bracket is within the required period 1/f_req, i.e. v * age_upper <= epsilon.  violated: the lower
    end exceeds the period AND the client itself sustained the rate, on average and in its longest send gap
    (otherwise the client, not the implementation, is the limit).  undetermined otherwise, and always without a
    setpoint channel (feedback target crossings are a diagnostic, never a verdict)."""
    if est is None or est.channel != "setpoint_cp" or est.status != "ok":
        return "undetermined"
    req_period = 1.0 / f_required_hz
    client_ok = (not math.isnan(est.client_rate_achieved_hz) and est.client_rate_achieved_hz >= f_required_hz * (1 - 1e-6)
                 and not (not math.isnan(est.client_max_send_gap_s) and est.client_max_send_gap_s > req_period * (1 + 1e-6)))
    if est.age_upper_s <= req_period * (1 + 1e-9):
        return "satisfied"
    if est.age_lower_s > req_period * (1 + 1e-9) and client_ok:
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
