"""Synthetic-feedback tests of the 0.1.1 observable-rate estimator (no ROS needed).

Each test builds a feedback stream from a schedule of commanded targets under a stated execution /
publication / noise model and checks the estimator's count and status.  Reviewer #2 M4 regression: a
run with 100 commands must never report more than 100 executed transitions, at any noise level.
"""
import math

import numpy as np
import pytest

from crtk_conformance.rate_estimator import (
    classify_samples,
    count_transitions,
    estimate_noise_sigma,
    estimate_rate,
    match_tolerance_from_sigma,
)


def targets_along_x(n=100, step=0.002, base=(0.0, 0.0, -0.1)):
    G = np.tile(np.asarray(base, dtype=float), (n, 1))
    G[:, 0] += step * (np.arange(n) + 1) / n
    return G


def synth(
    n=100,
    client_hz=100.0,
    exec_hz=None,
    publish_hz=200.0,
    noise=0.0,
    interpolate_s=0.0,
    drop_every=0,
    delay_s=0.0,
    step=0.002,
    seed=0,
    window_tail_s=0.3,
):
    """Feedback samples for n commands sent at client_hz.

    exec_hz: execution rate (None = every command executes immediately when due; otherwise a latest-wins
    loop at exec_hz).  interpolate_s: time the controller takes to move linearly between successive targets.
    drop_every: every k-th command is lost.  delay_s: constant response delay.
    """
    rng = np.random.default_rng(seed)
    G = targets_along_x(n, step)
    send = np.arange(n) / client_hz
    # executed target index as a function of time
    due = send + delay_s
    executed = [(due[k], k) for k in range(n) if not (drop_every and (k + 1) % drop_every == 0)]
    if exec_hz:
        # latest-wins sampling at exec ticks
        ticks = np.arange(0, send[-1] + delay_s + window_tail_s, 1.0 / exec_hz)
        ex = []
        last = -1
        for t in ticks:
            cand = [k for (d, k) in executed if d <= t]
            if cand and cand[-1] != last:
                last = cand[-1]
                ex.append((t, last))
        executed = ex
    base = G[0].copy()
    base[0] -= step / n  # start one spacing before the first target
    t_end = send[-1] + delay_s + window_tail_s
    times = np.arange(0, t_end, 1.0 / publish_hz)
    P = np.zeros((len(times), 3))
    for i, t in enumerate(times):
        past = [(te, k) for (te, k) in executed if te <= t]
        if not past:
            P[i] = base
            continue
        te, k = past[-1]
        prev = G[past[-2][1]] if len(past) > 1 else base
        if interpolate_s > 0 and t - te < interpolate_s:
            a = (t - te) / interpolate_s
            P[i] = prev + a * (G[k] - prev)
        else:
            P[i] = G[k]
    P += rng.normal(0.0, noise, P.shape) if noise > 0 else 0.0
    return G, P, times, send, t_end


def run(delta=None, **kw):
    G, P, times, send, t_end = synth(**kw)
    if delta is None:
        delta = match_tolerance_from_sigma(kw.get("noise", 0.0))
    return estimate_rate(G, P, times, send, kw.get("client_hz", 100.0), delta, "test", 0.0, t_end)


def test_zero_noise_counts_every_command():
    e = run()
    assert e.transitions == 100 and e.status == "ok"
    assert abs(e.observable_rate_hz * e.window_s - 100) < 1e-9


@pytest.mark.parametrize("noise", [1e-6, 5e-6, 2e-5, 5e-5, 5e-4])
def test_noise_never_inflates_count(noise):
    # step 0.002 over 100 targets = 20 um spacing; at 0.5 mm noise the targets are not separable
    e = run(noise=noise)
    assert e.transitions <= 100
    if noise <= 1e-6:
        assert e.transitions == 100 and e.status == "ok"
    if noise >= 2e-5:
        assert e.status == "undetermined"


def test_noise_near_tolerance_reports_unmatched_not_extra_transitions():
    sigma = 2e-6
    delta = match_tolerance_from_sigma(sigma)  # 10 um; spacing 20 um -> separable
    e = run(noise=sigma * 1.2, delta=delta)
    assert e.transitions <= 100
    assert e.samples_unmatched >= 0


def test_continuous_interpolation_never_exceeds_commands():
    for interp in (0.005, 0.02, 0.05):
        e = run(interpolate_s=interp, publish_hz=1000.0)
        assert e.transitions <= 100
        assert e.targets_reached <= 100


def test_dropped_commands_reduce_count():
    e = run(drop_every=3)
    assert e.transitions == 100 - 33 and e.status == "ok"


def test_delayed_observations_keep_count():
    e = run(delay_s=0.05)
    assert e.transitions == 100 and e.status == "ok"


def test_feedback_publish_slower_than_execution_is_channel_bounded():
    e = run(publish_hz=30.0, exec_hz=None)
    assert e.transitions <= e.samples_total
    assert e.observation_bounded_by == "publish"
    assert e.observable_rate_hz < 40


def test_client_slower_than_requested_is_reported():
    e = run(client_hz=60.0, publish_hz=1000.0)
    assert abs(e.client_rate_achieved_hz - 60.0) < 1.0
    assert e.observation_bounded_by == "client"


def test_execution_slower_than_publication():
    e = run(exec_hz=50.0, publish_hz=200.0)
    assert 40 <= e.transitions <= 60
    assert e.observation_bounded_by == "none"


def test_reviewer2_regression_120_distinct_for_100_commands():
    """R_divergent_but_noisy (v0.1.0): 0.5 mm noise, 100 commands, publish 120 Hz -> 120 'distinct positions'."""
    e = run(noise=5e-4, publish_hz=120.0)
    assert e.transitions <= 100
    assert e.status == "undetermined"  # 20 um targets are not separable under 0.5 mm noise: say so


def test_invariant_random_configurations():
    rng = np.random.default_rng(1)
    for _ in range(200):
        n = int(rng.integers(5, 60))
        kw = dict(n=n, client_hz=float(rng.choice([20, 50, 100, 200])), publish_hz=float(rng.choice([30, 100, 200, 500])),
                  noise=float(rng.choice([0, 1e-7, 1e-6, 1e-5, 1e-4])), interpolate_s=float(rng.choice([0, 0.01, 0.05])),
                  drop_every=int(rng.choice([0, 2, 5])), delay_s=float(rng.choice([0, 0.02, 0.1])), seed=int(rng.integers(0, 1000)))
        e = run(**kw)
        assert e.transitions <= n


def test_noise_sigma_and_tolerance():
    rng = np.random.default_rng(3)
    P = rng.normal(0, 1e-4, (500, 3))
    s = estimate_noise_sigma(P)
    assert 0.7e-4 < s < 1.4e-4
    assert match_tolerance_from_sigma(0.0) == 1e-6


def test_classify_and_count_basics():
    G = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=float)
    S = np.array([[0.01, 0, 0], [0.5, 0, 0], [1.02, 0, 0], [0.99, 0, 0], [2.0, 0, 0], [1.0, 0, 0]])
    idx = classify_samples(S, G, 0.1)
    assert list(idx) == [0, -1, 1, 1, 2, 1]
    c = count_transitions(idx)
    assert c["transitions"] == 3 and c["targets_reached"] == 3


def test_subverdict_legacy_crossing_rule():
    # 0.1.1 rule on the feedback crossings statistic; kept only as a record (not used for verdicts since 0.1.2,
    # see test_reviewer_final_command_only_counterexample)
    from crtk_conformance.rate_estimator import crossing_subverdict_legacy
    e = run()
    assert crossing_subverdict_legacy(e, 50.0) == "satisfied"
    assert crossing_subverdict_legacy(e, 150.0) == "undetermined"  # the client itself only presented 100 Hz
    e_slow = run(exec_hz=50.0, publish_hz=200.0)
    assert crossing_subverdict_legacy(e_slow, 80.0) == "violated"
    e2 = run(publish_hz=30.0)
    assert crossing_subverdict_legacy(e2, 50.0) == "undetermined"
    assert crossing_subverdict_legacy(None, 50.0) == "undetermined"


# ------------------------------------------------------------------ 0.1.3: applied-setpoint source age (RC3 finding 2; RC4 findings 1-3)
# The cases below pin the 0.1.3 DECISION RULE, which 0.1.4 retains only for re-deriving the archived campaign;
# test_rate_class_carries_no_verdict_0_1_4 pins what the tool reports now.
from crtk_conformance.rate_estimator import estimate_applied_age  # noqa: E402
from crtk_conformance.rate_estimator import rate_subverdict, rate_subverdict_v013_archival  # noqa: E402
_rule = rate_subverdict_v013_archival


def _held_channel(targets, send, apply_times, st, pre=None):
    """channel samples at st: the target applied last before each sample (apply_times[j] = when target j was applied);
    before the first application the pre-window value (or the first target minus 1 m)"""
    sp = np.zeros((len(st), 3))
    for k, t in enumerate(st):
        applied = [j for j in range(len(apply_times)) if apply_times[j] <= t]
        sp[k] = targets[applied[-1]] if applied else (pre if pre is not None else targets[0] - np.array([1.0, 0, 0]))
    return sp


def test_rc3_reviewer_final_command_only_counterexample():
    # RC3 review, finding 2: 100 crossings of measured_cp for 1 accepted command; the verdict rests on the channel
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    samples = np.vstack([np.zeros((100, 3)), targets])
    est = estimate_rate(targets, samples, np.arange(200) * 0.01, np.arange(100) * 0.01, 100.0, 5e-5, "synthetic", 0.0, 2.0)
    assert est.transitions == 100  # the crossings statistic is fooled, by construction ...
    send = np.arange(100) * 0.01
    st = np.arange(0, 1.2, 0.005)
    sp = _held_channel(targets, send, [np.inf] * 99 + [0.995], st)  # only the last command is ever applied
    acc = estimate_applied_age(targets, sp, st, send, 5e-5)
    assert acc.accepted == 1 and acc.age_lower_s > 0.9
    assert _rule(acc, 50.0) == "violated"
    assert _rule(None, 50.0) == "undetermined"


def test_rc4_reviewer_source_age_not_update_cadence():
    # RC4 review, finding 1: 100 Hz source, ordered delivery applied at 50 Hz after 10 ms: the channel changes every
    # 20 ms (the 0.1.2 quantity passed) while the applied target ages to 0.5 s; the age bracket says violated
    send = np.arange(100) * 0.01
    targets = np.zeros((100, 3)); targets[:, 0] = send * 0.05
    apply = 0.01 + np.arange(100) * 0.02
    st = np.arange(403) * 0.005
    sp = _held_channel(targets, send, apply, st, pre=np.array([-0.001, 0, 0]))
    acc = estimate_applied_age(targets, sp, st, send, 1e-7)
    assert acc.accepted == 100 and acc.max_update_gap_s < 0.025
    # the last target (sent at 0.99 s) is applied at 1.99 s: the supremum of the source age over the window is 1.0 s
    assert acc.age_lower_s > 0.95 and acc.age_upper_s < 1.05
    assert _rule(acc, 50.0) == "violated"


def test_rc4_reviewer_publication_uncertainty_is_not_extra_allowance():
    # RC4 review, finding 2: updates every 24 ms, publication every 12 ms, client every 6 ms, requirement 20 ms:
    # 0.1.2 passed because it added the channel period to the allowed period; the bracket [age_lower, age_upper]
    # brackets the true supremum (24 ms) and the verdict is not satisfied
    send = np.arange(200) * 0.006
    targets = np.zeros((200, 3)); targets[:, 0] = np.arange(200) * 0.0003
    apply = np.full(200, np.inf); apply[::4] = np.arange(50) * 0.024  # every fourth command applied, at its send time
    st = np.arange(101) * 0.012
    sp = _held_channel(targets, send, apply, st)
    acc = estimate_applied_age(targets, sp, st, send, 1e-7)
    assert acc.age_lower_s <= 0.024 + 1e-9 <= acc.age_upper_s + 1e-9
    assert _rule(acc, 50.0) != "satisfied"


def test_rc4_reviewer_preexisting_setpoint_is_not_a_future_acceptance():
    # RC4 review, finding 3: the channel starts at a value equal to this sweep's FINAL target; every new command is
    # applied after 5 ms; causal matching must not credit the old sample as an acceptance of command 99
    send = np.arange(100) * 0.01
    targets = np.zeros((100, 3)); targets[:, 0] = (np.arange(100) + 1) * 0.00002
    apply = send + 0.004
    st = np.arange(202) * 0.005
    sp = _held_channel(targets, send, apply, st, pre=targets[-1])
    acc = estimate_applied_age(targets, sp, st, send, 1e-7)
    assert acc.accepted == 100 and acc.age_upper_s <= 0.015 + 1e-9
    assert _rule(acc, 50.0) == "satisfied"


def test_growing_queue_delay_is_violated_and_prompt_application_is_satisfied():
    send = np.arange(100) * 0.01
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.001
    st = np.arange(0, 1.05, 0.002)
    prompt = _held_channel(targets, send, send + 0.003, st)
    acc = estimate_applied_age(targets, prompt, st, send, 1e-6)
    assert _rule(acc, 50.0) == "satisfied" and acc.age_upper_s < 0.02
    growing = _held_channel(targets, send, send * 1.5 + 0.003, st)  # delivery delay grows with time
    acc2 = estimate_applied_age(targets, growing, st, send, 1e-6)
    assert acc2.age_lower_s > 0.3 and _rule(acc2, 50.0) == "violated"


def test_client_send_stall_is_not_attributed_to_the_implementation():
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.001
    send = np.arange(100) * 0.01; send[50:] += 0.06
    st = np.arange(0, 1.2, 0.002)
    sp = _held_channel(targets, send, send + 0.002, st)
    acc = estimate_applied_age(targets, sp, st, send, 1e-5)
    assert acc.accepted == 100 and acc.age_lower_s > 0.06 and acc.client_max_send_gap_s > 0.06
    assert _rule(acc, 50.0) == "undetermined"


def test_channel_never_showing_a_window_target_is_violated():
    targets = np.zeros((50, 3)); targets[:, 0] = np.arange(1, 51) * 0.001
    st = np.arange(0, 1.0, 0.002); sp = np.full((len(st), 3), -0.5)
    acc = estimate_applied_age(targets, sp, st, np.arange(50) * 0.02, 1e-5)
    assert acc.status == "ok" and acc.accepted == 0 and acc.age_lower_s >= 0.9
    assert _rule(acc, 50.0) == "violated"


def test_interpolating_low_level_setpoint_is_undetermined():
    targets = np.zeros((20, 3)); targets[:, 0] = np.arange(1, 21) * 0.001
    send = np.arange(20) * 0.01
    st = np.arange(0, 0.2, 0.001)
    sp = np.zeros((len(st), 3)); sp[:, 0] = np.interp(st, send, targets[:, 0])  # ramps through the targets
    acc = estimate_applied_age(targets, sp, st, send, 1e-6)
    assert acc.status == "undetermined" and acc.unmatched_after_first_fraction > 0.5


def test_sparse_channel_widens_the_bracket_towards_undetermined():
    # the same prompt application observed at 20 Hz: the upper end of the bracket grows by the sampling interval
    send = np.arange(100) * 0.01
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.001
    st = np.arange(0, 1.05, 0.05)
    sp = _held_channel(targets, send, send + 0.003, st)
    acc = estimate_applied_age(targets, sp, st, send, 1e-6)
    assert acc.age_upper_s >= 0.05 and acc.age_lower_s < 0.02
    assert _rule(acc, 50.0) == "undetermined"


def test_feedback_latency_allowance_lowers_only_the_lower_bound():
    # a sample received at t_k was published up to L_fb earlier: the age AT the sample overstates the true age by
    # up to L_fb, so the lower bound subtracts it; the upper bound (hold until the next sample) is unchanged
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    send = np.arange(100) * 0.01
    apply = send + 0.030  # every target applied 30 ms after it was sent and held for one period: sup age 40 ms
    st = np.arange(0.0, 1.3, 0.002)
    sp = _held_channel(targets, send, apply, st)
    a0 = estimate_applied_age(targets, sp, st, send, 5e-5)
    a1 = estimate_applied_age(targets, sp, st, send, 5e-5, feedback_latency_allowance_s=0.004)
    assert abs(a0.age_lower_s - 0.040) < 0.0025 and a0.feedback_latency_allowance_s == 0.0
    assert abs(a1.age_lower_s - (a0.age_lower_s - 0.004)) < 1e-9 and a1.age_lower_at_receipt_s == a0.age_lower_s
    assert a1.age_upper_s == a0.age_upper_s
    # 40 ms of age against a 20 ms period: violated with or without the allowance; an allowance as large as the
    # excess makes it undetermined, never satisfied
    assert _rule(a0, 50.0) == "violated" and _rule(a1, 50.0) == "violated"
    a2 = estimate_applied_age(targets, sp, st, send, 5e-5, feedback_latency_allowance_s=0.025)
    assert _rule(a2, 50.0) == "undetermined"


def test_rc4_reviewer_fifo_queue_at_half_the_client_rate_is_violated():
    # RC4 review, finding 1: an ordered queue applying one command per 20 ms while the client sends every 10 ms
    # changes the channel regularly (update gap 20 ms) but applies ever older commands; the source age grows to
    # ~0.5 s over a 1 s window
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    send = np.arange(100) * 0.01
    apply = 0.005 + np.arange(100) * 0.02  # FIFO at 50 Hz
    st = np.arange(0.0, 2.2, 0.002)
    sp = _held_channel(targets, send, apply, st)
    acc = estimate_applied_age(targets, sp, st, send, 5e-5)
    assert acc.max_update_gap_s < 0.025  # the 0.1.2 statistic would have passed this
    assert acc.age_lower_s > 0.45 and acc.age_upper_s > acc.age_lower_s
    assert _rule(acc, 50.0) == "violated"


def test_window_end_uses_the_age_of_the_last_target_at_its_application():
    # a 20 Hz channel (period 50 ms) on an instantaneous implementation: the sample that first shows the LAST
    # target may arrive up to 50 ms after its application; the client's intent stops moving at the last send, so
    # the age charged at the window end is the last target's age at application, bracketed between the previous
    # sample and this one -- not the age at the sample's receipt (which would charge the channel's publication
    # interval to the implementation: found by the v0.1.3 event-log truth check)
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    send = np.arange(100) * 0.01
    apply = send + 0.001
    st = 0.049 + np.arange(0, 1.2, 0.05)  # last send at 0.99, applied 0.991; samples at 0.999 (shows 98) and 1.049 (shows 99)
    sp = _held_channel(targets, send, apply, st)
    acc = estimate_applied_age(targets, sp, st, send, 5e-5)
    assert acc.age_lower_s < 0.015 and acc.age_upper_s > 0.05  # undetermined: the sparse channel cannot show satisfaction
    assert _rule(acc, 50.0) == "undetermined"
    # the same channel on a FIFO buffer that applies the last target 0.7 s late: the lower bound keeps the delay
    apply2 = send + 0.7
    st2 = 0.049 + np.arange(0, 2.0, 0.05)
    sp2 = _held_channel(targets, send, apply2, st2)
    acc2 = estimate_applied_age(targets, sp2, st2, send, 5e-5)
    assert acc2.age_lower_s > 0.64 and _rule(acc2, 50.0) == "violated"


# ------------------------------------------------------------------ 0.1.4: the rate class carries no verdict
def test_rate_class_carries_no_verdict_0_1_4():
    """Whatever the bracket says, the reported rate sub-verdict is 'undetermined' (0.1.4).

    The bracket itself is unchanged -- each case below still reproduces its 0.1.3 classification under the
    archival rule, which is what the v0.1.3 validation archive was scored with."""
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    send = np.arange(100) * 0.01
    st = np.arange(0, 1.2, 0.005)

    # (a) a channel that applies only the last command: 0.1.3 said "violated"
    sp = _held_channel(targets, send, [np.inf] * 99 + [0.995], st)
    late = estimate_applied_age(targets, sp, st, send, 5e-5)
    assert rate_subverdict_v013_archival(late, 50.0) == "violated"
    assert rate_subverdict(late, 50.0) == "undetermined"

    # (b) a channel that applies every command promptly: 0.1.3 said "satisfied"
    sp = _held_channel(targets, send, list(send + 0.001), st)
    prompt = estimate_applied_age(targets, sp, st, send, 5e-5)
    assert rate_subverdict_v013_archival(prompt, 50.0) == "satisfied"
    assert rate_subverdict(prompt, 50.0) == "undetermined"

    # (c) no setpoint channel at all
    assert rate_subverdict(None, 50.0) == "undetermined"

    # the measurement survives the withdrawal: the brackets still differ, and still order as before
    assert late.age_lower_s > prompt.age_upper_s
