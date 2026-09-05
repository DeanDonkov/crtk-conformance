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
    rate_subverdict,
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


# ------------------------------------------------------------------ 0.1.2: accepted-command channel (RC3 review, finding 2)
def test_reviewer_final_command_only_counterexample():
    from crtk_conformance.rate_estimator import estimate_acceptance, rate_subverdict
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.0005
    samples = np.vstack([np.zeros((100, 3)), targets])
    est = estimate_rate(targets, samples, np.arange(200) * 0.01, np.arange(100) * 0.01, 100.0, 5e-5, "synthetic", 0.0, 2.0)
    assert est.transitions == 100  # the crossings statistic is fooled, by construction ...
    # ... but the verdict comes from the accepted-command channel: only the final target ever appears there
    sp = np.zeros((200, 3)); sp[99:, 0] = 0.05
    acc = estimate_acceptance(targets, sp, np.arange(200) * 0.01, np.arange(100) * 0.01, 5e-5, 2.0)
    assert acc.accepted == 1 and acc.max_stale_s > 0.9
    assert rate_subverdict(acc, 50.0) == "violated"
    assert rate_subverdict(None, 50.0) == "undetermined"


def test_acceptance_stale_interval_not_mean_rate():
    from crtk_conformance.rate_estimator import estimate_acceptance, rate_subverdict
    # 100 commands at 100 Hz; the channel accepts 50 in the first 0.25 s and 50 in the last 0.25 s: mean rate 100 Hz
    # over the window but a 0.5 s stale interval in the middle -> violated for f_req = 50 Hz (period 20 ms)
    targets = np.zeros((100, 3)); targets[:, 0] = np.arange(1, 101) * 0.001
    send = np.arange(100) * 0.01
    t_acc = np.concatenate([np.arange(50) * 0.005, 0.75 + np.arange(50) * 0.005])
    sp_t = np.arange(0, 1.2, 0.002)
    sp = np.zeros((len(sp_t), 3))
    for k, t in enumerate(sp_t):
        j = int(np.searchsorted(t_acc, t, side="right")) - 1
        sp[k, 0] = targets[j, 0] if j >= 0 else 0.0
    acc = estimate_acceptance(targets, sp, sp_t, send, 1e-4, 1.2)
    assert acc.accepted == 100 and acc.max_stale_s > 0.45
    assert rate_subverdict(acc, 50.0) == "violated"
    # the same 100 acceptances spread evenly at 100 Hz -> satisfied
    t_acc2 = np.arange(100) * 0.01 + 0.002
    for k, t in enumerate(sp_t):
        j = int(np.searchsorted(t_acc2, t, side="right")) - 1
        sp[k, 0] = targets[j, 0] if j >= 0 else 0.0
    acc2 = estimate_acceptance(targets, sp, sp_t, send, 1e-4, 1.2)
    assert rate_subverdict(acc2, 50.0) == "satisfied"


def test_acceptance_channel_must_be_piecewise_constant():
    # a low-level controller that interpolates its setpoint towards each goal reports intermediate setpoints; an
    # acceptance cannot then be told from a pass-through, so the channel yields undetermined (0.1.2)
    from crtk_conformance.rate_estimator import estimate_acceptance
    targets = np.zeros((20, 3)); targets[:, 0] = np.arange(1, 21) * 0.001
    t_sp = np.arange(400) * 0.001
    sp = np.zeros((400, 3)); sp[:, 0] = np.linspace(0.0, 0.020, 400)  # ramps through the targets
    acc = estimate_acceptance(targets, sp, t_sp, np.arange(20) * 0.01, 1e-5, 0.4)
    assert acc.status == "undetermined" and acc.channel_unmatched_fraction > 0.5
    sp2 = np.zeros((400, 3)); sp2[:, 0] = np.repeat(targets[:, 0], 20)  # piecewise constant
    acc2 = estimate_acceptance(targets, sp2, t_sp, np.arange(20) * 0.01, 1e-5, 0.4)
    assert acc2.status == "ok" and acc2.accepted == 20 and acc2.channel_unmatched_fraction == 0.0
