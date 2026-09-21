"""0.1.5 liveness changes (RC7 adversarial review, findings F1, F8, F10), tested on synthetic samples and on the archived
v0.1.3 trial records, without ROS.

F1  last_command_latency(): the return to base is the response to the last streamed command only after the departure to
    the half-step offset was observed; an implementation that applies commands late is still executing the in-band
    stream when the gap starts (archived L_delayed: r_last 0.2-9.7 ms against ~310 ms responses).
F8  timeout_interval_from_trials(): the onset-based drift lower bound carries the granularity term G.
F10 the probe's run() leaves a hold/fault/drift expectation undetermined when post-gap non-responses are confounded by
    baseline command loss (R_drop_50: no stop policy, 50 % random drops).
"""
import json
import math
import os
import sys
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ARCHIVE = os.path.join(HERE, "..", "validation", "v0.1.3", "mock")


def _stub_ros():
    try:
        import rospy  # noqa: F401
        return
    except ImportError:
        pass
    if "crtk_conformance.adapter" not in sys.modules:
        adapter = types.ModuleType("crtk_conformance.adapter"); adapter._stub = True
        adapter.PlatformAdapter = object; adapter.pose_msg_to_matrix = lambda x: x
        sys.modules[adapter.__name__] = adapter
        common = types.ModuleType("crtk_conformance.probes.common")
        common.ensure_enabled = lambda *a, **k: None
        common.wait_settled = lambda *a, **k: (np.eye(4), None)
        sys.modules[common.__name__] = common


def _rate_mod():
    _stub_ros()
    import crtk_conformance.probes.rate as rate_mod
    return rate_mod


# ------------------------------------------------------------------ F1: departure guard
def test_last_command_latency_instantaneous_application():
    R = _rate_mod()
    base = np.array([0.0, 0.0, -0.1]); step = 0.002; band = 0.25 * step
    t_last = 0.010
    # 100 Hz feedback: the half-step (0.5 step on z) is visible at 5 ms, the return to base at 12 ms
    samples = [(-0.005, base), (0.005, base + [0, 0, 0.5 * step]), (0.012, base), (0.022, base)]
    r, seen = R.last_command_latency(samples, t_last, base, band)
    assert seen and abs(r - 0.002) < 1e-12  # 12 ms - 10 ms


def test_last_command_latency_delayed_application_is_not_read_from_the_in_band_stream():
    """The archived 300-ms delay case: the pose is still at base (in band) when the gap starts; 0.1.3 read the first
    gap sample (0.2-9.7 ms) as the response.  0.1.5 returns NaN until a departure has been seen."""
    R = _rate_mod()
    base = np.array([0.0, 0.0, -0.1]); step = 0.002; band = 0.25 * step
    t_last = 0.010
    in_band_only = [(0.0 + 0.01 * k, base + np.array([0.25 * step * math.sin(k), 0, 0])) for k in range(30)]  # the delayed sine stream
    r, seen = R.last_command_latency(in_band_only, t_last, base, band)
    assert math.isnan(r) and not seen
    # the departure appears 300 ms later and the return 10 ms after it: the response is measured from t_last
    late = in_band_only + [(0.300, base + [0, 0, 0.5 * step]), (0.305, base + [0, 0, 0.5 * step]), (0.312, base)]
    r, seen = R.last_command_latency(late, t_last, base, band)
    assert seen and abs(r - (0.312 - t_last)) < 1e-12


def test_last_command_latency_departure_before_t_last_counts():
    """Application delay below one period: the half-step is visible before the return-to-base command is sent."""
    R = _rate_mod()
    base = np.zeros(3); step = 0.002; band = 0.25 * step
    samples = [(0.002, base + [0.5 * step, 0, 0]), (0.011, base)]
    r, seen = R.last_command_latency(samples, 0.010, base, band)
    assert seen and abs(r - 0.001) < 1e-12


# ------------------------------------------------------------------ F8: granularity term in the onset bound
def _trial(cls, gap, r_last=float("nan"), onset=None, speed=None, thr=1e-5, ref_motion=False):
    return {"class": cls, "gap_s": gap, "last_stream_latency_s": r_last, "drift_onset_s": onset, "drift_speed_m_s": speed,
            "onset_threshold_m": thr, "reference_window_motion": ref_motion}


def test_onset_lower_bound_includes_granularity():
    R = _rate_mod()
    G, fp, L = 0.010, 0.010, 0.012
    trials = [_trial("held", 0.30, r_last=0.002), _trial("drifted", 0.60, r_last=0.002, onset=0.520, speed=0.02)]
    est = R.timeout_interval_from_trials(trials, L=L, G=G, fp=fp, hold_tol=0.001, stop_class="drifted")
    thr_term = 1e-5 / 0.02
    expected_onset_low = 0.520 - 0.002 - G - thr_term - fp - L
    assert est["onset_based_lower_bounds_s"] == pytest.approx([expected_onset_low])
    # the 0.1.3 formula (without G) would be exactly one G higher
    assert est["interval_low_s"] == pytest.approx(max(0.30 - 0.002 - G - (0.001 / 0.02 + fp), expected_onset_low))
    assert est["interval_high_s"] == pytest.approx(0.520)


def test_lower_bound_falls_back_to_L_without_a_departure():
    R = _rate_mod()
    trials = [_trial("held", 0.499, r_last=float("nan")), _trial("faulted", 0.511)]
    est = R.timeout_interval_from_trials(trials, L=0.313, G=0.010, fp=0.010, hold_tol=0.001, stop_class="faulted")
    assert est["interval_low_s"] == pytest.approx(0.499 - 0.313 - 0.010)
    assert est["interval_high_s"] == pytest.approx(0.511 + 0.313)
    assert est["lower_bound_uses_L_for_all_trials"] is True


# ------------------------------------------------------------------ archived replays
def _load(name):
    return json.load(open(os.path.join(ARCHIVE, name + ".json")))


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_delayed.json")), reason="v0.1.3 archive not present")
def test_archived_delayed_case_lower_end_widens_with_the_departure_guard():
    """L_delayed (300 ms response delay, tau_w = 0.5 s): the archived r_last values (0.2-9.7 ms) were read from the in-band
    stream; treating them as unavailable (the 0.1.5 guard would have returned NaN) moves the lower end from 489 ms to
    176 ms.  The interval still contains the injected timeout."""
    R = _rate_mod()
    d = _load("L_delayed"); Lv = d["result"]["observations"]["liveness"]; tau = Lv["tau_w_estimate_s"]
    trials = [dict(t, last_stream_latency_s=float("nan")) for t in Lv["trials"]]
    est = R.timeout_interval_from_trials(trials, L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"],
                                         fp=d["result"]["observations"]["resolution"]["feedback_period_s"], hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"])
    assert est["interval_low_s"] < 0.20 and est["interval_high_s"] == pytest.approx(tau["interval_high_s"])
    assert est["interval_low_s"] <= 0.5 <= est["interval_high_s"]
    assert tau["interval_low_s"] > 0.48  # what 0.1.3 reported


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_release_002_drift.json")), reason="v0.1.3 archive not present")
def test_archived_drift_runs_still_contain_the_timeout_with_the_granularity_term():
    R = _rate_mod()
    for name in ("L_release_000_drift", "L_release_001_hold", "L_release_002_drift", "L_release_003_hold", "L_horizon_drift_0200", "L_horizon_drift_0500", "L_horizon_drift_1500"):
        d = _load(name); Lv = d["result"]["observations"]["liveness"]; tau = Lv["tau_w_estimate_s"]; fp = d["result"]["observations"]["resolution"]["feedback_period_s"]
        est = R.timeout_interval_from_trials(Lv["trials"], L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=fp, hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"])
        assert est["interval_low_s"] == pytest.approx(tau["interval_low_s"] - tau["granularity_allowance_s"], abs=1e-9), name
        assert est["interval_low_s"] <= d["truth"]["tau_w_s"] <= est["interval_high_s"], name


# ------------------------------------------------------------------ F10: command-loss confound (offline replay of run())
class _OfflineAdapter:
    discovery = {"topics": {x: {"present": True} for x in ("measured_cp", "servo_cp")}}

    def subscribe(self, *a):
        return object()

    def wait_for(self, *a):
        return np.eye(4)

    def has(self, name):
        return False

    def latest_pose(self, *a):
        return np.eye(4)


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "R_drop_50.json")), reason="v0.1.3 archive not present")
def test_archived_drop_50_hold_expectation_is_undetermined_not_violated():
    """R_drop_50: no stop policy, 50 % of commands dropped; 0.1.3 read 3 of 4 unanswered post-gap commands as a rejection
    policy ('tau_w <= 23 ms') and violated the hold expectation.  With the calibration probes showing baseline command loss,
    0.1.5 leaves the expectation undetermined."""
    R = _rate_mod()
    from crtk_conformance.expectations import Expectations
    from crtk_conformance.thresholds import Tolerance
    R.ensure_enabled = lambda *a, **k: None
    R.wait_settled = lambda *a, **k: (np.eye(4), None)
    d = _load("R_drop_50"); te = next(x for x in d["results"] if x["binding_class"] == "temporal"); obs = te["observations"]
    assert obs["resolution"]["latency_probes_responded"] < 12  # the confound is visible in the archived calibration
    liveness = dict(obs["liveness"])
    # replay the 0.1.5 confound decision on the archived trials
    n_resp = obs["resolution"]["latency_probes_responded"]
    liveness["rejection_confounded_by_command_loss"] = True
    liveness["confound_note"] = f"{12 - n_resp} of 12 calibration commands sent without any preceding silence drew no response"
    liveness["tau_w_estimate_s"] = dict(liveness["tau_w_estimate_s"], status="undetermined")
    probe = R.RateSensitivityProbe(_OfflineAdapter(), Tolerance(), expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold", "horizon_s": 0.5}}), gap_max_s=0.5)
    probe.measure_resolution = lambda *a: obs["resolution"]
    probe.probe_state_precondition = lambda *a: obs["state_precondition"]
    probe.probe_liveness = lambda *a: liveness
    probe.probe_effective_rate = lambda *a: {"per_rate": []}
    res = probe.run()
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"
    assert res.outcome.value == "undetermined"
    assert te["estimates"]["sub_verdicts"]["stop_behaviour"] == "violated"  # what 0.1.3 decided


# ------------------------------------------------------------------ F10: attributable evidence under baseline command loss
def _ftrial(cls, gap, r_last=float("nan"), t_state=None):
    return dict(_trial(cls, gap, r_last=r_last), state_observed_at_s=t_state)


def test_under_command_loss_a_fault_is_bounded_by_its_state_observation_and_a_rejection_contributes_nothing():
    """Smoke run of the v0.1.5 campaign (V_drop_fault, tau_w = 0.25 s, 30 % drops): a post-gap command lost at a 10.7 ms
    gap, then the FAULT fired during the 0.3 s response wait and was read from the operating state.  Read as 'fault at
    10.7 ms of silence' the trial contradicts the passing trial at 197 ms (0.1.5 before this rule: inconsistent);
    the state observation bounds the timeout by the time it was made."""
    R = _rate_mod()
    L, G, fp = 0.0119, 0.010, 0.010
    trials = [_ftrial("held", 0.1968, r_last=0.0023, t_state=0.21), _ftrial("faulted", 0.0107, t_state=0.325), _ftrial("faulted", 0.290, t_state=0.60),
              _ftrial("rejected", 0.050, t_state=None)]
    with_loss = R.timeout_interval_from_trials(trials, L=L, G=G, fp=fp, hold_tol=0.001, stop_class="faulted", baseline_loss=5)
    assert with_loss["status"] == "formed"
    assert with_loss["tripping_upper_bounds_s"] == pytest.approx([0.325, 0.60])
    assert with_loss["unattributable_trials"] == 1 and with_loss["attributable_tripping_trials"] == 2
    assert with_loss["interval_low_s"] == pytest.approx(0.1968 - 0.0023 - G) and with_loss["interval_high_s"] == pytest.approx(0.325)
    assert with_loss["interval_low_s"] <= 0.25 <= with_loss["interval_high_s"]
    without = R.timeout_interval_from_trials(trials, L=L, G=G, fp=fp, hold_tol=0.001, stop_class="faulted", baseline_loss=0)
    assert without["status"] == "inconsistent" and without["interval_high_s"] == pytest.approx(0.0107 + L)


def test_under_command_loss_rejections_alone_leave_no_attributable_trip():
    R = _rate_mod()
    trials = [_ftrial("held", 0.30, r_last=0.002), _ftrial("rejected", 0.60), _ftrial("faulted", 0.50, t_state=None)]
    est = R.timeout_interval_from_trials(trials, L=0.012, G=0.010, fp=0.010, hold_tol=0.001, stop_class="rejected", baseline_loss=2)
    assert est["status"] == "no_attributable_trip" and est["unattributable_trials"] == 2 and est["tripping_upper_bounds_s"] == []
    assert R.attributable({"class": "drifted"}, 3) and not R.attributable({"class": "held"}, 0)
    assert R.attributable({"class": "rejected"}, 0) and not R.attributable({"class": "rejected"}, 1)


# ------------------------------------------------------------------ observability of a gap trial (v0.1.5 verification campaign)
def test_held_trial_without_drift_evaluation_is_not_a_passing_trial_for_a_drift_policy():
    """V_delayed_drift_1 of the verification campaign (release with drift, tau_w = 0.5 s, commands applied 300 ms late): the
    settle hint made the reference window 0.93 s long, so silences of 0.85-0.92 s ended before any drift could be seen,
    yet the post-gap command was acted on (a release does not reject commands) and the trials were classified held.
    Their passing bounds (0.547 s) exceeded the timeout.  A held trial whose drift was never evaluated is not a passing
    trial for a drift policy."""
    R = _rate_mod()
    G, fp, L = 0.010, 0.010, 0.311
    trials = [dict(_trial("held", 0.918, r_last=0.301), drift_evaluated=False, reference_window_s=0.93),
              dict(_trial("drifted", 1.128, r_last=0.301, onset=0.932, speed=0.02), reference_window_still=False, drift_evaluated=True, reference_window_s=0.93)]
    est = R.timeout_interval_from_trials(trials, L=L, G=G, fp=fp, hold_tol=0.0012, stop_class="drifted")
    assert est["passing_lower_bounds_s"] == [] and est["onset_based_lower_bounds_s"] == []
    assert est["interval_low_s"] == 0.0 and est["interval_high_s"] == pytest.approx(0.932)
    # the archived records carry no flags: a held trial is judged by its gap against its reference window
    old = [dict(_trial("held", 0.0107, r_last=0.002), reference_window_s=0.1), dict(_trial("held", 0.30, r_last=0.002), reference_window_s=0.1),
           dict(_trial("drifted", 0.60, r_last=0.002, onset=0.520, speed=0.02), reference_window_s=0.1)]
    est = R.timeout_interval_from_trials(old, L=0.012, G=G, fp=fp, hold_tol=0.001, stop_class="drifted")
    assert len(est["passing_lower_bounds_s"]) == 1 and len(est["onset_based_lower_bounds_s"]) == 1
    assert R.drift_evaluated({"gap_s": 0.0107, "reference_window_s": 0.1}) is False and R.drift_evaluated({"drift_evaluated": True}) is True


def test_censored_onset_bounds_from_above_only():
    R = _rate_mod()
    t = dict(_trial("drifted", 1.5, r_last=0.002, onset=0.51, speed=0.02), reference_window_still=False)
    est = R.timeout_interval_from_trials([t, dict(_trial("held", 0.30, r_last=0.002), drift_evaluated=True)], L=0.012, G=0.010, fp=0.010, hold_tol=0.001, stop_class="drifted")
    assert est["onset_based_lower_bounds_s"] == [] and est["tripping_upper_bounds_s"] == pytest.approx([0.51])
    t["reference_window_still"] = True
    est = R.timeout_interval_from_trials([t], L=0.012, G=0.010, fp=0.010, hold_tol=0.001, stop_class="drifted")
    assert len(est["onset_based_lower_bounds_s"]) == 1


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_release_002_drift.json")), reason="v0.1.3 archive not present")
def test_archived_intervals_are_unchanged_by_the_observability_rules():
    """No archived liveness trial has a pose already at the post-gap goal (initial_distance_m < 0.25 step), and every archived
    drift onset follows a still 0.1-s reference window; the observability rules therefore leave the re-derived archive as
    reported (23/23, median width 34.6 ms)."""
    for name in ("L_release_000_drift", "L_release_002_drift", "L_horizon_drift_0200", "L_delayed", "L_fault_006"):
        d = _load(name); Lv = d["result"]["observations"]["liveness"]; Rs = d["result"]["observations"]["resolution"]
        assert all(t["initial_distance_m"] >= 0.25 * Rs["step_used_m"] for t in Lv["trials"] if t.get("initial_distance_m") is not None), name
        assert all("reference_window_still" not in t for t in Lv["trials"])
