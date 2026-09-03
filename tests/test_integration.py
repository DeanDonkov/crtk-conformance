"""Integration tests: need a ROS 1 Python stack (rospy, rosmaster, crtk_msgs). Skipped otherwise.

0.1.1 additions cover the Reviewer #2 defects as regressions: client-relative spatial decision (M3),
noise-robust rate estimation (M4), liveness resolution and stop-behaviour classes (M6), delayed
commands executed rather than discarded (M5.3), n >= 3 guard on the liveness estimate (M5.4), and
no-response handling of dropped commands (minor 3).
"""
import math
import os
import sys
import time

import numpy as np
import pytest

rospy = pytest.importorskip("rospy")
pytest.importorskip("rosmaster")
pytest.importorskip("crtk_msgs.msg")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "validation"))
from harness import ros_master, mock_node  # noqa: E402
from crtk_conformance import geometry as G  # noqa: E402
from crtk_conformance.adapter import PlatformAdapter  # noqa: E402
from crtk_conformance.expectations import Expectations  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.probes.base import Outcome  # noqa: E402

TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=100, jitter_max_s=0.005)
JHU = {"bind_translation_m": [0.20, 0.0, 0.0], "bind_axis": [1.0, 0.0, 0.0], "bind_angle_deg": -150.0}


def jhu_expectation():
    R = G.axis_angle([1, 0, 0], math.radians(-150.0))
    x, y, z, w = G.rot_to_quat(R)
    return Expectations.from_dict({"spatial": {"mode": "expected_transform", "translation_m": [0.20, 0, 0], "quaternion_xyzw": [x, y, z, w]}})


@pytest.fixture(scope="module")
def master():
    with ros_master(port=11511):
        yield


def adapter(anchor=True):
    a = PlatformAdapter("/PSM1", anchor_topic="/PSM1/mock/ground_truth_cp" if anchor else None)
    a.discover()
    return a


# ------------------------------------------------------------------ spatial (M3)
def test_frame_probe_recovers_injected_transform_discover_only(master):
    with mock_node("reference", {"bind_translation_m": [0.02, -0.01, 0.005], "bind_angle_deg": 30.0, "bind_axis": [0, 0, 1]}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED  # no expectation declared -> no verdict
    t = np.array(r.estimates["binding_translation_m"])
    assert np.linalg.norm(t - np.array([0.02, -0.01, 0.005])) < 1e-6
    assert abs(r.estimates["binding_rotation_deg"]["mean"] - 30.0) < 1e-3


def test_frame_probe_identity_expectation(master):
    e = Expectations.from_dict({"spatial": {"mode": "identity"}})
    with mock_node("reference", {}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.CONFORMANT
    with mock_node("reference", {"bind_translation_m": [0.002, 0, 0]}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.DIVERGENT


def test_frame_probe_non_identity_expected_transform(master):
    e = jhu_expectation()
    with mock_node("reference", JHU):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.CONFORMANT  # the JHU-like binding is what the client expects
    assert r.estimates["residual_translation_norm_m"] < 1e-6
    with mock_node("reference", JHU):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3, expectations=Expectations.from_dict({"spatial": {"mode": "identity"}})).run()
        a.close()
    assert r.outcome == Outcome.DIVERGENT  # the same binding against an identity expectation


def test_frame_probe_undetermined_without_local_whatever_the_expectation(master):
    with mock_node("emul-src-v1"):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=2, expectations=Expectations.from_dict({"spatial": {"mode": "identity"}})).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED
    assert "T_b_w_translation_m" in r.estimates


def test_frame_probe_orientation_noise_and_stamp_skew(master):
    with mock_node("reference", dict(JHU, orientation_noise_deg=0.2, noise_m=0.0001, stamp_skew_s=0.002)):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=5, expectations=jhu_expectation()).run()
        a.close()
    assert abs(r.estimates["binding_rotation_deg"]["mean"] - 150.0) < 0.5
    assert r.outcome in (Outcome.CONFORMANT, Outcome.UNDETERMINED)


# ------------------------------------------------------------------ dimensional
def test_scale_probe_internal_ratio_is_one_but_anchor_recovers_scale(master):
    e = Expectations.from_dict({"dimensional": {"mode": "si"}})
    with mock_node("emul-src-v1"):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=3, settle_s=0.5, expectations=e).run()
        a.close()
    assert abs(r.estimates["internal_ratio"]["mean"] - 1.0) < 1e-6  # feedback invisibility
    assert abs(r.estimates["scale_anchored"]["mean"] - 0.1) < 1e-6
    assert r.outcome == Outcome.DIVERGENT


def test_scale_probe_discover_only_reports_without_verdict(master):
    with mock_node("emul-src-v1"):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=3, settle_s=0.5).run()
        a.close()
    assert abs(r.estimates["scale_anchored"]["mean"] - 0.1) < 1e-6
    assert r.outcome == Outcome.UNDETERMINED


def test_scale_probe_undetermined_without_anchor(master):
    with mock_node("emul-src-v1"):
        a = adapter(anchor=False)
        r = ScalingUnitsProbe(a, TOL, trials=3, settle_s=0.5, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED


def test_scale_probe_dropped_commands_are_no_response_not_zero_motion(master):
    with mock_node("reference", {"drop_prob": 0.5, "seed": 7}):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.4, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run()
        a.close()
    assert r.observations["no_response_trials"] >= 1
    for t in r.observations["trials"]:
        if t.get("ok"):
            assert t["r_int"] > 0.5  # a responded trial is a full step, never a zero-motion measurement
    assert r.outcome != Outcome.DIVERGENT


# ------------------------------------------------------------------ temporal (M4, M5, M6)
def test_state_machine_required_and_forbidden(master):
    req = Expectations.from_dict({"temporal": {"state_machine": "required"}})
    forb = Expectations.from_dict({"temporal": {"state_machine": "forbidden"}})
    def one(preset, exp):
        with mock_node(preset):
            a = adapter()
            r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,), expectations=exp).run()
            a.close()
        return r.outcome
    assert one("emul-dvrk-jhu-psm2", req) == Outcome.CONFORMANT
    assert one("emul-dvrk-jhu-psm2", forb) == Outcome.DIVERGENT


def test_state_machine_absent_required_forbidden_discover_only(master):
    req = Expectations.from_dict({"temporal": {"state_machine": "required"}})
    forb = Expectations.from_dict({"temporal": {"state_machine": "forbidden"}})
    def one(preset, exp):
        with mock_node(preset):
            a = adapter()
            r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,), expectations=exp).run()
            a.close()
        return r.outcome
    assert one("emul-src-v1", req) == Outcome.DIVERGENT
    assert one("emul-src-v1", forb) == Outcome.CONFORMANT
    assert one("emul-src-v1", Expectations()) == Outcome.UNDETERMINED  # discover-only


def test_liveness_fault_estimate_and_resolution(master):
    with mock_node("reference", {"watchdog_s": 0.25, "watchdog_mode": "fault"}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.6, bisection_steps=6, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault"}})).run()
        a.close()
    L = r.observations["liveness"]
    assert L["stop_class"] == "fault"
    tau = L["tau_w_estimate_s"]
    assert tau["status"] == "ok" and abs(tau["mean"] - 0.25) < 0.03 and tau["n"] == 3
    assert r.observations["resolution"]["resolution_floor_s"] < 0.1
    assert r.outcome == Outcome.CONFORMANT  # the client's 100 Hz stream keeps well inside 0.25 s


def test_liveness_release_is_a_stop_policy_not_no_policy(master):
    with mock_node("emul-ambf-object-watchdog"):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=1.0, bisection_steps=5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})).run()
        a.close()
    L = r.observations["liveness"]
    assert L["stop_class"] == "release"
    assert "no stop policy" not in L["finding"]
    assert r.outcome == Outcome.DIVERGENT  # client expects hold; the implementation releases


def test_liveness_hold_no_policy(master):
    with mock_node("reference", {}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})).run()
        a.close()
    assert r.observations["liveness"]["stop_class"] == "no_policy_within_range"
    assert r.outcome == Outcome.CONFORMANT


def test_liveness_below_resolution_is_not_a_number(master):
    with mock_node("reference", {"watchdog_s": 0.003, "watchdog_mode": "fault"}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.3, bisection_steps=4, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault"}})).run()
        a.close()
    tau = r.observations["liveness"]["tau_w_estimate_s"]
    assert tau is None or "upper_bound_s" in tau or tau.get("status") == "undetermined"
    assert r.outcome != Outcome.CONFORMANT


def test_rate_estimator_with_noise_never_exceeds_commands(master):
    for noise in (0.0, 0.00002, 0.0005):
        with mock_node("reference", {"noise_m": noise, "loop_rate_hz": 120, "publish_rate_hz": 120, "seed": 3}):
            a = adapter()
            r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100, 500),
                                     expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
            a.close()
        for row in r.observations["effective_rate"]["per_rate"]:
            assert row["transitions"] <= row["commands_sent"]
            assert row["observable_rate_hz"] <= 1.05 * min(row["client_rate_achieved_hz"], row["publish_rate_hz"], 120.0) + 5


def test_delayed_commands_are_executed_not_discarded(master):
    # delay 50 ms with 5 ms jitter (< the 10 ms command spacing): every setpoint is executed late, none is lost
    with mock_node("reference", {"response_delay_s": 0.05, "response_jitter_s": 0.005, "seed": 4}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert row["transitions"] >= 0.9 * row["commands_sent"]
    # jitter 20 ms (> spacing): commands can become due out of order; the mock discards stale ones (latest wins),
    # so some setpoints are skipped, but the count never exceeds the commands sent and is far from the v0.1.0 '1'
    with mock_node("reference", {"response_delay_s": 0.05, "response_jitter_s": 0.02, "seed": 4}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert 0.4 * row["commands_sent"] <= row["transitions"] <= row["commands_sent"]


def test_continuous_interpolation_mock(master):
    with mock_node("reference", {"max_speed_m_s": 0.05, "publish_rate_hz": 500, "loop_rate_hz": 1000}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,)).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert row["transitions"] <= row["commands_sent"]


def test_missing_topic_gives_undetermined_not_conformant(master):
    with mock_node("reference", {"publish_measured_cp": False}):
        a = adapter()
        rs = [FrameSemanticsProbe(a, TOL, trials=2, expectations=Expectations.from_dict({"spatial": {"mode": "identity"}})).run(),
              ScalingUnitsProbe(a, TOL, trials=2, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run(),
              RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, rates_hz=(100,), expectations=Expectations.from_dict({"temporal": {"state_machine": "required"}})).run()]
        a.close()
    assert all(r.outcome == Outcome.UNDETERMINED for r in rs)


def test_rate_targets_not_separable_is_undetermined_and_sends_nothing(master):
    # resting noise 2 mm -> delta = 10 mm, spacing 40 mm: fewer than MIN_RATE_TARGETS fit in the 50 mm excursion
    with mock_node("reference", {"noise_m": 0.002, "seed": 5}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert row["status"] == "undetermined" and row["reason"] == "targets_not_separable"
    assert row["commands_sent"] == 0 and row["transitions"] == 0
    assert r.outcome == Outcome.UNDETERMINED


def test_rate_reduced_targets_keep_the_requested_rate(master):
    # resting noise 0.5 mm -> delta = 2.5 mm, spacing 10 mm: 5 targets fit; the client rate must stay at 100 Hz
    with mock_node("reference", {"noise_m": 0.0005, "loop_rate_hz": 1000, "publish_rate_hz": 1000, "seed": 6}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert row["commands_sent"] == 5 and "window shortened" in row.get("note", "")
    assert 70.0 <= row["client_rate_achieved_hz"] <= 130.0
    assert row["transitions"] <= row["commands_sent"]
