"""Integration tests: need a ROS 1 Python stack (rospy, rosmaster, crtk_msgs). Skipped otherwise.

0.1.1 additions cover the Reviewer #2 defects as regressions: client-relative spatial decision (M3),
noise-robust rate estimation (M4), liveness resolution and stop-behaviour classes (M6), delayed
commands executed rather than discarded (M5.3), n >= 3 guard on the liveness estimate (M5.4), and
no-response handling of dropped commands (minor 3).  0.1.2 (RC3 adversarial review): calibrated spatial region,
accepted-command rate verdicts, liveness intervals that contain the timeout, observation horizons.
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
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=3).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED  # no expectation declared -> no verdict
    t = np.array(r.estimates["binding_translation_m"])
    assert np.linalg.norm(t - np.array([0.02, -0.01, 0.005])) < 1e-6
    assert abs(r.estimates["binding_rotation_deg"]["mean"] - 30.0) < 1e-3


def test_frame_probe_identity_expectation(master):
    e = Expectations.from_dict({"spatial": {"mode": "identity"}})
    with mock_node("reference", {}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.CONFORMANT
    with mock_node("reference", {"bind_translation_m": [0.002, 0, 0]}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.DIVERGENT


def test_frame_probe_non_identity_expected_transform(master):
    e = jhu_expectation()
    with mock_node("reference", JHU):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=3, expectations=e).run()
        a.close()
    assert r.outcome == Outcome.CONFORMANT  # the JHU-like binding is what the client expects
    assert r.estimates["residual_translation_norm_m"] < 1e-6
    with mock_node("reference", JHU):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=5, samples_per_trial=3, expectations=Expectations.from_dict({"spatial": {"mode": "identity"}})).run()
        a.close()
    assert r.outcome == Outcome.DIVERGENT  # the same binding against an identity expectation


def test_frame_probe_undetermined_without_local_whatever_the_expectation(master):
    with mock_node("emul-src-v1"):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=2, expectations=Expectations.from_dict({"spatial": {"mode": "identity"}})).run()  # n <= 3: undetermined by construction
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
    # 50 % drops: the streamed goal gets through, every responded trial is a full step (never a zero-motion r_int = 0)
    with mock_node("reference", {"drop_prob": 0.5, "seed": 7}):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.4, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run()
        a.close()
    for t in r.observations["trials"]:
        if t.get("ok"):
            assert t["r_int"] > 0.5
    assert r.outcome != Outcome.DIVERGENT
    # every command dropped: no-response trials are counted, not recorded as zero displacements; < 3 valid -> undetermined
    with mock_node("reference", {"drop_prob": 1.0, "seed": 7}):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.4, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run()
        a.close()
    assert r.observations["no_response_trials"] == 6 and r.observations["valid_trials"] == 0
    assert r.outcome == Outcome.UNDETERMINED


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
    assert L["stop_class"] == "faulted"
    tau = L["tau_w_estimate_s"]
    # 0.1.2: the interval must CONTAIN the injected timeout (RC3 review, finding 3), and be tight
    assert tau["status"] == "ok" and tau["n"] == 3
    assert tau["interval_low_s"] <= 0.25 <= tau["interval_high_s"], tau
    # 0.1.7: faulted trials are bounded by the time the FAULT was observed (sound whether or not the post-gap command
    # arrived; wider by about the response wait); the 0.1.6 interval, conditional on post-gap arrival, stays tight
    ce = tau["conditional_estimate_s"]
    assert ce["interval_low_s"] <= 0.25 <= ce["interval_high_s"] and ce["interval_high_s"] - ce["interval_low_s"] < 0.15, ce
    assert tau["interval_high_s"] - tau["interval_low_s"] < 0.15 + L["fault_observation_window_s"], tau
    assert r.observations["resolution"]["resolution_floor_s"] < 0.1
    assert r.outcome == Outcome.CONFORMANT  # the client's 100 Hz stream keeps well inside 0.25 s


def test_liveness_release_is_a_stop_policy_not_no_policy(master):
    with mock_node("emul-ambf-object-watchdog"):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=1.0, bisection_steps=5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})).run()
        a.close()
    L = r.observations["liveness"]
    assert L["stop_class"] == "drifted"
    assert "held within" not in L["finding"]
    assert r.outcome == Outcome.DIVERGENT  # client expects hold within the horizon; the pose drifted at 0.5 s


def test_liveness_hold_no_policy(master):
    with mock_node("reference", {}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})).run()
        a.close()
    assert r.observations["liveness"]["stop_class"] == "held_through_range"
    assert "longer timeout is not excluded" in r.observations["liveness"]["finding"]
    assert r.outcome == Outcome.CONFORMANT  # held through the (default) horizon = tested range


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
    # resting noise 0.4 mm -> delta = 2.0 mm, spacing 8 mm: 6 targets fit in the 50-mm excursion (5-7 with the spread of
    # the MAD noise estimate; 0.5 mm sat exactly on the 5-target boundary and made the test a coin flip); the client
    # rate must stay at 100 Hz
    with mock_node("reference", {"noise_m": 0.0004, "loop_rate_hz": 1000, "publish_rate_hz": 1000, "seed": 6}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert 5 <= row["commands_sent"] <= 7 and "window shortened" in row.get("note", "")
    assert 70.0 <= row["client_rate_achieved_hz"] <= 130.0
    assert row["transitions"] <= row["commands_sent"]


def test_liveness_short_release_with_drift_is_detected(master):
    # release at 0.1 s with drift: the resting-noise window must not let the policy fire, otherwise the drift
    # inflates the hold tolerance and the policy is missed (first v0.1.1 mock campaign, L_release_000)
    with mock_node("reference", {"watchdog_s": 0.1, "watchdog_mode": "release", "release_drift_m_s": 0.02}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=1.0, bisection_steps=5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold"}})).run()
        a.close()
    L = r.observations["liveness"]
    assert L["stop_class"] == "drifted"
    assert r.observations["resolution"]["resting_noise_sigma_m"] < 0.001
    tau = L["tau_w_estimate_s"]
    assert tau["status"] == "ok" and tau["interval_low_s"] <= 0.1 <= tau["interval_high_s"], tau  # onset-based, no 0.2 s window
    assert r.outcome == Outcome.DIVERGENT


def test_scale_probe_streams_goals_so_a_release_policy_does_not_corrupt_the_estimate(master):
    # AMBF-watchdog emulation: release after 0.5 s of silence with 20 mm/s drift; the scale probe must keep its
    # command stream alive during settle and measurement (first v0.1.1 campaign: anchored s_hat = 1.91 for a unit of 1)
    with mock_node("emul-ambf-object-watchdog", {}):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=6, settle_s=0.6, expectations=Expectations.from_dict({"dimensional": {"mode": "si"}})).run()
        a.close()
    s = r.estimates["scale_anchored"]
    assert abs(s["mean"] - 1.0) < 0.02 and r.outcome == Outcome.CONFORMANT


# ------------------------------------------------------------------ 0.1.2: RC3 adversarial review regressions
def test_rate_final_command_only_is_not_credited(master):
    # RC3 review finding 2: only 1 of every 100 commands is accepted and the arm moves continuously to it,
    # crossing every earlier target.  Feedback crossings (diagnostic) can be ~100; the verdict must come from
    # setpoint_cp: one acceptance per 100 commands -> a stale interval of ~1 s -> violated for a 50 Hz requirement.
    with mock_node("reference", {"accept_every_k": 100, "max_speed_m_s": 0.05, "loop_rate_hz": 1000, "publish_rate_hz": 200, "seed": 8}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    acc = row["acceptance"]
    assert acc["channel"] == "setpoint_cp" and acc["accepted"] <= 2 and acc["age_lower_s"] > 0.5
    assert row["feedback_count_is_evidence_of_execution"] is False
    # 0.1.4/0.1.5: the rate class carries no verdict; the archival 0.1.3 rule on the same bracket says violated
    from crtk_conformance.rate_estimator import AppliedAgeEstimate, rate_subverdict_v013_archival
    assert r.estimates["sub_verdicts"]["rate"] == "undetermined"
    assert rate_subverdict_v013_archival(AppliedAgeEstimate(**acc), TOL.required_rate_hz()) == "violated"
    assert r.outcome == Outcome.UNDETERMINED


def test_rate_without_setpoint_channel_is_undetermined(master):
    # the SRC emulations publish no setpoint_cp: no rate verdict, whatever the feedback shows
    with mock_node("emul-src-v2", {}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    assert row["acceptance"]["channel"] == "none"
    assert r.estimates["sub_verdicts"]["rate"] == "undetermined"
    assert r.outcome == Outcome.UNDETERMINED


def test_rate_accepted_channel_satisfied_on_reference(master):
    with mock_node("reference", {"loop_rate_hz": 1000, "publish_rate_hz": 500, "seed": 9}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    acc = r.observations["effective_rate"]["per_rate"][0]["acceptance"]
    assert acc["accepted"] >= 0.9 * acc["commands_sent"] and acc["age_upper_s"] <= 0.02
    # 0.1.4/0.1.5: reported as a diagnostic only; the archival 0.1.3 rule on this bracket would have said satisfied
    from crtk_conformance.rate_estimator import AppliedAgeEstimate, rate_subverdict_v013_archival
    assert r.estimates["sub_verdicts"]["rate"] == "undetermined"
    assert rate_subverdict_v013_archival(AppliedAgeEstimate(**acc), TOL.required_rate_hz()) == "satisfied"


def test_rate_fifty_percent_drops_violate_the_stale_interval_criterion(master):
    # random 50 % drops of a 100 Hz stream leave stale intervals well above the 20 ms period a 50 Hz requirement allows
    with mock_node("reference", {"drop_prob": 0.5, "loop_rate_hz": 1000, "publish_rate_hz": 500, "seed": 10}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    acc = r.observations["effective_rate"]["per_rate"][0]["acceptance"]
    assert acc["age_lower_s"] > 0.02
    assert r.estimates["sub_verdicts"]["rate"] in ("violated", "undetermined")  # violated unless the probe's own send loop stalled


def test_rate_delayed_application_is_charged_to_the_implementation(master):
    # RC4 review finding 1: a 50 ms application delay ages every applied setpoint by 50 ms (2.5 mm at 50 mm/s):
    # violated for a 1 mm / 50 mm/s client; the 0.1.2 channel reported receipt and passed it
    with mock_node("reference", {"response_delay_s": 0.05, "loop_rate_hz": 1000, "publish_rate_hz": 500, "seed": 11}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    acc = r.observations["effective_rate"]["per_rate"][0]["acceptance"]
    assert acc["age_lower_s"] > 0.045 and acc["status"] == "ok"
    assert r.estimates["sub_verdicts"]["rate"] in ("violated", "undetermined")
    assert "trace" in r.observations["effective_rate"]["per_rate"][0]


def test_hold_expectation_beyond_tested_horizon_is_undetermined(master):
    with mock_node("reference", {}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.4, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "hold", "horizon_s": 3.0}})).run()
        a.close()
    assert r.observations["liveness"]["stop_class"] == "held_through_range"
    assert r.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"


def test_release_expectation_is_undetermined_from_pose_alone(master):
    with mock_node("emul-ambf-object-watchdog"):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=2, gap_max_s=1.0, bisection_steps=3, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "release"}})).run()
        a.close()
    assert r.observations["liveness"]["stop_class"] == "drifted"
    assert r.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"
    assert r.outcome == Outcome.UNDETERMINED


def test_fault_expected_but_timeout_beyond_horizon_is_not_violated(master):
    # a fault policy at 1.5 s, client claims a fault within 0.5 s: within the tested 0.6 s nothing trips -> violated
    # for the declared horizon; with horizon 2 s (beyond the tested range) -> undetermined
    with mock_node("reference", {"watchdog_s": 1.5, "watchdog_mode": "fault"}):
        a = adapter()
        r1 = RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.6, bisection_steps=2, rates_hz=(100,),
                                  expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": 0.5}})).run()
        r2 = RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.6, bisection_steps=2, rates_hz=(100,),
                                  expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": 2.0}})).run()
        a.close()
    assert r1.estimates["sub_verdicts"]["stop_behaviour"] == "violated"
    assert r2.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"


def test_liveness_margin_uses_the_interval_not_a_point(master):
    # RC3 review finding 3: a 50 ms fault policy and a client whose period + jitter (48 ms) sits inside the
    # timeout interval must not be approved; a client at 100 Hz + 5 ms (15 ms) is approved
    with mock_node("reference", {"watchdog_s": 0.05, "watchdog_mode": "fault"}):
        a = adapter()
        tight = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=1.0 / 0.043, jitter_max_s=0.005)
        r = RateSensitivityProbe(a, tight, trials=3, gap_max_s=0.3, bisection_steps=5, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault"}})).run()
        a.close()
    tau = r.observations["liveness"]["tau_w_estimate_s"]
    assert tau["status"] in ("ok", "undetermined")
    if tau["status"] == "ok":
        assert tau["interval_low_s"] <= 0.05 <= tau["interval_high_s"], tau
    assert r.estimates["sub_verdicts"]["stop_behaviour"] != "satisfied"


def test_fault_horizon_below_across_above_the_interval(master):
    # 0.1.3 (RC4 review finding 4): a declared horizon is enforced against the tau_w INTERVAL, not the tested
    # range. Fault policy at 0.25 s: horizon 0.1 s -> violated (interval_low > horizon); horizon = 0.25 s lies inside
    # the interval -> undetermined; horizon 1.0 s -> satisfied only if interval_high <= horizon and the eq. (7)
    # margin is below interval_low. The interval is conditional on the latency allowance (run maximum) and says so.
    with mock_node("reference", {"watchdog_s": 0.25, "watchdog_mode": "fault"}):
        a = adapter()
        rs = {}
        for h in (0.1, 0.25, 1.0):
            rs[h] = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.6, bisection_steps=6, rates_hz=(100,),
                                         expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": h}})).run()
        a.close()
    tau = rs[1.0].observations["liveness"]["tau_w_estimate_s"]
    assert tau["status"] == "ok" and tau["interval_low_s"] <= 0.25 <= tau["interval_high_s"], tau
    assert "run maximum" in tau["latency_allowance_source"] and tau["conditional"] is True
    assert any("stop policy" in s for s in tau["assumptions"])
    assert tau["n_trials_with_own_last_latency"] >= tau["n"]  # every bracketing trial carries its own last-command latency
    v = {h: rs[h].estimates["sub_verdicts"]["stop_behaviour"] for h in rs}
    assert v[0.1] == "violated", v
    assert v[0.25] != "violated", v
    if rs[0.25].observations["liveness"]["tau_w_estimate_s"]["interval_high_s"] > 0.25:
        assert v[0.25] == "undetermined", v
    assert v[1.0] == "satisfied", (v, tau)
    assert rs[1.0].outcome == Outcome.CONFORMANT and rs[0.1].outcome == Outcome.DIVERGENT


def test_drift_horizon_uses_the_onset_interval(master):
    # release with drift at 0.5 s (AMBF object-layer emulation): a client expecting drift within 0.2 s is violated
    # (onset lower bound beyond the horizon); within 1.5 s satisfied (onset upper bound within it); the onset lower
    # bound carries the drift-speed assumption
    with mock_node("emul-ambf-object-watchdog"):
        a = adapter()
        r_lo = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=1.0, bisection_steps=3, rates_hz=(100,),
                                    expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "drift", "horizon_s": 0.2}})).run()
        r_hi = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=1.0, bisection_steps=3, rates_hz=(100,),
                                    expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "drift", "horizon_s": 1.5}})).run()
        a.close()
    for r in (r_lo, r_hi):
        assert r.observations["liveness"]["stop_class"] == "drifted"
    tau = r_hi.observations["liveness"]["tau_w_estimate_s"]
    assert tau["status"] == "ok" and tau["interval_low_s"] <= 0.5 <= tau["interval_high_s"], tau
    assert any("drift speed" in s for s in tau["assumptions"])
    assert r_lo.estimates["sub_verdicts"]["stop_behaviour"] == "violated", r_lo.observations["liveness"]
    assert r_hi.estimates["sub_verdicts"]["stop_behaviour"] == "satisfied", r_hi.observations["liveness"]


def test_fifo_queue_slower_than_the_client_is_violated_and_logged(master, tmp_path):
    # RC4 review, finding 1: an ordered buffer applying one command per 20 ms tick while the client sends every
    # 10 ms; the channel changes every 20 ms (the 0.1.2 statistic passed it) but the applied command ages by ~0.5 s
    # over the 1 s window.  The mock's event log is the estimator-independent truth (finding 5): the largest source
    # age of an applied command, from the send stamps in the probe's trace and the apply events, must lie inside
    # the estimator's bracket
    log = tmp_path / "events.jsonl"
    with mock_node("reference", {"queue_policy": "fifo", "loop_rate_hz": 50, "publish_rate_hz": 500, "event_log_path": str(log), "seed": 9}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.3, bisection_steps=2, rates_hz=(100,),
                                 expectations=Expectations.from_dict({"temporal": {"rate": "required"}})).run()
        a.close()
    row = r.observations["effective_rate"]["per_rate"][0]
    acc = row["acceptance"]
    assert acc["status"] == "ok" and acc["age_lower_s"] > 0.3, acc
    from crtk_conformance.rate_estimator import AppliedAgeEstimate, rate_subverdict_v013_archival
    assert r.estimates["sub_verdicts"]["rate"] == "undetermined"  # 0.1.4/0.1.5: no rate verdict
    assert rate_subverdict_v013_archival(AppliedAgeEstimate(**acc), TOL.required_rate_hz()) == "violated"
    # truth from the event log, matched by the client's header stamp
    import json as _json
    events = [_json.loads(l) for l in open(log)]
    tr = row["trace"]
    by_stamp = {round(s, 6): t for s, t in zip(tr["send_stamps_wall"], tr["send_times_mono"])}
    applies = sorted((e["t"], by_stamp.get(round(e["stamp"], 6))) for e in events if e["event"] == "apply" and round(e["stamp"], 6) in by_stamp)
    assert len(applies) >= 50, len(applies)
    # the truth window: from the first send to the application of the last target or the end of the observation;
    # the source age just before each application (the previously applied command's age), the pre-window lead
    # (nothing applied yet: age from the first send) and the age at the window end
    w0 = tr["send_times_mono"][0]
    s_last = tr["send_times_mono"][-1]
    t_last_apply = next((t for t, s in applies if s == s_last), None)
    w1 = min(t_last_apply, tr["window_end_mono"]) if t_last_apply is not None else tr["window_end_mono"]
    inwin = [(t, s) for t, s in applies if t <= w1]
    ages = [inwin[0][0] - w0] + [inwin[k][0] - inwin[k - 1][1] for k in range(1, len(inwin))] + [w1 - inwin[-1][1]]
    true_sup = max(ages)
    assert acc["window_start_s"] == pytest.approx(w0, abs=1e-6) and acc["window_end_s"] <= tr["window_end_mono"] + 1e-6
    assert acc["age_lower_s"] - 1e-6 <= true_sup <= acc["age_upper_s"] + 1e-6, (acc["age_lower_s"], true_sup, acc["age_upper_s"])


def test_skip_rate_sweep_runs_and_leaves_the_rate_diagnostic_empty(master):
    # 0.1.6 (RC9): --skip-rate-sweep raised KeyError('per_rate') when forming the sub-verdicts (found on the first
    # dVRK-sim campaign, T_fault025, all three launches)
    with mock_node("reference", {"watchdog_s": 0.25, "watchdog_mode": "fault", "seed": 12}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=3, gap_max_s=0.6, bisection_steps=3, skip_rate_sweep=True,
                                 expectations=Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": 0.1}})).run()
        a.close()
    assert r.observations["effective_rate"]["skipped"] is True
    assert r.estimates["sub_verdicts"]["stop_behaviour"] == "violated"   # a 0.25 s fault policy cannot satisfy a 0.1 s horizon
    assert r.estimates["sub_verdicts"]["rate"] is None
