"""Integration tests: need a ROS 1 Python stack (rospy, rosmaster, crtk_msgs). Skipped otherwise."""
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
from crtk_conformance.adapter import PlatformAdapter  # noqa: E402
from crtk_conformance.thresholds import Tolerance  # noqa: E402
from crtk_conformance.probes.frame import FrameSemanticsProbe  # noqa: E402
from crtk_conformance.probes.scale import ScalingUnitsProbe  # noqa: E402
from crtk_conformance.probes.rate import RateSensitivityProbe  # noqa: E402
from crtk_conformance.probes.base import Outcome  # noqa: E402

TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=100, jitter_max_s=0.005)


@pytest.fixture(scope="module")
def master():
    with ros_master(port=11511):
        yield


def adapter():
    a = PlatformAdapter("/PSM1", anchor_topic="/PSM1/mock/ground_truth_cp")
    a.discover()
    return a


def test_frame_probe_recovers_injected_transform(master):
    with mock_node("reference", {"bind_translation_m": [0.02, -0.01, 0.005], "bind_angle_deg": 30.0, "bind_axis": [0, 0, 1]}):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=3, samples_per_trial=3).run()
        a.close()
    assert r.outcome == Outcome.DIVERGENT
    t = np.array(r.estimates["binding_translation_m"])
    assert np.linalg.norm(t - np.array([0.02, -0.01, 0.005])) < 1e-6
    assert abs(r.estimates["binding_rotation_deg"]["mean"] - 30.0) < 1e-3


def test_frame_probe_undetermined_without_local(master):
    with mock_node("emul-src-v1"):
        a = adapter()
        r = FrameSemanticsProbe(a, TOL, trials=2).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED


def test_scale_probe_internal_ratio_is_one_but_anchor_recovers_scale(master):
    with mock_node("emul-src-v1"):
        a = adapter()
        r = ScalingUnitsProbe(a, TOL, trials=3, settle_s=0.5).run()
        a.close()
    assert abs(r.estimates["internal_ratio"]["mean"] - 1.0) < 1e-6  # feedback invisibility
    assert abs(r.estimates["scale_anchored"]["mean"] - 0.1) < 1e-6
    assert r.outcome == Outcome.DIVERGENT


def test_scale_probe_undetermined_without_anchor(master):
    with mock_node("emul-src-v1"):
        a = PlatformAdapter("/PSM1"); a.discover()
        r = ScalingUnitsProbe(a, TOL, trials=3, settle_s=0.5).run()
        a.close()
    assert r.outcome == Outcome.UNDETERMINED


def test_liveness_estimate(master):
    with mock_node("reference", {"watchdog_s": 0.25, "watchdog_mode": "fault"}):
        a = adapter()
        r = RateSensitivityProbe(a, TOL, trials=2, gap_max_s=0.6, bisection_steps=5, rates_hz=(100,)).run()
        a.close()
    tau = r.observations["liveness"]["tau_w_estimate_s"]
    assert tau is not None and abs(tau["mean"] - 0.25) < 0.03


def test_missing_topic_gives_undetermined_not_conformant(master):
    with mock_node("reference", {"publish_measured_cp": False}):
        a = adapter()
        rs = [FrameSemanticsProbe(a, TOL, trials=2).run(), ScalingUnitsProbe(a, TOL, trials=2).run(), RateSensitivityProbe(a, TOL, trials=1, gap_max_s=0.2, rates_hz=(100,)).run()]
        a.close()
    assert all(r.outcome == Outcome.UNDETERMINED for r in rs)
