"""Offline replay of the temporal verdict logic (0.1.3) on archived observations, without ROS: the data-collection
methods of RateSensitivityProbe are replaced by stubs returning archived observations (the technique of the RC4
adversarial review), so that the decision code of run() is exercised unchanged.

RC4 review finding 4: a fault policy at 1.0 s satisfied a 'fault within 0.1 s' expectation in 0.1.2.
"""
import json
import os
import sys
import types

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ARCHIVE = os.path.join(HERE, "..", "validation", "v0.1.2", "mock")


def _stub_ros():
    if "crtk_conformance.adapter" not in sys.modules or not hasattr(sys.modules["crtk_conformance.adapter"], "_stub"):
        try:
            import rospy  # noqa: F401
            return  # a real ROS environment: nothing to stub
        except ImportError:
            pass
        adapter = types.ModuleType("crtk_conformance.adapter"); adapter._stub = True
        adapter.PlatformAdapter = object; adapter.pose_msg_to_matrix = lambda x: x
        sys.modules[adapter.__name__] = adapter
        common = types.ModuleType("crtk_conformance.probes.common")
        common.ensure_enabled = lambda *a, **k: None
        common.wait_settled = lambda *a, **k: (np.eye(4), None)
        sys.modules[common.__name__] = common


class OfflineAdapter:
    discovery = {"topics": {x: {"present": True} for x in ("measured_cp", "servo_cp")}}

    def subscribe(self, *a):
        return object()

    def wait_for(self, *a):
        return np.eye(4)

    def has(self, name):
        return False

    def latest_pose(self, *a):
        return np.eye(4)


def _replay(archive_file, expectation, gap_max_s=1.5):
    _stub_ros()
    from crtk_conformance.expectations import Expectations
    from crtk_conformance.probes.rate import RateSensitivityProbe
    from crtk_conformance.thresholds import Tolerance
    import crtk_conformance.probes.rate as rate_mod
    # in a real ROS environment the settling helpers are the real ones: replace them on the probe module too
    rate_mod.ensure_enabled = lambda *a, **k: None
    rate_mod.wait_settled = lambda *a, **k: (np.eye(4), None)
    r = json.load(open(os.path.join(ARCHIVE, archive_file)))
    obs = r["result"]["observations"]
    probe = RateSensitivityProbe(OfflineAdapter(), Tolerance(), expectations=Expectations.from_dict(expectation), gap_max_s=gap_max_s)
    probe.measure_resolution = lambda *a: obs["resolution"]
    probe.probe_state_precondition = lambda *a: obs["state_precondition"]
    probe.probe_liveness = lambda *a: obs["liveness"]
    probe.probe_effective_rate = lambda *a: {"per_rate": []}
    return probe.run(), obs


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_fault_008.json")), reason="v0.1.2 archive not present")
def test_rc4_reviewer_fault_policy_after_the_horizon_is_violated():
    # archived 1.0 s fault policy, interval [0.980, 1.010] s; the client expects a fault within 0.1 s
    res, obs = _replay("L_fault_008.json", {"temporal": {"stop_behaviour": "fault", "horizon_s": 0.1}})
    assert obs["liveness"]["tau_w_estimate_s"]["interval_low_s"] > 0.1
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "violated"
    assert res.outcome.value == "divergent"


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_fault_008.json")), reason="v0.1.2 archive not present")
def test_fault_horizon_across_and_above_the_interval():
    res, _ = _replay("L_fault_008.json", {"temporal": {"stop_behaviour": "fault", "horizon_s": 1.0}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"  # the interval straddles the horizon
    res, _ = _replay("L_fault_008.json", {"temporal": {"stop_behaviour": "fault", "horizon_s": 1.5}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "satisfied"  # fires by 1.01 s < 1.5 s; margin 15 ms < 0.98 s


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_release_002_drift.json")), reason="v0.1.2 archive not present")
def test_drift_horizon_below_across_above():
    # archived release-with-drift at 0.5 s (interval ~[0.49, 0.52] s)
    res, obs = _replay("L_release_002_drift.json", {"temporal": {"stop_behaviour": "drift", "horizon_s": 0.2}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "violated"
    res, _ = _replay("L_release_002_drift.json", {"temporal": {"stop_behaviour": "drift", "horizon_s": 0.5}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"
    res, _ = _replay("L_release_002_drift.json", {"temporal": {"stop_behaviour": "drift", "horizon_s": 1.0}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "satisfied"


@pytest.mark.skipif(not os.path.exists(os.path.join(ARCHIVE, "L_fault_001.json")), reason="v0.1.2 archive not present")
def test_undetermined_interval_never_satisfies_a_margin():
    # archived 15 ms fault policy, interval undetermined; the client's need is 15 ms
    res, _ = _replay("L_fault_001.json", {"temporal": {"stop_behaviour": "fault"}})
    assert res.estimates["sub_verdicts"]["stop_behaviour"] == "undetermined"
