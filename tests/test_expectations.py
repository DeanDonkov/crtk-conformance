"""Client-expectation semantics (0.1.1, Reviewer #2 M3): no expectation -> no conformance verdict."""
import math
import os
import tempfile

import numpy as np
import pytest

from crtk_conformance import geometry as G
from crtk_conformance.expectations import Expectations, ExpectationError, combine
from crtk_conformance.probes.base import Outcome, decide
from crtk_conformance.thresholds import Tolerance

TOL = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=100.0, jitter_max_s=0.005)


def residual_error(T_obs, T_exp):
    E = T_obs @ G.invert(T_exp)
    return TOL.spatial_error(float(np.linalg.norm(E[:3, 3])), G.rotation_angle(E[:3, :3]))


def test_default_is_discover_only():
    e = Expectations()
    assert not e.spatial.declared and not e.dimensional.declared and not e.temporal.declared
    assert e.spatial.matrix() is None
    assert combine({"state_machine": None, "stop_behaviour": None, "rate": None}) == "undetermined"


def test_identity_expectation_matches_identity_binding():
    e = Expectations.from_dict({"spatial": {"mode": "identity"}})
    assert e.spatial.declared
    assert residual_error(np.eye(4), e.spatial.matrix()) == 0.0
    assert decide(0.0, 0.0, TOL.epsilon_m) == Outcome.CONFORMANT
    # a 2 mm binding against an identity expectation is divergent at 1 mm
    T = G.make_pose(None, [0.002, 0, 0])
    err = residual_error(T, e.spatial.matrix())
    assert abs(err - 0.002) < 1e-12
    assert decide(err, err, TOL.epsilon_m) == Outcome.DIVERGENT


def test_non_identity_expected_transform_is_conformant_when_matched():
    # JHU-like binding: 0.20 m along x, -150 deg about x
    R = G.axis_angle([1, 0, 0], math.radians(-150.0))
    T_true = G.make_pose(R, [0.20, 0, 0])
    x, y, z, w = G.rot_to_quat(R)
    e = Expectations.from_dict({"spatial": {"mode": "expected_transform", "translation_m": [0.20, 0, 0], "quaternion_xyzw": [x, y, z, w]}})
    err = residual_error(T_true, e.spatial.matrix())
    assert err < 1e-9
    assert decide(err, err, TOL.epsilon_m) == Outcome.CONFORMANT
    # the same binding against an identity expectation: 200 mm + 2 sin(75 deg) * 0.1 m = 393 mm -> divergent
    err_id = residual_error(T_true, np.eye(4))
    assert abs(err_id - (0.20 + 2 * math.sin(math.radians(75)) * 0.1)) < 1e-9
    assert decide(err_id, err_id, TOL.epsilon_m) == Outcome.DIVERGENT
    # and a wrong sign of the rotation (the 0.1.0 preset error) is divergent against the correct expectation
    T_wrong = G.make_pose(G.axis_angle([1, 0, 0], math.radians(150.0)), [0.20, 0, 0])
    err_w = residual_error(T_wrong, e.spatial.matrix())
    assert err_w > 0.05


def test_missing_expectation_gives_undetermined_even_for_identity_binding():
    e = Expectations()
    assert e.spatial.matrix() is None  # the probe then reports T_hat and returns UNDETERMINED
    assert combine({"state_machine": None}) == "undetermined"


def test_temporal_required_forbidden_any():
    assert combine({"state_machine": "satisfied", "stop_behaviour": None, "rate": None}) == "conformant"
    assert combine({"state_machine": "violated", "stop_behaviour": "satisfied", "rate": None}) == "divergent"
    assert combine({"state_machine": "satisfied", "stop_behaviour": "undetermined", "rate": None}) == "undetermined"
    assert combine({"state_machine": None, "stop_behaviour": None, "rate": None}) == "undetermined"
    e = Expectations.from_dict({"temporal": {"state_machine": "required"}})
    assert e.temporal.declared and e.temporal.stop_behaviour == "any"
    e = Expectations.from_dict({"temporal": {"state_machine": "forbidden", "stop_behaviour": "hold", "rate": "required"}})
    assert e.temporal.declared


def test_dimensional_expectation():
    e = Expectations.from_dict({"dimensional": {"mode": "si"}})
    assert e.dimensional.declared and e.dimensional.expected_unit_m == 1.0
    e = Expectations.from_dict({"dimensional": {"mode": "si", "expected_unit_m": 0.1}})
    assert e.dimensional.expected_unit_m == 0.1
    # a 0.1 m/unit interface against a metre client: |1 - 0.1| * 0.1 m = 90 mm -> divergent
    err = TOL.dimensional_error(0.1 / 1.0)
    assert decide(err, err, TOL.epsilon_m) == Outcome.DIVERGENT
    # the same interface against a client that assumes 0.1 m/unit: conformant
    assert decide(TOL.dimensional_error(0.1 / 0.1), TOL.dimensional_error(0.1 / 0.1), TOL.epsilon_m) == Outcome.CONFORMANT


def test_yaml_loading_and_validation():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "e.yaml")
        with open(p, "w") as f:
            f.write("spatial:\n  mode: expected_transform\n  translation_m: [0.0, 0.0, 0.0]\n  quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]\ntemporal:\n  state_machine: required\n")
        e = Expectations.load(p)
        assert e.spatial.declared and np.allclose(e.spatial.matrix(), np.eye(4))
        assert e.temporal.state_machine == "required"
        with open(p, "w") as f:
            f.write("spatial:\n  mode: nonsense\n")
        with pytest.raises(ExpectationError):
            Expectations.load(p)
        with open(p, "w") as f:
            f.write("temporal:\n  stop_behaviour: drift\n  horizon_s: 1.0\n")
        e = Expectations.load(p)  # 0.1.2: drift and release are declarable stop behaviours; horizon_s is optional
        assert e.temporal.stop_behaviour == "drift" and e.temporal.horizon_s == 1.0
        with open(p, "w") as f:
            f.write("temporal:\n  stop_behaviour: explode\n")
        with pytest.raises(ExpectationError):
            Expectations.load(p)
        with open(p, "w") as f:
            f.write("temporal:\n  stop_behaviour: hold\n  horizon_s: -1\n")
        with pytest.raises(ExpectationError):
            Expectations.load(p)


def test_boundary_guard_band():
    eps = 0.005
    assert decide(0.0049999999999999975, 0.0049999999999999975, eps) == Outcome.UNDETERMINED
    assert decide(4.999999999999999e-3, 4.999999999999999e-3, eps) == Outcome.UNDETERMINED
    assert decide(0.0049, 0.00499, eps) == Outcome.CONFORMANT
    assert decide(0.00501, 0.0051, eps) == Outcome.DIVERGENT
    assert decide(0.004, 0.006, eps) == Outcome.UNDETERMINED
