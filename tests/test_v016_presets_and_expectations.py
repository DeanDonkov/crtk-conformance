"""0.1.6: new mock options default to the archived behaviour; every 0.1.5 preset is unchanged (RC9 WP0), and the
geometry_anchor expectation parses and validates."""
import json
import os

import numpy as np
import pytest

HERE = os.path.dirname(__file__)


def test_presets_unchanged_since_v015():
    from crtk_mock.presets import PRESETS
    from crtk_mock.config import MockConfig
    old = json.load(open(os.path.join(HERE, "fixtures_presets_v015.json")))
    new = {k: v.to_dict() for k, v in PRESETS.items()}
    new["__default__"] = MockConfig().to_dict()
    assert set(old) == set(new)
    for name, d in old.items():
        for k, v in d.items():
            assert new[name][k] == v, (name, k)
        # new 0.1.6 keys hold the values that reproduce the archived behaviour
        extra = {k: new[name][k] for k in new[name] if k not in d}
        assert extra["cmd_bind_translation_m"] is None and extra["cmd_bind_axis"] is None and extra["cmd_bind_angle_deg"] is None
        assert extra["noise_model"] == "gaussian" and extra["drop_model"] == "iid"


def test_command_binding_defaults_to_feedback_binding():
    from crtk_mock.presets import get
    cfg = get("emul-dvrk-jhu-psm2")
    assert np.allclose(cfg.cmd_bind_T(), cfg.bind_T())
    cfg.cmd_bind_translation_m = [0.02, 0.0, 0.0]
    cfg.cmd_bind_axis = [0, 0, 1]
    cfg.cmd_bind_angle_deg = 10.0
    assert not np.allclose(cfg.cmd_bind_T(), cfg.bind_T())


def test_geometry_anchor_expectation_parses():
    from crtk_conformance.expectations import Expectations, ExpectationError
    e = Expectations.from_dict({"dimensional": {"mode": "geometry_anchor", "L_m": 0.0091, "u_rel": 0.015, "expected_unit_m": 1.0,
                                                "L_source": "dVRK 2.4.0 LARGE_NEEDLE_DRIVER_400006.json"}})
    assert e.dimensional.declared and e.dimensional.L_m == 0.0091 and e.dimensional.u_rel == 0.015
    assert e.dimensional.pitch_joint == "wrist_pitch" and e.dimensional.axes_angle_deg == 90.0
    with pytest.raises(ExpectationError):
        Expectations.from_dict({"dimensional": {"mode": "geometry_anchor", "u_rel": 0.015}})
    with pytest.raises(ExpectationError):
        Expectations.from_dict({"dimensional": {"mode": "geometry_anchor", "L_m": 0.0091}})
