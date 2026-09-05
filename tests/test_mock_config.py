"""crtk_mock's configuration and presets are importable without ROS (0.1.2 packaging; RC3 review: the analyzer
imported rospy through crtk_mock.presets)."""
import numpy as np


def test_presets_importable_without_rospy():
    import crtk_mock
    from crtk_mock.presets import PRESETS, get
    cfg = get("emul-dvrk-jhu-psm2")
    T = cfg.bind_T()
    assert np.allclose(T[:3, 3], [0.20, 0.0, 0.0]) and abs(T[1, 1] + 0.866) < 1e-3
    assert set(PRESETS) >= {"reference", "emul-dvrk-jhu-psm2", "emul-src-v1", "emul-src-v2", "emul-ambf-object-watchdog"}
    assert crtk_mock.MockConfig().accept_every_k == 1 and crtk_mock.get("emul-src-v1").publish_setpoint_cp is False


def test_config_module_has_no_ros_dependency():
    import importlib
    src = open(importlib.import_module("crtk_mock.config").__file__).read()
    assert "import rospy" not in src and "from .node" not in src
