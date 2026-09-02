"""Presets: emulations of *documented* behaviours of released artifacts.

Each preset is built only from source lines cited in PRIMARY_SOURCES.md of the paper. A preset is
an emulation of those declarations in a kinematic mock; it is not the platform, it does not run
the platform's code, and nothing measured against it is a measurement of the platform.
"""
from __future__ import annotations

from .node import MockConfig

PRESETS = {
    # CRTK read literally: operating-state machine present, arm-base binding, SI, no liveness timeout.
    "reference": MockConfig(),
    # Emulation of the JHU released PSM2 configuration (dvrk_config_jhu 2.5.0, system-MTMR-PSM2-Teleop.json l.22-28):
    # base_frame = rotation of 150 deg about x, translation 0.20 m along x; local/ published (mtsIntuitiveResearchKitArm.cpp l.269);
    # servo_cp rejected unless ready (l.2191); publish 100 Hz, arm loop 1.5 kHz (dvrk-readthedocs middleware.rst l.73-75).
    "emul-dvrk-jhu-psm2": MockConfig(
        bind_translation_m=[0.20, 0.0, 0.0], bind_axis=[1.0, 0.0, 0.0], bind_angle_deg=150.0,
        publish_local=True, measured_frame_id="ECM", local_frame_id="PSM1_base",
        state_machine=True, require_enabled=True, watchdog_s=0.0, loop_rate_hz=1500.0, publish_rate_hz=100.0,
    ),
    # Emulation of the SRC v1.0.0 CRTK interface (surgical_robotics_challenge v1.0.0): arm-base binding (psm_arm.py l.160-166,
    # 203-206), frame_id '<psm>/baselink' and a T_b_w topic (launch_crtk_interface.py l.138-140), no local/ topic, no
    # operating_state, interface units of 0.1 m (psmFK.py l.70-71; psm_arm.py l.106), 120 Hz latest-wins loop (l.100, 335).
    "emul-src-v1": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        unit_m=0.1, state_machine=False, require_enabled=False, watchdog_s=0.0, loop_rate_hz=120.0, publish_rate_hz=120.0,
    ),
    # Same as above with SI units (v2.0.0: units_conversion.py l.6, psm_400006.json l.32; CHANGELOG.md l.29).
    "emul-src-v2": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        unit_m=1.0, state_machine=False, require_enabled=False, watchdog_s=0.0, loop_rate_hz=120.0, publish_rate_hz=120.0,
    ),
    # Emulation of the AMBF object-layer command watchdog (ambf d816d70: ObjectCommPlugin.cpp l.100 timeOut = 0.5;
    # RigidBodyRosCom.cpp l.69-84 reset to zero effort). Exposed here directly on the CRTK topics so that the liveness
    # probe can be validated; on the real SRC interface it is masked by the client republisher (see paper Sec. 4.3).
    "emul-ambf-object-watchdog": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        state_machine=False, require_enabled=False, watchdog_s=0.5, watchdog_mode="release", release_drift_m_s=0.02,
        loop_rate_hz=120.0, publish_rate_hz=120.0,
    ),
}


def get(name: str) -> MockConfig:
    if name not in PRESETS:
        raise KeyError(f"unknown preset {name!r}; available: {sorted(PRESETS)}")
    import copy

    return copy.deepcopy(PRESETS[name])
