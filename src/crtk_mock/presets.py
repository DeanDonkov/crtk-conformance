"""Presets: emulations of *documented* behaviours of released artifacts.

Each preset is built only from source lines cited in the paper's evidence ledger (evidence_ledger.csv,
rows named below). A preset is an emulation of those declarations in a kinematic mock; it is not the
platform, it does not run the platform's code, and nothing measured against it is a measurement of the
platform.  Parameters that no artifact declares (e.g. the drift speed of the release preset) are labelled
VALIDATION PARAMETER: they exist so that a behaviour class can be exercised, and their values are arbitrary.
"""
from __future__ import annotations

from .node import MockConfig

PRESETS = {
    # CRTK read literally: operating-state machine present, arm-base binding, SI, no liveness timeout.
    "reference": MockConfig(),
    # Emulation of the JHU released PSM2 configuration (ledger P19: dvrk_config_jhu 53d6a12d, system-MTMR-PSM2-Teleop.json l.22-28):
    # base_frame rows [1,0,0],[0,-0.866,0.5],[0,-0.5,-0.866] = rotation of -150 deg about x (cos = -0.866, sin = -0.5), translation
    # 0.20 m along x.  The mock's T_bind plays the role of base_frame (measured_cp = T_bind * local), so the sign is -150 deg
    # (0.1.0 used +150 deg, the transpose; corrected in 0.1.1, Reviewer #2 minor 1).  local/ published (P18, l.269); servo_cp
    # rejected unless ready (P27, l.2191); publish 100 Hz, arm loop 1.5 kHz (P28).
    "emul-dvrk-jhu-psm2": MockConfig(
        bind_translation_m=[0.20, 0.0, 0.0], bind_axis=[1.0, 0.0, 0.0], bind_angle_deg=-150.0,
        publish_local=True, measured_frame_id="ECM", local_frame_id="PSM1_base",
        state_machine=True, require_enabled=True, watchdog_s=0.0, loop_rate_hz=1500.0, publish_rate_hz=100.0,
    ),
    # Emulation of the SRC v1.0.0 CRTK interface (ledger P21, P22, P30, P31, P01, P04): arm-base binding (psm_arm.py l.160-166,
    # 203-206), frame_id '<psm>/baselink' and a T_b_w topic (launch_crtk_interface.py l.138-140), no local/ topic, no
    # operating_state, interface units of 0.1 m (psmFK.py l.70-71; psm_arm.py l.106); servo_cp executed in the subscriber
    # callback (l.143-145) and republished to the simulator by the AMBF client at 120 Hz latest-wins (P34); publishing loop 120 Hz (l.335).
    "emul-src-v1": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        unit_m=0.1, state_machine=False, require_enabled=False, watchdog_s=0.0, loop_rate_hz=120.0, publish_rate_hz=120.0,
        publish_setpoint_cp=False,  # no setpoint_cp on the SRC interface (live inventory, ML101)
    ),
    # Same as above with SI units (v2.0.0: units_conversion.py l.6, psm_400006.json l.32; CHANGELOG.md l.29).
    "emul-src-v2": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        unit_m=1.0, state_machine=False, require_enabled=False, watchdog_s=0.0, loop_rate_hz=120.0, publish_rate_hz=120.0,
        publish_setpoint_cp=False,  # no setpoint_cp on the SRC interface (live inventory, ML201)
    ),
    # Emulation of the AMBF object-layer command watchdog (ledger P32, P33: ambf d816d708 ObjectCommPlugin.cpp l.100 timeOut = 0.5;
    # RigidBodyRosCom.cpp l.69-84 reset to zero effort). Exposed here directly on the CRTK topics so that the liveness
    # probe can be validated; on the real SRC interface it is masked by the client republisher (P34; paper Sec. 4.3).
    # release_drift_m_s = 0.02 is a VALIDATION PARAMETER (no artifact declares a drift speed; "zero effort" leaves the
    # motion to the simulated dynamics).
    "emul-ambf-object-watchdog": MockConfig(
        publish_local=False, publish_T_b_w=True, measured_frame_id="psm1/baselink",
        state_machine=False, require_enabled=False, watchdog_s=0.5, watchdog_mode="release", release_drift_m_s=0.02,
        loop_rate_hz=120.0, publish_rate_hz=120.0, publish_setpoint_cp=False,
    ),
}


def get(name: str) -> MockConfig:
    if name not in PRESETS:
        raise KeyError(f"unknown preset {name!r}; available: {sorted(PRESETS)}")
    import copy

    return copy.deepcopy(PRESETS[name])
