"""Mock configuration (ROS-free).

MockConfig is the parameter set of the kinematic mock (crtk_mock.node) and of the presets (crtk_mock.presets). It
lives in its own module so that analysis scripts can read preset parameters without a ROS installation (RC3 review:
the analyzer imported rospy through crtk_mock.presets).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import List

import numpy as np

from crtk_conformance import geometry as G


@dataclass
class MockConfig:
    namespace: str = "/PSM1"
    # spatial
    bind_translation_m: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bind_axis: List[float] = field(default_factory=lambda: [1.0, 0.0, 0.0])
    bind_angle_deg: float = 0.0
    publish_local: bool = True  # publish local/measured_cp (dVRK style)
    publish_T_b_w: bool = False  # publish T_b_w (SRC style)
    publish_tf: bool = False  # publish /tf world -> <frame of measured_cp>
    measured_frame_id: str = "PSM1_base"
    local_frame_id: str = "PSM1_base"
    # dimensional
    unit_m: float = 1.0  # metres per interface unit
    # temporal
    state_machine: bool = True  # publish operating_state and accept state_command
    require_enabled: bool = True  # reject servo_cp unless ENABLED and homed
    watchdog_s: float = 0.0  # 0 = no liveness policy
    watchdog_mode: str = "fault"  # 'fault' -> state FAULT, commands rejected until 'enable'; 'release' -> stop tracking until next command
    release_drift_m_s: float = 0.0  # drift while released (emulates loss of actuation); 0 = hold
    loop_rate_hz: float = 1000.0  # execution loop; the latest *due* command wins per tick
    publish_rate_hz: float = 100.0  # measured_cp publish rate
    response_delay_s: float = 0.0  # fixed delay between command receipt and execution (commands are queued, not discarded)
    response_jitter_s: float = 0.0  # uniform jitter added to the delay
    drop_prob: float = 0.0  # probability of ignoring a servo_cp message
    noise_m: float = 0.0  # Gaussian noise (per axis, metres) added to measured_cp / local/measured_cp positions
    orientation_noise_deg: float = 0.0  # Gaussian noise (deg, random axis) added to the published orientations (0.1.1)
    stamp_skew_s: float = 0.0  # header.stamp offset applied to local/measured_cp relative to measured_cp (0.1.1)
    max_speed_m_s: float = 0.0  # 0 = setpoints are attained instantaneously; >0 = move toward the goal at this speed (0.1.1)
    anchor_noise_m: float = 0.0  # Gaussian noise added to the out-of-band ground-truth (anchor) topic, validation only (0.1.1)
    accept_every_k: int = 1  # 0.1.2: accept only every k-th received servo_cp (the others are silently ignored); 1 = all
    publish_setpoint_cp: bool = True  # 0.1.2: publish the low-level setpoint on setpoint_cp (the dVRK does; the SRC does not); 0.1.3: the goal currently APPLIED by the execution loop, not the last received (CRTK: 'current setpoint to low-level controller')
    event_log_path: str = ""  # 0.1.3 (validation only): JSON-lines log of every command receipt / drop / rejection / application, with the command's header.stamp, for an estimator-independent truth
    # misc
    publish_measured_cp: bool = True  # False emulates a missing topic
    publish_measured_js: bool = True
    seed: int = 0
    home_pose_m: List[float] = field(default_factory=lambda: [0.0, 0.0, -0.10])

    def bind_T(self) -> np.ndarray:
        R = G.axis_angle(self.bind_axis, math.radians(self.bind_angle_deg))
        return G.make_pose(R, self.bind_translation_m)

    def to_dict(self):
        return asdict(self)
