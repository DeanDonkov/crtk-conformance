"""crtk-mock: a configurable CRTK-compatible reference node with injectable semantic divergences.

The mock is *kinematic*: it holds a tool pose in an arm-base frame whose origin is the remote
centre of motion (the frame the dVRK calls `local/` and SRC calls `<psm>/baselink`), and it
exposes the CRTK topics with a configurable semantic binding:

  spatial      T_bind  : the unqualified `measured_cp`/`servo_cp` are expressed in a frame related to
                         the arm base by T_bind (dVRK: base_frame; identity = arm-base binding)
  dimensional  unit_m  : metres per interface unit (1.0 = SI; 0.1 = the SRC v1.0.0 convention)
  temporal     state machine on/off, liveness timeout tau_w with 'fault' or 'release' semantics,
               loop rate (latest-wins execution), publish rate, response delay/jitter, drops

A ground-truth pose in SI metres in the arm-base frame is published out of band on
`<ns>/mock/ground_truth_cp`. It is the validation oracle and the unit anchor used in validation;
a probe run against a real platform never has it.

Presets (presets.py) emulate *documented* behaviours of released artifacts. They are emulations
built from the source lines cited in the paper, not the platforms.
"""
from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from crtk_msgs.msg import OperatingState, StringStamped

from crtk_conformance import geometry as G
from crtk_conformance.adapter import pose_msg_to_matrix, matrix_to_pose_msg


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
    loop_rate_hz: float = 1000.0  # execution loop; latest command wins per tick
    publish_rate_hz: float = 100.0  # measured_cp publish rate
    response_delay_s: float = 0.0  # fixed delay between command receipt and execution
    response_jitter_s: float = 0.0  # uniform jitter added to the delay
    drop_prob: float = 0.0  # probability of ignoring a servo_cp message
    noise_m: float = 0.0  # Gaussian noise (per axis, metres) added to measured_cp
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


class MockCRTKNode:
    def __init__(self, cfg: MockConfig):
        self.cfg = cfg
        self.ns = cfg.namespace.rstrip("/")
        self.T_bind = cfg.bind_T()
        self.T_bind_inv = G.invert(self.T_bind)
        self.rng = random.Random(cfg.seed)
        self.nrng = np.random.default_rng(cfg.seed)
        self.lock = threading.Lock()
        # ground-truth tool pose in the arm-base frame, SI
        self.pose = G.make_pose(None, cfg.home_pose_m)
        self.goal: Optional[np.ndarray] = None
        self.goal_due: float = 0.0
        self.state = "DISABLED" if cfg.state_machine else "ENABLED"
        self.homed = not cfg.state_machine
        self.released = False
        self.last_cmd_time: Optional[float] = None
        self.received = 0
        self.executed = 0
        self.rejected = 0
        self.stop = threading.Event()

        if not rospy.core.is_initialized():
            rospy.init_node("crtk_mock", anonymous=True, disable_signals=True)
        q = 10
        self.pub_measured = rospy.Publisher(f"{self.ns}/measured_cp", PoseStamped, queue_size=q) if cfg.publish_measured_cp else None
        self.pub_local = rospy.Publisher(f"{self.ns}/local/measured_cp", PoseStamped, queue_size=q) if cfg.publish_local else None
        self.pub_tbw = rospy.Publisher(f"{self.ns}/T_b_w", PoseStamped, queue_size=q) if cfg.publish_T_b_w else None
        self.pub_js = rospy.Publisher(f"{self.ns}/measured_js", JointState, queue_size=q) if cfg.publish_measured_js else None
        self.pub_truth = rospy.Publisher(f"{self.ns}/mock/ground_truth_cp", PoseStamped, queue_size=q)
        self.pub_state = rospy.Publisher(f"{self.ns}/operating_state", OperatingState, queue_size=q, latch=True) if cfg.state_machine else None
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=q) if cfg.publish_tf else None
        self.sub_servo = rospy.Subscriber(f"{self.ns}/servo_cp", PoseStamped, self._servo_cp_cb, queue_size=1)
        self.sub_state = rospy.Subscriber(f"{self.ns}/state_command", StringStamped, self._state_cmd_cb, queue_size=10) if cfg.state_machine else None
        self.threads = [threading.Thread(target=self._loop, daemon=True), threading.Thread(target=self._publish_loop, daemon=True)]

    # ------------------------------------------------------------------ callbacks
    def _servo_cp_cb(self, msg: PoseStamped):
        now = time.monotonic()
        with self.lock:
            self.received += 1
            if self.cfg.drop_prob > 0 and self.rng.random() < self.cfg.drop_prob:
                return
            if self.cfg.require_enabled and not (self.state == "ENABLED" and self.homed):
                self.rejected += 1
                return
            T_if = pose_msg_to_matrix(msg)  # interface units, bound frame
            T_if[:3, 3] *= self.cfg.unit_m  # -> metres
            T_local = self.T_bind_inv @ T_if  # -> arm-base frame
            delay = self.cfg.response_delay_s + self.rng.uniform(0.0, self.cfg.response_jitter_s)
            self.goal = T_local
            self.goal_due = now + delay
            self.last_cmd_time = now
            self.released = False

    def _state_cmd_cb(self, msg: StringStamped):
        cmd = msg.string.strip().lower()
        with self.lock:
            if cmd == "enable":
                if self.state != "FAULT" or True:
                    self.state = "ENABLED"
                    self.last_cmd_time = time.monotonic()
            elif cmd == "disable":
                self.state = "DISABLED"
            elif cmd == "home":
                self.homed = True
            elif cmd == "unhome":
                self.homed = False
            elif cmd == "pause":
                self.state = "PAUSED"
            elif cmd == "resume":
                self.state = "ENABLED"
        self._publish_state()

    # ------------------------------------------------------------------ loops
    def _loop(self):
        period = 1.0 / self.cfg.loop_rate_hz
        next_t = time.monotonic()
        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                # liveness policy
                if self.cfg.watchdog_s > 0 and self.last_cmd_time is not None and self.state == "ENABLED":
                    if now - self.last_cmd_time > self.cfg.watchdog_s and not self.released:
                        if self.cfg.watchdog_mode == "fault":
                            self.state = "FAULT"
                            self.goal = None
                            self._publish_state()
                        else:
                            self.released = True
                            self.goal = None
                if self.released and self.cfg.release_drift_m_s > 0:
                    self.pose[2, 3] -= self.cfg.release_drift_m_s * period
                # execute latest command (latest wins)
                if self.goal is not None and now >= self.goal_due and self.state == "ENABLED":
                    self.pose = self.goal
                    self.goal = None
                    self.executed += 1
            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    def _publish_state(self):
        if self.pub_state is None:
            return
        m = OperatingState()
        m.header.stamp = rospy.Time.now()
        m.state = self.state
        m.is_homed = self.homed
        m.is_busy = False
        self.pub_state.publish(m)

    def _publish_loop(self):
        period = 1.0 / self.cfg.publish_rate_hz
        next_t = time.monotonic()
        k = 0
        while not self.stop.is_set():
            with self.lock:
                pose = self.pose.copy()
            # unqualified topic: bound frame, interface units, noise
            T_if = self.T_bind @ pose
            if self.cfg.noise_m > 0:
                T_if[:3, 3] += self.nrng.normal(0.0, self.cfg.noise_m, 3)
            T_if[:3, 3] /= self.cfg.unit_m
            if self.pub_measured is not None:
                self.pub_measured.publish(matrix_to_pose_msg(T_if, self.cfg.measured_frame_id))
            if self.pub_local is not None:
                T_loc = pose.copy()
                if self.cfg.noise_m > 0:
                    T_loc[:3, 3] += self.nrng.normal(0.0, self.cfg.noise_m, 3)
                T_loc[:3, 3] /= self.cfg.unit_m
                self.pub_local.publish(matrix_to_pose_msg(T_loc, self.cfg.local_frame_id))
            if self.pub_tbw is not None:
                # base-in-world; in the mock the "world" is the bound frame
                self.pub_tbw.publish(matrix_to_pose_msg(self.T_bind, "world"))
            if self.pub_js is not None:
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = ["outer_yaw", "outer_pitch", "outer_insertion"]
                js.position = [0.0, 0.0, float(np.linalg.norm(pose[:3, 3]) / self.cfg.unit_m)]
                self.pub_js.publish(js)
            self.pub_truth.publish(matrix_to_pose_msg(pose, "mock_ground_truth_base"))
            if self.pub_tf is not None:
                tr = TransformStamped()
                tr.header.stamp = rospy.Time.now()
                tr.header.frame_id = "world"
                tr.child_frame_id = self.cfg.measured_frame_id
                tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = [float(v) for v in self.T_bind[:3, 3]]
                x, y, z, w = G.rot_to_quat(self.T_bind[:3, :3])
                tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = x, y, z, w
                self.pub_tf.publish(TFMessage([tr]))
            k += 1
            if k % 10 == 0:
                self._publish_state()
            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    def start(self):
        for t in self.threads:
            t.start()
        self._publish_state()
        return self

    def shutdown(self):
        self.stop.set()
        for t in self.threads:
            t.join(timeout=1.0)
        for s in (self.sub_servo, self.sub_state):
            if s is not None:
                s.unregister()
        for p in (self.pub_measured, self.pub_local, self.pub_tbw, self.pub_js, self.pub_truth, self.pub_state, self.pub_tf):
            if p is not None:
                p.unregister()

    def counters(self) -> Dict[str, int]:
        with self.lock:
            return {"received": self.received, "executed": self.executed, "rejected": self.rejected, "state": self.state, "homed": self.homed}
