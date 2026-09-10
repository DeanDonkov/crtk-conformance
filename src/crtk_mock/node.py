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

import json
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


from .config import MockConfig  # noqa: F401  (re-exported; defined ROS-free in config.py)


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
        self.goal: Optional[np.ndarray] = None  # current target being tracked (max_speed_m_s > 0) or None
        self.queue: List = []  # (due_time, seq, T_local) pending commands (0.1.1: delayed commands are queued, not discarded)
        self.seq = 0
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
        self.pub_setpoint = rospy.Publisher(f"{self.ns}/setpoint_cp", PoseStamped, queue_size=q) if cfg.publish_setpoint_cp else None
        self.accepted_goal_if: Optional[np.ndarray] = None  # goal currently applied by the execution loop, interface frame/units (what setpoint_cp reports; 0.1.3)
        self._event_log = open(cfg.event_log_path, "a") if cfg.event_log_path else None
        self.accept_counter = 0
        self.pub_state = rospy.Publisher(f"{self.ns}/operating_state", OperatingState, queue_size=q, latch=True) if cfg.state_machine else None
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=q) if cfg.publish_tf else None
        self.sub_servo = rospy.Subscriber(f"{self.ns}/servo_cp", PoseStamped, self._servo_cp_cb, queue_size=1)
        self.sub_state = rospy.Subscriber(f"{self.ns}/state_command", StringStamped, self._state_cmd_cb, queue_size=10) if cfg.state_machine else None
        self.threads = [threading.Thread(target=self._loop, daemon=True), threading.Thread(target=self._publish_loop, daemon=True)]

    # ------------------------------------------------------------------ callbacks
    def _log(self, event: str, t: float, stamp: float, seq: int, pos_if=None):
        if self._event_log is not None:
            self._event_log.write(json.dumps({"event": event, "t": t, "stamp": stamp, "seq": seq,
                                              "pos_if": ([float(v) for v in pos_if] if pos_if is not None else None)}) + "\n")
            self._event_log.flush()

    def _servo_cp_cb(self, msg: PoseStamped):
        now = time.monotonic()
        stamp = msg.header.stamp.to_sec()  # the client's own stamp: the key by which the validation matches events to sends
        with self.lock:
            self.received += 1
            T_if = pose_msg_to_matrix(msg)  # interface units, bound frame
            if self.cfg.drop_prob > 0 and self.rng.random() < self.cfg.drop_prob:
                self._log("drop", now, stamp, self.received, T_if[:3, 3])
                return
            if self.cfg.require_enabled and not (self.state == "ENABLED" and self.homed):
                self.rejected += 1
                self._log("reject", now, stamp, self.received, T_if[:3, 3])
                return
            self.accept_counter += 1
            if self.cfg.accept_every_k > 1 and (self.accept_counter % self.cfg.accept_every_k) != 0:
                self._log("ignore", now, stamp, self.received, T_if[:3, 3])
                return  # silently ignored (0.1.2: counterexample of the RC3 review, finding 2)
            goal_if = T_if.copy()
            T_if[:3, 3] *= self.cfg.unit_m  # -> metres
            T_local = self.T_bind_inv @ T_if  # -> arm-base frame
            delay = self.cfg.response_delay_s + self.rng.uniform(0.0, self.cfg.response_jitter_s)
            self.seq += 1
            # 0.1.3: the goal in interface units travels with the queued command and becomes the reported setpoint
            # only when the execution loop applies it (RC4 review, finding 1: the channel must report the stage its
            # name denotes, not receipt)
            self.queue.append((now + delay, self.seq, T_local, goal_if, stamp))
            self._log("receive", now, stamp, self.seq, goal_if[:3, 3])
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
                            self.queue.clear()
                            self._publish_state()
                        else:
                            self.released = True
                            self.goal = None
                            self.queue.clear()
                if self.released and self.cfg.release_drift_m_s > 0:
                    self.pose[2, 3] -= self.cfg.release_drift_m_s * period
                # execute: among the commands that are due, the latest wins (a delayed pipeline still executes
                # every setpoint whose due time has passed, in order, one per tick at most)
                if self.state == "ENABLED":
                    due = [q for q in self.queue if q[0] <= now]
                    if due:
                        # latest-wins among the due commands (highest sequence number); anything older than the
                        # command executed is stale and is discarded, so a jittered pipeline never moves backwards
                        newest = max(due, key=lambda q: q[1])
                        for q in self.queue:
                            if q[1] < newest[1]:
                                self._log("supersede", now, q[4], q[1], q[3][:3, 3])
                        self.queue = [q for q in self.queue if q[1] > newest[1]]
                        self.goal = newest[2]
                        self.accepted_goal_if = newest[3]
                        self.executed += 1
                        self._log("apply", now, newest[4], newest[1], newest[3][:3, 3])
                    if self.goal is not None:
                        if self.cfg.max_speed_m_s <= 0:
                            self.pose = self.goal
                            self.goal = None
                        else:
                            d = self.goal[:3, 3] - self.pose[:3, 3]
                            dist = float(np.linalg.norm(d))
                            step = self.cfg.max_speed_m_s * period
                            if dist <= step:
                                self.pose = self.goal
                                self.goal = None
                            else:
                                P = self.pose.copy()
                                P[:3, 3] += d / dist * step
                                P[:3, :3] = self.goal[:3, :3]
                                self.pose = P
            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    def _rot_noise(self) -> np.ndarray:
        axis = self.nrng.normal(size=3)
        axis /= np.linalg.norm(axis)
        return G.axis_angle(axis, math.radians(self.nrng.normal(0.0, self.cfg.orientation_noise_deg)))

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
            if self.cfg.orientation_noise_deg > 0:
                T_if[:3, :3] = self._rot_noise() @ T_if[:3, :3]
            T_if[:3, 3] /= self.cfg.unit_m
            stamp = rospy.Time.now()
            if self.pub_measured is not None:
                m = matrix_to_pose_msg(T_if, self.cfg.measured_frame_id)
                m.header.stamp = stamp
                self.pub_measured.publish(m)
            if self.pub_setpoint is not None:
                with self.lock:
                    sp = self.accepted_goal_if
                if sp is not None:
                    m = matrix_to_pose_msg(sp, self.cfg.measured_frame_id)
                    m.header.stamp = stamp
                    self.pub_setpoint.publish(m)
            if self.pub_local is not None:
                T_loc = pose.copy()
                if self.cfg.noise_m > 0:
                    T_loc[:3, 3] += self.nrng.normal(0.0, self.cfg.noise_m, 3)
                if self.cfg.orientation_noise_deg > 0:
                    T_loc[:3, :3] = self._rot_noise() @ T_loc[:3, :3]
                T_loc[:3, 3] /= self.cfg.unit_m
                m = matrix_to_pose_msg(T_loc, self.cfg.local_frame_id)
                m.header.stamp = stamp + rospy.Duration.from_sec(self.cfg.stamp_skew_s)
                self.pub_local.publish(m)
            if self.pub_tbw is not None:
                # base-in-world; in the mock the "world" is the bound frame
                self.pub_tbw.publish(matrix_to_pose_msg(self.T_bind, "world"))
            if self.pub_js is not None:
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = ["outer_yaw", "outer_pitch", "outer_insertion"]
                js.position = [0.0, 0.0, float(np.linalg.norm(pose[:3, 3]) / self.cfg.unit_m)]
                self.pub_js.publish(js)
            T_truth = pose.copy()
            if self.cfg.anchor_noise_m > 0:
                T_truth[:3, 3] += self.nrng.normal(0.0, self.cfg.anchor_noise_m, 3)
            self.pub_truth.publish(matrix_to_pose_msg(T_truth, "mock_ground_truth_base"))
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
        if self._event_log is not None:
            self._event_log.close()
            self._event_log = None
        for t in self.threads:
            t.join(timeout=1.0)
        for s in (self.sub_servo, self.sub_state):
            if s is not None:
                s.unregister()
        for p in (self.pub_measured, self.pub_local, self.pub_tbw, self.pub_js, self.pub_truth, self.pub_state, self.pub_tf, self.pub_setpoint):
            if p is not None:
                p.unregister()

    def counters(self) -> Dict[str, int]:
        with self.lock:
            return {"received": self.received, "executed": self.executed, "rejected": self.rejected, "state": self.state, "homed": self.homed}
