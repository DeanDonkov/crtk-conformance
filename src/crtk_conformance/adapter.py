"""ROS 1 platform adapter: everything the probes see, they see through here.

Only public interfaces are used: the ROS master API for topic discovery, ordinary subscriptions
for data, ordinary publications for commands. No platform-specific code, no hidden adaptation.

ROS 2 is not supported in this release (see README, "What is and is not implemented").
"""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from crtk_msgs.msg import OperatingState, StringStamped

from . import geometry as G


def pose_msg_to_matrix(msg: PoseStamped) -> np.ndarray:
    q = msg.pose.orientation
    R = G.quat_to_rot(q.x, q.y, q.z, q.w)
    p = msg.pose.position
    return G.make_pose(R, (p.x, p.y, p.z))


def matrix_to_pose_msg(T: np.ndarray, frame_id: str = "") -> PoseStamped:
    m = PoseStamped()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = frame_id
    m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in T[:3, 3]]
    x, y, z, w = G.rot_to_quat(T[:3, :3])
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = x, y, z, w
    return m


def transform_msg_to_matrix(msg: TransformStamped) -> np.ndarray:
    q = msg.transform.rotation
    R = G.quat_to_rot(q.x, q.y, q.z, q.w)
    t = msg.transform.translation
    return G.make_pose(R, (t.x, t.y, t.z))


class Buffer:
    """Thread-safe ring buffer of (receive_time, msg) pairs."""

    def __init__(self, maxlen: int = 5000):
        self._d: Deque[Tuple[float, object]] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._event = threading.Event()
        self.count = 0

    def push(self, msg):
        with self._lock:
            self._d.append((time.monotonic(), msg))
            self.count += 1
        self._event.set()

    def latest(self):
        with self._lock:
            return self._d[-1] if self._d else None

    def since(self, t_mono: float) -> List[Tuple[float, object]]:
        with self._lock:
            return [(t, m) for (t, m) in self._d if t >= t_mono]

    def clear(self):
        with self._lock:
            self._d.clear()

    def wait_new(self, timeout: float) -> bool:
        self._event.clear()
        return self._event.wait(timeout)


class PlatformAdapter:
    """Discovers and talks to one CRTK arm namespace, e.g. '/PSM1'."""

    TOPIC_TYPES = {
        "measured_cp": PoseStamped,
        "setpoint_cp": PoseStamped,
        "local/measured_cp": PoseStamped,
        "local/setpoint_cp": PoseStamped,
        "servo_cp": PoseStamped,
        "servo_cr": PoseStamped,
        "measured_js": JointState,
        "operating_state": OperatingState,
        "state_command": StringStamped,
        "T_b_w": PoseStamped,
    }

    def __init__(self, namespace: str, node_name: str = "crtk_conformance", anchor_topic: Optional[str] = None):
        self.ns = namespace.rstrip("/")
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
        self.master = rosgraph.Master(rospy.get_name())
        self.anchor_topic = anchor_topic
        self._subs: Dict[str, rospy.Subscriber] = {}
        self._bufs: Dict[str, Buffer] = {}
        self._pubs: Dict[str, rospy.Publisher] = {}
        self.tf: Dict[Tuple[str, str], Tuple[float, np.ndarray]] = {}  # (parent, child) -> (recv time, T)
        self._tf_lock = threading.Lock()
        self.discovery: Dict[str, object] = {}

    # ------------------------------------------------------------------ discovery
    def discover(self) -> Dict[str, object]:
        """Topic discovery through the ROS master API. A topic counts as present only if some node
        *other than this one* currently publishes or subscribes to it (the master remembers types of
        topics that no longer exist, so getTopicTypes alone would over-report)."""
        t0 = time.monotonic()
        topic_types = dict(self.master.getTopicTypes())
        pubs, subs, _ = self.master.getSystemState()
        me = rospy.get_name()
        published = {t for t, nodes in pubs if any(n != me for n in nodes)}
        subscribed = {t for t, nodes in subs if any(n != me for n in nodes)}
        found = {}
        for name, cls in self.TOPIC_TYPES.items():
            full = f"{self.ns}/{name}"
            present = full in published or full in subscribed
            found[name] = {
                "topic": full,
                "present": present,
                "type": topic_types.get(full) if present else None,
                "type_matches": (topic_types.get(full) == cls._type) if present else None,
                "published": full in published,
                "subscribed": full in subscribed,
            }
        live = published | subscribed
        self.discovery = {
            "namespace": self.ns,
            "discovery_latency_s": time.monotonic() - t0,
            "topics": found,
            "tf_present": "/tf" in published,
            "tf_static_present": "/tf_static" in published,
            "all_topics_in_namespace": sorted(t for t in live if t.startswith(self.ns + "/")),
        }
        return self.discovery

    def has(self, name: str) -> bool:
        return bool(self.discovery.get("topics", {}).get(name, {}).get("present"))

    # ------------------------------------------------------------------ subscriptions
    def subscribe(self, name: str, cls=None, full_topic: Optional[str] = None) -> Buffer:
        topic = full_topic or f"{self.ns}/{name}"
        if topic in self._bufs:
            return self._bufs[topic]
        cls = cls or self.TOPIC_TYPES[name]
        buf = Buffer()
        self._bufs[topic] = buf
        self._subs[topic] = rospy.Subscriber(topic, cls, buf.push, queue_size=200)
        return buf

    def subscribe_tf(self):
        def cb(msg: TFMessage):
            now = time.monotonic()
            with self._tf_lock:
                for tr in msg.transforms:
                    self.tf[(tr.header.frame_id, tr.child_frame_id)] = (now, transform_msg_to_matrix(tr))

        if "/tf" not in self._subs:
            self._subs["/tf"] = rospy.Subscriber("/tf", TFMessage, cb, queue_size=100)
            self._subs["/tf_static"] = rospy.Subscriber("/tf_static", TFMessage, cb, queue_size=100)

    def tf_lookup(self, parent: str, child: str, max_age_s: float = 2.0) -> Optional[np.ndarray]:
        """Direct or one-hop lookup, public /tf only."""
        with self._tf_lock:
            now = time.monotonic()
            d = self.tf.get((parent, child))
            if d and now - d[0] <= max_age_s:
                return d[1]
            # one hop: parent->x, x->child
            for (a, b), (ta, Ta) in self.tf.items():
                if a == parent and now - ta <= max_age_s:
                    d2 = self.tf.get((b, child))
                    if d2 and now - d2[0] <= max_age_s:
                        return Ta @ d2[1]
        return None

    def wait_for(self, buf: Buffer, timeout: float):
        """Return the next message after now (None on timeout)."""
        t = time.monotonic()
        deadline = t + timeout
        while time.monotonic() < deadline:
            buf.wait_new(min(0.05, max(0.0, deadline - time.monotonic())))
            new = buf.since(t)
            if new:
                return new[-1][1]
        return None

    def latest_pose(self, buf: Buffer, max_age_s: float = 0.5) -> Optional[np.ndarray]:
        d = buf.latest()
        if d is None or time.monotonic() - d[0] > max_age_s:
            return None
        return pose_msg_to_matrix(d[1])

    # ------------------------------------------------------------------ publications
    def publisher(self, name: str, cls=None) -> rospy.Publisher:
        topic = f"{self.ns}/{name}"
        if topic not in self._pubs:
            cls = cls or self.TOPIC_TYPES[name]
            self._pubs[topic] = rospy.Publisher(topic, cls, queue_size=10)
            time.sleep(0.3)  # let the subscriber connect (TCPROS handshake)
        return self._pubs[topic]

    def servo_cp(self, T: np.ndarray, frame_id: str = ""):
        self.publisher("servo_cp").publish(matrix_to_pose_msg(T, frame_id))

    def state_command(self, cmd: str):
        m = StringStamped()
        m.header.stamp = rospy.Time.now()
        m.string = cmd
        self.publisher("state_command").publish(m)

    def operating_state(self, timeout: float = 1.0) -> Optional[OperatingState]:
        buf = self.subscribe("operating_state")
        d = buf.latest()
        if d is not None and time.monotonic() - d[0] < timeout:
            return d[1]
        return self.wait_for(buf, timeout)

    def close(self):
        for s in self._subs.values():
            s.unregister()
        for p in self._pubs.values():
            p.unregister()
