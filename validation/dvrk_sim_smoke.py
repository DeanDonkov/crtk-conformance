#!/usr/bin/env python3
"""Feasibility smoke test for a dVRK 2.x kinematic-simulation PSM over ROS 1 (public CRTK topics only).

1. inventory + operating state; enable/home through state_command
2. passive frame relation T_hat = X * X_local^-1 versus the configured base_frame
3. active shared-binding check: servo_cp goal in the top-level frame -> local/measured_cp moves by base^-1 * goal
4. wrist-pitch servo_jp step: chord of the measured_cp point versus 2 L sin(dq/2), L = 0.0091 m (LND 400006 tool file)
Prints one JSON document.  Usage: dvrk_smoke.py <arm namespace> <system json> <out.json>
"""
import json, math, sys, time, re
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from crtk_msgs.msg import OperatingState, StringStamped

ns, system_json, out = sys.argv[1].rstrip('/'), sys.argv[2], sys.argv[3]
res = {"namespace": ns, "system_json": system_json}

def load_base(path):
    txt = open(path).read()
    txt = re.sub(r'/\*.*?\*/', '', txt, flags=re.S)
    arm = [a for a in json.loads(txt)["arms"] if "/" + a["name"] == ns][0]
    return np.array(arm["base_frame"]["transform"]) if "base_frame" in arm else np.eye(4)

def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w); x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def R_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2, R[2,1] - R[1,2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2, R[0,2] - R[2,0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2, R[1,0] - R[0,1])
    return x, y, z, w

def to_T(m):
    T = np.eye(4); q = m.pose.orientation; p = m.pose.position
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w); T[:3, 3] = [p.x, p.y, p.z]; return T

def ang(R):
    """Rotation-vector angle of the orthonormalised matrix (no acos precision floor near the identity)."""
    U, _, Vt = np.linalg.svd(R); Rn = U @ Vt
    w = np.array([Rn[2, 1] - Rn[1, 2], Rn[0, 2] - Rn[2, 0], Rn[1, 0] - Rn[0, 1]])
    return math.atan2(np.linalg.norm(w) / 2, (np.trace(Rn) - 1) / 2)

def orthonormalise(T):
    U, _, Vt = np.linalg.svd(T[:3, :3]); Tn = T.copy(); Tn[:3, :3] = U @ Vt; return Tn

def screw_axis(T0, T1):
    """Axis (point c, unit direction k) of the rigid motion M = T1 T0^-1 expressed in the reference frame."""
    M = T1 @ np.linalg.inv(T0); R, t = M[:3, :3], M[:3, 3]
    U, _, Vt = np.linalg.svd(R); R = U @ Vt
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]); k = w / np.linalg.norm(w)
    c = np.linalg.pinv(np.eye(3) - R) @ (t - (k @ t) * k)
    return c, k, float(abs(k @ t))

def line_distance(c1, k1, c2, k2):
    n = np.cross(k1, k2); return float(abs((c2 - c1) @ n) / np.linalg.norm(n)), float(np.linalg.norm(n))

import collections
last = {}; hist = {"cp": collections.deque(maxlen=200), "local": collections.deque(maxlen=200)}
def keep(name):
    def cb(m):
        last[name] = (time.monotonic(), m)
        if name in hist: hist[name].append(m)
    return cb

rospy.init_node("dvrk_smoke", anonymous=True, disable_signals=True)
rospy.Subscriber(ns + "/measured_cp", PoseStamped, keep("cp"), queue_size=10)
rospy.Subscriber(ns + "/local/measured_cp", PoseStamped, keep("local"), queue_size=10)
rospy.Subscriber(ns + "/measured_js", JointState, keep("js"), queue_size=10)
rospy.Subscriber(ns + "/operating_state", OperatingState, keep("os"), queue_size=10)
pub_state = rospy.Publisher(ns + "/state_command", StringStamped, queue_size=10, latch=False)
pub_cp = rospy.Publisher(ns + "/servo_cp", PoseStamped, queue_size=10)
pub_jp = rospy.Publisher(ns + "/servo_jp", JointState, queue_size=10)
time.sleep(2.0)

def wait(name, t=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < t:
        if name in last and last[name][0] > t0: return last[name][1]
        time.sleep(0.01)
    return None

def os_summary():
    time.sleep(0.5)
    d = last.get("os")
    return None if d is None else {"state": d[1].state, "is_homed": d[1].is_homed, "is_busy": d[1].is_busy}

res["topics"] = sorted(t for t, _ in rospy.get_published_topics() if t.startswith(ns + "/"))
res["operating_state_initial"] = os_summary()
for cmd, dwell in (("enable", 3.0), ("home", 8.0)):
    s = StringStamped(); s.header.stamp = rospy.Time.now(); s.string = cmd; pub_state.publish(s)
    time.sleep(dwell)
    res[f"operating_state_after_{cmd}"] = os_summary()

def js_now():
    m = wait("js"); return list(m.name), np.array(m.position)

def servo_jp(names, q, dur=1.5):
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        m = JointState(); m.header.stamp = rospy.Time.now(); m.name = names; m.position = list(q); pub_jp.publish(m); time.sleep(0.01)
    time.sleep(0.5)

# move away from the home singularity to a mid-range configuration (joint space only)
names, q = js_now()
res["joint_names"] = names
q_ref = q.copy()
for i, n in enumerate(names):
    if "insertion" in n: q_ref[i] = 0.12
    elif n.endswith("yaw") and "wrist" not in n: q_ref[i] = 0.1
    elif n.endswith("pitch") and "wrist" not in n: q_ref[i] = 0.1
    elif "wrist_pitch" in n: q_ref[i] = 0.0
servo_jp(names, q_ref)
res["joints_at_reference"] = dict(zip(names, js_now()[1].round(6).tolist()))

# 2. passive frame relation: pair messages carrying the SAME header stamp
base_raw = load_base(system_json); base = orthonormalise(base_raw)
time.sleep(1.0)
loc = {m.header.stamp.to_nsec(): m for m in list(hist["local"])}
Ts = [to_T(a) @ np.linalg.inv(to_T(loc[a.header.stamp.to_nsec()])) for a in list(hist["cp"]) if a.header.stamp.to_nsec() in loc and a.header.stamp.to_nsec() > 0]
res["stamp_matched_pairs"] = len(Ts)
if not Ts:  # stamps never identical on dVRK 2.4.0: nearest-stamp pairing within 50 ms (arm static here)
    ls = sorted(loc)
    for a in list(hist["cp"]):
        sa = a.header.stamp.to_nsec(); j = min(ls, key=lambda x: abs(x - sa))
        if sa > 0 and abs(j - sa) < 50e6: Ts.append(to_T(a) @ np.linalg.inv(to_T(loc[j])))
    res["nearest_stamp_pairs_50ms"] = len(Ts)
res["zero_stamps_seen"] = sum(1 for a in list(hist["cp"]) if a.header.stamp.to_nsec() == 0)
if Ts:
    E = [T @ np.linalg.inv(base) for T in Ts]
    res["frame_relation"] = {"pairs": len(Ts), "header_frame_ids": [wait("cp").header.frame_id, wait("local").header.frame_id],
                              "max_translation_residual_m": max(float(np.linalg.norm(e[:3, 3])) for e in E),
                              "max_rotation_residual_rad": max(ang(e[:3, :3]) for e in E),
                              "configured_base_frame_raw": base_raw.tolist(),
                              "raw_base_orthonormality_error": float(np.abs(base_raw[:3, :3].T @ base_raw[:3, :3] - np.eye(3)).max())}

# 3. active shared-binding check (top-level frame goal)
X0 = to_T(wait("cp"))
d = np.eye(4); c, s = math.cos(math.radians(5)), math.sin(math.radians(5))
d[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; d[:3, 3] = [0.005, 0.0, 0.0]
goal = X0.copy(); goal[:3, 3] = X0[:3, 3] + d[:3, 3]; goal[:3, :3] = d[:3, :3] @ X0[:3, :3]
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    m = PoseStamped(); m.header.stamp = rospy.Time.now()
    qx, qy, qz, qw = R_to_quat(goal[:3, :3])
    m.pose.position.x, m.pose.position.y, m.pose.position.z = goal[:3, 3]
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
    pub_cp.publish(m); time.sleep(0.01)
time.sleep(0.5)
X1, L1 = to_T(wait("cp")), to_T(wait("local"))
pred_local = np.linalg.inv(base) @ goal
res["shared_binding_check"] = {"goal_minus_measured_m": float(np.linalg.norm(X1[:3, 3] - goal[:3, 3])),
                               "goal_vs_measured_rot_rad": ang(X1[:3, :3].T @ goal[:3, :3]),
                               "local_vs_base_inv_goal_m": float(np.linalg.norm(L1[:3, 3] - pred_local[:3, 3])),
                               "local_vs_base_inv_goal_rot_rad": ang(L1[:3, :3].T @ pred_local[:3, :3]),
                               "measured_vs_base_times_local_m": float(np.linalg.norm(X1[:3, 3] - (base @ L1)[:3, 3])),
                               "measured_vs_base_times_local_rot_rad": ang(X1[:3, :3].T @ (base @ L1)[:3, :3]),
                               "X1": X1.tolist(), "L1": L1.tolist(), "goal": goal.tolist(),
                               "commanded_step_m": 0.005, "commanded_rot_deg": 5.0}

# 4. wrist-pitch step: instrument-geometry chord
servo_jp(names, q_ref)
P0 = to_T(wait("cp")); q0 = js_now()[1]
dq = 0.5
q1 = q_ref.copy(); iw = [i for i, n in enumerate(names) if "wrist_pitch" in n][0]; q1[iw] += dq
servo_jp(names, q1)
P1 = to_T(wait("cp")); qq1 = js_now()[1]
chord = float(np.linalg.norm(P1[:3, 3] - P0[:3, 3]))
res["wrist_pitch_step"] = {"joint": names[iw], "dq_rad": dq, "other_joints_max_change": float(np.max(np.abs(np.delete(qq1 - q0, iw)))),
                           "chord_interface_units": chord, "chord_predicted_L0.0091_m": 2 * 0.0091 * math.sin(dq / 2),
                           "lambda_hat_m_per_unit": 2 * 0.0091 * math.sin(dq / 2) / chord if chord > 0 else None,
                           "orientation_change_rad": ang(P0[:3, :3].T @ P1[:3, :3])}
# 5. instrument-geometry anchor independent of the published point: common normal of the wrist-pitch and wrist-yaw screw axes
iy = [i for i, n in enumerate(names) if "wrist_yaw" in n][0]
servo_jp(names, q_ref); A0 = to_T(wait("cp"))
qp = q_ref.copy(); qp[iw] += dq; servo_jp(names, qp); Ap = to_T(wait("cp"))
servo_jp(names, q_ref); A0b = to_T(wait("cp"))
qy = q_ref.copy(); qy[iy] += dq; servo_jp(names, qy); Ay = to_T(wait("cp"))
cp_, kp_, slide_p = screw_axis(A0, Ap); cy_, ky_, slide_y = screw_axis(A0b, Ay)
d_int, sin_between = line_distance(cp_, kp_, cy_, ky_)
res["screw_axis_anchor"] = {"pitch_axis_dir": kp_.tolist(), "yaw_axis_dir": ky_.tolist(), "sin_angle_between_axes": sin_between,
                            "common_normal_interface_units": d_int, "L_phys_m (dVRK LND 400006 A)": 0.0091,
                            "lambda_hat_m_per_unit": 0.0091 / d_int if d_int > 0 else None,
                            "translation_along_axes (should be 0)": [slide_p, slide_y]}
json.dump(res, open(out, "w"), indent=1)
print(json.dumps(res, indent=1))
rospy.signal_shutdown("done")
