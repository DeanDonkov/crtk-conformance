#!/usr/bin/env python3
"""Auxiliary live observations on a running SRC CRTK interface (not part of crtk-conformance).

1. topic inventory of the CRTK namespace with message types and frame_id strings;
2. presence/absence of local/, setpoint_cp, operating_state, state_command, T_b_w; T_b_w value;
3. measured_cp publish rate;
4. Reviewer #2 M2 check: measured_js (reported, de-perturbed) vs the simulator's own joint state
   (/ambf/env/<arm>/baselink/State, field joint_positions) after commanding a joint configuration;
5. cross-version differential for D2: measured_cp at a fixed joint configuration (q = 0), whose norm is
   determined by the kinematic link lengths only, so that the ratio between v1.0.0 and v2.0.0 is the
   unit factor (an inter-version comparison, not a unit anchor).
6. servo_cp -> measured_cp residual after settling (interface-level agreement despite injected joint errors).
Everything is written as JSON to --out.
"""
import argparse
import json
import math
import time

import numpy as np
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class Latest:
    def __init__(self):
        self.msgs = []

    def cb(self, m):
        self.msgs.append((rospy.get_time(), time.monotonic(), m))


def pose_to_list(p):
    return [p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="/CRTK/psm1")
    ap.add_argument("--ambf-arm", default="/ambf/env/psm1/baselink")
    ap.add_argument("--out", required=True)
    ap.add_argument("--version", required=True)
    args = ap.parse_args()
    rospy.init_node("rc3_live_aux", anonymous=True, disable_signals=True)
    out = {"version": args.version, "namespace": args.ns, "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    master = rosgraph.Master("/rc3_live_aux")
    topic_types = dict(master.getTopicTypes())
    pubs, subs, _ = master.getSystemState()
    ns_topics = sorted(t for t in topic_types if t.startswith(args.ns + "/"))
    out["topics_in_namespace"] = [{"topic": t, "type": topic_types[t],
                                   "publishers": [n for tt, n in pubs if tt == t], "subscribers": [n for tt, n in subs if tt == t]} for t in ns_topics]
    out["all_topics_count"] = len(topic_types)
    out["all_topics"] = sorted(topic_types)
    names = ["measured_cp", "local/measured_cp", "setpoint_cp", "local/setpoint_cp", "servo_cp", "servo_cr", "measured_js", "servo_jp",
             "operating_state", "state_command", "T_b_w", "measured_cv"]
    out["presence"] = {n: (args.ns + "/" + n) in topic_types for n in names}
    out["tf_present"] = "/tf" in topic_types
    out["tf_static_present"] = "/tf_static" in topic_types
    # frame_id strings and rates
    L = Latest()
    sub = rospy.Subscriber(args.ns + "/measured_cp", PoseStamped, L.cb, queue_size=200)
    t0 = time.monotonic()
    time.sleep(3.0)
    n = len(L.msgs)
    out["measured_cp"] = {"n_3s": n, "rate_hz": n / (time.monotonic() - t0),
                          "frame_id": L.msgs[-1][2].header.frame_id if n else None,
                          "stamp_is_zero": (L.msgs[-1][2].header.stamp.to_sec() == 0.0) if n else None,
                          "sample": pose_to_list(L.msgs[-1][2].pose) if n else None}
    if n > 2:
        d = np.diff([m[1] for m in L.msgs])
        out["measured_cp"]["interarrival_s"] = {"mean": float(d.mean()), "std": float(d.std()), "min": float(d.min()), "max": float(d.max())}
    if out["presence"]["T_b_w"]:
        B = Latest()
        s2 = rospy.Subscriber(args.ns + "/T_b_w", PoseStamped, B.cb, queue_size=10)
        time.sleep(2.0)
        if B.msgs:
            m = B.msgs[-1][2]
            out["T_b_w"] = {"frame_id": m.header.frame_id, "pose": pose_to_list(m.pose), "rate_hz": len(B.msgs) / 2.0}
        s2.unregister()
    if out["presence"]["measured_js"]:
        J = Latest()
        s3 = rospy.Subscriber(args.ns + "/measured_js", JointState, J.cb, queue_size=10)
        time.sleep(1.5)
        if J.msgs:
            m = J.msgs[-1][2]
            out["measured_js"] = {"frame_id": m.header.frame_id, "names": list(m.name), "position": list(m.position), "rate_hz": len(J.msgs) / 1.5}
        s3.unregister()
    # ---- M2 check and D2 differential: command q = [0, 0, q3, 0, 0, 0] via servo_jp and read back
    A = Latest()
    try:
        from ambf_msgs.msg import RigidBodyState
        s4 = rospy.Subscriber(args.ambf_arm + "/State", RigidBodyState, A.cb, queue_size=10)
        have_ambf = True
    except Exception as e:  # pragma: no cover
        have_ambf = False
        out["ambf_state_error"] = repr(e)
    pub_jp = rospy.Publisher(args.ns + "/servo_jp", JointState, queue_size=1)
    time.sleep(1.0)
    J = Latest()
    s3 = rospy.Subscriber(args.ns + "/measured_js", JointState, J.cb, queue_size=10)
    time.sleep(1.0)
    q_start = list(J.msgs[-1][2].position) if J.msgs else None
    out["initial_measured_js"] = q_start
    # q_ins: insertion 0.1 interface units keeps the tip clear of the RCM; the position norm at a fixed joint vector is
    # determined by the kinematic link lengths, so it can be compared between versions.  The arm is returned to its
    # initial joint configuration afterwards so that the probes start from the released scene's home pose.
    configs = {"q_ins": [0.0, 0.0, 0.1, 0.0, 0.0, 0.0], "q_ins_yaw": [0.3, 0.0, 0.1, 0.0, 0.0, 0.0]}
    if q_start is not None:
        configs["q_return"] = q_start[:6]
    out["joint_configs"] = {}
    for name, q in configs.items():
        js = JointState()
        js.position = q
        for _ in range(int(3.0 * 50)):
            js.header.stamp = rospy.Time.now()
            pub_jp.publish(js)
            time.sleep(0.02)
        time.sleep(1.0)
        L.msgs.clear(); J.msgs.clear(); A.msgs.clear()
        time.sleep(1.0)
        rec = {"commanded_q": q}
        if L.msgs:
            P = np.array([pose_to_list(m[2].pose)[:3] for m in L.msgs])
            rec["measured_cp_position_mean"] = P.mean(axis=0).tolist()
            rec["measured_cp_position_std"] = P.std(axis=0).tolist()
            rec["measured_cp_position_norm"] = float(np.linalg.norm(P.mean(axis=0)))
        if J.msgs:
            Q = np.array([list(m[2].position) for m in J.msgs])
            rec["measured_js_mean"] = Q.mean(axis=0).tolist()
        if have_ambf and A.msgs:
            Qa = np.array([list(m[2].joint_positions) for m in A.msgs])
            rec["ambf_joint_positions_mean"] = Qa.mean(axis=0).tolist()
            rec["ambf_joint_names"] = list(A.msgs[-1][2].joint_names)
            k = min(len(rec["measured_js_mean"]) if "measured_js_mean" in rec else 0, Qa.shape[1])
            if k:
                rec["reported_minus_simulated_joint"] = (np.array(rec["measured_js_mean"][:k]) - Qa.mean(axis=0)[:k]).tolist()
            p = A.msgs[-1][2].pose
            rec["ambf_baselink_pose_world"] = [p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        out["joint_configs"][name] = rec
    # ---- servo_cp -> measured_cp residual after a small Cartesian step from the home pose
    pub_cp = rospy.Publisher(args.ns + "/servo_cp", PoseStamped, queue_size=1)
    time.sleep(0.5)
    L.msgs.clear(); time.sleep(0.5)
    if L.msgs:
        import copy
        m0 = L.msgs[-1][2]
        goal = PoseStamped()
        goal.header.frame_id = m0.header.frame_id
        goal.pose = copy.deepcopy(m0.pose)
        out["servo_cp_step_start"] = pose_to_list(m0.pose)
        goal.pose.position.x += 0.01
        for _ in range(100):
            goal.header.stamp = rospy.Time.now()
            pub_cp.publish(goal)
            time.sleep(0.02)
        time.sleep(1.5)
        L.msgs.clear(); time.sleep(1.0)
        P = np.array([pose_to_list(m[2].pose)[:3] for m in L.msgs])
        g = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        out["servo_cp_step"] = {"goal_position": g.tolist(), "measured_mean": P.mean(axis=0).tolist(),
                                "residual_norm_if_units": float(np.linalg.norm(P.mean(axis=0) - g)), "measured_std": P.std(axis=0).tolist(),
                                "commanded_step_if_units": 0.01}
        # return to the start pose
        goal.pose = copy.deepcopy(m0.pose)
        for _ in range(100):
            goal.header.stamp = rospy.Time.now()
            pub_cp.publish(goal)
            time.sleep(0.02)
        time.sleep(1.0)
    out["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps({k: out[k] for k in ("presence", "measured_cp", "T_b_w", "joint_configs", "servo_cp_step") if k in out}, indent=1))


if __name__ == "__main__":
    main()
