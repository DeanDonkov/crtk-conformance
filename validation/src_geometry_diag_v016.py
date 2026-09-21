"""RC9 EXPLORATORY diagnostic, run after the pre-registered SRC runs (not a result; not pooled): why the geometry-anchor
gates failed on SRC v1.0.0 / v2.0.0.  Starts the same stack as src_live_v016.py, moves to the geometry reference
configuration, steps the wrist pitch (index 4) and then the wrist yaw (index 5) by 0.5 rad as the probe does (2.0 s settle,
0.3 s window), and records measured_js for the whole sequence: the change of every OTHER joint between the settled
windows, and the realized change of the stepped joint.

Usage: python3 validation/src_geometry_diag_v016.py v1|v2
"""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import src_live_v016 as S

v = sys.argv[1]
out = os.path.join(S.REPO, "validation", "v0.1.6", "src_live", f"exploratory-geometry-diag-{v}")
os.makedirs(out, exist_ok=True)
import subprocess
subprocess.run(f"cd {S.LIVE}/src-{v}/scripts && pip3 install -q --no-deps --no-build-isolation -e . > /dev/null 2>&1", shell=True)  # as run()
S.move_to_work = lambda version, logdir: None  # this process subscribes itself (a re-subscription after unregister received nothing)
with S.Stack(v, os.path.join(out, "logs")):
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node("src_geometry_diag", anonymous=True, disable_signals=True)
    from geometry_msgs.msg import PoseStamped
    hist = {"js": [], "cp": []}
    rospy.Subscriber(S.NS + "/measured_js", JointState, lambda m: hist["js"].append((time.monotonic(), list(m.position))))
    rospy.Subscriber(S.NS + "/measured_cp", PoseStamped, lambda m: hist["cp"].append((time.monotonic(), [m.pose.position.x, m.pose.position.y, m.pose.position.z,
                                                                                                      m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])))
    pub = rospy.Publisher(S.NS + "/servo_jp", JointState, queue_size=10)
    for _ in range(50):
        if hist["js"]:
            break
        time.sleep(0.1)
    print("js messages so far:", len(hist["js"]), "cp:", len(hist["cp"]), flush=True)
    q_ref = np.array([0.0, 0.0, S.Q_INS[v], 0.0, 0.1, -0.2])

    def stream(q, dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            m = JointState(); m.header.stamp = rospy.Time.now(); m.position = list(q); pub.publish(m); time.sleep(0.01)

    def settled(q):
        stream(q, 2.0); t0 = time.monotonic(); stream(q, 0.3)
        J = np.array([p for t, p in hist["js"] if t >= t0])
        return J.mean(axis=0)[:6], J.std(axis=0)[:6]

    res = {"version": v, "q_ref": q_ref.tolist(), "trials": []}
    for k in range(3):
        q0, s0 = settled(q_ref)
        qp = q_ref.copy(); qp[4] += 0.5
        q1, s1 = settled(qp)
        q0b, _ = settled(q_ref)
        qy = q_ref.copy(); qy[5] += 0.5
        q2, s2 = settled(qy)
        res["trials"].append({"pitch_step": {"delta_measured": (q1 - q0).tolist(), "delta_commanded": (qp - q_ref).tolist()},
                              "yaw_step": {"delta_measured": (q2 - q0b).tolist(), "delta_commanded": (qy - q_ref).tolist()},
                              "settled_std_max": float(max(s0.max(), s1.max(), s2.max())),
                              "settled_std_per_joint": {"ref": s0.tolist(), "pitch_step": s1.tolist(), "yaw_step": s2.tolist()}})
    stream(q_ref, 1.0)
    json.dump(res, open(os.path.join(out, "diag.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))
