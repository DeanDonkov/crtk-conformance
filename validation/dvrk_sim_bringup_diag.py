"""RC9 harness diagnostic (no probe): how the released dVRK 2.4.0 console in kinematic simulation reacts to the
state_command sequences the harness and the tool send.  Written after the first campaign attempt failed at bring-up
(PID tracking error on insertion: the harness streamed the reference joints while homing was still in progress),
before any probe had run.  Logs operating_state (state, is_homed, is_busy) and measured_js at 20 Hz."""
import json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dvrk_sim_v016 as H
import rospy
from sensor_msgs.msg import JointState
from crtk_msgs.msg import OperatingState, StringStamped

def first(out):
    out = os.path.join(H.REPO, "validation/v0.1.6/dvrk_sim/harness_diag")
    os.makedirs(out, exist_ok=True)
    log = []
    with H.Launch(os.path.join(H.REPO, "validation/v0.1.6/dvrk_sim/configs/system-PSM2_KIN_SIM-jhu.json"), os.path.join(out, "launch")):
        rospy.init_node("diag", anonymous=True, disable_signals=True)
        last = {}
        rospy.Subscriber(H.NS + "/operating_state", OperatingState, lambda m: last.__setitem__("os", m))
        rospy.Subscriber(H.NS + "/measured_js", JointState, lambda m: last.__setitem__("js", m))
        pub = rospy.Publisher(H.NS + "/state_command", StringStamped, queue_size=10)
        pj = rospy.Publisher(H.NS + "/servo_jp", JointState, queue_size=10)
        time.sleep(1.5)
        T0 = time.monotonic()

        def snap(tag):
            o, j = last.get("os"), last.get("js")
            log.append({"t": round(time.monotonic() - T0, 3), "tag": tag, "state": o.state if o else None, "homed": o.is_homed if o else None, "busy": o.is_busy if o else None,
                        "q": [round(v, 5) for v in j.position] if j else None})

        def cmd(c):
            m = StringStamped(); m.header.stamp = rospy.Time.now(); m.string = c; pub.publish(m); snap("sent " + c)

        def watch(sec, tag):
            t1 = time.monotonic()
            while time.monotonic() - t1 < sec:
                snap(tag); time.sleep(0.05)

        def interp(q1, sec=3.0):
            j = last["js"]; q0 = np.array(j.position); names = list(j.name)
            t1 = time.monotonic()
            while True:
                a = min(1.0, (time.monotonic() - t1) / sec)
                m = JointState(); m.header.stamp = rospy.Time.now(); m.name = names; m.position = list(q0 + a * (np.array(q1) - q0)); pj.publish(m)
                if a >= 1.0:
                    break
                time.sleep(0.01)
            watch(1.0, "after interp")

        watch(0.5, "initial")
        cmd("enable"); watch(0.2, "after enable")
        cmd("home"); watch(12.0, "homing")
        interp(H.REF_JOINTS)
        cmd("enable"); cmd("home"); watch(6.0, "re-enable/home while homed")
        cmd("disable"); watch(1.0, "disabled")
        cmd("enable"); cmd("home"); watch(12.0, "enable/home after disable")
    json.dump(log, open(os.path.join(out, "state_log.json"), "w"), indent=0)
    prev = None
    for e in log:
        key = (e["tag"], e["state"], e["homed"], e["busy"], tuple(e["q"] or []))
        if key != prev:
            print(e)
        prev = key


# ---------------------------------------------------------------------------------------------------------------
# Second diagnostic (after the fix, still before any probe run): the corrected harness bring-up, then the tool's
# 0.1.6 ensure_enabled() after a `disable` (what the state-machine sub-probe does), then bring-up again.
def second(out2):
    from crtk_conformance.adapter import PlatformAdapter
    from crtk_conformance.probes.common import ensure_enabled
    res = {}
    with H.Launch(os.path.join(H.REPO, "validation/v0.1.6/dvrk_sim/configs/system-PSM2_KIN_SIM-jhu.json"), os.path.join(out2, "launch")):
        res["bring_up_1"] = H.bring_up()
        a = PlatformAdapter(H.NS)
        a.discover()
        a.subscribe("operating_state"); time.sleep(0.5)
        a.state_command("disable"); time.sleep(1.0)
        s = a.operating_state(1.0)
        res["after_disable"] = None if s is None else {"state": s.state, "is_homed": s.is_homed, "is_busy": s.is_busy}
        res["ensure_enabled_after_disable"] = ensure_enabled(a, timeout_s=15.0)
        res["ensure_enabled_when_ready"] = ensure_enabled(a, timeout_s=15.0)
        a.close()
        res["bring_up_2"] = H.bring_up()
    json.dump(res, open(os.path.join(out2, "diag2.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    base = os.path.join(H.REPO, "validation/v0.1.6/dvrk_sim/harness_diag")
    if (sys.argv[1] if len(sys.argv) > 1 else "first") == "first":
        first(base)
    else:
        os.makedirs(os.path.join(base, "second"), exist_ok=True)
        second(os.path.join(base, "second"))
