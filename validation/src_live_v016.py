"""RC9 WP3.5: the released Surgical Robotics Challenge CRTK interfaces (v1.0.0 = 158b554e, v2.0.0 = 03befbf1) on AMBF
branch ambf-2.0 @ 16a81518, headless, re-run with the 0.1.6 instrument-geometry unit anchor (PREREGISTRATION.md, section 2).
The launch sequence is that of validation/environment/run_live.sh (RC3/RC8): the simulator with each release's own launch
arguments under Xvfb, then launch_crtk_interface.py with --two False --ecm False --scene False (PSM1 only).  No SRC or
AMBF source is modified.

Usage (inside the campaign image; build dir at /ws with src_live/{ambf-2.0,src-v1,src-v2}; this repository at /repo):
    python3 validation/src_live_v016.py configs
    python3 validation/src_live_v016.py run v1|v2 [--out validation/v0.1.6/src_live]
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
LIVE = os.environ.get("SRC_LIVE", "/ws/src_live")
NS = "/CRTK/psm1"
L_LND = 0.0091
U_REL = 0.015
Q_INS = {"v1": 1.0, "v2": 0.1}  # insertion used by RC8 (about 42 % of the range), interface units
AMBF_ARGS = {
    "v1": "--launch_file {src}/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120 -g false",
    "v2": "--launch_file {src}/launch.yaml -l 0,1,6,7,8,9 -p 200 -t 1 --override_max_comm_freq 100 --override_min_comm_freq 100 -g false",
}
RC8_PARAMS = ["--trials", "10", "--temporal-trials", "5", "--gap-max-s", "2.0", "--settle-s", "2.0", "--still-tol-mm", "0.05",
              "--rates-hz", "50,100,200,500", "--rate-max-step-mm", "100", "--workspace-radius-m", "0.10", "--speed-mm-s", "50",
              "--client-rate-hz", "100", "--jitter-max-ms", "5", "--geometry-trials", "5", "--geometry-settle-s", "2.0"]


def geo(version):
    return {"mode": "geometry_anchor", "expected_unit_m": 1.0, "L_m": L_LND, "u_rel": U_REL, "delta_q_rad": 0.5,
            "pitch_joint": "4", "yaw_joint": "5", "axes_angle_deg": 90.0, "reference_joints": [0.0, 0.0, Q_INS[version], 0.0, 0.1, -0.2],
            "L_source": "sawIntuitiveResearchKit 2.4.0 (bc33e79) share/tool/LARGE_NEEDLE_DRIVER_400006.json l.29, wrist_yaw A = 0.0091 m"}


def write_configs(out):
    import yaml
    env = os.path.join(HERE, "environment")
    sr = yaml.safe_load(open(os.path.join(env, "expectations_src_client.yaml")))
    dv = yaml.safe_load(open(os.path.join(env, "expectations_dvrk_client.yaml")))
    os.makedirs(os.path.join(out, "expectations"), exist_ok=True)
    for v in ("v1", "v2"):
        for name, base in (("src_client", sr), ("dvrk_client", dv)):
            with open(os.path.join(out, "expectations", f"{name}_geometry_{v}.yaml"), "w") as f:
                yaml.safe_dump(dict(base, dimensional=geo(v)), f, sort_keys=False)


class Stack:
    def __init__(self, version, logdir):
        self.v, self.logdir, self.procs = version, logdir, []

    def _p(self, cmd, log, cwd=None):
        # as run_live.sh (RC3/RC8): the AMBF catkin environment and the ambf_client Python module (addendum D: the first
        # v1 attempt lacked both, launch_crtk_interface.py failed with ModuleNotFoundError: ambf_client)
        amb = os.path.join(LIVE, "ambf-2.0")
        cmd = (f"source {amb}/build/devel/setup.bash; export PYTHONPATH={amb}/ambf_ros_modules/ambf_client/python:$PYTHONPATH; " + cmd)
        self.procs.append(subprocess.Popen(["bash", "-c", cmd], cwd=cwd, stdout=open(os.path.join(self.logdir, log), "w"), stderr=subprocess.STDOUT,
                                           preexec_fn=os.setsid, env=os.environ.copy()))

    def __enter__(self):
        os.makedirs(self.logdir, exist_ok=True)
        src = os.path.join(LIVE, f"src-{self.v}")
        self._p("roscore", "roscore.log")
        time.sleep(4)
        args = AMBF_ARGS[self.v].format(src=src)
        self._p(f"xvfb-run -a -s '-screen 0 1280x720x24' ./ambf_simulator {args}", "ambf_simulator.log", cwd=os.path.join(LIVE, "ambf-2.0", "bin", "lin-x86_64"))
        time.sleep(25)
        self._p("python3 -u launch_crtk_interface.py --two False --ecm False --scene False", "launch_crtk_interface.log",
                cwd=os.path.join(src, "scripts", "surgical_robotics_challenge"))
        time.sleep(15)
        subprocess.run(f"rostopic list > {self.logdir}/topics.txt 2>&1", shell=True)
        self.q_work = move_to_work(self.v, self.logdir)
        return self

    def __exit__(self, *a):
        for p in reversed(self.procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
        time.sleep(3)
        for p in reversed(self.procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        subprocess.run("pkill -9 -f ambf_simulator; pkill -9 -f Xvfb; pkill -9 -f rosmaster", shell=True)
        time.sleep(2)


def move_to_work(version, logdir):
    """RC8 precondition (live_aux.py): the initially near-singular arm is moved by servo_jp to q_work = [0, 0, q3, 0, 0, 0]
    (about 42 % of the insertion range) before any probe runs; streamed at 100 Hz for 4 s, then the measured joints
    are recorded."""
    import rospy
    from sensor_msgs.msg import JointState
    if not rospy.core.is_initialized():
        rospy.init_node("src_live_harness", anonymous=True, disable_signals=True)
    last = {}
    sub = rospy.Subscriber(NS + "/measured_js", JointState, lambda m: last.__setitem__("js", m))
    pub = rospy.Publisher(NS + "/servo_jp", JointState, queue_size=10)
    time.sleep(1.5)
    q = [0.0, 0.0, Q_INS[version], 0.0, 0.0, 0.0]
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:
        m = JointState(); m.header.stamp = rospy.Time.now(); m.position = q  # positions only, as live_aux.py
        pub.publish(m); time.sleep(0.01)
    time.sleep(1.0)
    out = {"q_work_commanded": q, "measured_js": None if "js" not in last else {"name": list(last["js"].name), "position": list(last["js"].position)}}
    json.dump(out, open(os.path.join(logdir, "q_work.json"), "w"), indent=1)
    sub.unregister(); pub.unregister()
    return out


def cli(name, outdir, probes, exp, tol_mm):
    cmd = [sys.executable, "-m", "crtk_conformance.cli", "run", "--namespace", NS, "--tolerance-mm", str(tol_mm), "--probes", probes,
           *RC8_PARAMS, "--out", os.path.join(outdir, f"{name}.json"), "--quiet"]
    if exp:
        cmd += ["--expectations", exp]
    t0 = time.time()
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=2400)
    open(os.path.join(outdir, f"{name}.log"), "w").write(" ".join(cmd) + "\n" + r.stdout)
    try:
        s = json.load(open(os.path.join(outdir, f"{name}.json")))["summary"]
    except Exception:
        s = None
    row = {"case": name, "returncode": r.returncode, "wall_s": time.time() - t0, "summary": s}
    print(json.dumps(row), flush=True)
    return row


def run(version, out):
    src = os.path.join(LIVE, f"src-{version}")
    subprocess.run(f"cd {src}/scripts && pip3 install -q --no-deps --no-build-isolation -e . > /dev/null 2>&1", shell=True)
    odir = os.path.join(out, f"live-src-{version}")
    os.makedirs(odir, exist_ok=True)
    rows = []
    exps = os.path.join(out, "expectations")
    with Stack(version, os.path.join(odir, "logs")):
        rows.append(cli("src_client_geometry_1mm", odir, "frame,geometry,rate", os.path.join(exps, f"src_client_geometry_{version}.yaml"), 1.0))
        rows.append(cli("src_client_geometry_5mm", odir, "geometry", os.path.join(exps, f"src_client_geometry_{version}.yaml"), 5.0))
        rows.append(cli("dvrk_client_geometry_1mm", odir, "frame,geometry,rate", os.path.join(exps, f"dvrk_client_geometry_{version}.yaml"), 1.0))
    json.dump(rows, open(os.path.join(odir, "rows.json"), "w"), indent=1)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["configs", "run"])
    ap.add_argument("version", nargs="?", choices=["v1", "v2"])
    ap.add_argument("--out", default=os.path.join(REPO, "validation", "v0.1.6", "src_live"))
    a = ap.parse_args()
    if a.cmd == "configs":
        write_configs(a.out)
    else:
        run(a.version, a.out)
