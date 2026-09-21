"""RC9 independent-implementation campaign: the released dVRK console (sawIntuitiveResearchKit 2.4.0) in kinematic
simulation, run unmodified in Docker (validation/v0.1.6/environment), probed through its public ROS 1 topics.

Usage (inside the campaign image, workspace mounted at /ws, this repository at /ws/crtk-conformance):
    python3 validation/dvrk_sim_v016.py configs      # write the platform configurations and client declarations
    python3 validation/dvrk_sim_v016.py smoke        # WP1 inventory / smoke test (no probe)
    python3 validation/dvrk_sim_v016.py campaign --launches 3 --out validation/v0.1.6/dvrk_sim

The platform configurations select released files only: the simulated PSM arm file of sawIntuitiveResearchKit and the
PSM2 base_frame copied verbatim (by script) from dvrk_config_jhu/jhu-dVRK/system-MTMR-PSM2-Teleop.json.  Every client
declaration is derived by script from that file (nothing typed by hand); PREREGISTRATION.md lists the expected outcome
of every case and was committed before the first probe run.  Each launch starts a fresh ROS master and dvrk_system
(`sleep infinity |` as stdin: the text-only console otherwise spins), brings the arm to ENABLED/homed through
state_command, checks that measured_cp carries a non-zero stamp and a non-identity pose, and then runs the tool's CLI
once per case.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "src"))

from crtk_conformance import geometry as G  # noqa: E402

DVRK_WS = os.environ.get("DVRK_WS", "/ws/dvrk_ws")
JHU_SYSTEM = os.path.join(DVRK_WS, "src/dvrk/dvrk_config_jhu/jhu-dVRK/system-MTMR-PSM2-Teleop.json")
ARM_FILE = "arm/PSM_KIN_SIMULATED_LARGE_NEEDLE_DRIVER_400006.json"
NS = "/PSM2"
REF_JOINTS = [0.1, 0.1, 0.12, 0.0, 0.1, -0.2]  # yaw, pitch, insertion (m), roll, wrist_pitch, wrist_yaw: away from the home singularity
TASK = ["--tolerance-mm", "1.0", "--workspace-radius-m", "0.10", "--speed-mm-s", "50", "--client-rate-hz", "100", "--jitter-max-ms", "5"]
L_LND = 0.0091  # m, sawIntuitiveResearchKit 2.4.0 share/tool/LARGE_NEEDLE_DRIVER_400006.json, wrist_yaw DH "A" ("see dVRK user guide")
U_REL = 0.015   # declared relative uncertainty of L (released models disagree: SRC uses 9.0 mm); fixed before any run
E_AXIS = np.array([0.48, 0.60, 0.64])  # a generic unit axis for the perturbed declarations


def _strip_json_comments(txt: str) -> str:
    txt = re.sub(r"/\*.*?\*/", "", txt, flags=re.S)
    return "\n".join(l for l in txt.splitlines() if not l.strip().startswith("//"))


def jhu_base_frame() -> dict:
    cfg = json.loads(_strip_json_comments(open(JHU_SYSTEM).read()))
    return [a for a in cfg["arms"] if a["name"] == "PSM2"][0]["base_frame"]


def orthonormal(T: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(T[:3, :3])
    Tn = T.copy()
    Tn[:3, :3] = U @ Vt
    return Tn


def pose_to_decl(T: np.ndarray) -> dict:
    x, y, z, w = G.rot_to_quat(T[:3, :3])
    return {"mode": "expected_transform", "translation_m": [float(v) for v in T[:3, 3]], "quaternion_xyzw": [float(x), float(y), float(z), float(w)]}


def residual_target(t_par_mm: float, theta_deg: float) -> np.ndarray:
    k = E_AXIS / np.linalg.norm(E_AXIS)
    return G.make_pose(G.axis_angle(k, math.radians(theta_deg)), (t_par_mm * 1e-3) * k)


def write_configs(outdir: str) -> dict:
    """Platform configurations (released values only) and client declarations (derived by script)."""
    os.makedirs(os.path.join(outdir, "configs"), exist_ok=True)
    os.makedirs(os.path.join(outdir, "expectations"), exist_ok=True)
    bf = jhu_base_frame()
    arm = {"name": "PSM2", "type": "PSM", "simulation": "KINEMATIC", "arm_file": ARM_FILE}
    for name, extra in (("jhu", {"base_frame": bf}), ("nobase", {})):
        with open(os.path.join(outdir, "configs", f"system-PSM2_KIN_SIM-{name}.json"), "w") as f:
            json.dump({"$id": "dvrk-system.schema.json", "$version": "1", "arms": [dict(arm, **extra)]}, f, indent=4)
    T_true = orthonormal(np.array(bf["transform"], dtype=float))
    geo_si = {"mode": "geometry_anchor", "expected_unit_m": 1.0, "L_m": L_LND, "u_rel": U_REL, "delta_q_rad": 0.5,
              "pitch_joint": "wrist_pitch", "yaw_joint": "wrist_yaw", "axes_angle_deg": 90.0, "reference_joints": REF_JOINTS,
              "L_source": "sawIntuitiveResearchKit 2.4.0 (bc33e79) share/tool/LARGE_NEEDLE_DRIVER_400006.json l.29, wrist_yaw A = 0.0091 m"}
    geo_mm = dict(geo_si, expected_unit_m=0.001)
    envdir = os.path.join(REPO, "validation", "environment")
    import yaml
    dv = yaml.safe_load(open(os.path.join(envdir, "expectations_dvrk_client.yaml")))
    sr = yaml.safe_load(open(os.path.join(envdir, "expectations_src_client.yaml")))
    decls = {
        # client runs (Table V-style rows): the RC8 client files unchanged except that the dimensional class uses the anchor
        "C_dvrk": dict(dv, dimensional=geo_si),
        "C_src": dict(sr, dimensional=geo_si),
        "C_disc": {},
        # frame cases (spatial only)
        "F2_identity": {"spatial": {"mode": "identity"}},
        "F3_inverse": {"spatial": pose_to_decl(G.invert(T_true))},
        "F4_0874": {"spatial": pose_to_decl(G.invert(residual_target(0.7, 0.3)) @ T_true)},
        "F5a_095": {"spatial": pose_to_decl(G.invert(residual_target(0.7927, 0.3)) @ T_true)},
        "F5b_098": {"spatial": pose_to_decl(G.invert(residual_target(0.8284, 0.3)) @ T_true)},
        "F5c_102": {"spatial": pose_to_decl(G.invert(residual_target(0.8754, 0.3)) @ T_true)},
        "F5d_105": {"spatial": pose_to_decl(G.invert(residual_target(0.9101, 0.3)) @ T_true)},
        "F_exactJHU": {"spatial": pose_to_decl(T_true)},
        # unit cases (geometry anchor)
        "U_si": {"dimensional": geo_si},
        "U_mm": {"dimensional": geo_mm},
        # stop expectation that the held behaviour violates
        "T_fault025": {"temporal": {"stop_behaviour": "fault", "horizon_s": 0.25}},
    }
    for k, d in decls.items():
        with open(os.path.join(outdir, "expectations", f"{k}.yaml"), "w") as f:
            yaml.safe_dump(d, f, sort_keys=False)
    # derived truths (independent of the tool's decision code: eq. (3') evaluated here from the construction)
    from crtk_conformance.spatial import exact_max_error
    truths = {}
    for k in ("F2_identity", "F3_inverse", "F4_0874", "F5a_095", "F5b_098", "F5c_102", "F5d_105", "F_exactJHU"):
        from crtk_conformance.expectations import Expectations
        Te = Expectations.from_dict(decls[k]).spatial.matrix()
        E = T_true @ G.invert(Te)
        truths[k] = {"jhu": exact_max_error(E[:3, :3], E[:3, 3], 0.10)}
        E0 = G.invert(Te)  # nobase: T_true = I
        truths[k]["nobase"] = exact_max_error(E0[:3, :3], E0[:3, 3], 0.10)
    json.dump({"T_true_jhu": T_true.tolist(), "E_max_m": truths, "reference_joints": REF_JOINTS, "L_m": L_LND, "u_rel": U_REL},
              open(os.path.join(outdir, "derived_truths.json"), "w"), indent=1)
    return truths


# ------------------------------------------------------------------ processes
class Launch:
    def __init__(self, system_json: str, logdir: str, port: int = 11311):
        self.system_json, self.logdir, self.port = system_json, logdir, port
        self.procs = []

    def __enter__(self):
        os.makedirs(self.logdir, exist_ok=True)
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = f"http://127.0.0.1:{self.port}"
        os.environ["ROS_MASTER_URI"] = env["ROS_MASTER_URI"]
        self.procs.append(subprocess.Popen(["roscore", "-p", str(self.port)], stdout=open(os.path.join(self.logdir, "roscore.log"), "w"),
                                           stderr=subprocess.STDOUT, preexec_fn=os.setsid, env=env))
        time.sleep(4)
        cmd = f"sleep infinity | rosrun dvrk_robot dvrk_system -j {self.system_json} -t"
        self.procs.append(subprocess.Popen(["bash", "-c", cmd], cwd=self.logdir, stdout=open(os.path.join(self.logdir, "dvrk_system.log"), "w"),
                                           stderr=subprocess.STDOUT, preexec_fn=os.setsid, env=env))
        time.sleep(8)
        return self

    def __exit__(self, *a):
        for p in reversed(self.procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
        time.sleep(2)
        for p in reversed(self.procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        time.sleep(1)


def bring_up(timeout_s: float = 30.0) -> dict:
    """enable, then home, through state_command (one command at a time: the console keeps only the latest state
    command, cisst-ros mtsROSBridge.h l.295-296); wait for ENABLED, then for is_homed and not is_busy (homing moves
    the insertion to 0.12 m over ~1.5 s and reports is_homed before the motion ends); interpolate servo_jp from the
    measured joints to the reference configuration over 3 s (a step would exceed the simulated PID's tracking-error
    tolerance); check that measured_cp carries a non-zero stamp and a non-identity pose.  Returns what was observed.

    Harness change of 21 Sep 2026, before any probe run on this target: the first campaign attempt (archived in
    runs-failed-bringup/) sent enable and home 0.2 s apart and streamed the reference joints as soon as is_homed was
    reported, while homing was still moving the arm; the PID tracking-error check faulted the arm and no case ran."""
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from crtk_msgs.msg import OperatingState, StringStamped
    if not rospy.core.is_initialized():
        rospy.init_node("dvrk_sim_harness", anonymous=True, disable_signals=True)
    last = {}

    def keep(k):
        return lambda m: last.__setitem__(k, (time.monotonic(), m))
    subs = [rospy.Subscriber(NS + "/operating_state", OperatingState, keep("os")),
            rospy.Subscriber(NS + "/measured_cp", PoseStamped, keep("cp")),
            rospy.Subscriber(NS + "/measured_js", JointState, keep("js"))]
    pub = rospy.Publisher(NS + "/state_command", StringStamped, queue_size=10)
    pj = rospy.Publisher(NS + "/servo_jp", JointState, queue_size=10)
    time.sleep(1.5)
    out = {"initial": None}

    def st():
        d = last.get("os")
        return (None, None) if d is None else d

    if st()[1] is not None:
        out["initial"] = {"state": st()[1].state, "is_homed": st()[1].is_homed, "is_busy": st()[1].is_busy}
    cp0 = last.get("cp")
    out["stamp_before_enable_ns"] = None if cp0 is None else cp0[1].header.stamp.to_nsec()
    t0 = time.monotonic()

    def send(c):
        m = StringStamped(); m.header.stamp = rospy.Time.now(); m.string = c
        pub.publish(m)
        return time.monotonic()

    def await_(pred, t_sent, limit):
        while time.monotonic() - t0 < limit:
            t_rx, o = st()
            if o is not None and (t_rx >= t_sent or time.monotonic() - t_sent > 0.3) and pred(o):
                return True
            time.sleep(0.02)
        return False

    if st()[1] is None or st()[1].state != "ENABLED":
        out["enabled"] = await_(lambda o: o.state == "ENABLED", send("enable"), timeout_s)
    out["enable_latency_s"] = time.monotonic() - t0
    out["homed"] = await_(lambda o: o.state == "ENABLED" and o.is_homed and not o.is_busy, send("home"), timeout_s)
    out["enable_home_latency_s"] = time.monotonic() - t0
    o = st()[1]
    out["after"] = None if o is None else {"state": o.state, "is_homed": o.is_homed, "is_busy": o.is_busy}
    js = last.get("js")
    if js is not None and out["homed"]:
        names = list(js[1].name)
        q0 = np.array(js[1].position[:len(REF_JOINTS)], dtype=float)
        q1 = np.array(REF_JOINTS, dtype=float)
        names = names[:len(REF_JOINTS)]
        t1 = time.monotonic()
        while True:
            a = min(1.0, (time.monotonic() - t1) / 3.0)
            m = JointState(); m.header.stamp = rospy.Time.now(); m.name = names; m.position = list(q0 + a * (q1 - q0))
            pj.publish(m)
            if a >= 1.0:
                break
            time.sleep(0.01)
        time.sleep(1.0)
        out["joints"] = dict(zip(names, [round(v, 6) for v in last["js"][1].position]))
    o = st()[1]
    out["state_at_reference"] = None if o is None else {"state": o.state, "is_homed": o.is_homed, "is_busy": o.is_busy}
    cp = last.get("cp")
    out["stamp_after_ns"] = None if cp is None else cp[1].header.stamp.to_nsec()
    if cp is not None:
        p = cp[1].pose.position
        q = cp[1].pose.orientation
        out["pose_after"] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        out["ready"] = bool(out["homed"] and o is not None and o.state == "ENABLED" and cp[1].header.stamp.to_nsec() != 0
                            and not (abs(p.x) + abs(p.y) + abs(p.z) < 1e-12 and abs(q.w) > 1 - 1e-12)
                            and time.monotonic() - cp[0] < 0.5
                            and "joints" in out and all(abs(a_ - b_) < 1e-3 for a_, b_ in zip(list(out["joints"].values()), REF_JOINTS)))
    else:
        out["ready"] = False
    for s_ in subs:
        s_.unregister()
    pub.unregister(); pj.unregister()
    return out


def run_cli(name: str, outdir: str, probes: str, expectations: str, extra=None, tol_mm: float = 1.0) -> dict:
    task = list(TASK)
    task[1] = str(tol_mm)
    cmd = [sys.executable, "-m", "crtk_conformance.cli", "run", "--namespace", NS, *task, "--probes", probes,
           "--enable-timeout-s", "15", "--out", os.path.join(outdir, f"{name}.json"), "--quiet"]
    if expectations:
        cmd += ["--expectations", expectations]
    cmd += list(extra or [])
    env = os.environ.copy()
    env["PYTHONPATH"] = os.path.join(REPO, "src") + ":" + env.get("PYTHONPATH", "")
    t0 = time.time()
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=1800)
    with open(os.path.join(outdir, f"{name}.log"), "w") as f:
        f.write(" ".join(cmd) + "\n" + r.stdout)
    summ = None
    try:
        summ = json.load(open(os.path.join(outdir, f"{name}.json")))["summary"]
    except Exception:
        pass
    return {"case": name, "returncode": r.returncode, "wall_s": time.time() - t0, "summary": summ}


CASES_JHU = [  # (name, probes, expectations file, extra args, tolerance mm)
    ("C_dvrk", "frame,geometry,rate", "C_dvrk", [], 1.0),
    ("C_src", "frame,geometry,rate", "C_src", [], 1.0),
    ("C_disc", "frame,scale,rate", None, [], 1.0),
    ("F2_identity", "frame", "F2_identity", [], 1.0),
    ("F3_inverse", "frame", "F3_inverse", [], 1.0),
    ("F4_0874", "frame", "F4_0874", [], 1.0),
    ("F5a_095", "frame", "F5a_095", [], 1.0),
    ("F5b_098", "frame", "F5b_098", [], 1.0),
    ("F5c_102", "frame", "F5c_102", [], 1.0),
    ("F5d_105", "frame", "F5d_105", [], 1.0),
    ("U_si_5mm", "geometry", "U_si", [], 5.0),
    ("U_mm_1mm", "geometry", "U_mm", [], 1.0),
    ("U_mm_5mm", "geometry", "U_mm", [], 5.0),
    ("T_fault025", "rate", "T_fault025", ["--skip-rate-sweep"], 1.0),
]
CASES_NOBASE = [
    ("C_src", "frame,geometry,rate", "C_src", [], 1.0),
    ("C_dvrk", "frame,geometry,rate", "C_dvrk", [], 1.0),
    ("F_exactJHU", "frame", "F_exactJHU", [], 1.0),
]


def campaign(out: str, launches: int, only: str = None):
    os.makedirs(out, exist_ok=True)
    rows = []
    for cfg, cases in (("jhu", CASES_JHU), ("nobase", CASES_NOBASE)):
        if only and cfg != only:
            continue
        system_json = os.path.join(out, "configs", f"system-PSM2_KIN_SIM-{cfg}.json")
        for k in range(launches):
            ldir = os.path.join(out, "runs", f"{cfg}_launch{k+1}")
            os.makedirs(ldir, exist_ok=True)
            with Launch(system_json, ldir):
                # bring-up before every case: each case starts ENABLED, homed, at the reference joints (the state
                # sub-probe of the temporal runs disables the arm, and the console resets the simulated joints then)
                for name, probes, exp, extra, tol_mm in cases:
                    up = bring_up()
                    json.dump(up, open(os.path.join(ldir, f"{name}.bring_up.json"), "w"), indent=1)
                    if not up.get("ready"):
                        r = {"config": cfg, "launch": k + 1, "case": name, "error": "bring-up not ready", "bring_up": up}
                        rows.append(r)
                        print(json.dumps(r), flush=True)
                        continue
                    e = os.path.join(out, "expectations", f"{exp}.yaml") if exp else None
                    r = run_cli(name, ldir, probes, e, extra, tol_mm)
                    r.update(config=cfg, launch=k + 1, probes=probes, tolerance_mm=tol_mm, bring_up_latency_s=up.get("enable_home_latency_s"))
                    rows.append(r)
                    print(json.dumps(r), flush=True)
            json.dump(rows, open(os.path.join(out, "campaign_rows.json"), "w"), indent=1)
    return rows


def smoke(out: str):
    os.makedirs(out, exist_ok=True)
    res = {}
    for cfg in ("jhu", "nobase"):
        ldir = os.path.join(out, "smoke", cfg)
        with Launch(os.path.join(out, "configs", f"system-PSM2_KIN_SIM-{cfg}.json"), ldir):
            r = subprocess.run(["python3", os.path.join(HERE, "dvrk_sim_smoke.py"), NS, os.path.join(out, "configs", f"system-PSM2_KIN_SIM-{cfg}.json"),
                                os.path.join(ldir, "smoke.json")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=300)
            open(os.path.join(ldir, "smoke.log"), "w").write(r.stdout)
            res[cfg] = json.load(open(os.path.join(ldir, "smoke.json")))
    json.dump(res, open(os.path.join(out, "smoke", "smoke_summary.json"), "w"), indent=1)
    return res


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["configs", "smoke", "campaign"])
    ap.add_argument("--out", default=os.path.join(REPO, "validation", "v0.1.6", "dvrk_sim"))
    ap.add_argument("--launches", type=int, default=3)
    ap.add_argument("--only", default=None)
    a = ap.parse_args()
    if a.cmd == "configs":
        print(json.dumps(write_configs(a.out), indent=1))
    elif a.cmd == "smoke":
        print(json.dumps({k: {kk: v[kk] for kk in ("operating_state_after_home", "screw_axis_anchor") if kk in v} for k, v in smoke(a.out).items()}, indent=1))
    else:
        campaign(a.out, a.launches, a.only)
