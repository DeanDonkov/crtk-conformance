#!/usr/bin/env python3
"""RC14 campaigns of crtk-conformance 0.1.8 (validation/v0.1.8/PREREGISTRATION.md, committed before any run of F, D or S).

  F  reference node: the frame probe's correlation guard under AR(1) position noise, against the 0.1.7 procedure
  D  released dVRK console 2.4.0 in kinematic simulation (controls): frame cases with the guard, joint-space anchor cases
  S  released SRC v1.0.0 and v2.0.0 on AMBF: the joint-space anchor (detection of the v1.0.0 unit), resting pose traces

Usage (inside the campaign image; this repository at /repo, build dir at /ws):
    python3 validation/run_validation_v018.py F [--n 20] [--port 11418] [--rerun name,...]
    python3 validation/run_validation_v018.py D [--launches 3]
    python3 validation/run_validation_v018.py S v1|v2 [--launches 3]
    python3 validation/run_validation_v018.py smoke-dvrk        # EXPLORATORY feasibility check of the harness (not a result)
"""
import argparse
import json
import os
import platform
import subprocess
import sys
import time
from fractions import Fraction

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "src"))
from crtk_conformance import __version__  # noqa: E402

OUT = os.path.join(HERE, "v0.1.8")
SEED0 = 81000
RERUN = set()


def meta(path, **kw):
    rev = subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", REPO, "status", "--porcelain", "--untracked-files=no", "src", "validation/run_validation_v018.py"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True).stdout.strip()
    json.dump(dict(crtk_conformance_version=__version__, repo_commit=rev, tracked_changes_in_src_or_harness=dirty, python=sys.version,
                   platform=platform.platform(), started_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **kw), open(path, "w"), indent=1)


# ------------------------------------------------------------------ F: reference node
F_PHIS = (0.0, 0.99)
F_RATIOS = ("0.95", "1.00")
F_NOISE = 2e-5  # 0.02 mm per channel


def exp_F(n_rep, port):
    from harness import ros_master, mock_node
    from run_validation_v013 import save, log
    from run_validation_v016 import run_probe, TOL
    from crtk_conformance.expectations import Expectations
    from crtk_conformance.probes.frame import FrameSemanticsProbe
    out = os.path.join(OUT, "mock", "F")
    os.makedirs(out, exist_ok=True)
    meta(os.path.join(out, f"meta{'_rerun' if RERUN else ''}.json"), n_rep=n_rep, seed0=SEED0, rerun=sorted(RERUN))
    E_ID = Expectations.from_dict({"spatial": {"mode": "identity"}})
    with ros_master(port):
        from crtk_conformance.adapter import PlatformAdapter
        for _ in range(3):  # discarded warm-up start-up (as in the v0.1.6 and v0.1.7 campaigns)
            with mock_node("reference", {"seed": 1}):
                a = PlatformAdapter("/PSM1"); d = a.discover(); a.close()
                if d["topics"]["measured_cp"]["present"]:
                    break
        k = 0
        for phi in F_PHIS:
            for rs in F_RATIOS:
                t = float(Fraction(rs) * Fraction(1, 1000))
                for guard in (False, True):
                    for j in range(n_rep):
                        name = f"F18_{'guard' if guard else 'b2b'}_phi{phi:g}_{rs}_{j:02d}"
                        over = {"bind_translation_m": [t, 0.0, 0.0], "bind_angle_deg": 0.0, "bind_axis": [0, 0, 1], "noise_m": F_NOISE,
                                "noise_model": "ar1" if phi > 0 else "gaussian", "ar_phi": phi, "seed": SEED0 + k}
                        k += 1
                        if RERUN and name not in RERUN:
                            continue
                        rec = run_probe(lambda a, g=guard: FrameSemanticsProbe(a, TOL, trials=10, samples_per_trial=5, expectations=E_ID, correlation_guard=g),
                                        "reference", over, expectation=E_ID)
                        rec["truth"] = {"ratio": rs, "E_true_m": t, "truth_conformant": Fraction(rs) <= 1, "phi": phi, "noise_m": F_NOISE,
                                        "procedure": "0.1.8" if guard else "0.1.7"}
                        save(out, name, rec)
                        r = rec["result"]; sd = r["estimates"].get("spatial_decision", {}); cpn = r["observations"].get("correlation_plan", {})
                        log(f"{name}: {r['outcome']} [{sd.get('ci_low_m', float('nan'))*1e3:.4f}..{sd.get('ci_high_m', float('nan'))*1e3:.4f}] mm "
                            f"tau {cpn.get('tau_int_samples')} spacing {cpn.get('spacing_samples')} ok {cpn.get('ok')} ({rec['wall_s']:.0f}s)")


# ------------------------------------------------------------------ D: dVRK console (kinematic simulation)
D_CASES = [  # (name, probes, expectations file in validation/v0.1.6/dvrk_sim/expectations, tolerance mm)
    ("F2_identity", "frame", "F2_identity", 1.0),
    ("F4_0874", "frame", "F4_0874", 1.0),
    ("F5a_095", "frame", "F5a_095", 1.0),
    ("F5b_098", "frame", "F5b_098", 1.0),
    ("F5c_102", "frame", "F5c_102", 1.0),
    ("F5d_105", "frame", "F5d_105", 1.0),
    ("U_si_1mm", "geometry", "U_si", 1.0),
    ("U_si_5mm", "geometry", "U_si", 5.0),
    ("U_mm_1mm", "geometry", "U_mm", 1.0),
]


def run_cli(ns, name, outdir, probes, expectations, tol_mm, extra=()):
    cmd = [sys.executable, "-m", "crtk_conformance.cli", "run", "--namespace", ns, "--tolerance-mm", str(tol_mm), "--workspace-radius-m", "0.10",
           "--speed-mm-s", "50", "--client-rate-hz", "100", "--jitter-max-ms", "5", "--probes", probes, "--out", os.path.join(outdir, f"{name}.json"), "--quiet",
           *extra]  # 0.1.8 defaults: liveness rule 0.1.7, consistency gate, correlation guard, joint-space anchor
    if expectations:
        cmd += ["--expectations", expectations]
    env = os.environ.copy()
    env["PYTHONPATH"] = os.path.join(REPO, "src") + ":" + env.get("PYTHONPATH", "")
    t0 = time.time()
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=2400)
    open(os.path.join(outdir, f"{name}.log"), "w").write(" ".join(cmd) + "\n" + r.stdout)
    try:
        summ = json.load(open(os.path.join(outdir, f"{name}.json")))["summary"]
    except Exception:
        summ = None
    return {"case": name, "returncode": r.returncode, "wall_s": time.time() - t0, "summary": summ}


def exp_D(launches, cases=D_CASES, sub="D"):
    import dvrk_sim_v016 as DV
    base = os.path.join(HERE, "v0.1.6", "dvrk_sim")
    out = os.path.join(OUT, "dvrk_sim", sub)
    os.makedirs(out, exist_ok=True)
    meta(os.path.join(out, "campaign_meta.json"), launches=launches, cases=[c[0] for c in cases], configs_and_declarations=os.path.relpath(base, REPO))
    rows = []
    system_json = os.path.join(base, "configs", "system-PSM2_KIN_SIM-jhu.json")
    for k in range(launches):
        ldir = os.path.join(out, "runs", f"jhu_launch{k + 1}")
        os.makedirs(ldir, exist_ok=True)
        with DV.Launch(system_json, ldir):
            for name, probes, exp, tol in cases:
                up = DV.bring_up()
                json.dump(up, open(os.path.join(ldir, f"{name}.bring_up.json"), "w"), indent=1)
                if not up.get("ready"):
                    r = {"launch": k + 1, "case": name, "error": "bring-up not ready"}
                else:
                    r = run_cli(DV.NS, name, ldir, probes, os.path.join(base, "expectations", f"{exp}.yaml"), tol, ["--enable-timeout-s", "15"])
                    r.update(launch=k + 1, probes=probes, tolerance_mm=tol)
                rows.append(r)
                print(json.dumps(r), flush=True)
        json.dump(rows, open(os.path.join(out, "campaign_rows.json"), "w"), indent=1)
    return rows


# ------------------------------------------------------------------ S: SRC releases
def src_expectations(outdir):
    """The v0.1.6 SRC declarations with the joint-space anchor made explicit (anchor_method poe, joint_types RRPRRR)."""
    import yaml
    base = os.path.join(HERE, "v0.1.6", "src_live", "expectations")
    os.makedirs(outdir, exist_ok=True)
    for f in sorted(os.listdir(base)):
        d = yaml.safe_load(open(os.path.join(base, f)))
        d["dimensional"] = dict(d["dimensional"], anchor_method="poe", joint_types="RRPRRR")
        with open(os.path.join(outdir, f), "w") as g:
            yaml.safe_dump(d, g, sort_keys=False)


def record_trace(version, path, seconds=20.0):
    """Resting trace: measured_cp and measured_js at the geometry reference configuration, streamed by servo_jp."""
    import rospy
    import src_live_v016 as S
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    if not rospy.core.is_initialized():
        rospy.init_node("src_trace_v018", anonymous=True, disable_signals=True)
    h = {"cp": [], "js": []}
    s1 = rospy.Subscriber(S.NS + "/measured_cp", PoseStamped, lambda m: h["cp"].append([time.monotonic(), m.header.stamp.to_sec(), m.pose.position.x, m.pose.position.y, m.pose.position.z,
                                                                                         m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]))
    s2 = rospy.Subscriber(S.NS + "/measured_js", JointState, lambda m: h["js"].append([time.monotonic()] + list(m.position)))
    pub = rospy.Publisher(S.NS + "/servo_jp", JointState, queue_size=10)
    q = S.geo(version)["reference_joints"]
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0 + seconds:  # 4 s to settle, then the recorded window
        m = JointState(); m.header.stamp = rospy.Time.now(); m.position = list(q); pub.publish(m); time.sleep(0.01)
        if time.monotonic() - t0 < 4.0:
            h["cp"].clear(); h["js"].clear()
    s1.unregister(); s2.unregister(); pub.unregister()
    json.dump({"version": version, "q_ref": q, "seconds": seconds, "measured_cp": h["cp"], "measured_js": h["js"],
               "columns_cp": ["t_monotonic", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"]}, open(path, "w"))
    return len(h["cp"]), len(h["js"])


def exp_S(version, launches):
    import src_live_v016 as S
    out = os.path.join(OUT, "src_live")
    exps = os.path.join(out, "expectations")
    src_expectations(exps)
    src = os.path.join(S.LIVE, f"src-{version}")
    subprocess.run(f"cd {src}/scripts && pip3 install -q --no-deps --no-build-isolation -e . > /dev/null 2>&1", shell=True)
    rows = []
    for k in range(launches):
        odir = os.path.join(out, f"live-src-{version}", f"launch{k + 1}")
        os.makedirs(odir, exist_ok=True)
        meta(os.path.join(odir, "meta.json"), version=version, launch=k + 1)
        with S.Stack(version, os.path.join(odir, "logs")):
            n = record_trace(version, os.path.join(odir, "resting_trace.json"))
            print(json.dumps({"version": version, "launch": k + 1, "trace_samples": n}), flush=True)
            for name, decl, tol in (("src_client_geometry_1mm", "src_client", 1.0), ("src_client_geometry_5mm", "src_client", 5.0), ("dvrk_client_geometry_1mm", "dvrk_client", 1.0)):
                r = run_cli(S.NS, name, odir, "geometry", os.path.join(exps, f"{decl}_geometry_{version}.yaml"), tol,
                            ["--geometry-trials", "5", "--geometry-settle-s", "2.0"])
                r.update(version=version, launch=k + 1)
                rows.append(r)
                print(json.dumps(r), flush=True)
        json.dump(rows, open(os.path.join(out, f"live-src-{version}", "rows.json"), "w"), indent=1)
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("part", choices=["F", "D", "S", "smoke-dvrk"])
    ap.add_argument("version", nargs="?", choices=["v1", "v2"])
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--launches", type=int, default=3)
    ap.add_argument("--port", type=int, default=11418)
    ap.add_argument("--rerun", default=None)
    a = ap.parse_args()
    if a.rerun:
        RERUN.update(a.rerun.split(","))
    if a.part == "F":
        exp_F(a.n, a.port)
    elif a.part == "D":
        exp_D(a.launches)
    elif a.part == "S":
        exp_S(a.version, a.launches)
    else:
        exp_D(1, [c for c in D_CASES if c[0] in ("F4_0874", "U_si_5mm")], sub="exploratory-smoke")
