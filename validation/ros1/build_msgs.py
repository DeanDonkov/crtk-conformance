"""Generate Python message modules for a set of ROS 1 packages using genpy (no catkin)."""
import os, sys, subprocess, glob
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "genmsg/src")); sys.path.insert(0, os.path.join(ROOT, "genpy/src"))
PKGS = {
 "std_msgs": "std_msgs/msg",
 "rosgraph_msgs": "ros_comm_msgs/rosgraph_msgs/msg",
 "geometry_msgs": "common_msgs/geometry_msgs/msg",
 "sensor_msgs": "common_msgs/sensor_msgs/msg",
 "tf2_msgs": "geometry2/tf2_msgs/msg",
 "actionlib_msgs": "common_msgs/actionlib_msgs/msg",
 "crtk_msgs": "../primary/crtk_msgs/msg",
 "std_srvs": None,
}
OUT = os.path.join(ROOT, "site")
includes = []
for pkg, rel in PKGS.items():
    if rel: includes += ["-I", f"{pkg}:{os.path.join(ROOT, rel)}"]
for pkg, rel in PKGS.items():
    if not rel: continue
    outdir = os.path.join(OUT, pkg, "msg"); os.makedirs(outdir, exist_ok=True)
    msgs = sorted(glob.glob(os.path.join(ROOT, rel, "*.msg")))
    for m in msgs:
        subprocess.check_call([sys.executable, os.path.join(ROOT, "genpy/scripts/genmsg_py.py"), "-p", pkg, "-o", outdir] + includes + [m])
    subprocess.check_call([sys.executable, os.path.join(ROOT, "genpy/scripts/genmsg_py.py"), "--initpy", "-p", pkg, "-o", outdir])
    open(os.path.join(OUT, pkg, "__init__.py"), "w").close()
    print(pkg, len(msgs), "messages")
# services for rosgraph_msgs / std_srvs not needed

# roscpp msgs + srvs (needed by rospy.client)
pkg = "roscpp"; base = os.path.join(ROOT, "ros_comm/clients/roscpp")
inc = includes + ["-I", f"roscpp:{base}/msg"]
for kind, script in (("msg", "genmsg_py.py"), ("srv", "gensrv_py.py")):
    outdir = os.path.join(OUT, pkg, kind); os.makedirs(outdir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(base, kind, f"*.{kind}")))
    for m in files:
        subprocess.check_call([sys.executable, os.path.join(ROOT, "genpy/scripts", script), "-p", pkg, "-o", outdir] + inc + [m])
    subprocess.check_call([sys.executable, os.path.join(ROOT, "genpy/scripts", script), "--initpy", "-p", pkg, "-o", outdir])
open(os.path.join(OUT, pkg, "__init__.py"), "w").close()
print("roscpp done")
