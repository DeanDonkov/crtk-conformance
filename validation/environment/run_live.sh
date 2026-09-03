#!/bin/bash
# Live run of crtk-conformance 0.1.1 against a released Surgical Robotics Challenge CRTK interface on AMBF.
# Usage (inside focal-crtk:rc3 with /home/claude/rc3 mounted at /rc3):  run_live.sh v1|v2 <outdir>
set +u
VER=$1; OUT=$2
mkdir -p "$OUT/logs"
source /rc3/live/env.sh
source /rc3/sources/ambf-2.0/build/devel/setup.bash
export PYTHONPATH=/rc3/sources/ambf-2.0/ambf_ros_modules/ambf_client/python:$PYTHONPATH
if [ "$VER" = "v1" ]; then
  SRC=/rc3/sources/src-v1; AMBF_ARGS="--launch_file $SRC/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1 --override_max_comm_freq 120 -g false"; Q3=1.0; AMBF_ARM=/ambf/env/psm1/baselink
else
  SRC=/rc3/sources/src-v2; AMBF_ARGS="--launch_file $SRC/launch.yaml -l 0,1,6,7,8,9 -p 200 -t 1 --override_max_comm_freq 100 --override_min_comm_freq 100 -g false"; Q3=0.1; AMBF_ARM=/ambf/env/psm1/baselinksimple
fi
log() { echo "[$(date -u +%Y-%m-%dT%H:%M:%SZ)] $*" | tee -a "$OUT/logs/run.log"; }
git config --global --add safe.directory "*"
log "target SRC $VER at $(git -C $SRC rev-parse HEAD); AMBF ambf-2.0 at $(git -C /rc3/sources/ambf-2.0 rev-parse HEAD); crtk-conformance at $(git -C /rc3/preprint/crtk-conformance rev-parse HEAD)"
(cd $SRC/scripts && pip3 install -q --no-deps --no-build-isolation -e . 2>&1 | tail -1)
log "starting roscore"; roscore > "$OUT/logs/roscore.log" 2>&1 & sleep 4
log "starting ambf_simulator: $AMBF_ARGS"
(cd /rc3/sources/ambf-2.0/bin/lin-x86_64 && xvfb-run -a -s "-screen 0 1280x720x24" ./ambf_simulator $AMBF_ARGS > "$OUT/logs/ambf_simulator.log" 2>&1) &
sleep 25
log "starting SRC CRTK interface (PSM1 only: --two False --ecm False --scene False)"
(cd $SRC/scripts/surgical_robotics_challenge && python3 -u launch_crtk_interface.py --two False --ecm False --scene False > "$OUT/logs/launch_crtk_interface.log" 2>&1) &
sleep 15
rostopic list > "$OUT/topics.txt" 2>&1
for t in measured_cp servo_cp measured_js T_b_w; do rostopic info /CRTK/psm1/$t > "$OUT/info_$t.txt" 2>&1; done
log "auxiliary observations"
python3 /rc3/live/live_aux.py --version "$VER" --q3-work $Q3 --ambf-arm $AMBF_ARM --out "$OUT/live_aux.json" > "$OUT/logs/live_aux.log" 2>&1
tail -5 "$OUT/logs/live_aux.log"
log "crtk-conformance run A: discover-only (no expectations)"
python3 -m crtk_conformance.cli run --namespace /CRTK/psm1 --tolerance-mm 1.0 --workspace-radius-m 0.10 --speed-mm-s 50 --client-rate-hz 100 --jitter-max-ms 5 \
  --trials 10 --temporal-trials 5 --gap-max-s 2.0 --settle-s 2.0 --still-tol-mm 0.05 --rates-hz 50,100,200,500 --rate-max-step-mm 100 \
  --out "$OUT/report_discover_only.json" > "$OUT/logs/run_discover_only.log" 2>&1
tail -12 "$OUT/logs/run_discover_only.log"
log "crtk-conformance run B: SRC-authored client expectation (expectations_src_client.yaml)"
python3 -m crtk_conformance.cli run --namespace /CRTK/psm1 --tolerance-mm 1.0 --workspace-radius-m 0.10 --speed-mm-s 50 --client-rate-hz 100 --jitter-max-ms 5 \
  --trials 10 --temporal-trials 5 --gap-max-s 2.0 --settle-s 2.0 --still-tol-mm 0.05 --rates-hz 50,100,200,500 --rate-max-step-mm 100 \
  --expectations /rc3/live/expectations_src_client.yaml --out "$OUT/report_src_client.json" > "$OUT/logs/run_src_client.log" 2>&1
tail -12 "$OUT/logs/run_src_client.log"
log "crtk-conformance run C: dVRK-authored client expectation (expectations_dvrk_client.yaml)"
python3 -m crtk_conformance.cli run --namespace /CRTK/psm1 --tolerance-mm 1.0 --workspace-radius-m 0.10 --speed-mm-s 50 --client-rate-hz 100 --jitter-max-ms 5 \
  --trials 10 --temporal-trials 5 --gap-max-s 2.0 --settle-s 2.0 --still-tol-mm 0.05 --rates-hz 50,100,200,500 --rate-max-step-mm 100 \
  --expectations /rc3/live/expectations_dvrk_client.yaml --out "$OUT/report_dvrk_client.json" > "$OUT/logs/run_dvrk_client.log" 2>&1
tail -12 "$OUT/logs/run_dvrk_client.log"
log "done"
python3 - "$OUT" <<'EOF'
import json, sys, platform, subprocess, os, time
out = sys.argv[1]
def sh(c):
    try: return subprocess.check_output(c, shell=True, text=True).strip()
    except Exception as e: return f"error: {e}"
meta = {
 "finished_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
 "python": sys.version, "platform": platform.platform(), "os_release": sh("cat /etc/os-release | head -2"),
 "ros_distro": os.environ.get("ROS_DISTRO"), "ros_source_manifest": "/rc3/live/ros_src/manifest.txt",
 "pip_freeze": sh("pip3 freeze 2>/dev/null"), "ambf_commit": sh("git -C /rc3/sources/ambf-2.0 rev-parse HEAD"),
 "crtk_conformance_commit": sh("git -C /rc3/preprint/crtk-conformance rev-parse HEAD"),
 "crtk_msgs_commit": sh("git -C /rc3/sources/crtk_msgs rev-parse HEAD"),
 "env": {k: os.environ.get(k) for k in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME", "PYTHONPATH")},
}
json.dump(meta, open(os.path.join(out, "meta_env.json"), "w"), indent=1)
EOF
pkill -f launch_crtk_interface.py; pkill -f ambf_simulator; pkill -f roscore; pkill -f rosmaster; sleep 2
